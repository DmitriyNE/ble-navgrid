//! `navgrid control`: the controller's commands as desired-state records (FR-C5, architecture
//! §12). The host signs the record with the controller key (from the fleet key, DEF-27), sends it
//! to a sink over its serial port, and the sink injects it into its replica, from where sessions
//! carry it toward the target; the target acts on it and writes a Reported record, which the
//! monitor shows. The sink mirrors the injected record back at once, which this command waits for.
//!
//! The target `all` is every enrolled device, one record each: a rollout pins the fleet to the
//! build it runs (`all firmware CURRENT`), names the new build for the canaries, and once they
//! report it applied, for everyone.

use std::io::Write;
use std::path::Path;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use navgrid_mesh::{Body, CONTROLLER, Command, NodeId, Record, Writer, auth};
use navgrid_proto::{MAX_DOWNLINK_FRAME, Message, UpdateMsg};

use crate::fleet::{Fleet, FleetArgs, Registry, label_name};
use crate::monitor;
use crate::sign::parse_tag;

#[derive(clap::Args)]
pub struct ControlArgs {
    /// The sink's serial port (its VCOM1). Stop `navgrid monitor` on the port first.
    #[arg(long)]
    sink: String,
    #[arg(long, default_value_t = 115_200)]
    baud: u32,
    /// The node: its fleet number (`N042`), node ID (12 hex digits), or `all` for every
    /// enrolled device.
    target: String,
    #[command(subcommand)]
    command: ControlCmd,
    #[command(flatten)]
    fleet: FleetArgs,
}

#[derive(clap::Subcommand)]
pub enum ControlCmd {
    /// Blink the LED.
    Blink {
        #[arg(long, default_value_t = 3)]
        times: u8,
    },
    /// BME688 sampling: the period in seconds (0 = off) and whether to sample gas.
    Environment {
        #[arg(long)]
        period_s: u16,
        #[arg(long)]
        gas: bool,
    },
    /// Antenna diversity on or off.
    Diversity {
        #[arg(value_parser = clap::builder::BoolishValueParser::new(), action = clap::ArgAction::Set)]
        on: bool,
    },
    /// Advertising interval.
    Advertising {
        #[arg(long)]
        interval_ms: u16,
    },
    /// The build the node should run (`0.1.0+1950d45d`): it takes offers of that build only, and
    /// the command stays pending until it runs it.
    Firmware { tag: String },
    /// How often anchor links survey, in seconds (10–3600); 0 = the default, 15 min.
    Survey {
        #[arg(long)]
        period_s: u16,
    },
}

/// How long to wait for the sink to mirror the record back.
const ECHO_WITHIN: Duration = Duration::from_secs(5);

pub fn run(args: &ControlArgs) -> Result<()> {
    let fleet = Fleet::open(&args.fleet)?;
    let registry = fleet.registry()?;
    let targets = parse_targets(&args.target, &registry)?;
    let command = match &args.command {
        ControlCmd::Blink { times } => Command::Blink { times: *times },
        ControlCmd::Environment { period_s, gas } => Command::Environment {
            period_s: *period_s,
            gas: *gas,
        },
        ControlCmd::Diversity { on } => Command::Diversity(*on),
        ControlCmd::Advertising { interval_ms } => Command::Advertising {
            interval_ms: *interval_ms,
        },
        ControlCmd::Firmware { tag } => {
            Command::Firmware(parse_tag(tag).with_context(|| format!("{tag}: not a build tag like 0.1.0+1950d45d"))?)
        }
        ControlCmd::Survey { period_s } => Command::Survey { period_s: *period_s },
    };
    let key = auth::controller_key(&fleet.key()?);
    // The controller's epoch is the clock: every run writes newer versions than the last.
    let epoch = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(1, |d| d.as_secs() as u32);
    let mut writer = Writer::new(CONTROLLER, epoch);
    let records: Vec<Record> = targets
        .iter()
        .map(|&target| auth::sign(&mut writer, &key, target, command))
        .collect();
    let mut port = serialport::new(&args.sink, args.baud)
        .timeout(Duration::from_millis(200))
        .open()
        .with_context(|| format!("opening {}", args.sink))?;
    // One record at a time, each sent again if the sink doesn't mirror it: records sent back to
    // back get lost in the sink's UART receive (DEF-43).
    let mut pending = Vec::new();
    let mut awaited = Vec::new();
    for (record, &target) in records.iter().zip(&targets) {
        let mut record_buf = [0u8; navgrid_proto::MESH_RECORD_LEN];
        let bytes = postcard::to_slice(record, &mut record_buf).context("encoding the record")?;
        let mut out = vec![0u8; MAX_DOWNLINK_FRAME];
        let frame = navgrid_proto::encode_downlink(&UpdateMsg::Record(bytes), &mut out);
        let mut echoed = false;
        for _attempt in 0..3 {
            port.write_all(frame).context("writing to the sink")?;
            if wait_echo(&mut *port, &mut pending, record.version, ECHO_WITHIN)? {
                echoed = true;
                break;
            }
        }
        println!(
            "sent to {}: {} v{}.{} for {}{}",
            Path::new(&args.sink).display(),
            command_text(&command),
            record.version.epoch,
            record.version.counter,
            name(target, &registry),
            if echoed { "" } else { ", not mirrored back" }
        );
        if !echoed {
            awaited.push(record.version);
        }
    }
    if awaited.is_empty() {
        println!(
            "the sink took {}; sessions carry {} to the target{}, and the monitor shows the report",
            if records.len() == 1 {
                "it".to_string()
            } else {
                format!("all {}", records.len())
            },
            if records.len() == 1 { "it" } else { "them" },
            if records.len() == 1 { "" } else { "s" },
        );
        return Ok(());
    }
    bail!(
        "the sink didn't mirror {} of {} record(s) back within {} s: is it a sink on protocol 6?",
        awaited.len(),
        records.len(),
        ECHO_WITHIN.as_secs()
    )
}

/// Reads the sink until it mirrors the controller's record of `version` back, for up to `within`.
fn wait_echo(
    port: &mut dyn serialport::SerialPort,
    pending: &mut Vec<u8>,
    version: navgrid_mesh::Version,
    within: Duration,
) -> Result<bool> {
    let deadline = Instant::now() + within;
    while Instant::now() < deadline {
        let mut buf = [0u8; 256];
        match port.read(&mut buf) {
            Ok(n) => pending.extend_from_slice(&buf[..n]),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => return Err(e).context("reading from the sink"),
        }
        for mut f in monitor::take_frames(pending) {
            if let Ok(Message::Record(r)) = navgrid_proto::decode(&mut f)
                && let Ok(echo) = postcard::from_bytes::<Record>(&r.bytes)
                && echo.origin == CONTROLLER
                && matches!(echo.body, Body::Desired(_))
                && echo.version == version
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// `all` is every enrolled device; anything else is one target.
pub fn parse_targets(s: &str, registry: &Registry) -> Result<Vec<NodeId>> {
    if s == "all" {
        let all: Vec<NodeId> = registry
            .devices
            .iter()
            .filter(|d| d.status != crate::fleet::Status::Retired)
            .map(|d| d.id)
            .collect();
        if all.is_empty() {
            bail!("no enrolled devices in the registry");
        }
        return Ok(all);
    }
    Ok(vec![parse_target(s, registry)?])
}

/// A command in words: `Blink { times: 3 }`, `Firmware 0.1.0+1950d45d`.
pub fn command_text(command: &Command) -> String {
    match command {
        Command::Firmware(fw) => format!("Firmware {fw}"),
        other => format!("{other:?}"),
    }
}

pub fn parse_target(s: &str, registry: &crate::fleet::Registry) -> Result<NodeId> {
    if let Some(number) = s.strip_prefix('N').and_then(|n| n.parse::<u16>().ok())
        && let Some(d) = registry.by_number(number)
    {
        return Ok(d.id);
    }
    if s.len() == 12
        && let Ok(id) = u64::from_str_radix(s, 16)
    {
        return Ok(NodeId(id));
    }
    bail!("target {s}: not a fleet number in the registry (N042) nor a node ID (12 hex digits)")
}

fn name(id: NodeId, registry: &crate::fleet::Registry) -> String {
    registry
        .by_id(id)
        .map_or_else(|| format!("{:012x}", id.0), |d| label_name(d.number))
}
