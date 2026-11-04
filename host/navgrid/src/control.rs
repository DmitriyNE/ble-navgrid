//! `navgrid control`: the controller's commands as desired-state records (FR-C5, architecture
//! §12). The host signs the record with the controller key (from the fleet key, DEF-27), sends it
//! to a sink over its serial port, and the sink injects it into its replica, from where sessions
//! carry it toward the target; the target acts on it and writes a Reported record, which the
//! monitor shows. The sink mirrors the injected record back at once, which this command waits for.

use std::io::Write;
use std::path::Path;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use navgrid_mesh::{Body, CONTROLLER, Command, NodeId, Record, Writer, auth};
use navgrid_proto::{MAX_DOWNLINK_FRAME, Message, UpdateMsg};

use crate::fleet::{Fleet, FleetArgs, label_name};
use crate::monitor;

#[derive(clap::Args)]
pub struct ControlArgs {
    /// The sink's serial port (its VCOM1). Stop `navgrid monitor` on the port first.
    #[arg(long)]
    sink: String,
    #[arg(long, default_value_t = 115_200)]
    baud: u32,
    /// The node: its fleet number (`N042`) or node ID (12 hex digits).
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
        #[arg(value_parser = clap::builder::BoolishValueParser::new())]
        on: bool,
    },
    /// Advertising interval.
    Advertising {
        #[arg(long)]
        interval_ms: u16,
    },
}

/// How long to wait for the sink to mirror the record back.
const ECHO_WITHIN: Duration = Duration::from_secs(5);

pub fn run(args: &ControlArgs) -> Result<()> {
    let fleet = Fleet::open(&args.fleet)?;
    let registry = fleet.registry()?;
    let target = parse_target(&args.target, &registry)?;
    let command = match args.command {
        ControlCmd::Blink { times } => Command::Blink { times },
        ControlCmd::Environment { period_s, gas } => Command::Environment { period_s, gas },
        ControlCmd::Diversity { on } => Command::Diversity(on),
        ControlCmd::Advertising { interval_ms } => Command::Advertising { interval_ms },
    };
    let key = auth::controller_key(&fleet.key()?);
    // The controller's epoch is the clock: every run writes newer versions than the last.
    let epoch = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(1, |d| d.as_secs() as u32);
    let mut writer = Writer::new(CONTROLLER, epoch);
    let record = auth::sign(&mut writer, &key, target, command);
    let mut record_buf = [0u8; navgrid_proto::MESH_RECORD_LEN];
    let bytes = postcard::to_slice(&record, &mut record_buf).context("encoding the record")?;
    let mut out = vec![0u8; MAX_DOWNLINK_FRAME];
    let frame = navgrid_proto::encode_downlink(&UpdateMsg::Record(bytes), &mut out);
    let mut port = serialport::new(&args.sink, args.baud)
        .timeout(Duration::from_millis(200))
        .open()
        .with_context(|| format!("opening {}", args.sink))?;
    port.write_all(frame).context("writing to the sink")?;
    println!(
        "sent to {}: {:?} v{}.{} for {}",
        Path::new(&args.sink).display(),
        command,
        record.version.epoch,
        record.version.counter,
        name(target, &registry)
    );
    let deadline = Instant::now() + ECHO_WITHIN;
    let mut pending = Vec::new();
    while Instant::now() < deadline {
        let mut buf = [0u8; 256];
        match port.read(&mut buf) {
            Ok(n) => pending.extend_from_slice(&buf[..n]),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => return Err(e).context("reading from the sink"),
        }
        for mut f in monitor::take_frames(&mut pending) {
            if let Ok(Message::Record(r)) = navgrid_proto::decode(&mut f)
                && let Ok(echo) = postcard::from_bytes::<Record>(&r.bytes)
                && echo.origin == record.origin
                && echo.version == record.version
                && matches!(echo.body, Body::Desired(_))
            {
                println!("the sink took it; sessions carry it to the target, and the monitor shows the report");
                return Ok(());
            }
        }
    }
    bail!(
        "the sink didn't mirror the record back within {} s: is it a sink on protocol 6?",
        ECHO_WITHIN.as_secs()
    )
}

pub fn parse_target(s: &str, registry: &crate::fleet::Registry) -> Result<NodeId> {
    if let Some(number) = s.strip_prefix('N').and_then(|n| n.parse::<u16>().ok())
        && let Some(d) = registry.devices.iter().find(|d| d.number == number)
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
