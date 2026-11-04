//! `navgrid provision`: enrolment (FR-O1, D-29). It reads the device's ID through the probe,
//! keeps or assigns its fleet number from the registry, writes the provisioning page to
//! [`PROVISIONING_ADDRESS`], resets the device, and writes its label. Flashing firmware leaves
//! the page alone.

use std::path::PathBuf;

use anyhow::{Context, Result};
use navgrid_proto::{
    MAX_PROVISIONING_LEN, NodeConfig, NodeId, PROVISIONING_ADDRESS, PointMm, Provisioning, ReportLevel, Role,
};

use crate::fleet::{Fleet, FleetArgs, label_name, today};
use crate::probe;

#[derive(clap::Args)]
pub struct ProvisionArgs {
    /// Debug probe of the device, as `probe-rs list` shows it (e.g. a DK's J-Link serial).
    #[arg(long)]
    probe: Option<String>,
    #[command(flatten)]
    fleet: FleetArgs,
    /// Fleet number; by default the device keeps its own, or a new device gets the next one.
    #[arg(long)]
    number: Option<u16>,
    #[arg(long, value_enum, default_value = "anchor")]
    role: RoleArg,
    /// Also a sink: a host link over USB, serving the host, whatever the role.
    #[arg(long)]
    sink: bool,
    /// Bring-up field, ignored by the mesh firmware (it picks its own links): the reflector to
    /// range with (node ID as the device logs it, 12 hex digits).
    #[arg(long, value_parser = parse_node)]
    peer: Option<NodeId>,
    /// Bring-up field, ignored by the mesh firmware: the sink to report ranges to.
    #[arg(long, value_parser = parse_node)]
    report_to: Option<NodeId>,
    /// Hold the radio on antenna 0: one antenna path per link.
    #[arg(long)]
    no_diversity: bool,
    /// Surveyed position, millimetres: `X,Y,Z`.
    #[arg(long, value_parser = parse_pin)]
    pin: Option<PointMm>,
    #[arg(long, default_value_t = NodeConfig::default().adv_interval_ms)]
    adv_interval_ms: u16,
    /// Environmental sampling period, 0 = off.
    #[arg(long, default_value_t = NodeConfig::default().env_interval_s)]
    env_interval_s: u16,
    /// Also sample the BME688 gas sensor.
    #[arg(long)]
    gas: bool,
    /// How much of its log the device sends toward the host: errors only, as in production, or
    /// more on a dev bench.
    #[arg(long, value_enum, default_value = "error")]
    report_level: ReportLevelArg,
    /// Write the page to this file instead of the device; the registry is left alone.
    #[arg(long)]
    out: Option<PathBuf>,
}

#[derive(Clone, Copy, clap::ValueEnum)]
enum RoleArg {
    Anchor,
    Asset,
    Mobile,
}

#[derive(Clone, Copy, clap::ValueEnum)]
enum ReportLevelArg {
    Off,
    Error,
    Warn,
    Info,
    Debug,
    Trace,
}

impl From<ReportLevelArg> for ReportLevel {
    fn from(l: ReportLevelArg) -> Self {
        match l {
            ReportLevelArg::Off => ReportLevel::Off,
            ReportLevelArg::Error => ReportLevel::Error,
            ReportLevelArg::Warn => ReportLevel::Warn,
            ReportLevelArg::Info => ReportLevel::Info,
            ReportLevelArg::Debug => ReportLevel::Debug,
            ReportLevelArg::Trace => ReportLevel::Trace,
        }
    }
}

impl From<RoleArg> for Role {
    fn from(r: RoleArg) -> Self {
        match r {
            RoleArg::Anchor => Role::Anchor,
            RoleArg::Asset => Role::Asset,
            RoleArg::Mobile => Role::Mobile,
        }
    }
}

fn parse_node(s: &str) -> Result<NodeId, String> {
    let id = u64::from_str_radix(s, 16).map_err(|_| format!("{s:?} is not hex"))?;
    if s.len() != 12 {
        return Err(format!("{s:?}: a node ID is 12 hex digits"));
    }
    Ok(NodeId(id))
}

fn parse_pin(s: &str) -> Result<PointMm, String> {
    let v: Vec<i32> = s
        .split(',')
        .map(|x| x.trim().parse().map_err(|_| format!("bad coordinate {x:?}")))
        .collect::<Result<_, _>>()?;
    let [x, y, z] = v[..] else {
        return Err("expected X,Y,Z in millimetres".into());
    };
    Ok(PointMm { x, y, z })
}

pub fn run(args: &ProvisionArgs) -> Result<()> {
    let fleet = Fleet::open(&args.fleet)?;
    let mut registry = fleet.registry()?;
    let id = match &args.out {
        Some(_) => None,
        None => Some(probe::read_id(args.probe.as_deref())?),
    };
    let number = args
        .number
        .or_else(|| id.and_then(|id| registry.by_id(id)).map(|d| d.number))
        .or_else(|| id.map(|_| registry.next_number()));
    let signing_key = match fleet.signing_key() {
        Ok(k) => Some(k.verifying_key().to_bytes()),
        Err(e) => {
            println!("no signing key ({e:#}); the device will take no updates. `navgrid fleet init` makes one.");
            None
        }
    };
    let p = Provisioning {
        config: NodeConfig {
            role: args.role.into(),
            sink: args.sink,
            diversity: !args.no_diversity,
            pinned: args.pin,
            adv_interval_ms: args.adv_interval_ms,
            env_interval_s: args.env_interval_s,
            gas: args.gas,
        },
        fleet_key: fleet.key()?,
        peer: args.peer,
        sink: args.report_to,
        number,
        signing_key,
        report_level: args.report_level.into(),
    };
    let mut buf = [0u8; MAX_PROVISIONING_LEN];
    let page = navgrid_proto::encode_provisioning(&p, &mut buf)?;
    let hex = |id: Option<NodeId>| id.map_or("none".into(), |n| format!("{:012x}", n.0));
    println!(
        "{} {}: {:?}{}, diversity {}, pin {:?}, peer {}, reports to {}; {} bytes",
        number.map_or("unnumbered".into(), label_name),
        id.map_or("(no device)".into(), |id| format!("{:012x}", id.0)),
        p.config.role,
        if p.config.sink { " + sink" } else { "" },
        if p.config.diversity { "on" } else { "off" },
        p.config.pinned,
        hex(p.peer),
        hex(p.sink),
        page.len()
    );
    if let Some(out) = &args.out {
        std::fs::write(out, page).with_context(|| format!("writing {}", out.display()))?;
        println!("wrote {}", out.display());
        return Ok(());
    }
    let (Some(id), Some(number)) = (id, number) else {
        unreachable!("a device to provision has an ID and a number")
    };
    // Refuse a taken number before touching the device.
    registry.enrol(id, number, p.config.role, p.config.sink, today())?;
    // probe-rs reads the page from a file; it holds the fleet key, so it doesn't stay.
    let file = std::env::temp_dir().join(format!("navgrid-provision-{}.bin", std::process::id()));
    std::fs::write(&file, page).with_context(|| format!("writing {}", file.display()))?;
    let probe = args.probe.as_deref();
    let flashed = probe::download(probe, &file, Some(PROVISIONING_ADDRESS)).and_then(|()| probe::reset(probe));
    let _ = std::fs::remove_file(&file);
    flashed?;
    fleet.save(&registry)?;
    let label = fleet.write_label(number, id)?;
    println!("{} enrolled; label {}", label_name(number), label.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_ids_and_pins() {
        assert_eq!(parse_node("df96e04a5900"), Ok(NodeId(0xdf96_e04a_5900)));
        assert!(parse_node("df96e04a59").is_err());
        assert_eq!(
            parse_pin("1200,-300, 2700"),
            Ok(PointMm {
                x: 1200,
                y: -300,
                z: 2700
            })
        );
        assert!(parse_pin("1,2").is_err());
    }
}
