//! `navgrid provision`: enrolment (FR-O1, D-29). It reads the device's ID through the probe,
//! keeps or assigns its fleet number from the registry, writes the provisioning page to
//! [`PROVISIONING_ADDRESS`], resets the device, and writes its label. Flashing firmware leaves
//! the page alone.

use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{Context, Result, bail};
use navgrid_proto::{MAX_PROVISIONING_LEN, NodeConfig, NodeId, PROVISIONING_ADDRESS, PointMm, Provisioning, Role};

use crate::fleet::{Fleet, FleetArgs, label_name, today};

/// FICR `DEVICEADDR[0]`, the first of the two words the node ID comes from.
const FICR_DEVICEADDR: u32 = 0x00ff_c3a4;

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
    #[arg(long, value_enum, default_value = "node")]
    role: RoleArg,
    /// Bring-up: range with this reflector as CS initiator (node ID as the device logs it, 12 hex
    /// digits).
    #[arg(long, value_parser = parse_node)]
    peer: Option<NodeId>,
    /// Bring-up: report ranges to this sink.
    #[arg(long, value_parser = parse_node)]
    sink: Option<NodeId>,
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
    /// Write the page to this file instead of the device; the registry is left alone.
    #[arg(long)]
    out: Option<PathBuf>,
}

#[derive(Clone, Copy, clap::ValueEnum)]
enum RoleArg {
    Node,
    Asset,
    Person,
    Sink,
}

impl From<RoleArg> for Role {
    fn from(r: RoleArg) -> Self {
        match r {
            RoleArg::Node => Role::Node,
            RoleArg::Asset => Role::Asset,
            RoleArg::Person => Role::Person,
            RoleArg::Sink => Role::Sink,
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
        None => Some(read_id(args.probe.as_deref())?),
    };
    let number = args
        .number
        .or_else(|| id.and_then(|id| registry.by_id(id)).map(|d| d.number))
        .or_else(|| id.map(|_| registry.next_number()));
    let p = Provisioning {
        config: NodeConfig {
            role: args.role.into(),
            diversity: !args.no_diversity,
            pinned: args.pin,
            adv_interval_ms: args.adv_interval_ms,
            env_interval_s: args.env_interval_s,
            gas: args.gas,
        },
        fleet_key: fleet.key()?,
        peer: args.peer,
        sink: args.sink,
        number,
    };
    let mut buf = [0u8; MAX_PROVISIONING_LEN];
    let page = navgrid_proto::encode_provisioning(&p, &mut buf)?;
    let hex = |id: Option<NodeId>| id.map_or("none".into(), |n| format!("{:012x}", n.0));
    println!(
        "{} {}: {:?}, diversity {}, pin {:?}, peer {}, sink {}; {} bytes",
        number.map_or("unnumbered".into(), label_name),
        id.map_or("(no device)".into(), |id| format!("{:012x}", id.0)),
        p.config.role,
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
    registry.enrol(id, number, p.config.role, today())?;
    // probe-rs reads the page from a file; it holds the fleet key, so it doesn't stay.
    let file = std::env::temp_dir().join(format!("navgrid-provision-{}.bin", std::process::id()));
    std::fs::write(&file, page).with_context(|| format!("writing {}", file.display()))?;
    let flashed = flash(args.probe.as_deref(), &file);
    let _ = std::fs::remove_file(&file);
    flashed?;
    fleet.save(&registry)?;
    let label = fleet.write_label(number, id)?;
    println!("{} enrolled; label {}", label_name(number), label.display());
    Ok(())
}

/// The device's node ID, from its FICR through the probe.
fn read_id(probe: Option<&str>) -> Result<NodeId> {
    let mut read = Command::new("probe-rs");
    read.arg("read").args(["--chip", "nRF54L15"]);
    if let Some(p) = probe {
        read.args(["--probe", p]);
    }
    read.args(["b32", &format!("{FICR_DEVICEADDR:#x}"), "2"]);
    let out = read.output().context("running probe-rs")?;
    if !out.status.success() {
        bail!("probe-rs read failed: {}", String::from_utf8_lossy(&out.stderr).trim());
    }
    parse_read(&String::from_utf8_lossy(&out.stdout))
}

/// `00ffc3a4: dfcc4c2d d1aece36` → the node ID.
fn parse_read(text: &str) -> Result<NodeId> {
    let line = text
        .lines()
        .find(|l| l.contains(':'))
        .context("no words in probe-rs output")?;
    let words: Vec<u32> = line
        .split(':')
        .nth(1)
        .unwrap_or("")
        .split_whitespace()
        .map(|w| u32::from_str_radix(w, 16))
        .collect::<Result<_, _>>()
        .context("unexpected probe-rs output")?;
    let [w0, w1] = words[..] else {
        bail!("expected two words from probe-rs, got {line:?}");
    };
    Ok(NodeId::from_ficr([w0, w1]))
}

/// Writes the page and resets the device, at the firmware runner's settings (DEF-16, DEF-19).
fn flash(probe: Option<&str>, file: &Path) -> Result<()> {
    let probe_args = |cmd: &mut Command| {
        cmd.args(["--chip", "nRF54L15"]);
        if let Some(p) = probe {
            cmd.args(["--probe", p]);
        }
    };
    let mut download = Command::new("probe-rs");
    download.arg("download");
    probe_args(&mut download);
    download
        .args(["--speed", "100", "--disable-double-buffering", "--verify"])
        .args(["--binary-format", "bin", "--base-address"])
        .arg(format!("{PROVISIONING_ADDRESS:#x}"))
        .arg(file);
    run_cmd(&mut download)?;
    let mut reset = Command::new("probe-rs");
    reset.arg("reset");
    probe_args(&mut reset);
    run_cmd(&mut reset)?;
    println!("provisioned and reset");
    Ok(())
}

fn run_cmd(cmd: &mut Command) -> Result<()> {
    let status = cmd.status().context("running probe-rs")?;
    if !status.success() {
        bail!("probe-rs failed: {status}");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_ids_pins_and_probe_output() {
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
        assert_eq!(
            parse_read("00ffc3a4: dfcc4c2d d1aece36\n").unwrap(),
            NodeId(0xce36_dfcc_4c2d)
        );
        assert!(parse_read("00ffc3a4: dfcc4c2d\n").is_err());
    }
}
