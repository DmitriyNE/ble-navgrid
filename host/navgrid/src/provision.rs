//! `navgrid provision` and `navgrid fleet-key`: a device's identity and configuration (FR-O1). The
//! provisioning page goes to [`PROVISIONING_ADDRESS`] through `probe-rs`; the firmware reads it
//! at boot. Flashing firmware leaves it alone.

use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{Context, Result, bail};
use navgrid_proto::{MAX_PROVISIONING_LEN, NodeConfig, NodeId, PROVISIONING_ADDRESS, PointMm, Provisioning, Role};

#[derive(clap::Args)]
pub struct ProvisionArgs {
    /// Debug probe of the device, as `probe-rs list` shows it (e.g. a DK's J-Link serial).
    #[arg(long)]
    probe: Option<String>,
    /// Fleet key file, from `navgrid fleet-key`.
    #[arg(long)]
    fleet_key: PathBuf,
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
    /// Write the page to this file instead of the device.
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

/// Reads a fleet key file: 32 hex digits.
fn read_key(path: &Path) -> Result<[u8; 16]> {
    let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    let hex = text.trim();
    if hex.len() != 32 {
        bail!("{}: a fleet key is 32 hex digits", path.display());
    }
    let mut key = [0u8; 16];
    for (i, b) in key.iter_mut().enumerate() {
        *b = u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).with_context(|| format!("{}: not hex", path.display()))?;
    }
    Ok(key)
}

pub fn run(args: &ProvisionArgs) -> Result<()> {
    let p = Provisioning {
        config: NodeConfig {
            role: args.role.into(),
            diversity: !args.no_diversity,
            pinned: args.pin,
            adv_interval_ms: args.adv_interval_ms,
            env_interval_s: args.env_interval_s,
            gas: args.gas,
        },
        fleet_key: read_key(&args.fleet_key)?,
        peer: args.peer,
        sink: args.sink,
    };
    let mut buf = [0u8; MAX_PROVISIONING_LEN];
    let page = navgrid_proto::encode_provisioning(&p, &mut buf)?;
    let hex = |id: Option<NodeId>| id.map_or("none".into(), |n| format!("{:012x}", n.0));
    println!(
        "{:?}, diversity {}, pin {:?}, peer {}, sink {}; {} bytes",
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
    // probe-rs reads the page from a file; it holds the fleet key, so it doesn't stay.
    let file = std::env::temp_dir().join(format!("navgrid-provision-{}.bin", std::process::id()));
    std::fs::write(&file, page).with_context(|| format!("writing {}", file.display()))?;
    let flashed = flash(args.probe.as_deref(), &file);
    let _ = std::fs::remove_file(&file);
    flashed
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

/// `navgrid fleet-key FILE`: a new random fleet key, readable by its owner only.
pub fn new_key(path: &Path) -> Result<()> {
    use std::os::unix::fs::OpenOptionsExt;
    let mut key = [0u8; 16];
    std::fs::File::open("/dev/urandom")
        .and_then(|mut f| f.read_exact(&mut key))
        .context("reading /dev/urandom")?;
    let hex: String = key.iter().map(|b| format!("{b:02x}")).collect();
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(0o600)
        .open(path)
        .with_context(|| format!("creating {} (it must not exist yet)", path.display()))?;
    writeln!(file, "{hex}").with_context(|| format!("writing {}", path.display()))?;
    println!("wrote a new fleet key to {}", path.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_ids_pins_and_keys() {
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
        let path = std::env::temp_dir().join(format!("navgrid-key-test-{}", std::process::id()));
        std::fs::write(&path, "000102030405060708090a0b0c0d0e0f\n").unwrap();
        let key = read_key(&path);
        std::fs::remove_file(&path).unwrap();
        assert_eq!(key.unwrap(), core::array::from_fn(|i| i as u8));
    }
}
