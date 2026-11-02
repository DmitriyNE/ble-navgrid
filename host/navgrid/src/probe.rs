//! `probe-rs` through the command line: a device's ID from its FICR, writing images, resetting. At
//! the firmware runner's settings: SWD at 100 kHz and no double buffering (DEF-16, DEF-19).

use std::path::Path;
use std::process::{Command, Output};

use anyhow::{Context, Result, bail};
use navgrid_proto::NodeId;

/// FICR `DEVICEADDR[0]`, the first of the two words the node ID comes from.
const FICR_DEVICEADDR: u32 = 0x00ff_c3a4;

fn probe_rs(subcommand: &str, probe: Option<&str>) -> Command {
    let mut cmd = Command::new("probe-rs");
    cmd.arg(subcommand).args(["--chip", "nRF54L15"]);
    if let Some(p) = probe {
        cmd.args(["--probe", p]);
    }
    cmd
}

fn output(cmd: &mut Command) -> Result<Output> {
    let out = cmd.output().context("running probe-rs")?;
    if !out.status.success() {
        let err = String::from_utf8_lossy(&out.stderr);
        let last = err.lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("").trim();
        bail!("probe-rs failed: {last}");
    }
    Ok(out)
}

/// The device's node ID, from its FICR.
pub fn read_id(probe: Option<&str>) -> Result<NodeId> {
    let mut read = probe_rs("read", probe);
    read.args(["b32", &format!("{FICR_DEVICEADDR:#x}"), "2"]);
    let out = output(&mut read)?;
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

/// Writes `file` and verifies it: an ELF, or raw bytes at `base` when given.
pub fn download(probe: Option<&str>, file: &Path, base: Option<u32>) -> Result<()> {
    let mut download = probe_rs("download", probe);
    download.args(["--speed", "100", "--disable-double-buffering", "--verify"]);
    if let Some(base) = base {
        download
            .args(["--binary-format", "bin", "--base-address"])
            .arg(format!("{base:#x}"));
    }
    download.arg(file);
    output(&mut download).map(|_| ())
}

pub fn reset(probe: Option<&str>) -> Result<()> {
    output(&mut probe_rs("reset", probe)).map(|_| ())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_ficr_words() {
        assert_eq!(
            parse_read("00ffc3a4: dfcc4c2d d1aece36\n").unwrap(),
            NodeId(0xce36_dfcc_4c2d)
        );
        assert!(parse_read("00ffc3a4: dfcc4c2d\n").is_err());
    }
}
