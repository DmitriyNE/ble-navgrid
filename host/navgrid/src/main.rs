mod fix;
mod scenario;
mod track;

use std::io::Read;
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(version, about = "navgrid host tools")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// One-shot fixes from simulated CS ranges: ranging → multilateration.
    Sim(fix::SimArgs),
    /// A simulated walking target followed one range at a time; ranging rate vs accuracy, with
    /// and without IMU motion gating.
    Track(track::TrackArgs),
    /// Decode COBS/postcard frames from a capture file or stdin.
    Decode {
        /// Input file; stdin if omitted.
        path: Option<PathBuf>,
        /// One JSON object per line, for log shippers and monitoring pipelines.
        #[arg(long)]
        json: bool,
    },
}

fn main() -> Result<()> {
    match Cli::parse().cmd {
        Cmd::Sim(args) => fix::run(&args),
        Cmd::Track(args) => track::run(&args),
        Cmd::Decode { path, json } => decode(path, json),
    }
}

fn decode(path: Option<PathBuf>, json: bool) -> Result<()> {
    let mut data = Vec::new();
    match path {
        Some(p) => data = std::fs::read(&p).with_context(|| format!("reading {}", p.display()))?,
        None => _ = std::io::stdin().read_to_end(&mut data)?,
    }
    for frame in data.split(|&b| b == 0).filter(|f| !f.is_empty()) {
        match render(frame, json) {
            Ok(line) => println!("{line}"),
            Err(e) => eprintln!("bad frame ({e}): {frame:02x?}"),
        }
    }
    Ok(())
}

/// One COBS frame (without its `0x00` terminator) as a line of text.
fn render(frame: &[u8], json: bool) -> Result<String> {
    let msg = navgrid_proto::decode(&mut frame.to_vec())?;
    Ok(if json {
        serde_json::to_string(&msg)?
    } else {
        format!("{msg:?}")
    })
}

#[cfg(test)]
mod tests {
    use navgrid_proto::{Environment, Message, NodeId};

    use super::*;

    #[test]
    fn renders_environment_as_json() {
        let msg = Message::Environment(Environment {
            node: NodeId(0xc0ffee),
            uptime_s: 42,
            temperature_cdeg: 2150,
            humidity_cpct: 4100,
            pressure_cpa: 10_132_500,
            gas_ohm: None,
        });
        let mut buf = [0u8; navgrid_proto::MAX_FRAME_LEN];
        let frame = navgrid_proto::encode(&msg, &mut buf).unwrap();
        let body = &frame[..frame.len() - 1];
        let line = render(body, true).unwrap();
        let value: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(value["Environment"]["pressure_cpa"], 10_132_500);
        assert_eq!(value["Environment"]["gas_ohm"], serde_json::Value::Null);
    }
}
