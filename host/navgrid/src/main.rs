mod calibration;
mod capture;
mod fix;
mod mesh_sim;
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
    /// Distances from raw CS captures: the `CSR`/`CSC` lines both boards log over RTT.
    Cs(capture::CsArgs),
    /// Decode COBS/postcard frames from a capture file or stdin.
    Decode {
        /// Input file; stdin if omitted.
        path: Option<PathBuf>,
        /// One JSON object per line, for log shippers and monitoring pipelines.
        #[arg(long)]
        json: bool,
    },
    /// Print the frames a sink writes to its serial port as they arrive.
    Monitor {
        /// Serial device, e.g. the sink DK's VCOM1 (`/dev/cu.usbmodem…3` on macOS).
        port: String,
        #[arg(long, default_value_t = 115_200)]
        baud: u32,
        /// One JSON object per line.
        #[arg(long)]
        json: bool,
        /// Add each range's calibrated distance from this file (`calibrated_mm` in JSON).
        #[arg(long)]
        calibration: Option<PathBuf>,
    },
    /// Fit range offsets from `monitor --json` captures taken at measured distances.
    Calibrate(calibration::CalibrateArgs),
    /// The mesh state protocol over a synthetic deployment: staleness, command latency, bytes per
    /// contact and memory per node.
    MeshSim(mesh_sim::MeshSimArgs),
}

fn main() -> Result<()> {
    match Cli::parse().cmd {
        Cmd::Sim(args) => fix::run(&args),
        Cmd::Track(args) => track::run(&args),
        Cmd::Cs(args) => capture::run(&args),
        Cmd::Decode { path, json } => decode(path, json),
        Cmd::Monitor {
            port,
            baud,
            json,
            calibration,
        } => {
            let cal = calibration.as_deref().map(calibration::Calibration::load).transpose()?;
            monitor(&port, baud, json, cal.as_ref())
        }
        Cmd::Calibrate(args) => calibration::run(&args),
        Cmd::MeshSim(args) => mesh_sim::run(&args),
    }
}

fn monitor(path: &str, baud: u32, json: bool, cal: Option<&calibration::Calibration>) -> Result<()> {
    let mut port = serialport::new(path, baud)
        .timeout(std::time::Duration::from_secs(1))
        .open()
        .with_context(|| format!("opening {path}"))?;
    let mut pending = Vec::new();
    let mut buf = [0u8; 256];
    loop {
        match port.read(&mut buf) {
            Ok(n) => pending.extend_from_slice(&buf[..n]),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => return Err(e).with_context(|| format!("reading {path}")),
        }
        for frame in take_frames(&mut pending) {
            match render(&frame, json, cal) {
                Ok(line) => println!("{line}"),
                Err(e) => eprintln!("bad frame ({e}): {frame:02x?}"),
            }
        }
    }
}

/// Removes the complete frames (those ended by `0x00`) from the front of `pending`, without their
/// terminators. A partial frame stays for the next read; empty frames are skipped.
fn take_frames(pending: &mut Vec<u8>) -> Vec<Vec<u8>> {
    let mut frames = Vec::new();
    while let Some(end) = pending.iter().position(|&b| b == 0) {
        let frame: Vec<u8> = pending.drain(..=end).take(end).collect();
        if !frame.is_empty() {
            frames.push(frame);
        }
    }
    frames
}

fn decode(path: Option<PathBuf>, json: bool) -> Result<()> {
    let mut data = Vec::new();
    match path {
        Some(p) => data = std::fs::read(&p).with_context(|| format!("reading {}", p.display()))?,
        None => _ = std::io::stdin().read_to_end(&mut data)?,
    }
    for frame in data.split(|&b| b == 0).filter(|f| !f.is_empty()) {
        match render(frame, json, None) {
            Ok(line) => println!("{line}"),
            Err(e) => eprintln!("bad frame ({e}): {frame:02x?}"),
        }
    }
    Ok(())
}

/// One COBS frame (without its `0x00` terminator) as a line of text, with a range's calibrated
/// distance when `cal` has an entry for it.
fn render(frame: &[u8], json: bool, cal: Option<&calibration::Calibration>) -> Result<String> {
    let msg = navgrid_proto::decode(&mut frame.to_vec())?;
    let calibrated = match (&msg, cal) {
        (navgrid_proto::Message::Range(r), Some(cal)) => cal.apply(r),
        _ => None,
    };
    Ok(if json {
        let mut v = serde_json::to_value(msg)?;
        if let (Some(mm), Some(o)) = (calibrated, v.as_object_mut()) {
            o.insert("calibrated_mm".into(), mm.into());
        }
        v.to_string()
    } else {
        match calibrated {
            Some(mm) => format!("{msg:?} → calibrated {mm} mm"),
            None => format!("{msg:?}"),
        }
    })
}

#[cfg(test)]
mod tests {
    use navgrid_proto::{Environment, Message, NodeId};

    use super::*;

    #[test]
    fn frames_split_across_reads() {
        let mut pending = vec![0, 1, 2, 0, 3];
        assert_eq!(take_frames(&mut pending), vec![vec![1, 2]]);
        assert_eq!(pending, vec![3]);
        pending.extend_from_slice(&[4, 0, 0, 5]);
        assert_eq!(take_frames(&mut pending), vec![vec![3, 4]]);
        assert_eq!(pending, vec![5]);
    }

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
        let line = render(body, true, None).unwrap();
        let value: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(value["Environment"]["pressure_cpa"], 10_132_500);
        assert_eq!(value["Environment"]["gas_ohm"], serde_json::Value::Null);
    }
}
