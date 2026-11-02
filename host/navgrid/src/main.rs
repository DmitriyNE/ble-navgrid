mod calibration;
mod capture;
mod fix;
mod flash;
mod fleet;
mod mesh_sim;
mod monitor;
mod probe;
mod provision;
mod scenario;
mod track;

use std::path::PathBuf;

use anyhow::Result;
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
        /// Name nodes by fleet number from this fleet's registry (default: `NAVGRID_FLEET` or
        /// `~/.config/navgrid`, when there is one).
        #[command(flatten)]
        fleet: fleet::FleetArgs,
    },
    /// Fit range offsets from `monitor --json` captures taken at measured distances.
    Calibrate(calibration::CalibrateArgs),
    /// The mesh state protocol over a synthetic deployment: staleness, command latency, bytes per
    /// contact and memory per node.
    MeshSim(mesh_sim::MeshSimArgs),
    /// Enrol a device through its probe: fleet key, fleet number, role, bring-up peer and sink.
    Provision(provision::ProvisionArgs),
    /// Flash firmware through one or more probes at once, and record the build in the registry.
    Flash(flash::FlashArgs),
    /// The fleet directory: key, device registry, labels.
    #[command(subcommand)]
    Fleet(FleetCmd),
}

#[derive(Subcommand)]
enum FleetCmd {
    /// Create the fleet directory: a new fleet key and an empty registry, keeping any that exist.
    Init(fleet::FleetArgs),
    /// List the enrolled devices.
    List(fleet::FleetArgs),
}

fn main() -> Result<()> {
    match Cli::parse().cmd {
        Cmd::Sim(args) => fix::run(&args),
        Cmd::Track(args) => track::run(&args),
        Cmd::Cs(args) => capture::run(&args),
        Cmd::Decode { path, json } => monitor::decode(path, json),
        Cmd::Monitor {
            port,
            baud,
            json,
            calibration,
            fleet,
        } => {
            let cal = calibration.as_deref().map(calibration::Calibration::load).transpose()?;
            let registry = fleet::Fleet::open(&fleet).ok().map(|f| f.registry()).transpose()?;
            monitor::monitor(&port, baud, json, monitor::View::new(cal, registry))
        }
        Cmd::Calibrate(args) => calibration::run(&args),
        Cmd::MeshSim(args) => mesh_sim::run(&args),
        Cmd::Provision(args) => provision::run(&args),
        Cmd::Flash(args) => flash::run(&args),
        Cmd::Fleet(FleetCmd::Init(args)) => fleet::Fleet::init(&args).map(|_| ()),
        Cmd::Fleet(FleetCmd::List(args)) => fleet::list(&args),
    }
}
