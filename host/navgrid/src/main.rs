mod builds;
mod calibration;
mod capture;
mod control;
mod fix;
mod flash;
mod fleet;
mod link_sim;
mod mesh_sim;
mod mesh_view;
mod monitor;
mod probe;
mod provision;
mod scenario;
mod sign;
mod tones;
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
    /// Run the estimators on the raw tones initiators send through the sink (`navgrid monitor --json` logs).
    Tones(tones::TonesArgs),
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
    /// The link policy over a radio timeline: links, bursts, ranges per second and empty
    /// procedures per node kind, for the firmware's settings.
    LinkSim(link_sim::LinkSimArgs),
    /// The mesh state a sink mirrored into monitor logs: nodes, health, links, positions, commands.
    Mesh(mesh_view::MeshViewArgs),
    /// Enrol a device through its probe: fleet key, fleet number, role, bring-up peer and sink.
    Provision(provision::ProvisionArgs),
    /// Flash firmware through one or more probes at once, and record the build in the registry.
    Flash(flash::FlashArgs),
    /// Make a signed update file from a firmware ELF, with the fleet's signing key.
    Sign(sign::SignArgs),
    /// Stage a signed update on a device through its probe; it checks and installs it at boot.
    Update(sign::UpdateArgs),
    /// Command a node through a sink: a desired-state record, signed with the controller key.
    Control(control::ControlArgs),
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
        Cmd::Tones(args) => tones::run(&args),
        Cmd::Decode { path, json } => monitor::decode(path, json),
        Cmd::Monitor {
            port,
            baud,
            json,
            calibration,
            fleet,
        } => {
            let cal = calibration.as_deref().map(calibration::Calibration::load).transpose()?;
            let fleet = fleet::Fleet::open(&fleet).ok();
            let registry = fleet.as_ref().map(|f| f.registry()).transpose()?;
            let mut view = monitor::View::new(cal, registry);
            if let Some(fleet) = fleet {
                view = view.with_fleet(fleet);
            }
            monitor::monitor(&port, baud, json, view)
        }
        Cmd::Calibrate(args) => calibration::run(&args),
        Cmd::MeshSim(args) => mesh_sim::run(&args),
        Cmd::LinkSim(args) => link_sim::run(&args),
        Cmd::Mesh(args) => mesh_view::run(&args),
        Cmd::Provision(args) => provision::run(&args),
        Cmd::Flash(args) => flash::run(&args),
        Cmd::Sign(args) => sign::run_sign(&args),
        Cmd::Update(args) => sign::run_update(&args),
        Cmd::Control(args) => control::run(&args),
        Cmd::Fleet(FleetCmd::Init(args)) => fleet::Fleet::init(&args).map(|_| ()),
        Cmd::Fleet(FleetCmd::List(args)) => fleet::list(&args),
    }
}
