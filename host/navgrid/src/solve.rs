//! `navgrid solve`: the central solver (FR-M6, D-28) over the mesh state a sink mirrored into
//! monitor logs: every node's position from the Link records between them, with the pinned
//! nodes fixed (their pins travel in their node records, or come from `--pin`). Prints the
//! positions and, with `--sink`, writes them to the mesh as the controller's Position records,
//! which reach the nodes by the sessions: an anchor without a pin then advertises it, and tags
//! range against it.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use navgrid_mesh::{Body, CONTROLLER, Heap, NodeId, PointMm, Position, Record, Replica, Writer};
use navgrid_proto::{MAX_DOWNLINK_FRAME, Message, UpdateMsg};
use navgrid_solver::mesh::{LinkObs, MeshOptions, Pin, solve};

use crate::fleet::{Fleet, FleetArgs};
use crate::mesh_view::{node_name, replica_from_logs};
use crate::monitor;

#[derive(clap::Args)]
pub struct SolveArgs {
    /// `navgrid monitor --json` logs, oldest first.
    #[arg(required = true)]
    files: Vec<PathBuf>,
    /// A pin the records don't carry: `N004=x,y,z` in metres.
    #[arg(long = "pin", value_parser = parse_pin)]
    pins: Vec<(String, [f32; 3])>,
    /// Taken off every distance: the chip's delay (DEF-6).
    #[arg(long, default_value_t = 1.5)]
    offset_m: f32,
    /// Height prior for unpinned nodes around the pins' mean height, 1-sigma.
    #[arg(long, default_value_t = 0.3)]
    height_sigma_m: f32,
    /// Only links at least this good (the burst's quality, 0–255).
    #[arg(long, default_value_t = 100)]
    min_quality: u8,
    /// Write the positions to the mesh through the sink on this serial port.
    #[arg(long)]
    sink: Option<String>,
    #[arg(long, default_value_t = 115_200)]
    baud: u32,
    #[command(flatten)]
    fleet: FleetArgs,
}

fn parse_pin(s: &str) -> Result<(String, [f32; 3]), String> {
    let (name, xyz) = s.split_once('=').ok_or("expected NAME=x,y,z")?;
    let v: Vec<f32> = xyz
        .split(',')
        .map(|p| p.trim().parse::<f32>())
        .collect::<Result<_, _>>()
        .map_err(|e| e.to_string())?;
    if v.len() != 3 {
        return Err("expected three coordinates".into());
    }
    Ok((name.to_string(), [v[0], v[1], v[2]]))
}

/// The frame version positions are solved in until referencing (FR-A) numbers them.
const FRAME: u16 = 1;
const ECHO_WITHIN: Duration = Duration::from_secs(5);

/// One node of a solution.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Placed {
    pub id: NodeId,
    pub position: [f32; 3],
    pub sigma_m: [f32; 3],
    pub pinned: bool,
}

/// What the solver made of the mesh.
#[derive(Clone, Debug)]
pub struct Solution {
    pub nodes: usize,
    pub pins: usize,
    pub links: usize,
    pub iterations: u8,
    pub normalized_rms: f32,
    pub outliers: usize,
    pub placed: Vec<Placed>,
    pub unsolved: Vec<NodeId>,
}

/// Solves the mesh in `replica` from its Link records, with the pins in its node records and
/// `extra_pins` (which override them).
pub fn solve_replica(
    replica: &Replica<Heap>,
    extra_pins: &[(NodeId, [f32; 3])],
    offset_m: f32,
    height_sigma_m: f32,
    min_quality: u8,
) -> Result<Solution> {
    // Nodes: every origin with a node record or a link.
    let mut ids: Vec<NodeId> = Vec::new();
    let mut pins: Vec<(NodeId, [f32; 3])> = Vec::new();
    let mut raw_links: BTreeMap<(NodeId, NodeId), Vec<(f32, f32)>> = BTreeMap::new();
    for s in replica.records() {
        let r = s.record();
        match &r.body {
            Body::Node(n) if r.origin != crate::mesh_view::HOST => {
                if !ids.contains(&r.origin) {
                    ids.push(r.origin);
                }
                if let Some(p) = n.pinned {
                    pins.push((
                        r.origin,
                        [p.x as f32 / 1000.0, p.y as f32 / 1000.0, p.z as f32 / 1000.0],
                    ));
                }
            }
            Body::Link(l) if l.quality >= min_quality => {
                for id in [r.origin, l.peer] {
                    if !ids.contains(&id) {
                        ids.push(id);
                    }
                }
                let key = if r.origin < l.peer {
                    (r.origin, l.peer)
                } else {
                    (l.peer, r.origin)
                };
                raw_links
                    .entry(key)
                    .or_default()
                    .push((l.distance_mm as f32 / 1000.0, l.sigma_mm as f32 / 1000.0));
            }
            _ => {}
        }
    }
    for &(id, xyz) in extra_pins {
        pins.retain(|(p, _)| *p != id);
        pins.push((id, xyz));
        if !ids.contains(&id) {
            ids.push(id);
        }
    }
    if ids.is_empty() {
        bail!("no nodes or links in the mesh state");
    }
    let index = |id: NodeId| ids.iter().position(|&x| x == id).expect("listed");
    let pin_list: Vec<Pin> = pins
        .iter()
        .map(|&(id, position)| Pin {
            node: index(id),
            position,
        })
        .collect();
    // Both directions of a link, averaged; the sigma the smaller of the two, floored by the solver.
    let links: Vec<LinkObs> = raw_links
        .iter()
        .map(|(&(a, b), obs)| LinkObs {
            a: index(a),
            b: index(b),
            distance_m: obs.iter().map(|o| o.0).sum::<f32>() / obs.len() as f32,
            sigma_m: obs.iter().map(|o| o.1).fold(f32::INFINITY, f32::min),
        })
        .collect();
    let solved = solve(
        ids.len(),
        &pin_list,
        &links,
        &MeshOptions {
            offset_m,
            height_sigma_m,
            ..MeshOptions::default()
        },
    );
    let mut placed = Vec::new();
    let mut unsolved = Vec::new();
    for (i, &id) in ids.iter().enumerate() {
        match solved.positions[i] {
            Some(position) => placed.push(Placed {
                id,
                position,
                sigma_m: solved.sigma_m[i].unwrap_or([f32::NAN; 3]),
                pinned: pin_list.iter().any(|p| p.node == i),
            }),
            None => unsolved.push(id),
        }
    }
    Ok(Solution {
        nodes: ids.len(),
        pins: pin_list.len(),
        links: links.len(),
        iterations: solved.iterations,
        normalized_rms: solved.normalized_rms,
        outliers: solved.outliers,
        placed,
        unsolved,
    })
}

/// The controller's Position records for a solution.
pub fn position_records(solution: &Solution, writer: &mut Writer) -> Vec<Record> {
    let mm = |v: f32| (v * 1000.0).round() as i32;
    let smm = |v: f32| if v.is_finite() { (v * 1000.0).round() as u32 } else { 0 };
    solution
        .placed
        .iter()
        .map(|p| {
            writer.record(Body::Position(Position {
                of: p.id,
                point: PointMm {
                    x: mm(p.position[0]),
                    y: mm(p.position[1]),
                    z: mm(p.position[2]),
                },
                sigma_mm: [smm(p.sigma_m[0]), smm(p.sigma_m[1]), smm(p.sigma_m[2])],
                frame: FRAME,
            }))
        })
        .collect()
}

/// A controller writer whose epoch is the clock, so every run writes newer versions than the
/// last.
pub fn controller_writer() -> Writer {
    let epoch = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(1, |d| d.as_secs() as u32);
    Writer::new(CONTROLLER, epoch)
}

/// Sends one record down to the sink as a downlink frame.
pub fn send_record(port: &mut dyn std::io::Write, record: &Record) -> Result<()> {
    let mut record_buf = [0u8; navgrid_proto::MESH_RECORD_LEN];
    let bytes = postcard::to_slice(record, &mut record_buf).context("encoding a record")?;
    let mut out = vec![0u8; MAX_DOWNLINK_FRAME];
    let frame = navgrid_proto::encode_downlink(&UpdateMsg::Record(bytes), &mut out);
    port.write_all(frame).context("writing to the sink")
}

pub fn run(args: &SolveArgs) -> Result<()> {
    let fleet = Fleet::open(&args.fleet).ok();
    let registry = fleet.as_ref().and_then(|f| f.registry().ok());
    let (replica, _, _) = replica_from_logs(&args.files)?;
    let name = |id: NodeId| node_name(id, registry.as_ref());
    let by_name: BTreeMap<String, NodeId> = replica
        .records()
        .iter()
        .map(|s| s.record().origin)
        .filter(|&o| o != CONTROLLER && o != crate::mesh_view::HOST)
        .map(|o| (name(o), o))
        .collect();
    let mut extra = Vec::new();
    for (pin_name, xyz) in &args.pins {
        let Some(&id) = by_name.get(pin_name) else {
            bail!("pin {pin_name}: no such node in the logs");
        };
        extra.push((id, *xyz));
    }
    let solution = solve_replica(&replica, &extra, args.offset_m, args.height_sigma_m, args.min_quality)?;
    println!(
        "{} nodes, {} pins, {} links; {} iterations, normalized rms {:.2}, {} links cut as bounces\n",
        solution.nodes, solution.pins, solution.links, solution.iterations, solution.normalized_rms, solution.outliers
    );
    println!(
        "{:<8} {:>8} {:>8} {:>8} {:>18} how",
        "node", "x m", "y m", "z m", "σ m (x y z)"
    );
    for p in &solution.placed {
        println!(
            "{:<8} {:>8.2} {:>8.2} {:>8.2} {:>6.2} {:>5.2} {:>5.2} {}",
            name(p.id),
            p.position[0],
            p.position[1],
            p.position[2],
            p.sigma_m[0],
            p.sigma_m[1],
            p.sigma_m[2],
            if p.pinned { "pinned" } else { "solved" }
        );
    }
    for id in &solution.unsolved {
        println!(
            "{:<8} {:>8} {:>8} {:>8} {:>18} unsolved (fewer than two placed neighbours)",
            name(*id),
            "-",
            "-",
            "-",
            ""
        );
    }
    let Some(port) = &args.sink else {
        return Ok(());
    };
    let mut writer = controller_writer();
    let positions = position_records(&solution, &mut writer);
    let mut port = serialport::new(port, args.baud)
        .timeout(Duration::from_millis(200))
        .open()
        .with_context(|| format!("opening {port}"))?;
    let mut sent = 0;
    for record in &positions {
        send_record(&mut *port, record)?;
        if !echoed(&mut *port, record)? {
            bail!(
                "the sink didn't mirror the position of {} back within {} s",
                name(record_of(record)),
                ECHO_WITHIN.as_secs()
            );
        }
        sent += 1;
    }
    println!("\n{sent} positions written to the mesh through the sink");
    Ok(())
}

fn record_of(r: &Record) -> NodeId {
    match &r.body {
        Body::Position(p) => p.of,
        _ => r.origin,
    }
}

/// Waits for the sink to mirror `record` back.
fn echoed(port: &mut dyn serialport::SerialPort, record: &Record) -> Result<bool> {
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
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}
