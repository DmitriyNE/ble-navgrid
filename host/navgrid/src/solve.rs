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
use crate::survey::{History, Observation, PairStats, now_s};

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
    /// The survey history `navgrid serve` keeps (`survey.jsonl` in the fleet directory): every
    /// pair's median over its bursts, instead of the latest burst in the logs.
    #[arg(long)]
    survey: Option<PathBuf>,
    /// With `--survey`, only the bursts of the last this many seconds; 0 = all of them.
    #[arg(long, default_value_t = 0)]
    window_s: u64,
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

/// How one pair's measured distance fits the solution.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinkFit {
    pub a: NodeId,
    pub b: NodeId,
    /// The pair's median over its bursts, and the sigma the solver believed, before the offset.
    pub measured_m: f32,
    pub sigma_m: f32,
    pub n: usize,
    /// Between the solved positions; none while an end is unsolved.
    pub solved_m: Option<f32>,
    /// Measured (less the offset) minus solved: a bounce reads long.
    pub residual_m: Option<f32>,
    /// Beyond the solver's robust cut, so it hardly counted.
    pub cut: bool,
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
    pub fits: Vec<LinkFit>,
    /// With fewer than two pins, the solution is in a local frame: the first node at the origin,
    /// the second on +x at their measured distance, heights around 0. The mesh's shape, not
    /// where it is; such positions are not for writing to the mesh.
    pub local_frame: Option<(NodeId, NodeId)>,
}

impl Solution {
    pub fn fit(&self, a: NodeId, b: NodeId) -> Option<&LinkFit> {
        self.fits
            .iter()
            .find(|f| (f.a == a && f.b == b) || (f.a == b && f.b == a))
    }
}

/// Solves the mesh in `replica` from its latest Link records, with the pins in its node records
/// and `extra_pins` (which override them).
pub fn solve_replica(
    replica: &Replica<Heap>,
    extra_pins: &[(NodeId, [f32; 3])],
    offset_m: f32,
    height_sigma_m: f32,
    min_quality: u8,
) -> Result<Solution> {
    let pairs = latest_pairs(replica, min_quality);
    solve_pairs(replica, extra_pins, &pairs, offset_m, height_sigma_m)
}

/// Whether `id` is a tag by its node record in `replica`: a tag moves, so its links don't
/// survey the anchors. A node without a record counts as an anchor.
pub fn is_tag(replica: &Replica<Heap>, id: NodeId) -> bool {
    replica.records().any(|s| {
        let r = s.record();
        r.origin == id && matches!(&r.body, Body::Node(n) if n.role != navgrid_mesh::Role::Anchor)
    })
}

/// The pairs between anchors: what the survey solves the anchors from.
pub fn anchor_pairs(replica: &Replica<Heap>, pairs: Vec<PairStats>) -> Vec<PairStats> {
    pairs
        .into_iter()
        .filter(|p| !is_tag(replica, p.a) && !is_tag(replica, p.b))
        .collect()
}

/// Every pair's latest burst in `replica` (both directions, if it holds both), as the survey
/// would see them.
pub fn latest_pairs(replica: &Replica<Heap>, min_quality: u8) -> Vec<PairStats> {
    let mut history = History::default();
    for s in replica.records() {
        if let Some(obs) = Observation::from_record(s.record(), 0) {
            history.observe(obs);
        }
    }
    history.aggregate(0, min_quality)
}

/// Solves the mesh from `pairs` (the survey's aggregates), with the pins in `replica`'s node
/// records and `extra_pins` (which override them); every node with a record or a pair is placed
/// or listed unsolved.
pub fn solve_pairs(
    replica: &Replica<Heap>,
    extra_pins: &[(NodeId, [f32; 3])],
    pairs: &[PairStats],
    offset_m: f32,
    height_sigma_m: f32,
) -> Result<Solution> {
    let mut ids: Vec<NodeId> = Vec::new();
    let mut pins: Vec<(NodeId, [f32; 3])> = Vec::new();
    for s in replica.records() {
        let r = s.record();
        if let Body::Node(n) = &r.body
            && r.origin != crate::mesh_view::HOST
        {
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
    }
    for p in pairs {
        for id in [p.a, p.b] {
            if !ids.contains(&id) {
                ids.push(id);
            }
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
    let local_frame = if pins.len() < 2 {
        let gauge = local_gauge(pairs, offset_m);
        if let Some((a, b, d)) = gauge {
            pins = vec![(a, [0.0, 0.0, 0.0]), (b, [d, 0.0, 0.0])];
        }
        gauge.map(|(a, b, _)| (a, b))
    } else {
        None
    };
    let index = |id: NodeId| ids.iter().position(|&x| x == id).expect("listed");
    let pin_list: Vec<Pin> = pins
        .iter()
        .map(|&(id, position)| Pin {
            node: index(id),
            position,
        })
        .collect();
    let links: Vec<LinkObs> = pairs
        .iter()
        .map(|p| LinkObs {
            a: index(p.a),
            b: index(p.b),
            distance_m: p.median_m,
            sigma_m: p.sigma_m,
        })
        .collect();
    let opts = MeshOptions {
        offset_m,
        height_sigma_m,
        ..MeshOptions::default()
    };
    let solved = solve(ids.len(), &pin_list, &links, &opts);
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
    let fits = pairs
        .iter()
        .zip(&links)
        .map(|(p, l)| {
            let solved_m = match (solved.positions[l.a], solved.positions[l.b]) {
                (Some(pa), Some(pb)) => {
                    Some(((pa[0] - pb[0]).powi(2) + (pa[1] - pb[1]).powi(2) + (pa[2] - pb[2]).powi(2)).sqrt())
                }
                _ => None,
            };
            let residual_m = solved_m.map(|s| p.median_m - offset_m - s);
            let believed = p.sigma_m.max(opts.sigma_floor_m);
            LinkFit {
                a: p.a,
                b: p.b,
                measured_m: p.median_m,
                sigma_m: p.sigma_m,
                n: p.n,
                solved_m,
                residual_m,
                cut: residual_m.is_some_and(|r| r.abs() > opts.huber_k * believed),
            }
        })
        .collect();
    Ok(Solution {
        nodes: ids.len(),
        pins: pin_list.len(),
        links: links.len(),
        iterations: solved.iterations,
        normalized_rms: solved.normalized_rms,
        outliers: solved.outliers,
        placed,
        unsolved,
        fits,
        local_frame,
    })
}

/// The two nodes a local frame hangs on, and the distance between them less the offset: the node
/// in the most pairs, and of its pairs the one with the most bursts and then the smallest sigma.
fn local_gauge(pairs: &[PairStats], offset_m: f32) -> Option<(NodeId, NodeId, f32)> {
    let mut degree: BTreeMap<NodeId, usize> = BTreeMap::new();
    for p in pairs {
        *degree.entry(p.a).or_default() += 1;
        *degree.entry(p.b).or_default() += 1;
    }
    let (&origin, _) = degree.iter().max_by_key(|&(id, n)| (*n, core::cmp::Reverse(*id)))?;
    let best = pairs
        .iter()
        .filter(|p| p.a == origin || p.b == origin)
        .max_by(|x, y| x.n.min(10).cmp(&y.n.min(10)).then(y.sigma_m.total_cmp(&x.sigma_m)))?;
    let other = if best.a == origin { best.b } else { best.a };
    Some((origin, other, (best.median_m - offset_m).max(0.1)))
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
    let solution = match &args.survey {
        Some(path) => {
            let history = History::load(path)?;
            let since = if args.window_s == 0 {
                0
            } else {
                now_s().saturating_sub(args.window_s)
            };
            let pairs = anchor_pairs(&replica, history.aggregate(since, args.min_quality));
            println!(
                "survey {}: {} bursts over {} pairs, {} pairs in the window\n",
                path.display(),
                history.len(),
                history.pairs(),
                pairs.len()
            );
            solve_pairs(&replica, &extra, &pairs, args.offset_m, args.height_sigma_m)?
        }
        None => solve_replica(&replica, &extra, args.offset_m, args.height_sigma_m, args.min_quality)?,
    };
    println!(
        "{} nodes, {} pins, {} links; {} iterations, normalized rms {:.2}, {} links cut as bounces\n",
        solution.nodes, solution.pins, solution.links, solution.iterations, solution.normalized_rms, solution.outliers
    );
    if let Some((a, b)) = solution.local_frame {
        println!(
            "fewer than two pins: a local frame, {} at the origin and {} on +x; the shape only\n",
            name(a),
            name(b)
        );
    }
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
    println!(
        "\n{:<16} {:>6} {:>9} {:>6} {:>9} {:>9}",
        "pair", "bursts", "median m", "σ m", "solved m", "residual"
    );
    for f in &solution.fits {
        println!(
            "{:<16} {:>6} {:>9.2} {:>6.2} {:>9} {:>9}{}",
            format!("{}–{}", name(f.a), name(f.b)),
            f.n,
            f.measured_m,
            f.sigma_m,
            f.solved_m.map_or("-".to_string(), |s| format!("{s:.2}")),
            f.residual_m.map_or("-".to_string(), |r| format!("{r:+.2}")),
            if f.cut { " cut" } else { "" }
        );
    }
    let Some(port) = &args.sink else {
        return Ok(());
    };
    if solution.local_frame.is_some() {
        bail!("a local frame is not written to the mesh: pin at least two nodes");
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    fn pair(a: u64, b: u64, distance_m: f32) -> PairStats {
        PairStats {
            a: NodeId(a),
            b: NodeId(b),
            n: 20,
            from_a: 10,
            from_b: 10,
            median_m: distance_m + 1.5,
            spread_m: 0.3,
            sigma_m: 0.07,
            last_m: distance_m + 1.5,
            last_at_s: 0,
            quality: 250,
            paths: 4,
        }
    }

    #[test]
    fn without_pins_the_shape_comes_out_in_a_local_frame() {
        let at = [[0.0f32, 0.0], [5.0, 0.0], [0.0, 4.0], [5.0, 4.0]];
        let mut pairs = Vec::new();
        for i in 0..4 {
            for j in i + 1..4 {
                let d = ((at[i][0] - at[j][0]).powi(2) + (at[i][1] - at[j][1]).powi(2)).sqrt();
                pairs.push(pair(i as u64 + 1, j as u64 + 1, d));
            }
        }
        let replica = crate::mesh_view::host_replica();
        let s = solve_pairs(&replica, &[], &pairs, 1.5, 0.3).unwrap();
        let (origin, x_axis) = s.local_frame.expect("a local frame");
        assert_ne!(origin, x_axis);
        assert_eq!(s.placed.len(), 4, "unsolved: {:?}", s.unsolved);
        let pos = |id: u64| s.placed.iter().find(|p| p.id == NodeId(id)).unwrap().position;
        for i in 0..4 {
            for j in i + 1..4 {
                let (a, b) = (pos(i as u64 + 1), pos(j as u64 + 1));
                let solved = ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
                let truth = ((at[i][0] - at[j][0]).powi(2) + (at[i][1] - at[j][1]).powi(2)).sqrt();
                assert!(
                    (solved - truth).abs() < 0.05,
                    "{}–{}: {solved} against {truth}",
                    i + 1,
                    j + 1
                );
            }
        }
        // Two pins: their frame, no local one.
        let pinned = solve_pairs(
            &replica,
            &[(NodeId(1), [0.0, 0.0, 0.0]), (NodeId(2), [5.0, 0.0, 0.0])],
            &pairs,
            1.5,
            0.3,
        )
        .unwrap();
        assert!(pinned.local_frame.is_none());
    }
}
