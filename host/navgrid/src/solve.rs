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
use navgrid_solver::mesh::{HeightObs, LinkObs, MeshOptions, Pin, PositionObs, solve_with_priors};

use crate::fleet::{Fleet, FleetArgs, Registry};
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
    /// Longer than the other two sides of a triangle allow: a suspected reflection, weighed
    /// lightly from the start.
    pub suspect: bool,
    /// Left out by the consensus: the solution most links agree with doesn't have it.
    pub dropped: bool,
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
    /// The frame is the approximate positions' (three or more of them, no pins).
    pub by_approx: bool,
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

/// Whether `id` is a tag: a tag moves, so its links don't survey the anchors. By its node record
/// in `replica`, else by the registry (the host may not have the record yet, e.g. right after
/// it started); a node neither knows counts as an anchor.
pub fn is_tag(replica: &Replica<Heap>, registry: Option<&Registry>, id: NodeId) -> bool {
    let recorded = replica.records().find_map(|s| {
        let r = s.record();
        match &r.body {
            Body::Node(n) if r.origin == id => Some(n.role != navgrid_mesh::Role::Anchor),
            _ => None,
        }
    });
    recorded.unwrap_or_else(|| {
        registry
            .and_then(|r| r.by_id(id))
            .is_some_and(|d| d.role != navgrid_proto::Role::Anchor)
    })
}

/// Whether `id` only routes: the registry has its ranging off, so its links neither survey nor
/// fix positions.
pub fn routes_only(registry: Option<&Registry>, id: NodeId) -> bool {
    registry.and_then(|r| r.by_id(id)).is_some_and(|d| !d.ranging)
}

/// Whether the survey leaves `id`'s links out: a tag, or a node that only routes.
pub fn left_out(replica: &Replica<Heap>, registry: Option<&Registry>, id: NodeId) -> bool {
    is_tag(replica, registry, id) || routes_only(registry, id)
}

/// The pairs between ranging anchors: what the survey solves the anchors from.
pub fn anchor_pairs(replica: &Replica<Heap>, registry: Option<&Registry>, pairs: Vec<PairStats>) -> Vec<PairStats> {
    pairs
        .into_iter()
        .filter(|p| !left_out(replica, registry, p.a) && !left_out(replica, registry, p.b))
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
    solve_pairs_except(
        replica,
        extra_pins,
        pairs,
        offset_m,
        height_sigma_m,
        &[],
        &[],
        Frame::default(),
    )
}

/// Heights from elsewhere than ranges, by node: `z` metres and 1-sigma, in the frame the solve
/// ends up in (with pins, theirs; in a local frame, relative to one node at 0).
pub type Heights = [(NodeId, f32, f32)];

/// Where nodes are, roughly (a click on the plan): x, y, z if known, and 1-sigma across.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Approx {
    pub node: NodeId,
    pub position: [f32; 3],
    pub has_z: bool,
    pub sigma_m: f32,
}

/// What fixes the frame, beyond the ranges' shape.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Frame<'a> {
    pub approx: &'a [Approx],
    /// A local frame mirrored (y to -y): ranges can't tell its handedness.
    pub mirror: bool,
}

/// Approximate positions needed to fix the frame: three not in a line.
pub const APPROX_FOR_FRAME: usize = 3;

/// [`solve_pairs`], leaving the nodes in `except` out altogether (nodes that only route), with
/// `heights` for some nodes (barometry) and what else fixes the frame.
#[allow(clippy::too_many_arguments)]
pub fn solve_pairs_except(
    replica: &Replica<Heap>,
    extra_pins: &[(NodeId, [f32; 3])],
    pairs: &[PairStats],
    offset_m: f32,
    height_sigma_m: f32,
    except: &[NodeId],
    heights: &Heights,
    frame: Frame,
) -> Result<Solution> {
    let mut ids: Vec<NodeId> = Vec::new();
    let mut pins: Vec<(NodeId, [f32; 3])> = Vec::new();
    for s in replica.records() {
        let r = s.record();
        if let Body::Node(n) = &r.body
            && r.origin != crate::mesh_view::HOST
            && !except.contains(&r.origin)
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
    let height = |id: NodeId| heights.iter().find(|h| h.0 == id).map(|h| h.1);
    let suspects = triangle_suspects(pairs, offset_m);
    let is_suspect = |p: &PairStats| suspects.contains(&(p.a, p.b));
    let clean: Vec<PairStats> = pairs.iter().filter(|p| !is_suspect(p)).copied().collect();
    // Three approximate positions or more fix the frame themselves; fewer can't, and are left
    // for the local frame's gauge.
    let approx: Vec<Approx> = frame
        .approx
        .iter()
        .filter(|a| !except.contains(&a.node) && !pins.iter().any(|(p, _)| *p == a.node))
        .copied()
        .collect();
    let by_approx = pins.len() < 2 && approx.len() >= APPROX_FOR_FRAME;
    for a in &approx {
        if !ids.contains(&a.node) {
            ids.push(a.node);
        }
    }
    let index = |id: NodeId| ids.iter().position(|&x| x == id).expect("listed");
    let links: Vec<LinkObs> = pairs
        .iter()
        .map(|p| LinkObs {
            a: index(p.a),
            b: index(p.b),
            distance_m: p.median_m,
            sigma_m: if is_suspect(p) {
                p.sigma_m * SUSPECT_WEIGHT
            } else {
                p.sigma_m
            },
        })
        .collect();
    let opts = MeshOptions {
        offset_m,
        height_sigma_m,
        ..MeshOptions::default()
    };
    let height_obs: Vec<HeightObs> = heights
        .iter()
        .filter_map(|&(id, z_m, sigma_m)| {
            ids.iter()
                .position(|&x| x == id)
                .map(|node| HeightObs { node, z_m, sigma_m })
        })
        .collect();
    let approx_obs: Vec<PositionObs> = if pins.len() >= 2 || by_approx {
        approx
            .iter()
            .map(|a| PositionObs {
                node: index(a.node),
                position: a.position,
                sigma_m: [a.sigma_m, a.sigma_m, if a.has_z { a.sigma_m } else { f32::INFINITY }],
            })
            .collect()
    } else {
        Vec::new()
    };
    // One solve from a set of pins and extra priors, with the consensus when it fits badly.
    let solve_from = |pins: &[(NodeId, [f32; 3])], extra: &[PositionObs]| {
        let pin_list: Vec<Pin> = pins
            .iter()
            .map(|&(id, position)| Pin {
                node: index(id),
                position,
            })
            .collect();
        let priors: Vec<PositionObs> = approx_obs.iter().chain(extra).copied().collect();
        let mut solved = solve_with_priors(ids.len(), &pin_list, &links, &height_obs, &priors, &opts);
        let mut dropped: Vec<usize> = Vec::new();
        // Whenever the solve set links aside (or fits badly anyway): an explanation that needs
        // two outliers should meet one that needs one. Small meshes only: it solves once per
        // link and per two.
        if (solved.normalized_rms > CONSENSUS_ABOVE_RMS || solved.outliers > 0)
            && links.len() <= CONSENSUS_MAX_LINKS
            && let Some((better, without)) =
                consensus(ids.len(), &pin_list, &links, &height_obs, &priors, &opts, &solved)
        {
            solved = better;
            dropped = without;
        }
        (pin_list, solved, dropped)
    };
    // A local frame hangs on a pair held exactly, so a bad pair there can't be set aside: each
    // of a few candidate pairs gets a solve, and the one most links agree with wins.
    let (local_frame, pin_list, mut solved, dropped) = if pins.len() < 2 && !by_approx {
        let mut best: Option<GaugeTry> = None;
        for (a, b, d) in local_gauges(if clean.len() >= 2 { &clean } else { pairs }, offset_m) {
            // At their barometric heights where known, the second along +x at the horizontal
            // part of their distance.
            let za = height(a).unwrap_or(0.0);
            let zb = height(b).unwrap_or(za);
            let x = (d * d - (zb - za) * (zb - za)).max(0.01).sqrt();
            // The origin is held; the second only on the +x axis, its distance free, so the
            // gauge pair is a link like any other and can be set aside.
            let on_axis = PositionObs {
                node: index(b),
                position: [x, 0.0, zb],
                sigma_m: [
                    f32::INFINITY,
                    0.001,
                    if height(b).is_some() { 0.1 } else { f32::INFINITY },
                ],
            };
            let (pin_list, solved, dropped) = solve_from(&[(a, [0.0, 0.0, za])], &[on_axis]);
            let score = agreement(&solved, &links, &opts);
            let better = |s: (usize, f32), than: (usize, f32)| s.0 > than.0 || (s.0 == than.0 && s.1 < than.1 * 0.8);
            if best.as_ref().is_none_or(|(s, ..)| better(score, *s)) {
                best = Some((score, (a, b), pin_list, solved, dropped));
            }
        }
        match best {
            Some((_, gauge, pin_list, solved, dropped)) => (Some(gauge), pin_list, solved, dropped),
            None => {
                let (pin_list, solved, dropped) = solve_from(&pins, &[]);
                (None, pin_list, solved, dropped)
            }
        }
    } else {
        let (pin_list, solved, dropped) = solve_from(&pins, &[]);
        (None, pin_list, solved, dropped)
    };
    if frame.mirror && local_frame.is_some() {
        for p in solved.positions.iter_mut().flatten() {
            p[1] = -p[1];
        }
    }
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
        .enumerate()
        .map(|(i, (p, l))| {
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
                suspect: is_suspect(p),
                dropped: dropped.contains(&i),
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
        by_approx,
    })
}

/// A tag's position from its latest bursts with the solved anchors.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TagFix {
    pub id: NodeId,
    pub position: [f32; 3],
    pub sigma_m: [f32; 3],
    pub normalized_rms: f32,
    /// Anchors it was fixed from, and the one left out as a reflection, if any.
    pub anchors: usize,
    pub left_out: Option<NodeId>,
    /// When its newest burst was, seconds since the epoch.
    pub at_s: u64,
}

/// Bursts a tag's fix is the median of: those within the first of these of its newest that
/// holds three anchors.
pub const TAG_WINDOWS_S: [u64; 3] = [20, 120, 600];

/// Each tag's position from its bursts with anchors the solution placed, the window ending at
/// its newest burst: the medians per anchor, multilaterated, the tag's barometric height in
/// `heights` if it has one (else a loose prior at the anchors' mean height). With four anchors
/// or more and a bad fit, each is left out in turn and the best fit kept.
pub fn fix_tags(
    history: &crate::survey::History,
    solution: &Solution,
    tags: &[NodeId],
    heights: &Heights,
    offset_m: f32,
) -> Vec<TagFix> {
    use navgrid_solver::{Height, Options, RangeObs, multilaterate};
    let anchor_at = |id: NodeId| solution.placed.iter().find(|p| p.id == id).map(|p| p.position);
    let mut out = Vec::new();
    for &tag in tags {
        let all = history.aggregate(0, 0);
        let newest = all
            .iter()
            .filter(|p| p.a == tag || p.b == tag)
            .map(|p| p.last_at_s)
            .max();
        let Some(newest) = newest else {
            continue;
        };
        // The shortest window back from the newest burst that holds three anchors: a tag that
        // ranges only now and then (or sits still) is still placed, from older bursts.
        let mut obs: Vec<(NodeId, RangeObs)> = Vec::new();
        for window in TAG_WINDOWS_S {
            let recent = history.aggregate(newest.saturating_sub(window), 100);
            obs = recent
                .iter()
                .filter(|p| p.a == tag || p.b == tag)
                .filter_map(|p| {
                    let other = if p.a == tag { p.b } else { p.a };
                    anchor_at(other).map(|anchor| {
                        (
                            other,
                            RangeObs {
                                anchor,
                                distance_m: (p.median_m - offset_m).max(0.1),
                                sigma_m: p.sigma_m.max(0.25),
                            },
                        )
                    })
                })
                .collect();
            if obs.len() >= 3 {
                break;
            }
        }
        if obs.len() < 3 {
            continue;
        }
        let mean_z = obs.iter().map(|o| o.1.anchor[2]).sum::<f32>() / obs.len() as f32;
        let height = match heights.iter().find(|h| h.0 == tag) {
            Some(&(_, z, sigma)) => Height::Observed {
                z,
                sigma_m: sigma.max(0.1),
            },
            None => Height::Observed {
                z: mean_z,
                sigma_m: 1.0,
            },
        };
        let opts = Options {
            height,
            ..Options::default()
        };
        let solve = |use_obs: &[(NodeId, RangeObs)]| {
            let n = use_obs.len() as f32;
            let c: [f32; 3] = core::array::from_fn(|i| use_obs.iter().map(|o| o.1.anchor[i]).sum::<f32>() / n);
            let o: Vec<RangeObs> = use_obs.iter().map(|o| o.1).collect();
            multilaterate(&o, [c[0] + 0.5, c[1] + 0.3, mean_z], &opts)
        };
        let Some(mut fix) = solve(&obs) else {
            continue;
        };
        let mut left_out = None;
        if fix.normalized_rms > 2.0 && obs.len() >= 4 {
            for i in 0..obs.len() {
                let fewer: Vec<(NodeId, RangeObs)> = obs
                    .iter()
                    .enumerate()
                    .filter(|(j, _)| *j != i)
                    .map(|(_, o)| *o)
                    .collect();
                if let Some(f) = solve(&fewer)
                    && f.normalized_rms < fix.normalized_rms * 0.5
                {
                    fix = f;
                    left_out = Some(obs[i].0);
                }
            }
        }
        out.push(TagFix {
            id: tag,
            position: fix.position,
            sigma_m: fix.sigma_m,
            normalized_rms: fix.normalized_rms,
            anchors: obs.len() - usize::from(left_out.is_some()),
            left_out,
            at_s: newest,
        });
    }
    out
}

/// A suspected reflection's sigma, times this: it still counts, a little.
const SUSPECT_WEIGHT: f32 = 8.0;
/// Above this normalized rms the consensus looks for links to leave out.
const CONSENSUS_ABOVE_RMS: f32 = 2.0;
/// The consensus solves once per link and per two: up to this many links.
const CONSENSUS_MAX_LINKS: usize = 40;

/// Pairs longer than the other two sides of a triangle allow, beyond their sigmas and a margin:
/// a reflection only reads long, so in a triangle that can't close the long side is the suspect.
/// Keys with the lower ID first, as [`PairStats`] has them.
pub fn triangle_suspects(pairs: &[PairStats], offset_m: f32) -> Vec<(NodeId, NodeId)> {
    let key = |a: NodeId, b: NodeId| if a < b { (a, b) } else { (b, a) };
    let by: BTreeMap<(NodeId, NodeId), (f32, f32)> = pairs
        .iter()
        .map(|p| (key(p.a, p.b), (p.median_m - offset_m, p.sigma_m)))
        .collect();
    let nodes: Vec<NodeId> = {
        let mut v: Vec<NodeId> = pairs.iter().flat_map(|p| [p.a, p.b]).collect();
        v.sort();
        v.dedup();
        v
    };
    let mut out = Vec::new();
    for (i, &a) in nodes.iter().enumerate() {
        for (j, &b) in nodes.iter().enumerate().skip(i + 1) {
            for &c in nodes.iter().skip(j + 1) {
                let (Some(&ab), Some(&ac), Some(&bc)) = (by.get(&key(a, b)), by.get(&key(a, c)), by.get(&key(b, c)))
                else {
                    continue;
                };
                let margin = 3.0 * (ab.1 * ab.1 + ac.1 * ac.1 + bc.1 * bc.1).sqrt() + 0.2;
                for (long, s1, s2, k) in [
                    (ab, ac, bc, key(a, b)),
                    (ac, ab, bc, key(a, c)),
                    (bc, ab, ac, key(b, c)),
                ] {
                    if long.0 > s1.0 + s2.0 + margin && !out.contains(&k) {
                        out.push(k);
                    }
                }
            }
        }
    }
    out
}

/// Robust cost of a solution over every link: how many fit within 3σ (σ at least 25 cm), and the
/// rms of those that do.
fn agreement(solved: &navgrid_solver::mesh::Solved, links: &[LinkObs], opts: &MeshOptions) -> (usize, f32) {
    let mut inliers = 0;
    let mut sum = 0.0f32;
    for l in links {
        let (Some(pa), Some(pb)) = (solved.positions[l.a], solved.positions[l.b]) else {
            continue;
        };
        let d = ((pa[0] - pb[0]).powi(2) + (pa[1] - pb[1]).powi(2) + (pa[2] - pb[2]).powi(2)).sqrt();
        let sigma = l.sigma_m.max(0.25);
        let r = (l.distance_m - opts.offset_m - d) / sigma;
        if r.abs() <= 3.0 {
            inliers += 1;
            sum += r * r;
        }
    }
    (
        inliers,
        if inliers > 0 {
            (sum / inliers as f32).sqrt()
        } else {
            f32::INFINITY
        },
    )
}

/// The consensus: the solution with each link, then each two, left out; the one most links agree
/// with, if it beats `base`. A node keeps at least two links, or that subset isn't tried.
#[allow(clippy::too_many_arguments)]
fn consensus(
    nodes: usize,
    pins: &[Pin],
    links: &[LinkObs],
    heights: &[HeightObs],
    approx: &[PositionObs],
    opts: &MeshOptions,
    base: &navgrid_solver::mesh::Solved,
) -> Option<(navgrid_solver::mesh::Solved, Vec<usize>)> {
    let pinned = |n: usize| pins.iter().any(|p| p.node == n);
    let keeps_two = |without: &[usize]| {
        (0..nodes).filter(|&n| !pinned(n)).all(|n| {
            let total = links.iter().filter(|l| l.a == n || l.b == n).count();
            let kept = links
                .iter()
                .enumerate()
                .filter(|(i, l)| !without.contains(i) && (l.a == n || l.b == n))
                .count();
            total < 2 || kept >= 2
        })
    };
    let mut subsets: Vec<Vec<usize>> = (0..links.len()).map(|i| vec![i]).collect();
    for i in 0..links.len() {
        for j in i + 1..links.len() {
            subsets.push(vec![i, j]);
        }
    }
    let mut best: Option<((usize, f32), navgrid_solver::mesh::Solved, Vec<usize>)> = None;
    let base_score = agreement(base, links, opts);
    for without in subsets {
        if !keeps_two(&without) {
            continue;
        }
        let kept: Vec<LinkObs> = links
            .iter()
            .enumerate()
            .filter(|(i, _)| !without.contains(i))
            .map(|(_, l)| *l)
            .collect();
        let solved = solve_with_priors(nodes, pins, &kept, heights, approx, opts);
        let score = agreement(&solved, links, opts);
        let better = |a: (usize, f32), b: (usize, f32)| a.0 > b.0 || (a.0 == b.0 && a.1 < b.1 * 0.8);
        if best.as_ref().is_none_or(|(s, _, _)| better(score, *s)) {
            best = Some((score, solved, without));
        }
    }
    let (score, solved, without) = best?;
    (score.0 > base_score.0 || (score.0 == base_score.0 && score.1 < base_score.1 * 0.8)).then_some((solved, without))
}

/// The pairs a local frame may hang on, best first: the most bursts, then the smallest sigma,
/// then the lowest IDs (so the same data picks the same frame); at most a handful. Each is the
/// origin, the node on +x, and their distance less the offset.
fn local_gauges(pairs: &[PairStats], offset_m: f32) -> Vec<(NodeId, NodeId, f32)> {
    let mut sorted: Vec<&PairStats> = pairs.iter().collect();
    sorted.sort_by(|x, y| {
        y.n.min(20)
            .cmp(&x.n.min(20))
            .then(x.sigma_m.total_cmp(&y.sigma_m))
            .then((x.a, x.b).cmp(&(y.a, y.b)))
    });
    sorted
        .into_iter()
        .take(GAUGE_CANDIDATES)
        .map(|p| (p.a, p.b, (p.median_m - offset_m).max(0.1)))
        .collect()
}

/// A local frame's try: its agreement, its gauge pair, its pins, the solve and the links it
/// left out.
type GaugeTry = (
    (usize, f32),
    (NodeId, NodeId),
    Vec<Pin>,
    navgrid_solver::mesh::Solved,
    Vec<usize>,
);

/// Gauge pairs tried for a local frame.
const GAUGE_CANDIDATES: usize = 6;

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
            let pairs = anchor_pairs(&replica, registry.as_ref(), history.aggregate(since, args.min_quality));
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
    fn two_pairs_reading_long_through_bounces_dont_bend_the_layout() {
        // Six anchors on two walls, every pair measured; two pairs lock onto bounces.
        let at = [
            [0.0f32, 0.0],
            [5.0, 0.0],
            [10.0, 0.0],
            [0.0, 6.0],
            [5.0, 6.5],
            [10.0, 6.0],
        ];
        let mut pairs = Vec::new();
        for i in 0..at.len() {
            for j in i + 1..at.len() {
                let d = ((at[i][0] - at[j][0]).powi(2) + (at[i][1] - at[j][1]).powi(2)).sqrt();
                let bounce = match (i, j) {
                    (0, 1) => 4.0,
                    (2, 4) => 3.0,
                    _ => 0.0,
                };
                pairs.push(pair(i as u64 + 1, j as u64 + 1, d + bounce));
            }
        }
        let replica = crate::mesh_view::host_replica();
        let pins = [(NodeId(1), [0.0, 0.0, 0.0]), (NodeId(3), [10.0, 0.0, 0.0])];
        let s = solve_pairs(&replica, &pins, &pairs, 1.5, 0.3).unwrap();
        for (i, t) in at.iter().enumerate() {
            let p = s
                .placed
                .iter()
                .find(|p| p.id == NodeId(i as u64 + 1))
                .expect("placed")
                .position;
            let e = ((p[0] - t[0]).powi(2) + (p[1] - t[1]).powi(2)).sqrt();
            assert!(e < 0.3, "N{}: {p:?} against {t:?}", i + 1);
        }
        let fit = |a, b| s.fit(NodeId(a), NodeId(b)).unwrap();
        assert!(fit(1, 2).residual_m.unwrap() > 3.0, "the bounce shows as reading long");
        assert!(fit(3, 5).residual_m.unwrap() > 2.0, "the bounce shows as reading long");
    }

    #[test]
    fn a_bad_pair_with_the_most_bursts_doesnt_become_the_frame() {
        // Five anchors, every pair measured; the pair with the most bursts reads 2.5 m short.
        let at = [[0.0f32, 0.0], [6.0, 0.0], [6.0, 5.0], [0.0, 5.0], [3.0, 8.0]];
        let mut pairs = Vec::new();
        for i in 0..at.len() {
            for j in i + 1..at.len() {
                let d = ((at[i][0] - at[j][0]).powi(2) + (at[i][1] - at[j][1]).powi(2)).sqrt();
                let mut p = pair(i as u64 + 1, j as u64 + 1, d);
                if (i, j) == (0, 2) {
                    p.median_m -= 2.5;
                    p.n = 40;
                }
                pairs.push(p);
            }
        }
        let replica = crate::mesh_view::host_replica();
        let s = solve_pairs(&replica, &[], &pairs, 1.5, 0.3).unwrap();
        assert!(s.local_frame.is_some());
        let pos = |id: u64| s.placed.iter().find(|p| p.id == NodeId(id)).unwrap().position;
        for i in 0..at.len() {
            for j in i + 1..at.len() {
                if (i, j) == (0, 2) {
                    continue;
                }
                let (a, b) = (pos(i as u64 + 1), pos(j as u64 + 1));
                let solved = ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
                let truth = ((at[i][0] - at[j][0]).powi(2) + (at[i][1] - at[j][1]).powi(2)).sqrt();
                assert!(
                    (solved - truth).abs() < 0.2,
                    "{}–{}: {solved} against {truth}",
                    i + 1,
                    j + 1
                );
            }
        }
    }

    #[test]
    fn a_tag_is_fixed_from_its_latest_bursts_and_a_reflected_one_left_out() {
        use crate::survey::{History, Observation};
        let at = [[0.0f32, 0.0, 2.5], [8.0, 0.0, 2.5], [8.0, 6.0, 2.5], [0.0, 6.0, 2.5]];
        let mut pairs = Vec::new();
        for i in 0..at.len() {
            for j in i + 1..at.len() {
                let d = ((at[i][0] - at[j][0]).powi(2) + (at[i][1] - at[j][1]).powi(2)).sqrt();
                pairs.push(pair(i as u64 + 1, j as u64 + 1, d));
            }
        }
        let replica = crate::mesh_view::host_replica();
        let pins: Vec<(NodeId, [f32; 3])> = (0..3).map(|i| (NodeId(i as u64 + 1), at[i])).collect();
        let solution = solve_pairs(&replica, &pins, &pairs, 1.5, 0.3).unwrap();
        // The tag at (3, 2, 1.2); its burst to anchor 3 reads 3 m long; an old burst elsewhere.
        let tag = NodeId(100);
        let truth = [3.0f32, 2.0, 1.2];
        let mut history = History::default();
        let mut counter = 0;
        let mut burst = |history: &mut History, anchor: usize, at_s: u64, extra: f32| {
            counter += 1;
            let d = ((truth[0] - at[anchor][0]).powi(2)
                + (truth[1] - at[anchor][1]).powi(2)
                + (truth[2] - at[anchor][2]).powi(2))
            .sqrt();
            history.observe(Observation {
                at_s,
                from: tag,
                to: NodeId(anchor as u64 + 1),
                version: navgrid_mesh::Version { epoch: 1, counter },
                distance_mm: ((d + 1.5 + extra) * 1000.0) as u32,
                sigma_mm: 80,
                quality: 240,
                paths: 4,
                procedures: 4,
                failures: 0,
            });
        };
        burst(&mut history, 0, 100, 5.0); // long ago, from somewhere else
        for t in 1000..1005 {
            burst(&mut history, 0, t, 0.0);
            burst(&mut history, 1, t, 0.0);
            burst(&mut history, 2, t, 3.0);
            burst(&mut history, 3, t, 0.0);
        }
        let fixes = fix_tags(&history, &solution, &[tag], &[(tag, 1.2, 0.15)], 1.5);
        assert_eq!(fixes.len(), 1);
        let f = fixes[0];
        let e = ((f.position[0] - truth[0]).powi(2) + (f.position[1] - truth[1]).powi(2)).sqrt();
        assert!(e < 0.3, "{:?} against {truth:?}", f.position);
        assert_eq!(f.left_out, Some(NodeId(3)));
        assert_eq!(f.at_s, 1004);
    }

    #[test]
    fn a_side_longer_than_the_other_two_is_the_suspect() {
        // A, B and C nearly in line: AB reads 2 m long through a bounce.
        let pairs = [pair(1, 2, 5.0 + 2.0), pair(1, 3, 2.69), pair(2, 3, 2.69)];
        assert_eq!(triangle_suspects(&pairs, 1.5), vec![(NodeId(1), NodeId(2))]);
        let fine = [pair(1, 2, 5.0), pair(1, 3, 2.69), pair(2, 3, 2.69)];
        assert!(triangle_suspects(&fine, 1.5).is_empty());
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
