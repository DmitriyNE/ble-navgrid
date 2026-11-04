//! `navgrid mesh-sim`: the mesh state protocol (`navgrid-mesh`, architecture §12) over a synthetic
//! deployment, to size it before firmware: how far behind replicas fall, how long commands take,
//! and what contacts cost in bytes and nodes in memory.
//!
//! Every mesh link holds a sync session every `--contact` seconds. Nodes write health every
//! 30 min and environment every 10 min. A link's initiator (the lower node index) writes its link
//! record after each ranging burst, every `--ranging` minutes. The controller writes the frame at
//! the start, every node's position at 30 min and then hourly, and blinks a random node every
//! `--command-every` minutes, retrying unacknowledged commands every 15 min.

use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap, HashMap, HashSet};
use std::mem::size_of;

use anyhow::{Result, bail};
use navgrid_mesh::session::{self, To, Traffic};
use navgrid_mesh::{
    Body, Command, Config, Desired, Environment, Frame, Health, Heap, Key, Link, NodeId, NodeInfo, Pending, PointMm,
    Position, Record, Replica, Reported, Role, Route, Status, Stored, Topic, Version, Writer,
};
use navgrid_ranging::sim::SplitMix64;

use crate::scenario::quantile;

#[derive(clap::Args)]
pub struct MeshSimArgs {
    /// Mesh nodes, sinks included, on a jittered square grid.
    #[arg(long, default_value_t = 20)]
    nodes: usize,
    /// Sinks among them, spread along the middle row.
    #[arg(long, default_value_t = 1)]
    sinks: usize,
    /// Grid spacing, metres.
    #[arg(long, default_value_t = 6.0)]
    spacing: f64,
    /// Radio range of a mesh link, metres.
    #[arg(long, default_value_t = 13.0)]
    range: f64,
    /// Mesh links per node at most.
    #[arg(long, default_value_t = 6)]
    degree: usize,
    /// Seconds between sync sessions on each mesh link.
    #[arg(long, default_value_t = 60)]
    contact: u64,
    /// Minutes between ranging bursts on each mesh link.
    #[arg(long, default_value_t = 15.0)]
    ranging: f64,
    /// Bytes each side may send per session in records and pushes.
    #[arg(long, default_value_t = 2048)]
    budget: u32,
    /// Simulated hours. The first is warm-up, left out of the staleness figures.
    #[arg(long, default_value_t = 6.0)]
    hours: f64,
    /// Minutes between controller commands.
    #[arg(long, default_value_t = 5.0)]
    command_every: f64,
    /// Cut every link between the left and right halves from hour A to hour B, as `A-B`.
    #[arg(long, value_parser = parse_window)]
    partition: Option<(f64, f64)>,
    /// Link outages per link per hour, each lasting `--outage-min`.
    #[arg(long, default_value_t = 0.0)]
    outages_per_link_h: f64,
    #[arg(long, default_value_t = 5.0)]
    outage_min: f64,
    /// Node reboots per node per day: a fresh replica with a new epoch, nothing remembered.
    #[arg(long, default_value_t = 0.0)]
    reboots_per_node_day: f64,
    /// Share of sessions that break off before completing, so neither side remembers it.
    #[arg(long, default_value_t = 0.0)]
    session_drops: f64,
    /// Table limits for nodes other than sinks: records, outbox, routes.
    #[arg(long, value_delimiter = ',', default_value = "4096,4096,4096")]
    limits: Vec<usize>,
    #[arg(long, default_value_t = 1)]
    seed: u64,
}

fn parse_window(s: &str) -> Result<(f64, f64), String> {
    let (a, b) = s.split_once('-').ok_or("expected A-B (hours)")?;
    let a: f64 = a.parse().map_err(|_| format!("bad hour {a:?}"))?;
    let b: f64 = b.parse().map_err(|_| format!("bad hour {b:?}"))?;
    if b <= a {
        return Err("the partition must end after it starts".into());
    }
    Ok((a, b))
}

const SECOND: u64 = 1000;
const MINUTE: u64 = 60 * SECOND;
const HOUR: u64 = 60 * MINUTE;
const HEALTH_EVERY: u64 = 30 * MINUTE;
const ENVIRONMENT_EVERY: u64 = 10 * MINUTE;
const TICK_EVERY: u64 = MINUTE;
const SAMPLE_EVERY: u64 = 5 * MINUTE;
const RETRY_AFTER: u64 = 15 * MINUTE;
const WARM_UP: u64 = HOUR;
const CONTROLLER: NodeId = navgrid_mesh::CONTROLLER;

/// Node positions, mesh links and sinks.
struct Layout {
    pos: Vec<[f64; 2]>,
    links: Vec<(usize, usize)>,
    sinks: Vec<usize>,
    width: f64,
}

impl Layout {
    fn new(args: &MeshSimArgs, rng: &mut SplitMix64) -> Self {
        let cols = (args.nodes as f64).sqrt().ceil() as usize;
        let jitter = |rng: &mut SplitMix64| (rng.uniform() - 0.5) * 0.6 * args.spacing;
        let pos: Vec<[f64; 2]> = (0..args.nodes)
            .map(|i| {
                let (c, r) = ((i % cols) as f64, (i / cols) as f64);
                [c * args.spacing + jitter(rng), r * args.spacing + jitter(rng)]
            })
            .collect();
        let dist = |a: usize, b: usize| (pos[a][0] - pos[b][0]).hypot(pos[a][1] - pos[b][1]);
        // Nearest pairs first, while both ends have link budget left.
        let mut pairs: Vec<(usize, usize)> = (0..args.nodes)
            .flat_map(|a| (a + 1..args.nodes).map(move |b| (a, b)))
            .filter(|&(a, b)| dist(a, b) <= args.range)
            .collect();
        pairs.sort_by(|p, q| dist(p.0, p.1).total_cmp(&dist(q.0, q.1)));
        let mut degree = vec![0; args.nodes];
        let mut links = Vec::new();
        for (a, b) in pairs {
            if degree[a] < args.degree && degree[b] < args.degree {
                degree[a] += 1;
                degree[b] += 1;
                links.push((a, b));
            }
        }
        let width = pos.iter().map(|p| p[0]).fold(0.0, f64::max);
        let height = pos.iter().map(|p| p[1]).fold(0.0, f64::max);
        let mut sinks: Vec<usize> = Vec::new();
        for k in 0..args.sinks {
            let at = [width * (k as f64 + 0.5) / args.sinks as f64, height / 2.0];
            let nearest = (0..args.nodes)
                .filter(|i| !sinks.contains(i))
                .min_by(|&a, &b| {
                    let d = |i: usize| (pos[i][0] - at[0]).hypot(pos[i][1] - at[1]);
                    d(a).total_cmp(&d(b))
                })
                .expect("more nodes than sinks");
            sinks.push(nearest);
        }
        Self {
            pos,
            links,
            sinks,
            width,
        }
    }

    fn neighbours(&self) -> Vec<Vec<usize>> {
        let mut adj = vec![Vec::new(); self.pos.len()];
        for &(a, b) in &self.links {
            adj[a].push(b);
            adj[b].push(a);
        }
        adj
    }

    /// Hops from every node to its nearest sink, `None` where there is no path.
    fn hops(&self) -> Vec<Option<usize>> {
        let adj = self.neighbours();
        let mut hops = vec![None; self.pos.len()];
        let mut frontier = self.sinks.clone();
        for &s in &self.sinks {
            hops[s] = Some(0);
        }
        let mut depth = 0;
        while !frontier.is_empty() {
            depth += 1;
            let mut next = Vec::new();
            for &n in &frontier {
                for &m in &adj[n] {
                    if hops[m].is_none() {
                        hops[m] = Some(depth);
                        next.push(m);
                    }
                }
            }
            frontier = next;
        }
        hops
    }

    /// Every node's 2-hop neighbourhood, itself excluded.
    fn neighbourhoods(&self) -> Vec<Vec<usize>> {
        let adj = self.neighbours();
        (0..self.pos.len())
            .map(|n| {
                let mut near: Vec<usize> = adj[n]
                    .iter()
                    .flat_map(|&m| std::iter::once(m).chain(adj[m].iter().copied()))
                    .collect();
                near.sort_unstable();
                near.dedup();
                near.retain(|&m| m != n);
                near
            })
            .collect()
    }

    fn distance_mm(&self, a: usize, b: usize) -> u32 {
        ((self.pos[a][0] - self.pos[b][0]).hypot(self.pos[a][1] - self.pos[b][1]) * 1000.0) as u32
    }

    fn left(&self, n: usize) -> bool {
        self.pos[n][0] < self.width / 2.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Event {
    Contact(usize),
    /// A link goes down for a while.
    Outage(usize),
    Reboot(usize),
    Health(usize),
    Environment(usize),
    Command,
    Positions,
    Retry,
    Tick,
    Sample,
}

/// A controller command, from issue to acknowledgement.
struct Issued {
    target: usize,
    record: Record,
    issued_ms: u64,
    applied_ms: Option<u64>,
    acked_ms: Option<u64>,
    superseded: bool,
}

struct Sim<'a> {
    args: &'a MeshSimArgs,
    layout: Layout,
    ids: Vec<NodeId>,
    index: HashMap<NodeId, usize>,
    nodes: Vec<Replica<Heap>>,
    controller: Writer,
    rng: SplitMix64,
    queue: BinaryHeap<Reverse<(u64, u64, Event)>>,
    seq: u64,
    end_ms: u64,
    next_range: Vec<u64>,
    /// Every write: key → (version, time), oldest first.
    history: HashMap<Key, Vec<(Version, u64)>>,
    /// The keys each node has written.
    keys: Vec<Vec<Key>>,
    /// Record versions that have reached a sink.
    arrived: HashSet<(Key, Version)>,
    /// Write → first arrival at a sink, seconds, by kind.
    delivery: BTreeMap<&'static str, Vec<f32>>,
    commands: Vec<Issued>,
    /// Staleness samples, seconds: (neighbourhood, positions, host), outside and during the
    /// partition.
    stale: [[Vec<f32>; 3]; 2],
    /// Per sample: the share of each kind of staleness sample that was current.
    timeline: Vec<(u64, [f32; 3])>,
    /// Bytes each side sent per session.
    per_session: Vec<f32>,
    totals: Traffic,
    sent_by_node: Vec<u64>,
    /// Most records, outbox entries and routes each node held.
    peak: Vec<[usize; 3]>,
    sessions: u64,
    held_back: u64,
    refused: u64,
    /// Until when each link is down, if it is.
    down_until: Vec<u64>,
    epochs: Vec<u32>,
    outages: u64,
    reboots: u64,
    dropped_sessions: u64,
    write_errors: u64,
}

impl<'a> Sim<'a> {
    fn new(args: &'a MeshSimArgs, layout: Layout, mut rng: SplitMix64) -> Result<Self> {
        let [records, outbox, routes] = args.limits[..] else {
            bail!("--limits takes three numbers: records, outbox, routes");
        };
        let n = args.nodes;
        let mut ids = Vec::new();
        while ids.len() < n {
            // Static random addresses: 48 bits, top two set.
            let id = NodeId(0xc000_0000_0000 | (rng.next_u64() & 0x3fff_ffff_ffff));
            if !ids.contains(&id) && id != CONTROLLER {
                ids.push(id);
            }
        }
        let nodes = (0..n)
            .map(|i| {
                let sink = layout.sinks.contains(&i);
                let mut cfg = Config::new(ids[i], CONTROLLER, sink);
                cfg.peer_timeout_ms = (5 * args.contact * SECOND).max(2 * MINUTE) as u32;
                if !sink {
                    cfg.limits.records = records;
                    cfg.limits.outbox = outbox;
                    cfg.limits.routes = routes;
                }
                let info = NodeInfo {
                    role: Role::Anchor,
                    number: Some(i as u16 + 1),
                    firmware: navgrid_proto::Firmware::new(0, 1, 0, 0, false),
                    config: 0,
                    hops: 0,
                    frame: 0,
                    pinned: None,
                    neighbours: Default::default(),
                };
                Replica::new(cfg, 1, info, 0)
            })
            .collect();
        let links = layout.links.len();
        let mut sim = Self {
            args,
            index: ids.iter().enumerate().map(|(i, &id)| (id, i)).collect(),
            ids,
            nodes,
            controller: Writer::new(CONTROLLER, 1),
            rng,
            queue: BinaryHeap::new(),
            seq: 0,
            end_ms: (args.hours * HOUR as f64) as u64,
            next_range: vec![0; links],
            history: HashMap::new(),
            keys: vec![Vec::new(); n],
            arrived: HashSet::new(),
            delivery: BTreeMap::new(),
            commands: Vec::new(),
            stale: Default::default(),
            timeline: Vec::new(),
            per_session: Vec::new(),
            totals: Traffic::default(),
            sent_by_node: vec![0; n],
            peak: vec![[0; 3]; n],
            sessions: 0,
            held_back: 0,
            refused: 0,
            down_until: vec![0; links],
            epochs: vec![1; args.nodes],
            outages: 0,
            reboots: 0,
            dropped_sessions: 0,
            write_errors: 0,
            layout,
        };
        for i in 0..n {
            sim.observe_node(i, 0);
            let health = sim.phase(HEALTH_EVERY);
            sim.at(health, Event::Health(i));
            let env = sim.phase(ENVIRONMENT_EVERY);
            sim.at(env, Event::Environment(i));
        }
        for l in 0..links {
            let t = sim.phase(args.contact * SECOND);
            sim.at(t, Event::Contact(l));
            sim.next_range[l] = t;
            if args.outages_per_link_h > 0.0 {
                let first = sim.exponential((HOUR as f64 / args.outages_per_link_h) as u64);
                sim.at(first, Event::Outage(l));
            }
        }
        if args.reboots_per_node_day > 0.0 {
            for i in 0..args.nodes {
                let first = sim.exponential((24.0 * HOUR as f64 / args.reboots_per_node_day) as u64);
                sim.at(first, Event::Reboot(i));
            }
        }
        let frame = sim.controller.record(Body::Frame(Frame {
            version: 1,
            yaw_urad: 0,
            mirrored: false,
            offset: PointMm::default(),
        }));
        sim.inject(frame, 0);
        sim.at((args.command_every * MINUTE as f64) as u64, Event::Command);
        sim.at(30 * MINUTE, Event::Positions);
        sim.at(RETRY_AFTER, Event::Retry);
        sim.at(TICK_EVERY, Event::Tick);
        sim.at(WARM_UP, Event::Sample);
        Ok(sim)
    }

    fn phase(&mut self, period: u64) -> u64 {
        (self.rng.uniform() * period as f64) as u64
    }

    fn at(&mut self, t: u64, event: Event) {
        self.seq += 1;
        self.queue.push(Reverse((t, self.seq, event)));
    }

    fn run(&mut self) {
        while let Some(Reverse((now, _, event))) = self.queue.pop() {
            if now > self.end_ms {
                break;
            }
            match event {
                Event::Contact(l) => {
                    self.contact(l, now);
                    // ±5 % so contacts don't lock into step.
                    let period = self.args.contact * SECOND;
                    let next = now + period * 95 / 100 + self.phase(period / 10);
                    self.at(next, Event::Contact(l));
                }
                Event::Health(i) => {
                    let body = Body::Health(Health {
                        battery_mv: 3000 - (now / HOUR) as u16,
                        temperature_cdeg: 2500,
                        uptime_s: (now / SECOND) as u32,
                        queued: self.nodes[i].outbox().len() as u16,
                        bursts: 0,
                        procedures: 0,
                        sessions: 0,
                    });
                    self.write(i, body, now);
                    self.at(now + HEALTH_EVERY, Event::Health(i));
                }
                Event::Environment(i) => {
                    let body = Body::Environment(Environment {
                        temperature_cdeg: 2150,
                        humidity_cpct: 4100,
                        pressure_cpa: 10_132_500 + (self.rng.normal() * 200.0) as u32,
                        gas_ohm: None,
                    });
                    self.write(i, body, now);
                    self.at(now + ENVIRONMENT_EVERY, Event::Environment(i));
                }
                Event::Command => {
                    self.command(now);
                    self.at(now + (self.args.command_every * MINUTE as f64) as u64, Event::Command);
                }
                Event::Positions => {
                    for i in 0..self.nodes.len() {
                        let record = self.controller.record(Body::Position(Position {
                            of: self.ids[i],
                            point: PointMm {
                                x: (self.layout.pos[i][0] * 1000.0) as i32,
                                y: (self.layout.pos[i][1] * 1000.0) as i32,
                                z: 2500,
                            },
                            sigma_mm: [100; 3],
                            frame: 1,
                        }));
                        self.inject(record, now);
                    }
                    self.at(now + HOUR, Event::Positions);
                }
                Event::Retry => {
                    let due: Vec<Record> = self
                        .commands
                        .iter()
                        .filter(|c| c.acked_ms.is_none() && !c.superseded && now - c.issued_ms >= RETRY_AFTER)
                        .map(|c| c.record.clone())
                        .collect();
                    for record in due {
                        for s in self.sinks_for(&record) {
                            let _ = self.nodes[s].inject(record.clone(), now);
                        }
                    }
                    self.at(now + RETRY_AFTER, Event::Retry);
                }
                Event::Outage(l) => {
                    self.down_until[l] = now + (self.args.outage_min * MINUTE as f64) as u64;
                    self.outages += 1;
                    let next = self.exponential((HOUR as f64 / self.args.outages_per_link_h) as u64);
                    self.at(now + next, Event::Outage(l));
                }
                Event::Reboot(i) => {
                    self.reboot(i, now);
                    let next = self.exponential((24.0 * HOUR as f64 / self.args.reboots_per_node_day) as u64);
                    self.at(now + next, Event::Reboot(i));
                }
                Event::Tick => {
                    for i in 0..self.nodes.len() {
                        self.nodes[i].tick(now);
                        self.observe_node(i, now);
                    }
                    self.at(now + TICK_EVERY, Event::Tick);
                }
                Event::Sample => {
                    self.sample(now);
                    // Jittered, so samples don't line up with the hourly position updates.
                    let next = now + SAMPLE_EVERY / 2 + self.phase(SAMPLE_EVERY);
                    self.at(next, Event::Sample);
                }
            }
        }
    }

    /// The partition as times: from, to.
    fn cut(&self) -> Option<(u64, u64)> {
        self.args
            .partition
            .map(|(a, b)| ((a * HOUR as f64) as u64, (b * HOUR as f64) as u64))
    }

    fn partitioned(&self, now: u64) -> bool {
        self.cut().is_some_and(|(a, b)| a <= now && now < b)
    }

    fn contact(&mut self, l: usize, now: u64) {
        let (i, j) = self.layout.links[l];
        if self.partitioned(now) && self.layout.left(i) != self.layout.left(j) {
            return;
        }
        if self.down_until[l] > now {
            return;
        }
        if self.args.session_drops > 0.0 && self.rng.uniform() < self.args.session_drops {
            // Broke off: neither side remembers it, and nothing crossed.
            let (a, b) = (self.ids[i], self.ids[j]);
            self.nodes[i].forget(b);
            self.nodes[j].forget(a);
            self.dropped_sessions += 1;
            return;
        }
        if now >= self.next_range[l] {
            let body = Body::Link(Link {
                peer: self.ids[j],
                distance_mm: self.layout.distance_mm(i, j),
                sigma_mm: 100,
                quality: 200,
                antenna_paths: 4,
                procedures: 10,
                failures: 0,
            });
            self.write(i, body, now);
            self.next_range[l] = now + (self.args.ranging * MINUTE as f64) as u64;
        }
        let (left, right) = self.nodes.split_at_mut(j);
        let mut took = Vec::new();
        let out = session::session(&mut left[i], &mut right[0], now, self.args.budget, |to, r, rx| {
            took.push((to, r.clone(), rx))
        });
        self.sessions += 1;
        for (side, n) in [(&out.a, i), (&out.b, j)] {
            let t = side.sent;
            self.per_session.push(t.total() as f32);
            self.sent_by_node[n] += u64::from(t.total());
            self.totals.frames += t.frames;
            self.totals.control += t.control;
            self.totals.summary += t.summary;
            self.totals.versions += t.versions;
            self.totals.records += t.records;
            self.totals.pushes += t.pushes;
            self.held_back += u64::from(side.held_back);
            self.refused += u64::from(side.refused);
        }
        for (to, record, received) in took {
            let n = if to == To::A { i } else { j };
            if self.nodes[n].is_sink() && received.kept() {
                self.arrive(&record, now);
            }
        }
        for n in [i, j] {
            self.observe_node(n, now);
            self.apply(n, now);
            let r = &self.nodes[n];
            let p = &mut self.peak[n];
            *p = [
                p[0].max(r.records().len()),
                p[1].max(r.outbox().len()),
                p[2].max(r.routes().len()),
            ];
        }
    }

    /// A node restarts: a fresh replica with the next epoch, its records, outbox, routes and
    /// peer memory gone; its new node record counts as a write.
    fn reboot(&mut self, i: usize, now: u64) {
        let cfg = self.nodes[i].config().clone();
        self.epochs[i] += 1;
        let mut info = self.nodes[i].node().clone();
        info.neighbours.clear();
        let replica = Replica::new(cfg, self.epochs[i], info, now);
        if let Some(version) = replica
            .get(&Key {
                origin: self.ids[i],
                topic: Topic::Node,
            })
            .map(|s| s.record().version)
        {
            let key = Key {
                origin: self.ids[i],
                topic: Topic::Node,
            };
            self.wrote(key, version, now);
        }
        self.nodes[i] = replica;
        self.reboots += 1;
    }

    /// A random interval with this mean, as between independent events.
    fn exponential(&mut self, mean: u64) -> u64 {
        let u = self.rng.uniform().max(1e-9);
        (-(u.ln()) * mean as f64) as u64
    }

    fn write(&mut self, i: usize, body: Body, now: u64) {
        let topic = body.topic();
        match self.nodes[i].write(body, now) {
            Ok(version) => {
                let key = Key {
                    origin: self.ids[i],
                    topic,
                };
                self.wrote(key, version, now);
            }
            Err(_) => self.write_errors += 1,
        }
    }

    fn wrote(&mut self, key: Key, version: Version, now: u64) {
        let history = self.history.entry(key).or_default();
        if history.is_empty()
            && let Some(&i) = self.index.get(&key.origin)
        {
            self.keys[i].push(key);
        }
        history.push((version, now));
    }

    /// Replicas write their own node records; picks up a new one.
    fn observe_node(&mut self, i: usize, now: u64) {
        let key = Key {
            origin: self.ids[i],
            topic: Topic::Node,
        };
        let Some(version) = self.nodes[i].get(&key).map(|s| s.record().version) else {
            return;
        };
        let last = self.history.get(&key).and_then(|h| h.last()).map(|&(v, _)| v);
        if last < Some(version) {
            self.wrote(key, version, now);
        }
    }

    fn written_at(&self, key: &Key, version: Version) -> Option<u64> {
        let h = self.history.get(key)?;
        let i = h.binary_search_by_key(&version, |&(v, _)| v).ok()?;
        Some(h[i].1)
    }

    /// A record version reached a sink.
    fn arrive(&mut self, record: &Record, now: u64) {
        let key = record.key();
        if !self.arrived.insert((key, record.version)) {
            return;
        }
        // Node records only: the controller's start at the sinks.
        if self.index.contains_key(&record.origin)
            && let Some(t) = self.written_at(&key, record.version)
        {
            let kind = match key.topic {
                Topic::Node => "node",
                Topic::Health => "health",
                Topic::Environment => "environment",
                Topic::Link(_) => "link",
                Topic::Reported(_) => "report",
                Topic::Position(_) | Topic::Desired(..) | Topic::Frame | Topic::Retired(_) => "other",
            };
            self.delivery.entry(kind).or_default().push((now - t) as f32 / 1000.0);
        }
        if let Body::Reported(r) = &record.body
            && let Some(&target) = self.index.get(&record.origin)
        {
            for c in &mut self.commands {
                if c.target == target && c.acked_ms.is_none() {
                    if c.record.version == r.applied {
                        c.acked_ms = Some(now);
                    } else if c.record.version < r.applied {
                        c.superseded = true;
                    }
                }
            }
        }
    }

    /// Hands a controller record to the sinks that have a route to the node it is about, as the
    /// host would; to every sink if none has, or for the frame.
    fn inject(&mut self, record: Record, now: u64) {
        self.wrote(record.key(), record.version, now);
        for s in self.sinks_for(&record) {
            let _ = self.nodes[s].inject(record.clone(), now);
        }
    }

    fn sinks_for(&self, record: &Record) -> Vec<usize> {
        let subject = record.key().class(CONTROLLER).and_then(|c| c.subject());
        let routed: Vec<usize> = match subject {
            Some(n) => self
                .layout
                .sinks
                .iter()
                .copied()
                .filter(|&s| self.nodes[s].route(n).is_some())
                .collect(),
            None => Vec::new(),
        };
        if routed.is_empty() {
            self.layout.sinks.clone()
        } else {
            routed
        }
    }

    fn command(&mut self, now: u64) {
        let candidates: Vec<usize> = (0..self.nodes.len())
            .filter(|i| !self.layout.sinks.contains(i))
            .collect();
        if candidates.is_empty() {
            return;
        }
        let target = candidates[(self.rng.uniform() * candidates.len() as f64) as usize];
        let record = self.controller.record(Body::Desired(Desired {
            target: self.ids[target],
            command: Command::Blink { times: 3 },
            mac: [0; 8],
        }));
        self.commands.push(Issued {
            target,
            record: record.clone(),
            issued_ms: now,
            applied_ms: None,
            acked_ms: None,
            superseded: false,
        });
        self.inject(record, now);
    }

    /// The node acts on its pending commands and reports them.
    fn apply(&mut self, n: usize, now: u64) {
        let pending: Vec<(Version, Command)> = self.nodes[n].pending_commands().collect();
        for (version, command) in pending {
            let report = Body::Reported(Reported {
                setting: command.setting(),
                applied: version,
                status: Status::Applied,
            });
            self.write(n, report, now);
            for c in &mut self.commands {
                if c.target == n && c.record.version == version && c.applied_ms.is_none() {
                    c.applied_ms = Some(now);
                }
            }
        }
    }

    /// How long `held` has been behind the latest write of `key`: 0 if current, `None` if `key`
    /// was never written.
    fn lag(&self, key: &Key, held: Option<Version>, now: u64) -> Option<f32> {
        let h = self.history.get(key)?;
        Some(match h.iter().find(|&&(v, _)| Some(v) > held) {
            Some(&(_, t)) => (now - t) as f32 / 1000.0,
            None => 0.0,
        })
    }

    fn sample(&mut self, now: u64) {
        let near = self.layout.neighbourhoods();
        let mut samples: [Vec<f32>; 3] = Default::default();
        for (x, observers) in near.iter().enumerate() {
            let position = Key {
                origin: CONTROLLER,
                topic: Topic::Position(self.ids[x]),
            };
            for &o in observers {
                for key in &self.keys[x] {
                    if matches!(key.topic, Topic::Node | Topic::Link(_) | Topic::Environment) {
                        let held = self.nodes[o].get(key).map(|s| s.record().version);
                        samples[0].push(self.lag(key, held, now).unwrap_or(0.0));
                    }
                }
            }
            for &o in observers.iter().chain([&x]) {
                let held = self.nodes[o].get(&position).map(|s| s.record().version);
                if let Some(lag) = self.lag(&position, held, now) {
                    samples[1].push(lag);
                }
            }
            for key in &self.keys[x] {
                let held = self
                    .layout
                    .sinks
                    .iter()
                    .filter_map(|&s| self.nodes[s].get(key).map(|s| s.record().version))
                    .max();
                samples[2].push(self.lag(key, held, now).unwrap_or(0.0));
            }
        }
        let current = samples
            .each_ref()
            .map(|s| s.iter().filter(|&&x| x == 0.0).count() as f32 / s.len().max(1) as f32);
        self.timeline.push((now, current));
        // The hour after a heal goes into the recovery figure only.
        let bucket = match self.cut() {
            Some((a, b)) if a <= now && now < b => Some(1),
            Some((_, b)) if b <= now && now < b + HOUR => None,
            _ => Some(0),
        };
        if let Some(bucket) = bucket {
            for (all, new) in self.stale[bucket].iter_mut().zip(samples) {
                all.extend(new);
            }
        }
    }

    fn report(&mut self) {
        let a = self.args;
        let hops = self.layout.hops();
        let deepest = hops.iter().flatten().max().copied().unwrap_or(0);
        let unreachable = hops.iter().filter(|h| h.is_none()).count();
        let adj = self.layout.neighbours();
        let degrees: Vec<usize> = adj.iter().map(Vec::len).collect();
        println!(
            "mesh: {} nodes ({} sink{}), {} links, degree {}–{} (mean {:.1}), up to {deepest} hops{}",
            a.nodes,
            a.sinks,
            if a.sinks == 1 { "" } else { "s" },
            self.layout.links.len(),
            degrees.iter().min().unwrap_or(&0),
            degrees.iter().max().unwrap_or(&0),
            degrees.iter().sum::<usize>() as f64 / a.nodes as f64,
            if unreachable > 0 {
                format!(", {unreachable} with no path to a sink")
            } else {
                String::new()
            }
        );
        println!(
            "run: {} h, a session every {} s per link, ranging every {} min, budget {} B, {} sessions{}",
            a.hours,
            a.contact,
            a.ranging,
            a.budget,
            self.sessions,
            match a.partition {
                Some((x, y)) => format!(", halves cut from hour {x} to {y}"),
                None => String::new(),
            }
        );
        if self.outages + self.reboots + self.dropped_sessions > 0 {
            println!(
                "faults: {} link outages of {} min, {} reboots, {} sessions broke off",
                self.outages, self.args.outage_min, self.reboots, self.dropped_sessions
            );
        }

        println!("\nstaleness (how long a replica has lacked the latest version; first hour left out):");
        let names = ["neighbourhood", "positions", "host"];
        let current = |s: &[f32]| s.iter().filter(|&&x| x == 0.0).count() as f32 / s.len().max(1) as f32;
        let steady = self.stale[0].each_ref().map(|s| current(s));
        for (during, label) in [(0, ""), (1, " (during the partition)")] {
            for (name, s) in names.iter().zip(&mut self.stale[during]) {
                if s.is_empty() {
                    continue;
                }
                s.sort_by(f32::total_cmp);
                let current = s.iter().filter(|&&x| x == 0.0).count() as f64 / s.len() as f64;
                println!(
                    "  {:<28} {:5.1} % current, p90 {}, p99 {}, max {}",
                    format!("{name}{label}"),
                    current * 100.0,
                    secs(quantile(s, 0.9)),
                    secs(quantile(s, 0.99)),
                    secs(*s.last().unwrap())
                );
            }
        }
        if let Some((_, healed)) = self.cut() {
            // Back to within a point of the steady share current.
            let back = |k: usize| {
                self.timeline
                    .iter()
                    .find(|(t, c)| *t >= healed && c[k] >= steady[k] - 0.01)
                    .map_or("not within the run".into(), |(t, _)| secs((t - healed) as f32 / 1000.0))
            };
            println!(
                "  after the heal: neighbourhood back to normal in {}, positions in {}, host in {}",
                back(0),
                back(1),
                back(2)
            );
        }

        println!("\ndelivery to a sink (write → first arrival):");
        for (kind, d) in &mut self.delivery {
            d.sort_by(f32::total_cmp);
            println!(
                "  {kind:<12} {:>6} versions, p50 {}, p90 {}, max {}",
                d.len(),
                secs(quantile(d, 0.5)),
                secs(quantile(d, 0.9)),
                secs(*d.last().unwrap())
            );
        }

        let issued = self.commands.len();
        let mut applied: Vec<f32> = self
            .commands
            .iter()
            .filter_map(|c| c.applied_ms.map(|t| (t - c.issued_ms) as f32 / 1000.0))
            .collect();
        let mut acked: Vec<f32> = self
            .commands
            .iter()
            .filter_map(|c| c.acked_ms.map(|t| (t - c.issued_ms) as f32 / 1000.0))
            .collect();
        let superseded = self.commands.iter().filter(|c| c.superseded).count();
        applied.sort_by(f32::total_cmp);
        acked.sort_by(f32::total_cmp);
        println!(
            "\ncommands: {issued} issued, {} applied, {} acknowledged at a sink, {superseded} superseded",
            applied.len(),
            acked.len()
        );
        for (what, v) in [("applied", &applied), ("acknowledged", &acked)] {
            if !v.is_empty() {
                println!(
                    "  {what:<12} p50 {}, p90 {}, max {}",
                    secs(quantile(v, 0.5)),
                    secs(quantile(v, 0.9)),
                    secs(*v.last().unwrap())
                );
            }
        }

        let t = self.totals;
        let all = t.total().max(1) as f64;
        self.per_session.sort_by(f32::total_cmp);
        println!(
            "\ntraffic per session, each side: p50 {:.0} B, p90 {:.0} B, max {:.0} B; budget reached {} times, pushes refused {}",
            quantile(&self.per_session, 0.5),
            quantile(&self.per_session, 0.9),
            self.per_session.last().unwrap_or(&0.0),
            self.held_back,
            self.refused
        );
        println!(
            "  split: summaries {:.0} %, versions {:.0} %, records {:.0} %, pushes {:.0} %, control {:.0} %; {:.1} frames per side",
            100.0 * f64::from(t.summary) / all,
            100.0 * f64::from(t.versions) / all,
            100.0 * f64::from(t.records) / all,
            100.0 * f64::from(t.pushes) / all,
            100.0 * f64::from(t.control) / all,
            f64::from(t.frames) / (2 * self.sessions.max(1)) as f64
        );
        let mut per_hour: Vec<f32> = self.sent_by_node.iter().map(|&b| (b as f64 / a.hours) as f32).collect();
        per_hour.sort_by(f32::total_cmp);
        println!(
            "  sent per node: p50 {:.1} kB/h, p90 {:.1} kB/h, max {:.1} kB/h",
            quantile(&per_hour, 0.5) / 1000.0,
            quantile(&per_hour, 0.9) / 1000.0,
            per_hour.last().unwrap_or(&0.0) / 1000.0
        );

        let bytes =
            |p: &[usize; 3]| p[0] * size_of::<Stored>() + p[1] * size_of::<Pending>() + p[2] * size_of::<Route>();
        let nodes: Vec<usize> = (0..a.nodes).filter(|i| !self.layout.sinks.contains(i)).collect();
        let worst = |k: usize| nodes.iter().map(|&n| self.peak[n][k]).max().unwrap_or(0);
        let mut kb: Vec<f32> = nodes.iter().map(|&n| bytes(&self.peak[n]) as f32 / 1024.0).collect();
        kb.sort_by(f32::total_cmp);
        println!(
            "\nmemory, nodes (peak): records {}, outbox {}, routes {}; p50 {:.1} KiB, max {:.1} KiB ({} B per record)",
            worst(0),
            worst(1),
            worst(2),
            if kb.is_empty() { 0.0 } else { quantile(&kb, 0.5) },
            kb.last().unwrap_or(&0.0),
            size_of::<Stored>()
        );
        for &s in &self.layout.sinks {
            let p = self.peak[s];
            println!(
                "  sink {:012x}: records {}, outbox {}, routes {}; {:.1} KiB",
                self.ids[s].0,
                p[0],
                p[1],
                p[2],
                bytes(&p) as f32 / 1024.0
            );
        }
        if self.write_errors > 0 {
            println!("writes refused (full): {}", self.write_errors);
        }
    }
}

fn secs(s: f32) -> String {
    if s < 120.0 {
        format!("{s:.0} s")
    } else {
        format!("{:.1} min", s / 60.0)
    }
}

pub fn run(args: &MeshSimArgs) -> Result<()> {
    if args.sinks == 0 || args.sinks > args.nodes {
        bail!("need between 1 and {} sinks", args.nodes);
    }
    let mut rng = SplitMix64::new(args.seed);
    let layout = Layout::new(args, &mut rng);
    let mut sim = Sim::new(args, layout, rng)?;
    sim.run();
    sim.report();
    Ok(())
}
