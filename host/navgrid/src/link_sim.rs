//! `navgrid link-sim`: the link policy (`navgrid-link`) over a radio timeline.
//!
//! Anchors sit on a jittered grid; tags walk and rest. Nodes hear beacons in scan windows, link
//! as the policy says, idle at the slow connection interval and burst to the fast one. A CS
//! procedure is one or two subevents right after the link's connection events (the timing of
//! spike 2026-11-02); it comes back empty when another link's connection event, or another
//! procedure, overlaps a subevent on either end. The run counts links, bursts, ranges per second
//! and empties per node kind, for the settings the firmware will carry, and the switch time
//! between slow and fast parameters is a setting too, since that is the open question of D-10.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use anyhow::Result;
use navgrid_link::{Action, Beacon, Policy, Role, Settings};
use navgrid_proto::NodeId;
use serde_json::json;

#[derive(clap::Args)]
pub struct LinkSimArgs {
    #[arg(long, default_value_t = 8)]
    anchors: usize,
    #[arg(long, default_value_t = 2)]
    tags: usize,
    /// Metres between anchors.
    #[arg(long, default_value_t = 6.0)]
    spacing: f64,
    #[arg(long, default_value_t = 1.0)]
    hours: f64,
    #[arg(long, default_value_t = 1)]
    seed: u64,
    #[arg(long, default_value_t = 6)]
    degree: u8,
    #[arg(long, default_value_t = 3)]
    tag_links: u8,
    /// Minutes between survey bursts on an anchor link.
    #[arg(long, default_value_t = 15.0)]
    survey_min: f64,
    /// Procedures per survey burst.
    #[arg(long, default_value_t = 10)]
    burst: u8,
    /// Procedures per tag burst.
    #[arg(long, default_value_t = 4)]
    tag_burst: u8,
    /// Idle connection interval, ms.
    #[arg(long, default_value_t = 1000)]
    slow_ms: u32,
    /// Burst connection interval, ms.
    #[arg(long, default_value_t = 30)]
    fast_ms: u32,
    /// Antenna paths per procedure (1, 2 or 4).
    #[arg(long, default_value_t = 4)]
    paths: u8,
    /// Time to switch a link between slow and fast parameters, ms. Default: six slow intervals,
    /// a connection parameter update's earliest instant.
    #[arg(long)]
    switch_ms: Option<u32>,
    /// Peripheral latency on slow links: the peripheral listens on one event in this many plus one.
    #[arg(long, default_value_t = 4)]
    slow_latency: u16,
    /// Time a connection event blocks a subevent, µs (the SDC reserves 2.5 ms; the air time of an
    /// empty event is far less).
    #[arg(long, default_value_t = 2500)]
    acl_us: u64,
    /// Place a central's fast links' events at random instead of spaced apart.
    #[arg(long)]
    no_spacing: bool,
    /// Advertising interval, ms.
    #[arg(long, default_value_t = 1000)]
    adv_ms: u32,
    /// An anchor scans this long every so often (seconds).
    #[arg(long, default_value_t = 3.0)]
    scan_for_s: f64,
    #[arg(long, default_value_t = 60.0)]
    scan_every_s: f64,
    /// A moving tag scans 1 s every so often (seconds).
    #[arg(long, default_value_t = 5.0)]
    tag_scan_every_s: f64,
    #[arg(long, default_value_t = -85)]
    min_rssi: i8,
    /// Path-loss exponent (3 is a cluttered room).
    #[arg(long, default_value_t = 3.0)]
    path_loss: f64,
    /// A tag walks this long, then rests this long (minutes).
    #[arg(long, default_value_t = 2.0)]
    walk_min: f64,
    #[arg(long, default_value_t = 3.0)]
    rest_min: f64,
    #[arg(long)]
    json: bool,
}

const TICK_US: u64 = 100_000;
const CONNECT_US: u64 = 700_000;
const RX_FLOOR_DBM: f64 = -100.0;

/// Procedure timing by path count (spike 2026-11-02): events per procedure, subevents, subevent
/// length.
fn procedure_timing(paths: u8) -> (u64, u64, u64) {
    match paths {
        1 => (2, 1, 11_000),
        2 => (2, 1, 22_000),
        _ => (3, 2, 22_000),
    }
}

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn below(&mut self, n: u64) -> u64 {
        if n == 0 { 0 } else { self.next() % n }
    }
    fn gauss(&mut self) -> f64 {
        let u = self.unit().max(1e-12);
        let v = self.unit();
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Duty {
    Slow,
    Fast,
}

struct BurstRun {
    remaining: u8,
    ok: u16,
    empty: u16,
}

struct LinkRec {
    central: usize,
    peripheral: usize,
    interval_us: u64,
    phase_us: u64,
    latency: u16,
    duty: Duty,
    burst: Option<BurstRun>,
    /// Made at the fast interval and never switched: a tag's link.
    pinned_fast: bool,
}

impl LinkRec {
    fn other(&self, n: usize) -> usize {
        if self.central == n {
            self.peripheral
        } else {
            self.central
        }
    }
    /// Whether a connection event of this link, as end `n` sees it, overlaps [a, b). The
    /// central runs every event; a peripheral with latency listens on one in `latency + 1`.
    fn event_in(&self, n: usize, a: u64, b: u64, acl_us: u64) -> bool {
        let step = if n == self.peripheral {
            self.interval_us * (u64::from(self.latency) + 1)
        } else {
            self.interval_us
        };
        let first = a.saturating_sub(acl_us).saturating_sub(self.phase_us).div_ceil(step);
        let t = self.phase_us + first * step;
        t < b
    }
}

#[derive(Default)]
struct NodeStats {
    ticks: u64,
    links_sum: u64,
    links_max: usize,
    bursts: u32,
    ok: u32,
    empty_acl: u32,
    empty_cs: u32,
    fast_ticks: u64,
    events_per_s_sum: f64,
    // Tags
    moving_us: u64,
    ranges_moving: u32,
    walks: u32,
    first_range_us: Vec<u64>,
    no_link_moving_ticks: u64,
    anchors_per_window: Vec<usize>,
    window_anchors: Vec<usize>,
    window_end_us: u64,
}

struct Node {
    id: NodeId,
    role: Role,
    pos: (f64, f64),
    policy: Policy,
    moving: bool,
    walk_started_us: u64,
    ranged_this_walk: bool,
    links: Vec<usize>,
    /// Subevent windows this node takes part in now: (end, start).
    cs_windows: Vec<(u64, u64)>,
    adv_phase_us: u64,
    stats: NodeStats,
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Ev {
    Tick(usize),
    Scan(usize),
    ConnectDone { from: usize, to: usize },
    Switched(usize),
    Procedure(usize),
    BurstEnd(usize),
    Walk(usize),
}

struct Sim<'a> {
    args: &'a LinkSimArgs,
    rng: Rng,
    nodes: Vec<Node>,
    links: Vec<Option<LinkRec>>,
    queue: BinaryHeap<Reverse<(u64, u64, Ev)>>,
    seq: u64,
    now: u64,
    switch_us: u64,
    end_us: u64,
}

impl<'a> Sim<'a> {
    fn new(args: &'a LinkSimArgs) -> Self {
        let mut rng = Rng(args.seed.wrapping_mul(0x2545_f491_4f6c_dd1d) ^ 0xdead_beef);
        let settings = Settings {
            degree: args.degree,
            tag_links: args.tag_links,
            min_rssi_dbm: args.min_rssi,
            survey_period_ms: (args.survey_min * 60_000.0) as u32,
            burst_procedures: args.burst,
            tag_burst_procedures: args.tag_burst,
            slow_interval_1250us: (args.slow_ms * 4 / 5) as u16,
            slow_latency: args.slow_latency,
            fast_interval_1250us: (args.fast_ms * 4 / 5) as u16,
            paths: args.paths,
            ..Settings::default()
        };
        let side = (args.anchors as f64).sqrt().ceil() as usize;
        let mut nodes = Vec::new();
        for i in 0..args.anchors {
            let (gx, gy) = ((i % side) as f64, (i / side) as f64);
            let j = |r: &mut Rng| (r.unit() - 0.5) * 0.6 * args.spacing;
            let pos = (gx * args.spacing + j(&mut rng), gy * args.spacing + j(&mut rng));
            let id = NodeId(0x1000 + i as u64);
            nodes.push(Node {
                id,
                role: Role::Anchor,
                pos,
                policy: Policy::new(id, Role::Anchor, settings),
                moving: false,
                walk_started_us: 0,
                ranged_this_walk: false,
                links: Vec::new(),
                cs_windows: Vec::new(),
                adv_phase_us: rng.below(u64::from(args.adv_ms) * 1000),
                stats: NodeStats::default(),
            });
        }
        let extent = side as f64 * args.spacing;
        for i in 0..args.tags {
            let id = NodeId(0x9000 + i as u64);
            nodes.push(Node {
                id,
                role: Role::Mobile,
                pos: (rng.unit() * extent, rng.unit() * extent),
                policy: Policy::new(id, Role::Mobile, settings),
                moving: false,
                walk_started_us: 0,
                ranged_this_walk: false,
                links: Vec::new(),
                cs_windows: Vec::new(),
                adv_phase_us: rng.below(u64::from(args.adv_ms) * 1000),
                stats: NodeStats::default(),
            });
        }
        let switch_us = u64::from(args.switch_ms.unwrap_or(6 * args.slow_ms)) * 1000;
        let mut sim = Self {
            args,
            rng,
            nodes,
            links: Vec::new(),
            queue: BinaryHeap::new(),
            seq: 0,
            now: 0,
            switch_us,
            end_us: (args.hours * 3_600_000_000.0) as u64,
        };
        for n in 0..sim.nodes.len() {
            let t = sim.rng.below(TICK_US);
            sim.push(t, Ev::Tick(n));
            let s = sim.rng.below((sim.args.scan_every_s * 1e6) as u64);
            sim.push(s, Ev::Scan(n));
            if sim.nodes[n].role != Role::Anchor {
                let w = sim.rng.below((sim.args.rest_min * 60e6) as u64);
                sim.push(w, Ev::Walk(n));
            }
        }
        sim
    }

    fn push(&mut self, at: u64, ev: Ev) {
        self.seq += 1;
        self.queue.push(Reverse((at, self.seq, ev)));
    }

    fn rssi(&mut self, a: usize, b: usize) -> f64 {
        let (pa, pb) = (self.nodes[a].pos, self.nodes[b].pos);
        let d = ((pa.0 - pb.0).powi(2) + (pa.1 - pb.1).powi(2)).sqrt().max(0.5);
        -40.0 - 10.0 * self.args.path_loss * d.log10() + 3.0 * self.rng.gauss()
    }

    fn beacon_of(&self, n: usize) -> Beacon {
        let node = &self.nodes[n];
        Beacon {
            role: node.role,
            sink: false,
            attention: false,
            install: false,
            hops: u8::MAX,
            free_links: node.policy.free_links(),
            battery_mv: None,
            pressure_dpa: None,
            number: None,
            position: None,
        }
    }

    fn run(&mut self) {
        while let Some(Reverse((t, _, ev))) = self.queue.pop() {
            if t > self.end_us {
                break;
            }
            self.now = t;
            match ev {
                Ev::Tick(n) => self.tick(n),
                Ev::Scan(n) => self.scan(n),
                Ev::ConnectDone { from, to } => self.connect_done(from, to),
                Ev::Switched(l) => self.switched(l),
                Ev::Procedure(l) => self.procedure(l),
                Ev::BurstEnd(l) => self.burst_end(l),
                Ev::Walk(n) => self.walk(n),
            }
        }
    }

    fn tick(&mut self, n: usize) {
        let now_ms = self.now / 1000;
        self.nodes[n].policy.tick(now_ms);
        self.nodes[n].cs_windows.retain(|(end, _)| *end > self.now);
        // Stats.
        let links = self.nodes[n].links.len();
        let st = &mut self.nodes[n].stats;
        st.ticks += 1;
        st.links_sum += links as u64;
        st.links_max = st.links_max.max(links);
        let mut events = 0.0;
        let mut fast = false;
        for &l in &self.nodes[n].links {
            if let Some(l) = &self.links[l] {
                events += 1e6 / l.interval_us as f64;
                fast |= l.duty == Duty::Fast;
            }
        }
        let moving = self.nodes[n].moving;
        let st = &mut self.nodes[n].stats;
        st.events_per_s_sum += events;
        st.fast_ticks += u64::from(fast);
        if moving {
            st.moving_us += TICK_US;
            st.no_link_moving_ticks += u64::from(links == 0);
            if self.now >= st.window_end_us {
                if st.window_end_us > 0 {
                    let distinct = {
                        let mut w = st.window_anchors.clone();
                        w.sort_unstable();
                        w.dedup();
                        w.len()
                    };
                    st.anchors_per_window.push(distinct);
                }
                st.window_anchors.clear();
                st.window_end_us = self.now + 5_000_000;
            }
        }
        for _ in 0..4 {
            match self.nodes[n].policy.next(now_ms) {
                Action::Nothing => break,
                Action::Connect(peer) => {
                    let to = self.index(peer);
                    let at = self.now + CONNECT_US + self.rng.below(300_000);
                    self.push(at, Ev::ConnectDone { from: n, to });
                }
                Action::Burst(peer) => {
                    let to = self.index(peer);
                    let Some(li) = self.link_between(n, to) else {
                        self.nodes[n].policy.burst_done(peer, now_ms);
                        continue;
                    };
                    let length = self.nodes[n].policy.burst_length(peer);
                    let link = self.links[li].as_mut().expect("live link");
                    link.burst = Some(BurstRun {
                        remaining: length,
                        ok: 0,
                        empty: 0,
                    });
                    self.nodes[n].stats.bursts += 1;
                    if link.duty == Duty::Fast {
                        self.push(self.now, Ev::Procedure(li));
                    } else {
                        self.push(self.now + self.switch_us, Ev::Switched(li));
                    }
                }
                Action::Disconnect(peer) => {
                    let to = self.index(peer);
                    if let Some(li) = self.link_between(n, to) {
                        self.drop_link(li);
                    }
                }
            }
        }
        self.push(self.now + TICK_US, Ev::Tick(n));
    }

    fn index(&self, id: NodeId) -> usize {
        self.nodes.iter().position(|n| n.id == id).expect("known node")
    }

    fn link_between(&self, a: usize, b: usize) -> Option<usize> {
        self.nodes[a].links.iter().copied().find(|&l| {
            self.links[l]
                .as_ref()
                .is_some_and(|l| (l.central == a && l.peripheral == b) || (l.central == b && l.peripheral == a))
        })
    }

    fn scan(&mut self, n: usize) {
        let now_ms = self.now / 1000;
        let (window_us, every_us) = if self.nodes[n].role == Role::Anchor {
            (
                (self.args.scan_for_s * 1e6) as u64,
                (self.args.scan_every_s * 1e6) as u64,
            )
        } else {
            (1_000_000, (self.args.tag_scan_every_s * 1e6) as u64)
        };
        let listens = self.nodes[n].role == Role::Anchor || self.nodes[n].moving;
        if listens {
            let adv_us = u64::from(self.args.adv_ms) * 1000;
            for m in 0..self.nodes.len() {
                if m == n {
                    continue;
                }
                // Beacons of m inside [now, now + window).
                let first = self.now.saturating_sub(self.nodes[m].adv_phase_us).div_ceil(adv_us);
                let mut t = self.nodes[m].adv_phase_us + first * adv_us;
                while t < self.now + window_us {
                    let rssi = self.rssi(n, m);
                    if rssi >= RX_FLOOR_DBM {
                        let beacon = self.beacon_of(m);
                        let id = self.nodes[m].id;
                        self.nodes[n]
                            .policy
                            .heard(id, beacon, rssi.round() as i8, now_ms + (t - self.now) / 1000);
                    }
                    t += adv_us;
                }
            }
        }
        self.push(self.now + every_us, Ev::Scan(n));
    }

    fn connect_done(&mut self, from: usize, to: usize) {
        let now_ms = self.now / 1000;
        let (fid, frole) = (self.nodes[from].id, self.nodes[from].role);
        let (tid, trole) = (self.nodes[to].id, self.nodes[to].role);
        let in_range = self.rssi(from, to) >= f64::from(self.args.min_rssi);
        if !in_range || !self.nodes[to].policy.accepts(fid, frole) || self.link_between(from, to).is_some() {
            self.nodes[from].policy.connect_failed(tid, now_ms);
            return;
        }
        let (interval_1250, latency) = self.nodes[from].policy.connect_params(false);
        let interval_us = u64::from(interval_1250) * 1250;
        let pinned_fast = !self.nodes[from].policy.switches_for_burst(tid);
        let phase_us = self.spaced_phase(from, interval_us);
        let rec = LinkRec {
            central: from,
            peripheral: to,
            interval_us,
            phase_us,
            latency,
            duty: if pinned_fast { Duty::Fast } else { Duty::Slow },
            burst: None,
            pinned_fast,
        };
        let li = match self.links.iter().position(Option::is_none) {
            Some(i) => {
                self.links[i] = Some(rec);
                i
            }
            None => {
                self.links.push(Some(rec));
                self.links.len() - 1
            }
        };
        self.nodes[from].links.push(li);
        self.nodes[to].links.push(li);
        self.nodes[from].policy.link_up(tid, true, trole, false, now_ms);
        self.nodes[to].policy.link_up(fid, false, frole, false, now_ms);
    }

    /// The anchor point for a new central link at `interval`: the k-th fast link of a node that
    /// plans `n` of them goes `k × interval / n` after the first (the SDC's central event
    /// spacing, DEF-26, planned for the full count so each link's subevents fit before the next
    /// link's event), or anywhere.
    fn spaced_phase(&mut self, n: usize, interval_us: u64) -> u64 {
        let random = self.now + self.rng.below(interval_us);
        if self.args.no_spacing || interval_us > 200_000 {
            return random;
        }
        let planned = match self.nodes[n].role {
            Role::Anchor => 1,
            _ => u64::from(self.nodes[n].policy.settings().tag_links.max(1)),
        };
        let fast: Vec<u64> = self.nodes[n]
            .links
            .iter()
            .filter_map(|&l| self.links[l].as_ref())
            .filter(|l| l.central == n && l.duty == Duty::Fast)
            .map(|l| l.phase_us)
            .collect();
        let Some(&first) = fast.iter().min() else {
            return random;
        };
        let k = fast.len() as u64 % planned;
        let offset = (first + k * interval_us / planned) % interval_us;
        let base = self.now - self.now % interval_us;
        base + offset
    }

    fn drop_link(&mut self, li: usize) {
        let now_ms = self.now / 1000;
        let Some(rec) = self.links[li].take() else {
            return;
        };
        for n in [rec.central, rec.peripheral] {
            self.nodes[n].links.retain(|&l| l != li);
            let peer = self.nodes[rec.other(n)].id;
            self.nodes[n].policy.link_down(peer, now_ms);
        }
    }

    fn switched(&mut self, li: usize) {
        let Some(link) = self.links[li].as_mut() else {
            return;
        };
        let (fast, slow) = (u64::from(self.args.fast_ms) * 1000, u64::from(self.args.slow_ms) * 1000);
        let central = link.central;
        match (link.duty, link.burst.is_some()) {
            (Duty::Slow, true) => {
                let phase = self.spaced_phase(central, fast);
                let link = self.links[li].as_mut().expect("live");
                link.duty = Duty::Fast;
                link.interval_us = fast;
                link.phase_us = phase;
                self.push(self.now, Ev::Procedure(li));
            }
            (Duty::Fast, false) if !link.pinned_fast => {
                link.duty = Duty::Slow;
                link.interval_us = slow;
                link.phase_us = self.now + self.rng.below(slow);
            }
            _ => {}
        }
    }

    fn procedure(&mut self, li: usize) {
        let Some(link) = self.links[li].as_ref() else {
            return;
        };
        let (central, peripheral) = (link.central, link.peripheral);
        let (events, subevents, sub_us) = procedure_timing(self.args.paths);
        let interval = link.interval_us;
        let e = self.now.saturating_sub(link.phase_us).div_ceil(interval);
        let t0 = link.phase_us + e * interval + self.args.acl_us;
        let windows: Vec<(u64, u64)> = (0..subevents)
            .map(|s| (t0 + s * interval, t0 + s * interval + sub_us))
            .collect();
        // Collisions on either end: another link's connection event, or another procedure.
        let mut acl = false;
        let mut cs = false;
        for n in [central, peripheral] {
            for &(a, b) in &windows {
                for &other in &self.nodes[n].links {
                    if other == li {
                        continue;
                    }
                    if let Some(o) = &self.links[other] {
                        acl |= o.event_in(n, a, b, self.args.acl_us);
                    }
                }
                cs |= self.nodes[n]
                    .cs_windows
                    .iter()
                    .any(|&(end, start)| start < b && a < end);
            }
        }
        for n in [central, peripheral] {
            for &(a, b) in &windows {
                self.nodes[n].cs_windows.push((b, a));
            }
        }
        let empty = acl || cs;
        let link = self.links[li].as_mut().expect("live");
        let run = link.burst.as_mut().expect("bursting");
        if empty {
            run.empty += 1;
        } else {
            run.ok += 1;
        }
        run.remaining = run.remaining.saturating_sub(1);
        let done = run.remaining == 0;
        let (ok, empty_n) = (run.ok, run.empty);
        let st = &mut self.nodes[central].stats;
        if empty {
            if acl {
                st.empty_acl += 1;
            } else {
                st.empty_cs += 1;
            }
        } else {
            st.ok += 1;
            if self.nodes[central].moving {
                let st = &mut self.nodes[central].stats;
                st.ranges_moving += 1;
                st.window_anchors.push(peripheral);
                if !self.nodes[central].ranged_this_walk {
                    self.nodes[central].ranged_this_walk = true;
                    let since = self.now - self.nodes[central].walk_started_us;
                    self.nodes[central].stats.first_range_us.push(since);
                }
            }
        }
        let _ = (ok, empty_n);
        if done {
            let end = windows.last().map_or(self.now, |w| w.1);
            self.push(end, Ev::BurstEnd(li));
        } else {
            self.push(t0 + events * interval - self.args.acl_us, Ev::Procedure(li));
        }
    }

    fn burst_end(&mut self, li: usize) {
        let Some(link) = self.links[li].as_ref() else {
            return;
        };
        let (central, peripheral, pinned) = (link.central, link.peripheral, link.pinned_fast);
        let peer = self.nodes[peripheral].id;
        self.links[li].as_mut().expect("live").burst = None;
        self.nodes[central].policy.burst_done(peer, self.now / 1000);
        if !pinned {
            self.push(self.now + self.switch_us, Ev::Switched(li));
        }
    }

    fn walk(&mut self, n: usize) {
        let now_ms = self.now / 1000;
        let node = &mut self.nodes[n];
        if node.moving {
            node.moving = false;
            node.policy.set_moving(false, now_ms);
            let rest = (self.args.rest_min * 60e6) as u64;
            self.push(self.now + rest, Ev::Walk(n));
        } else {
            node.moving = true;
            node.walk_started_us = self.now;
            node.ranged_this_walk = false;
            node.stats.walks += 1;
            node.policy.set_moving(true, now_ms);
            // A new place for this walk.
            let side = (self.args.anchors as f64).sqrt().ceil() * self.args.spacing;
            node.pos = (self.rng.unit() * side, self.rng.unit() * side);
            let walk = (self.args.walk_min * 60e6) as u64;
            self.push(self.now + walk, Ev::Walk(n));
            // Scan right away.
            self.push(self.now, Ev::Scan(n));
        }
    }
}

fn pct(v: &mut [u64], p: f64) -> u64 {
    if v.is_empty() {
        return 0;
    }
    v.sort_unstable();
    v[((v.len() - 1) as f64 * p).round() as usize]
}

pub fn run(args: &LinkSimArgs) -> Result<()> {
    let mut sim = Sim::new(args);
    sim.run();
    let hours = args.hours;
    let anchors: Vec<&Node> = sim.nodes.iter().filter(|n| n.role == Role::Anchor).collect();
    let tags: Vec<&Node> = sim.nodes.iter().filter(|n| n.role != Role::Anchor).collect();
    let mean = |f: &dyn Fn(&Node) -> f64, v: &[&Node]| -> f64 {
        if v.is_empty() {
            0.0
        } else {
            v.iter().map(|n| f(n)).sum::<f64>() / v.len() as f64
        }
    };
    let links = |n: &Node| n.stats.links_sum as f64 / n.stats.ticks.max(1) as f64;
    let events = |n: &Node| n.stats.events_per_s_sum / n.stats.ticks.max(1) as f64;
    let fast = |n: &Node| n.stats.fast_ticks as f64 / n.stats.ticks.max(1) as f64;
    let total = |n: &Node| f64::from(n.stats.ok + n.stats.empty_acl + n.stats.empty_cs);
    let a_links = mean(&links, &anchors);
    let a_links_max = anchors.iter().map(|n| n.stats.links_max).max().unwrap_or(0);
    let a_bursts_h = mean(&|n| f64::from(n.stats.bursts), &anchors) / hours;
    let a_proc_h = mean(&total, &anchors) / hours;
    let a_empty = {
        let t: f64 = anchors.iter().map(|n| total(n)).sum();
        let e: f64 = anchors
            .iter()
            .map(|n| f64::from(n.stats.empty_acl + n.stats.empty_cs))
            .sum();
        if t > 0.0 { e / t } else { 0.0 }
    };
    let a_events = mean(&events, &anchors);
    let a_fast = mean(&fast, &anchors);
    let t_links = mean(&links, &tags);
    let t_moving_s = mean(&|n| n.stats.moving_us as f64 / 1e6, &tags);
    let t_ranges_s = {
        let r: f64 = tags.iter().map(|n| f64::from(n.stats.ranges_moving)).sum();
        let m: f64 = tags.iter().map(|n| n.stats.moving_us as f64 / 1e6).sum();
        if m > 0.0 { r / m } else { 0.0 }
    };
    let t_empty = {
        let t: f64 = tags.iter().map(|n| total(n)).sum();
        let e: f64 = tags
            .iter()
            .map(|n| f64::from(n.stats.empty_acl + n.stats.empty_cs))
            .sum();
        if t > 0.0 { e / t } else { 0.0 }
    };
    let t_anchors_5s = {
        let all: Vec<usize> = tags
            .iter()
            .flat_map(|n| n.stats.anchors_per_window.iter().copied())
            .collect();
        if all.is_empty() {
            0.0
        } else {
            all.iter().sum::<usize>() as f64 / all.len() as f64
        }
    };
    let mut first: Vec<u64> = tags
        .iter()
        .flat_map(|n| n.stats.first_range_us.iter().copied())
        .collect();
    let walks: u32 = tags.iter().map(|n| n.stats.walks).sum();
    let (first_p50, first_p90) = (pct(&mut first, 0.5) as f64 / 1e6, pct(&mut first, 0.9) as f64 / 1e6);
    let t_no_link = {
        let m: u64 = tags.iter().map(|n| n.stats.moving_us / TICK_US).sum();
        let z: u64 = tags.iter().map(|n| n.stats.no_link_moving_ticks).sum();
        if m > 0 { z as f64 / m as f64 } else { 0.0 }
    };
    let t_events = mean(&events, &tags);
    let a_empty_acl: u32 = anchors.iter().map(|n| n.stats.empty_acl).sum();
    let a_empty_cs: u32 = anchors.iter().map(|n| n.stats.empty_cs).sum();
    let t_empty_acl: u32 = tags.iter().map(|n| n.stats.empty_acl).sum();
    let t_empty_cs: u32 = tags.iter().map(|n| n.stats.empty_cs).sum();
    if args.json {
        println!(
            "{}",
            json!({
                "anchors": args.anchors, "tags": args.tags, "hours": hours,
                "slow_ms": args.slow_ms, "fast_ms": args.fast_ms, "paths": args.paths,
                "switch_ms": sim.switch_us / 1000, "acl_us": args.acl_us, "spacing": !args.no_spacing, "slow_latency": args.slow_latency,
                "anchor": {"links": a_links, "links_max": a_links_max, "bursts_per_h": a_bursts_h,
                           "procedures_per_h": a_proc_h, "empty": a_empty, "empty_acl": a_empty_acl, "empty_cs": a_empty_cs,
                           "events_per_s": a_events, "fast_fraction": a_fast},
                "tag": {"links": t_links, "moving_s": t_moving_s, "ranges_per_s_moving": t_ranges_s, "empty": t_empty,
                        "empty_acl": t_empty_acl, "empty_cs": t_empty_cs,
                        "anchors_per_5s": t_anchors_5s, "first_range_p50_s": first_p50, "first_range_p90_s": first_p90,
                        "walks": walks, "walks_with_a_range": first.len(), "moving_without_links": t_no_link,
                        "first_range_s": first.iter().map(|v| *v as f64 / 1e6).collect::<Vec<_>>(),
                        "events_per_s": t_events},
            })
        );
        return Ok(());
    }
    println!(
        "link-sim: {} anchors {:.0} m apart, {} tags walking {:.0} min and resting {:.0} min, {hours:.1} h; slow {} ms, fast {} ms, {} paths, switch {} ms{}",
        args.anchors,
        args.spacing,
        args.tags,
        args.walk_min,
        args.rest_min,
        args.slow_ms,
        args.fast_ms,
        args.paths,
        sim.switch_us / 1000,
        if args.no_spacing { ", no spacing" } else { "" }
    );
    println!(
        "anchors: {a_links:.1} links on average (max {a_links_max}); {a_bursts_h:.1} bursts/h and {a_proc_h:.0} procedures/h each, {:.1} % empty ({a_empty_acl} by a connection event, {a_empty_cs} by another procedure); {a_events:.1} connection events/s; a fast link {:.1} % of the time",
        100.0 * a_empty,
        100.0 * a_fast
    );
    println!(
        "tags: {t_links:.1} links on average; moving {t_moving_s:.0} s each; {t_ranges_s:.2} ranges/s while moving to {t_anchors_5s:.1} distinct anchors per 5 s; {:.1} % empty ({t_empty_acl} / {t_empty_cs}); first range {first_p50:.1} s after setting off (p90 {first_p90:.1}), {} of {walks} walks got one; moving without a link {:.1} % of the time; {t_events:.1} connection events/s",
        100.0 * t_empty,
        first.len(),
        100.0 * t_no_link
    );
    Ok(())
}
