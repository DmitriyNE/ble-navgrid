//! Mesh links (architecture §3, stage 3): which neighbours a node links with, who connects to
//! whom, when each link bursts to range, and what a burst yields.
//!
//! - **Neighbours** come from beacons ([`Policy::heard`]). An anchor links with up to
//!   [`Settings::degree`] anchors, the strongest first; a tag with up to [`Settings::tag_links`]
//!   anchors. A beacon with no free links is not a candidate (admission, FR-M4).
//! - **Who connects:** between two anchors the lower node ID is the central and connects; the
//!   higher advertises and accepts. A tag is always the central toward anchors (D-3), and an
//!   asset tag accepts a mobile tag for the final approach (FR-F5).
//! - **Bursts:** the central drives ranging. An anchor link idles at the slow connection
//!   parameters and bursts to the fast ones for [`Settings::burst_procedures`] procedures, which
//!   become one [`Link`] record; a tag's links are made at the fast parameters and stay there
//!   ([`Policy::connect_params`]). Several fast links on one central need their connection
//!   events spaced so one link's subevents fit before the next link's event (the technique of
//!   DEF-26); the firmware sets the SDC's central event spacing when it connects. A node runs one burst at a time; due times carry a jitter so neighbouring
//!   nodes don't burst in step. Anchor links survey every [`Settings::survey_period_ms`] and when
//!   the node is moved; a moving tag rotates bursts over its links without a pause, and takes
//!   one settling burst per link when it stops (FR-T6).
//! - **No clock** (NFR-5): callers pass their local time in milliseconds.

#![no_std]

use heapless::Vec;
pub use navgrid_mesh::Link;
pub use navgrid_proto::{Beacon, NodeId, Role};

/// Beacons a node remembers.
pub const MAX_HEARD: usize = 32;
/// Links a node can hold at once (architecture §5; the firmware's SDC configuration matches).
/// Each costs the firmware about 6.5 KiB of RAM for its link task plus the controller's share;
/// 8 is what fits the nRF54L15 beside the mesh state (spike 2026-11-04-firmware-ram-budget).
pub const MAX_LINKS: usize = 8;
/// Links a node can hold as central: all of an anchor's anchor links, when it is the lowest.
pub const CENTRAL_LINKS: u8 = 6;
/// Links a node can hold as peripheral: anchor links from lower anchors, and tags.
pub const PERIPHERAL_LINKS: u8 = 6;
/// Procedures a burst can hold.
pub const BURST_MAX: usize = 32;
/// Below this a burst's spread is not believed (DEF-7).
pub const SIGMA_FLOOR_MM: u32 = 50;
/// A connection attempt older than this has failed.
pub const CONNECT_TIMEOUT_MS: u64 = 10_000;
/// How long a peer whose first session failed is left alone.
pub const FIRST_SESSION_RETRY_MS: u64 = 5 * 60_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Settings {
    /// Anchor links an anchor keeps.
    pub degree: u8,
    /// Anchor links a tag keeps while it ranges.
    pub tag_links: u8,
    /// Weakest beacon worth linking to.
    pub min_rssi_dbm: i8,
    /// A tag with all its links swaps its weakest for a candidate this much stronger (dB).
    pub hysteresis_db: u8,
    /// Beacons older than this are forgotten.
    pub heard_ttl_ms: u32,
    /// Between survey bursts on an anchor link.
    pub survey_period_ms: u32,
    /// After a failed connection, before trying that peer again.
    pub retry_ms: u32,
    /// Procedures in a survey burst (anchor links).
    pub burst_procedures: u8,
    /// Procedures in a tag's burst: short, so its links take turns quickly.
    pub tag_burst_procedures: u8,
    /// Connection interval while idle, in 1.25 ms units, and the peripheral latency with it.
    pub slow_interval_1250us: u16,
    pub slow_latency: u16,
    /// Connection interval during a burst, in 1.25 ms units.
    pub fast_interval_1250us: u16,
    /// Idle interval of a link with a sink at either end, in 1.25 ms units: a sink is powered and
    /// its links carry the telemetry, so they idle faster than the mesh's.
    pub sink_interval_1250us: u16,
    /// Antenna paths a burst asks for: 4 with diversity on both ends (the 30 ms timing of
    /// spike 2026-11-02), 1 for a dedicated one-to-one link.
    pub paths: u8,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            degree: 6,
            tag_links: 3,
            min_rssi_dbm: -85,
            hysteresis_db: 6,
            heard_ttl_ms: 30_000,
            survey_period_ms: 15 * 60 * 1000,
            retry_ms: 20_000,
            burst_procedures: 10,
            tag_burst_procedures: 4,
            slow_interval_1250us: 800, // 1 s
            slow_latency: 4,
            fast_interval_1250us: 24,  // 30 ms
            sink_interval_1250us: 100, // 125 ms
            paths: 4,
        }
    }
}

impl Settings {
    pub fn slow_interval_ms(&self) -> u32 {
        u32::from(self.slow_interval_1250us) * 5 / 4
    }
    pub fn fast_interval_ms(&self) -> u32 {
        u32::from(self.fast_interval_1250us) * 5 / 4
    }

    /// A tag's connection interval: its links share the radio, each one's subevents fitting in
    /// the fast interval, so the interval is the fast one times the links it keeps, and the
    /// k-th link's events go k fast intervals after the first ([`Policy::connect_params`]).
    pub fn tag_interval_1250us(&self) -> u16 {
        self.fast_interval_1250us
            .saturating_mul(u16::from(self.tag_links.max(1)))
    }
}

/// A beacon heard, with where it came from.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Heard {
    pub id: NodeId,
    pub beacon: Beacon,
    pub rssi_dbm: i8,
    pub last_ms: u64,
}

/// One of the node's links.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LinkState {
    pub peer: NodeId,
    /// This end is the central, and so the one that bursts.
    pub central: bool,
    pub peer_role: Role,
    pub peer_sink: bool,
    pub up_since_ms: u64,
    /// When this end next bursts on the link; `None` while nothing is due (the other end
    /// bursts, or a still tag).
    pub next_burst_ms: Option<u64>,
    pub bursts: u16,
}

/// What the node should do now.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Action {
    /// Connect to this peer, as central.
    Connect(NodeId),
    /// Start a ranging burst on the link to this peer.
    Burst(NodeId),
    /// Drop the link to this peer: a tag that has stopped ranging.
    Disconnect(NodeId),
    Nothing,
}

/// The node's link policy: beacons in, links and actions out.
pub struct Policy {
    me: NodeId,
    role: Role,
    /// This node has a host link.
    sink: bool,
    settings: Settings,
    heard: Vec<Heard, MAX_HEARD>,
    links: Vec<LinkState, MAX_LINKS>,
    /// Peers not to connect to before the time given.
    retry: Vec<(NodeId, u64), 8>,
    /// Connections to a peer that failed in a row, for the backoff.
    failures: Vec<(NodeId, u8), 8>,
    /// Connections under way, with when to give up on them.
    pending: Vec<(NodeId, u64), 4>,
    /// The peer a burst runs with now.
    busy: Option<NodeId>,
    moving: bool,
    /// This node ranges; off, it only routes (the controller's `Ranging` setting).
    ranging: bool,
    /// Tags: links still to take their settling burst after the tag stopped.
    settling: u8,
    rng: u32,
}

impl Policy {
    pub fn new(me: NodeId, role: Role, settings: Settings) -> Self {
        Self::with_sink(me, role, false, settings)
    }

    pub fn with_sink(me: NodeId, role: Role, sink: bool, settings: Settings) -> Self {
        Self {
            me,
            role,
            sink,
            settings,
            heard: Vec::new(),
            links: Vec::new(),
            retry: Vec::new(),
            failures: Vec::new(),
            pending: Vec::new(),
            busy: None,
            moving: role == Role::Anchor,
            ranging: true,
            settling: 0,
            rng: (me.0 as u32) ^ ((me.0 >> 32) as u32) | 1,
        }
    }

    pub fn me(&self) -> NodeId {
        self.me
    }

    pub fn role(&self) -> Role {
        self.role
    }

    pub fn is_sink(&self) -> bool {
        self.sink
    }

    /// Hops to the nearest sink, until the mesh state carries them: 0 at a sink, 1 next to one.
    pub fn hops(&self) -> u8 {
        if self.sink {
            0
        } else if self.links.iter().any(|l| l.peer_sink) {
            1
        } else {
            u8::MAX
        }
    }

    /// Anchor links survey every `period_ms` from now on; links whose next survey is further off
    /// than that are brought forward, so a survey started now begins within a period.
    pub fn set_survey_period_ms(&mut self, period_ms: u32, now_ms: u64) {
        self.settings.survey_period_ms = period_ms;
        if self.role != Role::Anchor {
            return;
        }
        for i in 0..self.links.len() {
            let jitter = u64::from(self.jitter(period_ms / 10));
            let l = &mut self.links[i];
            if let Some(next) = l.next_burst_ms.as_mut() {
                *next = (*next).min(now_ms + u64::from(period_ms) + jitter);
            }
        }
    }

    /// Whether this node ranges. Off, its links don't burst and the beacon says so, so others
    /// leave it out too; it still links and routes. On again, its anchor links survey at once.
    pub fn set_ranging(&mut self, on: bool, now_ms: u64) {
        if self.ranging == on {
            return;
        }
        self.ranging = on;
        self.schedule_surveys(now_ms);
    }

    pub fn ranging(&self) -> bool {
        self.ranging
    }

    /// Whether a link with `peer` ranges: both ends do (a peer not heard counts as ranging).
    fn ranges_with(&self, peer: NodeId) -> bool {
        self.ranging
            && self
                .heard
                .iter()
                .find(|h| h.id == peer)
                .is_none_or(|h| h.beacon.ranging)
    }

    /// An anchor's central links that range and have no survey due get one now, spread over 2 s.
    fn schedule_surveys(&mut self, now_ms: u64) {
        if self.role != Role::Anchor {
            return;
        }
        for i in 0..self.links.len() {
            let (peer, central, due) = (self.links[i].peer, self.links[i].central, self.links[i].next_burst_ms);
            if central && due.is_none() && self.ranges_with(peer) {
                let jitter = u64::from(self.jitter(2_000));
                self.links[i].next_burst_ms = Some(now_ms + jitter);
            }
        }
    }

    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    pub fn links(&self) -> &[LinkState] {
        &self.links
    }

    pub fn heard_list(&self) -> &[Heard] {
        &self.heard
    }

    pub fn link(&self, peer: NodeId) -> Option<&LinkState> {
        self.links.iter().find(|l| l.peer == peer)
    }

    /// Between two anchors the lower ID connects; a tag connects to anchors.
    pub fn is_central(me: NodeId, my_role: Role, peer: NodeId, peer_role: Role) -> bool {
        match (my_role, peer_role) {
            (Role::Anchor, Role::Anchor) => me < peer,
            (Role::Asset | Role::Mobile, Role::Anchor) => true,
            (Role::Mobile, Role::Asset) => true,
            _ => false,
        }
    }

    /// Whether this node accepts a connection from `peer` (admission as peripheral).
    pub fn accepts(&self, peer: NodeId, peer_role: Role) -> bool {
        if self.link(peer).is_some() || self.links.len() >= MAX_LINKS || self.peripheral_links() >= PERIPHERAL_LINKS {
            return false;
        }
        match (self.role, peer_role) {
            (Role::Anchor, Role::Anchor) => peer < self.me && self.anchor_links() < self.settings.degree,
            (Role::Anchor, Role::Asset | Role::Mobile) => true,
            (Role::Asset, Role::Mobile) => true,
            _ => false,
        }
    }

    /// Links this node can still take, for its beacon.
    pub fn free_links(&self) -> u8 {
        (MAX_LINKS - self.links.len()).min(usize::from(PERIPHERAL_LINKS - self.peripheral_links())) as u8
    }

    fn peripheral_links(&self) -> u8 {
        self.links.iter().filter(|l| !l.central).count() as u8
    }

    fn central_links(&self) -> u8 {
        self.links.iter().filter(|l| l.central).count() as u8
    }

    fn anchor_links(&self) -> u8 {
        self.links.iter().filter(|l| l.peer_role == Role::Anchor).count() as u8
    }

    /// A beacon from `id` at `rssi_dbm`.
    pub fn heard(&mut self, id: NodeId, beacon: Beacon, rssi_dbm: i8, now_ms: u64) {
        if id == self.me {
            return;
        }
        // The pre-mesh build advertises 14 free links, more than a mesh node has: a node on it
        // answers no hello, and connecting to it left it off the air (log 2026-11-05). Its own
        // connections are still accepted. The beacon carries no version (DEF-64).
        if usize::from(beacon.free_links) > MAX_LINKS {
            return;
        }
        let entry = Heard {
            id,
            beacon,
            rssi_dbm,
            last_ms: now_ms,
        };
        if let Some(h) = self.heard.iter_mut().find(|h| h.id == id) {
            // Smooth the level a little: beacons come with fading.
            h.rssi_dbm = ((i16::from(h.rssi_dbm) + i16::from(rssi_dbm)) / 2) as i8;
            let resumed = !h.beacon.ranging && beacon.ranging;
            h.beacon = beacon;
            h.last_ms = now_ms;
            if resumed {
                self.schedule_surveys(now_ms);
            }
            return;
        }
        if self.heard.is_full() {
            // Forget the oldest.
            let (i, _) = self
                .heard
                .iter()
                .enumerate()
                .min_by_key(|(_, h)| h.last_ms)
                .expect("not empty");
            self.heard.swap_remove(i);
        }
        let _ = self.heard.push(entry);
    }

    /// Forgets stale beacons and retry marks, and gives up on connections that took too long.
    pub fn tick(&mut self, now_ms: u64) {
        let ttl = u64::from(self.settings.heard_ttl_ms);
        self.heard.retain(|h| now_ms.saturating_sub(h.last_ms) <= ttl);
        self.retry.retain(|(_, until)| *until > now_ms);
        let mut i = 0;
        while i < self.pending.len() {
            if self.pending[i].1 <= now_ms {
                let peer = self.pending[i].0;
                self.connect_failed(peer, now_ms);
            } else {
                i += 1;
            }
        }
    }

    /// A link to `peer` came up.
    pub fn link_up(&mut self, peer: NodeId, central: bool, peer_role: Role, peer_sink: bool, now_ms: u64) {
        self.retry.retain(|(p, _)| *p != peer);
        self.failures.retain(|(p, _)| *p != peer);
        self.pending.retain(|(p, _)| *p != peer);
        if self.link(peer).is_some() {
            return;
        }
        let bursts_now = central && (self.moving || self.role == Role::Anchor) && self.ranges_with(peer);
        // An anchor's first survey is spread over 2 s so neighbours don't burst in step; a tag
        // ranges at once.
        let jitter = if self.role == Role::Anchor {
            self.jitter(2_000)
        } else {
            0
        };
        let state = LinkState {
            peer,
            central,
            peer_role,
            peer_sink,
            up_since_ms: now_ms,
            next_burst_ms: bursts_now.then_some(now_ms + u64::from(jitter)),
            bursts: 0,
        };
        if self.links.push(state).is_err() {
            // Over the budget: the caller should not have accepted it.
        }
    }

    /// The link to `peer` dropped.
    pub fn link_down(&mut self, peer: NodeId, now_ms: u64) {
        self.links.retain(|l| l.peer != peer);
        if self.busy == Some(peer) {
            self.busy = None;
        }
        // An anchor's link that dropped is not retried straight away: the peer may be busy or
        // gone. A tag drops its links on purpose and wants them back as soon as it moves.
        if self.role == Role::Anchor {
            self.defer(peer, now_ms + u64::from(self.settings.retry_ms) / 4);
        }
    }

    /// The link to `peer` came up but its first session failed: a node on another build, or a
    /// broken one. Not tried again for a while, so it doesn't hold a slot and a connection every
    /// few seconds (log 2026-11-04).
    pub fn first_session_failed(&mut self, peer: NodeId, now_ms: u64) {
        self.pending.retain(|(p, _)| *p != peer);
        self.defer(peer, now_ms + FIRST_SESSION_RETRY_MS);
    }

    /// A connection attempt to `peer` failed.
    pub fn connect_failed(&mut self, peer: NodeId, now_ms: u64) {
        self.pending.retain(|(p, _)| *p != peer);
        // Doubling with each failure in a row, up to the first-session backoff: a node
        // initiating to a peer it can't reach neither advertises nor hears much meanwhile.
        let failures = match self.failures.iter_mut().find(|(p, _)| *p == peer) {
            Some((_, n)) => {
                *n = n.saturating_add(1);
                *n
            }
            None => {
                if self.failures.is_full() {
                    self.failures.swap_remove(0);
                }
                let _ = self.failures.push((peer, 1));
                1
            }
        };
        let retry = u64::from(self.settings.retry_ms);
        let jitter = u64::from(self.jitter(retry as u32));
        let wait = (retry / 2) << failures.min(10);
        self.defer(peer, now_ms + wait.min(FIRST_SESSION_RETRY_MS) + jitter);
    }

    fn defer(&mut self, peer: NodeId, until_ms: u64) {
        if let Some(r) = self.retry.iter_mut().find(|(p, _)| *p == peer) {
            r.1 = until_ms;
            return;
        }
        if self.retry.is_full() {
            self.retry.remove(0);
        }
        let _ = self.retry.push((peer, until_ms));
    }

    /// The node's motion state. An anchor that moves surveys all its links now (FR-M5); a tag
    /// ranges only while moving, with one settling burst per link when it stops (FR-T6).
    pub fn set_moving(&mut self, moving: bool, now_ms: u64) {
        match (self.role, moving) {
            (Role::Anchor, true) => {
                for i in 0..self.links.len() {
                    if self.links[i].central {
                        let due = now_ms + u64::from(self.jitter(2_000));
                        self.links[i].next_burst_ms = Some(due);
                    }
                }
                self.moving = true;
            }
            (Role::Anchor, false) => self.moving = true,
            (_, true) => {
                if !self.moving {
                    for l in self.links.iter_mut().filter(|l| l.central) {
                        l.next_burst_ms = Some(now_ms);
                    }
                    // Wherever it rested is not where it is going; what it heard there would
                    // only send it to far anchors first.
                    self.heard.clear();
                }
                self.moving = true;
                self.settling = 0;
            }
            (_, false) => {
                if self.moving {
                    self.settling = self.central_links();
                    for l in self.links.iter_mut().filter(|l| l.central) {
                        l.next_burst_ms = Some(now_ms);
                    }
                }
                self.moving = false;
            }
        }
    }

    pub fn is_moving(&self) -> bool {
        self.moving
    }

    /// What to do now. One burst at a time; connections while there are free central slots and
    /// wanted neighbours; a still tag drops its links once settled.
    pub fn next(&mut self, now_ms: u64) -> Action {
        // A walking tag's bursts never pause, so the handover and the connects come first.
        if let Some(weakest) = self.handover(now_ms) {
            return Action::Disconnect(weakest);
        }
        if (self.role == Role::Anchor || self.moving)
            && let Some(peer) = self.wanted(now_ms)
        {
            // One attempt at a time per peer, and it takes a slot while it runs.
            let _ = self.pending.push((peer, now_ms + CONNECT_TIMEOUT_MS));
            return Action::Connect(peer);
        }
        if self.busy.is_none()
            && let Some(l) = self
                .links
                .iter()
                .filter(|l| l.central && l.next_burst_ms.is_some_and(|t| t <= now_ms) && self.ranges_with(l.peer))
                .min_by_key(|l| l.next_burst_ms)
        {
            let peer = l.peer;
            self.busy = Some(peer);
            return Action::Burst(peer);
        }
        if self.role != Role::Anchor && !self.moving && self.settling == 0 {
            if let Some(l) = self.links.iter().find(|l| l.central && self.busy != Some(l.peer)) {
                return Action::Disconnect(l.peer);
            }
            return Action::Nothing;
        }
        Action::Nothing
    }

    /// A moving tag with all its links: the link to drop for a candidate heard
    /// [`Settings::hysteresis_db`] stronger than it (FR-T1; handover as the tag walks).
    fn handover(&self, now_ms: u64) -> Option<NodeId> {
        if self.role == Role::Anchor || !self.moving || !self.pending.is_empty() {
            return None;
        }
        if self.anchor_links() < self.settings.tag_links {
            return None;
        }
        let level = |id: NodeId| -> i16 {
            self.heard
                .iter()
                .find(|h| h.id == id)
                .map_or(i16::from(i8::MIN), |h| i16::from(h.rssi_dbm))
        };
        let weakest = self
            .links
            .iter()
            .filter(|l| l.central && l.peer_role == Role::Anchor && self.busy != Some(l.peer))
            .min_by_key(|l| level(l.peer))?;
        let best = self
            .heard
            .iter()
            .filter(|h| h.beacon.role == Role::Anchor && h.beacon.free_links > 0 && h.beacon.ranging)
            .filter(|h| h.rssi_dbm >= self.settings.min_rssi_dbm)
            .filter(|h| self.link(h.id).is_none())
            .filter(|h| !self.retry.iter().any(|(p, until)| *p == h.id && *until > now_ms))
            .max_by_key(|h| h.rssi_dbm)?;
        (i16::from(best.rssi_dbm) >= level(weakest.peer) + i16::from(self.settings.hysteresis_db))
            .then_some(weakest.peer)
    }

    /// The best neighbour to connect to now, if any.
    fn wanted(&self, now_ms: u64) -> Option<NodeId> {
        let pending = self.pending.len() as u8;
        if self.pending.is_full()
            || self.central_links() + pending >= CENTRAL_LINKS
            || self.links.len() + usize::from(pending) >= MAX_LINKS
        {
            return None;
        }
        let taken = self.anchor_links() + pending;
        let (slots, want_role) = match self.role {
            Role::Anchor => (self.settings.degree.saturating_sub(taken), Role::Anchor),
            Role::Asset | Role::Mobile => (self.settings.tag_links.saturating_sub(taken), Role::Anchor),
        };
        if slots == 0 {
            return None;
        }
        self.heard
            .iter()
            .filter(|h| h.beacon.role == want_role)
            // A tag links to range: an anchor that only routes is no use to it. Anchors link to
            // it all the same, for the routes.
            .filter(|h| self.role == Role::Anchor || h.beacon.ranging)
            .filter(|h| h.rssi_dbm >= self.settings.min_rssi_dbm)
            .filter(|h| h.beacon.free_links > 0)
            .filter(|h| Self::is_central(self.me, self.role, h.id, h.beacon.role))
            .filter(|h| self.link(h.id).is_none())
            .filter(|h| !self.pending.iter().any(|(p, _)| *p == h.id))
            .filter(|h| !self.retry.iter().any(|(p, until)| *p == h.id && *until > now_ms))
            .max_by_key(|h| h.rssi_dbm)
            .map(|h| h.id)
    }

    /// The burst on the link to `peer` ended; `record` is what it yielded, if anything.
    pub fn burst_done(&mut self, peer: NodeId, now_ms: u64) {
        if self.busy == Some(peer) {
            self.busy = None;
        }
        let (role, moving, settling) = (self.role, self.moving, self.settling);
        let period = u64::from(self.settings.survey_period_ms);
        let jitter = u64::from(self.jitter(self.settings.survey_period_ms / 10));
        let ranges = self.ranges_with(peer);
        let Some(l) = self.links.iter_mut().find(|l| l.peer == peer) else {
            return;
        };
        l.bursts = l.bursts.saturating_add(1);
        l.next_burst_ms = match role {
            _ if !ranges => None,
            Role::Anchor => Some(now_ms + period + jitter),
            _ if moving => Some(now_ms),
            _ => None,
        };
        if role != Role::Anchor && !moving && settling > 0 {
            self.settling -= 1;
        }
    }

    /// Connection parameters this node connects with, as central: `(interval in 1.25 ms units,
    /// peripheral latency)`. Anchor links idle slow and switch to the fast interval for a burst;
    /// a tag's links are short-lived and busy, so they are made at the fast interval and never
    /// switch (a parameter update's instant is six intervals away, too slow for a moving tag).
    pub fn connect_params(&self, peer_sink: bool) -> (u16, u16) {
        match self.role {
            Role::Anchor if self.sink || peer_sink => (self.settings.sink_interval_1250us, 0),
            Role::Anchor => (self.settings.slow_interval_1250us, self.settings.slow_latency),
            Role::Asset | Role::Mobile => (self.settings.tag_interval_1250us(), 0),
        }
    }

    /// Where a new central link's connection events go, in 1.25 ms units after the node's first
    /// fast link's: a tag spaces its links by the fast interval so their subevents don't meet
    /// (DEF-26's spacing, planned for the full count). The firmware passes it to the SDC's
    /// central event spacing; an anchor's bursts run one at a time, so it has nothing to space.
    pub fn event_spacing_1250us(&self) -> u16 {
        match self.role {
            Role::Anchor => 0,
            Role::Asset | Role::Mobile => {
                let fast = self.settings.fast_interval_1250us;
                let k = self.central_links() % self.settings.tag_links.max(1);
                fast.saturating_mul(u16::from(k))
            }
        }
    }

    /// Whether the link to `peer` has to switch to the fast parameters for a burst.
    pub fn switches_for_burst(&self, _peer: NodeId) -> bool {
        self.role == Role::Anchor
    }

    /// Procedures the next burst on the link to `peer` should run.
    pub fn burst_length(&self, _peer: NodeId) -> u8 {
        match self.role {
            Role::Anchor => self.settings.burst_procedures,
            _ => self.settings.tag_burst_procedures,
        }
    }

    pub fn busy(&self) -> Option<NodeId> {
        self.busy
    }

    fn jitter(&mut self, up_to_ms: u32) -> u32 {
        // xorshift32
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        if up_to_ms == 0 { 0 } else { x % up_to_ms }
    }
}

/// The procedures of one burst, and the [`Link`] record they make.
#[derive(Clone, Debug)]
pub struct Burst {
    pub peer: NodeId,
    pub paths: u8,
    distances_mm: Vec<u32, BURST_MAX>,
    qualities: Vec<u8, BURST_MAX>,
    pub failures: u16,
}

impl Burst {
    pub fn new(peer: NodeId, paths: u8) -> Self {
        Self {
            peer,
            paths,
            distances_mm: Vec::new(),
            qualities: Vec::new(),
            failures: 0,
        }
    }

    /// A procedure that produced a distance.
    pub fn add(&mut self, distance_m: f32, quality: u8) {
        if self.distances_mm.is_full() {
            return;
        }
        let mm = (distance_m.max(0.0) * 1000.0) as u32;
        let _ = self.distances_mm.push(mm);
        let _ = self.qualities.push(quality);
    }

    /// A procedure that produced nothing (aborted, too few tones).
    pub fn fail(&mut self) {
        self.failures = self.failures.saturating_add(1);
    }

    pub fn procedures(&self) -> u16 {
        self.distances_mm.len() as u16 + self.failures
    }

    pub fn estimates(&self) -> usize {
        self.distances_mm.len()
    }

    /// The link record: the median distance, a sigma from the spread, the median quality.
    pub fn finish(mut self) -> Option<Link> {
        if self.distances_mm.is_empty() {
            return None;
        }
        let median = median_u32(&mut self.distances_mm);
        let mut dev: Vec<u32, BURST_MAX> = Vec::new();
        for d in &self.distances_mm {
            let _ = dev.push(d.abs_diff(median));
        }
        let mad = median_u32(&mut dev);
        let sigma = (mad * 14826 / 10000).max(SIGMA_FLOOR_MM);
        let mut q: Vec<u32, BURST_MAX> = Vec::new();
        for v in &self.qualities {
            let _ = q.push(u32::from(*v));
        }
        Some(Link {
            peer: self.peer,
            distance_mm: median,
            sigma_mm: sigma,
            quality: median_u32(&mut q) as u8,
            antenna_paths: self.paths,
            procedures: self.distances_mm.len() as u16 + self.failures,
            failures: self.failures,
        })
    }
}

fn median_u32(v: &mut Vec<u32, BURST_MAX>) -> u32 {
    v.sort_unstable();
    let n = v.len();
    if n == 0 {
        0
    } else if n % 2 == 1 {
        v[n / 2]
    } else {
        (v[n / 2 - 1] + v[n / 2]) / 2
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn beacon(role: Role, free_links: u8) -> Beacon {
        Beacon {
            role,
            sink: false,
            attention: false,
            install: false,
            hops: u8::MAX,
            free_links,
            battery_mv: None,
            pressure_dpa: None,
            ranging: true,
            number: None,
            position: None,
        }
    }

    fn anchor(id: u64) -> Policy {
        Policy::new(NodeId(id), Role::Anchor, Settings::default())
    }

    #[test]
    fn the_lower_anchor_connects_and_tags_always_do() {
        assert!(Policy::is_central(NodeId(1), Role::Anchor, NodeId(2), Role::Anchor));
        assert!(!Policy::is_central(NodeId(2), Role::Anchor, NodeId(1), Role::Anchor));
        assert!(Policy::is_central(NodeId(9), Role::Mobile, NodeId(1), Role::Anchor));
        assert!(Policy::is_central(NodeId(9), Role::Asset, NodeId(1), Role::Anchor));
        assert!(Policy::is_central(NodeId(9), Role::Mobile, NodeId(1), Role::Asset));
        assert!(!Policy::is_central(NodeId(1), Role::Anchor, NodeId(9), Role::Mobile));
        assert!(!Policy::is_central(NodeId(1), Role::Asset, NodeId(9), Role::Mobile));
    }

    #[test]
    fn an_anchor_connects_to_the_strongest_higher_anchors_up_to_its_degree() {
        let mut p = anchor(10);
        p.heard(NodeId(5), beacon(Role::Anchor, 3), -50, 0); // lower: it connects to us
        p.heard(NodeId(20), beacon(Role::Anchor, 3), -70, 0);
        p.heard(NodeId(30), beacon(Role::Anchor, 3), -60, 0);
        p.heard(NodeId(40), beacon(Role::Anchor, 0), -40, 0); // full
        p.heard(NodeId(50), beacon(Role::Anchor, 3), -90, 0); // too weak
        p.heard(NodeId(60), beacon(Role::Mobile, 3), -40, 0); // a tag connects to us
        assert_eq!(p.next(1), Action::Connect(NodeId(30)));
        // Attempts run one after another, and a running one takes a slot.
        assert_eq!(p.next(2), Action::Connect(NodeId(20)));
        assert_eq!(p.next(3), Action::Nothing);
        p.link_up(NodeId(30), true, Role::Anchor, false, 100);
        p.connect_failed(NodeId(20), 100);
        assert_eq!(
            p.next(200),
            Action::Burst(NodeId(30)).into_burst_or_nothing(&p),
            "first survey or nothing yet"
        );
        // After the retry time 20 is wanted again.
        p.burst_done(NodeId(30), 300);
        let later = 100 + 2 * u64::from(Settings::default().retry_ms);
        assert_eq!(p.next(later), Action::Connect(NodeId(20)));
    }

    trait IntoBurst {
        fn into_burst_or_nothing(self, p: &Policy) -> Action;
    }
    impl IntoBurst for Action {
        /// The first survey burst is due 0–2 s after link-up; either answer is right at 100 ms in.
        fn into_burst_or_nothing(self, p: &Policy) -> Action {
            match p.busy() {
                Some(peer) => Action::Burst(peer),
                None => Action::Nothing,
            }
        }
    }

    #[test]
    fn degree_counts_links_either_way_round() {
        let mut p = Policy::new(
            NodeId(10),
            Role::Anchor,
            Settings {
                degree: 2,
                ..Settings::default()
            },
        );
        p.link_up(NodeId(5), false, Role::Anchor, false, 0); // 5 connected to us
        p.heard(NodeId(20), beacon(Role::Anchor, 3), -50, 0);
        p.heard(NodeId(30), beacon(Role::Anchor, 3), -60, 0);
        assert_eq!(p.next(1), Action::Connect(NodeId(20)));
        p.link_up(NodeId(20), true, Role::Anchor, false, 1);
        // Degree 2 reached: 30 is not wanted.
        p.burst_done(NodeId(20), 5_000);
        assert_eq!(p.next(5_001), Action::Nothing);
        assert!(!p.accepts(NodeId(7), Role::Anchor), "no room for a third anchor");
        assert!(p.accepts(NodeId(70), Role::Mobile), "tags are always welcome");
        assert!(
            !p.accepts(NodeId(70), Role::Anchor),
            "a higher anchor should connect, not be accepted"
        );
    }

    #[test]
    fn admission_follows_the_slots() {
        let mut p = anchor(100);
        for i in 0..PERIPHERAL_LINKS {
            assert!(p.accepts(NodeId(u64::from(i) + 1), Role::Mobile));
            p.link_up(NodeId(u64::from(i) + 1), false, Role::Mobile, false, 0);
        }
        assert!(!p.accepts(NodeId(99), Role::Mobile));
        assert_eq!(p.free_links(), 0);
        let mut asset = Policy::new(NodeId(200), Role::Asset, Settings::default());
        assert!(asset.accepts(NodeId(300), Role::Mobile), "the final approach");
        assert!(!asset.accepts(NodeId(1), Role::Anchor));
        let mobile = Policy::new(NodeId(300), Role::Mobile, Settings::default());
        assert!(!mobile.accepts(NodeId(200), Role::Asset));
        assert!(!mobile.accepts(NodeId(1), Role::Anchor));
        asset.set_moving(true, 0);
        asset.heard(NodeId(1), beacon(Role::Anchor, 3), -50, 0);
        assert_eq!(asset.next(1), Action::Connect(NodeId(1)));
    }

    #[test]
    fn one_burst_at_a_time_and_anchor_links_survey_on_the_period() {
        let s = Settings::default();
        let mut p = anchor(10);
        p.link_up(NodeId(20), true, Role::Anchor, false, 0);
        p.link_up(NodeId(30), true, Role::Anchor, false, 0);
        p.link_up(NodeId(5), false, Role::Anchor, false, 0);
        let first = p.next(3_000);
        let Action::Burst(a) = first else { panic!("{first:?}") };
        assert_eq!(p.next(3_001), Action::Nothing, "busy");
        p.burst_done(a, 4_000);
        let Action::Burst(b) = p.next(4_001) else {
            panic!("the other link")
        };
        assert_ne!(a, b);
        p.burst_done(b, 5_000);
        assert_eq!(p.next(6_000), Action::Nothing, "both surveyed");
        let next = p.links().iter().filter_map(|l| l.next_burst_ms).min().unwrap();
        assert!(next >= 4_000 + u64::from(s.survey_period_ms));
        assert!(next <= 4_000 + u64::from(s.survey_period_ms) * 11 / 10);
        assert!(p.link(NodeId(5)).unwrap().next_burst_ms.is_none(), "5 bursts, not us");
        // Moved: every link again, now.
        p.set_moving(true, 100_000);
        assert!(matches!(p.next(103_000), Action::Burst(_)));
    }

    #[test]
    fn a_moving_tag_rotates_its_links_and_settles_when_it_stops() {
        let mut t = Policy::new(NodeId(900), Role::Mobile, Settings::default());
        t.heard(NodeId(7), beacon(Role::Anchor, 2), -40, 0);
        assert_eq!(t.next(1), Action::Nothing, "still: no links wanted");
        t.set_moving(true, 10);
        assert!(t.heard_list().is_empty(), "what it heard while resting is forgotten");
        t.heard(NodeId(1), beacon(Role::Anchor, 2), -50, 10);
        t.heard(NodeId(2), beacon(Role::Anchor, 2), -55, 10);
        assert_eq!(t.next(11), Action::Connect(NodeId(1)));
        assert_eq!(t.next(12), Action::Connect(NodeId(2)));
        t.link_up(NodeId(1), true, Role::Anchor, false, 500);
        t.link_up(NodeId(2), true, Role::Anchor, false, 600);
        let mut order = [NodeId(0); 4];
        let mut now = 1_000;
        for slot in &mut order {
            let Action::Burst(peer) = t.next(now) else {
                panic!("a burst at {now}")
            };
            *slot = peer;
            now += 400;
            t.burst_done(peer, now);
        }
        assert_eq!(order[0], order[2]);
        assert_eq!(order[1], order[3]);
        assert_ne!(order[0], order[1]);
        assert_eq!(t.burst_length(NodeId(1)), Settings::default().tag_burst_procedures);
        // Stops: one settling burst per link, then the links go.
        t.set_moving(false, now);
        let Action::Burst(a) = t.next(now + 1) else { panic!() };
        t.burst_done(a, now + 500);
        let Action::Burst(b) = t.next(now + 501) else { panic!() };
        assert_ne!(a, b);
        t.burst_done(b, now + 1_000);
        assert!(matches!(t.next(now + 1_001), Action::Disconnect(_)));
    }

    #[test]
    fn a_pending_attempt_takes_a_slot_until_it_resolves_or_times_out() {
        let mut t = Policy::new(
            NodeId(900),
            Role::Mobile,
            Settings {
                tag_links: 1,
                ..Settings::default()
            },
        );
        t.set_moving(true, 0);
        t.heard(NodeId(1), beacon(Role::Anchor, 2), -50, 0);
        t.heard(NodeId(2), beacon(Role::Anchor, 2), -55, 0);
        assert_eq!(t.next(1), Action::Connect(NodeId(1)));
        assert_eq!(t.next(2), Action::Nothing, "the one slot is taken by the attempt");
        t.tick(CONNECT_TIMEOUT_MS + 2);
        // Timed out: 1 is deferred, 2 is next.
        assert_eq!(t.next(CONNECT_TIMEOUT_MS + 3), Action::Connect(NodeId(2)));
        t.link_up(NodeId(2), true, Role::Anchor, false, CONNECT_TIMEOUT_MS + 500);
        assert!(matches!(t.next(CONNECT_TIMEOUT_MS + 501), Action::Burst(_)));
    }

    #[test]
    fn a_node_that_only_routes_links_but_doesnt_burst_and_tags_skip_it() {
        let mut relay = beacon(Role::Anchor, 3);
        relay.ranging = false;
        // An anchor links to a relay for the routes, but doesn't survey it.
        let mut a = anchor(1);
        a.heard(NodeId(2), relay, -50, 0);
        assert_eq!(a.next(1), Action::Connect(NodeId(2)));
        a.link_up(NodeId(2), true, Role::Anchor, false, 10);
        assert_eq!(a.next(5_000), Action::Nothing, "no survey with a relay");
        // Ranging again: the survey comes within the spread.
        a.heard(NodeId(2), beacon(Role::Anchor, 3), -50, 6_000);
        assert_eq!(a.next(8_001), Action::Burst(NodeId(2)));
        a.burst_done(NodeId(2), 9_000);
        // This node stops ranging: nothing more is due.
        a.set_ranging(false, 9_100);
        assert_eq!(a.next(9_000 + 20 * 60_000), Action::Nothing);
        a.set_ranging(true, 2_000_000);
        assert_eq!(a.next(2_002_001), Action::Burst(NodeId(2)));
        // A tag doesn't link to a relay at all.
        let mut t = Policy::new(NodeId(900), Role::Mobile, Settings::default());
        t.set_moving(true, 0);
        t.heard(NodeId(2), relay, -40, 0);
        t.heard(NodeId(3), beacon(Role::Anchor, 3), -70, 0);
        assert_eq!(t.next(1), Action::Connect(NodeId(3)));
        assert_eq!(t.next(2), Action::Nothing, "the relay is no use to a tag");
    }

    #[test]
    fn a_shorter_survey_period_brings_the_next_surveys_forward() {
        let mut p = anchor(1);
        p.link_up(NodeId(2), true, Role::Anchor, false, 0);
        let Action::Burst(peer) = p.next(5_000) else {
            panic!("first survey")
        };
        p.burst_done(peer, 10_000);
        assert_eq!(p.next(70_000), Action::Nothing, "next survey 15 min away");
        p.set_survey_period_ms(60_000, 70_000);
        assert_eq!(
            p.next(70_000 + 66_001),
            Action::Burst(NodeId(2)),
            "within a period and its jitter"
        );
        p.burst_done(NodeId(2), 140_000);
        assert_eq!(p.next(140_000 + 59_000), Action::Nothing);
        assert_eq!(p.next(140_000 + 66_001), Action::Burst(NodeId(2)));
        // Back to the default: the next one at the default period, not brought forward.
        p.burst_done(NodeId(2), 210_000);
        p.set_survey_period_ms(Settings::default().survey_period_ms, 210_000);
        assert_eq!(
            p.next(210_000 + 66_001),
            Action::Burst(NodeId(2)),
            "already scheduled stays"
        );
    }

    #[test]
    fn failed_connects_to_a_peer_back_off_twice_as_long_each_time() {
        let mut t = anchor(1);
        let retry = u64::from(t.settings.retry_ms);
        let peer = NodeId(900);
        let mut now = 1;
        t.heard(peer, beacon(Role::Anchor, 2), -50, now);
        assert_eq!(t.next(now), Action::Connect(peer));
        // The wait before each new attempt, keeping the beacon fresh meanwhile.
        let wait_until_next_attempt = |t: &mut Policy, now: &mut u64| {
            let deferred = *now;
            loop {
                *now += 1_000;
                t.heard(peer, beacon(Role::Anchor, 2), -50, *now);
                if t.next(*now) == Action::Connect(peer) {
                    return *now - deferred;
                }
                assert!(*now - deferred <= FIRST_SESSION_RETRY_MS + retry, "capped");
            }
        };
        let mut waits = Vec::<u64, 8>::new();
        for _ in 0..5 {
            t.connect_failed(peer, now);
            let _ = waits.push(wait_until_next_attempt(&mut t, &mut now));
        }
        // Each wait is about twice the one before, until the cap.
        for w in waits.windows(2) {
            assert!(w[1] >= w[0] * 3 / 2 || w[0] >= FIRST_SESSION_RETRY_MS, "{:?}", waits);
        }
        assert!(waits[0] < retry * 2 && waits[4] > retry * 8, "{:?}", waits);
        // A link that comes up forgets the failures.
        t.link_up(peer, true, Role::Anchor, false, now);
        t.link_down(peer, now + 1);
        now += retry / 4 + 2; // a dropped link's own pause
        t.heard(peer, beacon(Role::Anchor, 2), -50, now);
        assert_eq!(t.next(now), Action::Connect(peer));
        t.connect_failed(peer, now);
        let wait = wait_until_next_attempt(&mut t, &mut now);
        assert!(wait < retry * 2, "{wait} ms");
    }

    #[test]
    fn a_tag_links_at_the_fast_interval_times_its_links_and_spaces_them() {
        let s = Settings::default();
        let mut t = Policy::new(NodeId(900), Role::Mobile, s);
        assert_eq!(t.connect_params(false), (72, 0), "3 links × 30 ms = 90 ms");
        assert_eq!(s.tag_interval_1250us(), 72);
        assert_eq!(t.event_spacing_1250us(), 0);
        t.link_up(NodeId(1), true, Role::Anchor, false, 0);
        assert_eq!(t.event_spacing_1250us(), 24, "the second link 30 ms after the first");
        t.link_up(NodeId(2), true, Role::Anchor, false, 0);
        assert_eq!(t.event_spacing_1250us(), 48);
        let a = anchor(1);
        assert_eq!(a.connect_params(false), (800, 4), "anchors idle slow");
        assert_eq!(a.connect_params(true), (100, 0), "a link to a sink idles faster");
        assert_eq!(a.event_spacing_1250us(), 0);
        assert!(a.switches_for_burst(NodeId(2)));
        assert!(!t.switches_for_burst(NodeId(2)));
    }

    #[test]
    fn a_walking_tag_swaps_its_weakest_link_for_a_much_stronger_anchor() {
        let mut t = Policy::new(NodeId(900), Role::Mobile, Settings::default());
        t.set_moving(true, 0);
        for (id, rssi) in [(1, -60), (2, -65), (3, -70)] {
            t.heard(NodeId(id), beacon(Role::Anchor, 2), rssi, 0);
            t.link_up(NodeId(id), true, Role::Anchor, false, 0);
        }
        // Take the first burst so nothing is busy, then nothing to do.
        let Action::Burst(b) = t.next(1) else { panic!() };
        t.burst_done(b, 2);
        t.set_moving(true, 2);
        // A new anchor 4 dB better than the weakest: not enough.
        t.heard(NodeId(4), beacon(Role::Anchor, 2), -66, 3);
        let mut bursts = 0;
        for i in 0..3 {
            match t.next(10 + i) {
                Action::Burst(p) => {
                    bursts += 1;
                    t.burst_done(p, 11 + i);
                }
                Action::Disconnect(_) => panic!("4 dB is within the hysteresis"),
                _ => {}
            }
        }
        assert!(bursts > 0);
        // 10 dB better: the weakest goes, and the newcomer is connected next.
        t.heard(NodeId(4), beacon(Role::Anchor, 2), -58, 20);
        let mut saw_disconnect = false;
        for i in 0..6 {
            match t.next(30 + i) {
                Action::Burst(p) => t.burst_done(p, 31 + i),
                Action::Disconnect(p) => {
                    assert_eq!(p, NodeId(3));
                    saw_disconnect = true;
                    t.link_down(p, 31 + i);
                }
                Action::Connect(p) => {
                    assert!(saw_disconnect);
                    assert_eq!(p, NodeId(4));
                    return;
                }
                Action::Nothing => {}
            }
        }
        panic!("no handover");
    }

    #[test]
    fn beacons_expire_and_the_table_keeps_the_newest() {
        let mut p = Policy::new(
            NodeId(1),
            Role::Anchor,
            Settings {
                heard_ttl_ms: 1_000,
                ..Settings::default()
            },
        );
        for i in 0..(MAX_HEARD as u64 + 5) {
            p.heard(NodeId(100 + i), beacon(Role::Anchor, 1), -60, i);
        }
        assert_eq!(p.heard_list().len(), MAX_HEARD);
        assert!(p.heard_list().iter().all(|h| h.id.0 >= 105), "the oldest went");
        p.tick(2_000);
        assert!(p.heard_list().is_empty());
    }

    #[test]
    fn a_burst_makes_a_link_record_from_its_median_and_spread() {
        let mut b = Burst::new(NodeId(2), 4);
        for d in [3.0, 3.1, 2.9, 3.05, 9.0] {
            b.add(d, 200);
        }
        b.fail();
        let link = b.finish().unwrap();
        assert_eq!(link.peer, NodeId(2));
        assert_eq!(link.distance_mm, 3_050);
        assert_eq!(link.sigma_mm, 74, "1.4826 × MAD 50 mm");
        assert_eq!(
            (link.quality, link.antenna_paths, link.procedures, link.failures),
            (200, 4, 6, 1)
        );
        assert!(Burst::new(NodeId(2), 4).finish().is_none());
        let mut tight = Burst::new(NodeId(3), 1);
        tight.add(1.0, 255);
        tight.add(1.0, 255);
        assert_eq!(tight.finish().unwrap().sigma_mm, SIGMA_FLOOR_MM);
    }
}
