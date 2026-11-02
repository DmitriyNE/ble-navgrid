//! A node's replica of the mesh state:
//!
//! - **records** it keeps: its own, its 2-hop neighbourhood's, and the controller's about them
//!   (everything, at a sink);
//! - an **outbox** of records in its custody, on their way to sinks or to the node they are about;
//! - **routes** back toward the nodes whose records it carried;
//! - its mesh **peers**, which give it its hop count and neighbourhood.

use heapless::Vec;
use navgrid_proto::NodeId;

use crate::table::{Storage, Table};
use crate::wire::{Header, Summary};
use crate::{Body, Class, Command, Key, MAX_NEIGHBOURS, NodeInfo, Record, Topic, Version, Writer};

/// Hop count of a node with no path to a sink.
pub const UNREACHABLE: u8 = u8::MAX;
/// Longer paths count as unreachable, which ends count-to-infinity after a sink goes away.
pub const MAX_HOPS: u8 = 32;
/// A time to live that never runs out.
pub const NEVER: u32 = u32::MAX;

/// The most nodes in a 2-hop neighbourhood: each peer and each of theirs.
pub const MAX_NEIGHBOURHOOD: usize = MAX_NEIGHBOURS * (MAX_NEIGHBOURS + 1);

/// How long records stay valid without a newer version, by topic.
#[derive(Clone, Copy, Debug)]
pub struct Ttl {
    pub node: u32,
    pub health: u32,
    pub environment: u32,
    pub link: u32,
    pub position: u32,
    pub desired: u32,
    pub reported: u32,
    pub frame: u32,
}

impl Default for Ttl {
    fn default() -> Self {
        const HOUR: u32 = 3_600_000;
        Self {
            node: 2 * HOUR,
            health: 2 * HOUR,
            environment: HOUR,
            link: 24 * HOUR,
            position: NEVER,
            desired: NEVER,
            reported: NEVER,
            frame: NEVER,
        }
    }
}

impl Ttl {
    pub fn of(&self, topic: &Topic) -> u32 {
        match topic {
            Topic::Node => self.node,
            Topic::Health => self.health,
            Topic::Environment => self.environment,
            Topic::Link(_) => self.link,
            Topic::Position(_) => self.position,
            Topic::Desired(..) => self.desired,
            Topic::Reported(_) => self.reported,
            Topic::Frame => self.frame,
        }
    }

    fn expired(&self, topic: &Topic, age_ms: u32) -> bool {
        let ttl = self.of(topic);
        ttl != NEVER && age_ms > ttl
    }
}

/// Table limits below the tables' capacity, e.g. to model a device on the host.
#[derive(Clone, Copy, Debug)]
pub struct Limits {
    pub records: usize,
    pub outbox: usize,
    pub routes: usize,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            records: usize::MAX,
            outbox: usize::MAX,
            routes: usize::MAX,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Config {
    pub id: NodeId,
    pub controller: NodeId,
    pub sink: bool,
    pub ttl: Ttl,
    /// A peer not heard from for this long no longer counts for hops or the neighbourhood.
    pub peer_timeout_ms: u32,
    /// Records still in the outbox after this long are dropped.
    pub outbox_ttl_ms: u32,
    /// Routes not refreshed for this long are forgotten.
    pub route_ttl_ms: u32,
    pub limits: Limits,
}

impl Config {
    pub fn new(id: NodeId, controller: NodeId, sink: bool) -> Self {
        const MINUTE: u32 = 60_000;
        Self {
            id,
            controller,
            sink,
            ttl: Ttl::default(),
            peer_timeout_ms: 10 * MINUTE,
            outbox_ttl_ms: 24 * 60 * MINUTE,
            route_ttl_ms: 6 * 60 * MINUTE,
            limits: Limits::default(),
        }
    }
}

/// A record as a replica keeps it.
#[derive(Clone, Debug)]
pub struct Stored {
    class: Class,
    /// Its age is as of `since_ms`.
    record: Record,
    since_ms: u64,
}

impl Stored {
    pub fn class(&self) -> Class {
        self.class
    }

    /// The record, with its age when this replica got it; see [`age_ms`](Self::age_ms).
    pub fn record(&self) -> &Record {
        &self.record
    }

    pub fn age_ms(&self, now_ms: u64) -> u32 {
        age(self.record.age_ms, self.since_ms, now_ms)
    }

    fn order(&self) -> (Class, Key) {
        (self.class, self.record.key())
    }
}

/// Where a record in the outbox is going.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Toward {
    Sinks,
    Node(NodeId),
}

/// A record in a replica's custody.
#[derive(Clone, Debug)]
pub struct Pending {
    record: Record,
    since_ms: u64,
    /// When its key entered the outbox. A newer version takes the older one's place in the
    /// queue, so a record rewritten faster than it travels still gets through.
    queued_ms: u64,
    toward: Toward,
}

impl Pending {
    pub fn record(&self) -> &Record {
        &self.record
    }

    pub fn toward(&self) -> Toward {
        self.toward
    }

    /// Commands and their reports go before everything else.
    fn urgent(&self) -> bool {
        matches!(self.record.body.topic(), Topic::Desired(..) | Topic::Reported(_))
    }

    fn order(&self) -> (u64, Key) {
        (self.queued_ms, self.record.key())
    }
}

/// The peer that `origin`'s records last came from, uphill: the way back to `origin`.
#[derive(Clone, Copy, Debug)]
pub struct Route {
    pub origin: NodeId,
    pub via: NodeId,
    since_ms: u64,
}

#[derive(Clone, Copy, Debug)]
pub struct Peer {
    pub id: NodeId,
    pub hops: u8,
    pub sink: bool,
    last_ms: u64,
}

/// What a replica did with a record it was given.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Received {
    /// Kept, under a key it didn't have.
    New,
    /// Kept, newer than the version it had.
    Updated,
    /// Taken into the outbox, to pass on.
    Relayed,
    /// It already had this version or a newer one.
    Stale,
    /// Outside what it keeps.
    Unwanted,
    Expired,
    /// The origin may not write this record.
    Invalid,
    /// Its own record, coming back.
    Own,
}

impl Received {
    pub fn kept(self) -> bool {
        matches!(self, Self::New | Self::Updated)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// A table is full.
    Full,
    /// Not this replica's to write or take: desired state from a node, a push from a peer that
    /// isn't uphill, controller records at a node that isn't a sink.
    Refused,
}

pub struct Replica<S: Storage> {
    cfg: Config,
    writer: Writer,
    node: NodeInfo,
    /// By class, then key.
    records: S::Records,
    /// By key.
    outbox: S::Outbox,
    /// By origin.
    routes: S::Routes,
    peers: Vec<Peer, MAX_NEIGHBOURS>,
}

impl<S: Storage> Replica<S> {
    /// A fresh replica, which writes its node record. `epoch` must be greater than at any earlier
    /// boot (see [`Version`]); `node`'s hop count and neighbours are the replica's to fill in.
    pub fn new(cfg: Config, epoch: u32, mut node: NodeInfo, now_ms: u64) -> Self {
        node.hops = if cfg.sink { 0 } else { UNREACHABLE };
        node.neighbours.clear();
        let mut replica = Self {
            writer: Writer::new(cfg.id, epoch),
            cfg,
            node: node.clone(),
            records: Default::default(),
            outbox: Default::default(),
            routes: Default::default(),
            peers: Vec::new(),
        };
        let _ = replica.write(Body::Node(node), now_ms);
        replica
    }

    pub fn id(&self) -> NodeId {
        self.cfg.id
    }

    pub fn config(&self) -> &Config {
        &self.cfg
    }

    pub fn is_sink(&self) -> bool {
        self.cfg.sink
    }

    pub fn hops(&self) -> u8 {
        self.node.hops
    }

    pub fn node(&self) -> &NodeInfo {
        &self.node
    }

    pub fn header(&self) -> Header {
        Header {
            node: self.cfg.id,
            hops: self.node.hops,
            sink: self.cfg.sink,
        }
    }

    pub fn records(&self) -> &[Stored] {
        &self.records
    }

    pub fn outbox(&self) -> &[Pending] {
        &self.outbox
    }

    pub fn routes(&self) -> &[Route] {
        &self.routes
    }

    pub fn peers(&self) -> &[Peer] {
        &self.peers
    }

    pub fn get(&self, key: &Key) -> Option<&Stored> {
        let class = key.class(self.cfg.controller)?;
        self.find(class, key).ok().map(|i| &self.records[i])
    }

    /// The peer toward `node`, from the records of `node` this replica carried.
    pub fn route(&self, node: NodeId) -> Option<NodeId> {
        let i = self.routes.binary_search_by_key(&node, |r| r.origin).ok()?;
        Some(self.routes[i].via)
    }

    /// Writes one of this node's own records. Records for sinks also go into the outbox.
    pub fn write(&mut self, body: Body, now_ms: u64) -> Result<Version, Error> {
        let key = Key {
            origin: self.cfg.id,
            topic: body.topic(),
        };
        let class = key.class(self.cfg.controller).ok_or(Error::Refused)?;
        let queue = class.upward() && !self.cfg.sink;
        let room = (self.find(class, &key).is_ok() || self.records.len() < self.cfg.limits.records)
            && (!queue || self.outbox_find(&key).is_ok() || self.outbox.len() < self.cfg.limits.outbox);
        if !room {
            return Err(Error::Full);
        }
        let record = self.writer.record(body);
        let version = record.version;
        if queue {
            self.enqueue(record.clone(), Toward::Sinks, now_ms)?;
        }
        self.merge(class, record, now_ms)?;
        Ok(version)
    }

    /// Changes what the node says about itself (role, firmware, config, frame version) and
    /// rewrites its node record if that changed anything. The hop count and neighbours are the
    /// replica's.
    pub fn set_node(&mut self, change: impl FnOnce(&mut NodeInfo), now_ms: u64) -> Result<(), Error> {
        let mut node = self.node.clone();
        change(&mut node);
        node.hops = self.node.hops;
        node.neighbours = self.node.neighbours.clone();
        if node == self.node {
            return Ok(());
        }
        self.node = node.clone();
        self.write(Body::Node(node), now_ms).map(|_| ())
    }

    /// Starts a contact with a peer, from its [`Header`]: updates the peer, the hop count and the
    /// node record.
    pub fn contact(&mut self, peer: &Header, now_ms: u64) {
        if peer.node == self.cfg.id {
            return;
        }
        let entry = Peer {
            id: peer.node,
            hops: peer.hops,
            sink: peer.sink,
            last_ms: now_ms,
        };
        match self.peers.iter().position(|p| p.id == peer.node) {
            Some(i) => self.peers[i] = entry,
            None => {
                if self.peers.is_full()
                    && let Some(i) = (0..self.peers.len()).min_by_key(|&i| self.peers[i].last_ms)
                {
                    self.peers.swap_remove(i);
                }
                let _ = self.peers.push(entry);
            }
        }
        self.update_node(now_ms);
    }

    /// Housekeeping, every few seconds to minutes: drops silent peers, expired and out-of-scope
    /// records, stale outbox entries and routes; refreshes the node record before it expires.
    pub fn tick(&mut self, now_ms: u64) {
        let timeout = u64::from(self.cfg.peer_timeout_ms);
        let peers = self.peers.len();
        self.peers.retain(|p| now_ms.saturating_sub(p.last_ms) <= timeout);
        if self.peers.len() != peers {
            self.update_node(now_ms);
        }
        let own_node = Key {
            origin: self.cfg.id,
            topic: Topic::Node,
        };
        if self
            .get(&own_node)
            .is_none_or(|s| s.age_ms(now_ms) > self.cfg.ttl.node / 2)
        {
            let _ = self.write(Body::Node(self.node.clone()), now_ms);
        }
        let scope = self.scope();
        let ttl = self.cfg.ttl;
        self.records
            .retain(|s| scope.keeps(s.class) && !ttl.expired(&s.record.body.topic(), s.age_ms(now_ms)));
        let outbox_ttl = u64::from(self.cfg.outbox_ttl_ms);
        self.outbox.retain(|p| {
            now_ms.saturating_sub(p.since_ms) <= outbox_ttl
                && !ttl.expired(&p.record.body.topic(), age(p.record.age_ms, p.since_ms, now_ms))
        });
        let route_ttl = u64::from(self.cfg.route_ttl_ms);
        self.routes.retain(|r| now_ms.saturating_sub(r.since_ms) <= route_ttl);
    }

    /// Whether this replica keeps records of `class`: always at a sink; otherwise its own, the
    /// frame, its 2-hop neighbourhood's (and the controller's about them), and desired state for
    /// itself and its peers.
    pub fn keeps(&self, class: Class) -> bool {
        let me = self.cfg.id;
        self.cfg.sink
            || match class {
                Class::Global => true,
                Class::Desired(n) => n == me || self.peers.iter().any(|p| p.id == n),
                Class::About(n) | Class::Near(n) => n == me || self.in_neighbourhood(n),
                Class::Up(n) => n == me,
            }
    }

    /// Whether `node` is a peer or a peer's peer, from the peers' node records.
    pub fn in_neighbourhood(&self, node: NodeId) -> bool {
        node != self.cfg.id
            && self
                .peers
                .iter()
                .any(|p| p.id == node || self.node_info(p.id).is_some_and(|i| i.neighbours.contains(&node)))
    }

    /// This side's summary for a sync session, in class order: every class it keeps, with its
    /// hash. A sink keeps everything, so given the peer's summary (`theirs`) it answers for the
    /// same classes instead. Stops when `out` is full.
    pub fn summary(&self, theirs: Option<&[Summary]>, now_ms: u64, out: &mut impl Table<Summary>) {
        if let (true, Some(theirs)) = (self.cfg.sink, theirs) {
            for s in theirs {
                if out.push(self.summarise(s.class, now_ms)).is_err() {
                    return;
                }
            }
            return;
        }
        let scope = self.scope();
        let me = self.cfg.id;
        let mut targets: Vec<NodeId, { MAX_NEIGHBOURS + 1 }> = scope.peers.iter().copied().collect();
        let _ = targets.push(me);
        targets.sort_unstable();
        let mut subjects: Vec<NodeId, { MAX_NEIGHBOURHOOD + 1 }> = scope.neighbourhood.iter().copied().collect();
        let _ = subjects.push(me);
        subjects.sort_unstable();
        let classes = core::iter::once(Class::Global)
            .chain(targets.iter().map(|&n| Class::Desired(n)))
            .chain(subjects.iter().map(|&n| Class::About(n)))
            .chain(subjects.iter().map(|&n| Class::Near(n)));
        for class in classes {
            if out.push(self.summarise(class, now_ms)).is_err() {
                return;
            }
        }
    }

    /// The topics and versions held in `class`, in topic order.
    pub fn versions(&self, class: Class, now_ms: u64, mut out: impl FnMut(Topic, Version)) {
        for s in self.live(class, now_ms) {
            out(s.record.body.topic(), s.record.version);
        }
    }

    /// The records in `class` that the peer lacks or holds an older version of, given its
    /// versions in topic order. Ages are as of `now_ms`. Stops, returning `false`, when `out`
    /// does.
    pub fn missing(
        &self,
        class: Class,
        theirs: &[(Topic, Version)],
        now_ms: u64,
        mut out: impl FnMut(Record) -> bool,
    ) -> bool {
        for s in self.live(class, now_ms) {
            let topic = s.record.body.topic();
            let newer = match theirs.binary_search_by(|(t, _)| t.cmp(&topic)) {
                Ok(i) => theirs[i].1 < s.record.version,
                Err(_) => true,
            };
            if newer && !out(aged(&s.record, s.since_ms, now_ms)) {
                return false;
            }
        }
        true
    }

    /// A record from a peer's sync step: kept if it falls in this replica's scope.
    pub fn receive(&mut self, record: Record, now_ms: u64) -> Result<Received, Error> {
        let class = match self.check(&record) {
            Ok(class) => class,
            Err(r) => return Ok(r),
        };
        if !self.keeps(class) {
            return Ok(Received::Unwanted);
        }
        self.merge(class, record, now_ms)
    }

    /// The outbox records to push to `peer` now, with their ages as of `now_ms`: those bound for
    /// sinks if the peer is downhill, and those for a node if the peer is that node or the route
    /// to it. Commands and reports come first, then the rest oldest first, so that bulk can't
    /// starve anything. Stops when `out` returns `false`.
    pub fn pushes(&self, peer: &Header, now_ms: u64, mut out: impl FnMut(Record) -> bool) {
        let downhill = !self.cfg.sink && peer.hops < self.node.hops;
        let go = |p: &Pending| match p.toward {
            Toward::Sinks => downhill,
            Toward::Node(n) => n == peer.node || self.route(n) == Some(peer.node),
        };
        for urgent in [true, false] {
            let mut last = None;
            while let Some(p) = self
                .outbox
                .iter()
                .filter(|p| p.urgent() == urgent && last.is_none_or(|l| p.order() > l) && go(p))
                .min_by_key(|p| p.order())
            {
                if !out(aged(&p.record, p.since_ms, now_ms)) {
                    return;
                }
                last = Some(p.order());
            }
        }
    }

    /// A record a peer pushes into this replica's custody. `Ok` means it was taken (or wasn't
    /// needed) and the peer may drop it; an error means the peer keeps it.
    ///
    /// - Bound for sinks: a sink keeps it; other nodes queue it, keep a copy if in scope, and
    ///   learn the route back to its origin.
    /// - The controller's, for a node: kept if in scope (the neighbourhood spreads it from there),
    ///   otherwise queued toward the node.
    pub fn receive_push(&mut self, from: &Header, record: Record, now_ms: u64) -> Result<Received, Error> {
        let class = match self.check(&record) {
            Ok(class) => class,
            Err(r) => return Ok(r),
        };
        let origin = record.origin;
        if class.upward() {
            let received = if self.cfg.sink {
                self.merge(class, record, now_ms)?
            } else {
                if from.hops <= self.node.hops {
                    return Err(Error::Refused);
                }
                let received = self.enqueue(record.clone(), Toward::Sinks, now_ms)?;
                if self.keeps(class) {
                    let _ = self.merge(class, record, now_ms);
                }
                received
            };
            self.learn_route(origin, from.node, now_ms);
            return Ok(received);
        }
        // The frame spreads by sync alone.
        let Some(target) = class.subject() else {
            return Ok(Received::Unwanted);
        };
        if self.keeps(class) {
            return self.merge(class, record, now_ms);
        }
        self.enqueue(record, Toward::Node(target), now_ms)
    }

    /// Drops the pushes a peer accepted from the outbox, unless a newer version has replaced them
    /// since.
    pub fn pushed(&mut self, accepted: impl IntoIterator<Item = (Key, Version)>) {
        for (key, version) in accepted {
            if let Ok(i) = self.outbox_find(&key)
                && self.outbox[i].record.version == version
            {
                self.outbox.remove(i);
            }
        }
    }

    /// A record from the controller, at a sink: kept, and queued toward the node it is about.
    /// Injecting the version held again queues it again, which is how the controller retries.
    pub fn inject(&mut self, record: Record, now_ms: u64) -> Result<Received, Error> {
        if !self.cfg.sink || record.origin != self.cfg.controller {
            return Err(Error::Refused);
        }
        let key = record.key();
        let class = key.class(self.cfg.controller).ok_or(Error::Refused)?;
        let received = self.merge(class, record.clone(), now_ms)?;
        let held = self.get(&key).map(|s| s.record.version);
        if let Some(target) = class.subject()
            && target != self.cfg.id
            && held == Some(record.version)
        {
            self.enqueue(record, Toward::Node(target), now_ms)?;
        }
        Ok(received)
    }

    /// Desired state for this node that it hasn't reported acting on: `(version, command)`.
    pub fn pending_commands(&self) -> impl Iterator<Item = (Version, Command)> + '_ {
        let me = self.cfg.id;
        self.records[self.class_range(Class::Desired(me))]
            .iter()
            .filter_map(move |s| {
                let Body::Desired(d) = &s.record.body else {
                    return None;
                };
                let reported = Key {
                    origin: me,
                    topic: Topic::Reported(d.command.setting()),
                };
                let applied = self.get(&reported).and_then(|r| match &r.record.body {
                    Body::Reported(r) => Some(r.applied),
                    _ => None,
                });
                (applied < Some(s.record.version)).then_some((s.record.version, d.command))
            })
    }

    fn find(&self, class: Class, key: &Key) -> Result<usize, usize> {
        self.records.binary_search_by(|s| s.order().cmp(&(class, *key)))
    }

    fn class_range(&self, class: Class) -> core::ops::Range<usize> {
        self.records.partition_point(|s| s.class < class)..self.records.partition_point(|s| s.class <= class)
    }

    fn live(&self, class: Class, now_ms: u64) -> impl Iterator<Item = &Stored> {
        self.records[self.class_range(class)]
            .iter()
            .filter(move |s| !self.cfg.ttl.expired(&s.record.body.topic(), s.age_ms(now_ms)))
    }

    fn summarise(&self, class: Class, now_ms: u64) -> Summary {
        let hash = self.live(class, now_ms).fold(0u32, |h, s| {
            h.wrapping_add(entry_hash(&s.record.key(), s.record.version))
        });
        Summary {
            class,
            hash: hash.to_le_bytes(),
        }
    }

    fn outbox_find(&self, key: &Key) -> Result<usize, usize> {
        self.outbox.binary_search_by(|p| p.record.key().cmp(key))
    }

    fn node_info(&self, node: NodeId) -> Option<&NodeInfo> {
        let key = Key {
            origin: node,
            topic: Topic::Node,
        };
        match &self.get(&key)?.record.body {
            Body::Node(info) => Some(info),
            _ => None,
        }
    }

    /// The class of a record from elsewhere, or what to answer without looking further.
    fn check(&self, record: &Record) -> Result<Class, Received> {
        let key = record.key();
        let class = key.class(self.cfg.controller).ok_or(Received::Invalid)?;
        if record.origin == self.cfg.id {
            return Err(Received::Own);
        }
        if self.cfg.ttl.expired(&key.topic, record.age_ms) {
            return Err(Received::Expired);
        }
        Ok(class)
    }

    fn merge(&mut self, class: Class, record: Record, now_ms: u64) -> Result<Received, Error> {
        let stored = Stored {
            class,
            record,
            since_ms: now_ms,
        };
        match self.find(class, &stored.record.key()) {
            Ok(i) if self.records[i].record.version >= stored.record.version => Ok(Received::Stale),
            Ok(i) => {
                self.records[i] = stored;
                Ok(Received::Updated)
            }
            Err(_) if self.records.len() >= self.cfg.limits.records => Err(Error::Full),
            Err(i) => {
                self.records.insert(i, stored).map_err(|_| Error::Full)?;
                Ok(Received::New)
            }
        }
    }

    fn enqueue(&mut self, record: Record, toward: Toward, now_ms: u64) -> Result<Received, Error> {
        let mut pending = Pending {
            record,
            since_ms: now_ms,
            queued_ms: now_ms,
            toward,
        };
        match self.outbox_find(&pending.record.key()) {
            Ok(i) if self.outbox[i].record.version >= pending.record.version => Ok(Received::Stale),
            Ok(i) => {
                pending.queued_ms = self.outbox[i].queued_ms;
                self.outbox[i] = pending;
                Ok(Received::Relayed)
            }
            Err(_) if self.outbox.len() >= self.cfg.limits.outbox => Err(Error::Full),
            Err(i) => {
                self.outbox.insert(i, pending).map_err(|_| Error::Full)?;
                Ok(Received::Relayed)
            }
        }
    }

    fn learn_route(&mut self, origin: NodeId, via: NodeId, now_ms: u64) {
        let route = Route {
            origin,
            via,
            since_ms: now_ms,
        };
        match self.routes.binary_search_by_key(&origin, |r| r.origin) {
            Ok(i) => self.routes[i] = route,
            Err(mut i) => {
                if self.routes.len() >= self.cfg.limits.routes {
                    // Forget the stalest route.
                    let Some(old) = (0..self.routes.len()).min_by_key(|&j| self.routes[j].since_ms) else {
                        return;
                    };
                    self.routes.remove(old);
                    if old < i {
                        i -= 1;
                    }
                }
                let _ = self.routes.insert(i, route);
            }
        }
    }

    /// Recomputes the hop count and neighbour list from the peers, and rewrites the node record
    /// if either changed.
    fn update_node(&mut self, now_ms: u64) {
        let hops = if self.cfg.sink {
            0
        } else {
            match self.peers.iter().map(|p| p.hops).min() {
                Some(h) if h < MAX_HOPS => h + 1,
                _ => UNREACHABLE,
            }
        };
        let mut neighbours: Vec<NodeId, MAX_NEIGHBOURS> = self.peers.iter().map(|p| p.id).collect();
        neighbours.sort_unstable();
        if hops != self.node.hops || neighbours != self.node.neighbours {
            self.node.hops = hops;
            self.node.neighbours = neighbours;
            let _ = self.write(Body::Node(self.node.clone()), now_ms);
        }
    }

    fn scope(&self) -> Scope {
        let peers: Vec<NodeId, MAX_NEIGHBOURS> = self.peers.iter().map(|p| p.id).collect();
        let mut neighbourhood: Vec<NodeId, MAX_NEIGHBOURHOOD> = Vec::new();
        for p in &self.peers {
            let _ = neighbourhood.push(p.id);
            for &n in self.node_info(p.id).map_or(&[][..], |i| &i.neighbours[..]) {
                if n != self.cfg.id {
                    let _ = neighbourhood.push(n);
                }
            }
        }
        neighbourhood.sort_unstable();
        dedup_sorted(&mut neighbourhood);
        Scope {
            me: self.cfg.id,
            sink: self.cfg.sink,
            peers,
            neighbourhood,
        }
    }
}

/// [`Replica::keeps`] with the neighbourhood worked out once, for bulk use.
struct Scope {
    me: NodeId,
    sink: bool,
    peers: Vec<NodeId, MAX_NEIGHBOURS>,
    /// Ascending.
    neighbourhood: Vec<NodeId, MAX_NEIGHBOURHOOD>,
}

impl Scope {
    fn keeps(&self, class: Class) -> bool {
        self.sink
            || match class {
                Class::Global => true,
                Class::Desired(n) => n == self.me || self.peers.contains(&n),
                Class::About(n) | Class::Near(n) => n == self.me || self.neighbourhood.binary_search(&n).is_ok(),
                Class::Up(n) => n == self.me,
            }
    }
}

fn dedup_sorted<const N: usize>(v: &mut Vec<NodeId, N>) {
    let mut kept = 0;
    for i in 0..v.len() {
        if kept == 0 || v[i] != v[kept - 1] {
            v[kept] = v[i];
            kept += 1;
        }
    }
    v.truncate(kept);
}

/// `age_ms` as of `since_ms`, carried forward to `now_ms`.
fn age(age_ms: u32, since_ms: u64, now_ms: u64) -> u32 {
    let held = now_ms.saturating_sub(since_ms).min(u64::from(u32::MAX)) as u32;
    age_ms.saturating_add(held)
}

fn aged(record: &Record, since_ms: u64, now_ms: u64) -> Record {
    Record {
        age_ms: age(record.age_ms, since_ms, now_ms),
        ..record.clone()
    }
}

/// SplitMix64's finaliser.
fn mix(mut z: u64) -> u64 {
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

/// One record's share of its class hash. Class hashes add these up, so they don't depend on
/// order.
fn entry_hash(key: &Key, version: Version) -> u32 {
    let (tag, id, setting) = key.topic.parts();
    let mut h = mix(key.origin.0 ^ 0x9e37_79b9_7f4a_7c15);
    h = mix(h ^ ((u64::from(tag) << 56) | u64::from(setting)));
    h = mix(h ^ id);
    h = mix(h ^ ((u64::from(version.epoch) << 32) | u64::from(version.counter)));
    (h >> 32) as u32
}
