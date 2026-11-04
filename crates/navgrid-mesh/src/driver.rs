//! One side of a sync session as a state machine over frames ([`wire`]), for devices and the
//! simulator. It runs the same steps as [`session::session`](crate::session::session), which
//! holds both replicas in memory, but each side on its own, fed the peer's frames one at a time
//! and asked for the next frame to send. Memory is bounded: the peer's versions are kept for the
//! class under comparison and the next, records and pushes are drawn from the replica as they are
//! sent, and the pushes sent are remembered by key for the peer's `Accepted`.
//!
//! The steps, both sides alike unless said:
//! 1. `Hello` each way.
//! 2. Summaries: a node sends its own classes; a sink waits for the node's and answers class for
//!    class (two sinks, or two nodes, both send their own). `SummaryDone` ends them.
//! 3. For each class whose hashes differ, in class order: `Versions` frames, the last one short
//!    (empty if the versions filled the last full one), then the `Record`s the peer lacks, as
//!    the peer's versions show, within the budget. `VersionsDone` after the last class.
//! 4. `Push`es, within the budget, then `Done`. A side answers the peer's `Done` with
//!    `Accepted(n)`, how many pushes it took in order before refusing one; the pusher then drops
//!    those from its outbox.
//!
//! A session is complete on a side once it has sent its `Accepted` and received the peer's.

use heapless::Vec;

use crate::session::{MAX_CLASS, MAX_PUSHES, MAX_SUMMARY, Side, frame_len};
use crate::table::Storage;
use crate::wire::{self, Header, SUMMARIES_PER_FRAME, Summary, Sync, VERSIONS_PER_FRAME};
use crate::{Class, Key, Received, Record, Replica, Topic, Version};

/// What the peer did that the protocol doesn't allow.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Protocol {
    /// A frame before the peer's `Hello`.
    NoHello,
    /// More summary classes than a side can hold.
    SummaryTooLong,
    /// Versions of a class not under comparison, or more than a class can hold.
    Versions,
    /// A `Record`, `Push`, `Done` or `Accepted` before the steps that come first.
    OutOfOrder,
}

/// The peer's versions of one class under comparison.
#[derive(Clone, Debug)]
struct ClassVersions {
    class: Option<usize>,
    versions: Vec<(Topic, Version), MAX_CLASS>,
    complete: bool,
}

impl ClassVersions {
    const EMPTY: Self = Self {
        class: None,
        versions: Vec::new(),
        complete: false,
    };
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Out {
    Hello,
    /// The next summary chunk to send, once the summary is computed.
    Summary(usize),
    SummaryDone,
    /// The next versions chunk of the class at this index.
    Versions(usize, usize),
    /// Records of the class at this index, this many sent.
    Records(usize, usize),
    VersionsDone,
    /// This many pushes sent.
    Pushes(usize),
    Done,
    Accepted,
    Finished,
}

/// One side of a session. Call [`next_out`](Self::next_out) for frames to send until it gives
/// none, [`on_frame`](Self::on_frame) with each frame received, and go on until
/// [`is_complete`](Self::is_complete).
pub struct Driver {
    mine: Header,
    theirs: Option<Header>,
    budget_left: u32,
    my_summary: Vec<Summary, MAX_SUMMARY>,
    my_summary_ready: bool,
    their_summary: Vec<Summary, MAX_SUMMARY>,
    their_summary_done: bool,
    classes: Option<Vec<Class, MAX_SUMMARY>>,
    out: Out,
    /// The peer's versions, by class index parity: the class being compared and the next.
    versions: [ClassVersions; 2],
    peer_done: bool,
    accepted_sent: bool,
    accepted_received: bool,
    accepted: u16,
    refused: bool,
    pushed: Vec<(Key, Version), MAX_PUSHES>,
    side: Side,
}

impl Driver {
    /// A session with `budget` bytes of records and pushes to send.
    pub fn new(mine: Header, budget: u32) -> Self {
        Self {
            mine,
            theirs: None,
            budget_left: budget,
            my_summary: Vec::new(),
            my_summary_ready: false,
            their_summary: Vec::new(),
            their_summary_done: false,
            classes: None,
            out: Out::Hello,
            versions: [ClassVersions::EMPTY, ClassVersions::EMPTY],
            peer_done: false,
            accepted_sent: false,
            accepted_received: false,
            accepted: 0,
            refused: false,
            pushed: Vec::new(),
            side: Side::default(),
        }
    }

    pub fn is_complete(&self) -> bool {
        self.accepted_sent && self.accepted_received
    }

    /// What this side sent and took, so far.
    pub fn side(&self) -> Side {
        self.side
    }

    pub fn peer(&self) -> Option<Header> {
        self.theirs
    }

    fn classes_ready<S: Storage>(&mut self, _r: &Replica<S>) -> bool {
        if self.classes.is_none() && self.my_summary_ready && self.their_summary_done {
            let classes = wire::differing(&self.my_summary, &self.their_summary).collect();
            self.classes = Some(classes);
        }
        self.classes.is_some()
    }

    fn class_at(&self, i: usize) -> Option<Class> {
        self.classes.as_ref().and_then(|c| c.get(i).copied())
    }

    /// The next frame to send, or `None` while this side waits for the peer, or when it is done.
    pub fn next_out<S: Storage>(&mut self, r: &mut Replica<S>, now_ms: u64) -> Option<Sync> {
        loop {
            match self.out {
                Out::Hello => {
                    self.out = Out::Summary(0);
                    return Some(self.count(Sync::Hello(self.mine)));
                }
                Out::Summary(chunk) => {
                    if !self.my_summary_ready {
                        let theirs = self.theirs?;
                        if self.mine.sink && !theirs.sink {
                            if !self.their_summary_done {
                                return None;
                            }
                            r.summary(Some(&self.their_summary), now_ms, &mut self.my_summary);
                        } else {
                            r.summary(None, now_ms, &mut self.my_summary);
                        }
                        self.my_summary_ready = true;
                    }
                    let start = chunk * SUMMARIES_PER_FRAME;
                    if start >= self.my_summary.len() {
                        self.out = Out::SummaryDone;
                        continue;
                    }
                    let end = (start + SUMMARIES_PER_FRAME).min(self.my_summary.len());
                    self.out = Out::Summary(chunk + 1);
                    return Some(self.count(Sync::Summary(self.my_summary[start..end].iter().copied().collect())));
                }
                Out::SummaryDone => {
                    self.out = Out::Versions(0, 0);
                    return Some(self.count(Sync::SummaryDone));
                }
                Out::Versions(i, chunk) => {
                    if !self.classes_ready(r) {
                        return None;
                    }
                    let Some(class) = self.class_at(i) else {
                        self.out = Out::VersionsDone;
                        continue;
                    };
                    let mut all: Vec<(Topic, Version), MAX_CLASS> = Vec::new();
                    r.versions(class, now_ms, |t, v| _ = all.push((t, v)));
                    let start = chunk * VERSIONS_PER_FRAME;
                    let end = (start + VERSIONS_PER_FRAME).min(all.len());
                    let versions: Vec<(Topic, Version), VERSIONS_PER_FRAME> =
                        all.get(start..end).unwrap_or(&[]).iter().copied().collect();
                    // A short frame ends the class; an empty one when the last was full.
                    self.out = if versions.len() < VERSIONS_PER_FRAME {
                        Out::Records(i, 0)
                    } else {
                        Out::Versions(i, chunk + 1)
                    };
                    return Some(self.count(Sync::Versions { class, versions }));
                }
                Out::Records(i, sent) => {
                    let Some(class) = self.class_at(i) else {
                        self.out = Out::VersionsDone;
                        continue;
                    };
                    let slot = &self.versions[i % 2];
                    if slot.class != Some(i) || !slot.complete {
                        return None;
                    }
                    let mut n = 0;
                    let mut next: Option<Record> = None;
                    r.missing(class, &slot.versions, now_ms, |rec| {
                        if n == sent {
                            next = Some(rec);
                            return false;
                        }
                        n += 1;
                        true
                    });
                    match next {
                        Some(rec) => {
                            let msg = Sync::Record(rec);
                            let len = frame_len(&msg);
                            if len > self.budget_left {
                                self.side.held_back = true;
                                self.out = Out::Versions(i + 1, 0);
                                continue;
                            }
                            self.budget_left -= len;
                            self.out = Out::Records(i, sent + 1);
                            self.side.sent.add(&msg, len);
                            return Some(msg);
                        }
                        None => {
                            self.out = Out::Versions(i + 1, 0);
                            continue;
                        }
                    }
                }
                Out::VersionsDone => {
                    self.out = Out::Pushes(0);
                    return Some(self.count(Sync::VersionsDone));
                }
                Out::Pushes(sent) => {
                    let theirs = self.theirs?;
                    if self.pushed.is_full() {
                        self.side.held_back = true;
                        self.out = Out::Done;
                        continue;
                    }
                    let mut n = 0;
                    let mut next: Option<Record> = None;
                    r.pushes(&theirs, now_ms, |rec| {
                        if n == sent {
                            next = Some(rec);
                            return false;
                        }
                        n += 1;
                        true
                    });
                    match next {
                        Some(rec) => {
                            let msg = Sync::Push(rec);
                            let len = frame_len(&msg);
                            if len > self.budget_left {
                                self.side.held_back = true;
                                self.out = Out::Done;
                                continue;
                            }
                            self.budget_left -= len;
                            let Sync::Push(rec) = &msg else { unreachable!() };
                            let _ = self.pushed.push((rec.key(), rec.version));
                            self.out = Out::Pushes(sent + 1);
                            self.side.sent.add(&msg, len);
                            return Some(msg);
                        }
                        None => {
                            self.out = Out::Done;
                            continue;
                        }
                    }
                }
                Out::Done => {
                    self.out = Out::Accepted;
                    return Some(self.count(Sync::Done));
                }
                Out::Accepted => {
                    if !self.peer_done {
                        return None;
                    }
                    self.out = Out::Finished;
                    self.accepted_sent = true;
                    return Some(self.count(Sync::Accepted(self.accepted)));
                }
                Out::Finished => return None,
            }
        }
    }

    fn count(&mut self, msg: Sync) -> Sync {
        let len = frame_len(&msg);
        self.side.sent.add(&msg, len);
        msg
    }

    /// A frame from the peer. `took` hears of every record this side kept or took into its
    /// custody.
    pub fn on_frame<S: Storage>(
        &mut self,
        r: &mut Replica<S>,
        msg: Sync,
        now_ms: u64,
        mut took: impl FnMut(&Record, Received),
    ) -> Result<(), Protocol> {
        if self.theirs.is_none() && !matches!(msg, Sync::Hello(_)) {
            return Err(Protocol::NoHello);
        }
        match msg {
            Sync::Hello(h) => {
                self.theirs = Some(h);
                r.contact(&h, now_ms);
            }
            Sync::Summary(chunk) => {
                for s in chunk {
                    self.their_summary.push(s).map_err(|_| Protocol::SummaryTooLong)?;
                }
            }
            Sync::SummaryDone => self.their_summary_done = true,
            Sync::Versions { class, versions } => {
                if !self.classes_ready(r) {
                    return Err(Protocol::OutOfOrder);
                }
                let i = self
                    .classes
                    .as_ref()
                    .and_then(|c| c.iter().position(|&x| x == class))
                    .ok_or(Protocol::Versions)?;
                let slot = &mut self.versions[i % 2];
                if slot.class != Some(i) {
                    *slot = ClassVersions::EMPTY;
                    slot.class = Some(i);
                }
                if slot.complete {
                    return Err(Protocol::Versions);
                }
                let short = versions.len() < VERSIONS_PER_FRAME;
                for v in versions {
                    slot.versions.push(v).map_err(|_| Protocol::Versions)?;
                }
                slot.complete = short;
            }
            Sync::VersionsDone => {}
            Sync::Record(rec) => {
                if let Ok(rx) = r.receive(rec.clone(), now_ms) {
                    self.note(&rec, rx, &mut took);
                }
            }
            Sync::Push(rec) => {
                let theirs = self.theirs.ok_or(Protocol::NoHello)?;
                if self.refused {
                    return Ok(());
                }
                match r.receive_push(&theirs, rec.clone(), now_ms) {
                    Ok(rx) => {
                        self.accepted = self.accepted.saturating_add(1);
                        self.note(&rec, rx, &mut took);
                    }
                    Err(_) => {
                        self.refused = true;
                        self.side.refused += 1;
                    }
                }
            }
            Sync::Done => self.peer_done = true,
            Sync::Accepted(n) => {
                let n = usize::from(n).min(self.pushed.len());
                r.pushed(self.pushed[..n].iter().copied());
                self.accepted_received = true;
            }
        }
        Ok(())
    }

    fn note(&mut self, rec: &Record, rx: Received, took: &mut impl FnMut(&Record, Received)) {
        if rx.kept() || rx == Received::Relayed {
            self.side.took += 1;
            took(rec, rx);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::session::session;
    use crate::{Body, Config, Fixed, Health, Link, NodeId, NodeInfo, Role};
    use std::collections::VecDeque;
    use std::format;
    use std::vec::Vec as StdVec;

    type Node = Replica<Fixed<96, 64, 32>>;
    const CONTROLLER: NodeId = NodeId(0xc0_0000_0000);

    fn node(id: u64, sink: bool) -> Node {
        let info = NodeInfo {
            role: Role::Anchor,
            number: None,
            firmware: navgrid_proto::Firmware::new(0, 1, 0, 0, false),
            config: 0,
            hops: 0,
            frame: 0,
            pinned: None,
            neighbours: Default::default(),
        };
        Replica::new(Config::new(NodeId(id), CONTROLLER, sink), 1, info, 0)
    }

    fn link(peer: u64, distance_mm: u32) -> Body {
        Body::Link(Link {
            peer: NodeId(peer),
            distance_mm,
            sigma_mm: 100,
            quality: 200,
            antenna_paths: 4,
            procedures: 10,
            failures: 0,
        })
    }

    fn health(battery_mv: u16) -> Body {
        Body::Health(Health {
            battery_mv,
            temperature_cdeg: 2500,
            uptime_s: 0,
            queued: 0,
            bursts: 0,
            procedures: 0,
            sessions: 0,
        })
    }

    /// Two drivers over encoded frames, until both complete; the frames each side sent.
    fn run(a: &mut Node, b: &mut Node, now: u64, budget: u32) -> (Driver, Driver, StdVec<Sync>, StdVec<Sync>) {
        let (mut da, mut db) = (Driver::new(a.header(), budget), Driver::new(b.header(), budget));
        let (mut ab, mut ba) = (VecDeque::new(), VecDeque::new());
        let (mut sent_a, mut sent_b) = (StdVec::new(), StdVec::new());
        let through = |m: Sync| {
            let mut buf = [0u8; wire::MAX_FRAME_LEN];
            let frame = wire::encode(&m, &mut buf).expect("fits a frame");
            let end = frame.len() - 1;
            wire::decode(&mut frame[..end]).expect("decodes")
        };
        for _ in 0..10_000 {
            let mut moved = false;
            while let Some(m) = da.next_out(a, now) {
                sent_a.push(m.clone());
                ab.push_back(through(m));
                moved = true;
            }
            while let Some(m) = db.next_out(b, now) {
                sent_b.push(m.clone());
                ba.push_back(through(m));
                moved = true;
            }
            while let Some(m) = ab.pop_front() {
                db.on_frame(b, m, now, |_, _| {}).unwrap();
                moved = true;
            }
            while let Some(m) = ba.pop_front() {
                da.on_frame(a, m, now, |_, _| {}).unwrap();
                moved = true;
            }
            if da.is_complete() && db.is_complete() {
                return (da, db, sent_a, sent_b);
            }
            assert!(moved, "deadlock: {:?} / {:?}", da.out, db.out);
        }
        panic!("no end");
    }

    fn keys(r: &Node) -> StdVec<(Key, Version)> {
        let mut v: StdVec<_> = r
            .records()
            .iter()
            .map(|s| (s.record().key(), s.record().version))
            .collect();
        v.sort_by_key(|(k, v)| (k.origin.0, format!("{:?}", k.topic), v.epoch, v.counter));
        v
    }

    fn fixtures() -> (Node, Node) {
        let (mut sink, mut n) = (node(1, true), node(2, false));
        n.write(link(1, 3_000), 10).unwrap();
        n.write(link(3, 4_500), 20).unwrap();
        n.write(health(2_950), 30).unwrap();
        sink.write(link(2, 3_010), 40).unwrap();
        (sink, n)
    }

    #[test]
    fn two_drivers_end_where_the_in_memory_session_ends() {
        let now = 1_000;
        let (mut sa, mut na) = fixtures();
        let out = session(&mut sa, &mut na, now, 4096, |_, _, _| {});
        let (mut sb, mut nb) = fixtures();
        let (da, db, _, _) = run(&mut sb, &mut nb, now, 4096);
        assert_eq!(keys(&sa), keys(&sb), "the sink holds the same records");
        assert_eq!(keys(&na), keys(&nb), "so does the node");
        assert_eq!(sa.outbox().len(), sb.outbox().len());
        assert_eq!(na.outbox().len(), nb.outbox().len());
        assert_eq!((da.side().took, db.side().took), (out.a.took, out.b.took));
        assert_eq!(
            da.side().sent.total(),
            out.a.sent.total(),
            "the sink sends the same bytes"
        );
        assert_eq!(db.side().sent.total(), out.b.sent.total(), "and the node");
        assert!(sb.outbox().is_empty(), "a sink keeps nothing in custody");
        assert!(
            nb.outbox().is_empty(),
            "the node's health and links were pushed and accepted"
        );
    }

    #[test]
    fn in_sync_sides_exchange_only_control_frames() {
        let now = 1_000;
        let (mut s, mut n) = fixtures();
        run(&mut s, &mut n, now, 4096);
        let (_, _, sent_s, sent_n) = run(&mut s, &mut n, now + 1, 4096);
        for sent in [&sent_s, &sent_n] {
            assert!(
                sent.iter()
                    .all(|m| !matches!(m, Sync::Record(_) | Sync::Push(_) | Sync::Versions { .. })),
                "{sent:?}"
            );
            assert!(matches!(sent.last(), Some(Sync::Accepted(0))));
        }
    }

    #[test]
    fn the_sink_answers_the_summary_it_is_sent() {
        let now = 1_000;
        let (mut s, mut n) = fixtures();
        let (_, _, sent_s, sent_n) = run(&mut s, &mut n, now, 4096);
        let position = |sent: &StdVec<Sync>, f: fn(&Sync) -> bool| sent.iter().position(f).unwrap();
        // The node's summary goes before the sink's answer, which lists the same classes.
        let node_summary_classes: StdVec<Class> = sent_n
            .iter()
            .filter_map(|m| match m {
                Sync::Summary(v) => Some(v.iter().map(|s| s.class).collect::<StdVec<_>>()),
                _ => None,
            })
            .flatten()
            .collect();
        let sink_summary_classes: StdVec<Class> = sent_s
            .iter()
            .filter_map(|m| match m {
                Sync::Summary(v) => Some(v.iter().map(|s| s.class).collect::<StdVec<_>>()),
                _ => None,
            })
            .flatten()
            .collect();
        assert_eq!(node_summary_classes, sink_summary_classes);
        assert!(position(&sent_n, |m| matches!(m, Sync::SummaryDone)) >= 1);
    }

    #[test]
    fn the_budget_holds_records_back_and_the_next_session_brings_them() {
        let now = 1_000;
        let (mut s, mut n) = node_pair_with_many_links();
        let (da, db, _, _) = run(&mut s, &mut n, now, 120);
        assert!(db.side().held_back || da.side().held_back);
        let before = keys(&s).len();
        run(&mut s, &mut n, now + 1, 120);
        run(&mut s, &mut n, now + 2, 4096);
        assert!(keys(&s).len() > before);
        assert_eq!(
            keys(&s).iter().filter(|(k, _)| k.origin == NodeId(2)).count(),
            keys(&n).iter().filter(|(k, _)| k.origin == NodeId(2)).count()
        );
    }

    fn node_pair_with_many_links() -> (Node, Node) {
        let (sink, mut n) = (node(1, true), node(2, false));
        for peer in 3..12 {
            n.write(link(peer, 1_000 * peer as u32), 10 + peer).unwrap();
        }
        (sink, n)
    }

    #[test]
    fn a_frame_before_the_hello_is_a_protocol_error() {
        let (mut s, _) = fixtures();
        let mut d = Driver::new(s.header(), 4096);
        assert_eq!(
            d.on_frame(&mut s, Sync::SummaryDone, 0, |_, _| {}),
            Err(Protocol::NoHello)
        );
    }
}
