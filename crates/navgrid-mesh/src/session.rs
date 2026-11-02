//! One sync session between two replicas in memory: the steps of [`wire`](crate::wire), for
//! tests and the host simulator. Devices run the same steps over L2CAP. The bytes counted are
//! the encoded frames each side would send.

use heapless::Vec;

use crate::table::Storage;
use crate::wire::{self, Header, MAX_FRAME_LEN, SUMMARIES_PER_FRAME, Summary, Sync, VERSIONS_PER_FRAME};
use crate::{Key, MAX_NEIGHBOURHOOD, MAX_NEIGHBOURS, Received, Record, Replica, Topic, Version};

/// The most classes a summary lists: the frame, desired state for a node and its peers, and the
/// node's and controller's records for its 2-hop neighbourhood.
pub const MAX_SUMMARY: usize = 1 + (MAX_NEIGHBOURS + 1) + 2 * (MAX_NEIGHBOURHOOD + 1);
/// The most records one class holds that a session compares.
pub const MAX_CLASS: usize = 32;
/// The most pushes one session carries each way.
pub const MAX_PUSHES: usize = 64;

/// Bytes one side sent, by step.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Traffic {
    pub frames: u32,
    /// `Hello`, `SummaryDone`, `VersionsDone`, `Done` and `Accepted`.
    pub control: u32,
    pub summary: u32,
    pub versions: u32,
    pub records: u32,
    pub pushes: u32,
}

impl Traffic {
    pub fn total(&self) -> u32 {
        self.control + self.summary + self.versions + self.records + self.pushes
    }

    fn count(&mut self, msg: &Sync) -> u32 {
        let len = frame_len(msg);
        self.add(msg, len);
        len
    }

    fn add(&mut self, msg: &Sync, len: u32) {
        self.frames += 1;
        let step = match msg {
            Sync::Summary(_) => &mut self.summary,
            Sync::Versions { .. } => &mut self.versions,
            Sync::Record(_) => &mut self.records,
            Sync::Push(_) => &mut self.pushes,
            _ => &mut self.control,
        };
        *step += len;
    }
}

fn frame_len(msg: &Sync) -> u32 {
    let mut buf = [0u8; MAX_FRAME_LEN];
    wire::encode(msg, &mut buf).map_or(MAX_FRAME_LEN, |f| f.len()) as u32
}

/// One side's part in a session.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Side {
    pub sent: Traffic,
    /// Records it took: kept, or queued to pass on.
    pub took: u32,
    /// Pushes it turned away (full, or not uphill), which the sender keeps.
    pub refused: u32,
    /// Records the budget held back.
    pub held_back: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Outcome {
    pub a: Side,
    pub b: Side,
}

/// Which side of a session a record went to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum To {
    A,
    B,
}

/// Runs one sync session between neighbours `a` and `b`. Each side sends at most `budget` bytes
/// of records and pushes. `took` hears of every record a side took, with what it did with it.
pub fn session<A: Storage, B: Storage>(
    a: &mut Replica<A>,
    b: &mut Replica<B>,
    now_ms: u64,
    budget: u32,
    mut took: impl FnMut(To, &Record, Received),
) -> Outcome {
    let mut out = Outcome::default();
    let (ha, hb) = (a.header(), b.header());
    out.a.sent.count(&Sync::Hello(ha));
    out.b.sent.count(&Sync::Hello(hb));
    a.contact(&hb, now_ms);
    b.contact(&ha, now_ms);

    let mut sa: Vec<Summary, MAX_SUMMARY> = Vec::new();
    let mut sb: Vec<Summary, MAX_SUMMARY> = Vec::new();
    match (a.is_sink(), b.is_sink()) {
        (true, false) => {
            b.summary(None, now_ms, &mut sb);
            a.summary(Some(&sb), now_ms, &mut sa);
        }
        (false, true) => {
            a.summary(None, now_ms, &mut sa);
            b.summary(Some(&sa), now_ms, &mut sb);
        }
        _ => {
            a.summary(None, now_ms, &mut sa);
            b.summary(None, now_ms, &mut sb);
        }
    }
    count_summary(&mut out.a.sent, &sa);
    count_summary(&mut out.b.sent, &sb);

    let (mut left_a, mut left_b) = (budget, budget);
    for class in wire::differing(&sa, &sb) {
        let mut va: Vec<(Topic, Version), MAX_CLASS> = Vec::new();
        let mut vb: Vec<(Topic, Version), MAX_CLASS> = Vec::new();
        a.versions(class, now_ms, |t, v| _ = va.push((t, v)));
        b.versions(class, now_ms, |t, v| _ = vb.push((t, v)));
        count_versions(&mut out.a.sent, class, &va);
        count_versions(&mut out.b.sent, class, &vb);
        let mut to_b: Vec<Record, MAX_CLASS> = Vec::new();
        let mut to_a: Vec<Record, MAX_CLASS> = Vec::new();
        out.a.held_back |= !a.missing(class, &vb, now_ms, |r| {
            send(&mut out.a.sent, &mut left_a, &mut to_b, r, false)
        });
        out.b.held_back |= !b.missing(class, &va, now_ms, |r| {
            send(&mut out.b.sent, &mut left_b, &mut to_a, r, false)
        });
        for r in to_b {
            if let Ok(rx) = b.receive(r.clone(), now_ms) {
                note(&mut out.b, To::B, &r, rx, &mut took);
            }
        }
        for r in to_a {
            if let Ok(rx) = a.receive(r.clone(), now_ms) {
                note(&mut out.a, To::A, &r, rx, &mut took);
            }
        }
    }
    out.a.sent.count(&Sync::VersionsDone);
    out.b.sent.count(&Sync::VersionsDone);

    push(
        a,
        &ha,
        b,
        &hb,
        now_ms,
        &mut left_a,
        &mut out.a,
        &mut out.b,
        To::B,
        &mut took,
    );
    push(
        b,
        &hb,
        a,
        &ha,
        now_ms,
        &mut left_b,
        &mut out.b,
        &mut out.a,
        To::A,
        &mut took,
    );
    out
}

/// Pushes from `from` into `to`'s custody, then drops what `to` accepted from `from`'s outbox.
#[allow(clippy::too_many_arguments)]
fn push<A: Storage, B: Storage>(
    from: &mut Replica<A>,
    from_header: &Header,
    to: &mut Replica<B>,
    to_header: &Header,
    now_ms: u64,
    left: &mut u32,
    sender: &mut Side,
    receiver: &mut Side,
    side: To,
    took: &mut impl FnMut(To, &Record, Received),
) {
    let mut pushes: Vec<Record, MAX_PUSHES> = Vec::new();
    let mut held_back = false;
    from.pushes(to_header, now_ms, |r| {
        let more = send(&mut sender.sent, left, &mut pushes, r, true);
        held_back |= !more;
        more
    });
    sender.held_back |= held_back;
    let mut accepted: Vec<(Key, Version), MAX_PUSHES> = Vec::new();
    for r in pushes {
        match to.receive_push(from_header, r.clone(), now_ms) {
            Ok(rx) => {
                let _ = accepted.push((r.key(), r.version));
                note(receiver, side, &r, rx, took);
            }
            Err(_) => {
                receiver.refused += 1;
                break;
            }
        }
    }
    sender.sent.count(&Sync::Done);
    receiver.sent.count(&Sync::Accepted(accepted.len() as u16));
    from.pushed(accepted);
}

/// Counts `record`'s frame against the budget and queues it; `false` once the budget or the
/// queue is spent.
fn send<const N: usize>(
    sent: &mut Traffic,
    left: &mut u32,
    queue: &mut Vec<Record, N>,
    record: Record,
    push: bool,
) -> bool {
    let msg = if push { Sync::Push(record) } else { Sync::Record(record) };
    let len = frame_len(&msg);
    if len > *left || queue.is_full() {
        return false;
    }
    *left -= len;
    sent.add(&msg, len);
    let (Sync::Push(record) | Sync::Record(record)) = msg else {
        unreachable!()
    };
    let _ = queue.push(record);
    true
}

fn note(side: &mut Side, to: To, record: &Record, received: Received, took: &mut impl FnMut(To, &Record, Received)) {
    if received.kept() || received == Received::Relayed {
        side.took += 1;
        took(to, record, received);
    }
}

fn count_summary(sent: &mut Traffic, summary: &[Summary]) {
    for chunk in summary.chunks(SUMMARIES_PER_FRAME) {
        sent.count(&Sync::Summary(chunk.iter().copied().collect()));
    }
    sent.count(&Sync::SummaryDone);
}

fn count_versions(sent: &mut Traffic, class: crate::Class, versions: &[(Topic, Version)]) {
    for chunk in versions.chunks(VERSIONS_PER_FRAME) {
        sent.count(&Sync::Versions {
            class,
            versions: chunk.iter().copied().collect(),
        });
    }
    if versions.is_empty() {
        sent.count(&Sync::Versions {
            class,
            versions: Vec::new(),
        });
    }
}
