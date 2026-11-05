//! Sync sessions on a mesh link (architecture §12). Each message is one postcard + COBS frame,
//! one L2CAP SDU on [`SYNC_SPSM`].
//!
//! Both sides run the same steps:
//!
//! 1. [`Sync::Hello`] with their [`Header`]. Each then calls
//!    [`Replica::contact`](crate::Replica::contact) with the other's.
//! 2. Their [`Summary`]: a hash per class they keep, in `Summary` batches ending with
//!    `SummaryDone`. A sink keeps everything, so it answers the peer's summary with the same
//!    classes instead of listing its own.
//! 3. For each class both listed with different hashes ([`differing`]): `Versions`, the topics
//!    and versions held, ending with `VersionsDone`.
//! 4. The records the other side lacks in those classes (`Record`), then the records pushed into
//!    its custody (`Push`), then `Done`.
//! 5. `Accepted`: how many of the other side's pushes it took. The sender drops those from its
//!    outbox ([`Replica::pushed`](crate::Replica::pushed)).

use heapless::Vec;
use navgrid_proto::NodeId;
use serde::{Deserialize, Serialize};

use crate::{Class, Record, Role, Topic, Version};

/// L2CAP LE credit-based channel for sync sessions, next to telemetry's `0x0081`.
pub const SYNC_SPSM: u16 = 0x0082;

/// Upper bound for one encoded frame, terminator included. Fits one 251-byte LL payload after
/// the L2CAP headers.
pub const MAX_FRAME_LEN: usize = 240;

pub const SUMMARIES_PER_FRAME: usize = 16;
pub const VERSIONS_PER_FRAME: usize = 12;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Header {
    pub node: NodeId,
    pub role: Role,
    pub hops: u8,
    pub sink: bool,
    /// The sender remembers the summaries of its last complete session with the receiver, and
    /// will send only the classes that changed if the receiver does too (DEF-28).
    pub delta: bool,
}

/// A class one side keeps, and an order-free hash of the keys and versions it holds in it.
/// Packed to 11 bytes, the class as a kind byte and a 48-bit node ID: a node holds a few hundred
/// of these in its session driver and its memory of each peer's last session.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Summary {
    kind: u8,
    node: [u8; 6],
    pub hash: [u8; 4],
}

impl Summary {
    pub fn new(class: Class, hash: [u8; 4]) -> Self {
        let (kind, node) = match class {
            Class::Global => (0, NodeId(0)),
            Class::Desired(n) => (1, n),
            Class::About(n) => (2, n),
            Class::Near(n) => (3, n),
            Class::Up(n) => (4, n),
        };
        debug_assert!(node.0 >> 48 == 0, "node IDs are 48-bit");
        let b = node.0.to_le_bytes();
        Self {
            kind,
            node: [b[0], b[1], b[2], b[3], b[4], b[5]],
            hash,
        }
    }

    pub fn class(&self) -> Class {
        let b = self.node;
        let node = NodeId(u64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], 0, 0]));
        match self.kind {
            1 => Class::Desired(node),
            2 => Class::About(node),
            3 => Class::Near(node),
            4 => Class::Up(node),
            _ => Class::Global,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Sync {
    Hello(Header),
    Summary(Vec<Summary, SUMMARIES_PER_FRAME>),
    SummaryDone,
    Versions {
        class: Class,
        versions: Vec<(Topic, Version), VERSIONS_PER_FRAME>,
    },
    VersionsDone,
    Record(Record),
    Push(Record),
    Done,
    Accepted(u16),
}

pub type Error = postcard::Error;

/// Encodes `msg` as one COBS frame including the `0x00` terminator; returns the used part of `buf`.
pub fn encode<'a>(msg: &Sync, buf: &'a mut [u8]) -> Result<&'a mut [u8], Error> {
    postcard::to_slice_cobs(msg, buf)
}

/// Decodes one COBS frame in place. `frame` excludes the `0x00` terminator.
pub fn decode(frame: &mut [u8]) -> Result<Sync, Error> {
    postcard::from_bytes_cobs(frame)
}

/// Whether an encoded frame is a [`Sync::Hello`], without decoding it: the hello is the first
/// variant, so its tag byte is zero, which COBS writes as a leading code byte of 1.
pub fn is_hello(frame: &[u8]) -> bool {
    frame.first() == Some(&1)
}

/// The entries a side sends when both remember the last exchange: the classes of `full` whose
/// hash differs from `memory` or that `memory` lacks, and the classes `memory` has that `full`
/// no longer does, with an all-zero hash. Both lists must be in class order.
pub fn delta<const N: usize>(full: &[Summary], memory: &[Summary], out: &mut Vec<Summary, N>) {
    let (mut i, mut j) = (0, 0);
    while i < full.len() || j < memory.len() {
        let push = match (full.get(i), memory.get(j)) {
            (Some(f), Some(m)) if f.class() == m.class() => {
                i += 1;
                j += 1;
                (f.hash != m.hash).then_some(*f)
            }
            (Some(f), Some(m)) if f.class() < m.class() => {
                i += 1;
                Some(*f)
            }
            (Some(f), None) => {
                i += 1;
                Some(*f)
            }
            (_, Some(m)) => {
                j += 1;
                Some(Summary::new(m.class(), [0; 4]))
            }
            (None, None) => break,
        };
        if let Some(s) = push
            && out.push(s).is_err()
        {
            return;
        }
    }
}

/// The peer's full summary from what was remembered and the entries it sent: entries replace or
/// add; an all-zero hash drops the class. In class order.
pub fn reconstruct<const N: usize>(memory: &[Summary], entries: &[Summary], out: &mut Vec<Summary, N>) {
    let (mut i, mut j) = (0, 0);
    while i < memory.len() || j < entries.len() {
        let next = match (memory.get(i), entries.get(j)) {
            (Some(m), Some(e)) if m.class() == e.class() => {
                i += 1;
                j += 1;
                (e.hash != [0; 4]).then_some(*e)
            }
            (Some(m), Some(e)) if m.class() < e.class() => {
                i += 1;
                Some(*m)
            }
            (Some(m), None) => {
                i += 1;
                Some(*m)
            }
            (_, Some(e)) => {
                j += 1;
                (e.hash != [0; 4]).then_some(*e)
            }
            (None, None) => break,
        };
        if let Some(s) = next
            && out.push(s).is_err()
        {
            return;
        }
    }
}

/// The classes in both summaries whose hashes differ, in class order. Both summaries must be in
/// class order, as [`Replica::summary`](crate::Replica::summary) makes them.
pub fn differing<'a>(a: &'a [Summary], b: &'a [Summary]) -> impl Iterator<Item = Class> + 'a {
    let (mut i, mut j) = (0, 0);
    core::iter::from_fn(move || {
        while i < a.len() && j < b.len() {
            let (x, y) = (&a[i], &b[j]);
            match x.class().cmp(&y.class()) {
                core::cmp::Ordering::Less => i += 1,
                core::cmp::Ordering::Greater => j += 1,
                core::cmp::Ordering::Equal => {
                    i += 1;
                    j += 1;
                    if x.hash != y.hash {
                        return Some(x.class());
                    }
                }
            }
        }
        None
    })
}

#[cfg(test)]
mod tests {
    #[test]
    fn a_hello_frame_is_told_by_its_first_byte() {
        let mut buf = [0u8; 64];
        let hello = Sync::Hello(Header {
            node: ID,
            role: navgrid_proto::Role::Anchor,
            hops: 1,
            sink: false,
            delta: false,
        });
        assert!(is_hello(encode(&hello, &mut buf).unwrap()));
        assert!(!is_hello(encode(&Sync::SummaryDone, &mut buf).unwrap()));
        assert!(!is_hello(&[]));
    }

    use navgrid_proto::{Firmware, PointMm, Role};

    use super::*;
    use crate::*;

    /// The largest node ID: device addresses are 48 bits.
    const ID: NodeId = NodeId(0xffff_ffff_ffff);
    const V: Version = Version {
        epoch: u32::MAX,
        counter: u32::MAX,
    };

    fn record(body: Body) -> Record {
        Record {
            origin: ID,
            version: V,
            age_ms: u32::MAX,
            body,
        }
    }

    fn worst_case() -> [Sync; 7] {
        let summary = Summary::new(Class::Near(ID), [0xff; 4]);
        let mut summaries = Vec::new();
        while summaries.push(summary).is_ok() {}
        let mut versions = Vec::new();
        while versions.push((Topic::Desired(ID, Setting::Advertising), V)).is_ok() {}
        let mut neighbours = Vec::new();
        while neighbours.push(ID).is_ok() {}
        [
            Sync::Hello(Header {
                node: ID,
                role: Role::Anchor,
                hops: u8::MAX,
                sink: true,
                delta: true,
            }),
            Sync::Summary(summaries),
            Sync::Versions {
                class: Class::Desired(ID),
                versions,
            },
            Sync::Record(record(Body::Node(NodeInfo {
                role: Role::Mobile,
                number: Some(u16::MAX),
                firmware: Firmware::new(u8::MAX, u8::MAX, u8::MAX, u32::MAX, true),
                config: u32::MAX,
                hops: u8::MAX,
                frame: u16::MAX,
                pinned: Some(PointMm {
                    x: i32::MIN,
                    y: i32::MIN,
                    z: i32::MIN,
                }),
                neighbours,
            }))),
            Sync::Push(record(Body::Position(Position {
                of: ID,
                point: PointMm {
                    x: i32::MIN,
                    y: i32::MIN,
                    z: i32::MIN,
                },
                sigma_mm: [u32::MAX; 3],
                frame: u16::MAX,
            }))),
            Sync::Record(record(Body::Environment(Environment {
                temperature_cdeg: i16::MIN,
                humidity_cpct: u16::MAX,
                pressure_cpa: u32::MAX,
                gas_ohm: Some(u32::MAX),
                samples: u8::MAX,
                height: true,
            }))),
            Sync::Accepted(u16::MAX),
        ]
    }

    #[test]
    fn worst_case_roundtrips_within_max_frame() {
        for msg in worst_case() {
            let mut buf = [0u8; MAX_FRAME_LEN];
            let frame = encode(&msg, &mut buf).unwrap_or_else(|e| panic!("{msg:?}: {e}"));
            let (terminator, body) = frame.split_last_mut().unwrap();
            assert_eq!(*terminator, 0);
            assert_eq!(decode(body).unwrap(), msg);
        }
    }

    #[test]
    fn differing_skips_classes_only_one_side_lists() {
        let s = |class, h| Summary::new(class, [h; 4]);
        let a = [
            s(Class::Global, 1),
            s(Class::Near(NodeId(1)), 2),
            s(Class::Near(NodeId(3)), 3),
        ];
        let b = [
            s(Class::Global, 1),
            s(Class::Near(NodeId(2)), 9),
            s(Class::Near(NodeId(3)), 4),
        ];
        assert!(differing(&a, &b).eq([Class::Near(NodeId(3))]));
    }
}
