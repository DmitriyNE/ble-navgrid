//! Mesh state (requirements FR-C, architecture §12): single-writer records, merged by version,
//! synced between neighbours, and carried toward sinks and back toward the nodes they concern.
//!
//! - **Records** ([`Record`]) have one writer, their origin, which numbers its writes
//!   ([`Version`]). A replica keeps the newest version of each [`Key`], so replicas that have
//!   seen the same writes agree, in whatever order they arrived (D-24).
//! - **Classes** ([`Class`]) decide who keeps a record: a node keeps its own, its 2-hop
//!   neighbourhood's, and the controller's records about them; sinks keep everything (D-25).
//! - **Sync** between neighbours compares one order-free hash per class both keep, then the
//!   versions in the classes that differ, then sends the records each side lacks ([`wire`]).
//! - **Routing** (D-26): records for sinks are pushed to a neighbour with a lower hop count,
//!   which takes custody of them. The controller's records for a node go back along the way that
//!   node's records came up.
//! - **No clock** (NFR-5): callers pass their local time in milliseconds, and records carry
//!   their age.

#![no_std]

#[cfg(feature = "alloc")]
extern crate alloc;
#[cfg(test)]
extern crate std;

pub mod auth;
mod driver;
mod record;
mod replica;
pub mod session;
mod table;
pub mod wire;

pub use driver::{Driver, Memory, Protocol};

/// The controller's node ID, which Desired records and the frame are written under, until
/// provisioning carries it (DEF-27).
pub const CONTROLLER: NodeId = NodeId(0xc0ff_ee00_0000);
pub use navgrid_proto::{NodeId, PointMm, Role};
pub use record::{
    Body, Class, Command, Desired, Environment, Frame, Health, Key, Link, MAX_NEIGHBOURS, NodeInfo, Position, Record,
    Reported, Setting, Status, Topic, Version, Writer,
};
pub use replica::{
    Config, Error, Limits, MAX_HOPS, MAX_NEIGHBOURHOOD, NEVER, Peer, Pending, Received, Replica, Route, SUMMARY_MEMORY,
    Stored, Toward, Ttl, UNREACHABLE,
};
#[cfg(feature = "alloc")]
pub use table::Heap;
pub use table::{Fixed, Storage, Table};
