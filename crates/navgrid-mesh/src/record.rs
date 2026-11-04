//! Records: what the mesh state is made of. Lengths are integer millimetres and times
//! milliseconds, as in `navgrid-proto`.

use heapless::Vec;
use navgrid_proto::{Firmware, NodeId, PointMm, Role};
use serde::{Deserialize, Serialize};

/// Neighbours a node lists in its [`NodeInfo`]: its mesh-link budget (architecture §5).
pub const MAX_NEIGHBOURS: usize = 8;

/// The order of one origin's writes: the boot epoch, which the origin persists and bumps at every
/// boot, then a counter from 1. Later writes compare greater.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Version {
    pub epoch: u32,
    pub counter: u32,
}

/// Numbers one origin's writes.
#[derive(Clone, Debug)]
pub struct Writer {
    origin: NodeId,
    last: Version,
}

impl Writer {
    /// `epoch` must be greater than at any earlier boot of `origin`.
    pub fn new(origin: NodeId, epoch: u32) -> Self {
        Self {
            origin,
            last: Version { epoch, counter: 0 },
        }
    }

    pub fn origin(&self) -> NodeId {
        self.origin
    }

    /// The version the next record will get.
    pub fn next(&self) -> Version {
        Version {
            counter: self.last.counter + 1,
            ..self.last
        }
    }

    /// A new record from the origin, newer than every earlier one.
    pub fn record(&mut self, body: Body) -> Record {
        self.last = self.next();
        Record {
            origin: self.origin,
            version: self.last,
            age_ms: 0,
            body,
        }
    }
}

/// One setting of a node that the controller can command.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Setting {
    Blink,
    Environment,
    Diversity,
    Advertising,
}

/// What a record is about, among its origin's records. With the origin it forms the [`Key`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Topic {
    Node,
    Health,
    Environment,
    /// A link, by its peer.
    Link(NodeId),
    /// A position, by the node it places.
    Position(NodeId),
    /// The controller's desired state for one setting of a node.
    Desired(NodeId, Setting),
    /// What a node applied of its desired state for one setting.
    Reported(Setting),
    Frame,
    /// The controller retired a node: its records go everywhere, and nothing more of it is kept.
    Retired(NodeId),
}

impl Topic {
    /// The topic as numbers, for hashing.
    pub(crate) fn parts(&self) -> (u8, u64, u8) {
        match *self {
            Self::Node => (0, 0, 0),
            Self::Health => (1, 0, 0),
            Self::Environment => (2, 0, 0),
            Self::Link(peer) => (3, peer.0, 0),
            Self::Position(of) => (4, of.0, 0),
            Self::Desired(target, setting) => (5, target.0, setting as u8),
            Self::Reported(setting) => (6, 0, setting as u8),
            Self::Frame => (7, 0, 0),
            Self::Retired(node) => (8, node.0, 0),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Key {
    pub origin: NodeId,
    pub topic: Topic,
}

impl Key {
    /// The class of a record with this key, or `None` if its origin may not write it. Only the
    /// controller writes desired state, the frame and other nodes' positions, and it writes
    /// nothing else.
    pub fn class(&self, controller: NodeId) -> Option<Class> {
        let origin = self.origin;
        match (origin == controller, self.topic) {
            (true, Topic::Frame | Topic::Retired(_)) => Some(Class::Global),
            (true, Topic::Desired(target, _)) => Some(Class::Desired(target)),
            (true, Topic::Position(of)) => Some(Class::About(of)),
            (true, _) => None,
            (false, Topic::Node | Topic::Environment | Topic::Link(_)) => Some(Class::Near(origin)),
            (false, Topic::Position(of)) if of == origin => Some(Class::Near(origin)),
            (false, Topic::Health | Topic::Reported(_)) => Some(Class::Up(origin)),
            (false, _) => None,
        }
    }
}

/// Who keeps a record. Every record is in one class, and a replica keeps a class whole or not
/// at all. Variant order is sync priority.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Class {
    /// The controller's mesh-wide records (the frame): kept everywhere.
    Global,
    /// The controller's desired state for a node: kept by the node and its peers. On the way
    /// there it is pushed along the reverse path.
    Desired(NodeId),
    /// The controller's records about a node (its position): kept by the node's neighbourhood,
    /// and pushed there like desired state.
    About(NodeId),
    /// A node's own records for its neighbourhood: node info, links, environment, and its
    /// position once it solves its own. Kept by its 2-hop neighbourhood, and pushed to sinks.
    Near(NodeId),
    /// A node's own records for sinks only: health and reported state. Only pushed.
    Up(NodeId),
}

impl Class {
    /// Records of the class travel toward sinks.
    pub fn upward(self) -> bool {
        matches!(self, Self::Near(_) | Self::Up(_))
    }

    /// The node the class is about.
    pub fn subject(self) -> Option<NodeId> {
        match self {
            Self::Global => None,
            Self::Desired(n) | Self::About(n) | Self::Near(n) | Self::Up(n) => Some(n),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Record {
    pub origin: NodeId,
    pub version: Version,
    /// Time since the origin wrote it, as of sending. Every holder adds the time it held it, so
    /// no clock sync is needed (NFR-5).
    pub age_ms: u32,
    pub body: Body,
}

impl Record {
    pub fn key(&self) -> Key {
        Key {
            origin: self.origin,
            topic: self.body.topic(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Body {
    Node(NodeInfo),
    Health(Health),
    Environment(Environment),
    Link(Link),
    Position(Position),
    Desired(Desired),
    Reported(Reported),
    Frame(Frame),
    Retired(Retired),
}

/// The controller retired a node (FR-O4): every replica drops the node's records and takes no
/// more from it; the node itself stops. Kept for good, so a retired node that comes back is
/// refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Retired {
    pub node: NodeId,
    /// The controller's MAC over origin, version and node ([`auth`](crate::auth)).
    pub mac: [u8; 8],
}

impl Body {
    pub fn topic(&self) -> Topic {
        match self {
            Self::Node(_) => Topic::Node,
            Self::Health(_) => Topic::Health,
            Self::Environment(_) => Topic::Environment,
            Self::Link(l) => Topic::Link(l.peer),
            Self::Position(p) => Topic::Position(p.of),
            Self::Desired(d) => Topic::Desired(d.target, d.command.setting()),
            Self::Reported(r) => Topic::Reported(r.setting),
            Self::Frame(_) => Topic::Frame,
            Self::Retired(r) => Topic::Retired(r.node),
        }
    }
}

/// A node's description of itself. Its neighbour list gives the others their 2-hop
/// neighbourhood.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NodeInfo {
    pub role: Role,
    /// Its fleet number (`N042`, D-31), so the host's registry can be rebuilt from the mesh.
    pub number: Option<u16>,
    pub firmware: Firmware,
    /// Hash of the node's `NodeConfig`, so the controller sees what it runs.
    pub config: u32,
    /// Hops to the nearest sink: 0 at a sink, [`UNREACHABLE`](crate::UNREACHABLE) with no path.
    pub hops: u8,
    /// The frame version the node has seen (FR-A5).
    pub frame: u16,
    /// Surveyed coordinates, if the node is a pin (FR-A3a): the central solver's fixed points.
    pub pinned: Option<PointMm>,
    /// The mesh neighbours it syncs with, ascending.
    pub neighbours: Vec<NodeId, MAX_NEIGHBOURS>,
}

/// Device health (FR-S2).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Health {
    pub battery_mv: u16,
    /// Die temperature, 0.01 °C.
    pub temperature_cdeg: i16,
    pub uptime_s: u32,
    /// Records waiting in the outbox.
    pub queued: u16,
    /// Radio work since boot, for the energy accounting (NFR-6): ranging bursts run as initiator,
    /// CS procedures in them, and sync sessions.
    pub bursts: u16,
    pub procedures: u16,
    pub sessions: u16,
}

/// BME688 sample (FR-S3). Neighbours keep it: pressure is their barometric height reference.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Environment {
    /// 0.01 °C.
    pub temperature_cdeg: i16,
    /// 0.01 %RH.
    pub humidity_cpct: u16,
    /// 0.01 Pa.
    pub pressure_cpa: u32,
    pub gas_ohm: Option<u32>,
}

/// A mesh link, written by its initiator after each ranging burst.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Link {
    pub peer: NodeId,
    /// Over the burst: the median distance and its 1-sigma uncertainty.
    pub distance_mm: u32,
    pub sigma_mm: u32,
    /// 0 = unusable, 255 = best, as in `navgrid_proto::Range`.
    pub quality: u8,
    pub antenna_paths: u8,
    pub procedures: u16,
    pub failures: u16,
}

/// A node's position: written by the controller until nodes solve their own (D-28).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Position {
    pub of: NodeId,
    pub point: PointMm,
    /// 1-sigma per axis.
    pub sigma_mm: [u32; 3],
    /// The frame version it was solved in (FR-A5).
    pub frame: u16,
}

/// What the controller wants of a node (D-27). A newer version replaces the older one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Desired {
    pub target: NodeId,
    pub command: Command,
    /// The controller's MAC over origin, version, target and command ([`auth`](crate::auth)),
    /// so only the controller's commands are acted on (DEF-27).
    pub mac: [u8; 8],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    /// Blink the LED. An action: each new version of the record blinks once more.
    Blink { times: u8 },
    /// BME688 sampling period, 0 = off, and gas sampling (FR-S3).
    Environment { period_s: u16, gas: bool },
    /// Antenna diversity (FR-R7).
    Diversity(bool),
    /// Advertising interval (FR-M4).
    Advertising { interval_ms: u16 },
}

impl Command {
    pub fn setting(&self) -> Setting {
        match self {
            Self::Blink { .. } => Setting::Blink,
            Self::Environment { .. } => Setting::Environment,
            Self::Diversity(_) => Setting::Diversity,
            Self::Advertising { .. } => Setting::Advertising,
        }
    }
}

/// What a node did with the desired state for one setting.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Reported {
    pub setting: Setting,
    /// The version of the desired-state record it acted on.
    pub applied: Version,
    pub status: Status,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Status {
    Applied,
    /// The node can't do it, e.g. no BME688.
    Unsupported,
    /// Out of range for the node.
    Invalid,
}

/// The mesh→world transform (FR-A5), as `navgrid_solver::frame::Frame`: mirror, rotate, then
/// translate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Frame {
    pub version: u16,
    /// Microradians.
    pub yaw_urad: i32,
    pub mirrored: bool,
    pub offset: PointMm,
}
