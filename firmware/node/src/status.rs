//! What the node knows about itself, for the LED status display (architecture §13), its beacon
//! and its Health reports. Until the mesh state runs on devices (§12, stage 4) connectivity is
//! just which links are up.

// The DK has no `ui` module yet; its four LEDs will show the sink's status.
#![cfg_attr(not(feature = "board-tag"), allow(dead_code))]

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;

/// Encrypted links up.
static LINKS: AtomicU8 = AtomicU8::new(0);
/// Links up that lead to a sink.
static SINK_LINKS: AtomicU8 = AtomicU8::new(0);
/// Links the node can still take, as the mesh policy counts them (for the beacon, FR-M4).
static FREE_LINKS: AtomicU8 = AtomicU8::new(0);
/// Hops to the nearest sink, as the replica knows them (for the beacon); 255 with no path.
static HOPS: AtomicU8 = AtomicU8::new(u8::MAX);
/// Uptime in seconds when a link last failed to encrypt (another fleet key), 0 if never.
static FOREIGN_S: AtomicU32 = AtomicU32::new(0);
static BATTERY_MV: AtomicU16 = AtomicU16::new(0);
static ATTENTION_UNTIL_S: AtomicU32 = AtomicU32::new(0);
static SINK: AtomicBool = AtomicBool::new(false);
static ENROLLED: AtomicBool = AtomicBool::new(false);

/// What a beacon says changed since advertising started (attention, so far).
pub static BEACON_CHANGED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// How long a failed encryption keeps the display on "another fleet".
const FOREIGN_FOR_S: u32 = 60;
/// How long a button press keeps the beacon's attention flag up.
pub const ATTENTION_FOR_S: u32 = 30;

fn uptime_s() -> u32 {
    Instant::now().as_secs() as u32
}

pub fn set_identity(enrolled: bool, sink: bool) {
    ENROLLED.store(enrolled, Ordering::Relaxed);
    SINK.store(sink, Ordering::Relaxed);
}

/// An encrypted link, counted while this is alive.
pub struct Link {
    to_sink: bool,
}

impl Link {
    pub fn up(to_sink: bool) -> Self {
        LINKS.fetch_add(1, Ordering::Relaxed);
        if to_sink {
            SINK_LINKS.fetch_add(1, Ordering::Relaxed);
        }
        Self { to_sink }
    }
}

impl Drop for Link {
    fn drop(&mut self) {
        LINKS.fetch_sub(1, Ordering::Relaxed);
        if self.to_sink {
            SINK_LINKS.fetch_sub(1, Ordering::Relaxed);
        }
    }
}

pub fn links() -> u8 {
    LINKS.load(Ordering::Relaxed)
}

pub fn set_free_links(n: u8) {
    FREE_LINKS.store(n, Ordering::Relaxed);
}

pub fn free_links() -> u8 {
    FREE_LINKS.load(Ordering::Relaxed)
}

pub fn set_hops(hops: u8) {
    if HOPS.swap(hops, Ordering::Relaxed) != hops {
        BEACON_CHANGED.signal(());
    }
}

pub fn hops() -> u8 {
    HOPS.load(Ordering::Relaxed)
}

/// A link failed to encrypt or its peer turned out not to be in the fleet.
pub fn foreign() {
    FOREIGN_S.store(uptime_s().max(1), Ordering::Relaxed);
}

pub fn set_battery(mv: u16) {
    BATTERY_MV.store(mv, Ordering::Relaxed);
}

pub fn battery_mv() -> Option<u16> {
    Some(BATTERY_MV.load(Ordering::Relaxed)).filter(|&mv| mv != 0)
}

/// Raises the beacon's attention flag for [`ATTENTION_FOR_S`].
pub fn attention() {
    ATTENTION_UNTIL_S.store(uptime_s() + ATTENTION_FOR_S, Ordering::Relaxed);
    BEACON_CHANGED.signal(());
}

/// Seconds left on the attention flag.
pub fn attention_left_s() -> u32 {
    ATTENTION_UNTIL_S.load(Ordering::Relaxed).saturating_sub(uptime_s())
}

/// Can this node get data out, as the status display shows it.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Connectivity {
    /// A route to a sink this many hops long; 0 at a sink.
    Sink(u8),
    /// Peers, but no route to a sink.
    Mesh,
    /// Links fail to encrypt: the neighbours hold another fleet key.
    Foreign,
    Alone,
    Unenrolled,
}

pub fn connectivity() -> Connectivity {
    if !ENROLLED.load(Ordering::Relaxed) {
        return Connectivity::Unenrolled;
    }
    if SINK.load(Ordering::Relaxed) {
        return Connectivity::Sink(0);
    }
    let hops = hops();
    if hops != u8::MAX {
        return Connectivity::Sink(hops);
    }
    if SINK_LINKS.load(Ordering::Relaxed) > 0 {
        return Connectivity::Sink(1);
    }
    if links() > 0 {
        return Connectivity::Mesh;
    }
    let foreign = FOREIGN_S.load(Ordering::Relaxed);
    if foreign != 0 && uptime_s().saturating_sub(foreign) < FOREIGN_FOR_S {
        return Connectivity::Foreign;
    }
    Connectivity::Alone
}
