//! The node's state in a few numbers, kept where a reset leaves it (DEF-75): a node that drops
//! off the air without crashing loses its log to the first reset, so the firmware task saves this
//! snapshot every [`SAVE_EVERY`] into `.uninit`, and the next boot reports the last run's at
//! WARN, which reaches the host. The mesh's tasks mark their stages as they go.

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU32, Ordering};

use defmt::warn;
use embassy_time::{Duration, Instant};
use navgrid_proto::crc32;

pub const SAVE_EVERY: Duration = Duration::from_secs(30);

/// What each number is.
#[derive(Clone, Copy)]
#[repr(usize)]
pub enum F {
    /// The advertising loop: 1 waiting for a free link, 2 advertising, 3 handing a link over.
    AdvStage,
    AdvSince,
    /// The mesh driver: 1 ticking, 2 connecting, 3 handing a link over.
    DriveStage,
    DriveSince,
    /// The peer the driver is connecting to, low 32 bits.
    DrivePeer,
    /// The last scan's start; the last advertising report; the last event on any link.
    ScanAt,
    ReportAt,
    LinkEventAt,
    /// Slots serving a link, the policy's links and free links.
    SlotsBusy,
    Links,
    FreeLinks,
    /// The session slot taken since (0: free); a transfer being taken (1).
    SessionSince,
    Taking,
}

const N: usize = F::Taking as usize + 1;

static FIELDS: [AtomicU32; N] = [const { AtomicU32::new(0) }; N];

/// Seconds of uptime, as the fields keep times.
fn now_s() -> u32 {
    Instant::now().as_secs() as u32
}

pub fn set(f: F, v: u32) {
    FIELDS[f as usize].store(v, Ordering::Relaxed);
}

pub fn add(f: F, delta: i32) {
    if delta >= 0 {
        FIELDS[f as usize].fetch_add(delta as u32, Ordering::Relaxed);
    } else {
        FIELDS[f as usize].fetch_sub(delta.unsigned_abs(), Ordering::Relaxed);
    }
}

/// Marks `f` with the time now.
pub fn stamp(f: F) {
    set(f, now_s().max(1));
}

/// A stage and when it began.
pub fn stage(f: F, since: F, v: u32) {
    if FIELDS[f as usize].swap(v, Ordering::Relaxed) != v {
        stamp(since);
    }
}

const MAGIC: u32 = 0x4e47_4447;

#[repr(C)]
#[derive(Clone, Copy)]
struct Snapshot {
    magic: u32,
    uptime_s: u32,
    fields: [u32; N],
    crc: u32,
}

#[unsafe(link_section = ".uninit.navgrid_diag")]
static mut SNAPSHOT: MaybeUninit<Snapshot> = MaybeUninit::uninit();

fn crc(s: &Snapshot) -> u32 {
    // SAFETY: a `repr(C)` struct of integers, read up to its CRC.
    let bytes = unsafe {
        core::slice::from_raw_parts(
            (s as *const Snapshot).cast::<u8>(),
            core::mem::offset_of!(Snapshot, crc),
        )
    };
    crc32(bytes)
}

/// Saves the fields where a reset keeps them.
pub fn save() {
    let mut s = Snapshot {
        magic: MAGIC,
        uptime_s: now_s(),
        fields: core::array::from_fn(|i| FIELDS[i].load(Ordering::Relaxed)),
        crc: 0,
    };
    s.crc = crc(&s);
    // SAFETY: only the firmware task writes it, and boot reads it before that task starts.
    unsafe { (&raw mut SNAPSHOT).cast::<Snapshot>().write_volatile(s) };
}

/// Reports the last run's snapshot, if a reset kept one, and clears it. At boot, before the
/// firmware task starts saving.
pub fn report_last_run() {
    // SAFETY: as in `save`; a power cut leaves noise, which the magic and the CRC reject.
    let s = unsafe { (&raw const SNAPSHOT).cast::<Snapshot>().read_volatile() };
    unsafe {
        (&raw mut SNAPSHOT)
            .cast::<Snapshot>()
            .write_volatile(Snapshot { magic: 0, ..s })
    };
    if s.magic != MAGIC || s.crc != crc(&s) {
        return;
    }
    let f = |x: F| s.fields[x as usize];
    let ago = |x: F| {
        if f(x) == 0 {
            u32::MAX
        } else {
            s.uptime_s.saturating_sub(f(x))
        }
    };
    warn!(
        "last run at {=u32} s: advertising stage {=u32} for {=u32} s; driver stage {=u32} for {=u32} s (peer ..{=u32:08x}); scan {=u32} s ago, report {=u32} s ago, link event {=u32} s ago; slots busy {=u32}, links {=u32}, free {=u32}; session held {=u32} s, taking {=u32}",
        s.uptime_s,
        f(F::AdvStage),
        ago(F::AdvSince),
        f(F::DriveStage),
        ago(F::DriveSince),
        f(F::DrivePeer),
        ago(F::ScanAt),
        ago(F::ReportAt),
        ago(F::LinkEventAt),
        f(F::SlotsBusy),
        f(F::Links),
        f(F::FreeLinks),
        if f(F::SessionSince) == 0 {
            0
        } else {
            ago(F::SessionSince)
        },
        f(F::Taking),
    );
}
