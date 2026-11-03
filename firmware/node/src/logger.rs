//! The `defmt` global logger: every frame to RTT, as defmt-rtt does, and a copy of each one at or
//! above the device's report level to a small queue that `report::logs` sends toward the host
//! (architecture §13).
//!
//! The RTT half is defmt-rtt 1.3.1's default configuration (MIT OR Apache-2.0, the knurling-rs
//! developers), cut down to it: the control block, the channel and the encoder glue. defmt allows
//! one global logger, so the copy can't sit beside defmt-rtt; it has to be in it.
//!
//! A frame's level isn't in the frame. defmt's linker script places each level's strings in its
//! own range of the `.defmt` table, between marker symbols, and a frame starts with its string's
//! index: the logger compares the two.

use core::cell::{RefCell, UnsafeCell};
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use critical_section::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use heapless::{Deque, Vec};
use navgrid_proto::{LOG_FRAME_LEN, ReportLevel};

/// RTT buffer size: `DEFMT_RTT_BUFFER_SIZE` at build time, as with defmt-rtt, else 1 KiB.
const BUF_SIZE: usize = match option_env!("DEFMT_RTT_BUFFER_SIZE") {
    Some(s) => parse(s),
    None => 1024,
};

const fn parse(s: &str) -> usize {
    let b = s.as_bytes();
    let mut i = 0;
    let mut v = 0;
    while i < b.len() {
        v = v * 10 + (b[i] - b'0') as usize;
        i += 1;
    }
    v
}

/// Frames waiting to be sent; when full, newer ones stay in RTT only.
const QUEUED: usize = 8;

type Frame = Vec<u8, LOG_FRAME_LEN>;

static LEVEL: AtomicU8 = AtomicU8::new(ReportLevel::Error as u8);
static QUEUE: Mutex<RefCell<Deque<Frame, QUEUED>>> = Mutex::new(RefCell::new(Deque::new()));
/// Raised when a frame is queued.
pub static QUEUED_FRAME: Signal<CriticalSectionRawMutex, ()> = Signal::new();
/// Frames at the report level that didn't fit the queue or a [`Frame`].
pub static DROPPED: AtomicU32 = AtomicU32::new(0);

/// Sets the report level, from provisioning. Until then it's `Error`.
pub fn set_level(level: ReportLevel) {
    LEVEL.store(level as u8, Ordering::Relaxed);
}

/// The oldest queued frame.
pub fn take() -> Option<Frame> {
    critical_section::with(|cs| QUEUE.borrow_ref_mut(cs).pop_front())
}

unsafe extern "C" {
    static __DEFMT_MARKER_TRACE_START: u8;
    static __DEFMT_MARKER_DEBUG_START: u8;
    static __DEFMT_MARKER_INFO_START: u8;
    static __DEFMT_MARKER_WARN_START: u8;
    static __DEFMT_MARKER_ERROR_START: u8;
    static __DEFMT_MARKER_ERROR_END: u8;
}

/// The level of the string at `index` in the `.defmt` table, `None` for strings outside the
/// levels (`println!`, formats of arguments). The markers' addresses are table indices.
fn level_of(index: usize) -> Option<ReportLevel> {
    let at = |marker: *const u8| marker as usize;
    let bounds = [
        (at(&raw const __DEFMT_MARKER_TRACE_START), ReportLevel::Trace),
        (at(&raw const __DEFMT_MARKER_DEBUG_START), ReportLevel::Debug),
        (at(&raw const __DEFMT_MARKER_INFO_START), ReportLevel::Info),
        (at(&raw const __DEFMT_MARKER_WARN_START), ReportLevel::Warn),
        (at(&raw const __DEFMT_MARKER_ERROR_START), ReportLevel::Error),
    ];
    let end = at(&raw const __DEFMT_MARKER_ERROR_END);
    if index < bounds[0].0 || index >= end {
        return None;
    }
    bounds
        .iter()
        .rev()
        .find(|(start, _)| index >= *start)
        .map(|&(_, level)| level)
}

#[defmt::global_logger]
struct Logger;

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        ENCODER.acquire();
    }

    unsafe fn flush() {
        _SEGGER_RTT.up_channel.flush();
    }

    unsafe fn release() {
        unsafe { ENCODER.release() };
    }

    unsafe fn write(bytes: &[u8]) {
        unsafe { ENCODER.write(bytes) };
    }
}

static ENCODER: Encoder = Encoder::new();

struct Encoder {
    taken: AtomicBool,
    cs_restore: UnsafeCell<critical_section::RestoreState>,
    encoder: UnsafeCell<defmt::Encoder>,
    /// The raw frame, for the copy; `None` once it outgrew a [`Frame`].
    copy: UnsafeCell<Option<Frame>>,
}

unsafe impl Sync for Encoder {}

impl Encoder {
    const fn new() -> Self {
        Self {
            taken: AtomicBool::new(false),
            cs_restore: UnsafeCell::new(critical_section::RestoreState::invalid()),
            encoder: UnsafeCell::new(defmt::Encoder::new()),
            copy: UnsafeCell::new(None),
        }
    }

    fn acquire(&self) {
        // SAFETY: paired with the release in `release`.
        let restore = unsafe { critical_section::acquire() };
        if self.taken.load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly")
        }
        self.taken.store(true, Ordering::Relaxed);
        // SAFETY: the critical section is ours.
        unsafe {
            self.cs_restore.get().write(restore);
            *self.copy.get() = (LEVEL.load(Ordering::Relaxed) != ReportLevel::Off as u8).then(Vec::new);
            (*self.encoder.get()).start_frame(|b| _SEGGER_RTT.up_channel.write_all(b));
        }
    }

    unsafe fn write(&self, bytes: &[u8]) {
        // SAFETY: called between `acquire` and `release`, in their critical section.
        unsafe {
            let copy = &mut *self.copy.get();
            if let Some(frame) = copy
                && frame.extend_from_slice(bytes).is_err()
            {
                *copy = None;
                DROPPED.fetch_add(1, Ordering::Relaxed);
            }
            (*self.encoder.get()).write(bytes, |b| _SEGGER_RTT.up_channel.write_all(b));
        }
    }

    unsafe fn release(&self) {
        if !self.taken.load(Ordering::Relaxed) {
            panic!("defmt release out of context")
        }
        // SAFETY: as in `write`.
        unsafe {
            (*self.encoder.get()).end_frame(|b| _SEGGER_RTT.up_channel.write_all(b));
            if let Some(frame) = (*self.copy.get()).take() {
                self.forward(frame);
            }
            let restore = self.cs_restore.get().read();
            self.taken.store(false, Ordering::Relaxed);
            critical_section::release(restore);
        }
    }

    /// Queues `frame` if its level is at or above the report level.
    fn forward(&self, frame: Frame) {
        let Some(index) = frame.get(..2).map(|b| u16::from_le_bytes([b[0], b[1]]) as usize) else {
            return;
        };
        let wanted = level_of(index).is_some_and(|level| level as u8 <= LEVEL.load(Ordering::Relaxed));
        if !wanted {
            return;
        }
        // SAFETY: inside the logger's critical section.
        let cs = unsafe { critical_section::CriticalSection::new() };
        if QUEUE.borrow_ref_mut(cs).push_back(frame).is_err() {
            DROPPED.fetch_add(1, Ordering::Relaxed);
        } else {
            QUEUED_FRAME.signal(());
        }
    }
}

// RTT, from defmt-rtt.

const MODE_MASK: u32 = 0b11;
/// Block the application if the RTT buffer is full; probe-rs sets this while it reads.
const MODE_BLOCK_IF_FULL: u32 = 2;
/// Don't block if the RTT buffer is full; drop what doesn't fit.
const MODE_NON_BLOCKING_TRIM: u32 = 1;

#[repr(C)]
struct Header {
    id: [u8; 16],
    max_up_channels: u32,
    max_down_channels: u32,
    up_channel: Channel,
}

unsafe impl Sync for Header {}

#[unsafe(no_mangle)]
static _SEGGER_RTT: Header = Header {
    id: *b"SEGGER RTT\0\0\0\0\0\0",
    max_up_channels: 1,
    max_down_channels: 0,
    up_channel: Channel {
        name: NAME.as_ptr(),
        buffer: BUFFER.get(),
        size: BUF_SIZE as u32,
        write: AtomicU32::new(0),
        read: AtomicU32::new(0),
        flags: AtomicU32::new(MODE_NON_BLOCKING_TRIM),
    },
};

#[unsafe(link_section = ".uninit.defmt-rtt.BUFFER")]
static BUFFER: Buffer = Buffer(UnsafeCell::new([0; BUF_SIZE]));

/// In RAM, so the whole control block can be read from RAM.
#[unsafe(link_section = ".data.defmt-rtt.NAME")]
static NAME: [u8; 6] = *b"defmt\0";

struct Buffer(UnsafeCell<[u8; BUF_SIZE]>);

impl Buffer {
    const fn get(&self) -> *mut u8 {
        self.0.get() as _
    }
}

unsafe impl Sync for Buffer {}

#[repr(C)]
struct Channel {
    name: *const u8,
    buffer: *mut u8,
    size: u32,
    write: AtomicU32,
    read: AtomicU32,
    flags: AtomicU32,
}

impl Channel {
    fn write_all(&self, mut bytes: &[u8]) {
        let write = if self.host_is_connected() {
            Self::blocking_write
        } else {
            Self::nonblocking_write
        };
        while !bytes.is_empty() {
            let consumed = write(self, bytes);
            if consumed != 0 {
                bytes = &bytes[consumed..];
            }
        }
    }

    fn blocking_write(&self, bytes: &[u8]) -> usize {
        let read = self.read.load(Ordering::Relaxed) as usize;
        let write = self.write.load(Ordering::Acquire) as usize;
        let available = available_buffer_size(read, write);
        if available == 0 {
            return 0;
        }
        self.write_impl(bytes, write, available)
    }

    fn nonblocking_write(&self, bytes: &[u8]) -> usize {
        let write = self.write.load(Ordering::Acquire) as usize;
        // Overwrites what the host hasn't read: with no host there's nothing to lose.
        self.write_impl(bytes, write, BUF_SIZE)
    }

    fn write_impl(&self, bytes: &[u8], cursor: usize, available: usize) -> usize {
        let len = bytes.len().min(available);
        // SAFETY: the buffer is BUF_SIZE long; the copy wraps at its end.
        unsafe {
            if cursor + len > BUF_SIZE {
                let pivot = BUF_SIZE - cursor;
                ptr::copy_nonoverlapping(bytes.as_ptr(), self.buffer.add(cursor), pivot);
                ptr::copy_nonoverlapping(bytes.as_ptr().add(pivot), self.buffer, len - pivot);
            } else {
                ptr::copy_nonoverlapping(bytes.as_ptr(), self.buffer.add(cursor), len);
            }
        }
        self.write
            .store((cursor.wrapping_add(len) % BUF_SIZE) as u32, Ordering::Release);
        len
    }

    fn flush(&self) {
        if !self.host_is_connected() {
            return;
        }
        while self.read.load(Ordering::Relaxed) != self.write.load(Ordering::Relaxed) {
            core::hint::spin_loop();
        }
    }

    fn host_is_connected(&self) -> bool {
        self.flags.load(Ordering::Relaxed) & MODE_MASK == MODE_BLOCK_IF_FULL
    }
}

fn available_buffer_size(read: usize, write: usize) -> usize {
    if read > write {
        read - write - 1
    } else {
        BUF_SIZE - write - 1 + read
    }
}
