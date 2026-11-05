//! How the last run ended, when it crashed. A panic or a hard fault leaves a record in RAM that a
//! reset keeps, then restarts the device; the next boot reports it with the chip's reset reasons
//! (`firmware::run`, architecture §13).
//!
//! The record sits in `.uninit`, which neither this image nor the bootloader initialises: the
//! bootloader's statics and stack stay at the two ends of RAM, far from it. A power cut leaves
//! noise there, which the magic and the CRC reject.

use core::fmt::{self, Write};
use core::mem::{MaybeUninit, offset_of};
use core::panic::PanicInfo;

use cortex_m_rt::{ExceptionFrame, exception};
use embassy_nrf::pac;
use navgrid_proto::{CRASH_TEXT_LEN, Crash, CrashKind, crc32};

const MAGIC: u32 = 0x4e47_4352;

#[repr(C)]
#[derive(Clone, Copy)]
struct Record {
    magic: u32,
    /// 0 a panic, 1 a hard fault.
    kind: u32,
    line: u32,
    pc: u32,
    len: u32,
    text: [u8; CRASH_TEXT_LEN],
    crc: u32,
}

#[unsafe(link_section = ".uninit.navgrid_crash")]
static mut RECORD: MaybeUninit<Record> = MaybeUninit::uninit();

fn crc(record: &Record) -> u32 {
    // SAFETY: a `repr(C)` struct of integers, read up to its CRC.
    let bytes = unsafe { core::slice::from_raw_parts((record as *const Record).cast::<u8>(), offset_of!(Record, crc)) };
    crc32(bytes)
}

fn store(kind: CrashKind, line: u32, pc: u32, text: &str) {
    let mut record = Record {
        magic: MAGIC,
        kind: kind as u32,
        line,
        pc,
        len: text.len() as u32,
        text: [0; CRASH_TEXT_LEN],
        crc: 0,
    };
    record.text[..text.len()].copy_from_slice(text.as_bytes());
    record.crc = crc(&record);
    // SAFETY: written once, as the device goes down.
    unsafe { (&raw mut RECORD).cast::<Record>().write_volatile(record) };
}

/// The chip's reset reasons and the record the last run left, if any; clears both.
pub fn take() -> (u32, Option<Crash>) {
    let reasons = pac::RESET.resetreas().read();
    // Write-one-to-clear.
    pac::RESET.resetreas().write_value(reasons);
    // SAFETY: plain integers, whatever RAM held; checked below before use.
    let record = unsafe { (&raw const RECORD).cast::<Record>().read_volatile() };
    unsafe { (&raw mut RECORD).cast::<u32>().write_volatile(0) };
    let valid = record.magic == MAGIC && record.crc == crc(&record) && record.len as usize <= CRASH_TEXT_LEN;
    let crash = valid.then(|| {
        let text = core::str::from_utf8(&record.text[..record.len as usize]).unwrap_or("");
        Crash {
            kind: if record.kind == 1 {
                CrashKind::HardFault
            } else {
                CrashKind::Panic
            },
            line: record.line,
            pc: record.pc,
            text: text.try_into().unwrap_or_default(),
        }
    });
    (reasons.0, crash)
}

/// Keeps what fits.
struct Truncating(heapless::String<CRASH_TEXT_LEN>);

impl Write for Truncating {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.chars() {
            if self.0.push(c).is_err() {
                break;
            }
        }
        Ok(())
    }
}

/// Restarts the device on purpose, leaving a record that says why, so the next boot reports it
/// like a crash: for a radio that stopped answering (DEF-71).
pub fn restart(text: &str) -> ! {
    store(CrashKind::Panic, 0, 0, text);
    defmt::error!("restarting: {}", text);
    cortex_m::peripheral::SCB::sys_reset();
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let (file, line) = info.location().map_or(("?", 0), |l| (l.file(), l.line()));
    let file = file.rsplit('/').next().unwrap_or(file);
    let mut text = Truncating(heapless::String::new());
    let _ = write!(text, "{}: {}", file, info.message());
    store(CrashKind::Panic, line, 0, &text.0);
    defmt::error!("{}", defmt::Display2Format(info));
    cortex_m::peripheral::SCB::sys_reset();
}

/// `defmt::panic!` and `unwrap!` log their message, then end up here: the record says only that.
#[defmt::panic_handler]
fn defmt_panic() -> ! {
    core::panic!("defmt panic, see the log")
}

/// Test aid (`crash-test`): 20 s after boot, panics on even boots and runs an undefined
/// instruction on odd ones, so each boot reports the other kind of crash.
#[cfg(feature = "crash-test")]
#[embassy_executor::task]
pub async fn test(epoch: u32) {
    embassy_time::Timer::after_secs(20).await;
    if epoch.is_multiple_of(2) {
        panic!("crash test at boot {}", epoch);
    }
    cortex_m::asm::udf();
}

#[exception]
unsafe fn HardFault(frame: &ExceptionFrame) -> ! {
    store(CrashKind::HardFault, 0, frame.pc(), "");
    cortex_m::peripheral::SCB::sys_reset();
}
