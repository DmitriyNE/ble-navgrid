//! This image under the bootloader (architecture §13, D-34, D-35).
//!
//! - **Staged updates:** at boot, an update waiting in the update slot is checked against the
//!   provisioned signing key and handed to the bootloader ([`apply_staged`]).
//! - **Watchdog:** the image takes over the one the bootloader started and feeds it, so a hung
//!   image resets.
//! - **Confirming:** a freshly swapped-in image confirms itself once the node is back in the mesh
//!   (a fleet link up). One that isn't within 5 min restarts unconfirmed, and the bootloader puts
//!   the previous image back.
//! - **Passing it on:** the header of an installed update is kept ([`servable`]), so the node can
//!   offer its image to neighbours (`update`) once it has confirmed it. After a revert the kept
//!   header names the build that failed, and the node declines it ([`reverted_from`]). A staged update for another board isn't installed:
//!   a sink carries it for the nodes ([`carried`]).
//!
//! After boot the RRAM belongs to MPSL's timeslot driver ([`share_rram`]), shared by this and the
//! update receiver. Its writes run in timeslots, clear of the radio. Written directly, a 4 KiB page
//! takes tens of milliseconds, code fetched from RRAM waits meanwhile, and the SoftDevice
//! Controller asserts (DEF-38).

use core::cell::RefCell;

use defmt::{info, warn};
use embassy_boot::{FirmwareState, State};
use embassy_boot_nrf::{BlockingFirmwareUpdater, FirmwareUpdaterConfig};
use embassy_embedded_hal::flash::partition::Partition;
use embassy_nrf::Peri;
use embassy_nrf::peripherals::WDT0;
use embassy_nrf::rramc::Nvmc;
use embassy_nrf::wdt::{self, Watchdog};
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_storage::nor_flash::NorFlash;
use navgrid_proto::layout::{
    ACTIVE, ACTIVE_HEADER, ACTIVE_HEADER_LEN, BOOTLOADER_STATE, BOOTLOADER_STATE_LEN, DFU, STAGED_UPDATE,
    STAGED_UPDATE_LEN,
};
use navgrid_proto::{Firmware, ProvisioningError, UPDATE_HEADER_LEN, UpdateHeader};
use nrf_sdc::mpsl::Flash;
use static_cell::StaticCell;

use crate::board::BOARD;
use crate::status;

/// The RRAM once boot is done.
pub type Rram = Mutex<ThreadModeRawMutex, Flash<'static>>;

/// Hands the RRAM over to MPSL's timeslot driver, after the boot-time writes.
pub fn share_rram(flash: Flash<'static>) -> &'static Rram {
    static RRAM: StaticCell<Rram> = StaticCell::new();
    RRAM.init(Mutex::new(flash))
}

/// The bootloader's state, through the shared RRAM.
fn bootloader_state<'a>(
    rram: &'static Rram,
    aligned: &'a mut [u8; 16],
) -> FirmwareState<'a, Partition<'static, ThreadModeRawMutex, Flash<'static>>> {
    FirmwareState::new(Partition::new(rram, BOOTLOADER_STATE, BOOTLOADER_STATE_LEN), aligned)
}

/// What this node can offer its neighbours, and where the image lies: an update it carries for
/// another board, or else the update it was installed from, if it still runs that build.
pub fn servable() -> Option<(UpdateHeader, u32)> {
    if let Some(header) = carried() {
        return Some((header, DFU));
    }
    header_at(ACTIVE_HEADER)
        .filter(|h| h.firmware == crate::FIRMWARE && h.board == BOARD)
        .map(|h| (h, ACTIVE))
}

/// The build this node last reverted from: the kept header still names it, as nothing has
/// installed since. Offers of it are declined, or a bad build would come back at every offer.
pub fn reverted_from() -> Option<Firmware> {
    header_at(ACTIVE_HEADER)
        .filter(|h| h.board == BOARD && h.firmware != crate::FIRMWARE)
        .map(|h| h.firmware)
}

/// An update for another board, staged in the update slot to pass on (a sink's, from the host).
pub fn carried() -> Option<UpdateHeader> {
    header_at(STAGED_UPDATE).filter(|h| h.board != BOARD)
}

fn header_at(at: u32) -> Option<UpdateHeader> {
    // SAFETY: memory-mapped RRAM.
    let page = unsafe { core::slice::from_raw_parts(at as *const u8, UPDATE_HEADER_LEN) };
    UpdateHeader::decode(page).ok()
}

/// Whether the running image is confirmed. Until it is, the update slot holds the image a revert
/// would put back, so nothing may write it.
pub async fn confirmed(rram: &'static Rram) -> bool {
    let mut aligned = [0u8; 16];
    matches!(bootloader_state(rram, &mut aligned).get_state().await, Ok(State::Boot))
}

const FEED_EVERY: Duration = Duration::from_secs(1);
const CONFIRM_WITHIN: Duration = Duration::from_secs(5 * 60);

#[embassy_executor::task]
pub async fn run(wdt: Peri<'static, WDT0>, rram: &'static Rram) -> ! {
    // Running only if the bootloader started it.
    let mut handle = match wdt::Config::try_new(&wdt).map(|config| Watchdog::try_new::<_, 1>(wdt, config)) {
        Some(Ok((_wdt, [handle]))) => Some(handle),
        Some(Err(_)) => {
            warn!("firmware: can't take over the watchdog");
            None
        }
        None => {
            warn!("firmware: no watchdog running: started without the bootloader?");
            None
        }
    };
    let mut aligned = [0u8; 16];
    let mut state = bootloader_state(rram, &mut aligned);
    let mut unconfirmed = match state.get_state().await {
        Ok(State::Swap) => {
            info!("firmware: new image, unconfirmed until a fleet link is up");
            true
        }
        Ok(State::Revert) => {
            // This image ran confirmed before the update, and updates wait for a confirmed one.
            match reverted_from() {
                Some(failed) => warn!("firmware: reverted from {}", failed),
                None => warn!("firmware: reverted to the previous image"),
            }
            if let Err(e) = state.mark_booted().await {
                warn!("firmware: confirm: {:?}", e);
            }
            false
        }
        Ok(_) => false,
        Err(e) => {
            warn!("firmware: bootloader state: {:?}", e);
            false
        }
    };
    let started = Instant::now();
    loop {
        if let Some(h) = handle.as_mut() {
            h.pet();
        }
        if unconfirmed {
            // A test image may hold off, so the bootloader reverts it.
            if status::links() > 0 && !cfg!(feature = "never-confirm") {
                let t = Instant::now();
                match state.mark_booted().await {
                    Ok(()) => info!(
                        "firmware: image confirmed ({=u64} ms to write)",
                        t.elapsed().as_millis()
                    ),
                    Err(e) => warn!("firmware: confirm: {:?}", e),
                }
                unconfirmed = false;
            } else if started.elapsed() > CONFIRM_WITHIN {
                warn!("firmware: not back in the mesh within 5 min: restarting, so the bootloader reverts");
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
        Timer::after(FEED_EVERY).await;
    }
}

/// Checks an update staged in the update slot (`navgrid update` writes one through the probe): its
/// signature against the provisioned key, over the image's SHA-512 digest. A good one is marked
/// for the bootloader and the device restarts into it; a bad one is discarded, and one for
/// another board kept. Runs at boot, before the radio starts, as it writes RRAM.
pub fn apply_staged(rram: &mut Nvmc<'_>, signing_key: Option<[u8; 32]>) {
    // SAFETY: memory-mapped RRAM, written only through `rram`.
    let page = unsafe { core::slice::from_raw_parts(STAGED_UPDATE as *const u8, UPDATE_HEADER_LEN) };
    let header = match UpdateHeader::decode(page) {
        Ok(header) => header,
        Err(ProvisioningError::Blank) => return,
        Err(e) => {
            warn!("update: staged header unreadable ({}), discarding it", e);
            discard(rram);
            return;
        }
    };
    if header.board != BOARD {
        info!(
            "update: carrying {} for the {} board, to pass on",
            header.firmware,
            header.board.name()
        );
        return;
    }
    let Some(key) = signing_key else {
        warn!("update: no signing key provisioned, discarding {}", header.firmware);
        discard(rram);
        return;
    };
    let t = Instant::now();
    let verified = {
        let flash = BlockingMutex::<NoopRawMutex, _>::new(RefCell::new(&mut *rram));
        let config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash, &flash);
        let mut aligned = [0u8; 16];
        let mut updater = BlockingFirmwareUpdater::new(config, &mut aligned);
        updater.verify_and_mark_updated(&key, &header.signature, header.len)
    };
    discard(rram);
    match verified {
        Ok(()) => {
            // Kept so the installed image can be passed on (`servable`).
            let mut line = [0xffu8; 96];
            line[..UPDATE_HEADER_LEN].copy_from_slice(&header.encode());
            let kept = rram
                .erase(ACTIVE_HEADER, ACTIVE_HEADER + ACTIVE_HEADER_LEN)
                .and_then(|()| rram.write(ACTIVE_HEADER, &line));
            if let Err(e) = kept {
                warn!("update: keeping the header: {:?}", e);
            }
            info!(
                "update: {} verified in {=u64} ms, restarting into it",
                header.firmware,
                t.elapsed().as_millis()
            );
            // Time for the log to leave over RTT.
            embassy_time::block_for(Duration::from_millis(100));
            cortex_m::peripheral::SCB::sys_reset();
        }
        Err(e) => warn!("update: {} rejected: {:?}", header.firmware, e),
    }
}

fn discard(rram: &mut Nvmc<'_>) {
    if let Err(e) = rram.erase(STAGED_UPDATE, STAGED_UPDATE + STAGED_UPDATE_LEN) {
        warn!("update: clearing the staging page: {:?}", e);
    }
}
