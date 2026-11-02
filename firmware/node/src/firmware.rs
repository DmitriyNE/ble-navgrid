//! This image under the bootloader (architecture §13, D-34, D-35).
//!
//! - **Staged updates:** at boot, an update waiting in the update slot is checked against the
//!   provisioned signing key and handed to the bootloader ([`apply_staged`]).
//! - **Watchdog:** the image takes over the one the bootloader started and feeds it, so a hung
//!   image resets.
//! - **Confirming:** a freshly swapped-in image confirms itself once the node is back in the mesh
//!   (a fleet link up). One that isn't within 5 min restarts unconfirmed, and the bootloader puts
//!   the previous image back.

use core::cell::RefCell;

use defmt::{info, warn};
use embassy_boot::State;
use embassy_boot_nrf::{BlockingFirmwareUpdater, FirmwareUpdaterConfig};
use embassy_nrf::Peri;
use embassy_nrf::peripherals::WDT0;
use embassy_nrf::rramc::Nvmc;
use embassy_nrf::wdt::{self, Watchdog};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_storage::nor_flash::NorFlash;
use navgrid_proto::layout::{STAGED_UPDATE, STAGED_UPDATE_LEN};
use navgrid_proto::{ProvisioningError, UPDATE_HEADER_LEN, UpdateHeader};

use crate::status;

const FEED_EVERY: Duration = Duration::from_secs(1);
const CONFIRM_WITHIN: Duration = Duration::from_secs(5 * 60);

#[embassy_executor::task]
pub async fn run(rram: Nvmc<'static>, wdt: Peri<'static, WDT0>) -> ! {
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
    let flash = Mutex::<NoopRawMutex, _>::new(RefCell::new(rram));
    let config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash, &flash);
    let mut aligned = [0u8; 16];
    let mut updater = BlockingFirmwareUpdater::new(config, &mut aligned);
    let mut unconfirmed = match updater.get_state() {
        Ok(State::Swap) => {
            info!("firmware: new image, unconfirmed until a fleet link is up");
            true
        }
        Ok(State::Revert) => {
            // This image ran confirmed before the update, and updates wait for a confirmed one.
            warn!("firmware: reverted to the previous image");
            if let Err(e) = updater.mark_booted() {
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
            if status::links() > 0 {
                let t = Instant::now();
                match updater.mark_booted() {
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
/// for the bootloader and the device restarts into it; a bad one is discarded. Runs at boot,
/// before the radio starts, as it writes RRAM.
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
    let Some(key) = signing_key else {
        warn!("update: no signing key provisioned, discarding {}", header.firmware);
        discard(rram);
        return;
    };
    let t = Instant::now();
    let verified = {
        let flash = Mutex::<NoopRawMutex, _>::new(RefCell::new(&mut *rram));
        let config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash, &flash);
        let mut aligned = [0u8; 16];
        let mut updater = BlockingFirmwareUpdater::new(config, &mut aligned);
        updater.verify_and_mark_updated(&key, &header.signature, header.len)
    };
    discard(rram);
    match verified {
        Ok(()) => {
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
