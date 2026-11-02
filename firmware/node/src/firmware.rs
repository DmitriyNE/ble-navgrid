//! This image under the bootloader (architecture §13, D-34). It takes over the watchdog the
//! bootloader started and feeds it, so a hung image resets. A freshly swapped-in image confirms
//! itself once the node is back in the mesh (a fleet link up). One that isn't within 5 min
//! restarts unconfirmed, and the bootloader puts the previous image back.

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
            warn!("firmware: reverted to the previous image");
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
