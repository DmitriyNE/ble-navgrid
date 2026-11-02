//! navgrid bootloader (architecture §13): embassy-boot over the slots in `memory.x`.
//!
//! - **Update:** an update marked in the state region is swapped into the active slot.
//! - **Revert:** an update the application didn't confirm is swapped back on the next boot.
//! - **Watchdog:** the bootloader starts it (5 s); the application takes it over, so a hung image
//!   resets and reverts.

#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m_rt::{entry, exception};
#[cfg(feature = "defmt")]
use defmt_rtt as _;
use embassy_boot_nrf::{BootLoader, BootLoaderConfig, WatchdogFlash};
use embassy_nrf::rramc::Nvmc;
use embassy_nrf::wdt::{self, HaltConfig, SleepConfig};
use embassy_sync::blocking_mutex::Mutex;

#[entry]
fn main() -> ! {
    let p = embassy_nrf::init(Default::default());
    #[cfg(feature = "defmt")]
    defmt::info!("bootloader: start");

    let mut wdt_config = wdt::Config::default();
    wdt_config.timeout_ticks = 32768 * 5;
    wdt_config.action_during_sleep = SleepConfig::Run;
    // So a debugger halting the core doesn't reset it.
    wdt_config.action_during_debug_halt = HaltConfig::Pause;
    let flash = WatchdogFlash::start(Nvmc::new(p.RRAMC), p.WDT0, wdt_config);
    let flash = Mutex::new(RefCell::new(flash));

    let config = BootLoaderConfig::from_linkerfile_blocking(&flash, &flash, &flash);
    let active = config.active.offset();
    let bootloader: BootLoader = BootLoader::prepare(config);
    #[cfg(feature = "defmt")]
    defmt::info!("bootloader: starting the image at {=u32:#x}", active);
    // SAFETY: the active slot holds an application linked for it.
    unsafe { bootloader.load(active) }
}

#[unsafe(no_mangle)]
#[cfg_attr(target_os = "none", unsafe(link_section = ".HardFault.user"))]
unsafe extern "C" fn HardFault() {
    cortex_m::peripheral::SCB::sys_reset();
}

#[exception]
unsafe fn DefaultHandler(_: i16) -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    #[cfg(feature = "defmt")]
    defmt::error!("bootloader: panic: {}", defmt::Display2Format(_info));
    cortex_m::peripheral::SCB::sys_reset();
}
