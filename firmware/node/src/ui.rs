//! The TAG's RGB LED and button (FR-O7, architecture §13). Nothing on the LED runs periodically:
//! the LED shows the boot state, and status when the button is pressed.
//!
//! - **Boot:** 1 s green when enrolled, blue when not, red when the provisioning page is corrupt.
//! - **Short press:** a white acknowledgement, connectivity, then battery, as colour plus pattern
//!   ([`show_status`]); the beacon raises its attention flag.
//! - **Long press (3 s):** toggles install mode (two blue blinks on, one off): for 10 minutes the
//!   beacon's install flag is up, advertising is fast, and the LED shows connectivity every
//!   10 s, so an installer sees the node find its links and a route.
//! - **Held for 5 s at power-up:** factory reset ([`factory_reset_if_held`]).

use defmt::{info, warn};
use embassy_futures::select::{Either, select};
use embassy_nrf::Peri;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::peripherals::{P0_00, P2_08, P2_09, P2_10};
use embassy_nrf::rramc::Nvmc;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_storage::nor_flash::NorFlash;
use navgrid_proto::{BATTERY_CRITICAL_MV, BATTERY_GOOD_MV, BATTERY_LOW_MV, PROVISIONING_ADDRESS};

use crate::status::{self, Connectivity};

const LONG_PRESS: Duration = Duration::from_secs(3);
const RESET_HOLD: Duration = Duration::from_secs(5);
const DEBOUNCE: Duration = Duration::from_millis(20);

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Colour {
    Off,
    Red,
    Green,
    Blue,
    Yellow,
    Magenta,
    White,
}

/// LED 1: red P2.08, green P2.10, blue P2.09, all active low.
pub struct Led {
    red: Output<'static>,
    green: Output<'static>,
    blue: Output<'static>,
}

impl Led {
    pub fn new(red: Peri<'static, P2_08>, green: Peri<'static, P2_10>, blue: Peri<'static, P2_09>) -> Self {
        Self {
            red: Output::new(red, Level::High, OutputDrive::Standard),
            green: Output::new(green, Level::High, OutputDrive::Standard),
            blue: Output::new(blue, Level::High, OutputDrive::Standard),
        }
    }

    pub fn set(&mut self, colour: Colour) {
        let (r, g, b) = match colour {
            Colour::Off => (false, false, false),
            Colour::Red => (true, false, false),
            Colour::Green => (false, true, false),
            Colour::Blue => (false, false, true),
            Colour::Yellow => (true, true, false),
            Colour::Magenta => (true, false, true),
            Colour::White => (true, true, true),
        };
        let level = |on: bool| if on { Level::Low } else { Level::High };
        self.red.set_level(level(r));
        self.green.set_level(level(g));
        self.blue.set_level(level(b));
    }

    /// `times` blinks of `on_ms` lit and `off_ms` dark.
    pub async fn blink(&mut self, colour: Colour, times: u8, on_ms: u64, off_ms: u64) {
        for _ in 0..times {
            self.set(colour);
            Timer::after_millis(on_ms).await;
            self.set(Colour::Off);
            Timer::after_millis(off_ms).await;
        }
    }
}

/// The button on P0.00, active low.
pub fn button(pin: Peri<'static, P0_00>) -> Input<'static> {
    Input::new(pin, Pull::Up)
}

/// At power-up, with the button held: if it stays down for 5 s, the LED turns red, the
/// provisioning page is wiped and the device restarts unenrolled. Releasing it earlier boots
/// normally.
pub async fn factory_reset_if_held(button: &mut Input<'static>, led: &mut Led, rram: &mut Nvmc<'_>) {
    if button.is_high() {
        return;
    }
    let held = with_timeout(RESET_HOLD, button.wait_for_high()).await.is_err();
    if !held {
        return;
    }
    led.set(Colour::Red);
    warn!("factory reset: wiping the provisioning page");
    if let Err(e) = rram.erase(
        PROVISIONING_ADDRESS,
        PROVISIONING_ADDRESS + embassy_nrf::rramc::PAGE_SIZE as u32,
    ) {
        warn!("factory reset: {:?}", e);
    }
    button.wait_for_high().await;
    cortex_m::peripheral::SCB::sys_reset();
}

/// The boot colour, then the button, forever.
#[embassy_executor::task]
pub async fn run(mut led: Led, mut button: Input<'static>, boot: Colour) -> ! {
    led.set(boot);
    Timer::after_secs(1).await;
    led.set(Colour::Off);
    loop {
        button.wait_for_low().await;
        let pressed = Instant::now();
        Timer::after(DEBOUNCE).await;
        if with_timeout(LONG_PRESS, button.wait_for_high()).await.is_err() {
            let on = status::toggle_install();
            info!("button: long press: install mode {}", if on { "on" } else { "off" });
            led.blink(Colour::Blue, if on { 2 } else { 1 }, 150, 150).await;
            button.wait_for_high().await;
            Timer::after(DEBOUNCE).await;
            while status::install_left_s() > 0 {
                // Connectivity every 10 s, until the mode ends or the button is pressed again.
                if let Either::Second(()) = select(Timer::after_secs(10), button.wait_for_low()).await {
                    break;
                }
                show_connectivity(&mut led).await;
            }
            continue;
        }
        info!("button: short press ({=u64} ms)", pressed.elapsed().as_millis());
        status::attention();
        Timer::after(DEBOUNCE).await;
        // A second press restarts the display: the loop sees the button already down.
        if let Either::Second(()) = select(show_status(&mut led), button.wait_for_low()).await {
            led.set(Colour::Off);
        }
    }
}

/// The short-press display (architecture §13): a white flash, connectivity, a 0.7 s gap, then
/// Connectivity as colour plus pattern: green for a route to a sink (one blink per hop, long at
/// a sink), yellow for peers without a route, magenta for another fleet, red for no one.
async fn show_connectivity(led: &mut Led) {
    let connectivity = status::connectivity();
    info!("status: {}", connectivity);
    match connectivity {
        Connectivity::Sink(0) => led.blink(Colour::Green, 1, 1000, 0).await,
        Connectivity::Sink(hops @ 1..=4) => led.blink(Colour::Green, hops, 100, 150).await,
        Connectivity::Sink(_) => led.blink(Colour::Green, 1, 1000, 0).await,
        Connectivity::Mesh => led.blink(Colour::Yellow, 1, 1000, 0).await,
        Connectivity::Foreign => led.blink(Colour::Magenta, 2, 300, 300).await,
        Connectivity::Alone => led.blink(Colour::Red, 5, 50, 50).await,
        Connectivity::Unenrolled => led.blink(Colour::Blue, 1, 1000, 0).await,
    }
}

/// battery. Connectivity blinks are short and battery blinks long, so the two don't run together.
/// A critically low cell shows only one short red blink.
async fn show_status(led: &mut Led) {
    let battery = status::battery_mv();
    if battery.is_some_and(|mv| mv < BATTERY_CRITICAL_MV) {
        led.blink(Colour::Red, 1, 100, 0).await;
        return;
    }
    led.blink(Colour::White, 1, 50, 300).await;
    info!("status: battery {} mV", battery);
    show_connectivity(led).await;
    let Some(mv) = battery else {
        return;
    };
    Timer::after_millis(700).await;
    let (colour, times) = if mv >= BATTERY_GOOD_MV {
        (Colour::Green, 3)
    } else if mv >= BATTERY_LOW_MV {
        (Colour::Yellow, 2)
    } else {
        (Colour::Red, 1)
    };
    led.blink(colour, times, 300, 300).await;
}
