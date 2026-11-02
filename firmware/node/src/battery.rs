//! Battery voltage (FR-S2): VDD through the SAADC's internal input. A CR2032 holds about 3.0 V at
//! rest almost to the end and only sags under current, so each reading is the lowest of a burst of
//! samples spread over a second, which with CS running lands on radio activity.

use defmt::info;
use embassy_nrf::peripherals::SAADC;
use embassy_nrf::saadc::{self, ChannelConfig, Saadc, VddInput};
use embassy_nrf::{Peri, bind_interrupts};
use embassy_time::{Duration, Timer};

use crate::status;

bind_interrupts!(struct Irqs {
    SAADC => saadc::InterruptHandler;
});

const PERIOD: Duration = Duration::from_secs(60);
const BURST: usize = 16;
const BURST_GAP: Duration = Duration::from_millis(64);
/// Full scale: the 0.9 V internal reference over gain 2/8.
const FULL_SCALE_MV: i32 = 3600;
/// A change this big restarts advertising, so the beacon carries it.
const BEACON_STEP_MV: u16 = 100;

#[embassy_executor::task]
pub async fn run(saadc: Peri<'static, SAADC>) -> ! {
    let channel = ChannelConfig::single_ended(VddInput);
    let mut adc = Saadc::new(saadc, Irqs, saadc::Config::default(), [channel]);
    adc.calibrate().await;
    let mut advertised: Option<u16> = None;
    loop {
        let mut lowest = i16::MAX;
        for _ in 0..BURST {
            let mut buf = [0i16; 1];
            adc.sample(&mut buf).await;
            lowest = lowest.min(buf[0]);
            Timer::after(BURST_GAP).await;
        }
        let mv = (i32::from(lowest.max(0)) * FULL_SCALE_MV / 4096) as u16;
        status::set_battery(mv);
        info!("battery: {=u16} mV", mv);
        if advertised.is_none_or(|a| a.abs_diff(mv) >= BEACON_STEP_MV) {
            advertised = Some(mv);
            status::BEACON_CHANGED.signal(());
        }
        Timer::after(PERIOD).await;
    }
}
