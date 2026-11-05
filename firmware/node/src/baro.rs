//! The BME688 (FR-S3, FR-T2): a forced measurement every few seconds, the mean of the last few
//! pressures in the beacon (while the controller lets it fix heights) and in an Environment record
//! now and then. The maths is in `navgrid-baro`; this is the I2C side. The DK has no barometer:
//! the task runs on the TAG only, and a node without one answers barometer commands unsupported.

#![cfg_attr(not(feature = "board-tag"), allow(dead_code))]

use defmt::{info, warn};
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{Peri, bind_interrupts, peripherals};
use embassy_time::{Duration, Instant, Timer};
use navgrid_baro::{ADDRESS, Average, COEFF1_LEN, COEFF2_LEN, COEFF3_LEN, Calibration, FIELD0_LEN, Settings, reg};
use navgrid_mesh::{Body, Environment};

use crate::{state, status};

bind_interrupts!(struct Irqs {
    SERIAL21 => twim::InterruptHandler<peripherals::SERIAL21>;
});

/// The longest mean the controller may ask for.
pub const AVERAGE_MAX: usize = 32;
/// The beacon carries a new pressure once the mean moved this far: 2 Pa, about 17 cm. Weather
/// moves it a pascal or two a minute, so beacons follow it without re-advertising every reading.
const BEACON_STEP_DPA: u32 = 20;
const SETTINGS: Settings = Settings::BAROMETRY;
const LOG_EVERY: Duration = Duration::from_secs(60);

#[embassy_executor::task]
pub async fn run(
    serial: Peri<'static, peripherals::SERIAL21>,
    sda: Peri<'static, peripherals::P1_12>,
    scl: Peri<'static, peripherals::P1_11>,
) -> ! {
    let mut config = twim::Config::default();
    // The board's pull-ups are on the sensor's breakout; the chip's own add to them harmlessly.
    config.sda_pullup = true;
    config.scl_pullup = true;
    let mut i2c = Twim::new(serial, Irqs, sda, scl, config, &mut []);
    let calibration = match start(&mut i2c).await {
        Ok(c) => c,
        Err(e) => {
            warn!("baro: no BME688: {}", e);
            status::set_baro_present(false);
            loop {
                Timer::after(Duration::from_secs(3600)).await;
            }
        }
    };
    status::set_baro_present(true);
    let mut average = Average::<AVERAGE_MAX>::new();
    let mut advertised: Option<u32> = None;
    let mut last_record = Instant::now();
    let mut last_log: Option<Instant> = None;
    loop {
        let (sample_s, window, height) = status::baro();
        if sample_s == 0 {
            status::set_pressure_dpa(None);
            average.clear();
            Timer::after(Duration::from_secs(5)).await;
            continue;
        }
        let started = Instant::now();
        match measure(&mut i2c, &calibration).await {
            Ok(sample) => {
                average.push(sample.pressure_pa, usize::from(window));
                let (mean, n) = average.mean().expect("one pushed");
                let dpa = (mean * 10.0) as u32;
                status::set_pressure_dpa(Some(dpa));
                if height && advertised.is_none_or(|a| a.abs_diff(dpa) >= BEACON_STEP_DPA) {
                    advertised = Some(dpa);
                    status::BEACON_CHANGED.signal(());
                }
                if last_log.is_none_or(|t| t.elapsed() >= LOG_EVERY) {
                    last_log = Some(Instant::now());
                    info!(
                        "baro: {=f32} Pa (mean of {=usize}), {=f32} °C, {=f32} %RH",
                        mean as f32, n, sample.temperature_c as f32, sample.humidity_pct as f32
                    );
                }
                let every = status::environment_s();
                if every > 0 && last_record.elapsed() >= Duration::from_secs(u64::from(every)) {
                    last_record = Instant::now();
                    state::write(Body::Environment(Environment {
                        temperature_cdeg: (sample.temperature_c * 100.0) as i16,
                        humidity_cpct: (sample.humidity_pct * 100.0) as u16,
                        pressure_cpa: (mean * 100.0) as u32,
                        gas_ohm: None,
                        samples: n as u8,
                        height,
                    }));
                }
            }
            Err(e) => warn!("baro: measurement failed: {}", e),
        }
        let period = Duration::from_secs(u64::from(sample_s));
        let spent = started.elapsed();
        Timer::after(if spent < period {
            period - spent
        } else {
            Duration::from_millis(100)
        })
        .await;
    }
}

#[derive(defmt::Format)]
enum BaroError {
    Bus(twim::Error),
    ChipId(u8),
    NoData,
}

impl From<twim::Error> for BaroError {
    fn from(e: twim::Error) -> Self {
        Self::Bus(e)
    }
}

/// Checks the chip, resets it, reads its calibration and sets it up: heater off, the oversampling
/// and filter of [`SETTINGS`].
async fn start(i2c: &mut Twim<'_>) -> Result<Calibration, BaroError> {
    let mut id = [0u8; 1];
    i2c.write_read(ADDRESS, &[reg::CHIP_ID], &mut id).await?;
    if id[0] != navgrid_baro::CHIP_ID {
        return Err(BaroError::ChipId(id[0]));
    }
    i2c.write(ADDRESS, &[reg::RESET, navgrid_baro::SOFT_RESET]).await?;
    Timer::after(Duration::from_millis(10)).await;
    let mut c1 = [0u8; COEFF1_LEN];
    let mut c2 = [0u8; COEFF2_LEN];
    let mut c3 = [0u8; COEFF3_LEN];
    i2c.write_read(ADDRESS, &[reg::COEFF1], &mut c1).await?;
    i2c.write_read(ADDRESS, &[reg::COEFF2], &mut c2).await?;
    i2c.write_read(ADDRESS, &[reg::COEFF3], &mut c3).await?;
    let mut variant = [0u8; 1];
    i2c.write_read(ADDRESS, &[reg::VARIANT_ID], &mut variant).await?;
    i2c.write(ADDRESS, &[reg::CTRL_GAS_0, navgrid_baro::HEAT_OFF]).await?;
    i2c.write(ADDRESS, &[reg::CTRL_GAS_1, 0]).await?;
    i2c.write(ADDRESS, &[reg::CTRL_HUM, SETTINGS.ctrl_hum()]).await?;
    i2c.write(ADDRESS, &[reg::CONFIG, SETTINGS.config()]).await?;
    let calibration = Calibration::from_blocks(&c1, &c2, &c3);
    info!("baro: BME68x variant {=u8}, calibration {}", variant[0], calibration);
    Ok(calibration)
}

/// One forced measurement, compensated.
async fn measure(i2c: &mut Twim<'_>, calibration: &Calibration) -> Result<navgrid_baro::Sample, BaroError> {
    i2c.write(ADDRESS, &[reg::CTRL_MEAS, SETTINGS.ctrl_meas_forced()])
        .await?;
    Timer::after(Duration::from_micros(u64::from(SETTINGS.duration_us()))).await;
    let mut field0 = [0u8; FIELD0_LEN];
    for _ in 0..10 {
        i2c.write_read(ADDRESS, &[reg::FIELD0], &mut field0).await?;
        if let Some(sample) = calibration.sample(&field0) {
            return Ok(sample);
        }
        Timer::after(Duration::from_millis(5)).await;
    }
    Err(BaroError::NoData)
}
