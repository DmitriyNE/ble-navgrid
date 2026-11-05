//! Barometry (FR-S3, FR-T2, FR-M3): the BME688's registers, calibration and compensation, a
//! moving average of pressure, and heights from pressure differences.
//!
//! The compensation follows Bosch's BME68x Sensor API (v4.4.8, `bme68x.c`, the floating-point
//! variant), in `f64`: the device computes it once every few seconds, so the cost doesn't matter
//! and single precision would lose a few hundredths of a pascal. The integer variant lives in the
//! tests, an independent transcription the float one is checked against.

#![no_std]

/// The sensor's I2C address on the TAG (SDO to ground); 0x77 with SDO high.
pub const ADDRESS: u8 = 0x76;

/// Registers (BME688 datasheet §5.2).
pub mod reg {
    pub const CHIP_ID: u8 = 0xD0;
    pub const VARIANT_ID: u8 = 0xF0;
    pub const RESET: u8 = 0xE0;
    pub const CTRL_GAS_0: u8 = 0x70;
    pub const CTRL_GAS_1: u8 = 0x71;
    pub const CTRL_HUM: u8 = 0x72;
    pub const CTRL_MEAS: u8 = 0x74;
    pub const CONFIG: u8 = 0x75;
    /// Field 0: status, then pressure (3), temperature (3), humidity (2).
    pub const FIELD0: u8 = 0x1D;
    pub const COEFF1: u8 = 0x8A;
    pub const COEFF2: u8 = 0xE1;
    pub const COEFF3: u8 = 0x00;
}

/// What `CHIP_ID` reads on a BME680 or BME688.
pub const CHIP_ID: u8 = 0x61;
/// Written to `RESET`.
pub const SOFT_RESET: u8 = 0xB6;
/// `CTRL_GAS_0`: heater off.
pub const HEAT_OFF: u8 = 0x08;
/// Bytes read from `FIELD0`: status, gas index, measurement index, pressure, temperature,
/// humidity.
pub const FIELD0_LEN: usize = 11;
pub const COEFF1_LEN: usize = 23;
pub const COEFF2_LEN: usize = 14;
pub const COEFF3_LEN: usize = 5;

/// Oversampling settings for `ctrl_meas` and `ctrl_hum`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Oversampling {
    Skip = 0,
    X1 = 1,
    X2 = 2,
    X4 = 3,
    X8 = 4,
    X16 = 5,
}

impl Oversampling {
    /// Conversion cycles it takes.
    pub fn cycles(self) -> u32 {
        match self {
            Self::Skip => 0,
            Self::X1 => 1,
            Self::X2 => 2,
            Self::X4 => 4,
            Self::X8 => 8,
            Self::X16 => 16,
        }
    }
}

/// How a measurement is taken.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Settings {
    pub temperature: Oversampling,
    pub pressure: Oversampling,
    pub humidity: Oversampling,
    /// The IIR filter's coefficient code, 0 (off) to 7.
    pub filter: u8,
}

impl Settings {
    /// Pressure at x16 for the height, temperature at x2 for its compensation, humidity at x1 for
    /// the sideband, the IIR filter off: the node averages samples itself, at a period it is told.
    pub const BAROMETRY: Self = Self {
        temperature: Oversampling::X2,
        pressure: Oversampling::X16,
        humidity: Oversampling::X1,
        filter: 0,
    };

    /// `ctrl_hum`.
    pub fn ctrl_hum(&self) -> u8 {
        self.humidity as u8
    }

    /// `ctrl_meas` starting a forced measurement.
    pub fn ctrl_meas_forced(&self) -> u8 {
        ((self.temperature as u8) << 5) | ((self.pressure as u8) << 2) | 0b01
    }

    /// `config`.
    pub fn config(&self) -> u8 {
        (self.filter & 7) << 2
    }

    /// How long a forced measurement takes, in microseconds, as the Sensor API reckons it
    /// (`bme68x_get_meas_dur`), heater off.
    pub fn duration_us(&self) -> u32 {
        let cycles = self.temperature.cycles() + self.pressure.cycles() + self.humidity.cycles();
        cycles * 1963 + 477 * 4 + 477 * 5 + 1000
    }
}

/// The factory calibration, from the three coefficient blocks.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Calibration {
    pub t1: u16,
    pub t2: i16,
    pub t3: i8,
    pub p1: u16,
    pub p2: i16,
    pub p3: i8,
    pub p4: i16,
    pub p5: i16,
    pub p6: i8,
    pub p7: i8,
    pub p8: i16,
    pub p9: i16,
    pub p10: u8,
    pub h1: u16,
    pub h2: u16,
    pub h3: i8,
    pub h4: i8,
    pub h5: i8,
    pub h6: u8,
    pub h7: i8,
}

impl Calibration {
    /// From `COEFF1` (0x8A, 23 bytes), `COEFF2` (0xE1, 14 bytes) and `COEFF3` (0x00, 5 bytes), as
    /// `bme68x.c` `get_calib_data` concatenates them.
    pub fn from_blocks(c1: &[u8; COEFF1_LEN], c2: &[u8; COEFF2_LEN], c3: &[u8; COEFF3_LEN]) -> Self {
        let mut c = [0u8; COEFF1_LEN + COEFF2_LEN + COEFF3_LEN];
        c[..COEFF1_LEN].copy_from_slice(c1);
        c[COEFF1_LEN..COEFF1_LEN + COEFF2_LEN].copy_from_slice(c2);
        c[COEFF1_LEN + COEFF2_LEN..].copy_from_slice(c3);
        let u16_at = |msb: usize, lsb: usize| u16::from(c[msb]) << 8 | u16::from(c[lsb]);
        Self {
            t1: u16_at(32, 31),
            t2: u16_at(1, 0) as i16,
            t3: c[2] as i8,
            p1: u16_at(5, 4),
            p2: u16_at(7, 6) as i16,
            p3: c[8] as i8,
            p4: u16_at(11, 10) as i16,
            p5: u16_at(13, 12) as i16,
            p7: c[14] as i8,
            p6: c[15] as i8,
            p8: u16_at(19, 18) as i16,
            p9: u16_at(21, 20) as i16,
            p10: c[22],
            h1: (u16::from(c[25]) << 4) | u16::from(c[24] & 0x0F),
            h2: (u16::from(c[23]) << 4) | u16::from(c[24] >> 4),
            h3: c[26] as i8,
            h4: c[27] as i8,
            h5: c[28] as i8,
            h6: c[29],
            h7: c[30] as i8,
        }
    }

    /// Temperature in °C and the `t_fine` the other two need.
    pub fn temperature(&self, adc: u32) -> (f64, f64) {
        let adc = f64::from(adc);
        let var1 = (adc / 16384.0 - f64::from(self.t1) / 1024.0) * f64::from(self.t2);
        let d = adc / 131072.0 - f64::from(self.t1) / 8192.0;
        let var2 = d * d * (f64::from(self.t3) * 16.0);
        let t_fine = var1 + var2;
        (t_fine / 5120.0, t_fine)
    }

    /// Pressure in pascals.
    pub fn pressure(&self, adc: u32, t_fine: f64) -> f64 {
        let mut var1 = t_fine / 2.0 - 64000.0;
        let mut var2 = var1 * var1 * (f64::from(self.p6) / 131072.0);
        var2 += var1 * f64::from(self.p5) * 2.0;
        var2 = var2 / 4.0 + f64::from(self.p4) * 65536.0;
        var1 = ((f64::from(self.p3) * var1 * var1) / 16384.0 + f64::from(self.p2) * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * f64::from(self.p1);
        if var1.abs() < 1.0 {
            return 0.0;
        }
        let mut p = 1048576.0 - f64::from(adc);
        p = ((p - var2 / 4096.0) * 6250.0) / var1;
        let var1 = f64::from(self.p9) * p * p / 2147483648.0;
        let var2 = p * (f64::from(self.p8) / 32768.0);
        let q = p / 256.0;
        let var3 = q * q * q * (f64::from(self.p10) / 131072.0);
        p + (var1 + var2 + var3 + f64::from(self.p7) * 128.0) / 16.0
    }

    /// Relative humidity in %, clamped to 0–100.
    pub fn humidity(&self, adc: u16, t_fine: f64) -> f64 {
        let t = t_fine / 5120.0;
        let var1 = f64::from(adc) - (f64::from(self.h1) * 16.0 + (f64::from(self.h3) / 2.0) * t);
        let var2 = var1
            * (f64::from(self.h2) / 262144.0
                * (1.0 + (f64::from(self.h4) / 16384.0) * t + (f64::from(self.h5) / 1048576.0) * t * t));
        let var3 = f64::from(self.h6) / 16384.0;
        let var4 = f64::from(self.h7) / 2097152.0;
        (var2 + (var3 + var4 * t) * var2 * var2).clamp(0.0, 100.0)
    }
}

/// One measurement, compensated.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Sample {
    pub temperature_c: f64,
    pub pressure_pa: f64,
    pub humidity_pct: f64,
}

/// The raw values in field 0, if a new measurement is there: `None` while it's still measuring.
pub fn raw(field0: &[u8; FIELD0_LEN]) -> Option<(u32, u32, u16)> {
    if field0[0] & 0x80 == 0 {
        return None;
    }
    let adc20 =
        |i: usize| (u32::from(field0[i]) << 12) | (u32::from(field0[i + 1]) << 4) | (u32::from(field0[i + 2]) >> 4);
    let pressure = adc20(2);
    let temperature = adc20(5);
    let humidity = u16::from(field0[8]) << 8 | u16::from(field0[9]);
    Some((temperature, pressure, humidity))
}

impl Calibration {
    /// A measurement from field 0, if a new one is there.
    pub fn sample(&self, field0: &[u8; FIELD0_LEN]) -> Option<Sample> {
        let (t, p, h) = raw(field0)?;
        let (temperature_c, t_fine) = self.temperature(t);
        Some(Sample {
            temperature_c,
            pressure_pa: self.pressure(p, t_fine),
            humidity_pct: self.humidity(h, t_fine),
        })
    }
}

/// The mean of the last `N` pressures (at most), and the number in it: the node's reported
/// pressure, its averaging length set by the controller.
#[derive(Clone, Debug)]
pub struct Average<const N: usize> {
    ring: [f64; N],
    next: usize,
    len: usize,
}

impl<const N: usize> Default for Average<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Average<N> {
    pub const fn new() -> Self {
        Self {
            ring: [0.0; N],
            next: 0,
            len: 0,
        }
    }

    /// Adds `value`, keeping the last `window` (clamped to 1..=N).
    pub fn push(&mut self, value: f64, window: usize) {
        let window = window.clamp(1, N);
        if self.len > window {
            // The window shrank: keep the newest `window`.
            let mut kept = [0.0; N];
            for (i, k) in kept.iter_mut().enumerate().take(window) {
                *k = self.ring[(self.next + N - window + i) % N];
            }
            self.ring = kept;
            self.next = window % N;
            self.len = window;
        }
        self.ring[self.next] = value;
        self.next = (self.next + 1) % N;
        self.len = (self.len + 1).min(window);
    }

    /// The mean and how many samples it holds; `None` before the first.
    pub fn mean(&self) -> Option<(f64, usize)> {
        if self.len == 0 {
            return None;
        }
        let sum: f64 = (0..self.len).map(|i| self.ring[(self.next + N - 1 - i) % N]).sum();
        Some((sum / self.len as f64, self.len))
    }

    pub fn clear(&mut self) {
        self.len = 0;
        self.next = 0;
    }
}

/// The dry-air gas constant, J/(kg·K), and standard gravity, m/s².
const R_DRY: f64 = 287.05;
const G: f64 = 9.80665;

/// How far above the reference a point is, in metres, from the pressures at both (pascals) and the
/// air temperature between them (°C): the hypsometric equation, which for indoor heights is
/// `(p_ref - p) / (ρ g)`, about 8.3 cm a pascal at 20 °C near sea level.
pub fn height_above_m(p_ref_pa: f64, p_pa: f64, temperature_c: f64) -> f64 {
    if p_ref_pa <= 0.0 || p_pa <= 0.0 {
        return 0.0;
    }
    R_DRY * (temperature_c + 273.15) / G * libm::log(p_ref_pa / p_pa)
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    /// A plausible BME680/688 calibration (coefficients of the size the parts carry).
    fn calibration() -> Calibration {
        Calibration {
            t1: 26245,
            t2: 26326,
            t3: 3,
            p1: 36226,
            p2: -10464,
            p3: 88,
            p4: 6993,
            p5: -56,
            p6: 30,
            p7: 34,
            p8: -1906,
            p9: -2879,
            p10: 30,
            h1: 687,
            h2: 1046,
            h3: 0,
            h4: 45,
            h5: 20,
            h6: 120,
            h7: -100,
        }
    }

    /// The Sensor API's integer temperature (0.01 °C) and `t_fine`.
    fn temperature_int(c: &Calibration, adc: u32) -> (i32, i32) {
        let var1: i64 = (i64::from(adc) >> 3) - (i64::from(c.t1) << 1);
        let var2: i64 = (var1 * i64::from(c.t2)) >> 11;
        let var3: i64 = ((var1 >> 1) * (var1 >> 1)) >> 12;
        let var3: i64 = (var3 * (i64::from(c.t3) << 4)) >> 14;
        let t_fine = (var2 + var3) as i32;
        (((t_fine * 5) + 128) >> 8, t_fine)
    }

    /// The Sensor API's integer pressure (Pa).
    fn pressure_int(c: &Calibration, adc: u32, t_fine: i32) -> u32 {
        let mut var1: i32 = (t_fine >> 1) - 64000;
        let mut var2: i32 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * i32::from(c.p6)) >> 2;
        var2 += (var1 * i32::from(c.p5)) << 1;
        var2 = (var2 >> 2) + (i32::from(c.p4) << 16);
        var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * (i32::from(c.p3) << 5)) >> 3) + ((i32::from(c.p2) * var1) >> 1);
        var1 >>= 18;
        var1 = ((32768 + var1) * i32::from(c.p1)) >> 15;
        let mut p: i32 = 1048576 - adc as i32;
        p = ((p - (var2 >> 12)) as u32).wrapping_mul(3125) as i32;
        if p >= 0x4000_0000 {
            p = (p / var1) << 1;
        } else {
            p = (p << 1) / var1;
        }
        let var1 = (i32::from(c.p9) * (((p >> 3) * (p >> 3)) >> 13)) >> 12;
        let var2 = ((p >> 2) * i32::from(c.p8)) >> 13;
        let q = i64::from(p >> 8);
        let var3 = ((q * q * q * i64::from(c.p10)) >> 17) as i32;
        (p + ((var1 + var2 + var3 + (i32::from(c.p7) << 7)) >> 4)) as u32
    }

    #[test]
    fn float_compensation_agrees_with_the_integer_one() {
        let c = calibration();
        let mut checked = 0;
        let mut worst = 0.0f64;
        for t_adc in (420_000u32..560_000).step_by(7_000) {
            let (t, t_fine) = c.temperature(t_adc);
            let (t_int, t_fine_int) = temperature_int(&c, t_adc);
            assert!(
                (t * 100.0 - f64::from(t_int)).abs() < 1.5,
                "{t_adc}: {t} °C against {t_int}"
            );
            assert!((t_fine - f64::from(t_fine_int)).abs() < 40.0);
            for p_adc in (250_000u32..600_000).step_by(25_000) {
                let p = c.pressure(p_adc, t_fine);
                // Rooms only: the C integer code overflows outside them.
                if !(80_000.0..110_000.0).contains(&p) {
                    continue;
                }
                let p_int = pressure_int(&c, p_adc, t_fine_int);
                let d = p - f64::from(p_int);
                worst = worst.max(d.abs());
                checked += 1;
            }
        }
        assert!(checked > 40, "{checked}");
        // The integer code truncates in its shifts: a sawtooth of a few pascals, no trend.
        assert!(worst < 6.0, "{worst}");
        // The middle of the range is a room: about 25 °C and 1000 hPa.
        let (t, t_fine) = c.temperature(500_000);
        assert!((20.0..30.0).contains(&t), "{t}");
        let p = c.pressure(330_000, t_fine);
        assert!((90_000.0..110_000.0).contains(&p), "{p}");
    }

    #[test]
    fn the_calibration_blocks_parse_as_the_sensor_api_concatenates_them() {
        let want = calibration();
        let mut c = [0u8; 42];
        let put16 = |c: &mut [u8; 42], msb: usize, lsb: usize, v: u16| {
            c[msb] = (v >> 8) as u8;
            c[lsb] = v as u8;
        };
        put16(&mut c, 32, 31, want.t1);
        put16(&mut c, 1, 0, want.t2 as u16);
        c[2] = want.t3 as u8;
        put16(&mut c, 5, 4, want.p1);
        put16(&mut c, 7, 6, want.p2 as u16);
        c[8] = want.p3 as u8;
        put16(&mut c, 11, 10, want.p4 as u16);
        put16(&mut c, 13, 12, want.p5 as u16);
        c[14] = want.p7 as u8;
        c[15] = want.p6 as u8;
        put16(&mut c, 19, 18, want.p8 as u16);
        put16(&mut c, 21, 20, want.p9 as u16);
        c[22] = want.p10;
        c[23] = (want.h2 >> 4) as u8;
        c[24] = ((want.h2 as u8 & 0x0F) << 4) | (want.h1 as u8 & 0x0F);
        c[25] = (want.h1 >> 4) as u8;
        c[26] = want.h3 as u8;
        c[27] = want.h4 as u8;
        c[28] = want.h5 as u8;
        c[29] = want.h6;
        c[30] = want.h7 as u8;
        let got = Calibration::from_blocks(
            c[..23].try_into().unwrap(),
            c[23..37].try_into().unwrap(),
            c[37..].try_into().unwrap(),
        );
        assert_eq!(got, want);
    }

    #[test]
    fn field0_needs_new_data_and_unpacks_twenty_bits() {
        let mut f = [0u8; FIELD0_LEN];
        assert_eq!(raw(&f), None);
        f[0] = 0x80;
        f[2..5].copy_from_slice(&[0x69, 0xF8, 0x30]);
        f[5..8].copy_from_slice(&[0x7A, 0x12, 0x00]);
        f[8..10].copy_from_slice(&[0x5A, 0x3C]);
        assert_eq!(raw(&f), Some((0x7A120, 0x69F83, 0x5A3C)));
        assert!(calibration().sample(&f).is_some());
    }

    #[test]
    fn a_pascal_is_about_eight_centimetres() {
        let h = height_above_m(101_325.0, 101_313.0, 20.0);
        assert!((h - 1.0).abs() < 0.03, "{h}");
        assert!(height_above_m(101_313.0, 101_325.0, 20.0) < 0.0);
    }

    #[test]
    fn the_average_keeps_its_window_and_follows_a_shorter_one() {
        let mut a = Average::<8>::new();
        assert_eq!(a.mean(), None);
        for v in [1.0, 2.0, 3.0, 4.0, 5.0, 6.0] {
            a.push(v, 4);
        }
        assert_eq!(a.mean(), Some((4.5, 4)));
        a.push(7.0, 2);
        assert_eq!(a.mean(), Some((6.5, 2)));
        a.push(8.0, 8);
        assert_eq!(a.mean(), Some((7.0, 3)));
        let s = Settings::BAROMETRY;
        assert_eq!(s.ctrl_meas_forced(), (2 << 5) | (5 << 2) | 1);
        assert!(s.duration_us() > 30_000 && s.duration_us() < 50_000);
    }
}
