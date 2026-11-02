//! Channel Sounding step data → distance.
//!
//! CS channel index `k` is at `2402 + k` MHz. Channels 0, 1, 23, 24, 25, 77 and 78 surround the
//! advertising channels and are never used, which leaves 72.
//!
//! Phase-based ranging works on the combined response `H[k] = PCT_init[k] · PCT_refl[k]`. Each side
//! measures the other's tone against its own local oscillator; multiplying the two cancels both LO
//! phases and leaves the round-trip phase `−4π·f·d/c`.

#![no_std]

pub mod sim;
pub mod steps;

use core::f32::consts::{PI, TAU};
use core::ops::{Add, AddAssign, Mul};

pub const SPEED_OF_LIGHT: f32 = 299_792_458.0;
pub const NUM_CHANNELS: usize = 79;
pub const CHANNEL_SPACING_HZ: f32 = 1.0e6;
const BASE_FREQUENCY_HZ: f64 = 2.402e9;

/// Unambiguous range of [`phase_slope`]: `c / (2·Δf)` ≈ 150 m.
pub const PHASE_SLOPE_MAX_RANGE_M: f32 = SPEED_OF_LIGHT / (2.0 * CHANNEL_SPACING_HZ);

pub fn is_cs_channel(channel: u8) -> bool {
    matches!(channel, 2..=22 | 26..=76)
}

pub fn channel_frequency_hz(channel: u8) -> Option<f64> {
    is_cs_channel(channel).then(|| BASE_FREQUENCY_HZ + f64::from(channel) * f64::from(CHANNEL_SPACING_HZ))
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Complex {
    pub re: f32,
    pub im: f32,
}

impl Complex {
    pub const ZERO: Self = Self::new(0.0, 0.0);

    pub const fn new(re: f32, im: f32) -> Self {
        Self { re, im }
    }

    pub fn from_polar(magnitude: f32, phase: f32) -> Self {
        Self::new(magnitude * libm::cosf(phase), magnitude * libm::sinf(phase))
    }

    pub fn conj(self) -> Self {
        Self::new(self.re, -self.im)
    }

    pub fn abs(self) -> f32 {
        libm::hypotf(self.re, self.im)
    }

    pub fn arg(self) -> f32 {
        libm::atan2f(self.im, self.re)
    }
}

impl Add for Complex {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.re + rhs.re, self.im + rhs.im)
    }
}

impl AddAssign for Complex {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl Mul for Complex {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self::new(self.re * rhs.re - self.im * rhs.im, self.re * rhs.im + self.im * rhs.re)
    }
}

/// Combined round-trip response per CS channel for one procedure (one antenna path).
#[derive(Clone, Debug)]
pub struct ChannelResponse {
    h: [Option<Complex>; NUM_CHANNELS],
}

impl Default for ChannelResponse {
    fn default() -> Self {
        Self::new()
    }
}

impl ChannelResponse {
    pub const fn new() -> Self {
        Self {
            h: [None; NUM_CHANNELS],
        }
    }

    /// Adds one combined tone. Repeats on the same channel are summed (coherent average).
    /// Returns `false` and ignores the tone if `channel` is not a CS channel.
    pub fn insert(&mut self, channel: u8, h: Complex) -> bool {
        if !is_cs_channel(channel) {
            return false;
        }
        let slot = &mut self.h[usize::from(channel)];
        *slot = Some(slot.unwrap_or(Complex::ZERO) + h);
        true
    }

    /// Adds one tone from the initiator's and reflector's phase correction terms.
    pub fn insert_pct(&mut self, channel: u8, initiator: Complex, reflector: Complex) -> bool {
        self.insert(channel, initiator * reflector)
    }

    pub fn get(&self, channel: u8) -> Option<Complex> {
        self.h.get(usize::from(channel)).copied().flatten()
    }

    pub fn len(&self) -> usize {
        self.h.iter().filter(|h| h.is_some()).count()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PhaseSlope {
    pub distance_m: f32,
    /// `|Σ H[k+1]·H*[k]| / Σ |H[k+1]|·|H[k]|` in `0..=1`. Tracks tone-to-tone noise; multipath
    /// barely moves it, because its phase ripple is slow across 1 MHz.
    pub coherence: f32,
    /// Magnitude-weighted RMS of phase residuals around the fitted line, radians. Multipath
    /// ripple across the band shows up here, so this is the main quality signal.
    pub residual_rad: f32,
    /// Adjacent-channel pairs that contributed.
    pub pairs: u8,
}

/// Distance from the slope of round-trip phase against frequency.
///
/// Stage 1 takes the angle of `Σ H[k+1]·H*[k]` over adjacent 1 MHz pairs, which needs no phase
/// unwrapping and is unambiguous over [`PHASE_SLOPE_MAX_RANGE_M`]. Stage 2 removes that coarse
/// slope and refines it with a magnitude-weighted least-squares fit across all channels, which
/// uses the full 74 MHz aperture instead of the 1 MHz pair baseline.
///
/// Returns `None` with fewer than `min_pairs` adjacent pairs.
pub fn phase_slope(resp: &ChannelResponse, min_pairs: u8) -> Option<PhaseSlope> {
    let mut pair_sum = Complex::ZERO;
    let mut pair_weight = 0.0;
    let mut pairs = 0u8;
    for k in 0..NUM_CHANNELS - 1 {
        if let (Some(a), Some(b)) = (resp.h[k], resp.h[k + 1]) {
            pair_sum += b * a.conj();
            pair_weight += a.abs() * b.abs();
            pairs += 1;
        }
    }
    if pairs < min_pairs.max(1) || pair_weight <= 0.0 {
        return None;
    }
    let coarse = pair_sum.arg();

    // Derotate by the coarse slope; what remains is a small residual phase around a common offset.
    let derotated = |k: usize, h: Complex| h * Complex::from_polar(1.0, -coarse * k as f32);
    let mut offset = Complex::ZERO;
    for (k, h) in resp.h.iter().enumerate() {
        if let Some(h) = *h {
            offset += derotated(k, h);
        }
    }
    let offset = Complex::from_polar(1.0, -offset.arg());
    let residual_phase = |k: usize, h: Complex| (derotated(k, h) * offset).arg();

    let (mut sw, mut swk, mut swp, mut swkk, mut swkp) = (0.0f32, 0.0f32, 0.0f32, 0.0f32, 0.0f32);
    for (k, h) in resp.h.iter().enumerate() {
        if let Some(h) = *h {
            let w = h.abs();
            let kf = k as f32;
            let psi = residual_phase(k, h);
            sw += w;
            swk += w * kf;
            swp += w * psi;
            swkk += w * kf * kf;
            swkp += w * kf * psi;
        }
    }
    let denom = sw * swkk - swk * swk;
    let refinement = if denom > 0.0 {
        (sw * swkp - swk * swp) / denom
    } else {
        0.0
    };
    let intercept = (swp - refinement * swk) / sw;

    let mut sq = 0.0;
    for (k, h) in resp.h.iter().enumerate() {
        if let Some(h) = *h {
            let e = residual_phase(k, h) - intercept - refinement * k as f32;
            sq += h.abs() * e * e;
        }
    }

    // Round-trip phase falls by 4π·Δf·d/c per channel.
    let mut per_channel = -(coarse + refinement);
    if per_channel < 0.0 {
        per_channel += TAU;
    }
    Some(PhaseSlope {
        distance_m: per_channel * SPEED_OF_LIGHT / (4.0 * PI * CHANNEL_SPACING_HZ),
        coherence: pair_sum.abs() / pair_weight,
        residual_rad: libm::sqrtf(sq / sw),
        pairs,
    })
}

/// 1-σ uncertainty of a phase-slope range: `0.05 m + 2 m/rad × residual`. A placeholder until
/// real captures calibrate it (DEF-7); firmware and host use the same one.
pub fn range_sigma_m(est: &PhaseSlope) -> f32 {
    0.05 + 2.0 * est.residual_rad
}

/// Range quality, 0 (unusable) to 255 (best), as `navgrid_proto::Range::quality` carries it: the
/// estimate's coherence scaled to a byte.
pub fn range_quality(est: &PhaseSlope) -> u8 {
    (est.coherence.clamp(0.0, 1.0) * 255.0 + 0.5) as u8
}

/// Mean one-way distance from mode-1 RTT steps.
///
/// Each item is `(toa_tod_initiator, tod_toa_reflector)` from CS step data, in units of 0.5 ns:
/// the initiator's ToA−ToD (round trip plus reflector turnaround) and the reflector's ToD−ToA
/// (turnaround). Units and sign follow the Core 6.0 step-data format; P0 confirms them on hardware.
pub fn rtt_distance_m(steps: impl IntoIterator<Item = (i16, i16)>) -> Option<f32> {
    let (mut sum_half_ns, mut n) = (0i64, 0u32);
    for (initiator, reflector) in steps {
        sum_half_ns += i64::from(initiator) - i64::from(reflector);
        n += 1;
    }
    (n > 0).then(|| {
        let round_trip_s = sum_half_ns as f32 / n as f32 * 0.5e-9;
        round_trip_s * SPEED_OF_LIGHT / 2.0
    })
}

/// How per-antenna-path estimates become one range.
///
/// In simulation (`navgrid sim --trials`) no strategy wins everywhere: [`Combine::Mean`] is best
/// when reflections are only 1–3 m longer than the direct path, [`Combine::LowestResidual`] when
/// they are ~6 m longer, and [`Combine::ResidualWeighted`] is never the worst, so it is the
/// default until real captures decide.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Combine {
    /// First path only: what a link without diversity gets.
    First,
    /// Path with the lowest phase-fit residual (selection diversity).
    LowestResidual,
    /// Plain mean of the paths' distances.
    Mean,
    /// Mean weighted by `1 / residual²`.
    #[default]
    ResidualWeighted,
}

/// Combines per-antenna-path estimates of one procedure. Averaging strategies report the mean
/// coherence and residual of their inputs and the smallest pair count.
pub fn combine_paths(paths: &[PhaseSlope], how: Combine) -> Option<PhaseSlope> {
    let first = *paths.first()?;
    let weight = |p: &PhaseSlope| match how {
        Combine::ResidualWeighted => 1.0 / (p.residual_rad * p.residual_rad + 1e-6),
        _ => 1.0,
    };
    match how {
        Combine::First => Some(first),
        Combine::LowestResidual => paths
            .iter()
            .copied()
            .min_by(|a, b| a.residual_rad.total_cmp(&b.residual_rad)),
        Combine::Mean | Combine::ResidualWeighted => {
            let n = paths.len() as f32;
            let total: f32 = paths.iter().map(weight).sum();
            Some(PhaseSlope {
                distance_m: paths.iter().map(|p| weight(p) * p.distance_m).sum::<f32>() / total,
                coherence: paths.iter().map(|p| p.coherence).sum::<f32>() / n,
                residual_rad: paths.iter().map(|p| p.residual_rad).sum::<f32>() / n,
                pairs: paths.iter().map(|p| p.pairs).min().unwrap_or(0),
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::sim::{Path, SplitMix64, response};
    use super::*;

    fn estimate(paths: &[Path], phase_noise_rad: f64, seed: u64) -> PhaseSlope {
        let resp = response(paths, phase_noise_rad, &mut SplitMix64::new(seed));
        phase_slope(&resp, 20).unwrap()
    }

    #[test]
    fn channel_map_has_72_channels() {
        assert_eq!((0..=u8::MAX).filter(|&c| is_cs_channel(c)).count(), 72);
        for c in [0, 1, 23, 24, 25, 77, 78] {
            assert_eq!(channel_frequency_hz(c), None);
        }
        assert_eq!(channel_frequency_hz(2), Some(2.404e9));
    }

    #[test]
    fn clean_single_path_is_exact_across_range() {
        for d in [0.3, 1.0, 7.3, 42.0, 149.0] {
            let est = estimate(
                &[Path {
                    length_m: d,
                    amplitude: 1.0,
                }],
                0.0,
                1,
            );
            assert!(libm::fabsf(est.distance_m - d as f32) < 0.005, "d={d} est={est:?}");
            assert!(est.coherence > 0.999);
            assert_eq!(est.pairs, 70);
        }
    }

    #[test]
    fn phase_noise_averages_out() {
        let est = estimate(
            &[Path {
                length_m: 12.0,
                amplitude: 1.0,
            }],
            0.1,
            7,
        );
        assert!(libm::fabsf(est.distance_m - 12.0) < 0.1, "{est:?}");
    }

    #[test]
    fn multipath_shows_in_residual_not_coherence() {
        let clean = estimate(
            &[Path {
                length_m: 10.0,
                amplitude: 1.0,
            }],
            0.0,
            3,
        );
        let reflected = estimate(
            &[
                Path {
                    length_m: 10.0,
                    amplitude: 1.0,
                },
                Path {
                    length_m: 13.0,
                    amplitude: 0.6,
                },
            ],
            0.0,
            3,
        );
        // The reflection biases the estimate long by well over a metre...
        assert!(reflected.distance_m - 10.0 > 1.0, "{reflected:?}");
        // ...while coherence stays near 1, so it cannot flag the problem.
        assert!(reflected.coherence > 0.999, "{reflected:?}");
        assert!(clean.residual_rad < 0.01, "{clean:?}");
        assert!(reflected.residual_rad > 0.1, "{reflected:?}");
    }

    #[test]
    fn combine_paths_strategies() {
        let path = |distance_m, residual_rad| PhaseSlope {
            distance_m,
            coherence: 1.0,
            residual_rad,
            pairs: 70,
        };
        let paths = [path(10.0, 0.4), path(11.0, 0.1), path(12.0, 0.2)];
        let d = |how| combine_paths(&paths, how).unwrap().distance_m;
        assert_eq!(d(Combine::First), 10.0);
        assert_eq!(d(Combine::LowestResidual), 11.0);
        assert!(libm::fabsf(d(Combine::Mean) - 11.0) < 1e-5);
        // Weights 1/r²: 6.25, 100, 25 → pulled toward the 0.1 rad path.
        let weighted = d(Combine::ResidualWeighted);
        assert!(weighted > 11.0 && weighted < 11.2, "{weighted}");
        assert_eq!(combine_paths(&[], Combine::Mean), None);
    }

    #[test]
    fn too_few_pairs_is_none() {
        let mut resp = ChannelResponse::new();
        resp.insert(10, Complex::new(1.0, 0.0));
        resp.insert(11, Complex::new(0.0, 1.0));
        assert!(phase_slope(&resp, 2).is_none());
        assert!(!resp.insert(24, Complex::new(1.0, 0.0)));
    }

    #[test]
    fn rtt_converts_half_nanoseconds() {
        // 10 m each way is a 66.7 ns round trip, 133 half-ns units; turnaround cancels.
        let rtt_half_ns = 133i16;
        let d = rtt_distance_m([(1000 + rtt_half_ns, 1000), (1200 + rtt_half_ns, 1200)]).unwrap();
        assert!(libm::fabsf(d - 10.0) < 0.05, "{d}");
        assert_eq!(rtt_distance_m([]), None);
    }
}
