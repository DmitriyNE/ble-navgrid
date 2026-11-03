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

/// The first path in the delay profile: what the phase slope can't give under multipath.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FirstPath {
    /// The earliest of the paths found, refined between grid points.
    pub distance_m: f32,
    /// Its power relative to the strongest path's, in `0..=1`; 1 when it is the strongest.
    pub strength: f32,
    /// Where the strongest path is: the phase slope's answer, roughly, under one dominant path.
    pub strongest_m: f32,
    /// Paths found at or above the threshold.
    pub peaks: u8,
    /// Every path found, strongest first: distance and power relative to the strongest.
    pub components: [(f32, f32); FIRST_PATH_MAX_PATHS as usize],
}

/// How far the profile is searched. Beyond it a reflection is weak and the grid costly.
pub const FIRST_PATH_MAX_RANGE_M: f32 = 60.0;
/// The grid step; a peak is refined to a fraction of it.
pub const FIRST_PATH_STEP_M: f32 = 0.05;
/// Paths taken out of the tones one by one, strongest first.
const FIRST_PATH_MAX_PATHS: u8 = 4;
/// The default `threshold` of [`first_path`]: −25 dB, below a direct path 10 dB under a
/// reflection (20 dB down round trip) and above the Hann sidelobes (−31 dB).
pub const FIRST_PATH_THRESHOLD: f32 = 0.003;
/// A path must lower the joint residual by this fraction of the tones' energy, or it's noise.
pub const FIRST_PATH_MIN_GAIN: f32 = 0.002;

/// The window over the band for the delay profile. Hann trades a wider main lobe for sidelobes
/// 31 dB down, so a weak path beside a strong one is a peak of its own rather than a sidelobe.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Window {
    Rectangular,
    #[default]
    Hann,
}

/// Round-trip phase of a path at `d` on channel `k`: `−4πfd/c`, as the tones carry it.
fn path_phase(k: usize, d: f32) -> Option<f32> {
    let f = channel_frequency_hz(k as u8)?;
    let turns = 2.0 * f * f64::from(d) / f64::from(SPEED_OF_LIGHT);
    Some((-core::f64::consts::TAU * (turns % 1.0)) as f32)
}

fn weight(window: Window, k: usize) -> f32 {
    match window {
        Window::Rectangular => 1.0,
        Window::Hann => {
            let x = (k as f32 - 2.0) / 74.0;
            0.5 * (1.0 - libm::cosf(TAU * x))
        }
    }
}

/// The windowed matched-filter output at `d`: the tones undone by a path's phase there, summed.
fn matched(resp: &ChannelResponse, window: Window, d: f32) -> Complex {
    let mut sum = Complex::ZERO;
    for (k, h) in resp.h.iter().enumerate() {
        let (Some(h), Some(phase)) = (h, path_phase(k, d)) else {
            continue;
        };
        sum += *h * Complex::from_polar(weight(window, k), -phase);
    }
    sum
}

fn power(c: Complex) -> f32 {
    c.re * c.re + c.im * c.im
}

/// The strongest peak of the profile of `resp`, refined: its distance and power.
fn strongest_peak(resp: &ChannelResponse, window: Window) -> Option<(f32, f32)> {
    let steps = (FIRST_PATH_MAX_RANGE_M / FIRST_PATH_STEP_M) as usize;
    let (mut best, mut best_i) = (0.0f32, 0usize);
    for i in 0..=steps {
        let p = power(matched(resp, window, i as f32 * FIRST_PATH_STEP_M));
        if p > best {
            best = p;
            best_i = i;
        }
    }
    if best <= 0.0 {
        return None;
    }
    // Parabolic interpolation over the neighbours.
    let at = |i: usize| power(matched(resp, window, i as f32 * FIRST_PATH_STEP_M));
    let offset = if best_i == 0 || best_i == steps {
        0.0
    } else {
        let (a, c) = (at(best_i - 1), at(best_i + 1));
        let denom = a - 2.0 * best + c;
        if denom < 0.0 { 0.5 * (a - c) / denom } else { 0.0 }
    };
    Some(((best_i as f32 + offset) * FIRST_PATH_STEP_M, best))
}

impl Complex {
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.re - rhs.re, self.im - rhs.im)
    }

    fn scale(self, s: f32) -> Self {
        Self::new(self.re * s, self.im * s)
    }

    fn div(self, rhs: Self) -> Self {
        let d = rhs.re * rhs.re + rhs.im * rhs.im;
        (self * rhs.conj()).scale(1.0 / d)
    }
}

const MAX_PATHS: usize = FIRST_PATH_MAX_PATHS as usize;

/// The path amplitudes that fit `resp` best, jointly, for paths at `ds`: least squares over the
/// tones, by the normal equations solved with Gaussian elimination. Returns the amplitudes and
/// the residual energy.
fn fit_amplitudes(resp: &ChannelResponse, ds: &[f32]) -> ([Complex; MAX_PATHS], f32) {
    let n = ds.len();
    let mut g = [[Complex::ZERO; MAX_PATHS]; MAX_PATHS];
    let mut b = [Complex::ZERO; MAX_PATHS];
    for (k, h) in resp.h.iter().enumerate() {
        let Some(h) = h else { continue };
        let mut atoms = [Complex::ZERO; MAX_PATHS];
        for (i, &d) in ds.iter().enumerate() {
            if let Some(phase) = path_phase(k, d) {
                atoms[i] = Complex::from_polar(1.0, phase);
            }
        }
        for i in 0..n {
            b[i] += atoms[i].conj() * *h;
            for j in 0..n {
                g[i][j] += atoms[i].conj() * atoms[j];
            }
        }
    }
    // Gaussian elimination with partial pivoting.
    for col in 0..n {
        let pivot = (col..n)
            .max_by(|&a, &c| g[a][col].abs().total_cmp(&g[c][col].abs()))
            .unwrap_or(col);
        g.swap(col, pivot);
        b.swap(col, pivot);
        let p = g[col][col];
        if p.abs() < 1e-9 {
            continue;
        }
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = g[row][col].div(p);
            let pivot_row = g[col];
            for (cell, &above) in g[row][..n].iter_mut().zip(&pivot_row[..n]) {
                *cell = cell.sub(factor * above);
            }
            b[row] = b[row].sub(factor * b[col]);
        }
    }
    let mut a = [Complex::ZERO; MAX_PATHS];
    for i in 0..n {
        if g[i][i].abs() >= 1e-9 {
            a[i] = b[i].div(g[i][i]);
        }
    }
    (a, residual_energy(resp, ds, &a))
}

/// `resp` less the paths at `ds` with amplitudes `a`.
fn residual(resp: &ChannelResponse, ds: &[f32], a: &[Complex; MAX_PATHS]) -> ChannelResponse {
    let mut r = resp.clone();
    for (k, h) in r.h.iter_mut().enumerate() {
        let Some(h) = h else { continue };
        for (i, &d) in ds.iter().enumerate() {
            if let Some(phase) = path_phase(k, d) {
                *h = h.sub(a[i] * Complex::from_polar(1.0, phase));
            }
        }
    }
    r
}

fn residual_energy(resp: &ChannelResponse, ds: &[f32], a: &[Complex; MAX_PATHS]) -> f32 {
    residual(resp, ds, a).h.iter().flatten().map(|h| power(*h)).sum()
}

/// Local refinement: each path's distance in turn over ±0.5 m in 1 cm steps, keeping what
/// lowers the joint residual.
fn refine(resp: &ChannelResponse, ds: &mut [f32]) {
    const HALF_SPAN: f32 = 0.5;
    const STEP: f32 = 0.01;
    for _ in 0..2 {
        for i in 0..ds.len() {
            let centre = ds[i];
            let (mut best_d, mut best_e) = (centre, fit_amplitudes(resp, ds).1);
            let steps = (HALF_SPAN / STEP) as i32;
            for j in -steps..=steps {
                let d = centre + j as f32 * STEP;
                if d < 0.0 {
                    continue;
                }
                ds[i] = d;
                let e = fit_amplitudes(resp, ds).1;
                if e < best_e {
                    best_e = e;
                    best_d = d;
                }
            }
            ds[i] = best_d;
        }
    }
}

/// The first path among those in `resp`: paths are found strongest first in the residual of
/// the ones so far, all fitted jointly by least squares and each refined against the joint
/// residual, which separates paths closer than the profile's main lobe (about 4 m with Hann over
/// the 74 MHz band). A path counts while its peak is at least `threshold` (in `0..1`) of the
/// strongest one's power and taking it out lowers the residual by at least
/// [`FIRST_PATH_MIN_GAIN`] of the tones' energy. Under one path it agrees with [`phase_slope`]
/// to a few centimetres; under multipath it finds the direct path where the phase slope reads
/// the mix. `None` with fewer than `min_channels` tones.
///
/// The round-trip response is the one-way channel squared, as each side's tone crosses the same
/// channel: two one-way paths at `ℓ₁ < ℓ₂` with amplitudes `a₁, a₂` give round-trip components
/// at `ℓ₁` (`a₁²`), at their mean (`2a₁a₂`) and at `ℓ₂` (`a₂²`). A direct path 10 dB below a
/// reflection is 20 dB down here, so `threshold` has to be small: [`FIRST_PATH_THRESHOLD`].
pub fn first_path(resp: &ChannelResponse, window: Window, threshold: f32, min_channels: usize) -> Option<FirstPath> {
    if resp.len() < min_channels {
        return None;
    }
    let total: f32 = resp.h.iter().flatten().map(|h| power(*h)).sum();
    let mut ds = [0.0f32; MAX_PATHS];
    let mut n = 0usize;
    let mut amplitudes = [Complex::ZERO; MAX_PATHS];
    let mut energy = total;
    let mut strongest_power = 0.0f32;
    while n < MAX_PATHS {
        let r = residual(resp, &ds[..n], &amplitudes);
        let Some((d, p)) = strongest_peak(&r, window) else {
            break;
        };
        if n == 0 {
            strongest_power = p;
        } else if p < threshold * strongest_power {
            break;
        }
        ds[n] = d;
        refine(resp, &mut ds[..=n]);
        let (a, e) = fit_amplitudes(resp, &ds[..=n]);
        if n > 0 && energy - e < FIRST_PATH_MIN_GAIN * total {
            break;
        }
        n += 1;
        amplitudes = a;
        energy = e;
    }
    if n == 0 {
        return None;
    }
    let strength_of = |i: usize| power(amplitudes[i]);
    let strongest = (0..n).max_by(|&a, &b| strength_of(a).total_cmp(&strength_of(b)))?;
    let first = (0..n).min_by(|&a, &b| ds[a].total_cmp(&ds[b]))?;
    let mut components = [(0.0f32, 0.0f32); MAX_PATHS];
    for (i, c) in components.iter_mut().enumerate().take(n) {
        *c = (ds[i], strength_of(i) / strength_of(strongest));
    }
    Some(FirstPath {
        distance_m: ds[first],
        strength: strength_of(first) / strength_of(strongest),
        strongest_m: ds[strongest],
        peaks: n as u8,
        components,
    })
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
    fn first_path_matches_single_path() {
        for d in [0.3, 1.0, 7.3, 42.0] {
            let resp = response(
                &[Path {
                    length_m: d,
                    amplitude: 1.0,
                }],
                0.0,
                &mut SplitMix64::new(1),
            );
            let fp = first_path(&resp, Window::Hann, FIRST_PATH_THRESHOLD, 20).unwrap();
            assert!(libm::fabsf(fp.distance_m - d as f32) < 0.05, "d={d} {fp:?}");
            assert_eq!(fp.peaks, 1, "{fp:?}");
            assert!(fp.strength > 0.999);
        }
    }

    #[test]
    fn first_path_finds_the_direct_path_under_a_reflection() {
        let paths = [
            Path {
                length_m: 10.0,
                amplitude: 1.0,
            },
            Path {
                length_m: 15.0,
                amplitude: 0.6,
            },
        ];
        let resp = response(&paths, 0.02, &mut SplitMix64::new(3));
        let slope = phase_slope(&resp, 20).unwrap();
        let fp = first_path(&resp, Window::Hann, FIRST_PATH_THRESHOLD, 20).unwrap();
        assert!(libm::fabsf(fp.distance_m - 10.0) < 0.3, "{fp:?}");
        // The direct path, the cross term at 12.5 m and the reflection.
        assert_eq!(fp.peaks, 3, "{fp:?}");
        // The phase slope reads the mix.
        assert!(libm::fabsf(slope.distance_m - 10.0) > 0.3, "{slope:?}");
    }

    #[test]
    fn first_path_survives_a_stronger_reflection() {
        // The floor: a weak direct path and a strong bounce, as the bench saw.
        let paths = [
            Path {
                length_m: 1.5,
                amplitude: 0.3,
            },
            Path {
                length_m: 6.0,
                amplitude: 1.0,
            },
        ];
        let resp = response(&paths, 0.02, &mut SplitMix64::new(5));
        let fp = first_path(&resp, Window::Hann, FIRST_PATH_THRESHOLD, 20).unwrap();
        assert!(libm::fabsf(fp.distance_m - 1.5) < 0.3, "{fp:?}");
        assert!(libm::fabsf(fp.strongest_m - 6.0) < 0.3, "{fp:?}");
        // Round trip, the direct path's power is (0.3/1.0)⁴ of the reflection's.
        assert!(fp.strength < 0.02 && fp.strength > 0.004, "{fp:?}");
        assert_eq!(fp.peaks, 3, "{fp:?}");
    }

    #[test]
    fn first_path_needs_channels() {
        let mut resp = ChannelResponse::new();
        resp.insert(10, Complex::new(1.0, 0.0));
        assert!(first_path(&resp, Window::Hann, FIRST_PATH_THRESHOLD, 20).is_none());
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
