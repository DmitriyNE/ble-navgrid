//! Synthetic CS channel responses for tests and host-side simulation.

use core::f64::consts::TAU;

use crate::{ChannelResponse, Complex, NUM_CHANNELS, SPEED_OF_LIGHT, channel_frequency_hz};

/// Deterministic PRNG (SplitMix64) so simulations reproduce without pulling in `rand`.
pub struct SplitMix64(u64);

impl SplitMix64 {
    pub const fn new(seed: u64) -> Self {
        Self(seed)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`.
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Standard normal (Box–Muller).
    pub fn normal(&mut self) -> f64 {
        let u1 = 1.0 - self.uniform();
        let u2 = self.uniform();
        libm::sqrt(-2.0 * libm::log(u1)) * libm::cos(TAU * u2)
    }
}

/// One propagation path between the two radios.
#[derive(Clone, Copy, Debug)]
pub struct Path {
    /// One-way path length.
    pub length_m: f64,
    pub amplitude: f64,
}

/// Simulates one procedure over all 72 CS channels.
///
/// Both sides see the same reciprocal one-way channel `H₁(f) = Σ a·e^{−j2πf·ℓ/c}`, rotated by
/// opposite LO phase offsets that are random per procedure. Each side also gets independent
/// Gaussian phase noise per tone. The product cancels the LO offsets, as on hardware.
pub fn response(paths: &[Path], phase_noise_rad: f64, rng: &mut SplitMix64) -> ChannelResponse {
    let lo_offset = TAU * rng.uniform();
    let mut resp = ChannelResponse::new();
    for channel in 0..NUM_CHANNELS as u8 {
        let Some(f) = channel_frequency_hz(channel) else {
            continue;
        };
        let (mut re, mut im) = (0.0, 0.0);
        for p in paths {
            let phase = -TAU * f * p.length_m / f64::from(SPEED_OF_LIGHT);
            re += p.amplitude * libm::cos(phase);
            im += p.amplitude * libm::sin(phase);
        }
        let one_way = Complex::new(re as f32, im as f32);
        let side = |lo: f64, rng: &mut SplitMix64| {
            one_way * Complex::from_polar(1.0, (lo + phase_noise_rad * rng.normal()) as f32)
        };
        let initiator = side(lo_offset, rng);
        let reflector = side(-lo_offset, rng);
        resp.insert_pct(channel, initiator, reflector);
    }
    resp
}
