//! Shared simulation scenario: room layout, CS channel model, and the range σ model.

use anyhow::{Context, Result};
use clap::ValueEnum;
use navgrid_ranging::sim::{self, Path, SplitMix64};
use navgrid_ranging::{Combine, PhaseSlope};
use navgrid_solver::Vec3;

/// 10 × 8 m room, nodes at mixed heights so z is observable.
pub const NODES: [Vec3; 6] = [
    [0.0, 0.0, 2.7],
    [10.0, 0.0, 0.4],
    [10.0, 8.0, 2.7],
    [0.0, 8.0, 0.4],
    [5.0, 0.0, 1.5],
    [5.0, 8.0, 2.0],
];

/// Carrier wavelength mid-band; antenna spacing on the order of this decorrelates multipath.
const WAVELENGTH_M: f64 = 0.123;

#[derive(clap::Args)]
pub struct ChannelArgs {
    /// Per-side, per-tone phase noise, radians (1σ).
    #[arg(long, default_value_t = 0.05)]
    pub phase_noise: f64,
    /// Add a reflection this many metres longer than the direct path (amplitude up to 0.6).
    #[arg(long)]
    pub multipath: Option<f64>,
    /// Antenna paths per link (1–4). Each path sees the reflection with its own phase and strength.
    #[arg(long, default_value_t = 1, value_parser = clap::value_parser!(u8).range(1..=4))]
    pub antenna_paths: u8,
    /// How per-path estimates are combined into one range.
    #[arg(long, value_enum, default_value_t = CombineArg::ResidualWeighted)]
    pub combine: CombineArg,
}

#[derive(Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum CombineArg {
    First,
    LowestResidual,
    Mean,
    ResidualWeighted,
    ShortestCoherent,
}

impl From<CombineArg> for Combine {
    fn from(c: CombineArg) -> Self {
        match c {
            CombineArg::First => Combine::First,
            CombineArg::LowestResidual => Combine::LowestResidual,
            CombineArg::Mean => Combine::Mean,
            CombineArg::ResidualWeighted => Combine::ResidualWeighted,
            CombineArg::ShortestCoherent => Combine::ShortestCoherent,
        }
    }
}

/// Per-antenna-path estimates for one simulated CS procedure over `truth` metres.
pub fn procedure(args: &ChannelArgs, truth: f32, rng: &mut SplitMix64) -> Result<Vec<PhaseSlope>> {
    (0..args.antenna_paths)
        .map(|_| {
            let mut channel = vec![Path {
                length_m: f64::from(truth),
                amplitude: 1.0,
            }];
            if let Some(extra) = args.multipath {
                channel.push(Path {
                    length_m: f64::from(truth) + extra + WAVELENGTH_M * rng.uniform(),
                    amplitude: 0.6 * (0.3 + 0.7 * rng.uniform()),
                });
            }
            let resp = sim::response(&channel, args.phase_noise, rng);
            navgrid_ranging::phase_slope(&resp, 20).context("too few channels")
        })
        .collect()
}

/// One combined range and its σ.
pub fn range(args: &ChannelArgs, truth: f32, rng: &mut SplitMix64) -> Result<(PhaseSlope, f32)> {
    let paths = procedure(args, truth, rng)?;
    let est = navgrid_ranging::combine_paths(&paths, args.combine.into()).context("no antenna paths")?;
    Ok((est, navgrid_ranging::range_sigma_m(&est)))
}

pub fn distance(a: Vec3, b: Vec3) -> f32 {
    a.iter().zip(b).map(|(p, q)| (p - q) * (p - q)).sum::<f32>().sqrt()
}

pub fn horizontal_distance(a: Vec3, b: Vec3) -> f32 {
    distance([a[0], a[1], 0.0], [b[0], b[1], 0.0])
}

pub fn centroid() -> Vec3 {
    let n = NODES.len() as f32;
    std::array::from_fn(|i| NODES.iter().map(|p| p[i]).sum::<f32>() / n)
}

/// Value at quantile `q` of an already sorted slice.
pub fn quantile(sorted: &[f32], q: f32) -> f32 {
    sorted[((sorted.len() - 1) as f32 * q) as usize]
}
