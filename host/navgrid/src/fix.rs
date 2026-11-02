//! `navgrid sim`: one-shot fixes from simulated CS ranges.

use anyhow::{Context, Result, bail};
use clap::ValueEnum;
use navgrid_ranging::sim::SplitMix64;
use navgrid_ranging::{Combine, PhaseSlope};
use navgrid_solver::{Fix, Height, Options, RangeObs, Vec3};

use crate::scenario::{self, ChannelArgs, CombineArg, NODES};

#[derive(clap::Args)]
pub struct SimArgs {
    /// Target position "x,y,z" in metres.
    #[arg(long, value_parser = parse_point, default_value = "3.2,4.1,1.2")]
    target: Vec3,
    #[command(flatten)]
    channel: ChannelArgs,
    /// Where the target height comes from.
    #[arg(long, value_enum, default_value_t = HeightArg::Baro)]
    height: HeightArg,
    /// Barometric height error (1σ), metres.
    #[arg(long, default_value_t = 0.5)]
    baro_sigma: f32,
    /// Monte-Carlo mode: this many fixes at random targets, compared across all combiners.
    #[arg(long)]
    trials: Option<u32>,
    #[arg(long, default_value_t = 1)]
    seed: u64,
}

#[derive(Clone, Copy, PartialEq, Eq, ValueEnum)]
enum HeightArg {
    /// 3D from ranges alone.
    Free,
    /// 2D at the true target height.
    Fixed,
    /// 3D plus a barometric height observation with `--baro-sigma` error.
    Baro,
}

struct Link {
    anchor: Vec3,
    truth: f32,
    paths: Vec<PhaseSlope>,
}

pub fn run(args: &SimArgs) -> Result<()> {
    match args.trials {
        Some(n) => trials(args, n),
        None => single(args),
    }
}

fn simulate_links(args: &SimArgs, target: Vec3, rng: &mut SplitMix64) -> Result<Vec<Link>> {
    NODES
        .iter()
        .map(|&anchor| {
            let truth = scenario::distance(anchor, target);
            let paths = scenario::procedure(&args.channel, truth, rng)?;
            Ok(Link { anchor, truth, paths })
        })
        .collect()
}

fn height(args: &SimArgs, target: Vec3, rng: &mut SplitMix64) -> Height {
    match args.height {
        HeightArg::Free => Height::Free,
        HeightArg::Fixed => Height::Fixed(target[2]),
        HeightArg::Baro => Height::Observed {
            z: target[2] + args.baro_sigma * rng.normal() as f32,
            sigma_m: args.baro_sigma,
        },
    }
}

/// Returns the fix and its error: horizontal for `Fixed`, 3D otherwise.
fn solve(links: &[Link], how: Combine, height: Height, target: Vec3) -> Option<(Fix, f32)> {
    let obs: Vec<RangeObs> = links
        .iter()
        .map(|l| {
            let est = navgrid_ranging::combine_paths(&l.paths, how).expect("at least one antenna path");
            RangeObs {
                anchor: l.anchor,
                distance_m: est.distance_m,
                sigma_m: navgrid_ranging::range_sigma_m(&est),
            }
        })
        .collect();
    let opts = Options {
        height,
        ..Options::default()
    };
    let fix = navgrid_solver::multilaterate(&obs, scenario::centroid(), &opts)?;
    let err = match height {
        Height::Fixed(_) => scenario::horizontal_distance(fix.position, target),
        _ => scenario::distance(fix.position, target),
    };
    Some((fix, err))
}

fn single(args: &SimArgs) -> Result<()> {
    let mut rng = SplitMix64::new(args.seed);
    let links = simulate_links(args, args.target, &mut rng)?;
    let how = Combine::from(args.channel.combine);
    println!("node                   true_m    est_m    err_m  resid_rad  paths");
    for l in &links {
        let est = navgrid_ranging::combine_paths(&l.paths, how).expect("at least one antenna path");
        println!(
            "{:<20} {:>8.3} {:>8.3} {:>+8.3} {:>10.4}  {}",
            format!("{:?}", l.anchor),
            l.truth,
            est.distance_m,
            est.distance_m - l.truth,
            est.residual_rad,
            l.paths.len(),
        );
    }
    let height = height(args, args.target, &mut rng);
    let (fix, err) = solve(&links, how, height, args.target).context("solver failed")?;
    let [x, y, z] = fix.position;
    let [sx, sy, sz] = fix.sigma_m;
    println!();
    println!("fix    ({x:.3}, {y:.3}, {z:.3}) m  ±({sx:.3}, {sy:.3}, {sz:.3})  height {height:?}");
    println!(
        "error  {err:.3} m {}  horizontal {:.3} m  vertical {:+.3} m   normalized rms {:.2}   iterations {}",
        if matches!(height, Height::Fixed(_)) {
            "horizontal"
        } else {
            "3D"
        },
        scenario::horizontal_distance(fix.position, args.target),
        z - args.target[2],
        fix.normalized_rms,
        fix.iterations
    );
    Ok(())
}

fn trials(args: &SimArgs, n: u32) -> Result<()> {
    const ALL: [(CombineArg, &str); 4] = [
        (CombineArg::First, "first"),
        (CombineArg::LowestResidual, "lowest-residual"),
        (CombineArg::Mean, "mean"),
        (CombineArg::ResidualWeighted, "residual-weighted"),
    ];
    let mut rng = SplitMix64::new(args.seed);
    let mut range_err = [0.0f64; 4];
    let mut fix_err: [Vec<f32>; 4] = Default::default();
    let mut links_total = 0usize;
    for _ in 0..n {
        let z = if args.height == HeightArg::Fixed {
            args.target[2]
        } else {
            0.5 + 1.5 * rng.uniform() as f32
        };
        let target = [1.0 + 8.0 * rng.uniform() as f32, 1.0 + 6.0 * rng.uniform() as f32, z];
        let links = simulate_links(args, target, &mut rng)?;
        let height = height(args, target, &mut rng);
        links_total += links.len();
        for (i, (c, _)) in ALL.iter().enumerate() {
            let how = Combine::from(*c);
            for l in &links {
                let est = navgrid_ranging::combine_paths(&l.paths, how).expect("at least one antenna path");
                range_err[i] += f64::from((est.distance_m - l.truth).abs());
            }
            fix_err[i].push(solve(&links, how, height, target).map_or(f32::INFINITY, |(_, e)| e));
        }
    }
    println!(
        "{n} trials, {} antenna path(s), multipath {:?}, height {}",
        args.channel.antenna_paths,
        args.channel.multipath,
        match args.height {
            HeightArg::Free => "free (3D)",
            HeightArg::Fixed => "fixed (2D, horizontal error)",
            HeightArg::Baro => "baro (3D)",
        }
    );
    println!("combiner             mean|range err|   fix p50   fix p90");
    for (i, (_, name)) in ALL.iter().enumerate() {
        let errs = &mut fix_err[i];
        errs.sort_by(f32::total_cmp);
        println!(
            "{name:<20} {:>14.3} m {:>7.3} m {:>7.3} m",
            range_err[i] / links_total as f64,
            scenario::quantile(errs, 0.5),
            scenario::quantile(errs, 0.9)
        );
    }
    Ok(())
}

fn parse_point(s: &str) -> Result<Vec3> {
    let parts: Vec<f32> = s.split(',').map(|p| p.trim().parse()).collect::<Result<_, _>>()?;
    match parts[..] {
        [x, y, z] => Ok([x, y, z]),
        _ => bail!("expected x,y,z"),
    }
}
