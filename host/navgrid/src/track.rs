//! `navgrid track`: a walking target followed by the [`Tracker`], one range per update, comparing
//! ranging rates with and without IMU motion gating. The procedure count is the energy proxy
//! (architecture §5): every procedure costs both the target and the node it ranges with.

use anyhow::{Context, Result};
use navgrid_ranging::sim::SplitMix64;
use navgrid_solver::tracker::{MotionNoise, Tracker};
use navgrid_solver::{Height, Options, RangeObs, Vec3};

use crate::scenario::{self, ChannelArgs, NODES};

#[derive(clap::Args)]
pub struct TrackArgs {
    #[command(flatten)]
    channel: ChannelArgs,
    /// Ranging rates to compare while moving, procedures per second.
    #[arg(long, value_delimiter = ',', default_value = "0.5,1,2,4")]
    rates: Vec<f32>,
    /// Ranging rate while the IMU says the target is still, after the settling burst.
    #[arg(long, default_value_t = 0.1)]
    still_rate: f32,
    /// Ranges taken at the moving rate right after the IMU reports a stop. While still, every range
    /// adds full information, so a short burst pins the position before ranging slows down.
    #[arg(long, default_value_t = 6)]
    settle: u32,
    /// Barometric height error (1σ), metres; one sample per second.
    #[arg(long, default_value_t = 0.5)]
    baro_sigma: f32,
    /// Runs (seeds) per configuration; errors are pooled, procedures averaged.
    #[arg(long, default_value_t = 20)]
    runs: u64,
    #[arg(long, default_value_t = 1)]
    seed: u64,
}

/// Walk a 6 × 4 m loop at 1 m/s, stand still, walk it again, at a constant carry height.
const LOOP: [[f32; 2]; 4] = [[2.0, 2.0], [8.0, 2.0], [8.0, 6.0], [2.0, 6.0]];
const LAP_S: f32 = 20.0;
const STILL_S: f32 = 30.0;
const DURATION_S: f32 = 2.0 * LAP_S + STILL_S;
const CARRY_HEIGHT: f32 = 1.2;
const TICK_S: f32 = 0.05;

/// True position at time `t`, and whether the target is still.
fn truth(t: f32) -> (Vec3, bool) {
    let (walked_s, still) = if t < LAP_S {
        (t, false)
    } else if t < LAP_S + STILL_S {
        (LAP_S, true)
    } else {
        (t - STILL_S, false)
    };
    // 1 m/s around a 20 m loop.
    let mut s = walked_s % 20.0;
    for i in 0..LOOP.len() {
        let (a, b) = (LOOP[i], LOOP[(i + 1) % LOOP.len()]);
        let len = (b[0] - a[0]).abs() + (b[1] - a[1]).abs();
        if s <= len {
            let f = s / len;
            return (
                [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), CARRY_HEIGHT],
                still,
            );
        }
        s -= len;
    }
    ([LOOP[0][0], LOOP[0][1], CARRY_HEIGHT], still)
}

struct Outcome {
    procedures: u32,
    moving: Vec<f32>,
    still: Vec<f32>,
    vertical: Vec<f32>,
}

fn run_one(args: &TrackArgs, rate: f32, imu: bool, seed: u64) -> Result<Outcome> {
    let mut rng = SplitMix64::new(seed);
    let baro = |rng: &mut SplitMix64| CARRY_HEIGHT + args.baro_sigma * rng.normal() as f32;

    // Acquisition: one range to every node, then a fix to start the filter from.
    let (start, _) = truth(0.0);
    let obs = NODES
        .iter()
        .map(|&anchor| {
            let (est, sigma_m) = scenario::range(&args.channel, scenario::distance(anchor, start), &mut rng)?;
            Ok(RangeObs {
                anchor,
                distance_m: est.distance_m,
                sigma_m,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let opts = Options {
        height: Height::Observed {
            z: baro(&mut rng),
            sigma_m: args.baro_sigma,
        },
        ..Options::default()
    };
    let fix = navgrid_solver::multilaterate(&obs, scenario::centroid(), &opts).context("acquisition fix failed")?;
    let mut tracker = Tracker::new(fix.position, fix.sigma_m, 0.5);

    let mut out = Outcome {
        procedures: NODES.len() as u32,
        moving: Vec::new(),
        still: Vec::new(),
        vertical: Vec::new(),
    };
    let (mut last_update, mut last_range, mut next_baro, mut next_node) = (0.0f32, 0.0f32, 1.0f32, 0usize);
    let (mut was_still, mut settle_left) = (false, 0u32);
    let ticks = (DURATION_S / TICK_S).round() as u32;
    for tick in 1..=ticks {
        let t = tick as f32 * TICK_S;
        let (pos, actually_still) = truth(t);
        // Without an IMU the filter never learns the target stopped.
        let still = imu && actually_still;
        if was_still && !still {
            tracker.motion_started(1.5, 0.2);
        }
        if still && !was_still {
            settle_left = args.settle;
        }
        was_still = still;
        let interval = 1.0
            / if still && settle_left == 0 {
                args.still_rate
            } else {
                rate
            };
        let range_due = t - last_range >= interval - 1e-4;
        let baro_due = t >= next_baro;
        if range_due || baro_due {
            let noise = if still {
                MotionNoise::STILL
            } else {
                MotionNoise::WALKING
            };
            tracker.predict(t - last_update, noise);
            last_update = t;
            if still {
                tracker.update_still(0.02);
            }
            if range_due {
                let anchor = NODES[next_node % NODES.len()];
                next_node += 1;
                let (est, sigma_m) = scenario::range(&args.channel, scenario::distance(anchor, pos), &mut rng)?;
                tracker.update_range(anchor, est.distance_m, sigma_m);
                out.procedures += 1;
                last_range = t;
                settle_left = settle_left.saturating_sub(1);
            }
            if baro_due {
                tracker.update_height(baro(&mut rng), args.baro_sigma);
                next_baro += 1.0;
            }
        }
        if tick % 10 == 0 {
            // Extrapolate to now, as the target would report between updates.
            let (p, v) = (tracker.position(), tracker.velocity());
            let dt = t - last_update;
            let est = [p[0] + v[0] * dt, p[1] + v[1] * dt, p[2] + v[2] * dt];
            let horizontal = scenario::horizontal_distance(est, pos);
            if actually_still {
                &mut out.still
            } else {
                &mut out.moving
            }
            .push(horizontal);
            out.vertical.push((est[2] - pos[2]).abs());
        }
    }
    Ok(out)
}

pub fn run(args: &TrackArgs) -> Result<()> {
    println!(
        "{DURATION_S:.0} s walk (2 laps at 1 m/s around a {STILL_S:.0} s stop), one range per update, {} antenna path(s), multipath {:?}, baro σ {} m, {} runs",
        args.channel.antenna_paths, args.channel.multipath, args.baro_sigma, args.runs
    );
    println!("                         ---- horizontal error ----");
    println!("rate/s  imu  procedures  moving p50  moving p90  still p90   vert p90");
    for &rate in &args.rates {
        for imu in [false, true] {
            let mut o = Outcome {
                procedures: 0,
                moving: Vec::new(),
                still: Vec::new(),
                vertical: Vec::new(),
            };
            for seed in args.seed..args.seed + args.runs {
                let r = run_one(args, rate, imu, seed)?;
                o.procedures += r.procedures;
                o.moving.extend(r.moving);
                o.still.extend(r.still);
                o.vertical.extend(r.vertical);
            }
            for v in [&mut o.moving, &mut o.still, &mut o.vertical] {
                v.sort_by(f32::total_cmp);
            }
            println!(
                "{rate:>6.1}  {:<3}  {:>10}  {:>8.3} m  {:>8.3} m  {:>7.3} m  {:>7.3} m",
                if imu { "yes" } else { "no" },
                o.procedures / args.runs as u32,
                scenario::quantile(&o.moving, 0.5),
                scenario::quantile(&o.moving, 0.9),
                scenario::quantile(&o.still, 0.9),
                scenario::quantile(&o.vertical, 0.9),
            );
        }
    }
    Ok(())
}
