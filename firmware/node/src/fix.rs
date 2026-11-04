//! A tag's own position (FR-T1, FR-T3, first cut): after each ranging burst the tag keeps the
//! distance to that anchor with the anchor's position from its beacon, and with three or more
//! fresh ranges multilaterates, holding its height near carry height; the fix goes into the
//! mesh state as its Position record (FR-T5), at most every few seconds. The tracking filter
//! (`navgrid_solver::tracker`) replaces the one-shot fix once the accelerometer drives motion.

use core::cell::RefCell;

use defmt::info;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant};
use navgrid_mesh::{Body, NodeId, Position};
use navgrid_proto::{BeaconPosition, PointMm};
use navgrid_solver::{Height, Options, RangeObs, multilaterate};

use crate::state;

/// Anchors a tag keeps ranges to.
const ANCHORS: usize = 8;
/// A range older than this no longer counts.
const FRESH: Duration = Duration::from_secs(10);
/// Between position records.
const FIX_EVERY: Duration = Duration::from_secs(3);
/// Where a carried tag sits above the floor, and how surely.
const CARRY_HEIGHT_M: f32 = 1.0;
const CARRY_SIGMA_M: f32 = 0.7;
/// The frame version until referencing numbers them (FR-A5).
const FRAME: u16 = 1;

struct Range {
    anchor: NodeId,
    position: [f32; 3],
    sigma_m: f32,
    distance_m: f32,
    at: Instant,
}

struct Fixes {
    ranges: heapless::Vec<Range, ANCHORS>,
    last_fix: Option<Instant>,
    last_position: Option<[f32; 3]>,
}

static FIXES: Mutex<CriticalSectionRawMutex, RefCell<Fixes>> = Mutex::new(RefCell::new(Fixes {
    ranges: heapless::Vec::new(),
    last_fix: None,
    last_position: None,
}));

/// A burst with `anchor` ended with this distance; `beacon` is the anchor's advertised position,
/// if the tag has heard one.
pub fn ranged(me: NodeId, anchor: NodeId, distance_m: f32, sigma_m: f32, beacon: Option<BeaconPosition>) {
    let position = beacon.map(|b| {
        (
            [
                f32::from(b.point_dm[0]) / 10.0,
                f32::from(b.point_dm[1]) / 10.0,
                f32::from(b.point_dm[2]) / 10.0,
            ],
            f32::from(b.sigma_dm) / 10.0,
        )
    });
    let fix = FIXES.lock(|f| {
        let mut f = f.borrow_mut();
        let now = Instant::now();
        match f.ranges.iter_mut().find(|r| r.anchor == anchor) {
            Some(r) => {
                r.distance_m = distance_m;
                r.sigma_m = sigma_m;
                r.at = now;
                if let Some((p, _)) = position {
                    r.position = p;
                }
            }
            None => {
                let (p, _) = position?;
                if f.ranges.is_full() {
                    let (i, _) = f.ranges.iter().enumerate().min_by_key(|(_, r)| r.at).expect("not empty");
                    f.ranges.swap_remove(i);
                }
                let _ = f.ranges.push(Range {
                    anchor,
                    position: p,
                    sigma_m,
                    distance_m,
                    at: now,
                });
            }
        }
        if f.last_fix.is_some_and(|t| now - t < FIX_EVERY) {
            return None;
        }
        let obs: heapless::Vec<RangeObs, ANCHORS> = f
            .ranges
            .iter()
            .filter(|r| now - r.at <= FRESH)
            .map(|r| RangeObs {
                anchor: r.position,
                distance_m: r.distance_m,
                sigma_m: r.sigma_m.max(0.3),
            })
            .collect();
        if obs.len() < 3 {
            return None;
        }
        let mut initial = f.last_position.unwrap_or_else(|| {
            let mut c = [0.0f32; 3];
            for o in &obs {
                for (ci, ai) in c.iter_mut().zip(o.anchor) {
                    *ci += ai / obs.len() as f32;
                }
            }
            c[0] += 0.3;
            c[2] = CARRY_HEIGHT_M;
            c
        });
        initial[2] = CARRY_HEIGHT_M;
        let fix = multilaterate(
            &obs,
            initial,
            &Options {
                height: Height::Observed {
                    z: CARRY_HEIGHT_M,
                    sigma_m: CARRY_SIGMA_M,
                },
                ..Options::default()
            },
        )?;
        f.last_fix = Some(now);
        f.last_position = Some(fix.position);
        Some((fix, obs.len()))
    });
    let Some((fix, anchors)) = fix else {
        return;
    };
    let mm = |v: f32| (v * 1000.0) as i32;
    let smm = |v: f32| (v.max(0.0) * 1000.0) as u32;
    info!(
        "fix from {=usize} anchors: {=f32} {=f32} {=f32} m, sigma {=f32} {=f32} {=f32}, rms {=f32}",
        anchors,
        fix.position[0],
        fix.position[1],
        fix.position[2],
        fix.sigma_m[0],
        fix.sigma_m[1],
        fix.sigma_m[2],
        fix.normalized_rms
    );
    state::write(Body::Position(Position {
        of: me,
        point: PointMm {
            x: mm(fix.position[0]),
            y: mm(fix.position[1]),
            z: mm(fix.position[2]),
        },
        sigma_mm: [smm(fix.sigma_m[0]), smm(fix.sigma_m[1]), smm(fix.sigma_m[2])],
        frame: FRAME,
    }));
}
