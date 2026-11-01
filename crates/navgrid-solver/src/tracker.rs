//! Target tracking filter (architecture D-14).
//!
//! A constant-velocity Kalman filter over position and velocity that takes measurements one at a
//! time: a single range, a height (differential barometry), or a zero-velocity update when the
//! IMU says the target is still. Because each range is its own update, a target can range to one
//! node per step instead of a full set at once: fewer simultaneous links and less energy for the
//! same information over time.
//!
//! There is no heading state. Without a magnetometer heading is not measured, and the
//! constant-velocity model does not need it. IMU-driven propagation comes later (D-15).

use crate::{Vec3, norm};

type Vec6 = [f32; 6];
type Mat6 = [[f32; 6]; 6];

/// Range and height updates whose innovation exceeds this many sigmas are rejected, which drops
/// most NLOS-biased ranges.
pub const GATE_SIGMA: f32 = 3.0;

/// After this many range rejections in a row the filter assumes it is the one that is wrong (not
/// the ranges, which come from different nodes) and re-opens its position uncertainty.
pub const MAX_REJECTIONS: u8 = 3;

/// White-noise acceleration driving the motion model, m/s² (1σ).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotionNoise {
    pub horizontal: f32,
    pub vertical: f32,
}

impl MotionNoise {
    /// Walking person: brisk horizontal changes, little vertical motion.
    pub const WALKING: Self = Self {
        horizontal: 1.0,
        vertical: 0.1,
    };
    /// The IMU reports the target is still.
    pub const STILL: Self = Self {
        horizontal: 0.01,
        vertical: 0.01,
    };
}

#[derive(Clone, Debug)]
pub struct Tracker {
    /// Position (m), then velocity (m/s).
    x: Vec6,
    p: Mat6,
    rejections: u8,
}

impl Tracker {
    /// Starts at `position`, typically a multilateration fix, with zero velocity.
    pub fn new(position: Vec3, position_sigma: Vec3, velocity_sigma: f32) -> Self {
        let mut p = [[0.0; 6]; 6];
        for i in 0..3 {
            p[i][i] = position_sigma[i] * position_sigma[i];
            p[i + 3][i + 3] = velocity_sigma * velocity_sigma;
        }
        Self {
            x: [position[0], position[1], position[2], 0.0, 0.0, 0.0],
            p,
            rejections: 0,
        }
    }

    pub fn position(&self) -> Vec3 {
        [self.x[0], self.x[1], self.x[2]]
    }

    pub fn velocity(&self) -> Vec3 {
        [self.x[3], self.x[4], self.x[5]]
    }

    pub fn position_sigma(&self) -> Vec3 {
        core::array::from_fn(|i| libm::sqrtf(self.p[i][i]))
    }

    pub fn velocity_sigma(&self) -> Vec3 {
        core::array::from_fn(|i| libm::sqrtf(self.p[i + 3][i + 3]))
    }

    /// Propagates the state by `dt` seconds.
    pub fn predict(&mut self, dt: f32, noise: MotionNoise) {
        let (x, p) = (&mut self.x, &mut self.p);
        for i in 0..3 {
            x[i] += dt * x[i + 3];
        }
        // P ← F·P·Fᵀ with F = [[I, dt·I], [0, I]]: rows, then columns, in place.
        let (position_rows, velocity_rows) = p.split_at_mut(3);
        for (row, velocity_row) in position_rows.iter_mut().zip(velocity_rows.iter()) {
            for (a, b) in row.iter_mut().zip(velocity_row) {
                *a += dt * b;
            }
        }
        for row in p.iter_mut() {
            for j in 0..3 {
                row[j] += dt * row[j + 3];
            }
        }
        // Discrete white-noise acceleration, per axis.
        let (dt2, dt3, dt4) = (dt * dt, dt * dt * dt, dt * dt * dt * dt);
        for i in 0..3 {
            let sigma = if i == 2 { noise.vertical } else { noise.horizontal };
            let q = sigma * sigma;
            p[i][i] += 0.25 * dt4 * q;
            p[i][i + 3] += 0.5 * dt3 * q;
            p[i + 3][i] += 0.5 * dt3 * q;
            p[i + 3][i + 3] += dt2 * q;
        }
    }

    /// One range to a node at a known position. Returns `false` if gated out as an outlier.
    pub fn update_range(&mut self, anchor: Vec3, distance_m: f32, sigma_m: f32) -> bool {
        let diff: Vec3 = core::array::from_fn(|i| self.x[i] - anchor[i]);
        let rho = norm(&diff);
        if rho < 1e-3 {
            return false;
        }
        let h = [diff[0] / rho, diff[1] / rho, diff[2] / rho, 0.0, 0.0, 0.0];
        let y = distance_m - rho;
        if self.update(&h, y, sigma_m * sigma_m, true) {
            self.rejections = 0;
            return true;
        }
        self.rejections += 1;
        if self.rejections < MAX_REJECTIONS {
            return false;
        }
        // Lost: widen position uncertainty enough to take this range, and carry on from it.
        self.rejections = 0;
        for i in 0..3 {
            self.p[i][i] += y * y;
        }
        self.update(&h, y, sigma_m * sigma_m, false)
    }

    /// The IMU reports that motion is starting. Re-opens the velocity uncertainty, which zero-velocity
    /// updates had collapsed, so the filter can follow instead of rejecting every range.
    pub fn motion_started(&mut self, horizontal_mps: f32, vertical_mps: f32) {
        for i in 3..6 {
            for j in 0..6 {
                self.p[i][j] = 0.0;
                self.p[j][i] = 0.0;
            }
            let sigma = if i == 5 { vertical_mps } else { horizontal_mps };
            self.p[i][i] = sigma * sigma;
        }
    }

    /// Height observation, e.g. differential barometry against nearby nodes. Returns `false` if
    /// gated out.
    pub fn update_height(&mut self, z: f32, sigma_m: f32) -> bool {
        let h = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
        self.update(&h, z - self.x[2], sigma_m * sigma_m, true)
    }

    /// Zero-velocity update: the IMU says the target is still.
    pub fn update_still(&mut self, sigma_mps: f32) {
        for axis in 3..6 {
            let mut h = [0.0; 6];
            h[axis] = 1.0;
            self.update(&h, -self.x[axis], sigma_mps * sigma_mps, false);
        }
    }

    /// Scalar Kalman update: measurement row `h`, innovation `y`, measurement variance `r`.
    fn update(&mut self, h: &Vec6, y: f32, r: f32, gate: bool) -> bool {
        let ph: Vec6 = core::array::from_fn(|i| dot6(&self.p[i], h));
        let s = dot6(h, &ph) + r;
        if gate && y * y > GATE_SIGMA * GATE_SIGMA * s {
            return false;
        }
        // K = P·h / s;  x += K·y;  P −= K·(P·h)ᵀ, which equals K·hᵀ·P for symmetric P.
        for i in 0..6 {
            let k = ph[i] / s;
            self.x[i] += k * y;
            for (pij, phj) in self.p[i].iter_mut().zip(&ph) {
                *pij -= k * phj;
            }
        }
        for i in 0..6 {
            for j in i + 1..6 {
                let m = 0.5 * (self.p[i][j] + self.p[j][i]);
                self.p[i][j] = m;
                self.p[j][i] = m;
            }
        }
        true
    }
}

fn dot6(a: &Vec6, b: &Vec6) -> f32 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

#[cfg(test)]
mod tests {
    use navgrid_ranging::sim::SplitMix64;

    use super::*;

    const NODES: [Vec3; 6] = [
        [0.0, 0.0, 2.7],
        [10.0, 0.0, 0.4],
        [10.0, 8.0, 2.7],
        [0.0, 8.0, 0.4],
        [5.0, 0.0, 1.5],
        [5.0, 8.0, 2.0],
    ];

    fn dist(a: Vec3, b: Vec3) -> f32 {
        norm(&core::array::from_fn(|i| a[i] - b[i]))
    }

    #[test]
    fn converges_on_still_target_one_range_at_a_time() {
        let truth = [6.0, 3.0, 1.2];
        let mut t = Tracker::new([5.0, 4.0, 1.5], [2.0; 3], 0.5);
        let mut rng = SplitMix64::new(1);
        for step in 0..120 {
            t.predict(0.25, MotionNoise::STILL);
            t.update_still(0.02);
            let node = NODES[step % NODES.len()];
            t.update_range(node, dist(node, truth) + 0.1 * rng.normal() as f32, 0.1);
        }
        assert!(dist(t.position(), truth) < 0.1, "{:?}", t.position());
    }

    #[test]
    fn tracks_walking_target_with_barometer() {
        let mut rng = SplitMix64::new(2);
        let dt = 0.25;
        // Back and forth along x at up to 1 m/s.
        let truth_at = |t: f32| [5.0 + 3.0 * libm::sinf(t / 3.0), 4.0, 1.2];
        let mut tracker = Tracker::new(truth_at(0.0), [0.3; 3], 1.0);
        let mut worst = 0.0f32;
        for step in 1..=240 {
            let time = step as f32 * dt;
            let truth = truth_at(time);
            tracker.predict(dt, MotionNoise::WALKING);
            let node = NODES[step % NODES.len()];
            tracker.update_range(node, dist(node, truth) + 0.1 * rng.normal() as f32, 0.1);
            if step % 4 == 0 {
                tracker.update_height(1.2 + 0.3 * rng.normal() as f32, 0.3);
            }
            if step > 20 {
                let p = tracker.position();
                worst = worst.max(dist([p[0], p[1], 0.0], [truth[0], truth[1], 0.0]));
            }
        }
        assert!(worst < 0.5, "worst horizontal error {worst}");
    }

    #[test]
    fn gate_rejects_nlos_outlier() {
        let truth = [6.0, 3.0, 1.2];
        let mut t = Tracker::new(truth, [0.05; 3], 0.1);
        let node = NODES[0];
        assert!(!t.update_range(node, dist(node, truth) + 3.0, 0.1));
        assert!(t.update_range(node, dist(node, truth) + 0.05, 0.1));
    }

    #[test]
    fn still_update_pins_velocity() {
        let mut t = Tracker::new([5.0, 4.0, 1.2], [0.1; 3], 1.0);
        t.update_still(0.01);
        assert!(t.velocity_sigma().iter().all(|&s| s < 0.02));
    }
}
