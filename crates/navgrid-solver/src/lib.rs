//! Position solvers: one-shot multilateration (this file) and the target tracking filter
//! ([`tracker`]). The distributed mesh solver comes later (architecture D-9).

#![no_std]

pub mod tracker;

pub type Vec3 = [f32; 3];
type Mat3 = [[f32; 3]; 3];

/// A range to a node at a known position.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RangeObs {
    pub anchor: Vec3,
    pub distance_m: f32,
    /// 1-sigma range uncertainty.
    pub sigma_m: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Fix {
    pub position: Vec3,
    /// 1-sigma per axis, from the inverse of the normal matrix.
    pub sigma_m: Vec3,
    /// RMS of residuals divided by their sigmas; about 1 when the sigmas are honest.
    pub normalized_rms: f32,
    pub iterations: u8,
}

/// How the solver treats the target's height.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Height {
    /// Pure 3D from ranges; needs nodes at varied heights.
    Free,
    /// 2D: hold the target at this height. Ranges stay 3D, so node heights still count.
    Fixed(f32),
    /// 3D with a height observation: differential barometry, or a carry-height prior with a
    /// loose sigma. Ranges refine z where node geometry allows; elsewhere the observation holds it.
    Observed { z: f32, sigma_m: f32 },
}

#[derive(Clone, Copy, Debug)]
pub struct Options {
    pub max_iterations: u8,
    /// Stop when a step moves the estimate less than this.
    pub tolerance_m: f32,
    /// Levenberg damping, relative to the normal matrix diagonal.
    pub damping: f32,
    pub height: Height,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            max_iterations: 20,
            tolerance_m: 1e-4,
            damping: 1e-3,
            height: Height::Free,
        }
    }
}

/// Weighted Gauss-Newton multilateration.
///
/// Needs more constraints than unknowns: 4 ranges with [`Height::Free`], 3 with
/// [`Height::Fixed`] or [`Height::Observed`]. With `Free` and coplanar anchors the problem has a
/// mirror solution; the result lands on the same side of the plane as `initial`, so never start
/// in that plane. The other modes start at the given height and have no such ambiguity.
pub fn multilaterate(obs: &[RangeObs], initial: Vec3, opts: &Options) -> Option<Fix> {
    let dims = if matches!(opts.height, Height::Fixed(_)) { 2 } else { 3 };
    let height_obs = usize::from(matches!(opts.height, Height::Observed { .. }));
    if obs.len() + height_obs <= dims {
        return None;
    }
    let mut x = initial;
    if let Height::Fixed(z) | Height::Observed { z, .. } = opts.height {
        x[2] = z;
    }
    let mut iterations = 0;
    while iterations < opts.max_iterations {
        iterations += 1;
        let (mut a, g, _) = normal_equations(obs, x, opts.height);
        for (i, row) in a.iter_mut().enumerate() {
            row[i] *= 1.0 + opts.damping;
        }
        let a_inv = invert(&a, dims)?;
        let step: Vec3 = core::array::from_fn(|i| -dot(&a_inv[i], &g));
        x = core::array::from_fn(|i| x[i] + step[i]);
        if norm(&step) < opts.tolerance_m {
            break;
        }
    }

    let (a, _, chi2) = normal_equations(obs, x, opts.height);
    let cov = invert(&a, dims)?;
    let dof = (obs.len() + height_obs - dims) as f32;
    Some(Fix {
        position: x,
        sigma_m: core::array::from_fn(|i| libm::sqrtf(cov[i][i])),
        normalized_rms: libm::sqrtf(chi2 / dof),
        iterations,
    })
}

/// Returns `(JᵀWJ, JᵀWr, rᵀWr)` at `x`, with range residual `r = |x − anchor| − d`, `W = 1/σ²`,
/// plus the height observation if there is one.
fn normal_equations(obs: &[RangeObs], x: Vec3, height: Height) -> (Mat3, Vec3, f32) {
    let mut a = [[0.0; 3]; 3];
    let mut g = [0.0; 3];
    let mut chi2 = 0.0;
    for o in obs {
        let diff: Vec3 = core::array::from_fn(|i| x[i] - o.anchor[i]);
        let rho = norm(&diff);
        if rho < 1e-6 {
            continue;
        }
        let u: Vec3 = core::array::from_fn(|i| diff[i] / rho);
        let r = rho - o.distance_m;
        let w = 1.0 / (o.sigma_m * o.sigma_m);
        for i in 0..3 {
            g[i] += w * u[i] * r;
            for j in 0..3 {
                a[i][j] += w * u[i] * u[j];
            }
        }
        chi2 += w * r * r;
    }
    if let Height::Observed { z, sigma_m } = height {
        let w = 1.0 / (sigma_m * sigma_m);
        let r = x[2] - z;
        a[2][2] += w;
        g[2] += w * r;
        chi2 += w * r * r;
    }
    (a, g, chi2)
}

/// Inverse of the leading `dims`×`dims` block, zero-padded to 3×3 so unsolved axes get no step
/// and zero variance.
fn invert(m: &Mat3, dims: usize) -> Option<Mat3> {
    if dims == 3 {
        return invert3(m);
    }
    let det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    // See `invert3` for the conditioning check.
    let scale = libm::fabsf(m[0][0] * m[1][1]);
    let well_conditioned = scale > 0.0 && libm::fabsf(det) >= 1e-6 * scale;
    if !well_conditioned {
        return None;
    }
    Some([
        [m[1][1] / det, -m[0][1] / det, 0.0],
        [-m[1][0] / det, m[0][0] / det, 0.0],
        [0.0; 3],
    ])
}

fn invert3(m: &Mat3) -> Option<Mat3> {
    let cof = |r0: usize, r1: usize, c0: usize, c1: usize| m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0];
    let adj = [
        [cof(1, 2, 1, 2), -cof(0, 2, 1, 2), cof(0, 1, 1, 2)],
        [-cof(1, 2, 0, 2), cof(0, 2, 0, 2), -cof(0, 1, 0, 2)],
        [cof(1, 2, 0, 1), -cof(0, 2, 0, 1), cof(0, 1, 0, 1)],
    ];
    let det = m[0][0] * adj[0][0] + m[0][1] * adj[1][0] + m[0][2] * adj[2][0];
    // Relative to the diagonal product, so the check does not depend on the weights' scale.
    // Written so that NaN anywhere also fails.
    let scale = libm::fabsf(m[0][0] * m[1][1] * m[2][2]);
    let well_conditioned = scale > 0.0 && libm::fabsf(det) >= 1e-6 * scale;
    if !well_conditioned {
        return None;
    }
    Some(core::array::from_fn(|i| core::array::from_fn(|j| adj[i][j] / det)))
}

pub(crate) fn dot(a: &Vec3, b: &Vec3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub(crate) fn norm(v: &Vec3) -> f32 {
    libm::sqrtf(dot(v, v))
}

#[cfg(test)]
mod tests {
    use super::*;

    const ROOM: [Vec3; 6] = [
        [0.0, 0.0, 2.7],
        [10.0, 0.0, 0.4],
        [10.0, 8.0, 2.7],
        [0.0, 8.0, 0.4],
        [5.0, 0.0, 1.5],
        [5.0, 8.0, 2.0],
    ];

    fn observe(anchors: &[Vec3], target: Vec3, noise: &[f32]) -> [RangeObs; 6] {
        core::array::from_fn(|i| {
            let a = anchors[i];
            let d = norm(&core::array::from_fn(|k| target[k] - a[k]));
            RangeObs {
                anchor: a,
                distance_m: d + noise[i],
                sigma_m: 0.1,
            }
        })
    }

    fn error(fix: &Fix, target: Vec3) -> f32 {
        norm(&core::array::from_fn(|k| fix.position[k] - target[k]))
    }

    #[test]
    fn exact_ranges_converge() {
        let target = [3.2, 4.1, 1.2];
        let obs = observe(&ROOM, target, &[0.0; 6]);
        let fix = multilaterate(&obs, [5.0, 4.0, 1.5], &Options::default()).unwrap();
        assert!(error(&fix, target) < 1e-3, "{fix:?}");
        assert!(fix.normalized_rms < 0.01);
    }

    #[test]
    fn noisy_ranges_stay_close() {
        let target = [7.5, 1.5, 0.9];
        let obs = observe(&ROOM, target, &[0.08, -0.12, 0.05, 0.1, -0.07, 0.03]);
        let fix = multilaterate(&obs, [5.0, 4.0, 1.5], &Options::default()).unwrap();
        assert!(error(&fix, target) < 0.3, "{fix:?}");
        assert!(fix.sigma_m.iter().all(|&s| s > 0.0 && s < 1.0));
    }

    #[test]
    fn coplanar_anchors_follow_initial_side() {
        let ceiling: [Vec3; 6] = core::array::from_fn(|i| [ROOM[i][0], ROOM[i][1], 2.7]);
        let target = [3.2, 4.1, 1.2];
        let obs = observe(&ceiling, target, &[0.0; 6]);
        let fix = multilaterate(&obs, [5.0, 4.0, 0.0], &Options::default()).unwrap();
        assert!(error(&fix, target) < 1e-2, "{fix:?}");
    }

    #[test]
    fn too_few_observations() {
        let obs = observe(&ROOM, [1.0, 1.0, 1.0], &[0.0; 6]);
        assert!(multilaterate(&obs[..3], [5.0, 4.0, 1.5], &Options::default()).is_none());
        let flat = Options {
            height: Height::Fixed(1.0),
            ..Options::default()
        };
        assert!(multilaterate(&obs[..2], [5.0, 4.0, 1.5], &flat).is_none());
    }

    #[test]
    fn two_d_with_three_nodes_at_mixed_heights() {
        let target = [3.2, 4.1, 1.2];
        let obs = observe(&ROOM, target, &[0.0; 6]);
        let flat = Options {
            height: Height::Fixed(1.2),
            ..Options::default()
        };
        let fix = multilaterate(&obs[..3], [5.0, 4.0, 0.0], &flat).unwrap();
        assert!(error(&fix, target) < 1e-3, "{fix:?}");
        assert_eq!(fix.position[2], 1.2);
        assert_eq!(fix.sigma_m[2], 0.0);
    }

    #[test]
    fn two_d_handles_coplanar_ceiling_nodes() {
        let ceiling: [Vec3; 6] = core::array::from_fn(|i| [ROOM[i][0], ROOM[i][1], 2.7]);
        let target = [7.5, 1.5, 1.0];
        let obs = observe(&ceiling, target, &[0.08, -0.12, 0.05, 0.1, -0.07, 0.03]);
        // Starting in the anchor plane is fatal in 3D but irrelevant in 2D.
        let flat = Options {
            height: Height::Fixed(1.0),
            ..Options::default()
        };
        let fix = multilaterate(&obs, [5.0, 4.0, 2.7], &flat).unwrap();
        assert!(error(&fix, target) < 0.3, "{fix:?}");
    }

    #[test]
    fn observed_height_makes_ceiling_nodes_solvable_in_3d() {
        let ceiling: [Vec3; 6] = core::array::from_fn(|i| [ROOM[i][0], ROOM[i][1], 2.7]);
        let target = [7.5, 1.5, 1.0];
        let obs = observe(&ceiling, target, &[0.0; 6]);
        // Barometer says 1.3 m ± 0.5 m; exact ranges pull z the rest of the way.
        let baro = Options {
            height: Height::Observed { z: 1.3, sigma_m: 0.5 },
            ..Options::default()
        };
        let fix = multilaterate(&obs, [5.0, 4.0, 2.7], &baro).unwrap();
        assert!(error(&fix, target) < 0.05, "{fix:?}");
        assert!(fix.sigma_m[2] < 0.5);
    }

    #[test]
    fn observed_height_needs_only_three_ranges() {
        let target = [3.2, 4.1, 1.2];
        let obs = observe(&ROOM, target, &[0.0; 6]);
        let baro = Options {
            height: Height::Observed { z: 1.2, sigma_m: 0.3 },
            ..Options::default()
        };
        let fix = multilaterate(&obs[..3], [5.0, 4.0, 1.5], &baro).unwrap();
        assert!(error(&fix, target) < 0.01, "{fix:?}");
    }
}
