//! The central mesh solver (FR-M6, D-28): every node's position from the links between them, at
//! once, with the pinned nodes fixed. It seeds each unknown node by multilateration from the
//! nodes already placed (pins first, then outward), then refines all of them together by
//! damped Gauss-Newton on every link, with Huber weights so a bounced link (a distance read
//! long) pulls little, and a height prior on unpinned nodes, since anchors at one height leave
//! z poorly determined by ranges alone. A per-link offset is taken off every distance first
//! (the chip's delay, DEF-6).
//!
//! Positions are in the pins' frame. A node that can't be seeded (fewer than three links to
//! placed nodes) stays unsolved.

use alloc::vec;
use alloc::vec::Vec;

use crate::{Height, Options, RangeObs, Vec3, multilaterate, norm};

/// A node with surveyed coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pin {
    pub node: usize,
    pub position: Vec3,
}

/// A measured distance between two nodes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinkObs {
    pub a: usize,
    pub b: usize,
    pub distance_m: f32,
    /// 1-sigma, after the floor in [`MeshOptions::sigma_floor_m`].
    pub sigma_m: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct MeshOptions {
    pub max_iterations: u8,
    pub tolerance_m: f32,
    /// Levenberg damping, relative to the normal matrix diagonal.
    pub damping: f32,
    /// Residuals beyond this many sigmas are down-weighted, as the square of the excess.
    pub huber_k: f32,
    /// Taken off every distance before solving: the chip's delay (DEF-6).
    pub offset_m: f32,
    /// Smallest link sigma believed.
    pub sigma_floor_m: f32,
    /// Height prior for unpinned nodes: the mean pinned height, this loose (anchors hang at
    /// one ceiling, give or take).
    pub height_sigma_m: f32,
}

impl Default for MeshOptions {
    fn default() -> Self {
        Self {
            max_iterations: 40,
            tolerance_m: 1e-3,
            damping: 1e-3,
            huber_k: 3.0,
            offset_m: 0.0,
            sigma_floor_m: 0.1,
            height_sigma_m: 0.3,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Solved {
    /// Per node; `None` where the node couldn't be placed.
    pub positions: Vec<Option<Vec3>>,
    /// 1-sigma per axis per node, from the inverse normal matrix; zero for pins.
    pub sigma_m: Vec<Option<Vec3>>,
    /// RMS of link residuals over their sigmas, about 1 when the sigmas are honest.
    pub normalized_rms: f32,
    pub iterations: u8,
    /// Links whose residual the Huber weight cut down at the end.
    pub outliers: usize,
}

/// Solves `nodes` positions from `pins` and `links`.
pub fn solve(nodes: usize, pins: &[Pin], links: &[LinkObs], opts: &MeshOptions) -> Solved {
    solve_with_heights(nodes, pins, links, &[], opts)
}

/// A node's height from elsewhere than ranges: barometry against a reference (FR-M3), metres and
/// 1-sigma. It replaces the loose prior for that node.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeightObs {
    pub node: usize,
    pub z_m: f32,
    pub sigma_m: f32,
}

/// [`solve`], with height observations for some nodes; the others keep the prior around the pins'
/// mean height.
pub fn solve_with_heights(
    nodes: usize,
    pins: &[Pin],
    links: &[LinkObs],
    heights: &[HeightObs],
    opts: &MeshOptions,
) -> Solved {
    let mut pos: Vec<Option<Vec3>> = vec![None; nodes];
    let mut pinned = vec![false; nodes];
    for p in pins {
        if p.node < nodes {
            pos[p.node] = Some(p.position);
            pinned[p.node] = true;
        }
    }
    let links: Vec<LinkObs> = links
        .iter()
        .filter(|l| l.a < nodes && l.b < nodes && l.a != l.b)
        .map(|l| LinkObs {
            distance_m: (l.distance_m - opts.offset_m).max(0.0),
            sigma_m: l.sigma_m.max(opts.sigma_floor_m),
            ..*l
        })
        .collect();
    let height = if pins.is_empty() {
        0.0
    } else {
        pins.iter().map(|p| p.position[2]).sum::<f32>() / pins.len() as f32
    };
    // Each node's height and its weight: the observation where there is one, else the prior.
    let height_of = |n: usize| -> (f32, f32) {
        heights
            .iter()
            .find(|h| h.node == n)
            .map_or((height, opts.height_sigma_m), |h| (h.z_m, h.sigma_m.max(0.01)))
    };

    // Seed outward from the pins: nodes with three placed neighbours first; when none is left,
    // one with two, choosing of its two mirror images the one nearer the pins' centre (nodes
    // tend to sit inside the pinned hull). The joint refinement settles the rest.
    let centre: Vec3 = {
        let mut c = [0.0f32; 3];
        for p in pins {
            for (ci, pi) in c.iter_mut().zip(p.position) {
                *ci += pi / pins.len().max(1) as f32;
            }
        }
        c
    };
    let mut gave_up = vec![false; nodes];
    loop {
        let mut best: Option<(usize, Vec<RangeObs>)> = None;
        for n in 0..nodes {
            if pos[n].is_some() || gave_up[n] {
                continue;
            }
            let obs: Vec<RangeObs> = links
                .iter()
                .filter_map(|l| {
                    let other = if l.a == n {
                        l.b
                    } else if l.b == n {
                        l.a
                    } else {
                        return None;
                    };
                    Some(RangeObs {
                        anchor: pos[other]?,
                        distance_m: l.distance_m,
                        sigma_m: l.sigma_m,
                    })
                })
                .collect();
            if obs.len() >= 2 && best.as_ref().is_none_or(|(_, b)| obs.len() > b.len()) {
                best = Some((n, obs));
            }
        }
        let Some((n, obs)) = best else {
            break;
        };
        let mut centroid = [0.0f32; 3];
        for o in &obs {
            for (ci, ai) in centroid.iter_mut().zip(o.anchor) {
                *ci += ai / obs.len() as f32;
            }
        }
        let (z_n, sigma_n) = height_of(n);
        let opts_seed = Options {
            height: Height::Observed {
                z: z_n,
                sigma_m: sigma_n,
            },
            ..Options::default()
        };
        let starts: Vec<Vec3> = if obs.len() >= 3 {
            // Off the anchors' centroid, so a degenerate start doesn't stall the solver.
            vec![[centroid[0] + 0.5, centroid[1] + 0.3, z_n]]
        } else {
            // Two anchors: the two points at the right distances, either side of the baseline.
            let (a, b) = (obs[0], obs[1]);
            let base: Vec3 = core::array::from_fn(|i| b.anchor[i] - a.anchor[i]);
            let len = libm::sqrtf(base[0] * base[0] + base[1] * base[1]).max(1e-3);
            let perp = [-base[1] / len, base[0] / len];
            let along =
                ((a.distance_m * a.distance_m - b.distance_m * b.distance_m + len * len) / (2.0 * len)).clamp(0.0, len);
            let h = libm::sqrtf((a.distance_m * a.distance_m - along * along).max(0.25));
            let foot = [a.anchor[0] + base[0] / len * along, a.anchor[1] + base[1] / len * along];
            vec![
                [foot[0] + perp[0] * h, foot[1] + perp[1] * h, z_n],
                [foot[0] - perp[0] * h, foot[1] - perp[1] * h, z_n],
            ]
        };
        let mut fix: Option<Vec3> = None;
        if obs.len() >= 3 {
            fix = multilaterate(&obs, starts[0], &opts_seed).map(|f| f.position);
        } else {
            // Two anchors can't be refined alone (the solver wants three): of the two mirror
            // candidates take the one clear of the nodes already placed (nodes don't sit on
            // top of each other), and nearer the pins' centre when both are clear.
            let mut best_score = f32::NEG_INFINITY;
            for start in starts {
                let clearance = (0..nodes)
                    .filter_map(|m| pos[m])
                    .map(|p| norm(&core::array::from_fn(|i| start[i] - p[i])))
                    .fold(f32::INFINITY, f32::min)
                    .min(3.0);
                let to_centre = norm(&core::array::from_fn(|i| start[i] - centre[i]));
                let score = clearance - 0.01 * to_centre;
                if score > best_score {
                    best_score = score;
                    fix = Some(start);
                }
            }
        }
        match fix {
            Some(p) => pos[n] = Some(p),
            // Can't be placed from here; leave it out of the seeding for good.
            None => gave_up[n] = true,
        }
    }

    // Refine the unpinned, placed nodes together.
    let unknowns: Vec<usize> = (0..nodes).filter(|&n| pos[n].is_some() && !pinned[n]).collect();
    let index = |n: usize| unknowns.iter().position(|&u| u == n);
    let m = 3 * unknowns.len();
    let mut iterations = 0;
    let mut outliers = 0;
    let mut chi2 = 0.0f32;
    let mut used = 0usize;
    if m > 0 {
        while iterations < opts.max_iterations {
            iterations += 1;
            let mut a = vec![0.0f64; m * m];
            let mut g = vec![0.0f64; m];
            chi2 = 0.0;
            used = 0;
            outliers = 0;
            for l in &links {
                let (Some(pa), Some(pb)) = (pos[l.a], pos[l.b]) else {
                    continue;
                };
                let (ia, ib) = (index(l.a), index(l.b));
                if ia.is_none() && ib.is_none() {
                    continue;
                }
                let diff: Vec3 = core::array::from_fn(|i| pa[i] - pb[i]);
                let rho = norm(&diff);
                if rho < 1e-6 {
                    continue;
                }
                let u: Vec3 = core::array::from_fn(|i| diff[i] / rho);
                let r = rho - l.distance_m;
                let huber = robust_weight(r / l.sigma_m, opts.huber_k * cut_scale(iterations));
                if huber < 1.0 {
                    outliers += 1;
                }
                let w = f64::from(huber / (l.sigma_m * l.sigma_m));
                chi2 += huber * r * r / (l.sigma_m * l.sigma_m);
                used += 1;
                // d rho / d pa = u, d rho / d pb = -u.
                let rows = jacobian_rows(ia, ib, &u);
                for &(ri, ji) in &rows {
                    g[ri] += w * ji * f64::from(r);
                    for &(rj, jj) in &rows {
                        a[ri * m + rj] += w * ji * jj;
                    }
                }
            }
            // Heights: observed, else the prior.
            for (k, &n) in unknowns.iter().enumerate() {
                let (z_n, sigma_n) = height_of(n);
                let wz = 1.0 / f64::from(sigma_n * sigma_n);
                let z = pos[n].expect("placed")[2];
                let r = f64::from(z - z_n);
                a[(3 * k + 2) * m + 3 * k + 2] += wz;
                g[3 * k + 2] += wz * r;
            }
            for i in 0..m {
                a[i * m + i] *= 1.0 + f64::from(opts.damping);
            }
            let Some(step) = cholesky_solve(&a, &g, m) else {
                break;
            };
            let mut longest = 0.0f32;
            for (k, &n) in unknowns.iter().enumerate() {
                let p = pos[n].as_mut().expect("placed");
                let s: Vec3 = core::array::from_fn(|i| -step[3 * k + i] as f32);
                for i in 0..3 {
                    p[i] += s[i];
                }
                longest = longest.max(norm(&s));
            }
            if longest < opts.tolerance_m && cut_scale(iterations) <= 1.0 {
                break;
            }
        }
    }

    // Covariance from the final normal matrix, without damping.
    let mut sigma: Vec<Option<Vec3>> = vec![None; nodes];
    for n in 0..nodes {
        if pinned[n] {
            sigma[n] = Some([0.0; 3]);
        }
    }
    if m > 0 {
        let mut a = vec![0.0f64; m * m];
        for l in &links {
            let (Some(pa), Some(pb)) = (pos[l.a], pos[l.b]) else {
                continue;
            };
            let (ia, ib) = (index(l.a), index(l.b));
            if ia.is_none() && ib.is_none() {
                continue;
            }
            let diff: Vec3 = core::array::from_fn(|i| pa[i] - pb[i]);
            let rho = norm(&diff);
            if rho < 1e-6 {
                continue;
            }
            let u: Vec3 = core::array::from_fn(|i| diff[i] / rho);
            let r = rho - l.distance_m;
            let huber = robust_weight(r / l.sigma_m, opts.huber_k);
            let w = f64::from(huber / (l.sigma_m * l.sigma_m));
            let rows = jacobian_rows(ia, ib, &u);
            for &(ri, ji) in &rows {
                for &(rj, jj) in &rows {
                    a[ri * m + rj] += w * ji * jj;
                }
            }
        }
        for (k, &n) in unknowns.iter().enumerate() {
            let (_, sigma_n) = height_of(n);
            a[(3 * k + 2) * m + 3 * k + 2] += 1.0 / f64::from(sigma_n * sigma_n);
        }
        if let Some(inv_diag) = cholesky_inverse_diag(&a, m) {
            for (k, &n) in unknowns.iter().enumerate() {
                sigma[n] = Some(core::array::from_fn(|i| {
                    libm::sqrtf(inv_diag[3 * k + i].max(0.0) as f32)
                }));
            }
        }
    }
    let dof = used.saturating_sub(m).max(1) as f32;
    Solved {
        positions: pos,
        sigma_m: sigma,
        normalized_rms: libm::sqrtf(chi2 / dof),
        iterations,
        outliers,
    }
}

/// The non-zero entries of a link's Jacobian row: `d rho / d pa = u`, `d rho / d pb = -u`, at
/// the unknown ends' columns.
fn jacobian_rows(ia: Option<usize>, ib: Option<usize>, u: &Vec3) -> Vec<(usize, f64)> {
    let mut rows: Vec<(usize, f64)> = Vec::with_capacity(6);
    if let Some(ia) = ia {
        for (i, ui) in u.iter().enumerate() {
            rows.push((3 * ia + i, f64::from(*ui)));
        }
    }
    if let Some(ib) = ib {
        for (i, ui) in u.iter().enumerate() {
            rows.push((3 * ib + i, -f64::from(*ui)));
        }
    }
    rows
}

/// The robust cut starts wide and halves every two iterations down to the final one, so a poor
/// seed doesn't leave the good links cut down with the bad, and a bad link stands out once the
/// good ones agree.
fn cut_scale(iteration: u8) -> f32 {
    match iteration {
        0..=2 => 16.0,
        3..=4 => 8.0,
        5..=6 => 4.0,
        7..=8 => 2.0,
        _ => 1.0,
    }
}

/// The weight of a residual `t` sigmas out: full within `k`, falling as the square beyond, so
/// a bounce metres long pulls almost nothing.
fn robust_weight(t: f32, k: f32) -> f32 {
    let t = libm::fabsf(t);
    if t > k { (k / t) * (k / t) } else { 1.0 }
}

/// Cholesky factor of a symmetric positive-definite `n×n` matrix (row-major), lower triangle.
fn cholesky(a: &[f64], n: usize) -> Option<Vec<f64>> {
    let mut l = vec![0.0f64; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut sum = a[i * n + j];
            for k in 0..j {
                sum -= l[i * n + k] * l[j * n + k];
            }
            if i == j {
                if sum <= 1e-12 {
                    return None;
                }
                l[i * n + i] = libm::sqrt(sum);
            } else {
                l[i * n + j] = sum / l[j * n + j];
            }
        }
    }
    Some(l)
}

fn cholesky_solve_with(l: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    let mut y = vec![0.0f64; n];
    for i in 0..n {
        let mut sum = b[i];
        for k in 0..i {
            sum -= l[i * n + k] * y[k];
        }
        y[i] = sum / l[i * n + i];
    }
    let mut x = vec![0.0f64; n];
    for i in (0..n).rev() {
        let mut sum = y[i];
        for k in i + 1..n {
            sum -= l[k * n + i] * x[k];
        }
        x[i] = sum / l[i * n + i];
    }
    x
}

fn cholesky_solve(a: &[f64], b: &[f64], n: usize) -> Option<Vec<f64>> {
    let l = cholesky(a, n)?;
    Some(cholesky_solve_with(&l, b, n))
}

/// The diagonal of the inverse of a symmetric positive-definite matrix.
fn cholesky_inverse_diag(a: &[f64], n: usize) -> Option<Vec<f64>> {
    let l = cholesky(a, n)?;
    let mut diag = vec![0.0f64; n];
    let mut e = vec![0.0f64; n];
    for i in 0..n {
        e.iter_mut().for_each(|v| *v = 0.0);
        e[i] = 1.0;
        let col = cholesky_solve_with(&l, &e, n);
        diag[i] = col[i];
    }
    Some(diag)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Anchors on a 3 × 3 grid 6 m apart under a ceiling, heights 2.3–2.7 m.
    fn grid() -> Vec<Vec3> {
        let mut v = Vec::new();
        for i in 0..9 {
            let (x, y) = ((i % 3) as f32 * 6.0, (i / 3) as f32 * 6.0);
            v.push([x, y, 2.5 + 0.2 * (((i * 7) % 5) as f32 - 2.0) / 2.0]);
        }
        v
    }

    struct Lcg(u64);
    impl Lcg {
        fn unit(&mut self) -> f32 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((self.0 >> 40) as f32) / (1u64 << 24) as f32
        }
        fn gauss(&mut self) -> f32 {
            // Sum of twelve uniforms, minus six: mean 0, variance 1.
            (0..12).map(|_| self.unit()).sum::<f32>() - 6.0
        }
    }

    fn links_of(truth: &[Vec3], noise_m: f32, offset_m: f32, rng: &mut Lcg) -> Vec<LinkObs> {
        let mut links = Vec::new();
        for a in 0..truth.len() {
            for b in a + 1..truth.len() {
                let d: Vec3 = core::array::from_fn(|i| truth[a][i] - truth[b][i]);
                let d = norm(&d);
                if d > 9.0 {
                    continue;
                }
                links.push(LinkObs {
                    a,
                    b,
                    distance_m: d + offset_m + noise_m * rng.gauss(),
                    sigma_m: 0.1,
                });
            }
        }
        links
    }

    fn pins(truth: &[Vec3], which: &[usize]) -> Vec<Pin> {
        which
            .iter()
            .map(|&n| Pin {
                node: n,
                position: truth[n],
            })
            .collect()
    }

    fn error(a: Vec3, b: Vec3) -> f32 {
        norm(&core::array::from_fn(|i| a[i] - b[i]))
    }

    #[test]
    fn barometric_heights_put_wall_anchors_at_their_heights() {
        // Anchors on walls at very different heights: the prior drags them to the pins' mean.
        let mut truth = grid();
        let heights = [2.4, 0.6, 2.5, 1.2, 0.4, 2.0, 2.6, 0.9, 2.5];
        for (t, z) in truth.iter_mut().zip(heights) {
            t[2] = z;
        }
        let mut rng = Lcg(3);
        let links = links_of(&truth, 0.05, 0.0, &mut rng);
        let pinned = [0, 2, 6, 8];
        let opts = MeshOptions::default();
        let prior_only = solve(truth.len(), &pins(&truth, &pinned), &links, &opts);
        let worst_prior = (0..truth.len())
            .map(|n| (prior_only.positions[n].expect("placed")[2] - truth[n][2]).abs())
            .fold(0.0f32, f32::max);
        assert!(worst_prior > 0.5, "the prior alone gets heights wrong: {worst_prior}");
        let obs: Vec<HeightObs> = (0..truth.len())
            .filter(|n| !pinned.contains(n))
            .map(|n| HeightObs {
                node: n,
                z_m: truth[n][2] + 0.1 * rng.gauss(),
                sigma_m: 0.15,
            })
            .collect();
        let with_baro = solve_with_heights(truth.len(), &pins(&truth, &pinned), &links, &obs, &opts);
        for (n, t) in truth.iter().enumerate() {
            let p = with_baro.positions[n].expect("placed");
            assert!((p[2] - t[2]).abs() < 0.3, "node {n}: z {} vs {}", p[2], t[2]);
            assert!(error(p, *t) < 0.35, "node {n}: {p:?} vs {t:?}");
        }
    }

    #[test]
    fn a_grid_with_four_pins_comes_back_within_decimetres() {
        let truth = grid();
        let mut rng = Lcg(1);
        let links = links_of(&truth, 0.1, 1.5, &mut rng);
        let solved = solve(
            truth.len(),
            &pins(&truth, &[0, 2, 6, 8]),
            &links,
            &MeshOptions {
                offset_m: 1.5,
                ..MeshOptions::default()
            },
        );
        for (n, t) in truth.iter().enumerate() {
            let p = solved.positions[n].expect("placed");
            assert!(error(p, *t) < 0.3, "node {n}: {p:?} vs {t:?}");
            let s = solved.sigma_m[n].expect("sigma");
            assert!(s[0] < 0.5 && s[1] < 0.5, "node {n}: sigma {s:?}");
        }
        assert!(solved.normalized_rms < 2.0, "{}", solved.normalized_rms);
        assert_eq!(solved.outliers, 0);
    }

    #[test]
    fn a_bounced_link_is_down_weighted_and_counted() {
        let truth = grid();
        let mut rng = Lcg(2);
        let mut links = links_of(&truth, 0.1, 0.0, &mut rng);
        // One link reads 3 m long: a reflection.
        links[5].distance_m += 3.0;
        let solved = solve(
            truth.len(),
            &pins(&truth, &[0, 2, 6, 8]),
            &links,
            &MeshOptions::default(),
        );
        for (n, t) in truth.iter().enumerate() {
            let p = solved.positions[n].expect("placed");
            assert!(error(p, *t) < 0.5, "node {n}: {p:?} vs {t:?}");
        }
        assert!(solved.outliers >= 1);
    }

    #[test]
    fn a_node_with_too_few_links_stays_unsolved_and_the_rest_are_placed() {
        let truth = grid();
        let mut rng = Lcg(3);
        let links: Vec<LinkObs> = links_of(&truth, 0.05, 0.0, &mut rng)
            .into_iter()
            .filter(|l| l.a != 4 && l.b != 4)
            .chain([LinkObs {
                a: 4,
                b: 0,
                distance_m: 8.5,
                sigma_m: 0.1,
            }])
            .collect();
        // Without its centre the ring of the others can flex between four corner pins (each
        // node slides against its two pins' line); a fifth pin on the ring holds it.
        let solved = solve(
            truth.len(),
            &pins(&truth, &[0, 2, 6, 7, 8]),
            &links,
            &MeshOptions::default(),
        );
        assert!(solved.positions[4].is_none());
        for n in [1, 3, 5] {
            let p = solved.positions[n].expect("placed");
            assert!(error(p, truth[n]) < 0.4, "node {n}: {p:?} vs {:?}", truth[n]);
        }
    }

    #[test]
    fn three_pins_hold_the_frame_and_sigmas_grow_away_from_them() {
        let truth = grid();
        let mut rng = Lcg(4);
        let links = links_of(&truth, 0.1, 0.0, &mut rng);
        let solved = solve(truth.len(), &pins(&truth, &[0, 1, 3]), &links, &MeshOptions::default());
        let near = solved.sigma_m[4].expect("sigma")[0];
        let far = solved.sigma_m[8].expect("sigma")[0];
        assert!(far >= near, "far {far} near {near}");
        for (n, t) in truth.iter().enumerate() {
            let p = solved.positions[n].expect("placed");
            assert!(error(p, *t) < 0.6, "node {n}: {p:?} vs {t:?}");
        }
    }
}
