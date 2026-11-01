//! Absolute referencing (architecture §11): the transform from the mesh's own frame to the world
//! frame, and the find pointer built on it.
//!
//! World frame: x east, y north, z up. Headings are radians counter-clockwise from +x (east); use
//! [`compass_to_heading`] for clockwise-from-north compass readings.
//!
//! A self-surveyed mesh knows its shape but not its orientation, handedness or origin. Barometry
//! makes its z axis vertical, so what remains is horizontal: a yaw, a possible mirror, and an
//! offset. Pins fix all three. Heading observations from a device with its own compass (a HUD) fix
//! yaw and handedness, which is all pointing needs.

use core::f32::consts::{FRAC_PI_2, PI, TAU};

use crate::Vec3;

/// Mesh frame → world frame: mirror (y ↦ −y), then rotate by `yaw_rad`, then translate.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Frame {
    pub yaw_rad: f32,
    pub mirrored: bool,
    pub offset: Vec3,
}

impl Frame {
    pub const IDENTITY: Self = Self {
        yaw_rad: 0.0,
        mirrored: false,
        offset: [0.0; 3],
    };

    pub fn to_world(&self, p: Vec3) -> Vec3 {
        let y = if self.mirrored { -p[1] } else { p[1] };
        let (s, c) = (libm::sinf(self.yaw_rad), libm::cosf(self.yaw_rad));
        [
            c * p[0] - s * y + self.offset[0],
            s * p[0] + c * y + self.offset[1],
            p[2] + self.offset[2],
        ]
    }

    pub fn heading_to_world(&self, heading_rad: f32) -> f32 {
        wrap(if self.mirrored { -heading_rad } else { heading_rad } + self.yaw_rad)
    }
}

/// Converts a compass reading (clockwise from north) into a heading (counter-clockwise from east).
/// The compass should be corrected for magnetic declination first if the world frame uses true
/// north.
pub fn compass_to_heading(compass_rad: f32) -> f32 {
    wrap(FRAC_PI_2 - compass_rad)
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PinFit {
    pub frame: Frame,
    /// Horizontal RMS misfit of the pins after the transform.
    pub rms_m: f32,
}

/// Fits the frame to pinned nodes: their mesh-frame positions and surveyed world positions.
///
/// Horizontal least-squares rotation (2D Kabsch, no scale; CS ranges already carry scale), tried
/// plain and mirrored, keeping the better fit. Needs ≥ 2 pins with horizontal separation.
/// Handedness needs ≥ 3 non-collinear pins; with collinear pins both fits tie and the unmirrored
/// one is kept. The z offset is the mean height difference.
pub fn fit_pins(mesh: &[Vec3], world: &[Vec3]) -> Option<PinFit> {
    if mesh.len() != world.len() || mesh.len() < 2 {
        return None;
    }
    let plain = fit_pins_handed(mesh, world, false)?;
    let mirrored = fit_pins_handed(mesh, world, true)?;
    // Prefer unmirrored unless mirrored is clearly better (collinear pins tie).
    Some(if mirrored.rms_m < plain.rms_m - 1e-4 {
        mirrored
    } else {
        plain
    })
}

fn fit_pins_handed(mesh: &[Vec3], world: &[Vec3], mirrored: bool) -> Option<PinFit> {
    let n = mesh.len() as f32;
    let flip = |p: &Vec3| [p[0], if mirrored { -p[1] } else { p[1] }];
    let mean = |pts: &mut dyn Iterator<Item = [f32; 2]>| {
        let (mut x, mut y) = (0.0, 0.0);
        for p in pts {
            x += p[0];
            y += p[1];
        }
        [x / n, y / n]
    };
    let cm = mean(&mut mesh.iter().map(flip));
    let cw = mean(&mut world.iter().map(|p| [p[0], p[1]]));

    let (mut dot, mut cross, mut spread) = (0.0, 0.0, 0.0);
    for (m, w) in mesh.iter().zip(world) {
        let a = flip(m);
        let (ax, ay) = (a[0] - cm[0], a[1] - cm[1]);
        let (bx, by) = (w[0] - cw[0], w[1] - cw[1]);
        dot += ax * bx + ay * by;
        cross += ax * by - ay * bx;
        spread += ax * ax + ay * ay;
    }
    if spread < 1e-6 {
        return None;
    }
    let yaw = libm::atan2f(cross, dot);
    let (s, c) = (libm::sinf(yaw), libm::cosf(yaw));
    let dz = mesh.iter().zip(world).map(|(m, w)| w[2] - m[2]).sum::<f32>() / n;
    let frame = Frame {
        yaw_rad: yaw,
        mirrored,
        offset: [cw[0] - (c * cm[0] - s * cm[1]), cw[1] - (s * cm[0] + c * cm[1]), dz],
    };
    let sq: f32 = mesh
        .iter()
        .zip(world)
        .map(|(m, w)| {
            let p = frame.to_world(*m);
            (p[0] - w[0]) * (p[0] - w[0]) + (p[1] - w[1]) * (p[1] - w[1])
        })
        .sum();
    Some(PinFit {
        frame,
        rms_m: libm::sqrtf(sq / n),
    })
}

/// A device with an absolute heading sensor, walking straight while the mesh tracks it.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HeadingObs {
    /// Walking direction from the mesh track, mesh frame.
    pub mesh_heading_rad: f32,
    /// The device's own heading, world frame.
    pub world_heading_rad: f32,
    /// Confidence, e.g. higher for faster, straighter, longer segments.
    pub weight: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HeadingFit {
    pub yaw_rad: f32,
    pub mirrored: bool,
    /// Mean resultant length of the heading differences: 1 = all agree, 0 = no information.
    pub concentration: f32,
    /// Concentration of the chosen handedness minus the other's. Small means the observations
    /// did not cover enough different directions to tell a mirror apart.
    pub handedness_margin: f32,
}

/// Estimates the mesh's yaw and handedness from heading observations: the weighted circular mean
/// of the world − mesh heading differences, for whichever handedness makes them agree better.
/// Local magnetic disturbances average out only if observations come from many places.
pub fn fit_headings(obs: &[HeadingObs]) -> Option<HeadingFit> {
    let plain = circular_mean(obs, false)?;
    let mirrored = circular_mean(obs, true)?;
    let (best, other, is_mirrored) = if mirrored.1 > plain.1 {
        (mirrored, plain, true)
    } else {
        (plain, mirrored, false)
    };
    Some(HeadingFit {
        yaw_rad: best.0,
        mirrored: is_mirrored,
        concentration: best.1,
        handedness_margin: best.1 - other.1,
    })
}

/// `(mean, resultant length)` of `world − mesh` (or `world + mesh` when mirrored).
fn circular_mean(obs: &[HeadingObs], mirrored: bool) -> Option<(f32, f32)> {
    let (mut c, mut s, mut w) = (0.0, 0.0, 0.0);
    for o in obs {
        let mesh = if mirrored {
            -o.mesh_heading_rad
        } else {
            o.mesh_heading_rad
        };
        let d = o.world_heading_rad - mesh;
        c += o.weight * libm::cosf(d);
        s += o.weight * libm::sinf(d);
        w += o.weight;
    }
    (w > 0.0).then(|| (libm::atan2f(s, c), libm::hypotf(c, s) / w))
}

/// What a HUD needs to point at something.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Pointer {
    /// Bearing relative to where the viewer faces, radians, positive to the left.
    pub relative_azimuth_rad: f32,
    /// Positive above the viewer.
    pub elevation_rad: f32,
    pub distance_m: f32,
}

/// Pointer from a viewer at `from` facing `heading_rad` to `to`, all in the world frame.
pub fn pointer(from: Vec3, to: Vec3, heading_rad: f32) -> Pointer {
    let (dx, dy, dz) = (to[0] - from[0], to[1] - from[1], to[2] - from[2]);
    let horizontal = libm::hypotf(dx, dy);
    Pointer {
        relative_azimuth_rad: wrap(libm::atan2f(dy, dx) - heading_rad),
        elevation_rad: libm::atan2f(dz, horizontal),
        distance_m: libm::hypotf(horizontal, dz),
    }
}

/// Wraps an angle into (−π, π].
fn wrap(a: f32) -> f32 {
    let mut a = a % TAU;
    if a > PI {
        a -= TAU;
    } else if a <= -PI {
        a += TAU;
    }
    a
}

#[cfg(test)]
mod tests {
    use navgrid_ranging::sim::SplitMix64;

    use super::*;

    const DEG: f32 = PI / 180.0;

    fn close(a: f32, b: f32, tol: f32) -> bool {
        libm::fabsf(wrap(a - b)) < tol
    }

    /// World pins and the same pins as a mesh would see them under `truth`.
    fn pins(truth: &Frame) -> ([Vec3; 4], [Vec3; 4]) {
        let mesh: [Vec3; 4] = [[0.0, 0.0, 1.0], [8.0, 0.5, 2.5], [7.0, 6.0, 0.4], [1.0, 5.0, 1.8]];
        (mesh, mesh.map(|p| truth.to_world(p)))
    }

    #[test]
    fn fit_pins_recovers_frame_either_handedness() {
        for mirrored in [false, true] {
            let truth = Frame {
                yaw_rad: 33.0 * DEG,
                mirrored,
                offset: [12.0, -4.0, 0.3],
            };
            let (mesh, world) = pins(&truth);
            let fit = fit_pins(&mesh, &world).unwrap();
            assert_eq!(fit.frame.mirrored, mirrored);
            assert!(close(fit.frame.yaw_rad, truth.yaw_rad, 1e-3), "{fit:?}");
            assert!(fit.rms_m < 1e-3, "{fit:?}");
            let p = fit.frame.to_world([3.0, 3.0, 1.0]);
            let q = truth.to_world([3.0, 3.0, 1.0]);
            assert!((0..3).all(|i| libm::fabsf(p[i] - q[i]) < 1e-3));
        }
    }

    #[test]
    fn two_pins_give_yaw_but_assume_unmirrored() {
        let truth = Frame {
            yaw_rad: -70.0 * DEG,
            mirrored: false,
            offset: [0.0; 3],
        };
        let (mesh, world) = pins(&truth);
        let fit = fit_pins(&mesh[..2], &world[..2]).unwrap();
        assert!(!fit.frame.mirrored);
        assert!(close(fit.frame.yaw_rad, truth.yaw_rad, 1e-3));
        assert!(fit_pins(&mesh[..1], &world[..1]).is_none());
    }

    #[test]
    fn noisy_compass_headings_give_yaw_and_handedness() {
        for mirrored in [false, true] {
            let truth = Frame {
                yaw_rad: 40.0 * DEG,
                mirrored,
                offset: [0.0; 3],
            };
            let mut rng = SplitMix64::new(9);
            // 40 straight walks in all directions; compass off by 15° (1σ) each time.
            let obs: [HeadingObs; 40] = core::array::from_fn(|_| {
                let mesh = (rng.uniform() as f32 * 2.0 - 1.0) * PI;
                HeadingObs {
                    mesh_heading_rad: mesh,
                    world_heading_rad: truth.heading_to_world(mesh) + 15.0 * DEG * rng.normal() as f32,
                    weight: 1.0,
                }
            });
            let fit = fit_headings(&obs).unwrap();
            assert_eq!(fit.mirrored, mirrored, "{fit:?}");
            assert!(close(fit.yaw_rad, truth.yaw_rad, 5.0 * DEG), "{fit:?}");
            assert!(fit.handedness_margin > 0.3, "{fit:?}");
        }
    }

    #[test]
    fn one_direction_cannot_tell_handedness() {
        let obs = [HeadingObs {
            mesh_heading_rad: 0.3,
            world_heading_rad: 1.0,
            weight: 1.0,
        }; 10];
        let fit = fit_headings(&obs).unwrap();
        assert!(fit.handedness_margin < 1e-4, "{fit:?}");
    }

    #[test]
    fn pointer_bearing_and_elevation() {
        // Facing north; keys 3 m east and 1 m lower: 90° to the right, looking down.
        let p = pointer([0.0, 0.0, 1.7], [3.0, 0.0, 0.7], compass_to_heading(0.0));
        assert!(close(p.relative_azimuth_rad, -90.0 * DEG, 1e-4), "{p:?}");
        assert!(p.elevation_rad < 0.0);
        assert!(libm::fabsf(p.distance_m - libm::sqrtf(10.0)) < 1e-4);
    }

    #[test]
    fn compass_and_heading_conventions() {
        assert!(close(compass_to_heading(0.0), 90.0 * DEG, 1e-6)); // north
        assert!(close(compass_to_heading(90.0 * DEG), 0.0, 1e-6)); // east
        assert!(close(wrap(3.0 * PI), PI, 1e-5));
    }
}
