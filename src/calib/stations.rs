// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Registration from the Lighthouse base stations. The Rift sensors catch the stations' laser
//! sweeps as brief blobs at fixed image positions; SteamVR knows each station's pose in its raw
//! universe. Those rays and points pin lighthouse_from_world without any controller held: every
//! session, with nothing in hand, against the frame every Lighthouse device is tracked in.

use nalgebra::{
    DMatrix, DVector, Isometry3, Point3, Translation3, UnitQuaternion, Vector2, Vector3,
};

use super::camera::CameraModel;

/// A station spot in one sensor: the station's index, the blob centroid, how many frames saw it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Spot {
    pub camera: usize,
    pub station: usize,
    pub xy: Vector2<f64>,
    pub frames: usize,
}

/// The centroid of the unlabelled blobs within `radius` px of `predicted` over many frames: the
/// median first, then the mean of those within `tight` px of it. None with fewer than `min`.
pub fn spot(
    blobs: &[Vector2<f64>],
    predicted: Vector2<f64>,
    radius: f64,
    tight: f64,
    min: usize,
) -> Option<(Vector2<f64>, usize)> {
    let near: Vec<Vector2<f64>> = blobs
        .iter()
        .filter(|b| (*b - predicted).norm() <= radius)
        .copied()
        .collect();
    if near.len() < min {
        return None;
    }
    let med = |k: usize| {
        let mut v: Vec<f64> = near.iter().map(|b| b[k]).collect();
        v.sort_by(f64::total_cmp);
        v[v.len() / 2]
    };
    let m = Vector2::new(med(0), med(1));
    let core: Vec<Vector2<f64>> = near
        .iter()
        .filter(|b| (*b - m).norm() <= tight)
        .copied()
        .collect();
    (core.len() >= min).then(|| {
        (
            core.iter().sum::<Vector2<f64>>() / core.len() as f64,
            core.len(),
        )
    })
}

/// Where each station's spot is expected under a registration: the station origin, moved
/// `aperture` metres along its local axis `facing` (the laser exit sits in front of the origin).
pub fn station_point(station: &Isometry3<f64>, aperture: f64) -> Point3<f64> {
    station * Point3::new(0.0, 0.0, -aperture)
}

fn exp6(v: &[f64]) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(v[3], v[4], v[5]),
        UnitQuaternion::from_scaled_axis(Vector3::new(v[0], v[1], v[2])),
    )
}

fn residual(
    cams: &[CameraModel],
    world_from_cam: &[Isometry3<f64>],
    stations: &[Isometry3<f64>],
    x: &Isometry3<f64>,
    aperture: f64,
    s: &Spot,
) -> Option<Vector2<f64>> {
    let pw = x.inverse() * station_point(&stations[s.station], aperture);
    Some(s.xy - cams[s.camera].project(&(world_from_cam[s.camera].inverse() * pw).coords)?)
}

#[derive(Debug, Clone, PartialEq)]
pub struct Fit {
    pub lighthouse_from_world: Isometry3<f64>,
    pub aperture: f64,
    /// Per spot, the residual after the fit (px).
    pub residuals: Vec<f64>,
}

/// Gauss-Newton on the registration (6) and, with `fit_aperture`, a common aperture offset.
pub fn solve(
    cams: &[CameraModel],
    world_from_cam: &[Isometry3<f64>],
    stations: &[Isometry3<f64>],
    spots: &[Spot],
    initial: &Isometry3<f64>,
    aperture: f64,
    fit_aperture: bool,
) -> Option<Fit> {
    let n = if fit_aperture { 7 } else { 6 };
    if 2 * spots.len() < n {
        return None;
    }
    let (mut x, mut a) = (*initial, aperture);
    for _ in 0..50 {
        let (mut h, mut g) = (DMatrix::<f64>::zeros(n, n), DVector::<f64>::zeros(n));
        for s in spots {
            let r0 = residual(cams, world_from_cam, stations, &x, a, s)?;
            let mut j = DMatrix::<f64>::zeros(2, n);
            for k in 0..n {
                let eps = 1e-6;
                let (xk, ak) = if k < 6 {
                    let mut d = [0.0; 6];
                    d[k] = eps;
                    (exp6(&d) * x, a)
                } else {
                    (x, a + eps)
                };
                let rk = residual(cams, world_from_cam, stations, &xk, ak, s)?;
                let col = -(rk - r0) / eps;
                j[(0, k)] = col.x;
                j[(1, k)] = col.y;
            }
            h += j.transpose() * &j;
            g += j.transpose() * DVector::from_vec(vec![r0.x, r0.y]);
        }
        for k in 0..n {
            h[(k, k)] *= 1.0 + 1e-6;
        }
        let step = h.cholesky()?.solve(&g);
        x = exp6(&step.as_slice()[..6]) * x;
        if fit_aperture {
            a += step[6];
        }
        if step.norm() < 1e-10 {
            break;
        }
    }
    let residuals = spots
        .iter()
        .filter_map(|s| residual(cams, world_from_cam, stations, &x, a, s).map(|r| r.norm()))
        .collect();
    Some(Fit {
        lighthouse_from_world: x,
        aperture: a,
        residuals,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::synth;

    #[test]
    fn the_spot_is_the_core_of_recurring_blobs() {
        let mut blobs = vec![Vector2::new(100.2, 50.1); 40];
        blobs.extend(vec![Vector2::new(100.0, 49.9); 40]);
        blobs.extend(vec![Vector2::new(112.0, 50.0); 10]); // a neighbouring flare
        blobs.push(Vector2::new(400.0, 400.0));
        let (c, n) = spot(&blobs, Vector2::new(103.0, 52.0), 20.0, 3.0, 50).unwrap();
        assert_eq!(n, 80);
        assert!((c - Vector2::new(100.1, 50.0)).norm() < 1e-9);
        assert!(spot(&blobs, Vector2::new(300.0, 300.0), 20.0, 3.0, 50).is_none());
    }

    /// Four stations seen by two sensors (three in one, one in the other): the registration comes
    /// back from a 5 cm, 1.5 degree start.
    #[test]
    fn recovers_the_registration_from_four_stations() {
        let target = Vector3::new(0.0, 1.0, 0.0);
        let world_from_cam = vec![
            synth::look_at(Vector3::new(-2.0, 2.3, 1.8), target, true),
            synth::look_at(Vector3::new(2.0, 2.3, 1.8), target, false),
        ];
        let cams = vec![synth::camera_model(); 2];
        let x_true = Isometry3::from_parts(
            Translation3::new(0.1, -0.6, -2.3),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.004, 1.2, -0.003)),
        );
        // stations high on the walls, facing the room: placed in the world, expressed in Lighthouse
        let world_stations = [
            Vector3::new(-1.5, 2.4, -2.5),
            Vector3::new(1.8, 2.5, -2.6),
            Vector3::new(0.0, 2.5, -3.4),
            Vector3::new(0.2, 2.4, 3.2),
        ];
        let stations: Vec<Isometry3<f64>> = world_stations
            .iter()
            .map(|p| {
                x_true * Isometry3::from_parts(Translation3::from(*p), UnitQuaternion::identity())
            })
            .collect();
        let mut spots = Vec::new();
        for (c, w) in world_from_cam.iter().enumerate() {
            for (i, st) in stations.iter().enumerate() {
                let pw = x_true.inverse() * station_point(st, 0.0);
                if let Some(uv) = cams[c].project(&(w.inverse() * pw).coords) {
                    if uv.x >= 0.0
                        && uv.y >= 0.0
                        && uv.x < 1280.0
                        && uv.y < 960.0
                        && (w.inverse() * pw).z > 0.0
                    {
                        spots.push(Spot {
                            camera: c,
                            station: i,
                            xy: uv,
                            frames: 100,
                        });
                    }
                }
            }
        }
        assert!(spots.len() >= 4, "{} spots", spots.len());
        let start = exp6(&[0.01, -0.02, 0.015, 0.03, -0.04, 0.02]) * x_true;
        let fit = solve(
            &cams,
            &world_from_cam,
            &stations,
            &spots,
            &start,
            0.0,
            false,
        )
        .unwrap();
        let d = fit.lighthouse_from_world.inverse() * x_true;
        assert!(
            d.translation.vector.norm() < 1e-4 && d.rotation.angle() < 1e-5,
            "{d:?}"
        );
    }
}
