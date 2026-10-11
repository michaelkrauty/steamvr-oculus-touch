// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Cross-sensor consistency of a recording under a calibration. In every exposure where two
//! calibrated sensors each found a controller's camera-relative pose, each implies a world pose;
//! their disagreement measures calibration error plus single-frame pose noise, independently of
//! the solve that produced the calibration.

use std::fmt;

use nalgebra::Isometry3;

use super::capture::{Capture, WandSample};
use super::file::CalibrationFile;
use super::iso_from_xyzw;

#[derive(Debug, Clone, PartialEq)]
pub struct Consistency {
    pub pairs: usize,
    pub pos_median_mm: f64,
    pub pos_p95_mm: f64,
    pub rot_median_deg: f64,
    pub rot_p95_deg: f64,
}

impl fmt::Display for Consistency {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{} sensor pairs: position median {:.2} mm p95 {:.2} mm, rotation median {:.3} deg p95 {:.3} deg",
            self.pairs, self.pos_median_mm, self.pos_p95_mm, self.rot_median_deg, self.rot_p95_deg
        )
    }
}

fn percentile(v: &mut [f64], p: f64) -> f64 {
    v.sort_by(f64::total_cmp);
    v[((v.len() - 1) as f64 * p).round() as usize]
}

pub fn measure(
    samples: &[WandSample],
    world_from_cam: &[Option<Isometry3<f64>>],
) -> Option<Consistency> {
    let (mut dp, mut da) = (Vec::new(), Vec::new());
    for s in samples {
        let poses: Vec<Isometry3<f64>> = s
            .views
            .iter()
            .filter(|v| v.strong)
            .filter_map(|v| {
                Some(world_from_cam.get(v.camera).copied().flatten()? * v.cam_from_obj?)
            })
            .collect();
        for (i, a) in poses.iter().enumerate() {
            for b in &poses[i + 1..] {
                dp.push((a.translation.vector - b.translation.vector).norm() * 1e3);
                da.push(a.rotation.angle_to(&b.rotation).to_degrees());
            }
        }
    }
    if dp.is_empty() {
        return None;
    }
    Some(Consistency {
        pairs: dp.len(),
        pos_median_mm: percentile(&mut dp, 0.5),
        pos_p95_mm: percentile(&mut dp, 0.95),
        rot_median_deg: percentile(&mut da, 0.5),
        rot_p95_deg: percentile(&mut da, 0.95),
    })
}

pub fn from_capture(c: &Capture, cal: &CalibrationFile) -> Result<Consistency, String> {
    let poses: Vec<Option<Isometry3<f64>>> = c
        .serials
        .iter()
        .map(|s| {
            cal.sensors
                .iter()
                .find(|e| &e.serial == s)
                .map(|e| iso_from_xyzw(e.orient_xyzw, e.pos))
        })
        .collect();
    if poses.iter().flatten().count() < 2 {
        return Err("fewer than two of the recorded sensors are in the calibration".into());
    }
    measure(&c.samples, &poses)
        .ok_or_else(|| "no exposure in which two calibrated sensors both found a controller".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::synth::{self, SceneParams};
    use nalgebra::{Translation3, Vector3};

    fn exact() -> synth::Scene {
        synth::scene(&SceneParams {
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            ..Default::default()
        })
    }

    #[test]
    fn exact_poses_agree() {
        let sc = exact();
        let poses: Vec<_> = sc.world_from_cam.iter().copied().map(Some).collect();
        let c = measure(&sc.samples, &poses).unwrap();
        assert!(
            c.pairs > 100 && c.pos_median_mm < 1e-6 && c.rot_median_deg < 1e-6,
            "{c}"
        );
    }

    #[test]
    fn a_shifted_sensor_shows_its_offset() {
        let mut sc = exact();
        for s in &mut sc.samples {
            s.views.retain(|v| v.camera != 2);
        }
        let mut poses: Vec<_> = sc.world_from_cam.iter().copied().map(Some).collect();
        poses[1] = Some(Translation3::from(Vector3::new(0.005, 0.0, 0.0)) * sc.world_from_cam[1]);
        let c = measure(&sc.samples, &poses).unwrap();
        assert!(
            (c.pos_median_mm - 5.0).abs() < 0.01 && (c.pos_p95_mm - 5.0).abs() < 0.01,
            "{c}"
        );
    }
}
