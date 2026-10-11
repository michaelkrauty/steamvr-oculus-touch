// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Rift CV1 sensor camera model: OpenCV fisheye (Kannala-Brandt, k1..k4, zero skew) with the
//! factory intrinsics the engine reads from each sensor's flash.

use nalgebra::{Vector2, Vector3};

use crate::ffi;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraModel {
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub k: [f64; 4],
}

impl CameraModel {
    pub fn from_sensor(s: &ffi::ohmd_cv1touch_sensor) -> CameraModel {
        CameraModel {
            width: s.width.max(0) as u32,
            height: s.height.max(0) as u32,
            fx: s.fx as f64,
            fy: s.fy as f64,
            cx: s.cx as f64,
            cy: s.cy as f64,
            k: s.k.map(|v| v as f64),
        }
    }

    /// Pixel position of a point in camera coordinates (cv::fisheye::projectPoints). None at or
    /// behind the camera plane.
    pub fn project(&self, p: &Vector3<f64>) -> Option<Vector2<f64>> {
        if p.z <= 1e-6 {
            return None;
        }
        let (a, b) = (p.x / p.z, p.y / p.z);
        let r = (a * a + b * b).sqrt();
        let theta = r.atan();
        let t2 = theta * theta;
        let [k1, k2, k3, k4] = self.k;
        let theta_d = theta * (1.0 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))));
        let scale = if r > 1e-12 { theta_d / r } else { 1.0 };
        Some(Vector2::new(
            self.fx * a * scale + self.cx,
            self.fy * b * scale + self.cy,
        ))
    }
}

impl CameraModel {
    /// The unit direction (camera frame) a pixel sees: the projection inverted by Newton steps.
    pub fn unproject(&self, uv: &Vector2<f64>) -> Option<Vector3<f64>> {
        let mut d = Vector3::new((uv.x - self.cx) / self.fx, (uv.y - self.cy) / self.fy, 1.0);
        for _ in 0..30 {
            let p = self.project(&d)?;
            let e = uv - p;
            if e.norm() < 1e-9 {
                break;
            }
            // the projection is close to fx, fy times the normalised coordinates locally
            let h = 1e-6;
            let px = (self.project(&(d + Vector3::new(h, 0.0, 0.0)))? - p) / h;
            let py = (self.project(&(d + Vector3::new(0.0, h, 0.0)))? - p) / h;
            let j = nalgebra::Matrix2::from_columns(&[px, py]);
            let step = j.try_inverse()? * e;
            d.x += step.x;
            d.y += step.y;
        }
        Some(d.normalize())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ffi;

    fn model() -> CameraModel {
        CameraModel {
            width: 1280,
            height: 960,
            fx: 715.0,
            fy: 715.0,
            cx: 640.0,
            cy: 480.0,
            k: [0.1, 0.01, 0.0, 0.0],
        }
    }

    #[test]
    fn optical_axis_maps_to_principal_point() {
        let uv = model().project(&Vector3::new(0.0, 0.0, 2.0)).unwrap();
        assert!((uv - Vector2::new(640.0, 480.0)).norm() < 1e-9);
    }

    #[test]
    fn fisheye_distortion_follows_opencv() {
        // theta = 0.5 rad off axis: theta_d = 0.5 * (1 + 0.1 * 0.25 + 0.01 * 0.0625) = 0.5128125
        let along_x = model()
            .project(&Vector3::new(0.5f64.tan() * 3.0, 0.0, 3.0))
            .unwrap();
        assert!(
            (along_x.x - (640.0 + 715.0 * 0.5128125)).abs() < 1e-9
                && (along_x.y - 480.0).abs() < 1e-9
        );
        let up_in_image = model()
            .project(&Vector3::new(0.0, -(0.5f64.tan()), 1.0))
            .unwrap();
        assert!((up_in_image.y - (480.0 - 715.0 * 0.5128125)).abs() < 1e-9);
    }

    #[test]
    fn unproject_inverts_project() {
        let m = model();
        for p in [
            Vector3::new(0.3, -0.2, 2.0),
            Vector3::new(-1.5, 1.0, 1.2),
            Vector3::new(0.0, 0.0, 1.0),
        ] {
            let d = m.unproject(&m.project(&p).unwrap()).unwrap();
            assert!(
                (d - p.normalize()).norm() < 1e-9,
                "{d:?} vs {:?}",
                p.normalize()
            );
        }
    }

    #[test]
    fn points_behind_the_camera_do_not_project() {
        assert!(model().project(&Vector3::new(0.1, 0.0, -1.0)).is_none());
        assert!(model().project(&Vector3::new(0.1, 0.0, 0.0)).is_none());
    }

    #[test]
    fn from_sensor_copies_the_factory_intrinsics() {
        let s = ffi::ohmd_cv1touch_sensor {
            width: 1280,
            height: 960,
            fx: 716.5,
            fy: 716.5,
            cx: 641.25,
            cy: 479.5,
            k: [0.1, -0.2, 0.3, -0.4],
            ..Default::default()
        };
        let m = CameraModel::from_sensor(&s);
        assert_eq!((m.width, m.fx, m.cx, m.cy), (1280, 716.5, 641.25, 479.5));
        assert!((m.k[3] + 0.4).abs() < 1e-7);
    }
}
