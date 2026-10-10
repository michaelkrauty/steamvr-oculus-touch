// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! Phase 3a calibration: camera-only wand bundle adjustment with gravity (spec "Calibration
//! (jig-free)", sources 2 and 3). Poses are nalgebra isometries named `a_from_b`, mapping
//! b-coordinates into a-coordinates. Camera frames use OpenCV axes (x right, y down, z forward).

pub mod ba;
pub mod camera;
pub mod capture;
pub mod consistency;
pub mod file;
pub mod gravity;
pub mod gyro;
pub mod init;
pub mod motion;
pub mod rig;
pub mod solve;
pub mod stations;
#[cfg(test)]
pub mod synth;

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};

/// OpenHMD device ids of the Touch controllers (0 is the headset).
pub const DEVICE_RIGHT: i32 = 1;
pub const DEVICE_LEFT: i32 = 2;

/// Pose from the C structs' `x y z w` quaternion and position.
pub fn iso_from_xyzw(q: [f32; 4], p: [f32; 3]) -> Isometry3<f64> {
    let q = UnitQuaternion::from_quaternion(Quaternion::new(
        q[3] as f64,
        q[0] as f64,
        q[1] as f64,
        q[2] as f64,
    ));
    Isometry3::from_parts(Translation3::new(p[0] as f64, p[1] as f64, p[2] as f64), q)
}

pub fn iso_to_xyzw(t: &Isometry3<f64>) -> ([f32; 4], [f32; 3]) {
    let q = t.rotation.quaternion();
    let p = t.translation.vector;
    (
        [q.i as f32, q.j as f32, q.k as f32, q.w as f32],
        [p.x as f32, p.y as f32, p.z as f32],
    )
}

pub fn vec3(v: [f32; 3]) -> Vector3<f64> {
    Vector3::new(v[0] as f64, v[1] as f64, v[2] as f64)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn xyzw_order_matches_the_c_structs() {
        let s = std::f32::consts::FRAC_1_SQRT_2;
        let t = iso_from_xyzw([0.0, 0.0, s, s], [1.0, 2.0, 3.0]); // 90 degrees about +Z
        let p = t * Point3::new(1.0, 0.0, 0.0);
        assert!((p.coords - Vector3::new(1.0, 3.0, 3.0)).norm() < 1e-6);
        let (q, pos) = iso_to_xyzw(&t);
        assert!((q[2] - s).abs() < 1e-6 && (q[3] - s).abs() < 1e-6 && q[0].abs() < 1e-6);
        assert_eq!(pos, [1.0, 2.0, 3.0]);
    }
}
