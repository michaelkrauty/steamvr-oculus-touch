// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

// Test-only, field-complete interchange between the actual Pipeline and daemon unit harnesses.
use super::{Output, Stamped};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Publication {
    stamps: [u64; 4],
    pos: [f64; 3],
    quat: [f64; 4],
    vectors: [[f64; 3]; 6],
    tracking: bool,
}

impl From<Stamped> for Publication {
    fn from(s: Stamped) -> Self {
        let o = s.output;
        let q = o.world_from_model.rotation;
        Self {
            stamps: [s.host_ns, s.last_fix_host_ns, o.t, o.last_fix],
            pos: o.world_from_model.translation.vector.into(),
            quat: [q.i, q.j, q.k, q.w],
            vectors: [
                o.vel,
                o.ang_vel_world,
                o.gyro_bias,
                o.accel_bias,
                s.gyro,
                s.gyro_bias,
            ]
            .map(Into::into),
            tracking: o.tracking,
        }
    }
}

impl From<Publication> for Stamped {
    fn from(p: Publication) -> Self {
        let [vel, ang_vel_world, gyro_bias, accel_bias, gyro, stamp_bias] =
            p.vectors.map(Vector3::from);
        Self {
            host_ns: p.stamps[0],
            last_fix_host_ns: p.stamps[1],
            gyro,
            gyro_bias: stamp_bias,
            output: Output {
                t: p.stamps[2],
                last_fix: p.stamps[3],
                world_from_model: Isometry3::from_parts(
                    Translation3::from(Vector3::from(p.pos)),
                    UnitQuaternion::new_unchecked(Quaternion::new(
                        p.quat[3], p.quat[0], p.quat[1], p.quat[2],
                    )),
                ),
                vel,
                ang_vel_world,
                gyro_bias,
                accel_bias,
                tracking: p.tracking,
            },
        }
    }
}
