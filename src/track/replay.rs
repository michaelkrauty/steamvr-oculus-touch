// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Offline replay of a cv1touchd recording through the tracking pipeline, in the order the daemon
//! wrote the records: a frame's record is written after its image processing, so frames reach the
//! trackers as late as they reached the engine live.

use std::collections::BTreeMap;
use std::path::Path;

use nalgebra::{Isometry3, Vector3};

use super::pipeline::{device_hand, FrameEvent, Pipeline, Probe};
use super::tracker::Stats;
use crate::calib::camera::CameraModel;
use crate::calib::capture::Led;
use crate::calib::file::CalibrationFile;
use crate::calib::{iso_from_xyzw, vec3};
use crate::engine::cstr_field;
use crate::eval::Published;
use crate::recorder::{self, Reader};

pub use super::pipeline::Options;

#[derive(Debug, Clone, Default)]
pub struct Replay {
    /// Per hand: every output while tracking, stamped with its IMU sample's host time.
    pub hands: [Vec<Published>; 2],
    /// Per hand, alongside `hands`: the filter's (gyro, accel) biases.
    pub biases: [Vec<(Vector3<f64>, Vector3<f64>)>; 2],
    /// With `Options::delayed_ns`: per hand, the pose that long before each output, with the
    /// frames that arrived since applied (stamped at that earlier time).
    pub delayed: [Vec<Published>; 2],
    pub stats: [Stats; 2],
    /// With `Options::trace`: each tracker's miss diagnostics (see `Tracker::misses`).
    pub misses: [Vec<(usize, usize, usize, usize)>; 2],
    pub fits: [Vec<(usize, usize)>; 2],
    pub frames_skipped: u64,
    /// Pending frames evicted by each registered camera's bounded backlog.
    pub frames_dropped: u64,
    pub events: Vec<FrameEvent>,
    pub probes: Vec<Probe>,
    /// Each sensor's agreement with the others at the end (see `Pipeline::camera_agreement`).
    pub agreement: Vec<(String, f64, f64, u64)>,
}

/// `imu_position` finds a controller's IMU position from its LED model (see `factory`).
pub fn replay(
    path: &Path,
    cal: &CalibrationFile,
    imu_position: &dyn Fn(&[Led]) -> Option<Vector3<f64>>,
    o: &Options,
) -> Result<Replay, String> {
    let poses: BTreeMap<String, Isometry3<f64>> = cal
        .sensors
        .iter()
        .map(|s| (s.serial.clone(), iso_from_xyzw(s.orient_xyzw, s.pos)))
        .collect();
    let mut p = Pipeline::new(o.clone());
    let mut out = Replay::default();
    let o_delayed = o.delayed_ns;
    for rec in Reader::open(path)? {
        match rec.rtype {
            recorder::SENSOR_INFO => {
                let Some(s) = recorder::decode_sensor_info(&rec.payload) else {
                    continue;
                };
                let serial = cstr_field(&s.serial);
                if let Some(w) = poses.get(&serial) {
                    p.sensor(&serial, CameraModel::from_sensor(&s), *w);
                } else if o.probe_sensor.as_deref() == Some(serial.as_str()) {
                    // a sensor being placed has no pose yet; probing needs only its frames
                    p.sensor(&serial, CameraModel::from_sensor(&s), Isometry3::identity());
                }
            }
            recorder::LED_MODEL => {
                let Some((device, model)) = recorder::decode_led_model(&rec.payload) else {
                    continue;
                };
                let Some(hand) = device_hand(device) else {
                    continue;
                };
                if p.has_body(hand) {
                    continue;
                }
                let leds: Vec<Led> = model
                    .iter()
                    .map(|l| Led {
                        pos: vec3(l.pos),
                        dir: vec3(l.dir),
                    })
                    .collect();
                let imu_pos = imu_position(&leds).ok_or_else(|| format!("no factory IMU position for device {device} (no cached Touch configuration matches its LED model)"))?;
                p.body(device, leds, imu_pos);
            }
            recorder::RAW_REPORT => {
                if let Some((0, d)) = recorder::decode_raw(&rec.payload) {
                    p.hmd_report(rec.host_ns, d);
                }
            }
            recorder::CONTROLLER_STATE => {
                let Some((hand, st)) = recorder::decode_state(&rec.payload) else {
                    continue;
                };
                if hand > 1 {
                    continue;
                }
                if let Some(s) = p.imu(
                    hand,
                    st.device_time_ns,
                    st.received_host_ns,
                    vec3(st.imu_gyro),
                    vec3(st.imu_accel),
                ) {
                    let o = s.output;
                    out.hands[hand].push(Published {
                        t_ns: s.host_ns,
                        pose: o.world_from_model,
                        vel: o.vel,
                        ang_vel_world: o.ang_vel_world,
                    });
                    out.biases[hand].push((o.gyro_bias, o.accel_bias));
                    if let (Some(d), Some(tr)) = (o_delayed, p.tracker(hand)) {
                        if let Some(f) = tr.filter_at(o.t.saturating_sub(d)) {
                            out.delayed[hand].push(Published {
                                t_ns: s.host_ns - d,
                                pose: tr.body.world_from_model(&f.x),
                                vel: f.x.v,
                                ang_vel_world: Vector3::zeros(),
                            });
                        }
                    }
                }
            }
            recorder::FRAME_OBS => {
                if let Some((info, blobs)) = recorder::decode_frame_obs(&rec.payload) {
                    p.frame(&info, &blobs);
                }
            }
            _ => {}
        }
    }
    for hand in 0..2 {
        out.stats[hand] = p.stats(hand);
        out.misses[hand] = p.tracker(hand).map_or_else(Vec::new, |t| t.misses.clone());
        out.fits[hand] = p.tracker(hand).map_or_else(Vec::new, |t| t.fits.clone());
    }
    out.frames_skipped = p.frames_skipped;
    out.frames_dropped = p.frames_dropped;
    out.agreement = p.camera_agreement();
    out.events = std::mem::take(&mut p.events);
    out.probes = std::mem::take(&mut p.probes);
    Ok(out)
}
