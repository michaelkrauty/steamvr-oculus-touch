// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Offline replay of a cv1touchd recording through the tracking pipeline, in the order the daemon
//! wrote the records: a frame's record is written after its image processing, so frames reach the
//! trackers as late as they reached the engine live.

use std::path::Path;

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};

use super::factory::Imu;
use super::pipeline::{device_hand, FrameEvent, Pipeline, Probe};
use super::tracker::Stats;
use super::vision::Camera;
use crate::calib::camera::CameraModel;
use crate::calib::capture::Led;
use crate::calib::file::CalibrationFile;
use crate::calib::refit::RefitSample;
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
    /// Why replay stopped before the end of the recording, if applicable.
    pub stopped: Option<String>,
    pub frames_skipped: u64,
    /// Pending frames evicted by each registered camera's bounded backlog.
    pub frames_dropped: u64,
    pub events: Vec<FrameEvent>,
    pub probes: Vec<Probe>,
    /// Each sensor's agreement with the others at the end (see `Pipeline::camera_agreement`).
    pub agreement: Vec<(String, super::tracker::CameraAgreement)>,
    /// With `Options::collect_refit`: thinned synchronised multi-camera tracker observations.
    pub refit_samples: Vec<RefitSample>,
    /// Registered sensors at the end of replay: `(camera_id, serial, Camera)` in serial order.
    pub sensors: Vec<(usize, String, Camera)>,
}

/// `imu` finds a controller's factory IMU data from its LED model (see `factory`).
pub fn replay(
    path: &Path,
    cal: &CalibrationFile,
    imu: &dyn Fn(&[Led]) -> Option<Imu>,
    o: &Options,
) -> Result<Replay, String> {
    let mut p = Pipeline::new(o.clone());
    let mut out = Replay::default();
    let mut pending_poses: std::collections::BTreeMap<String, Isometry3<f64>> = Default::default();
    let o_delayed = o.delayed_ns;
    let reader = Reader::open(path)?;
    let start_host_ns = reader.start_host_ns;
    for rec in reader {
        match rec.rtype {
            recorder::SENSOR_ADDED => {
                let serial = String::from_utf8_lossy(&rec.payload);
                let elapsed = rec.host_ns.saturating_sub(start_host_ns) as f64 * 1e-9;
                out.stopped = Some(format!("replay stopped {elapsed:.1} s into the recording when sensor {serial} was plugged in: the recording cannot be reproduced exactly past that point, so the rest was left out"));
                break;
            }
            recorder::SENSOR_INFO => {
                let Some(s) = recorder::decode_sensor_info(&rec.payload) else {
                    continue;
                };
                let serial = cstr_field(&s.serial);
                let id = crate::status::sensor_identity(&serial);
                // as live: the calibration's serial may be spelled in another letter case
                if let Some(entry) = cal.sensor(&serial) {
                    let w = if o.apply_recorded_poses {
                        pending_poses
                            .remove(&id)
                            .or_else(|| {
                                p.registered_sensors()
                                    .into_iter()
                                    .find(|(_, existing, _)| {
                                        crate::status::same_sensor(existing, &serial)
                                    })
                                    .map(|(_, _, cam)| cam.world_from_cam)
                            })
                            .unwrap_or_else(|| iso_from_xyzw(entry.orient_xyzw, entry.pos))
                    } else {
                        iso_from_xyzw(entry.orient_xyzw, entry.pos)
                    };
                    p.sensor(&serial, CameraModel::from_sensor(&s), w);
                } else if o
                    .probe_sensors
                    .iter()
                    .any(|s| crate::status::same_sensor(s, &serial))
                {
                    // a sensor being placed has no pose yet; probing needs only its frames
                    p.sensor(&serial, CameraModel::from_sensor(&s), Isometry3::identity());
                }
            }
            recorder::SENSOR_POSES if o.apply_recorded_poses => {
                let Some(poses) = recorder::decode_sensor_poses(&rec.payload) else {
                    continue;
                };
                let registered = p.registered_sensors();
                let mut applied_live = false;
                for entry in poses {
                    let (serial, orient, pos) = (entry.serial, entry.orient_xyzw, entry.pos);
                    let w = Isometry3::from_parts(
                        Translation3::new(pos[0], pos[1], pos[2]),
                        UnitQuaternion::new_normalize(Quaternion::new(
                            orient[3], orient[0], orient[1], orient[2],
                        )),
                    );
                    if let Some((_, existing, cam)) = registered
                        .iter()
                        .find(|(_, s, _)| crate::status::same_sensor(s, &serial))
                    {
                        p.sensor(existing, cam.model, w);
                        p.reset_agreement(existing);
                        applied_live = true;
                    } else {
                        pending_poses.insert(crate::status::sensor_identity(&serial), w);
                    }
                }
                if applied_live {
                    p.reset_refit_samples();
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
                let imu = imu(&leds).ok_or_else(|| format!("no factory IMU position for device {device} (no cached Touch configuration matches its LED model)"))?;
                p.body(device, leds, imu.position);
                p.gyro_range(device, imu.gyro);
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
            recorder::INSPECTION_IDENTITY => {
                if let Some(event) = recorder::decode_inspection_identity(&rec.payload) {
                    p.inspection_identity(&event);
                }
            }
            // As live, where each engine gets a new pipeline and the restored pose's
            // acknowledgement retires the sensor's tombstones: the frames before these
            // records were the engine's last for them.
            recorder::ENGINE_SESSION => {
                out.probes.append(&mut p.probes);
                p.new_engine_session();
            }
            recorder::IDENTITIES_RETIRED => {
                if let Ok(serial) = std::str::from_utf8(&rec.payload) {
                    p.retire_identities(serial);
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
    out.probes.append(&mut p.probes);
    out.refit_samples = p.take_refit_samples();
    out.sensors = p.registered_sensors();
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::file::{CalibrationFile, SensorEntry, Stats as CalStats, FORMAT, VERSION};
    use crate::engine::Sensor as SensorInfo;
    use crate::recorder::Recorder;

    fn sensor_info(serial: &str) -> SensorInfo {
        let mut s = SensorInfo::default();
        for (dst, b) in s.serial.iter_mut().zip(serial.bytes()) {
            *dst = b as _;
        }
        s.width = 1280;
        s.height = 960;
        s.fx = 715.0;
        s.fy = 715.0;
        s.cx = 640.0;
        s.cy = 480.0;
        s
    }

    #[test]
    fn replay_applies_recorded_sensor_poses_and_respects_opt_out() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("poses.rec");
        let rec = Recorder::new(1 << 20);
        rec.start(&path, 1_000).unwrap();
        // Pose recorded before SENSOR_INFO (e.g. recording started after an earlier session refit).
        let pre = recorder::encode_sensor_poses(&[("A", [0.0, 0.0, 0.0, 1.0], [1.5, 2.5, 3.5])]);
        rec.write_boundary(recorder::SENSOR_POSES, 1_100, &[&pre]);
        let sa = sensor_info("A");
        let sb = sensor_info("B");
        rec.write(recorder::SENSOR_INFO, 1_200, &[recorder::as_bytes(&sa)]);
        rec.write(recorder::SENSOR_INFO, 1_200, &[recorder::as_bytes(&sb)]);
        // Mid-recording session refit updating both sensors (including case-insensitive serial).
        let mid = recorder::encode_sensor_poses(&[
            ("a", [0.0, 0.0, 0.0, 1.0], [1.6, 2.6, 3.6]),
            ("B", [0.0, 0.0, 0.0, 1.0], [-0.5, 1.0, 2.0]),
        ]);
        rec.write_boundary(recorder::SENSOR_POSES, 2_000, &[&mid]);
        rec.stop();

        let cal = CalibrationFile {
            format: FORMAT.into(),
            version: VERSION,
            created_unix_s: 0,
            world: "test".into(),
            source: "test".into(),
            sensors: vec![
                SensorEntry {
                    serial: "A".into(),
                    orient_xyzw: [0.0, 0.0, 0.0, 1.0],
                    pos: [0.0, 0.0, 0.0],
                    rms_px: 0.1,
                    observations: 100,
                },
                SensorEntry {
                    serial: "B".into(),
                    orient_xyzw: [0.0, 0.0, 0.0, 1.0],
                    pos: [0.0, 0.0, 0.0],
                    rms_px: 0.1,
                    observations: 100,
                },
            ],
            stats: CalStats {
                reference: "A".into(),
                gravity_mps2: 9.81,
                gravity_spread_deg: None,
                motion_segments: 1,
                samples: 10,
                rejected_views: 0,
            },
        };

        let applied = replay(&path, &cal, &|_| None, &Options::default()).unwrap();
        let pos_a = applied
            .sensors
            .iter()
            .find(|(_, s, _)| s == "A")
            .unwrap()
            .2
            .world_from_cam
            .translation
            .vector;
        let pos_b = applied
            .sensors
            .iter()
            .find(|(_, s, _)| s == "B")
            .unwrap()
            .2
            .world_from_cam
            .translation
            .vector;
        assert!((pos_a - Vector3::new(1.6, 2.6, 3.6)).norm() < 1e-9);
        assert!((pos_b - Vector3::new(-0.5, 1.0, 2.0)).norm() < 1e-9);

        let fixed = replay(
            &path,
            &cal,
            &|_| None,
            &Options {
                apply_recorded_poses: false,
                ..Default::default()
            },
        )
        .unwrap();
        for (_, _, cam) in &fixed.sensors {
            assert!(cam.world_from_cam.translation.vector.norm() < 1e-9);
        }
    }

    #[test]
    fn replay_preserves_recorded_poses_across_repeated_sensor_info_after_reopen() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("reopen_poses.rec");
        let rec = Recorder::new(1 << 20);
        rec.start(&path, 1_000).unwrap();
        // Session 1 starts with pre-bump poses in SENSOR_POSES, then emits SENSOR_INFO.
        let pre = recorder::encode_sensor_poses(&[
            ("A", [0.0, 0.0, 0.0, 1.0], [1.5, 2.5, 3.5]),
            ("B", [0.0, 0.0, 0.0, 1.0], [-1.0, 0.5, 2.0]),
        ]);
        rec.write_boundary(recorder::SENSOR_POSES, 1_100, &[&pre]);
        let sa = sensor_info("A");
        let sb = sensor_info("B");
        rec.write(recorder::SENSOR_INFO, 1_200, &[recorder::as_bytes(&sa)]);
        rec.write(recorder::SENSOR_INFO, 1_200, &[recorder::as_bytes(&sb)]);
        // Engine reopen: ENGINE_SESSION, then session-2 SENSOR_POSES snapshot (with updated pose
        // for A, and same pre-bump pose for B), followed by repeated SENSOR_INFO.
        rec.write_boundary(recorder::ENGINE_SESSION, 2_000, &[]);
        let reopen = recorder::encode_sensor_poses(&[
            ("A", [0.0, 0.0, 0.0, 1.0], [1.7, 2.7, 3.7]),
            ("B", [0.0, 0.0, 0.0, 1.0], [-1.0, 0.5, 2.0]),
        ]);
        rec.write_boundary(recorder::SENSOR_POSES, 2_100, &[&reopen]);
        rec.write(recorder::SENSOR_INFO, 2_200, &[recorder::as_bytes(&sa)]);
        rec.write(recorder::SENSOR_INFO, 2_200, &[recorder::as_bytes(&sb)]);
        // Another reopen with repeated SENSOR_INFO and no new SENSOR_POSES still keeps poses.
        rec.write_boundary(recorder::ENGINE_SESSION, 3_000, &[]);
        rec.write(recorder::SENSOR_INFO, 3_100, &[recorder::as_bytes(&sa)]);
        rec.write(recorder::SENSOR_INFO, 3_100, &[recorder::as_bytes(&sb)]);
        rec.stop();

        // Calibration file on disk reflects a later post-bump pose (e.g. [9.0, 9.0, 9.0]).
        let cal = CalibrationFile {
            format: FORMAT.into(),
            version: VERSION,
            created_unix_s: 0,
            world: "test".into(),
            source: "test".into(),
            sensors: vec![
                SensorEntry {
                    serial: "A".into(),
                    orient_xyzw: [0.0, 0.0, 0.0, 1.0],
                    pos: [9.0, 9.0, 9.0],
                    rms_px: 0.1,
                    observations: 100,
                },
                SensorEntry {
                    serial: "B".into(),
                    orient_xyzw: [0.0, 0.0, 0.0, 1.0],
                    pos: [8.0, 8.0, 8.0],
                    rms_px: 0.1,
                    observations: 100,
                },
            ],
            stats: CalStats {
                reference: "A".into(),
                gravity_mps2: 9.81,
                gravity_spread_deg: None,
                motion_segments: 1,
                samples: 10,
                rejected_views: 0,
            },
        };

        let applied = replay(&path, &cal, &|_| None, &Options::default()).unwrap();
        let pos_a = applied
            .sensors
            .iter()
            .find(|(_, s, _)| s == "A")
            .unwrap()
            .2
            .world_from_cam
            .translation
            .vector;
        let pos_b = applied
            .sensors
            .iter()
            .find(|(_, s, _)| s == "B")
            .unwrap()
            .2
            .world_from_cam
            .translation
            .vector;
        assert!((pos_a - Vector3::new(1.7, 2.7, 3.7)).norm() < 1e-9);
        assert!((pos_b - Vector3::new(-1.0, 0.5, 2.0)).norm() < 1e-9);

        let fixed = replay(
            &path,
            &cal,
            &|_| None,
            &Options {
                apply_recorded_poses: false,
                ..Default::default()
            },
        )
        .unwrap();
        let fixed_a = fixed
            .sensors
            .iter()
            .find(|(_, s, _)| s == "A")
            .unwrap()
            .2
            .world_from_cam
            .translation
            .vector;
        assert!((fixed_a - Vector3::new(9.0, 9.0, 9.0)).norm() < 1e-9);
    }
}
