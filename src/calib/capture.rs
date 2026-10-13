// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Calibration input from a cv1touch recording (format v2): sensor intrinsics, controller LED
//! models, per-exposure LED sightings grouped across the synchronised sensors, and each
//! controller's IMU stream.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;

use nalgebra::{Isometry3, Vector2, Vector3};

use super::camera::CameraModel;
use super::{iso_from_xyzw, vec3, DEVICE_LEFT, DEVICE_RIGHT};
use crate::engine::cstr_field;
use crate::recorder::{self, Blob, FrameInfo, Reader, Record};
use crate::shm::{HAND_LEFT, HAND_RIGHT};

/// Frames from different sensors belong to one exposure when their exposure counters match and
/// their start times are this close (the counter is 16-bit and wraps every ~22 minutes).
pub const SAME_EXPOSURE_NS: u64 = 15_000_000;
/// RIFT_POSE_MATCH_GOOD and RIFT_POSE_MATCH_STRONG.
pub const MATCH_GOOD: u32 = 0x1;
pub const MATCH_STRONG: u32 = 0x2;
/// Fewest labelled LEDs for a sighting to be used.
pub const MIN_VIEW_OBS: usize = 4;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Led {
    pub pos: Vector3<f64>,
    pub dir: Vector3<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Observation {
    pub led: usize,
    pub xy: Vector2<f64>,
}

/// One sensor's sighting of one controller in one exposure.
#[derive(Debug, Clone, PartialEq)]
pub struct View {
    pub camera: usize,
    /// The engine's camera-relative pose of the controller, when it found one this frame.
    pub cam_from_obj: Option<Isometry3<f64>>,
    pub strong: bool,
    pub obs: Vec<Observation>,
}

/// One controller at one exposure, seen by one or more sensors.
#[derive(Debug, Clone, PartialEq)]
pub struct WandSample {
    pub device_id: i32,
    pub device_time_ns: u64,
    pub host_ns: u64,
    pub views: Vec<View>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    pub device_time_ns: u64,
    pub accel: Vector3<f64>,
    pub gyro: Vector3<f64>,
}

#[derive(Debug, Clone, Default)]
pub struct Capture {
    /// Sensor serials, sorted; a sensor's camera index is its position here.
    pub serials: Vec<String>,
    pub cameras: Vec<CameraModel>,
    pub leds: BTreeMap<i32, Vec<Led>>,
    pub samples: Vec<WandSample>,
    pub imu: BTreeMap<i32, Vec<ImuSample>>,
}

pub fn hand_device_id(hand: usize) -> i32 {
    if hand == HAND_RIGHT {
        DEVICE_RIGHT
    } else {
        debug_assert_eq!(hand, HAND_LEFT);
        DEVICE_LEFT
    }
}

pub fn load(path: &Path) -> Result<Capture, String> {
    let mut reader = Reader::open(path)?;
    let capture = from_records(reader.by_ref());
    if let Some(error) = reader.error() {
        return Err(error.to_string());
    }
    capture
}

pub fn from_records(records: impl IntoIterator<Item = Record>) -> Result<Capture, String> {
    let mut sensors: BTreeMap<String, CameraModel> = BTreeMap::new();
    let mut leds = BTreeMap::new();
    let mut frames = Vec::new();
    let mut imu: BTreeMap<i32, Vec<ImuSample>> = BTreeMap::new();
    for rec in records {
        match rec.rtype {
            recorder::SENSOR_INFO => {
                if let Some(s) = recorder::decode_sensor_info(&rec.payload) {
                    sensors.insert(cstr_field(&s.serial), CameraModel::from_sensor(&s));
                }
            }
            recorder::LED_MODEL => {
                if let Some((device, model)) = recorder::decode_led_model(&rec.payload) {
                    leds.insert(
                        device,
                        model
                            .iter()
                            .map(|l| Led {
                                pos: vec3(l.pos),
                                dir: vec3(l.dir),
                            })
                            .collect(),
                    );
                }
            }
            recorder::FRAME_OBS => {
                if let Some(f) = recorder::decode_frame_obs(&rec.payload) {
                    frames.push(f);
                }
            }
            recorder::CONTROLLER_STATE => {
                if let Some((hand, s)) = recorder::decode_state(&rec.payload) {
                    let series = imu.entry(hand_device_id(hand)).or_default();
                    if s.device_time_ns != 0
                        && series
                            .last()
                            .is_none_or(|l| l.device_time_ns < s.device_time_ns)
                    {
                        series.push(ImuSample {
                            device_time_ns: s.device_time_ns,
                            accel: vec3(s.imu_accel),
                            gyro: vec3(s.imu_gyro),
                        });
                    }
                }
            }
            _ => {}
        }
    }
    if sensors.is_empty() {
        return Err(
            "recording has no sensor info (record a new capture with the current cv1touchd)".into(),
        );
    }
    if leds.is_empty() {
        return Err("recording has no controller LED models (were the controllers awake?)".into());
    }
    let serials: Vec<String> = sensors.keys().cloned().collect();
    let index: HashMap<&str, usize> = serials
        .iter()
        .enumerate()
        .map(|(i, s)| (s.as_str(), i))
        .collect();
    let samples = group_samples(&frames, &index);
    let cameras = sensors.into_values().collect();
    Ok(Capture {
        serials,
        cameras,
        leds,
        samples,
        imu,
    })
}

/// Groups frames into exposures (same counter, start times within SAME_EXPOSURE_NS) and each
/// exposure into one sample per controller.
fn group_samples(
    frames: &[(FrameInfo, Vec<Blob>)],
    index: &HashMap<&str, usize>,
) -> Vec<WandSample> {
    let mut order: Vec<usize> = (0..frames.len())
        .filter(|&i| frames[i].0.exposure_valid != 0)
        .collect();
    order.sort_by_key(|&i| frames[i].0.frame_start_host_ns);
    let mut open: HashMap<u32, (u64, usize)> = HashMap::new();
    let mut groups: Vec<Vec<usize>> = Vec::new();
    for i in order {
        let f = &frames[i].0;
        let g = match open.get(&f.exposure_count) {
            Some(&(t0, g)) if f.frame_start_host_ns - t0 <= SAME_EXPOSURE_NS => g,
            _ => {
                groups.push(Vec::new());
                open.insert(f.exposure_count, (f.frame_start_host_ns, groups.len() - 1));
                groups.len() - 1
            }
        };
        groups[g].push(i);
    }
    let mut samples = Vec::new();
    for g in groups {
        let mut by_device: BTreeMap<i32, WandSample> = BTreeMap::new();
        for i in g {
            let (f, blobs) = &frames[i];
            let Some(&camera) = index.get(cstr_field(&f.serial).as_str()) else {
                continue;
            };
            for d in &f.devices[..(f.n_devices as usize).min(f.devices.len())] {
                let obs: Vec<Observation> = blobs
                    .iter()
                    .filter(|b| b.led_id >= 0 && (b.led_id >> 8) == d.device_id)
                    .map(|b| Observation {
                        led: (b.led_id & 0xff) as usize,
                        xy: Vector2::new(b.x as f64, b.y as f64),
                    })
                    .collect();
                if obs.len() < MIN_VIEW_OBS {
                    continue;
                }
                let sample = by_device.entry(d.device_id).or_insert_with(|| WandSample {
                    device_id: d.device_id,
                    device_time_ns: d.device_time_ns,
                    host_ns: f.frame_start_host_ns,
                    views: Vec::new(),
                });
                sample.views.push(View {
                    camera,
                    cam_from_obj: (d.pose_valid != 0)
                        .then(|| iso_from_xyzw(d.cam_orient, d.cam_pos)),
                    strong: d.match_flags & MATCH_STRONG != 0,
                    obs,
                });
            }
        }
        samples.extend(by_device.into_values());
    }
    samples
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recorder::{
        as_bytes, led_model_prefix, slice_bytes, Blob, FrameInfo, LedEntry, SensorInfo,
    };
    use crate::shm::ControllerState;
    use std::ffi::c_char;

    fn cstr<const N: usize>(s: &str) -> [c_char; N] {
        let mut f = [0 as c_char; N];
        for (d, b) in f.iter_mut().zip(s.bytes()) {
            *d = b as c_char;
        }
        f
    }

    fn sensor(serial: &str) -> Record {
        let s = SensorInfo {
            serial: cstr(serial),
            width: 1280,
            height: 960,
            fx: 715.0,
            fy: 715.0,
            cx: 640.0,
            cy: 480.0,
            ..Default::default()
        };
        Record {
            rtype: recorder::SENSOR_INFO,
            host_ns: 1,
            payload: as_bytes(&s).to_vec(),
        }
    }

    fn leds(device: i32) -> Record {
        let model = [LedEntry {
            pos: [0.01, 0.0, 0.0],
            dir: [1.0, 0.0, 0.0],
        }; 6];
        Record {
            rtype: recorder::LED_MODEL,
            host_ns: 1,
            payload: [&led_model_prefix(device, 6)[..], slice_bytes(&model)].concat(),
        }
    }

    /// One sensor frame of exposure `count`: each listed device with a valid strong pose and `n`
    /// labelled blobs, plus one unlabelled blob.
    fn frame(serial: &str, host: u64, count: u32, devices: &[i32], n: i32) -> Record {
        let mut info = FrameInfo {
            serial: cstr(serial),
            frame_start_host_ns: host,
            exposure_valid: 1,
            exposure_count: count,
            n_devices: devices.len() as u32,
            ..Default::default()
        };
        let mut blobs = Vec::new();
        for (slot, &id) in info.devices.iter_mut().zip(devices) {
            slot.device_id = id;
            slot.device_time_ns = host / 2;
            slot.match_flags = 0x3;
            slot.pose_valid = 1;
            slot.cam_orient = [0.0, 0.0, 0.0, 1.0];
            slot.cam_pos = [0.0, 0.0, 2.0];
            blobs.extend((0..n).map(|i| Blob {
                x: 100.0 + i as f32,
                y: 200.0,
                width: 3,
                height: 3,
                area: 9,
                led_id: (id << 8) | i,
            }));
        }
        blobs.push(Blob {
            x: 5.0,
            y: 5.0,
            width: 3,
            height: 3,
            area: 9,
            led_id: -1,
        });
        Record {
            rtype: recorder::FRAME_OBS,
            host_ns: host,
            payload: [as_bytes(&info), slice_bytes(&blobs)].concat(),
        }
    }

    fn state(hand: usize, device_ns: u64, accel: [f32; 3]) -> Record {
        let s = ControllerState {
            device_time_ns: device_ns,
            imu_accel: accel,
            ..Default::default()
        };
        Record {
            rtype: recorder::CONTROLLER_STATE,
            host_ns: device_ns,
            payload: [&(hand as u32).to_le_bytes()[..], &[0u8; 4], as_bytes(&s)].concat(),
        }
    }

    #[test]
    fn load_rejects_malformed_records_after_a_valid_capture_prefix() {
        let records = vec![
            sensor("WMTD000000001"),
            leds(1),
            frame("WMTD000000001", 1_000_000_000, 7, &[1], 5),
        ];
        assert!(from_records(vec![
            sensor("WMTD000000001"),
            leds(1),
            frame("WMTD000000001", 1_000_000_000, 7, &[1], 5)
        ])
        .is_ok());
        let path = std::env::temp_dir().join(format!(
            "cv1touch-capture-corrupt-{}.rec",
            std::process::id()
        ));
        let mut bytes = vec![0u8; 32];
        bytes[..8].copy_from_slice(recorder::MAGIC);
        bytes[8..12].copy_from_slice(&recorder::VERSION.to_le_bytes());
        bytes[12..16].copy_from_slice(&32u32.to_le_bytes());
        for record in records {
            bytes.extend_from_slice(&record.rtype.to_le_bytes());
            bytes.extend_from_slice(&0u16.to_le_bytes());
            bytes.extend_from_slice(&(record.payload.len() as u32).to_le_bytes());
            bytes.extend_from_slice(&record.host_ns.to_le_bytes());
            bytes.extend_from_slice(&record.payload);
        }
        bytes.extend_from_slice(&recorder::RAW_REPORT.to_le_bytes());
        bytes.extend_from_slice(&0u16.to_le_bytes());
        bytes.extend_from_slice(&u32::MAX.to_le_bytes());
        bytes.extend_from_slice(&0u64.to_le_bytes());
        std::fs::write(&path, bytes).unwrap();
        let error =
            load(&path).expect_err("malformed suffix must not become a usable calibration capture");
        assert!(error.contains("payload"), "{error}");
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn groups_sensors_by_exposure_and_device() {
        let c = from_records(vec![
            sensor("WMTD000000002"),
            sensor("WMTD000000001"),
            leds(1),
            leds(2),
            frame("WMTD000000002", 1_000_000_000, 7, &[1, 2], 5),
            frame("WMTD000000001", 1_000_400_000, 7, &[1], 5),
            frame("WMTD000000001", 1_020_000_000, 8, &[1, 2], 3), // too few labelled LEDs
        ])
        .unwrap();
        assert_eq!(c.serials, ["WMTD000000001", "WMTD000000002"]);
        assert_eq!(c.samples.len(), 2);
        let right = c.samples.iter().find(|s| s.device_id == 1).unwrap();
        let mut cams: Vec<usize> = right.views.iter().map(|v| v.camera).collect();
        cams.sort();
        assert_eq!(cams, [0, 1]);
        assert!(right
            .views
            .iter()
            .all(|v| v.strong && v.obs.len() == 5 && v.cam_from_obj.unwrap().translation.z == 2.0));
        let left = c.samples.iter().find(|s| s.device_id == 2).unwrap();
        assert_eq!(
            (
                left.views.len(),
                left.views[0].camera,
                left.views[0].obs[4].led
            ),
            (1, 1, 4)
        );
    }

    /// Review Focus 5: the 16-bit counter wraps; equal counts far apart are different exposures,
    /// and a later-arriving frame of the same exposure still joins it.
    #[test]
    fn exposure_grouping_handles_wrap() {
        let c = from_records(vec![
            sensor("A"),
            sensor("B"),
            leds(1),
            frame("B", 1_000_001_000, 65535, &[1], 5),
            frame("A", 1_000_000_000, 65535, &[1], 5),
            frame("A", 1_020_000_000, 0, &[1], 5),
            frame("A", 1_400_000_000_000, 65535, &[1], 5),
        ])
        .unwrap();
        assert_eq!(
            c.samples.iter().map(|s| s.views.len()).collect::<Vec<_>>(),
            [2, 1, 1]
        );
    }

    #[test]
    fn imu_is_deduplicated_and_keyed_by_device() {
        let c = from_records(vec![
            sensor("A"),
            leds(1),
            state(HAND_RIGHT, 10, [0.0, 9.8, 0.0]),
            state(HAND_RIGHT, 10, [0.0, 9.8, 0.0]),
            state(HAND_RIGHT, 12, [0.0, 9.7, 0.0]),
            state(HAND_LEFT, 11, [9.8, 0.0, 0.0]),
        ])
        .unwrap();
        assert_eq!(c.imu[&DEVICE_RIGHT].len(), 2);
        assert_eq!(c.imu[&DEVICE_LEFT][0].accel.x as f32, 9.8);
    }

    #[test]
    fn a_recording_without_sensor_info_is_an_error() {
        let err = from_records(vec![leds(1)]).unwrap_err();
        assert!(err.contains("sensor info"), "{err}");
    }
}
