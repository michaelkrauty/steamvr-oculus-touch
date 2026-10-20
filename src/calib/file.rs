// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! calibration.json: each sensor's world pose (camera -> world, OpenCV camera axes) plus residuals
//! and provenance. Loaded by cv1touchd at startup and on `calibration load`.

use std::collections::HashMap;
use std::io::Read;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use super::iso_to_xyzw;
use super::solve::Solution;
use crate::engine::SensorPose;
use crate::status::{same_sensor, sensor_identity};

pub const FORMAT: &str = "cv1touch-calibration";
pub const VERSION: u32 = 1;
/// Resource bound on serialized calibration, independent of the sensor count.
const MAX_FILE_BYTES: u64 = 16 * 1024 * 1024;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SensorEntry {
    pub serial: String,
    pub orient_xyzw: [f32; 4],
    pub pos: [f32; 3],
    pub rms_px: f64,
    pub observations: usize,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Stats {
    pub reference: String,
    /// |G| from the motion (m/s^2) and the largest angle between a controller's own estimate and
    /// the joint one (None with one controller).
    pub gravity_mps2: f64,
    pub gravity_spread_deg: Option<f64>,
    pub motion_segments: usize,
    pub samples: usize,
    pub rejected_views: usize,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CalibrationFile {
    pub format: String,
    pub version: u32,
    pub created_unix_s: u64,
    pub world: String,
    pub source: String,
    pub sensors: Vec<SensorEntry>,
    pub stats: Stats,
}

impl CalibrationFile {
    pub fn from_solution(sol: &Solution, source: &str, created_unix_s: u64) -> CalibrationFile {
        CalibrationFile {
            format: FORMAT.into(),
            version: VERSION,
            created_unix_s,
            world: format!(
                "gravity-aligned, +Y up; origin at the centroid of the calibration controller positions; \
                 sensor {}'s optical axis projected horizontally along -Z",
                sol.reference
            ),
            source: source.into(),
            sensors: sol
                .sensors
                .iter()
                .map(|s| {
                    let (orient_xyzw, pos) = iso_to_xyzw(&s.world_from_cam);
                    SensorEntry { serial: s.serial.clone(), orient_xyzw, pos, rms_px: s.rms_px, observations: s.observations }
                })
                .collect(),
            stats: Stats {
                reference: sol.reference.clone(),
                gravity_mps2: sol.gravity.gravity.norm(),
                gravity_spread_deg: sol.gravity.spread_deg(),
                motion_segments: sol.gravity.segments,
                samples: sol.samples,
                rejected_views: sol.rejected_views,
            },
        }
    }

    pub fn sensor_poses(&self) -> Vec<SensorPose> {
        self.sensors
            .iter()
            .map(|s| SensorPose {
                serial: s.serial.clone(),
                orient: s.orient_xyzw,
                pos: s.pos,
            })
            .collect()
    }

    /// The entry of the sensor a device calls `serial`, in whichever ASCII letter case either is
    /// spelled.
    pub fn sensor(&self, serial: &str) -> Option<&SensorEntry> {
        self.sensors.iter().find(|s| same_sensor(&s.serial, serial))
    }

    pub fn sensor_mut(&mut self, serial: &str) -> Option<&mut SensorEntry> {
        self.sensors
            .iter_mut()
            .find(|s| same_sensor(&s.serial, serial))
    }

    /// Drops the entry of the sensor a device calls `serial`, whatever the letter case of its
    /// spelling in the file.
    pub fn remove_sensor(&mut self, serial: &str) {
        self.sensors.retain(|s| !same_sensor(&s.serial, serial));
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.format != FORMAT || self.version != VERSION {
            return Err(format!("unsupported calibration format {:?} version {} (expected {FORMAT:?} version {VERSION})", self.format, self.version));
        }
        if self.sensors.is_empty() {
            return Err("calibration lists no sensors (expected at least one)".into());
        }
        // One sensor per identity: the engine matches serials without regard to ASCII letter case
        let mut seen: HashMap<String, &str> = HashMap::new();
        for s in &self.sensors {
            if s.serial.is_empty()
                || s.serial.len() > 31
                || !s.serial.bytes().all(|b| b.is_ascii_alphanumeric())
            {
                return Err(format!("invalid sensor serial {:?}", s.serial));
            }
            if let Some(first) = seen.insert(sensor_identity(&s.serial), &s.serial) {
                return Err(if first == s.serial {
                    format!("duplicate sensor {}", s.serial)
                } else {
                    format!(
                        "duplicate sensor {}: {first} is already listed (serial letter case does not matter)",
                        s.serial
                    )
                });
            }
            let norm = s
                .orient_xyzw
                .iter()
                .map(|v| (*v as f64).powi(2))
                .sum::<f64>()
                .sqrt();
            if !s.orient_xyzw.iter().all(|v| v.is_finite()) || (norm - 1.0).abs() > 1e-3 {
                return Err(format!(
                    "sensor {}: orientation is not a unit quaternion",
                    s.serial
                ));
            }
            if !s.pos.iter().all(|v| v.is_finite() && v.abs() < 20.0) {
                return Err(format!(
                    "sensor {}: position {:?} is not plausible",
                    s.serial, s.pos
                ));
            }
        }
        Ok(())
    }

    #[cfg(test)]
    pub fn example() -> CalibrationFile {
        let s = std::f32::consts::FRAC_1_SQRT_2;
        CalibrationFile {
            format: FORMAT.into(),
            version: VERSION,
            created_unix_s: 1_791_000_000,
            world: "test".into(),
            source: "/tmp/calib.rec".into(),
            sensors: vec![
                SensorEntry {
                    serial: "WMTD000000001".into(),
                    orient_xyzw: [0.0, 0.0, 0.0, 1.0],
                    pos: [0.3, 2.2, -2.6],
                    rms_px: 0.4,
                    observations: 1000,
                },
                SensorEntry {
                    serial: "WMTD000000002".into(),
                    orient_xyzw: [0.0, 0.0, s, s],
                    pos: [-2.0, 2.3, -1.8],
                    rms_px: 0.5,
                    observations: 900,
                },
            ],
            stats: Stats {
                reference: "WMTD000000001".into(),
                gravity_mps2: 9.78,
                gravity_spread_deg: Some(0.2),
                motion_segments: 160,
                samples: 2000,
                rejected_views: 3,
            },
        }
    }
}

/// Sensors in `previous` that a new solve over `serials` would drop.
pub fn missing_from(previous: &CalibrationFile, serials: &[String]) -> Vec<String> {
    previous
        .sensors
        .iter()
        .map(|s| s.serial.clone())
        .filter(|s| !serials.iter().any(|t| same_sensor(t, s)))
        .collect()
}

/// The sensors' poses (world <- camera) of a calibration, by serial.
pub fn sensor_poses(f: &CalibrationFile) -> Vec<(String, nalgebra::Isometry3<f64>)> {
    f.sensors
        .iter()
        .map(|s| (s.serial.clone(), super::iso_from_xyzw(s.orient_xyzw, s.pos)))
        .collect()
}

/// The sensors' poses (world <- camera) a recording was made under, as the daemon recorded them.
pub fn recording_sensor_poses(
    rec: &Path,
) -> Result<Vec<(String, nalgebra::Isometry3<f64>)>, String> {
    use crate::recorder::{self, Reader};
    let mut v: Vec<(String, nalgebra::Isometry3<f64>)> = Vec::new();
    for r in Reader::open(rec)? {
        if r.rtype != recorder::SENSOR_INFO {
            continue;
        }
        let Some(s) = recorder::decode_sensor_info(&r.payload) else {
            continue;
        };
        let serial = crate::engine::cstr_field(&s.serial);
        if s.have_camera_pose && !v.iter().any(|(t, _)| *t == serial) {
            v.push((serial, super::iso_from_xyzw(s.camera_orient, s.camera_pos)));
        }
    }
    Ok(v)
}

/// Sensors a recalibration may leave this far from where a rigid change of world frame carries
/// them (m, and radians) for the change to stand for both calibrations' frames: recalibrations
/// of sensors that stayed put leave them within 7 mm and 0.2 degrees.
pub const SENSOR_TOLERANCE_M: f64 = 0.010;
pub const SENSOR_TOLERANCE_RAD: f64 = 1.0 * std::f64::consts::PI / 180.0;

/// A change of world frame between two calibrations (`world_change`).
#[derive(Debug, Clone, PartialEq)]
pub struct WorldChange {
    /// world_now <- world_then.
    pub change: nalgebra::Isometry3<f64>,
    /// The farthest it leaves a sensor's position, and its orientation, from where the newer
    /// calibration has them (m, radians), over the sensors it is fitted to.
    pub worst_m: f64,
    pub worst_rad: f64,
    /// A sensor left out because the newer calibration moved it against the others.
    pub left_out: Option<String>,
}

impl WorldChange {
    pub fn within_tolerance(&self) -> bool {
        self.worst_m <= SENSOR_TOLERANCE_M && self.worst_rad <= SENSOR_TOLERANCE_RAD
    }
}

/// The rigid fit of the sensors' poses in `then` to those in `now` (their positions, and points
/// half a metre along their axes), with how far it leaves them.
fn fit_change(
    pairs: &[(&nalgebra::Isometry3<f64>, &nalgebra::Isometry3<f64>)],
) -> Option<(nalgebra::Isometry3<f64>, f64, f64)> {
    use nalgebra::{Matrix3, Point3, Vector3};
    let points = |p: &nalgebra::Isometry3<f64>| -> Vec<Vector3<f64>> {
        [
            Vector3::zeros(),
            Vector3::x() * 0.5,
            Vector3::y() * 0.5,
            Vector3::z() * 0.5,
        ]
        .iter()
        .map(|v| (p * Point3::from(*v)).coords)
        .collect()
    };
    let src: Vec<Vector3<f64>> = pairs.iter().flat_map(|(a, _)| points(a)).collect();
    let dst: Vec<Vector3<f64>> = pairs.iter().flat_map(|(_, b)| points(b)).collect();
    let k = src.len() as f64;
    let (ca, cb) = (
        src.iter().sum::<Vector3<f64>>() / k,
        dst.iter().sum::<Vector3<f64>>() / k,
    );
    let h: Matrix3<f64> = src
        .iter()
        .zip(&dst)
        .map(|(a, b)| (b - cb) * (a - ca).transpose())
        .sum();
    let svd = h.svd(true, true);
    let (u, vt) = (svd.u?, svd.v_t?);
    let d = if (u * vt).determinant() < 0.0 {
        -1.0
    } else {
        1.0
    };
    let r = u * Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, d)) * vt;
    let t = cb - r * ca;
    let change =
        nalgebra::Isometry3::from_parts(t.into(), nalgebra::UnitQuaternion::from_matrix(&r));
    let (mut worst_m, mut worst_rad) = (0.0f64, 0.0f64);
    for (a, b) in pairs {
        let e = (change * **a).inverse() * **b;
        worst_m = worst_m.max(e.translation.vector.norm());
        worst_rad = worst_rad.max(e.rotation.angle());
    }
    Some((change, worst_m, worst_rad))
}

/// How one calibration's world frame sits against another's, by the sensors both place (they
/// stay where they are mounted): the rigid fit of the sensors' poses in `then` to those in `now`.
/// When that leaves a sensor beyond tolerance and three or more others remain, the one whose
/// leaving out fits the rest best is left out (a sensor the newer calibration re-placed after it
/// was bumped). None with fewer than two sensors in common.
pub fn world_change(
    then: &[(String, nalgebra::Isometry3<f64>)],
    now: &[(String, nalgebra::Isometry3<f64>)],
) -> Option<WorldChange> {
    let pairs: Vec<(
        &String,
        &nalgebra::Isometry3<f64>,
        &nalgebra::Isometry3<f64>,
    )> = then
        .iter()
        .filter_map(|(s, a)| {
            now.iter()
                .find(|(t, _)| same_sensor(t, s))
                .map(|(_, b)| (s, a, b))
        })
        .collect();
    if pairs.len() < 2 {
        return None;
    }
    let fit = |skip: Option<usize>| {
        let v: Vec<_> = pairs
            .iter()
            .enumerate()
            .filter(|(i, _)| Some(*i) != skip)
            .map(|(_, (_, a, b))| (*a, *b))
            .collect();
        fit_change(&v).map(|(change, worst_m, worst_rad)| WorldChange {
            change,
            worst_m,
            worst_rad,
            left_out: skip.map(|i| pairs[i].0.clone()),
        })
    };
    let all = fit(None)?;
    if all.within_tolerance() || pairs.len() < 4 {
        return Some(all);
    }
    (0..pairs.len())
        .filter_map(|i| fit(Some(i)))
        .min_by(|a, b| {
            (a.worst_m / SENSOR_TOLERANCE_M)
                .max(a.worst_rad / SENSOR_TOLERANCE_RAD)
                .total_cmp(
                    &(b.worst_m / SENSOR_TOLERANCE_M).max(b.worst_rad / SENSOR_TOLERANCE_RAD),
                )
        })
        .filter(WorldChange::within_tolerance)
        .or(Some(all))
}

/// calibration.json in cv1touch's configuration directory (platform::dirs).
pub fn default_path() -> PathBuf {
    crate::platform::dirs::get().config.join("calibration.json")
}

pub fn load(path: &Path) -> Result<CalibrationFile, String> {
    // O_NONBLOCK makes a FIFO fail the regular-file check instead of blocking daemon reload.
    let file = crate::platform::fs::open_read_nonblocking(path)
        .map_err(|e| format!("{}: {e}", path.display()))?;
    let metadata = file
        .metadata()
        .map_err(|e| format!("{}: {e}", path.display()))?;
    if !metadata.is_file() {
        return Err(format!(
            "{}: calibration must be a regular file",
            path.display()
        ));
    }
    if metadata.len() > MAX_FILE_BYTES {
        return Err(format!(
            "{}: calibration exceeds {MAX_FILE_BYTES} byte limit",
            path.display()
        ));
    }
    // Bound reads as well as metadata: another writer may grow the file after the size check.
    let mut bytes = Vec::new();
    file.take(MAX_FILE_BYTES + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| format!("{}: {e}", path.display()))?;
    if bytes.len() as u64 > MAX_FILE_BYTES {
        return Err(format!(
            "{}: calibration exceeds {MAX_FILE_BYTES} byte limit",
            path.display()
        ));
    }
    let f: CalibrationFile =
        serde_json::from_slice(&bytes).map_err(|e| format!("{}: {e}", path.display()))?;
    f.validate()
        .map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(f)
}

/// Writes atomically (temp file + rename). An existing file is first renamed to
/// `<name>.bak-<unix nanoseconds>`, which is returned.
pub fn save(path: &Path, f: &CalibrationFile) -> Result<Option<PathBuf>, String> {
    save_with_fingerprint(path, f).map(|(backup, _)| backup)
}

/// Also returns the SHA-256 of the exact serialized bytes installed by this save,
/// without rereading a path another writer may subsequently replace.
pub fn save_with_fingerprint(
    path: &Path,
    f: &CalibrationFile,
) -> Result<(Option<PathBuf>, String), String> {
    f.validate()?;
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    let text = serde_json::to_string_pretty(f).map_err(|e| e.to_string())? + "\n";
    if text.len() as u64 > MAX_FILE_BYTES {
        return Err(format!(
            "{}: calibration exceeds {MAX_FILE_BYTES} byte limit",
            path.display()
        ));
    }
    let fingerprint = format!("{:x}", Sha256::digest(text.as_bytes()));
    let tmp = path.with_extension("json.tmp");
    std::fs::write(&tmp, text).map_err(|e| format!("{}: {e}", tmp.display()))?;
    let backup = if path.exists() {
        let mut name = path.file_name().unwrap_or_default().to_os_string();
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_nanos());
        name.push(format!(".bak-{now}"));
        let b = path.with_file_name(name);
        std::fs::rename(path, &b).map_err(|e| format!("{}: {e}", b.display()))?;
        Some(b)
    } else {
        None
    };
    std::fs::rename(&tmp, path).map_err(|e| format!("{}: {e}", path.display()))?;
    Ok((backup, fingerprint))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("cv1t-calfile-{}-{tag}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d.join("calibration.json")
    }

    /// A recalibration that keeps the sensors where they are but turns and shifts the world:
    /// the change comes back exactly. One that also re-placed a bumped sensor: with three others
    /// that sensor is left out; with two, the change says how far off it leaves it. A sensor
    /// turned in place is caught as well as one moved.
    #[test]
    fn the_world_change_between_calibrations_comes_from_the_sensors() {
        use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
        let sensors: Vec<(String, Isometry3<f64>)> = [
            ((1.2, 0.5, -2.2), (0.2, 3.0, 0.0)),
            ((-1.4, 0.6, -2.1), (0.3, -2.8, 0.1)),
            ((0.1, 2.3, 1.9), (-0.9, 0.2, 0.0)),
            ((2.0, 0.4, 1.0), (0.1, 1.5, 0.0)),
        ]
        .iter()
        .enumerate()
        .map(|(i, &((x, y, z), (a, b, c)))| {
            (
                format!("WMTD{i}"),
                Isometry3::from_parts(
                    Translation3::new(x, y, z),
                    UnitQuaternion::from_scaled_axis(Vector3::new(a, b, c)),
                ),
            )
        })
        .collect();
        let change = Isometry3::from_parts(
            Translation3::new(0.004, -0.002, 0.01),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.02, 0.001)),
        );
        let now: Vec<_> = sensors
            .iter()
            .map(|(s, p)| (s.clone(), change * p))
            .collect();
        let w = world_change(&sensors, &now).unwrap();
        let d = w.change.inverse() * change;
        assert!(d.translation.vector.norm() < 1e-9 && d.rotation.angle() < 1e-9);
        assert!(w.worst_m < 1e-9 && w.within_tolerance() && w.left_out.is_none());
        let mut bumped = now.clone();
        bumped[2].1 = Isometry3::translation(0.06, 0.0, 0.0) * bumped[2].1;
        let w = world_change(&sensors, &bumped).unwrap();
        assert_eq!(w.left_out.as_deref(), Some("WMTD2"));
        let d = w.change.inverse() * change;
        assert!(d.translation.vector.norm() < 1e-9 && w.within_tolerance());
        let w = world_change(&sensors[..3], &bumped[..3]).unwrap();
        assert!(w.left_out.is_none() && !w.within_tolerance(), "{w:?}");
        let mut turned = now.clone();
        turned[1].1 *= UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.0, 0.05));
        let w = world_change(&sensors[..3], &turned[..3]).unwrap();
        assert!(!w.within_tolerance(), "{w:?}");
        assert!(world_change(&sensors[..1], &now).is_none());
    }

    #[test]
    fn mixed_case_world_change_recovers_the_recorded_transform() {
        use nalgebra::{Isometry3, Translation3, UnitQuaternion};
        let then = vec![
            ("camera1".into(), Isometry3::translation(-1.0, 0.5, 2.0)),
            ("camera2".into(), Isometry3::translation(1.0, 1.5, -2.0)),
        ];
        let change = Isometry3::from_parts(
            Translation3::new(0.4, -0.2, 0.1),
            UnitQuaternion::from_euler_angles(0.1, -0.2, 0.3),
        );
        let now: Vec<_> = then
            .iter()
            .map(|(s, p): &(String, _)| (s.to_ascii_uppercase(), change * p))
            .collect();
        let recovered = world_change(&then, &now).expect("two common sensors");
        let error = recovered.change.inverse() * change;
        assert!(error.translation.vector.norm() < 1e-9);
        assert!(error.rotation.angle() < 1e-9);
        assert!(recovered.within_tolerance());
    }

    #[test]
    fn saved_fingerprint_matches_exact_bytes_and_survives_external_replacement() {
        let path = tmp("saved-fingerprint");
        let a = CalibrationFile::example();
        let (backup, accepted) = save_with_fingerprint(&path, &a).unwrap();
        assert!(backup.is_none());
        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(bytes.last(), Some(&b'\n'));
        assert_eq!(accepted, crate::setup::fingerprint(&path).unwrap());
        assert_eq!(accepted, format!("{:x}", Sha256::digest(&bytes)));
        let mut b = a.clone();
        b.source = "another writer's valid calibration".into();
        save(&path, &b).unwrap();
        assert_ne!(accepted, crate::setup::fingerprint(&path).unwrap());
        assert_eq!(accepted, format!("{:x}", Sha256::digest(bytes)));
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }
    #[test]
    fn save_and_load_round_trip_with_backup() {
        let path = tmp("roundtrip");
        let f = CalibrationFile::example();
        assert_eq!(save(&path, &f).unwrap(), None);
        let backup = save(&path, &f)
            .unwrap()
            .expect("second save backs up the first");
        assert!(backup.exists());
        assert_eq!(load(&path).unwrap(), f);
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }

    #[test]
    fn sensor_poses_carry_serial_and_pose() {
        let poses = CalibrationFile::example().sensor_poses();
        assert_eq!(poses.len(), 2);
        assert_eq!(
            (poses[1].serial.as_str(), poses[1].pos),
            ("WMTD000000002", [-2.0, 2.3, -1.8])
        );
    }

    /// Review Focus 4 (file side): every malformed file is refused with a reason.
    #[test]
    fn rejects_other_versions_duplicates_and_bad_quaternions() {
        let mut f = CalibrationFile::example();
        f.version = 2;
        assert!(f.validate().unwrap_err().contains("version"));
        let mut f = CalibrationFile::example();
        f.sensors[1].serial = f.sensors[0].serial.clone();
        assert!(f.validate().unwrap_err().contains("duplicate"));
        let mut f = CalibrationFile::example();
        f.sensors[0].orient_xyzw = [0.0, 0.0, 0.0, 2.0];
        assert!(f.validate().unwrap_err().contains("quaternion"));
        let mut f = CalibrationFile::example();
        f.sensors[0].pos[2] = f32::NAN;
        assert!(f.validate().is_err());
        let mut f = CalibrationFile::example();
        f.sensors.clear();
        assert!(f.validate().is_err());
    }

    /// The engine takes `camera1` and `CAMERA1` for one sensor and refuses both: the file is
    /// refused at load, saying which entries clash, rather than failing every engine start.
    #[test]
    fn serials_that_differ_only_in_letter_case_are_duplicates() {
        let mut f = CalibrationFile::example();
        f.sensors[0].serial = "Camera1".into();
        f.sensors[1].serial = "cAMERA1".into();
        let err = f.validate().unwrap_err();
        assert!(
            err.contains("duplicate sensor cAMERA1") && err.contains("Camera1 is already listed"),
            "{err}"
        );
        f.sensors[1].serial = "CAMERA10".into();
        f.validate().unwrap();
        let path = tmp("case-duplicates");
        f.sensors[1].serial = "CAMERA1".into();
        assert!(save(&path, &f).unwrap_err().contains("duplicate sensor"));
        std::fs::write(&path, serde_json::to_string(&f).unwrap()).unwrap();
        assert!(load(&path)
            .unwrap_err()
            .contains("duplicate sensor CAMERA1"));
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }

    /// An entry is the sensor a device reports whatever the letter case of either; the file's
    /// own spelling stays.
    #[test]
    fn sensors_are_looked_up_regardless_of_letter_case() {
        let mut f = CalibrationFile::example();
        f.sensors[0].serial = "wmtd000000001".into();
        assert_eq!(f.sensor("WMTD000000001").unwrap().serial, "wmtd000000001");
        assert!(f.sensor("WMTD00000000").is_none());
        assert!(f.sensor("WMTD0000000010").is_none());
        f.sensor_mut("WmTd000000001").unwrap().observations = 7;
        assert_eq!(f.sensors[0].observations, 7);
        f.remove_sensor("WMTD000000001");
        assert_eq!(f.sensors.len(), 1);
        assert_eq!(f.sensors[0].serial, "WMTD000000002");
        f.remove_sensor("WMTD000000009");
        assert_eq!(f.sensors.len(), 1);
    }

    #[test]
    fn missing_or_corrupt_files_name_the_path() {
        let path = tmp("corrupt");
        assert!(load(&path)
            .unwrap_err()
            .contains(&path.display().to_string()));
        std::fs::write(&path, "{").unwrap();
        assert!(load(&path)
            .unwrap_err()
            .contains(&path.display().to_string()));
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }

    /// Review fix 4: a sensor calibrated before but absent from a new solve (it never opened) is named.
    #[test]
    fn names_sensors_missing_from_a_new_solve() {
        let prev = CalibrationFile::example();
        assert_eq!(
            missing_from(
                &prev,
                &[
                    "WMTD000000001".into(),
                    "WMTD000000002".into(),
                    "WMTD000000003".into()
                ]
            ),
            Vec::<String>::new()
        );
        assert_eq!(
            missing_from(&prev, &["WMTD000000001".into()]),
            vec!["WMTD000000002".to_string()]
        );
        // a device's serial in another letter case is the same sensor
        assert_eq!(
            missing_from(&prev, &["wmtd000000001".into(), "Wmtd000000002".into()]),
            Vec::<String>::new()
        );
    }
    #[test]
    fn dynamic_sensors_round_trip_nine_and_sixty_five() {
        for count in [9, 65] {
            let path = tmp(&format!("dynamic-{count}"));
            let mut f = CalibrationFile::example();
            f.sensors = (0..count)
                .map(|i| {
                    let mut sensor = f.sensors[0].clone();
                    sensor.serial = format!("TEST{i:04}");
                    sensor
                })
                .collect();
            save(&path, &f).unwrap();
            assert_eq!(load(&path).unwrap(), f);
            f.sensors[count - 1].serial = f.sensors[0].serial.clone();
            assert!(f.validate().unwrap_err().contains("duplicate"));
            f.sensors[count - 1].serial = "UNIQUE".into();
            f.sensors[count - 1].pos[0] = f32::NAN;
            assert!(f.validate().is_err());
            std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
        }
    }

    #[test]
    fn dynamic_sensors_reject_oversized_file_before_parsing() {
        let path = tmp("oversized");
        let file = std::fs::File::create(&path).unwrap();
        file.set_len(16 * 1024 * 1024 + 1).unwrap();
        let error = load(&path).unwrap_err();
        assert!(error.contains("byte limit"), "{error}");
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }
    #[test]
    #[cfg(unix)]
    fn dynamic_sensors_reject_fifo_without_waiting_for_a_writer() {
        use std::ffi::CString;
        use std::os::unix::ffi::OsStrExt;
        use std::sync::mpsc;
        use std::time::Duration;
        let path = tmp("fifo");
        let name = CString::new(path.as_os_str().as_bytes()).unwrap();
        // SAFETY: the path is NUL-terminated and points inside this test's temporary directory.
        assert_eq!(unsafe { libc::mkfifo(name.as_ptr(), 0o600) }, 0);
        let (send, receive) = mpsc::channel();
        let reader_path = path.clone();
        let reader = std::thread::spawn(move || send.send(load(&reader_path)).unwrap());
        let result = receive.recv_timeout(Duration::from_secs(3));
        if result.is_err() {
            // Release a regressed blocking reader so the test can fail without hanging CI.
            use std::io::Write;
            use std::os::unix::fs::OpenOptionsExt;
            if let Ok(mut writer) = std::fs::OpenOptions::new()
                .write(true)
                .custom_flags(libc::O_NONBLOCK)
                .open(&path)
            {
                let _ = writer.write_all(b"{}");
            }
        }
        reader.join().unwrap();
        assert!(result.unwrap().unwrap_err().contains("regular file"));
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }
}
