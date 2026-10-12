// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! calibration.json: each sensor's world pose (camera -> world, OpenCV camera axes) plus residuals
//! and provenance. Loaded by cv1touchd at startup and on `calibration load`.

use std::collections::HashSet;
use std::io::Read;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use super::iso_to_xyzw;
use super::solve::Solution;
use crate::engine::SensorPose;

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

    pub fn validate(&self) -> Result<(), String> {
        if self.format != FORMAT || self.version != VERSION {
            return Err(format!("unsupported calibration format {:?} version {} (expected {FORMAT:?} version {VERSION})", self.format, self.version));
        }
        if self.sensors.is_empty() {
            return Err("calibration lists no sensors (expected at least one)".into());
        }
        let mut seen = HashSet::new();
        for s in &self.sensors {
            if s.serial.is_empty()
                || s.serial.len() > 31
                || !s.serial.bytes().all(|b| b.is_ascii_alphanumeric())
            {
                return Err(format!("invalid sensor serial {:?}", s.serial));
            }
            if !seen.insert(s.serial.as_str()) {
                return Err(format!("duplicate sensor {}", s.serial));
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
        .filter(|s| !serials.contains(s))
        .collect()
}

/// $XDG_CONFIG_HOME/cv1touch/calibration.json, falling back to ~/.config.
pub fn default_path() -> PathBuf {
    std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            PathBuf::from(std::env::var_os("HOME").unwrap_or_default()).join(".config")
        })
        .join("cv1touch")
        .join("calibration.json")
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
