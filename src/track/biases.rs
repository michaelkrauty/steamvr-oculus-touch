// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Each controller's IMU biases, remembered across daemon restarts. The accelerometer's take tens
//! of seconds of varied orientation to learn from scratch (Touch R's reads 0.45 m/s^2 high on one
//! axis), with up to a degree of tilt until then; a remembered estimate holds from the first fix.
//! Controllers are told apart by hand and LED model, so a replaced controller starts fresh.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::calib::capture::Led;

pub const FORMAT: &str = "cv1touch-imu-biases";

/// How far a remembered bias is trusted when seeding a new run: the gyro's turn-on spread and the
/// accelerometer's drift with temperature.
pub const SEED_SD_GYRO: f64 = 0.005;
pub const SEED_SD_ACCEL: f64 = 0.15;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Entry {
    /// rad/s, body frame.
    pub gyro: [f64; 3],
    /// m/s^2, body frame.
    pub accel: [f64; 3],
    pub updated_unix_s: u64,
}

impl Entry {
    pub fn gyro(&self) -> Vector3<f64> {
        Vector3::from(self.gyro)
    }

    pub fn accel(&self) -> Vector3<f64> {
        Vector3::from(self.accel)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BiasFile {
    pub format: String,
    pub version: u32,
    pub controllers: BTreeMap<String, Entry>,
}

impl Default for BiasFile {
    fn default() -> Self {
        BiasFile {
            format: FORMAT.into(),
            version: 1,
            controllers: BTreeMap::new(),
        }
    }
}

/// $XDG_STATE_HOME/cv1touch/imu-biases.json, else ~/.local/state/cv1touch/imu-biases.json.
pub fn default_path() -> PathBuf {
    std::env::var_os("XDG_STATE_HOME")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            PathBuf::from(std::env::var_os("HOME").unwrap_or_default()).join(".local/state")
        })
        .join("cv1touch/imu-biases.json")
}

/// "left-1a2b3c4d": the hand and a fingerprint of the LED model at 0.1 mm.
pub fn key(hand: usize, leds: &[Led]) -> String {
    let mut h: u32 = 0x811c_9dc5;
    for l in leds {
        for v in l.pos.iter() {
            for b in ((v * 1e4).round() as i64).to_le_bytes() {
                h = (h ^ b as u32).wrapping_mul(0x0100_0193);
            }
        }
    }
    format!(
        "{}-{h:08x}",
        ["left", "right"].get(hand).copied().unwrap_or("other")
    )
}

/// The file at `path`; an empty one when it is missing or unreadable.
pub fn load(path: &Path) -> BiasFile {
    std::fs::read_to_string(path)
        .ok()
        .and_then(|t| serde_json::from_str::<BiasFile>(&t).ok())
        .filter(|f| f.format == FORMAT)
        .unwrap_or_default()
}

/// Writes `f` to `path` through a temporary file, so a crash never leaves half a file.
pub fn save(path: &Path, f: &BiasFile) -> Result<(), String> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    let tmp = path.with_extension("json.tmp");
    let text = serde_json::to_string_pretty(f).map_err(|e| e.to_string())?;
    std::fs::write(&tmp, text).map_err(|e| format!("{}: {e}", tmp.display()))?;
    std::fs::rename(&tmp, path).map_err(|e| format!("{}: {e}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn leds(dx: f64) -> Vec<Led> {
        vec![
            Led {
                pos: Vector3::new(0.03 + dx, -0.008, -0.011),
                dir: Vector3::y(),
            },
            Led {
                pos: Vector3::new(-0.02, 0.01, 0.0),
                dir: Vector3::y(),
            },
        ]
    }

    #[test]
    fn controllers_are_told_apart_by_hand_and_led_model() {
        assert_eq!(key(0, &leds(0.0)), key(0, &leds(0.0)));
        assert_eq!(
            key(0, &leds(0.0)),
            key(0, &leds(1e-6)),
            "below the 0.1 mm fingerprint"
        );
        assert_ne!(key(0, &leds(0.0)), key(1, &leds(0.0)));
        assert_ne!(key(0, &leds(0.0)), key(0, &leds(1e-3)));
        assert!(key(1, &leds(0.0)).starts_with("right-"));
    }

    #[test]
    fn biases_round_trip_and_a_missing_file_is_empty() {
        let dir = std::env::temp_dir().join(format!("cv1touch-biases-{}", std::process::id()));
        let path = dir.join("sub/imu-biases.json");
        assert_eq!(load(&path), BiasFile::default());
        let mut f = BiasFile::default();
        f.controllers.insert(
            key(1, &leds(0.0)),
            Entry {
                gyro: [0.01, -0.005, 0.003],
                accel: [0.018, 0.449, -0.081],
                updated_unix_s: 1_791_500_000,
            },
        );
        save(&path, &f).unwrap();
        assert_eq!(load(&path), f);
        std::fs::write(&path, "{ not json").unwrap();
        assert_eq!(load(&path), BiasFile::default());
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
