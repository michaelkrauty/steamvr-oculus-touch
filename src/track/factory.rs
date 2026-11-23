// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! A Touch controller's factory IMU data. The engine reads each controller's calibration JSON
//! over the radio and caches it in OpenHMD's config directory (rift-touch-config-<hash>.bin); the
//! cached file whose LED model matches the controller's gives its ImuPosition (metres, LED model
//! frame, which the factory-calibrated IMU axes share) and the factory calibration the engine
//! applies to every gyro sample.

use std::path::{Path, PathBuf};

use nalgebra::{Matrix3, Vector3};

use crate::calib::capture::Led;

/// OpenHMD's cache directory, as the engine resolves it (platform::dirs::openhmd).
pub fn openhmd_config_dir() -> PathBuf {
    crate::platform::dirs::openhmd()
}

/// A controller's IMU as its cached configuration describes it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Imu {
    /// Metres, LED model frame.
    pub position: Vector3<f64>,
    pub gyro: Gyro,
}

impl From<Vector3<f64>> for Imu {
    /// An IMU at `position` whose gyro's factory calibration changes nothing.
    fn from(position: Vector3<f64>) -> Imu {
        Imu {
            position,
            gyro: Gyro::default(),
        }
    }
}

/// A controller's gyro through its factory calibration. It reports signed 16-bit counts, 32768
/// of them 2000 deg/s; the engine scales a count to rad/s, subtracts the factory offset and
/// applies the factory matrix (`rift_imu_apply_calibration`). Undoing that gives back a
/// calibrated sample's counts, and an axis within one count of either end of the range read its
/// rail: the true rate may lie anywhere beyond it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gyro {
    pub matrix: Matrix3<f64>,
    inverse: Matrix3<f64>,
    offset: Vector3<f64>,
}

/// rad/s per count, as the engine scales it.
pub const GYRO_PER_COUNT: f64 = std::f64::consts::PI / (16.384 * 180.0);

impl Gyro {
    fn new(matrix: Matrix3<f64>, offset: Vector3<f64>) -> Option<Gyro> {
        Some(Gyro {
            inverse: matrix.try_inverse()?,
            matrix,
            offset,
        })
    }

    /// The counts the gyro reported for the calibrated rate `w`.
    pub fn counts(&self, w: &Vector3<f64>) -> Vector3<f64> {
        (self.inverse * w + self.offset) / GYRO_PER_COUNT
    }

    /// The end of the range that `count` reads at, if it is within one count of one.
    pub fn rail(count: f64) -> Option<f64> {
        let (lo, hi) = (f64::from(i16::MIN), f64::from(i16::MAX));
        let c = count.round();
        if c >= hi - 1.0 {
            Some(hi)
        } else if c <= lo + 1.0 {
            Some(lo)
        } else {
            None
        }
    }
}

impl Default for Gyro {
    /// A gyro whose factory calibration changes nothing.
    fn default() -> Self {
        Gyro::new(Matrix3::identity(), Vector3::zeros()).unwrap()
    }
}

/// (LED positions, IMU) from one cached configuration.
fn parse(text: &str) -> Option<(Vec<Vector3<f64>>, Imu)> {
    let (a, b) = (text.find('{')?, text.rfind('}')?);
    if b < a {
        return None;
    }
    let json: serde_json::Value = serde_json::from_str(&text[a..=b]).ok()?;
    let obj = &json["TrackedObject"];
    let v3 = |v: &serde_json::Value| -> Option<Vector3<f64>> {
        Some(Vector3::new(
            v.get(0)?.as_f64()?,
            v.get(1)?.as_f64()?,
            v.get(2)?.as_f64()?,
        ))
    };
    // nine matrix entries row by row, then the offsets, zero when absent as the engine reads them
    let gyro = obj["GyroCalibration"].as_array()?;
    let m: Vec<f64> = (0..9)
        .map(|i| gyro.get(i)?.as_f64())
        .collect::<Option<_>>()?;
    let offset = |i: usize| gyro.get(i).map_or(Some(0.0), serde_json::Value::as_f64);
    let imu = Imu {
        position: v3(&obj["ImuPosition"])?,
        gyro: Gyro::new(
            Matrix3::from_row_slice(&m),
            Vector3::new(offset(9)?, offset(10)?, offset(11)?),
        )?,
    };
    let points = obj["ModelPoints"].as_object()?;
    let leds = (0..points.len())
        .map(|i| v3(points.get(&format!("Point{i}"))?))
        .collect::<Option<Vec<_>>>()?;
    Some((leds, imu))
}

/// The IMU from the cached configuration in `dir` whose LED positions match `leds` to within half
/// a millimetre.
pub fn imu(dir: &Path, leds: &[Led]) -> Option<Imu> {
    let entries = std::fs::read_dir(dir).ok()?;
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().into_owned();
        if !(name.starts_with("rift-touch-config-") && name.ends_with(".bin")) {
            continue;
        }
        let Ok(bytes) = std::fs::read(entry.path()) else {
            continue;
        };
        let Some((points, imu)) = parse(&String::from_utf8_lossy(&bytes)) else {
            continue;
        };
        if points.len() == leds.len()
            && points
                .iter()
                .zip(leds)
                .all(|(p, l)| (p - l.pos).norm() < 5e-4)
        {
            return Some(imu);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Axes turned and scaled as a real controller's, with offsets.
    const GYRO: &str =
        "[0.0, -1.005, 0.002, 0.0, 0.003, 0.999, -1.004, 0.0, 0.0, 0.01, -0.006, 0.005]";

    fn config(points: &[[f64; 3]], imu: [f64; 3]) -> String {
        let pts: Vec<String> = points
            .iter()
            .enumerate()
            .map(|(i, p)| {
                format!(
                    "\"Point{i}\": [{}, {}, {}, 0.0, 1.0, 0.0, 75, 75, 0]",
                    p[0], p[1], p[2]
                )
            })
            .collect();
        format!("{{\"TableVersion\": 1, \"TrackedObject\": {{\"JsonVersion\": 2, \"ImuPosition\": [{}, {}, {}], \"GyroCalibration\": {GYRO}, \"ModelPoints\": {{{}}}}}}}", imu[0], imu[1], imu[2], pts.join(", "))
    }

    #[test]
    fn the_matching_controllers_imu_is_found() {
        let tmp = tempfile::tempdir().unwrap();
        let dir = tmp.path();
        let left = [[-0.03, -0.008, -0.011], [-0.02, 0.01, 0.0]];
        let right = [[0.03, -0.008, -0.011], [0.02, 0.01, 0.0]];
        std::fs::write(
            dir.join("rift-touch-config-aa.bin"),
            config(&left, [-0.002, -0.0126, -0.0064]),
        )
        .unwrap();
        std::fs::write(
            dir.join("rift-touch-config-bb.bin"),
            config(&right, [0.0154, -0.0126, 0.0082]),
        )
        .unwrap();
        std::fs::write(dir.join("rift-touch-config-cc.bin"), b"\x00garbage").unwrap();
        std::fs::write(dir.join("rift-touch-config-dd.bin"), b"} broken {").unwrap();
        let leds = |pts: &[[f64; 3]]| {
            pts.iter()
                .map(|p| Led {
                    pos: Vector3::from(*p),
                    dir: Vector3::y(),
                })
                .collect::<Vec<_>>()
        };
        assert_eq!(
            imu(dir, &leds(&right)).map(|i| i.position),
            Some(Vector3::new(0.0154, -0.0126, 0.0082))
        );
        assert_eq!(imu(dir, &leds(&left)).map(|i| i.position.x), Some(-0.002));
        assert_eq!(imu(dir, &leds(&[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])), None);
    }

    /// A calibrated sample is taken back to the counts the gyro reported, through the factory
    /// matrix and offsets (absent offsets are zero), and an axis within one count of either end
    /// of the 16-bit range reads its rail.
    #[test]
    fn a_rate_at_full_scale_is_found_through_the_calibration() {
        let leds = [[0.03, -0.008, -0.011], [0.02, 0.01, 0.0]];
        let with = config(&leds, [0.0; 3]);
        let without = with.replace(", 0.01, -0.006, 0.005]", "]");
        for (text, offset) in [
            (with, Vector3::new(0.01, -0.006, 0.005)),
            (without, Vector3::zeros()),
        ] {
            let (_, imu) = parse(&text).unwrap();
            for (count, rail) in [
                (32767.0, Some(32767.0)),
                (32766.0, Some(32767.0)),
                (32765.0, None),
                (-32768.0, Some(-32768.0)),
                (-32767.0, Some(-32768.0)),
                (-32766.0, None),
                (0.0, None),
            ] {
                let raw = Vector3::new(count, -1000.0, 12.0) * GYRO_PER_COUNT;
                // as the engine computes it, in single precision
                let calibrated = (imu.gyro.matrix * (raw - offset)).map(|v| f64::from(v as f32));
                let back = imu.gyro.counts(&calibrated);
                assert!((back - raw / GYRO_PER_COUNT).norm() < 0.1, "{back}");
                assert_eq!(Gyro::rail(back.x), rail, "{count}");
                assert_eq!((Gyro::rail(back.y), Gyro::rail(back.z)), (None, None));
            }
        }
        assert!(parse(&config(&leds, [0.0; 3]).replace(GYRO, "[1.0, 0.0]")).is_none());
    }
}
