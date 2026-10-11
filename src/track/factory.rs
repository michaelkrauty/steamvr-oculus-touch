// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! A Touch controller's factory IMU position. The engine reads each controller's calibration JSON
//! over the radio and caches it in OpenHMD's config directory (rift-touch-config-<hash>.bin); the
//! cached file whose LED model matches the controller's gives its ImuPosition (metres, LED model
//! frame, which the factory-calibrated IMU axes share).

use std::path::{Path, PathBuf};

use nalgebra::Vector3;

use crate::calib::capture::Led;

/// $XDG_CONFIG_HOME/openhmd, else ~/.config/openhmd (as the engine resolves it).
pub fn openhmd_config_dir() -> PathBuf {
    match std::env::var_os("XDG_CONFIG_HOME") {
        Some(x) => PathBuf::from(x).join("openhmd"),
        None => PathBuf::from(std::env::var_os("HOME").unwrap_or_default()).join(".config/openhmd"),
    }
}

/// (LED positions, IMU position) from one cached configuration.
fn parse(text: &str) -> Option<(Vec<Vector3<f64>>, Vector3<f64>)> {
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
    let imu = v3(&obj["ImuPosition"])?;
    let points = obj["ModelPoints"].as_object()?;
    let leds = (0..points.len())
        .map(|i| v3(points.get(&format!("Point{i}"))?))
        .collect::<Option<Vec<_>>>()?;
    Some((leds, imu))
}

/// The IMU position from the cached configuration in `dir` whose LED positions match `leds` to
/// within half a millimetre.
pub fn imu_position(dir: &Path, leds: &[Led]) -> Option<Vector3<f64>> {
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
        format!("{{\"TableVersion\": 1, \"TrackedObject\": {{\"JsonVersion\": 2, \"ImuPosition\": [{}, {}, {}], \"ModelPoints\": {{{}}}}}}}", imu[0], imu[1], imu[2], pts.join(", "))
    }

    #[test]
    fn the_matching_controllers_imu_position_is_found() {
        let dir = std::env::temp_dir().join(format!("cv1touch-factory-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
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
            imu_position(&dir, &leds(&right)),
            Some(Vector3::new(0.0154, -0.0126, 0.0082))
        );
        assert_eq!(imu_position(&dir, &leds(&left)).map(|v| v.x), Some(-0.002));
        assert_eq!(
            imu_position(&dir, &leds(&[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])),
            None
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
