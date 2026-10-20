// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Tracking modes and the engine options each implies. Provisional: runtime sensor placement.
//! Capture: no placement, every sensor only observes (calibration capture). Calibrated: fixed
//! sensor poses from a calibration file.

use std::path::Path;

use crate::calib::file;
use crate::engine::{EngineOptions, SensorPose};
use crate::status::same_sensor;

pub fn provisional(base: &EngineOptions) -> EngineOptions {
    EngineOptions {
        fixed_poses_only: false,
        fixed_poses: Vec::new(),
        ..base.clone()
    }
}

pub fn capture(base: &EngineOptions) -> EngineOptions {
    EngineOptions {
        fixed_poses_only: true,
        fixed_poses: Vec::new(),
        ..base.clone()
    }
}

/// Options with the file's sensor poses and their count. An error leaves nothing changed.
pub fn calibrated(base: &EngineOptions, path: &Path) -> Result<(EngineOptions, usize), String> {
    let poses = file::load(path)?.sensor_poses();
    let n = poses.len();
    Ok((
        EngineOptions {
            fixed_poses_only: true,
            fixed_poses: poses,
            ..base.clone()
        },
        n,
    ))
}

/// `--ignore-sensor SERIAL`: leaves a sensor closed. Naming one twice, in whatever letter case,
/// is refused here: the engine would turn the whole configuration down on every open attempt.
pub fn ignore_sensor(opts: &mut EngineOptions, serial: String) -> Result<(), String> {
    if let Some(first) = opts.ignore_sensors.iter().find(|s| same_sensor(s, &serial)) {
        return Err(format!(
            "--ignore-sensor {serial} names the same sensor as an earlier --ignore-sensor {first} (serial letter case does not matter)"
        ));
    }
    opts.ignore_sensors.push(serial);
    Ok(())
}

/// The calibrated pose of the sensor the engine reports as `serial`. The native engine matches
/// serials without regard to ASCII letter case, and so does this: a calibration spelled
/// `wmtd...` places the camera a device reports as `WMTD...`.
pub fn calibrated_pose<'a>(opts: &'a EngineOptions, serial: &str) -> Option<&'a SensorPose> {
    opts.fixed_poses
        .iter()
        .find(|p| same_sensor(&p.serial, serial))
}

/// Open sensors that will only observe because the calibration has no pose for them (empty unless
/// calibrated mode).
pub fn unposed_sensors(opts: &EngineOptions, open: &[String]) -> Vec<String> {
    if !opts.fixed_poses_only || opts.fixed_poses.is_empty() {
        return Vec::new();
    }
    open.iter()
        .filter(|s| calibrated_pose(opts, s).is_none())
        .cloned()
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::file::{self, CalibrationFile};

    fn base() -> EngineOptions {
        EngineOptions {
            radio_wait_ms: 2,
            hmd_leds_off: true,
            ignore_sensors: vec!["WMTD000000004".into()],
            ..Default::default()
        }
    }

    fn tmp(tag: &str) -> std::path::PathBuf {
        let d = std::env::temp_dir().join(format!("cv1t-modes-{}-{tag}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d.join("calibration.json")
    }

    #[test]
    fn capture_observes_only_and_keeps_the_rest() {
        let o = capture(&EngineOptions {
            fixed_poses: CalibrationFile::example().sensor_poses(),
            ..base()
        });
        assert!(o.fixed_poses_only && o.fixed_poses.is_empty());
        assert_eq!(
            (o.radio_wait_ms, o.hmd_leds_off, o.ignore_sensors),
            (2, true, base().ignore_sensors)
        );
    }

    #[test]
    fn provisional_uses_runtime_sensor_placement() {
        let o = provisional(&EngineOptions {
            fixed_poses_only: true,
            fixed_poses: CalibrationFile::example().sensor_poses(),
            ..base()
        });
        assert!(!o.fixed_poses_only && o.fixed_poses.is_empty());
    }

    #[test]
    fn calibrated_loads_the_file() {
        let path = tmp("ok");
        file::save(&path, &CalibrationFile::example()).unwrap();
        let (o, n) = calibrated(&base(), &path).unwrap();
        assert_eq!(n, 2);
        assert!(o.fixed_poses_only);
        assert_eq!(o.fixed_poses[0].serial, "WMTD000000001");
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }

    /// Review Focus 4: a bad file is an error (the daemon then keeps its current mode).
    #[test]
    fn failed_load_is_an_error_and_changes_nothing() {
        let path = tmp("bad");
        assert!(calibrated(&base(), &path).is_err()); // missing
        std::fs::write(&path, "{\"format\":\"cv1touch-calibration\"}").unwrap();
        assert!(calibrated(&base(), &path).is_err()); // incomplete
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }

    #[test]
    fn a_sensor_ignored_twice_in_any_letter_case_is_refused() {
        let mut o = base();
        ignore_sensor(&mut o, "camera1".into()).unwrap();
        ignore_sensor(&mut o, "CAMERA2".into()).unwrap();
        let err = ignore_sensor(&mut o, "CAMERA1".into()).unwrap_err();
        assert!(
            err.contains("--ignore-sensor CAMERA1 names the same sensor as an earlier --ignore-sensor camera1"),
            "{err}"
        );
        assert!(ignore_sensor(&mut o, "camera2".into()).is_err());
        assert_eq!(o.ignore_sensors, ["WMTD000000004", "camera1", "CAMERA2"]);
    }

    /// A calibration spelled in another letter case than the device's names the same sensor, as in
    /// the native engine: the camera has its pose, and is not reported as unposed.
    #[test]
    fn calibrated_poses_match_device_serials_regardless_of_letter_case() {
        let mut poses = CalibrationFile::example().sensor_poses();
        poses[0].serial = poses[0].serial.to_ascii_lowercase();
        let o = EngineOptions {
            fixed_poses_only: true,
            fixed_poses: poses,
            ..base()
        };
        let pose = calibrated_pose(&o, "WMTD000000001").expect("the lowercase entry matches");
        assert_eq!(
            pose.serial, "wmtd000000001",
            "the calibration's spelling is kept"
        );
        assert!(calibrated_pose(&o, "wmtd000000001").is_some());
        assert!(calibrated_pose(&o, "WMTD00000000").is_none());
        assert!(calibrated_pose(&o, "WMTD0000000010").is_none());
        let open = ["WMTD000000001".to_string(), "WMTD000000003".to_string()];
        assert_eq!(
            unposed_sensors(&o, &open),
            vec!["WMTD000000003".to_string()]
        );
    }

    /// Review fix 4: in calibrated mode an open sensor without a pose only observes; name it.
    #[test]
    fn names_open_sensors_without_a_calibrated_pose() {
        let path = tmp("unposed");
        file::save(&path, &CalibrationFile::example()).unwrap();
        let (o, _) = calibrated(&base(), &path).unwrap();
        let open = [
            "WMTD000000001".to_string(),
            "WMTD000000002".to_string(),
            "WMTD000000003".to_string(),
        ];
        assert_eq!(
            unposed_sensors(&o, &open),
            vec!["WMTD000000003".to_string()]
        );
        assert!(unposed_sensors(&provisional(&o), &open).is_empty());
        assert!(unposed_sensors(&capture(&o), &open).is_empty());
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }
}
