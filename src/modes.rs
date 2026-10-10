// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! Tracking modes and the engine options each implies. Provisional: Phase 1 runtime placement.
//! Capture: no placement, every sensor only observes (calibration capture). Calibrated: fixed
//! sensor poses from a calibration file.

use std::path::Path;

use crate::calib::file;
use crate::engine::EngineOptions;

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

/// Open sensors that will only observe because the calibration has no pose for them (empty unless
/// calibrated mode).
pub fn unposed_sensors(opts: &EngineOptions, open: &[String]) -> Vec<String> {
    if !opts.fixed_poses_only || opts.fixed_poses.is_empty() {
        return Vec::new();
    }
    open.iter()
        .filter(|s| !opts.fixed_poses.iter().any(|p| &p.serial == *s))
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
    fn provisional_is_phase_1_behaviour() {
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
