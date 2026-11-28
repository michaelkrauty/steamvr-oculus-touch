// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! CLI recording joins, using isolated files and no hardware or installed configuration.
#![cfg(feature = "engine")]

use cv1touch::{calib::file, recorder};
use std::process::Command;

#[test]
fn mixed_case_refine_and_place_commands_find_the_recorded_camera() {
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path();
    let calibration = dir.join("calibration.json");
    let rec = dir.join("capture.rec");
    std::fs::write(&calibration, serde_json::to_vec(&serde_json::json!({
        "format": file::FORMAT, "version": 1, "created_unix_s": 1,
        "world": "test", "source": "synthetic recording",
        "sensors": [
            {"serial": "camera1", "orient_xyzw": [0,0,0,1], "pos": [0,0,0], "rms_px": 0, "observations": 100},
            {"serial": "camera2", "orient_xyzw": [0,0,0,1], "pos": [1,0,0], "rms_px": 0, "observations": 100}
        ],
        "stats": {"reference": "camera1", "gravity_mps2": 9.81, "gravity_spread_deg": 0,
            "motion_segments": 10, "samples": 1000, "rejected_views": 0}
    })).unwrap()).unwrap();
    let make_sensor = |name: &[u8]| {
        let mut sensor = recorder::SensorInfo {
            width: 1280,
            height: 960,
            fx: 715.0,
            fy: 715.0,
            cx: 640.0,
            cy: 480.0,
            ..Default::default()
        };
        for (out, byte) in sensor.serial.iter_mut().zip(name) {
            *out = *byte as _;
        }
        sensor
    };
    let recording = recorder::Recorder::new(1 << 20);
    recording.start(&rec, 1).unwrap();
    recording.write(
        recorder::SENSOR_INFO,
        1,
        &[recorder::as_bytes(&make_sensor(b"CAMERA1"))],
    );
    recording.write(
        recorder::SENSOR_INFO,
        1,
        &[recorder::as_bytes(&make_sensor(b"CAMERA2"))],
    );
    let stats = recording.stop();
    assert_eq!((stats.dropped, stats.write_errors), (0, 0));
    let run = |args: &[&str]| {
        Command::new(env!("CARGO_BIN_EXE_cv1touch"))
            .args(args)
            .arg(&rec)
            .arg("--calibration")
            .arg(&calibration)
            .env("HOME", dir)
            .env("LOCALAPPDATA", dir)
            .env("XDG_CONFIG_HOME", dir)
            .env("XDG_STATE_HOME", dir)
            .output()
            .unwrap()
    };
    let refined = run(&["calibrate", "refine"]);
    assert_eq!(refined.status.code(), Some(1));
    let refine_err = String::from_utf8_lossy(&refined.stderr);
    // The empty capture cannot improve poses, but must match both mixed-case sensors against
    // calibration.json and fail on exposure count rather than "needs at least two calibrated sensors".
    assert!(
        refine_err.contains("not enough multi-sensor exposures"),
        "{refine_err}"
    );
    let placed = run(&["calibrate", "place", "cAmErA1"]);
    assert_eq!(placed.status.code(), Some(1));
    let error = String::from_utf8_lossy(&placed.stderr);
    assert!(error.contains("only 0 labelled LED sightings"), "{error}");
    assert_eq!(
        file::load(&calibration).unwrap().sensors[0].serial,
        "camera1"
    );
}
