// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! CLI recording joins, using isolated files and no hardware or installed configuration.
#![cfg(feature = "engine")]

use cv1touch::{calib::file, recorder};
use std::process::Command;

#[test]
fn mixed_case_refine_and_place_commands_find_the_recorded_camera() {
    let dir = std::env::temp_dir().join(format!("cv1t-serial-command-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let calibration = dir.join("calibration.json");
    let rec = dir.join("capture.rec");
    std::fs::write(&calibration, serde_json::to_vec(&serde_json::json!({
        "format": file::FORMAT, "version": 1, "created_unix_s": 1,
        "world": "test", "source": "synthetic recording",
        "sensors": [{"serial": "camera1", "orient_xyzw": [0,0,0,1], "pos": [0,0,0], "rms_px": 0, "observations": 100}],
        "stats": {"reference": "camera1", "gravity_mps2": 9.81, "gravity_spread_deg": 0,
            "motion_segments": 10, "samples": 1000, "rejected_views": 0}
    })).unwrap()).unwrap();
    let mut sensor = recorder::SensorInfo {
        width: 1280,
        height: 960,
        fx: 715.0,
        fy: 715.0,
        cx: 640.0,
        cy: 480.0,
        ..Default::default()
    };
    for (out, byte) in sensor.serial.iter_mut().zip(b"CAMERA1") {
        *out = *byte as _;
    }
    let recording = recorder::Recorder::new(1 << 20);
    recording.start(&rec, 1).unwrap();
    recording.write(recorder::SENSOR_INFO, 1, &[recorder::as_bytes(&sensor)]);
    let stats = recording.stop();
    assert_eq!((stats.dropped, stats.write_errors), (0, 0));
    let run = |args: &[&str]| {
        Command::new(env!("CARGO_BIN_EXE_cv1touch"))
            .args(args)
            .arg(&rec)
            .arg("--calibration")
            .arg(&calibration)
            .env("HOME", &dir)
            .env("LOCALAPPDATA", &dir)
            .env("XDG_CONFIG_HOME", &dir)
            .env("XDG_STATE_HOME", &dir)
            .output()
            .unwrap()
    };
    let refined = run(&["calibrate", "refine"]);
    assert!(
        refined.status.success(),
        "{}",
        String::from_utf8_lossy(&refined.stderr)
    );
    let text = String::from_utf8_lossy(&refined.stdout);
    // The empty capture cannot improve a pose, but must attempt refinement. An exact
    // metadata lookup silently skips it and prints only "no sensor improves".
    assert!(
        text.contains("round 1: camera1: only 0 LED matches"),
        "{text}"
    );
    let placed = run(&["calibrate", "place", "cAmErA1"]);
    assert_eq!(placed.status.code(), Some(1));
    let error = String::from_utf8_lossy(&placed.stderr);
    assert!(error.contains("only 0 labelled LED sightings"), "{error}");
    assert_eq!(
        file::load(&calibration).unwrap().sensors[0].serial,
        "camera1"
    );
    std::fs::remove_dir_all(dir).unwrap();
}
