// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! CPU-only Hand -> shared ABI -> driver make_pose regression. No IPC objects or hardware.
//! Run: cargo run --no-default-features --example native_clock_pose -- DRIVER_TEST
//! where DRIVER_TEST is the Meson-built driver/test_native_clock_pose executable.
use cv1touch::{
    ffi,
    hand::{Hand, HandConfig},
    shm::{ControllerState, HAND_LEFT},
};
use std::io::Write;
use std::process::{Command, Stdio};

fn main() {
    let driver = std::env::args_os().nth(1).expect("DRIVER_TEST argument");
    let mut hand = Hand::new(HAND_LEFT, HandConfig::default());
    let mut c = ffi::ohmd_cv1touch_controller {
        calibrated: true,
        device_time_ns: 1_000_000_000,
        received_host_ns: 11_000_000_000,
        last_pose_fix_device_ns: 800_000_000,
        view_orient: [0., 0., 0., 1.],
        view_lin_vel: [3., 0., 0.],
        ..Default::default()
    };
    let before = hand.update(&c, c.received_host_ns);
    // Delayed final report: no newer report available in this drain. Both sample gaps and
    // host inter-read gaps stay below 100 ms; the historical optical fix is unchanged.
    c.device_time_ns += 2_000_000;
    c.received_host_ns += 62_000_000;
    let delayed = hand.update(&c, c.received_host_ns);
    c.device_time_ns = 1_064_000_000;
    c.received_host_ns = 11_064_000_000;
    let recovered = hand.update(&c, c.received_host_ns);

    let mut child = Command::new(driver).stdin(Stdio::piped()).spawn().unwrap();
    let mut input = child.stdin.take().unwrap();
    for state in [before, delayed, recovered] {
        // ControllerState is the production, padding-free shared ABI, read by the C++ owner.
        let bytes = unsafe {
            std::slice::from_raw_parts(
                &state as *const ControllerState as *const u8,
                std::mem::size_of::<ControllerState>(),
            )
        };
        input.write_all(bytes).unwrap();
    }
    drop(input);
    assert!(
        child.wait().unwrap().success(),
        "driver clock/pose contract"
    );
}
