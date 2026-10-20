// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! cv1touch: Rift CV1 Touch controller tracking daemon library.

#[cfg(test)]
#[path = "../tests/support/socket_dir.rs"]
mod socket_dir;

pub mod align;
pub mod calendar;
pub mod calib;
pub mod clock_map;
pub mod control;
pub mod engine;
pub mod eval;
pub mod ffi;
pub mod flight;
pub mod flight_writer;
pub mod hand;
pub mod headset_watch;
pub mod input_calibration;
mod log_lines;
pub mod modes;
pub mod platform;
pub mod recorder;
pub mod register;
pub mod rendermodel;
pub mod sensor_hotplug;
pub mod settings;
pub mod setup;
pub mod shm;
pub mod status;
pub mod steamvr;
pub mod steamvr_log;
pub mod summary;
pub mod supervise;
pub mod track;
pub mod vr;
pub mod zip;

pub mod frame_queue;
