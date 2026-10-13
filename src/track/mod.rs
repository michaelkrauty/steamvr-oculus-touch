// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! A per-controller error-state Kalman filter fusing the IMU with every sensor's LED blobs
//! by reprojection. Poses are nalgebra isometries named `a_from_b`; the world is the
//! calibration's (gravity-aligned, +Y up); camera frames use OpenCV axes.

pub mod biases;
pub mod clock;
pub mod eskf;
pub mod factory;
pub mod landmarks;
pub mod pipeline;
pub mod predict;
pub mod refine;
pub mod replay;
pub mod search;
pub mod tracker;
pub mod vision;
