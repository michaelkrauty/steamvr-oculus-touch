// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

use crate::engine::{ffi_callback, Blob, EngineSink, FrameInfo, InspectionIdentity, RadioReader};
use std::ffi::{c_int, c_void};
use std::sync::Arc;

pub(super) unsafe extern "C" fn frame_cb(
    user: *mut c_void,
    info: *const FrameInfo,
    n: c_int,
    blobs: *const Blob,
) {
    ffi_callback(|| unsafe {
        if info.is_null() {
            return;
        }
        let sink = &*(user as *const Arc<dyn EngineSink>);
        let blobs = if n > 0 && !blobs.is_null() {
            std::slice::from_raw_parts(blobs, n as usize)
        } else {
            &[]
        };
        sink.frame(&*info, blobs);
    });
}

pub(super) unsafe extern "C" fn identity_cb(
    user: *mut c_void,
    event: *const InspectionIdentity,
    host_ns: u64,
) {
    ffi_callback(|| unsafe {
        if event.is_null() {
            return;
        }
        let sink = &*(user as *const Arc<dyn EngineSink>);
        sink.inspection_identity(&*event, host_ns);
    });
}

pub(super) unsafe extern "C" fn search_cb(
    user: *mut c_void,
    info: *const FrameInfo,
    remaining_ns: u64,
) {
    ffi_callback(|| unsafe {
        if info.is_null() {
            return;
        }
        let sink = &*(user as *const Arc<dyn EngineSink>);
        sink.search_started(&*info, remaining_ns);
    });
}

pub(super) unsafe extern "C" fn image_cb(
    user: *mut c_void,
    info: *const FrameInfo,
    width: c_int,
    height: c_int,
    stride: c_int,
    pixels: *const u8,
) {
    ffi_callback(|| unsafe {
        if info.is_null() || pixels.is_null() || width <= 0 || height <= 0 || stride < width {
            return;
        }
        let sink = &*(user as *const Arc<dyn EngineSink>);
        let px = std::slice::from_raw_parts(pixels, stride as usize * height as usize);
        sink.image(&*info, width as usize, height as usize, stride as usize, px);
    });
}

pub(super) unsafe extern "C" fn report_cb(
    user: *mut c_void,
    source: c_int,
    ts: u64,
    data: *const u8,
    len: c_int,
) {
    ffi_callback(|| unsafe {
        let sink = &*(user as *const Arc<dyn EngineSink>);
        let data = if len > 0 && !data.is_null() {
            std::slice::from_raw_parts(data, len as usize)
        } else {
            &[]
        };
        sink.report(source, ts, data);
    });
}

pub(super) unsafe extern "C" fn activity_cb(
    user: *mut c_void,
    reader: c_int,
    device: c_int,
    device_ns: u64,
    host_ns: u64,
    asleep: bool,
    gyro_squared: f32,
) {
    ffi_callback(|| unsafe {
        let reader = match reader {
            0 => RadioReader::Engine,
            1 => RadioReader::Radio,
            _ => return,
        };
        let sink = &*(user as *const Arc<dyn EngineSink>);
        sink.activity(reader, device, device_ns, host_ns, asleep, gyro_squared);
    });
}
