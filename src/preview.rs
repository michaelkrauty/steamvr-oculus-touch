// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The sensors' latest images and blobs for a client that shows them (control `preview`). It
//! copies frames only while a client keeps asking: each `preview` holds a lease for
//! `PREVIEW_LEASE_NS`, `preview stop` ends it. Without a lease, a sensor's frame costs one relaxed
//! atomic load. Sensor threads never wait here: they skip a frame when the lock is taken.
//!
//! Images are max-pooled (each pixel the brightest of its block), so that an LED of a pixel or two
//! still shows, and sent zlib-compressed as base64 (`decode_pixels_b64`).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, MutexGuard, PoisonError, TryLockError};

use crate::engine::{Blob, FrameInfo};

/// How long one `preview` keeps frames coming.
pub const PREVIEW_LEASE_NS: u64 = 3_000_000_000;
/// Least time between two frames kept for one sensor: about 15 a second for every sensor, about
/// 30 for one sensor asked for by its serial.
const ALL_INTERVAL_NS: u64 = 60_000_000;
const ONE_INTERVAL_NS: u64 = 33_000_000;
/// A sensor with no frame for this long is dropped (unplugged).
const FORGET_NS: u64 = 10_000_000_000;
const MAX_SENSORS: usize = 8;
/// A pixel this bright or brighter counts in `bright_permille`: LEDs, and sunlight or lamps that
/// look like them.
pub const BRIGHT: u8 = 180;

#[derive(Default)]
pub struct PreviewHub {
    /// The lease ends then (`now_ns` clock); 0: no lease, or one found ended.
    lease_until_ns: AtomicU64,
    state: Mutex<State>,
}

#[derive(Default)]
struct State {
    /// 2 or 4 as asked for; 0 before any request.
    scale: u8,
    /// Only this sensor is kept (`status::same_sensor`).
    serial: Option<String>,
    sensors: Vec<Sensor>,
}

#[derive(Clone, Default)]
struct Sensor {
    serial: String,
    /// Exposure of the image and of the blobs, or of the blobs alone without an image.
    frame_ns: u64,
    /// Exposure of the last frame taken, which may still be pooling.
    taken_ns: u64,
    seen_ns: u64,
    raw_width: u32,
    raw_height: u32,
    width: u32,
    height: u32,
    /// GRAY8, `width * height`; empty without an image (as from JPEG video).
    pixels: Vec<u8>,
    /// The buffer the next image is pooled into, kept to be reused.
    spare: Vec<u8>,
    max_pixel: u8,
    bright_permille: u32,
    blobs: Vec<PreviewBlob>,
    devices: Vec<PreviewDevice>,
}

/// A blob, in the full image's pixels.
#[derive(Clone, Copy)]
struct PreviewBlob {
    x: f32,
    y: f32,
    width: u16,
    height: u16,
    /// `Blob::led_id`: OpenHMD device id << 8 | LED index, negative when unlabelled.
    led_id: i32,
}

#[derive(Clone, Copy)]
struct PreviewDevice {
    device_id: i32,
    matched_blobs: i32,
    visible_leds: i32,
    pose_valid: bool,
    match_flags: u32,
}

fn same_serial(serial: &str, field: &[std::ffi::c_char]) -> bool {
    let n = field.iter().position(|&c| c == 0).unwrap_or(field.len());
    n == serial.len()
        && serial
            .bytes()
            .zip(field)
            .all(|(a, &b)| a.eq_ignore_ascii_case(&(b as u8)))
}

impl PreviewHub {
    /// A lease was taken and not yet found ended: the one load a frame costs otherwise.
    #[inline]
    pub fn leased(&self) -> bool {
        self.lease_until_ns.load(Ordering::Relaxed) != 0
    }

    pub fn active(&self, now_ns: u64) -> bool {
        now_ns < self.lease_until_ns.load(Ordering::Relaxed)
    }

    /// When the lease ends, if it has yet to.
    pub fn lease_until(&self, now_ns: u64) -> Option<u64> {
        let until = self.lease_until_ns.load(Ordering::Relaxed);
        (until > now_ns).then_some(until)
    }

    fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn try_lock(&self) -> Option<MutexGuard<'_, State>> {
        match self.state.try_lock() {
            Ok(state) => Some(state),
            Err(TryLockError::Poisoned(e)) => Some(e.into_inner()),
            Err(TryLockError::WouldBlock) => None,
        }
    }

    /// Whether frames are wanted now. An ended lease, once found, frees the frames kept.
    fn admit(&self, now_ns: u64) -> bool {
        let until = self.lease_until_ns.load(Ordering::Relaxed);
        if until == 0 {
            return false;
        }
        if now_ns < until {
            return true;
        }
        // Renewals store under the lock, so a lease renewed meanwhile stays.
        if let Some(mut state) = self.try_lock() {
            if self
                .lease_until_ns
                .compare_exchange(until, 0, Ordering::Relaxed, Ordering::Relaxed)
                .is_ok()
            {
                state.sensors = Vec::new();
            }
        }
        false
    }

    /// Ends the lease and frees the frames kept.
    pub fn stop(&self) {
        let mut state = self.lock();
        self.lease_until_ns.store(0, Ordering::Relaxed);
        state.sensors = Vec::new();
    }

    /// A sensor's image, which `on_frame` for the same frame follows. It is pooled outside the
    /// lock, so that sensors exposed together do not skip each other's frames.
    pub fn on_image(
        &self,
        info: &FrameInfo,
        width: usize,
        height: usize,
        stride: usize,
        pixels: &[u8],
        now_ns: u64,
    ) {
        if !self.admit(now_ns) {
            return;
        }
        let at = info.frame_start_host_ns;
        let (scale, mut buffer) = {
            let Some(mut state) = self.try_lock() else {
                return;
            };
            let scale = state.scale();
            let Some(sensor) = state.take(info, now_ns) else {
                return;
            };
            sensor.taken_ns = at;
            (scale, std::mem::take(&mut sensor.spare))
        };
        let Some((pooled_width, pooled_height, bright)) =
            max_pool(pixels, width, height, stride, scale, &mut buffer)
        else {
            return;
        };
        let Some(mut state) = self.try_lock() else {
            return;
        };
        let Some(sensor) = state.sensor(&info.serial) else {
            return;
        };
        sensor.spare = std::mem::replace(&mut sensor.pixels, buffer);
        sensor.frame_ns = at;
        sensor.raw_width = width as u32;
        sensor.raw_height = height as u32;
        sensor.width = pooled_width as u32;
        sensor.height = pooled_height as u32;
        sensor.max_pixel = sensor.pixels.iter().copied().max().unwrap_or(0);
        let pooled = (pooled_width * pooled_height * scale * scale) as u64;
        sensor.bright_permille = (u64::from(bright) * 1000 / pooled) as u32;
        sensor.blobs.clear();
        sensor.devices.clear();
    }

    /// A sensor's blobs and its match of each device: those of the image kept, or the newest
    /// from a sensor that gives no image.
    pub fn on_frame(&self, info: &FrameInfo, blobs: &[Blob], now_ns: u64) {
        if !self.admit(now_ns) {
            return;
        }
        let Some(mut state) = self.try_lock() else {
            return;
        };
        let at = info.frame_start_host_ns;
        let sensor = match state
            .sensors
            .iter()
            .position(|s| same_serial(&s.serial, &info.serial))
        {
            Some(i) if state.sensors[i].frame_ns == at => &mut state.sensors[i],
            Some(i) if !state.sensors[i].pixels.is_empty() => return,
            _ => {
                let Some(sensor) = state.take(info, now_ns) else {
                    return;
                };
                sensor.taken_ns = at;
                sensor.frame_ns = at;
                sensor
            }
        };
        sensor.blobs.clear();
        sensor.blobs.extend(blobs.iter().map(|b| PreviewBlob {
            x: b.x,
            y: b.y,
            width: b.width,
            height: b.height,
            led_id: b.led_id,
        }));
        let n = (info.n_devices as usize).min(info.devices.len());
        sensor.devices.clear();
        sensor
            .devices
            .extend(info.devices[..n].iter().map(|d| PreviewDevice {
                device_id: d.device_id,
                matched_blobs: d.matched_blobs,
                visible_leds: d.visible_leds,
                pose_valid: d.pose_valid != 0,
                match_flags: d.match_flags,
            }));
    }

    /// The reply to `preview`: renews the lease, keeps images pooled by `scale` (2 or 4) and only
    /// `serial`'s from now on, and gives what is kept, as JSON.
    pub fn snapshot_json(&self, serial: Option<&str>, scale: u8, now_ns: u64) -> String {
        let sensors: Vec<Sensor> = {
            let mut state = self.lock();
            self.lease_until_ns
                .store(now_ns + PREVIEW_LEASE_NS, Ordering::Relaxed);
            state.scale = scale;
            state.serial = serial.map(str::to_owned);
            state
                .sensors
                .retain(|s| now_ns.saturating_sub(s.seen_ns) < FORGET_NS);
            state
                .sensors
                .iter()
                .filter(|s| serial.is_none_or(|w| crate::status::same_sensor(w, &s.serial)))
                .map(|s| Sensor {
                    spare: Vec::new(),
                    ..s.clone()
                })
                .collect()
        };
        let sensors: Vec<serde_json::Value> = sensors
            .iter()
            .map(|s| {
                let zlib = miniz_oxide::deflate::compress_to_vec_zlib(&s.pixels, 1);
                let blobs: Vec<_> = s
                    .blobs
                    .iter()
                    .map(|b| serde_json::json!([b.x, b.y, b.width, b.height, b.led_id]))
                    .collect();
                let devices: Vec<_> = s
                    .devices
                    .iter()
                    .map(|d| {
                        serde_json::json!({
                            "device_id": d.device_id,
                            "matched_blobs": d.matched_blobs,
                            "visible_leds": d.visible_leds,
                            "pose_valid": d.pose_valid,
                            "match_flags": d.match_flags,
                        })
                    })
                    .collect();
                serde_json::json!({
                    "serial": s.serial,
                    "frame_ns": s.frame_ns,
                    "age_ms": now_ns.saturating_sub(s.frame_ns) / 1_000_000,
                    "raw_width": s.raw_width,
                    "raw_height": s.raw_height,
                    "width": s.width,
                    "height": s.height,
                    "max_pixel": s.max_pixel,
                    "bright_permille": s.bright_permille,
                    "encoding": "zlib-gray8-b64",
                    "pixels_b64": base64(&zlib),
                    "blobs": blobs,
                    "devices": devices,
                })
            })
            .collect();
        serde_json::json!({
            "active": true,
            "lease_ms": PREVIEW_LEASE_NS / 1_000_000,
            "sensors": sensors,
        })
        .to_string()
    }
}

impl State {
    fn scale(&self) -> usize {
        if self.scale == 2 {
            2
        } else {
            4
        }
    }

    fn sensor(&mut self, serial: &[std::ffi::c_char]) -> Option<&mut Sensor> {
        self.sensors
            .iter_mut()
            .find(|s| same_serial(&s.serial, serial))
    }

    /// The sensor's entry, if its frame exposed at `info.frame_start_host_ns` is to be taken.
    fn take(&mut self, info: &FrameInfo, now_ns: u64) -> Option<&mut Sensor> {
        if self
            .serial
            .as_deref()
            .is_some_and(|w| !same_serial(w, &info.serial))
        {
            return None;
        }
        let interval = if self.serial.is_some() {
            ONE_INTERVAL_NS
        } else {
            ALL_INTERVAL_NS
        };
        let i = match self
            .sensors
            .iter()
            .position(|s| same_serial(&s.serial, &info.serial))
        {
            Some(i) => i,
            None => {
                if self.sensors.len() >= MAX_SENSORS {
                    let oldest = (0..self.sensors.len()).min_by_key(|&i| self.sensors[i].seen_ns);
                    self.sensors.swap_remove(oldest?);
                }
                self.sensors.push(Sensor {
                    serial: crate::engine::cstr_field(&info.serial),
                    ..Default::default()
                });
                self.sensors.len() - 1
            }
        };
        let sensor = &mut self.sensors[i];
        sensor.seen_ns = now_ns;
        let at = info.frame_start_host_ns;
        (sensor.taken_ns == 0 || at >= sensor.taken_ns.saturating_add(interval)).then_some(sensor)
    }
}

/// `pixels` (GRAY8, `stride` bytes a row) max-pooled by `scale` (2 or 4) into `out`: each output
/// pixel is the brightest of its `scale` x `scale` block, so that no LED is lost. Returns the size
/// pooled to and how many of the input pixels pooled are at least `BRIGHT`; None for a frame
/// smaller than a block or a buffer too short for its size.
fn max_pool(
    pixels: &[u8],
    width: usize,
    height: usize,
    stride: usize,
    scale: usize,
    out: &mut Vec<u8>,
) -> Option<(usize, usize, u32)> {
    let (out_width, out_height) = (width / scale, height / scale);
    let needed = stride
        .checked_mul(height.checked_sub(1)?)?
        .checked_add(width)?;
    if out_width == 0 || out_height == 0 || stride < width || pixels.len() < needed {
        return None;
    }
    out.clear();
    out.resize(out_width * out_height, 0);
    let bright = match scale {
        2 => pool::<2>(pixels, stride, out_width, out),
        4 => pool::<4>(pixels, stride, out_width, out),
        _ => return None,
    };
    Some((out_width, out_height, bright))
}

fn pool<const S: usize>(pixels: &[u8], stride: usize, out_width: usize, out: &mut [u8]) -> u32 {
    let width = out_width * S;
    let mut bright = 0;
    // each column's brightest of the block's rows, then of each block's columns: both vectorize
    let mut column = vec![0; width];
    for (y, dst) in out.chunks_exact_mut(out_width).enumerate() {
        column.fill(0);
        for r in 0..S {
            let at = (y * S + r) * stride;
            let row = &pixels[at..at + width];
            bright += row.iter().filter(|&&p| p >= BRIGHT).count() as u32;
            for (m, &p) in column.iter_mut().zip(row) {
                *m = (*m).max(p);
            }
        }
        for (d, block) in dst.iter_mut().zip(column.chunks_exact(S)) {
            *d = block.iter().fold(0, |m, &p| m.max(p));
        }
    }
    bright
}

const BASE64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// Standard base64, padded.
fn base64(data: &[u8]) -> String {
    let mut s = String::with_capacity(data.len().div_ceil(3) * 4);
    for chunk in data.chunks(3) {
        let n = chunk
            .iter()
            .enumerate()
            .fold(0u32, |n, (i, &b)| n | (u32::from(b) << (16 - 8 * i)));
        for k in 0..4 {
            s.push(if k <= chunk.len() {
                BASE64[((n >> (18 - 6 * k)) & 63) as usize] as char
            } else {
                '='
            });
        }
    }
    s
}

fn unbase64(s: &str) -> Option<Vec<u8>> {
    let s = s.as_bytes();
    if s.len() % 4 != 0 {
        return None;
    }
    let mut out = Vec::with_capacity(s.len() / 4 * 3);
    for (i, quad) in s.chunks(4).enumerate() {
        let pad = quad.iter().rev().take_while(|&&c| c == b'=').count();
        if pad > 2 || (pad > 0 && i + 1 < s.len() / 4) {
            return None;
        }
        let mut n = 0u32;
        for &c in &quad[..4 - pad] {
            n = (n << 6) | BASE64.iter().position(|&b| b == c)? as u32;
        }
        n <<= 6 * pad;
        out.extend_from_slice(&n.to_be_bytes()[1..4 - pad]);
    }
    Some(out)
}

/// A sensor's `pixels_b64`: exactly `expected_len` (`width * height`) pixels, or None. It never
/// inflates more than that.
pub fn decode_pixels_b64(b64: &str, expected_len: usize) -> Option<Vec<u8>> {
    let zlib = unbase64(b64)?;
    let pixels =
        miniz_oxide::inflate::decompress_to_vec_zlib_with_limit(&zlib, expected_len).ok()?;
    (pixels.len() == expected_len).then_some(pixels)
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    fn info(serial: &str, at: u64) -> FrameInfo {
        let mut info = FrameInfo {
            frame_start_host_ns: at,
            n_devices: 1,
            ..Default::default()
        };
        for (d, b) in info.serial.iter_mut().zip(serial.bytes()) {
            *d = b as std::ffi::c_char;
        }
        info.devices[0].device_id = 1;
        info.devices[0].matched_blobs = 5;
        info.devices[0].visible_leds = 7;
        info.devices[0].pose_valid = 1;
        info.devices[0].match_flags = 3;
        info
    }

    fn blob(x: f32, led_id: i32) -> Blob {
        Blob {
            x,
            y: 20.5,
            width: 3,
            height: 2,
            area: 5,
            led_id,
        }
    }

    fn frame(w: usize, h: usize) -> Vec<u8> {
        vec![10; w * h]
    }

    fn snapshot(hub: &PreviewHub, serial: Option<&str>, scale: u8, now: u64) -> serde_json::Value {
        serde_json::from_str(&hub.snapshot_json(serial, scale, now)).unwrap()
    }

    fn pixels(sensor: &serde_json::Value) -> Vec<u8> {
        let len = sensor["width"].as_u64().unwrap() * sensor["height"].as_u64().unwrap();
        decode_pixels_b64(sensor["pixels_b64"].as_str().unwrap(), len as usize).unwrap()
    }

    #[test]
    fn a_lease_starts_with_a_request_lasts_three_seconds_and_ends_on_stop() {
        let hub = PreviewHub::default();
        assert!(!hub.leased() && !hub.active(0) && hub.lease_until(0).is_none());
        let t = 1000 * MS;
        hub.snapshot_json(None, 4, t);
        assert!(hub.active(t + PREVIEW_LEASE_NS - 1));
        assert!(!hub.active(t + PREVIEW_LEASE_NS));
        assert_eq!(hub.lease_until(t), Some(t + PREVIEW_LEASE_NS));
        assert_eq!(hub.lease_until(t + PREVIEW_LEASE_NS), None);
        hub.snapshot_json(None, 4, t + 2000 * MS);
        assert!(hub.active(t + PREVIEW_LEASE_NS), "renewed");
        hub.on_image(&info("S1", t), 8, 8, 8, &frame(8, 8), t + 2001 * MS);
        assert_eq!(hub.lock().sensors.len(), 1);
        hub.stop();
        assert!(!hub.leased() && !hub.active(t + 2001 * MS));
        assert!(hub.lock().sensors.is_empty(), "stop frees the frames");
    }

    #[test]
    fn without_a_lease_frames_are_not_kept_and_an_ended_lease_frees_them() {
        let hub = PreviewHub::default();
        hub.on_image(&info("S1", 5), 8, 8, 8, &frame(8, 8), 5);
        hub.on_frame(&info("S1", 5), &[blob(1.0, 0x103)], 5);
        assert_eq!(hub.lock().sensors.capacity(), 0, "nothing allocated");
        let t = 1000 * MS;
        hub.snapshot_json(None, 4, t);
        hub.on_image(&info("S1", t), 8, 8, 8, &frame(8, 8), t);
        assert_eq!(hub.lock().sensors.len(), 1);
        let end = t + PREVIEW_LEASE_NS;
        hub.on_image(&info("S1", end), 8, 8, 8, &frame(8, 8), end);
        assert!(!hub.leased(), "found ended: back to one load a frame");
        assert_eq!(hub.lock().sensors.capacity(), 0);
    }

    #[test]
    fn max_pooling_keeps_a_single_bright_pixel_that_subsampling_drops() {
        let (w, h) = (1280, 960);
        // At scale 2, any 2 x 2 LED covers a block's top-left pixel.
        for (scale, size) in [(4, 1), (4, 2), (2, 1)] {
            let mut px = frame(w, h);
            // inside the block, never on its top-left pixel
            let (x, y) = (40 * scale + scale - size, 100 * scale + scale - size);
            for dy in 0..size {
                for dx in 0..size {
                    px[(y + dy) * w + x + dx] = 255;
                }
            }
            let mut out = Vec::new();
            let (ow, oh, bright) = max_pool(&px, w, h, w, scale, &mut out).unwrap();
            assert_eq!((ow, oh), (w / scale, h / scale));
            assert_eq!(px[100 * scale * w + 40 * scale], 10, "top-left is dark");
            assert_eq!(out[100 * ow + 40], 255, "scale {scale}, {size} px LED");
            assert_eq!(out.iter().filter(|&&p| p == 255).count(), 1);
            assert_eq!(bright, (size * size) as u32);
        }
    }

    #[test]
    fn pooling_reads_rows_by_stride_and_refuses_short_or_tiny_frames() {
        let (w, h, stride) = (8, 4, 12);
        let mut px = vec![0u8; stride * (h - 1) + w];
        px[3 * stride + 7] = 200;
        px[2 * stride + 9] = 255; // row padding, not a pixel
        let mut out = Vec::new();
        assert_eq!(max_pool(&px, w, h, stride, 4, &mut out), Some((2, 1, 1)));
        assert_eq!(out, [0, 200]);
        assert_eq!(
            max_pool(&px[..px.len() - 1], w, h, stride, 4, &mut out),
            None
        );
        assert_eq!(max_pool(&px, 3, 3, 3, 4, &mut out), None);
        assert_eq!(max_pool(&px, w, h, w - 1, 4, &mut out), None);
        assert_eq!(max_pool(&[], 0, 0, 0, 4, &mut out), None);
    }

    #[test]
    fn a_snapshot_gives_the_image_its_statistics_and_the_same_frames_blobs() {
        let hub = PreviewHub::default();
        let t = 1000 * MS;
        hub.snapshot_json(None, 4, t);
        let (w, h) = (1280, 960);
        let mut px = frame(w, h);
        px[..w * 12].fill(BRIGHT); // 1.25 % of the frame
        px[w * 500 + 600] = 254;
        hub.on_image(&info("S1", t), w, h, w, &px, t + MS);
        hub.on_frame(
            &info("S1", t),
            &[blob(600.25, 0x103), blob(9.0, -1)],
            t + MS,
        );
        let v = snapshot(&hub, None, 4, t + 15 * MS);
        assert_eq!(v["active"], true);
        assert_eq!(v["lease_ms"], 3000);
        let s = &v["sensors"][0];
        assert_eq!(s["serial"], "S1");
        assert_eq!(s["frame_ns"], t);
        assert_eq!(s["age_ms"], 15);
        assert_eq!([&s["raw_width"], &s["raw_height"]], [1280, 960]);
        assert_eq!([&s["width"], &s["height"]], [320, 240]);
        assert_eq!(s["max_pixel"], 254);
        assert_eq!(s["bright_permille"], 12);
        assert_eq!(s["encoding"], "zlib-gray8-b64");
        let out = pixels(s);
        assert_eq!(out[125 * 320 + 150], 254);
        assert_eq!(out[0], BRIGHT);
        assert_eq!(out[3 * 320], 10);
        assert_eq!(
            s["blobs"],
            serde_json::json!([[600.25, 20.5, 3, 2, 0x103], [9.0, 20.5, 3, 2, -1]])
        );
        assert_eq!(
            s["devices"],
            serde_json::json!([{
                "device_id": 1, "matched_blobs": 5, "visible_leds": 7, "pose_valid": true,
                "match_flags": 3
            }])
        );
    }

    #[test]
    fn blobs_of_a_frame_whose_image_was_not_kept_are_not_drawn_on_it() {
        let hub = PreviewHub::default();
        let t = 1000 * MS;
        hub.snapshot_json(None, 4, t);
        hub.on_image(&info("S1", t), 8, 8, 8, &frame(8, 8), t);
        hub.on_frame(&info("S1", t), &[blob(1.0, 0x103)], t);
        // the next frame, within the interval: neither its image nor its blobs are taken
        let next = t + 17 * MS;
        hub.on_image(&info("S1", next), 8, 8, 8, &[99; 64], next);
        hub.on_frame(&info("S1", next), &[blob(2.0, 0x104)], next);
        let v = snapshot(&hub, None, 4, next);
        assert_eq!(v["sensors"][0]["frame_ns"], t);
        assert_eq!(v["sensors"][0]["blobs"][0][0], 1.0);
        assert_eq!(pixels(&v["sensors"][0]), [10; 4]);
    }

    #[test]
    fn a_sensor_without_images_gives_its_newest_blobs() {
        let hub = PreviewHub::default();
        let t = 1000 * MS;
        hub.snapshot_json(None, 4, t);
        hub.on_frame(&info("S1", t), &[blob(1.0, 0x103)], t);
        let later = t + 60 * MS;
        hub.on_frame(&info("S1", later), &[blob(2.0, 0x203)], later);
        let v = snapshot(&hub, None, 4, later);
        let s = &v["sensors"][0];
        assert_eq!(s["frame_ns"], later);
        assert_eq!(s["width"], 0);
        assert_eq!(s["blobs"][0][0], 2.0);
        assert_eq!(pixels(s), Vec::<u8>::new());
    }

    #[test]
    fn frames_are_taken_15_a_second_for_all_and_30_for_one_sensor_at_its_scale() {
        let hub = PreviewHub::default();
        let t = 1000 * MS;
        let taken = |hub: &PreviewHub, serial: &str, from: u64| {
            (0..60)
                .filter(|&k| {
                    let at = from + k * 16_667_000; // 60 frames a second
                    hub.on_image(&info(serial, at), 16, 16, 16, &frame(16, 16), at);
                    let state = hub.lock();
                    state
                        .sensors
                        .iter()
                        .any(|s| s.serial == serial && s.frame_ns == at)
                })
                .count()
        };
        hub.snapshot_json(None, 4, t);
        assert_eq!(taken(&hub, "S1", t), 15);
        assert_eq!(snapshot(&hub, None, 4, t)["sensors"][0]["width"], 4);
        hub.snapshot_json(Some("s2"), 2, t);
        assert_eq!(taken(&hub, "S2", t), 30);
        assert_eq!(
            taken(&hub, "S1", t + 2000 * MS),
            0,
            "only the sensor asked for"
        );
        let v = snapshot(&hub, Some("s2"), 2, t);
        assert_eq!(v["sensors"].as_array().unwrap().len(), 1);
        assert_eq!(v["sensors"][0]["serial"], "S2");
        assert_eq!(v["sensors"][0]["width"], 8);
    }

    #[test]
    fn sensor_threads_skip_a_frame_rather_than_wait_for_the_lock() {
        let hub = PreviewHub::default();
        let t = 1000 * MS;
        hub.snapshot_json(None, 4, t);
        let held = hub.lock();
        // on the same thread, a blocking lock would never return
        hub.on_image(&info("S1", t), 8, 8, 8, &frame(8, 8), t);
        hub.on_frame(&info("S1", t), &[blob(1.0, 0x103)], t);
        assert!(held.sensors.is_empty());
        drop(held);
        hub.on_image(&info("S1", t), 8, 8, 8, &frame(8, 8), t);
        assert_eq!(hub.lock().sensors.len(), 1);
    }

    #[test]
    fn unplugged_sensors_are_forgotten_and_the_count_is_capped() {
        let hub = PreviewHub::default();
        let t = 1000 * MS;
        hub.snapshot_json(None, 4, t);
        for i in 0..MAX_SENSORS + 2 {
            let at = t + i as u64 * MS;
            hub.on_image(&info(&format!("S{i}"), at), 8, 8, 8, &frame(8, 8), at);
        }
        {
            let state = hub.lock();
            assert_eq!(state.sensors.len(), MAX_SENSORS);
            assert!(!state
                .sensors
                .iter()
                .any(|s| s.serial == "S0" || s.serial == "S1"));
        }
        let later = t + FORGET_NS + 20 * MS;
        hub.snapshot_json(None, 4, later);
        hub.on_image(&info("S9", later), 8, 8, 8, &frame(8, 8), later);
        let v = snapshot(&hub, None, 4, later + 2 * MS);
        assert_eq!(v["sensors"].as_array().unwrap().len(), 1);
    }

    #[test]
    fn pixels_survive_compression_and_base64_and_bad_input_is_refused() {
        let all: Vec<u8> = (0..=255).collect();
        for data in [&[][..], &[0], &[1, 2], &[255, 0, 7], &all] {
            let b64 = base64(&miniz_oxide::deflate::compress_to_vec_zlib(data, 1));
            assert_eq!(decode_pixels_b64(&b64, data.len()).as_deref(), Some(data));
            assert_eq!(decode_pixels_b64(&b64, data.len() + 1), None);
        }
        assert_eq!(base64(b""), "");
        assert_eq!(base64(b"f"), "Zg==");
        assert_eq!(base64(b"fo"), "Zm8=");
        assert_eq!(base64(b"foo"), "Zm9v");
        assert_eq!(base64(b"foobar"), "Zm9vYmFy");
        assert_eq!(unbase64("Zm9vYg==").unwrap(), b"foob");
        assert_eq!(unbase64("Zm9vYmE=").unwrap(), b"fooba");
        for bad in ["Zm9", "Zm=v", "Z===", "Zg==Zg==", "Zm9!"] {
            assert_eq!(unbase64(bad), None, "{bad}");
        }
        // 64 MiB of zeros deflates to a few dozen KiB; it is not inflated past the limit
        let bomb = base64(&miniz_oxide::deflate::compress_to_vec_zlib(
            &vec![0; 64 << 20],
            1,
        ));
        assert_eq!(decode_pixels_b64(&bomb, 320 * 240), None);
        assert_eq!(decode_pixels_b64("not base64", 4), None);
    }
}
