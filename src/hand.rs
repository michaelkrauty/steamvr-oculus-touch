// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! Converts one engine controller snapshot into the published shared-memory state: connection and
//! standby detection, position-hold window, per-hand button mapping, capacitive touch hysteresis
//! and device→host time mapping.

use crate::clock_map::ClockMap;
use crate::ffi;
use crate::shm::{ControllerState, HAND_LEFT, HAND_RIGHT};

pub const FLAG_CALIBRATED: u32 = ffi::CV1T_FLAG_CALIBRATED;
pub const FLAG_CONNECTED: u32 = ffi::CV1T_FLAG_CONNECTED;
pub const FLAG_POSITION: u32 = ffi::CV1T_FLAG_POSITION;
pub const BTN_A: u32 = ffi::CV1T_BTN_A;
pub const BTN_B: u32 = ffi::CV1T_BTN_B;
pub const BTN_X: u32 = ffi::CV1T_BTN_X;
pub const BTN_Y: u32 = ffi::CV1T_BTN_Y;
pub const BTN_SYSTEM: u32 = ffi::CV1T_BTN_SYSTEM;
pub const BTN_STICK: u32 = ffi::CV1T_BTN_STICK;
pub const TOUCH_A: u32 = ffi::CV1T_TOUCH_A;
pub const TOUCH_B: u32 = ffi::CV1T_TOUCH_B;
pub const TOUCH_X: u32 = ffi::CV1T_TOUCH_X;
pub const TOUCH_Y: u32 = ffi::CV1T_TOUCH_Y;
pub const TOUCH_TRIGGER: u32 = ffi::CV1T_TOUCH_TRIGGER;
pub const TOUCH_STICK: u32 = ffi::CV1T_TOUCH_STICK;
pub const TOUCH_THUMBREST: u32 = ffi::CV1T_TOUCH_THUMBREST;
pub const CAP_STICK: usize = ffi::CV1T_CAP_STICK as usize;
pub const CAP_BY: usize = ffi::CV1T_CAP_BY as usize;
pub const CAP_TRIGGER: usize = ffi::CV1T_CAP_TRIGGER as usize;
pub const CAP_AX: usize = ffi::CV1T_CAP_AX as usize;
pub const CAP_THUMBREST: usize = ffi::CV1T_CAP_THUMBREST as usize;

/// Battery reports in a radio report from the headset (64 bytes, id 0x0c, two 28-byte messages):
/// (hand, percent) for each Touch message whose multiplexed ADC channel is the battery (0x21).
pub fn battery_from_radio(d: &[u8]) -> Vec<(usize, i32)> {
    if d.len() != 64 || d[0] != 0x0c {
        return Vec::new();
    }
    (0..2)
        .filter_map(|i| {
            let m = &d[3 + 28 * i..3 + 28 * (i + 1)];
            let hand = match m[2] {
                2 => HAND_LEFT,
                3 => HAND_RIGHT,
                _ => return None,
            };
            (u16::from_le_bytes([m[0], m[1]]) == 0x1c && m[25] == 0x21)
                .then(|| (hand, u16::from_le_bytes([m[26], m[27]]).min(100) as i32))
        })
        .collect()
}

#[derive(Debug, Clone, Copy)]
pub struct HandConfig {
    /// No new IMU sample for this long means standby / out of range.
    pub connect_timeout_ns: u64,
    /// Position stays valid this long after the last optical fix (engine POSE_LOST_THRESHOLD).
    pub position_hold_ns: u64,
    /// Capacitive hysteresis thresholds on the normalized values.
    pub touch_on: f32,
    pub touch_off: f32,
}

impl Default for HandConfig {
    fn default() -> Self {
        Self {
            connect_timeout_ns: 100_000_000,
            position_hold_ns: 500_000_000,
            touch_on: 0.6,
            touch_off: 0.4,
        }
    }
}

pub struct Hand {
    hand: usize,
    cfg: HandConfig,
    clock: ClockMap,
    last_device_ns: u64,
    last_advance_host_ns: u64,
    touches: u32,
}

fn hysteresis(cur: u32, bit: u32, value: f32, on: f32, off: f32) -> u32 {
    if value >= on {
        cur | bit
    } else if value <= off {
        cur & !bit
    } else {
        cur
    }
}

impl Hand {
    pub fn new(hand: usize, cfg: HandConfig) -> Self {
        assert!(hand == HAND_LEFT || hand == HAND_RIGHT, "hand index");
        Self {
            hand,
            cfg,
            clock: ClockMap::default(),
            last_device_ns: 0,
            last_advance_host_ns: 0,
            touches: 0,
        }
    }

    fn map_buttons(&self, b: u8) -> u32 {
        let right = self.hand == HAND_RIGHT;
        let mut out = 0;
        if b & 0x1 != 0 {
            out |= if right { BTN_A } else { BTN_X };
        }
        if b & 0x2 != 0 {
            out |= if right { BTN_B } else { BTN_Y };
        }
        if b & 0x4 != 0 {
            out |= BTN_SYSTEM;
        }
        if b & 0x8 != 0 {
            out |= BTN_STICK;
        }
        out
    }

    fn update_touches(&mut self, cap: &[f32; 5]) {
        let right = self.hand == HAND_RIGHT;
        let (on, off) = (self.cfg.touch_on, self.cfg.touch_off);
        let mut t = self.touches;
        t = hysteresis(
            t,
            if right { TOUCH_A } else { TOUCH_X },
            cap[CAP_AX],
            on,
            off,
        );
        t = hysteresis(
            t,
            if right { TOUCH_B } else { TOUCH_Y },
            cap[CAP_BY],
            on,
            off,
        );
        t = hysteresis(t, TOUCH_TRIGGER, cap[CAP_TRIGGER], on, off);
        t = hysteresis(t, TOUCH_STICK, cap[CAP_STICK], on, off);
        t = hysteresis(t, TOUCH_THUMBREST, cap[CAP_THUMBREST], on, off);
        self.touches = t;
    }

    pub fn update(&mut self, c: &ffi::ohmd_cv1touch_controller, now_ns: u64) -> ControllerState {
        let mut out = ControllerState {
            publish_host_ns: now_ns,
            battery_percent: -1,
            ..Default::default()
        };
        if !c.calibrated {
            return out;
        }
        out.flags |= FLAG_CALIBRATED;

        if c.device_time_ns != self.last_device_ns {
            self.last_device_ns = c.device_time_ns;
            self.last_advance_host_ns = c.received_host_ns;
            self.clock.observe(c.device_time_ns, c.received_host_ns);
        }
        let connected = self.last_advance_host_ns != 0
            && now_ns.saturating_sub(self.last_advance_host_ns) < self.cfg.connect_timeout_ns;
        if connected {
            out.flags |= FLAG_CONNECTED;
            if c.last_pose_fix_device_ns != 0
                && c.device_time_ns.saturating_sub(c.last_pose_fix_device_ns)
                    < self.cfg.position_hold_ns
            {
                out.flags |= FLAG_POSITION;
            }
        }

        out.device_time_ns = c.device_time_ns;
        out.received_host_ns = c.received_host_ns;
        out.sample_host_ns = self.clock.to_host(c.device_time_ns).unwrap_or(0);
        out.last_position_fix_host_ns = self.clock.to_host(c.last_pose_fix_device_ns).unwrap_or(0);
        out.orient = c.view_orient;
        out.pos = c.view_pos;
        out.raw_orient = c.raw_orient;
        out.raw_pos = c.raw_pos;
        out.lin_vel = c.view_lin_vel;
        out.ang_vel = c.view_ang_vel;
        out.lin_acc = c.view_lin_acc;
        out.gyro_raw = c.gyro_raw;
        out.gyro_bias = c.gyro_bias;
        out.imu_accel = c.imu_accel;
        out.imu_gyro = c.imu_gyro;
        out.pos_stddev = c.pos_stddev;
        out.rot_stddev = c.rot_stddev;
        self.update_touches(&c.cap);
        // The engine keeps the last report after a dropout; publish released inputs instead.
        if connected {
            out.trigger = c.trigger;
            out.grip = c.grip;
            out.stick = c.stick;
            out.cap = c.cap;
            out.buttons = self.map_buttons(c.buttons);
            out.touches = self.touches;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input() -> ffi::ohmd_cv1touch_controller {
        ffi::ohmd_cv1touch_controller {
            calibrated: true,
            ..Default::default()
        }
    }

    #[test]
    fn uncalibrated() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let out = h.update(&ffi::ohmd_cv1touch_controller::default(), 1000);
        assert_eq!(out.flags, 0);
        assert_eq!(out.battery_percent, -1);
        assert_eq!(out.publish_host_ns, 1000);
    }

    /// Review Focus 1: standby must not be published as a live, positioned controller.
    #[test]
    fn standby_and_wake() {
        let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut c = input();
        let (mut host, mut dev) = (1_000_000_000_000u64, 50_000_000u64);
        let mut out = ControllerState::default();
        for _ in 0..50 {
            dev += 2_000_000;
            host += 2_000_000;
            c.device_time_ns = dev;
            c.received_host_ns = host;
            c.last_pose_fix_device_ns = dev;
            c.cap[CAP_THUMBREST] = 1.0;
            out = h.update(&c, host);
        }
        assert!(out.flags & FLAG_CONNECTED != 0 && out.flags & FLAG_POSITION != 0);
        assert!(out.touches & TOUCH_THUMBREST != 0);

        // Samples stop (standby); the engine keeps returning the frozen pose.
        let out = h.update(&c, host + 150_000_000);
        assert!(out.flags & FLAG_CALIBRATED != 0);
        assert_eq!(out.flags & (FLAG_CONNECTED | FLAG_POSITION), 0);
        assert_eq!(out.touches, 0);

        // Wake: samples resume; position needs a new optical fix.
        dev += 30_000_000_000;
        host += 30_000_000_000;
        c.device_time_ns = dev;
        c.received_host_ns = host;
        let out = h.update(&c, host);
        assert!(out.flags & FLAG_CONNECTED != 0);
        assert_eq!(out.flags & FLAG_POSITION, 0);
    }

    /// A controller that drops out (standby, out of radio range) must not keep publishing held
    /// inputs: a trigger held at the moment of disconnect would otherwise stay held forever.
    #[test]
    fn inputs_released_when_disconnected() {
        let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 10_000_000;
        c.received_host_ns = 1_000_000_000;
        c.buttons = 0x1;
        c.trigger = 0.9;
        c.grip = 0.8;
        c.stick = [0.5, -0.5];
        c.cap = [1.0; 5];
        let live = h.update(&c, 1_000_000_000);
        assert!(live.flags & FLAG_CONNECTED != 0 && live.buttons != 0 && live.trigger > 0.5);
        let gone = h.update(&c, 1_000_000_000 + 150_000_000);
        assert_eq!(gone.flags & FLAG_CONNECTED, 0);
        assert_eq!((gone.buttons, gone.touches), (0, 0));
        assert_eq!(
            (gone.trigger, gone.grip, gone.stick, gone.cap),
            (0.0, 0.0, [0.0; 2], [0.0; 5])
        );
    }

    #[test]
    fn publishes_the_imu_sample() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 5_000_000;
        c.received_host_ns = 1_000_000_000;
        c.imu_accel = [0.1, 9.8, -0.2];
        c.imu_gyro = [0.01, 0.0, -0.02];
        let out = h.update(&c, 1_000_000_000);
        assert_eq!(
            (out.imu_accel, out.imu_gyro),
            ([0.1, 9.8, -0.2], [0.01, 0.0, -0.02])
        );
    }

    #[test]
    fn position_hold_window() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 2_000_000_000;
        c.received_host_ns = 9_000_000_000;
        c.last_pose_fix_device_ns = c.device_time_ns - 400_000_000;
        assert!(h.update(&c, 9_000_000_000).flags & FLAG_POSITION != 0);
        c.device_time_ns += 2_000_000;
        c.received_host_ns += 2_000_000;
        c.last_pose_fix_device_ns = c.device_time_ns - 600_000_000;
        let out = h.update(&c, c.received_host_ns);
        assert_eq!(out.flags & FLAG_POSITION, 0);
        assert!(out.flags & FLAG_CONNECTED != 0);
    }

    #[test]
    fn buttons_per_hand() {
        let mut r = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut l = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 1;
        c.received_host_ns = 1;
        c.buttons = 0x1 | 0x4;
        assert_eq!(r.update(&c, 1).buttons, BTN_A | BTN_SYSTEM);
        c.buttons = 0x2 | 0x8;
        assert_eq!(l.update(&c, 1).buttons, BTN_Y | BTN_STICK);
    }

    #[test]
    fn touch_hysteresis() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 1;
        c.received_host_ns = 1;
        for (value, expect) in [(0.5f32, false), (0.65, true), (0.5, true), (0.39, false)] {
            c.device_time_ns += 2_000_000;
            c.received_host_ns += 2_000_000;
            c.cap[CAP_AX] = value;
            let out = h.update(&c, c.received_host_ns);
            assert_eq!(out.touches & TOUCH_X != 0, expect, "cap {value}");
        }
    }

    #[test]
    fn battery_comes_from_adc_channel_0x21() {
        let mut d = [0u8; 64];
        d[0] = 0x0c;
        // message 0: left Touch, channel 0x21 = 87
        d[3] = 0x1c;
        d[3 + 2] = 2;
        d[3 + 25] = 0x21;
        d[3 + 26] = 87;
        // message 1: right Touch, a capacitive channel
        d[31] = 0x1c;
        d[31 + 2] = 3;
        d[31 + 25] = 0x08;
        d[31 + 26] = 50;
        assert_eq!(battery_from_radio(&d), vec![(HAND_LEFT, 87)]);
        d[31 + 25] = 0x21;
        assert_eq!(
            battery_from_radio(&d),
            vec![(HAND_LEFT, 87), (HAND_RIGHT, 50)]
        );
        d[0] = 0x0b;
        assert!(battery_from_radio(&d).is_empty());
    }

    #[test]
    fn sample_time_mapping() {
        let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 5_000_000_000;
        c.received_host_ns = 6_000_000_000;
        c.last_pose_fix_device_ns = 4_990_000_000;
        let out = h.update(&c, 6_000_000_000);
        assert_eq!(out.sample_host_ns, 6_000_000_000);
        assert_eq!(out.last_position_fix_host_ns, 5_990_000_000);
        assert_eq!(
            (out.device_time_ns, out.received_host_ns),
            (5_000_000_000, 6_000_000_000)
        );
    }
}
