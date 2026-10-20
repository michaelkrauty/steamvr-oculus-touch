// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Converts one engine controller snapshot into the published shared-memory state: connection and
//! standby detection, radio gaps, position-hold window, per-hand button mapping, capacitive touch
//! hysteresis and device→host time mapping.

use crate::clock_map::ClockMap;
use crate::ffi;
use crate::shm::{ControllerState, HAND_LEFT, HAND_RIGHT};

pub const FLAG_CALIBRATED: u32 = ffi::CV1T_FLAG_CALIBRATED;
pub const FLAG_CONNECTED: u32 = ffi::CV1T_FLAG_CONNECTED;
pub const FLAG_POSITION: u32 = ffi::CV1T_FLAG_POSITION;
pub const FLAG_RADIO_GAP: u32 = ffi::CV1T_FLAG_RADIO_GAP;
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

/// Whether a published state is of a controller whose samples are arriving: connected and not in
/// a radio gap.
pub fn reporting(flags: u32) -> bool {
    flags & FLAG_CONNECTED != 0 && flags & FLAG_RADIO_GAP == 0
}

#[derive(Debug, Clone, Copy)]
pub struct HandConfig {
    /// No new IMU sample for this long means the controller's samples stopped.
    pub connect_timeout_ns: u64,
    /// A controller whose samples stop stays connected this long after its last sample, as it
    /// was then, inputs held (a radio gap); past it, it is in standby or out of range.
    pub ride_through_ns: u64,
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
            ride_through_ns: 10_000_000_000,
            position_hold_ns: 500_000_000,
            touch_on: 0.6,
            touch_off: 0.4,
        }
    }
}

/// A controller that turned faster than this within `MOTION_WINDOW_NS` of its last sample was in
/// use when its samples stopped, rad/s: a controller put down has been still for longer.
const MOTION_RAD_S: f32 = 0.5;
const MOTION_WINDOW_NS: u64 = 3_000_000_000;

/// Why a controller published live is held through a gap. A gap keeps the cause it began with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GapCause {
    /// Its samples stopped arriving: a radio gap.
    Radio,
    /// The tracking engine is closing or opening and has no sample to give.
    Reopen,
    /// The thread publishing it published nothing for longer than the connect timeout.
    Stall,
}

/// A change in a controller's gap (see `HandConfig::ride_through_ns`), with what began it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GapEvent {
    /// Its samples stopped: no sample since `last_sample_host_ns`. `moving`: it turned within a
    /// few seconds before.
    Began {
        last_sample_host_ns: u64,
        moving: bool,
        cause: GapCause,
    },
    /// Its samples came back after `silent_ns` without, the last before at `last_sample_host_ns`.
    Ended {
        last_sample_host_ns: u64,
        silent_ns: u64,
        cause: GapCause,
    },
    /// No sample for the whole ride-through: published as in standby from now.
    Expired { silent_ns: u64, cause: GapCause },
}

/// The inputs a controller last reported while its samples arrived, held through a radio gap.
#[derive(Debug, Clone, Copy, Default)]
struct HeldInputs {
    trigger: f32,
    grip: f32,
    stick: [f32; 2],
    cap: [f32; 5],
    buttons: u32,
    touches: u32,
}

pub struct Hand {
    hand: usize,
    cfg: HandConfig,
    clock: ClockMap,
    last_device_ns: u64,
    last_advance_host_ns: u64,
    touches: u32,
    held: HeldInputs,
    /// Published live since its samples last stopped for good: what a radio gap holds.
    seen_live: bool,
    in_gap: bool,
    gap_cause: GapCause,
    last_motion_host_ns: u64,
    event: Option<GapEvent>,
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
            held: HeldInputs::default(),
            seen_live: false,
            in_gap: false,
            gap_cause: GapCause::Radio,
            last_motion_host_ns: 0,
            event: None,
        }
    }

    /// The radio-gap change the last `update` found, if any (taken once).
    pub fn take_event(&mut self) -> Option<GapEvent> {
        self.event.take()
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
        if !c.calibrated {
            // a new engine has no calibrated controller until it reads the controller's setup
            return self.unavailable(now_ns, GapCause::Reopen);
        }
        if c.received_host_ns < self.last_advance_host_ns {
            if c.received_host_ns == 0 {
                // a new engine's controller before its first sample
                return self.unavailable(now_ns, GapCause::Reopen);
            }
            // The engine and the radio thread stamp their own reads of the same reports, so the
            // first samples after a hand passes between them can be stamped before the newest
            // one published: the controller still reporting. Its inputs count; its times, on the
            // other reader's clocks, do not, so it does not advance.
            let reread = ffi::ohmd_cv1touch_controller {
                device_time_ns: self.last_device_ns,
                received_host_ns: self.last_advance_host_ns,
                last_pose_fix_device_ns: 0,
                ..*c
            };
            return self.update_state(&reread, now_ns, None);
        }
        self.update_state(c, now_ns, None)
    }

    /// There is no sample to offer, for `cause` (a gap this starts keeps it). Keep the same gap
    /// and its deadline, even when the last sample was less than 100 ms ago.
    pub fn unavailable(&mut self, now_ns: u64, cause: GapCause) -> ControllerState {
        let c = ffi::ohmd_cv1touch_controller {
            calibrated: self.last_advance_host_ns != 0,
            device_time_ns: self.last_device_ns,
            received_host_ns: self.last_advance_host_ns,
            ..Default::default()
        };
        self.update_state(&c, now_ns, Some(cause))
    }

    /// `missing`: no sample, and why (see `unavailable`).
    fn update_state(
        &mut self,
        c: &ffi::ohmd_cv1touch_controller,
        now_ns: u64,
        missing: Option<GapCause>,
    ) -> ControllerState {
        let mut out = ControllerState {
            publish_host_ns: now_ns,
            battery_percent: -1,
            ..Default::default()
        };
        if !c.calibrated {
            return out;
        }
        out.flags |= FLAG_CALIBRATED;

        if c.device_time_ns != self.last_device_ns || c.received_host_ns > self.last_advance_host_ns
        {
            if self.in_gap {
                self.in_gap = false;
                self.event = Some(GapEvent::Ended {
                    last_sample_host_ns: self.last_advance_host_ns,
                    silent_ns: c.received_host_ns.saturating_sub(self.last_advance_host_ns),
                    cause: self.gap_cause,
                });
            }
            self.last_device_ns = c.device_time_ns;
            self.last_advance_host_ns = c.received_host_ns;
            self.clock.observe(c.device_time_ns, c.received_host_ns);
            let g = c.imu_gyro;
            if g[0] * g[0] + g[1] * g[1] + g[2] * g[2] > MOTION_RAD_S * MOTION_RAD_S {
                self.last_motion_host_ns = c.received_host_ns;
            }
        }
        let silent_ns = now_ns.saturating_sub(self.last_advance_host_ns);
        let connected = missing.is_none()
            && self.last_advance_host_ns != 0
            && silent_ns < self.cfg.connect_timeout_ns;
        // a controller whose samples stopped stays as it was for the ride-through: a radio that
        // drops it for a moment, or for the seconds it takes to find it again, takes nothing from
        // the hand holding it. Only one published live since: there is nothing else to hold.
        self.seen_live |= connected;
        let gap = self.seen_live && !connected && silent_ns < self.cfg.ride_through_ns;
        if gap && !self.in_gap {
            self.in_gap = true;
            self.gap_cause = missing.unwrap_or(GapCause::Radio);
            self.event = Some(GapEvent::Began {
                last_sample_host_ns: self.last_advance_host_ns,
                moving: self.last_motion_host_ns != 0
                    && self
                        .last_advance_host_ns
                        .saturating_sub(self.last_motion_host_ns)
                        <= MOTION_WINDOW_NS,
                cause: self.gap_cause,
            });
        } else if !gap && !connected {
            self.seen_live = false;
            if self.in_gap {
                self.in_gap = false;
                self.event = Some(GapEvent::Expired {
                    silent_ns,
                    cause: self.gap_cause,
                });
            }
        }
        if gap {
            out.flags |= FLAG_CONNECTED | FLAG_RADIO_GAP;
        } else if connected {
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
        if missing.is_none() {
            self.update_touches(&c.cap);
        }
        if connected {
            self.held = HeldInputs {
                trigger: c.trigger,
                grip: c.grip,
                stick: c.stick,
                cap: c.cap,
                buttons: self.map_buttons(c.buttons),
                touches: self.touches,
            };
        }
        // Through a radio gap the inputs stay as the controller last reported them, whatever the
        // engine keeps; in standby they are released (the engine keeps the last report).
        if connected || gap {
            let h = self.held;
            out.trigger = h.trigger;
            out.grip = h.grip;
            out.stick = h.stick;
            out.cap = h.cap;
            out.buttons = h.buttons;
            out.touches = h.touches;
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

        // Samples stop (standby); the engine keeps returning the frozen pose. A radio gap first,
        // then standby once the ride-through is over.
        let out = h.update(&c, host + 150_000_000);
        assert_eq!(out.flags & (FLAG_RADIO_GAP | FLAG_POSITION), FLAG_RADIO_GAP);
        let out = h.update(&c, host + 10_000_000_000);
        assert!(out.flags & FLAG_CALIBRATED != 0);
        assert_eq!(
            out.flags & (FLAG_CONNECTED | FLAG_POSITION | FLAG_RADIO_GAP),
            0
        );
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

    /// A controller that drops out for good (standby, out of radio range) must not keep publishing
    /// held inputs: a trigger held at the moment of disconnect would otherwise stay held forever.
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
        let gone = h.update(&c, 1_000_000_000 + 10_000_000_000);
        assert_eq!(gone.flags & FLAG_CONNECTED, 0);
        assert_eq!((gone.buttons, gone.touches), (0, 0));
        assert_eq!(
            (gone.trigger, gone.grip, gone.stick, gone.cap),
            (0.0, 0.0, [0.0; 2], [0.0; 5])
        );
    }

    /// One live sample at `host` with these inputs, the device clock a step on.
    fn live_sample(
        h: &mut Hand,
        c: &mut ffi::ohmd_cv1touch_controller,
        host: u64,
    ) -> ControllerState {
        c.device_time_ns += 2_000_000;
        c.received_host_ns = host;
        h.update(c, host)
    }

    /// The engine and the radio thread stamp their own reads of the same reports, so the first
    /// samples after a hand passes between them can be stamped before the newest one published.
    /// That is the controller still reporting: live, with no gap.
    #[test]
    fn a_sample_stamped_before_the_newest_is_no_gap() {
        let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut c = input();
        let host = 5_000_000_000;
        let first = live_sample(&mut h, &mut c, host);
        // the other reader: its own device clock, and an earlier stamp
        let mut other = input();
        other.device_time_ns = 900_000_000;
        other.received_host_ns = host - 1_000_000;
        other.buttons = 0x1;
        let reread = h.update(&other, host + 500_000);
        assert_eq!(reread.flags, FLAG_CALIBRATED | FLAG_CONNECTED);
        assert_eq!(h.take_event(), None);
        assert_eq!(reread.buttons, BTN_A);
        assert_eq!(
            (reread.received_host_ns, reread.sample_host_ns),
            (first.received_host_ns, first.sample_host_ns)
        );
        other.device_time_ns += 2_000_000;
        other.received_host_ns = host + 1_000_000;
        let next = h.update(&other, host + 1_000_000);
        assert_eq!(next.flags, FLAG_CALIBRATED | FLAG_CONNECTED);
        assert_eq!(next.received_host_ns, host + 1_000_000);
        assert_eq!(h.take_event(), None);
        // the connect timeout still runs from the newest stamp
        let late = h.update(&other, host + 101_000_001);
        assert_eq!(late.flags & FLAG_RADIO_GAP, FLAG_RADIO_GAP);
    }

    /// Through a radio gap the controller stays connected with every input as it last reported
    /// it, whatever the engine keeps or a sleeping controller's beacon says, and without a
    /// position; the gap is reported as it begins and as it ends.
    #[test]
    fn radio_gap_holds_the_inputs_as_they_were() {
        let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 10_000_000;
        c.buttons = 0x1 | 0x8;
        c.trigger = 0.9;
        c.grip = 0.8;
        c.stick = [0.5, -0.5];
        c.cap = [1.0; 5];
        c.imu_gyro = [0.0, 2.0, 0.0];
        let host = 1_000_000_000;
        let live = live_sample(&mut h, &mut c, host);
        assert!(reporting(live.flags));
        assert_eq!(h.take_event(), None);

        // the samples stop; the engine's copy of the inputs changes all the same
        c.buttons = 0;
        c.trigger = 0.0;
        c.grip = 0.0;
        c.stick = [0.0; 2];
        c.cap = [0.0; 5];
        let held = h.update(&c, host + 150_000_000);
        assert_eq!(
            held.flags,
            FLAG_CALIBRATED | FLAG_CONNECTED | FLAG_RADIO_GAP
        );
        assert!(!reporting(held.flags));
        assert_eq!(held.buttons, BTN_A | BTN_STICK);
        assert_eq!(
            (held.trigger, held.grip, held.stick, held.cap),
            (0.9, 0.8, [0.5, -0.5], [1.0; 5])
        );
        assert_eq!(held.touches, live.touches);
        assert_ne!(held.touches, 0);
        assert_eq!(
            h.take_event(),
            Some(GapEvent::Began {
                last_sample_host_ns: host,
                moving: true,
                cause: GapCause::Radio
            })
        );
        // reported once
        let still = h.update(&c, host + 9_000_000_000);
        assert_eq!(still.flags & FLAG_RADIO_GAP, FLAG_RADIO_GAP);
        assert_eq!(still.buttons, BTN_A | BTN_STICK);
        assert_eq!(h.take_event(), None);

        // back: the inputs are the controller's again
        let back = live_sample(&mut h, &mut c, host + 9_500_000_000);
        assert!(reporting(back.flags));
        assert_eq!((back.buttons, back.trigger), (0, 0.0));
        assert_eq!(
            h.take_event(),
            Some(GapEvent::Ended {
                last_sample_host_ns: host,
                silent_ns: 9_500_000_000,
                cause: GapCause::Radio
            })
        );
    }

    /// A gap keeps the cause it began with: one begun without a sample to offer says why, and a
    /// radio gap under way stays one when the engine then has no sample either.
    #[test]
    fn a_gap_keeps_the_cause_it_began_with() {
        let host = 5_000_000_000;
        for cause in [GapCause::Reopen, GapCause::Stall] {
            let mut h = Hand::new(HAND_LEFT, HandConfig::default());
            let mut c = input();
            c.trigger = 1.0;
            live_sample(&mut h, &mut c, host);
            let held = h.unavailable(host + 20_000_000, cause);
            assert_eq!(held.flags & FLAG_RADIO_GAP, FLAG_RADIO_GAP);
            assert_eq!(held.trigger, 1.0);
            assert!(
                matches!(h.take_event(), Some(GapEvent::Began { cause: began, .. }) if began == cause)
            );
            // a radio-late sample during it is the same gap
            h.update(&c, host + 3_000_000_000);
            assert_eq!(h.take_event(), None);
            live_sample(&mut h, &mut c, host + 7_000_000_000);
            assert_eq!(
                h.take_event(),
                Some(GapEvent::Ended {
                    last_sample_host_ns: host,
                    silent_ns: 7_000_000_000,
                    cause
                })
            );
            h.unavailable(host + 7_200_000_000, GapCause::Radio);
            assert!(matches!(
                h.take_event(),
                Some(GapEvent::Began {
                    cause: GapCause::Radio,
                    ..
                })
            ));
            h.unavailable(host + 17_000_000_000, cause);
            assert_eq!(
                h.take_event(),
                Some(GapEvent::Expired {
                    silent_ns: 10_000_000_000,
                    cause: GapCause::Radio
                })
            );
        }
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        live_sample(&mut h, &mut c, host);
        h.update(&c, host + 200_000_000);
        assert!(matches!(
            h.take_event(),
            Some(GapEvent::Began {
                cause: GapCause::Radio,
                ..
            })
        ));
        h.unavailable(host + 300_000_000, GapCause::Reopen);
        assert_eq!(h.take_event(), None);
        live_sample(&mut h, &mut c, host + 7_000_000_000);
        assert!(matches!(
            h.take_event(),
            Some(GapEvent::Ended {
                cause: GapCause::Radio,
                ..
            })
        ));
    }

    /// A new engine's controller can be calibrated before its first sample: that is no sample,
    /// and its empty readings leave the held touches alone.
    #[test]
    fn a_controller_without_a_sample_yet_keeps_the_gap_as_it_was() {
        let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut c = input();
        c.cap = [1.0; 5];
        let host = 5_000_000_000;
        live_sample(&mut h, &mut c, host);
        h.unavailable(host + 20_000_000, GapCause::Reopen);
        assert!(h.take_event().is_some());
        let empty = input();
        let held = h.update(&empty, host + 6_000_000_000);
        assert_eq!(held.flags & FLAG_RADIO_GAP, FLAG_RADIO_GAP);
        assert_eq!(h.take_event(), None);
        // in the hysteresis band: the touches stay as they were before the gap
        c.cap = [0.5; 5];
        let back = live_sample(&mut h, &mut c, host + 7_000_000_000);
        assert!(reporting(back.flags));
        assert_eq!(
            back.touches,
            TOUCH_A | TOUCH_B | TOUCH_TRIGGER | TOUCH_STICK | TOUCH_THUMBREST
        );
    }

    /// A gap longer than the ride-through ends in standby, reported once, inputs released.
    #[test]
    fn radio_gap_expires_into_standby() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        c.trigger = 1.0;
        let host = 5_000_000_000;
        live_sample(&mut h, &mut c, host);
        h.update(&c, host + 200_000_000);
        assert!(matches!(
            h.take_event(),
            Some(GapEvent::Began { moving: false, .. })
        ));
        let held = h.update(&c, host + 9_999_000_000);
        assert_eq!(held.trigger, 1.0);
        let gone = h.update(&c, host + 10_001_000_000);
        assert_eq!(gone.flags, FLAG_CALIBRATED);
        assert_eq!(gone.trigger, 0.0);
        assert_eq!(
            h.take_event(),
            Some(GapEvent::Expired {
                silent_ns: 10_001_000_000,
                cause: GapCause::Radio
            })
        );
        h.update(&c, host + 12_000_000_000);
        assert_eq!(h.take_event(), None);
        // a sample seen only once it is old is no gap of a controller published live long before
        c.device_time_ns += 2_000_000;
        c.received_host_ns = host + 20_000_000_000;
        assert_eq!(h.update(&c, host + 20_150_000_000).flags, FLAG_CALIBRATED);
        assert_eq!(h.take_event(), None);
        // waking long after is no gap's end
        live_sample(&mut h, &mut c, host + 30_000_000_000);
        assert_eq!(h.take_event(), None);
    }

    /// A gap holds what was published live: a controller never published live since its samples
    /// stopped (a publisher that first sees it late) is in standby, with nothing held.
    #[test]
    fn no_radio_gap_without_a_live_state_to_hold() {
        let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 10_000_000;
        c.received_host_ns = 1_000_000_000;
        c.buttons = 0x1;
        let late = h.update(&c, 1_150_000_000);
        assert_eq!(late.flags, FLAG_CALIBRATED);
        assert_eq!(late.buttons, 0);
        assert_eq!(h.take_event(), None);
    }

    /// `moving` says whether the controller turned within a few seconds of its last sample: one
    /// put down and left still has not.
    #[test]
    fn radio_gap_says_whether_the_controller_was_in_use() {
        let gap_after = |still_for_ns: u64| {
            let mut h = Hand::new(HAND_LEFT, HandConfig::default());
            let mut c = input();
            c.imu_gyro = [0.6, 0.0, 0.0];
            live_sample(&mut h, &mut c, 1_000_000_000);
            c.imu_gyro = [0.01, 0.0, 0.02];
            let mut t = 1_000_000_000;
            while t < 1_000_000_000 + still_for_ns {
                t += 2_000_000;
                live_sample(&mut h, &mut c, t);
            }
            h.update(&c, t + 150_000_000);
            match h.take_event() {
                Some(GapEvent::Began { moving, .. }) => moving,
                e => panic!("{e:?}"),
            }
        };
        assert!(gap_after(2_900_000_000));
        assert!(!gap_after(3_100_000_000));
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
