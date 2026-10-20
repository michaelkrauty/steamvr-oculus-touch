// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Text rendering of the live status board (`cv1touch status`).

use std::collections::HashMap;

use crate::hand::{
    BTN_A, BTN_B, BTN_STICK, BTN_SYSTEM, BTN_X, BTN_Y, FLAG_CALIBRATED, FLAG_CONNECTED,
    FLAG_POSITION, FLAG_RADIO_GAP, TOUCH_A, TOUCH_B, TOUCH_STICK, TOUCH_THUMBREST, TOUCH_TRIGGER,
    TOUCH_X, TOUCH_Y,
};
use crate::shm::{serial_str, ControllerState, SensorStatus};

const BUTTONS: [(&str, u32); 6] = [
    ("A", BTN_A),
    ("B", BTN_B),
    ("X", BTN_X),
    ("Y", BTN_Y),
    ("system", BTN_SYSTEM),
    ("stick", BTN_STICK),
];
const TOUCHES: [(&str, u32); 7] = [
    ("A", TOUCH_A),
    ("B", TOUCH_B),
    ("X", TOUCH_X),
    ("Y", TOUCH_Y),
    ("trigger", TOUCH_TRIGGER),
    ("stick", TOUCH_STICK),
    ("thumbrest", TOUCH_THUMBREST),
];
const HAND_NAMES: [&str; 2] = ["left", "right"];

/// One identity for serials from calibration, USB enumeration, daemon diagnostics and the
/// `--ignore-sensor` option. The native engine ignores ASCII letter case when it matches serials,
/// so everything on the Rust side does too; each source keeps the spelling it reported.
pub fn sensor_identity(serial: &str) -> String {
    serial.to_ascii_uppercase()
}

/// Whether two serials name the same sensor.
pub fn same_sensor(a: &str, b: &str) -> bool {
    a.eq_ignore_ascii_case(b)
}

/// Released analysis-buffer counters, retained for setup and older daemons' board rates.
pub fn frame_counters(value: &serde_json::Value) -> HashMap<String, u64> {
    sensor_counters(value, "frames")
}

/// Frames delivered by the camera, before analysis queues or locks can delay them.
pub fn received_frame_counters(value: &serde_json::Value) -> HashMap<String, u64> {
    sensor_counters(value, "frames_received")
}

/// The capability flag distinguishes an empty modern sensor list from an older daemon.
pub fn has_received_frame_counters(value: &serde_json::Value) -> bool {
    value["sensor_frames_received"] == true
        || value["sensors"]
            .as_array()
            .is_some_and(|sensors| sensors.iter().any(|s| s.get("frames_received").is_some()))
}

/// The board shows received video when available, with released-buffer rates for older daemons.
pub fn board_frame_counters(value: &serde_json::Value) -> HashMap<String, u64> {
    if has_received_frame_counters(value) {
        received_frame_counters(value)
    } else {
        frame_counters(value)
    }
}

fn sensor_counters(value: &serde_json::Value, field: &str) -> HashMap<String, u64> {
    value["sensors"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(|s| Some((sensor_identity(s["serial"].as_str()?), s[field].as_u64()?)))
        .collect()
}

/// Received counters are 64-bit. Any decrease, including an older u32 wrap, starts a new sample.
pub fn frames_advanced(previous: u64, current: u64) -> bool {
    current > previous
}

/// Decode the complete sensor list from control JSON, independently of the
/// fixed-size compatibility block in controller shared memory.
pub fn sensors_from_status(value: &serde_json::Value) -> Result<Vec<SensorStatus>, String> {
    #[derive(serde::Deserialize)]
    struct SensorSummary {
        serial: String,
        placed: u32,
        frames: u32,
        fixes: [u32; 2],
    }
    let entries: Vec<SensorSummary> = serde_json::from_value(value["sensors"].clone())
        .map_err(|e| format!("invalid sensor diagnostics: {e}"))?;
    if value["sensor_total"]
        .as_u64()
        .is_some_and(|n| n != entries.len() as u64)
    {
        return Err("sensor count disagrees with the complete sensor list".into());
    }
    entries
        .into_iter()
        .map(|s| {
            if s.serial.is_empty() || s.serial.len() >= 32 || s.serial.contains('\0') {
                return Err("invalid sensor serial in diagnostics".into());
            }
            let mut out = SensorStatus {
                placed: s.placed,
                frames: s.frames,
                pose_fixes: s.fixes,
                ..Default::default()
            };
            crate::shm::set_serial(&mut out, &s.serial);
            Ok(out)
        })
        .collect()
}

fn state_text(s: &ControllerState) -> &'static str {
    if s.flags & FLAG_CALIBRATED == 0 {
        "not paired / never seen"
    } else if s.flags & FLAG_CONNECTED == 0 {
        "STANDBY / out of range"
    } else if s.flags & FLAG_RADIO_GAP != 0 {
        "radio silent: held as it was"
    } else if s.flags & FLAG_POSITION != 0 {
        "6DoF"
    } else {
        "rotation only (no optical fix)"
    }
}

fn names(mask: u32, table: &[(&str, u32)]) -> String {
    let v: Vec<&str> = table
        .iter()
        .filter(|(_, bit)| mask & bit != 0)
        .map(|(n, _)| *n)
        .collect();
    if v.is_empty() {
        "-".into()
    } else {
        v.join(" ")
    }
}

pub fn render(
    hands: &[ControllerState; 2],
    sensors: &[SensorStatus],
    rates: &HashMap<String, (f64, f64, f64)>,
    heartbeat_age_s: f64,
    generation: u64,
) -> String {
    let mut lines = vec![
        format!(
            "cv1touchd generation {generation:#x}   heartbeat {:.0} ms ago",
            heartbeat_age_s * 1000.0
        ),
        String::new(),
    ];
    for (i, s) in hands.iter().enumerate() {
        lines.push(format!("{:>5}: {}", HAND_NAMES[i], state_text(s)));
    }
    lines.push(String::new());
    for (i, s) in hands.iter().enumerate() {
        if s.flags & FLAG_CONNECTED != 0 {
            let p = s.pos;
            lines.push(format!(
                "{:>5}  pos mm {:+8.1} {:+8.1} {:+8.1}   trig {:.2} grip {:.2} stick {:+.2} {:+.2}",
                HAND_NAMES[i],
                p[0] * 1000.0,
                p[1] * 1000.0,
                p[2] * 1000.0,
                s.trigger,
                s.grip,
                s.stick[0],
                s.stick[1]
            ));
            lines.push(format!(
                "       buttons {}   touch {}",
                names(s.buttons, &BUTTONS),
                names(s.touches, &TOUCHES)
            ));
        }
    }
    lines.push(String::new());
    for s in sensors {
        let serial = serial_str(s);
        let (frames, left, right) = rates.get(&serial).copied().unwrap_or((0.0, 0.0, 0.0));
        let placed = if s.placed != 0 {
            "placed"
        } else {
            "NOT placed"
        };
        lines.push(format!(
            "sensor {serial}: {placed}  frames/s {frames:5.1}  fixes/s L {left:5.1} R {right:5.1}"
        ));
    }
    lines.join("\n")
}

/// Per-second rates of monotonically increasing counters (frames, fixes) for the status board.
/// The daemon publishes sensor counters in 250 ms steps while the board samples faster, so rates
/// are measured between the times counter changes were first observed, over at least `window_ns`.
pub struct RateTracker {
    window_ns: u64,
    changes: std::collections::VecDeque<(u64, [u64; 3])>,
}

impl RateTracker {
    pub fn new(window_ns: u64) -> Self {
        Self {
            window_ns,
            changes: std::collections::VecDeque::new(),
        }
    }

    /// Records the counters seen at `now_ns`; returns per-second rates (zeros if the counters
    /// have not changed within the window).
    pub fn update(&mut self, now_ns: u64, counts: [u64; 3]) -> [f64; 3] {
        if self
            .changes
            .back()
            .is_some_and(|(_, previous)| (0..3).any(|i| counts[i] < previous[i]))
        {
            self.changes.clear();
        }
        if self.changes.back().is_none_or(|(_, c)| *c != counts) {
            self.changes.push_back((now_ns, counts));
        }
        while self.changes.len() > 2 && self.changes[1].0 + self.window_ns <= now_ns {
            self.changes.pop_front();
        }
        let (t0, c0) = self.changes[0];
        let (t1, c1) = *self.changes.back().unwrap();
        if t1 == t0 || now_ns.saturating_sub(t1) > self.window_ns {
            return [0.0; 3];
        }
        let dt = (t1 - t0) as f64 / 1e9;
        [0, 1, 2].map(|i| c1[i].saturating_sub(c0[i]) as f64 / dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hand::{BTN_A, FLAG_CALIBRATED, FLAG_CONNECTED, FLAG_POSITION, TOUCH_TRIGGER};

    #[test]
    fn serials_name_the_same_sensor_whatever_their_ascii_letter_case() {
        assert!(same_sensor("WMTD000000001", "wmtd000000001"));
        assert!(same_sensor("Camera1", "cAMERA1"));
        assert!(!same_sensor("CAMERA1", "CAMERA10"));
        assert!(!same_sensor("CAMERA1", "CAMERA"));
        assert_eq!(sensor_identity("Camera1"), sensor_identity("cAMERA1"));
        assert_ne!(sensor_identity("CAMERA1"), sensor_identity("CAMERA2"));
    }

    fn state(flags: u32) -> ControllerState {
        ControllerState {
            flags,
            pos: [0.1, -0.2, 0.3],
            buttons: BTN_A,
            touches: TOUCH_TRIGGER,
            ..Default::default()
        }
    }

    #[test]
    fn complete_control_diagnostics_render_every_sensor_beyond_the_shm_prefix() {
        let value = serde_json::json!({"sensor_total": 65, "sensors": (0..65).map(|i|
            serde_json::json!({"serial": format!("camera-{i}"), "placed": 1,
                "frames": i + 1, "fixes": [i, i + 2]})).collect::<Vec<_>>()});
        let sensors = sensors_from_status(&value).unwrap();
        assert_eq!(sensors.len(), 65);
        assert_eq!(sensors[64].frames, 65);
        assert_eq!(serial_str(&sensors[64]), "camera-64");
        let text = render(
            &[ControllerState::default(); 2],
            &sensors,
            &HashMap::new(),
            0.0,
            1,
        );
        assert_eq!(
            text.lines().filter(|s| s.starts_with("sensor ")).count(),
            65
        );
        assert!(text.contains("sensor camera-64:"));
        let mut invalid = value.clone();
        invalid["sensor_total"] = serde_json::json!(66);
        assert!(sensors_from_status(&invalid).is_err());
        invalid = value;
        invalid["sensors"][64]["fixes"] = serde_json::json!([1]);
        assert!(sensors_from_status(&invalid).is_err());
    }

    #[test]
    fn render_states_and_rates() {
        let mut sensor = SensorStatus::default();
        crate::shm::set_serial(&mut sensor, "WMTD000000001");
        sensor.placed = 1;
        let rates = HashMap::from([("WMTD000000001".to_string(), (50.0, 0.0, 48.5))]);
        let text = render(
            &[
                state(FLAG_CALIBRATED),
                state(FLAG_CALIBRATED | FLAG_CONNECTED | FLAG_POSITION),
            ],
            &[sensor],
            &rates,
            0.004,
            0x2A,
        );
        let lines: Vec<&str> = text.lines().collect();
        assert!(lines[2].contains("left") && lines[2].contains("STANDBY"));
        assert!(lines[3].contains("right") && lines[3].contains("6DoF"));
        assert!(text.contains("+100.0") && text.contains("-200.0"));
        assert!(text.contains("buttons A") && text.contains("touch trigger"));
        assert!(text.contains("WMTD000000001: placed") && text.contains("R  48.5"));
    }

    #[test]
    fn render_radio_gap() {
        let text = render(
            &[
                state(FLAG_CALIBRATED | FLAG_CONNECTED | FLAG_RADIO_GAP),
                state(FLAG_CALIBRATED | FLAG_CONNECTED),
            ],
            &[],
            &HashMap::new(),
            0.004,
            0x2A,
        );
        let lines: Vec<&str> = text.lines().collect();
        assert!(lines[2].contains("left") && lines[2].contains("radio silent: held as it was"));
        assert!(lines[3].contains("right") && lines[3].contains("rotation only"));
    }

    /// Sensor counters are published every 250 ms but the board samples every 200 ms; rates must
    /// not alias between 0 and 1.25x the real rate.
    #[test]
    fn rate_tracker_is_stable_with_stepped_counters() {
        let mut r = RateTracker::new(2_000_000_000);
        let mut last = [0.0; 3];
        for i in 0..20u64 {
            let now = 1_000_000_000 + i * 200_000_000;
            let steps = (now - 1_000_000_000) / 250_000_000; // publishes so far
            let frames = steps * 13; // 52 frames/s
            last = r.update(now, [frames, frames / 2, 0]);
        }
        assert!((last[0] - 52.0).abs() < 5.2, "frames/s {}", last[0]);
        assert!((last[1] - 26.0).abs() < 2.6, "fixes/s {}", last[1]);
        assert_eq!(last[2], 0.0);
    }

    #[test]
    fn rate_tracker_reports_zero_when_counters_stall() {
        let mut r = RateTracker::new(2_000_000_000);
        for i in 0..10u64 {
            r.update(i * 250_000_000, [i * 13, 0, 0]);
        }
        let stalled = r.update(10 * 250_000_000 + 2_500_000_000, [117, 0, 0]);
        assert_eq!(stalled, [0.0; 3]);
    }

    #[test]
    fn render_never_seen() {
        let text = render(
            &[ControllerState::default(), ControllerState::default()],
            &[],
            &HashMap::new(),
            0.0,
            1,
        );
        assert_eq!(text.matches("not paired / never seen").count(), 2);
    }

    #[test]
    fn upper_half_counter_resets_do_not_make_video_or_rate_progress() {
        assert!(!frames_advanced(3_000_000_000, 0));
        let mut rates = RateTracker::new(2_000_000_000);
        rates.update(1_000_000_000, [3_000_000_000, 0, 0]);
        assert_eq!(rates.update(2_000_000_000, [0, 0, 0]), [0.0; 3]);
        assert_eq!(rates.update(3_000_000_000, [52, 0, 0]), [52.0, 0.0, 0.0]);
    }
    #[test]
    fn board_rates_use_received_video_and_keep_legacy_released_counts() {
        for (modern, expected) in [(true, 52.0), (false, 1.0)] {
            let mut tracker = RateTracker::new(2_000_000_000);
            let mut rate = [0.0; 3];
            for (step, frames, received) in [(1, 40, 4_294_967_296u64), (2, 41, 4_294_967_348)] {
                let mut value =
                    serde_json::json!({"sensors":[{"serial":"Camera1", "frames":frames}]});
                if modern {
                    value["sensors"][0]["frames_received"] = received.into();
                }
                let counts = board_frame_counters(&value);
                rate = tracker.update(step * 1_000_000_000, [counts["CAMERA1"], 0, 0]);
            }
            assert_eq!(rate[0], expected);
        }
    }
}
