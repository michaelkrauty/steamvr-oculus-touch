// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

use crate::client::{StatusError, StatusResult};
use serde_json::Value;
use std::{collections::HashMap, time::Instant};

#[derive(Default)]
pub struct Snapshot {
    pub status: Option<Value>,
    pub status_error: Option<StatusError>,
    pub doctor: Option<Value>,
    pub doctor_error: Option<String>,
    pub rates: HashMap<String, f64>,
    pub video_rates: HashMap<String, f64>,
    /// Optical tracker updates per second, left and right.
    pub hand_rates: [Option<f64>; 2],
    previous: Option<(Instant, Value)>,
}

impl Snapshot {
    pub fn status(&mut self, result: StatusResult, now: Instant) {
        match result {
            Ok(value) => {
                self.rates.clear();
                self.video_rates.clear();
                self.hand_rates = [None; 2];
                if let Some((then, old)) = &self.previous {
                    let seconds = now.saturating_duration_since(*then).as_secs_f64();
                    if seconds > 0.0 && old["generation"] == value["generation"] {
                        for (i, rate) in self.hand_rates.iter_mut().enumerate() {
                            let updates = |v: &Value| v["hands"][i]["tracker"]["updates"].as_u64();
                            *rate = updates(&value)
                                .zip(updates(old))
                                .and_then(|(now, then)| now.checked_sub(then))
                                .map(|delta| delta as f64 / seconds);
                        }
                        let old_video = video_counters(old);
                        let old = counters(old);
                        for (serial, (kind, count)) in counters(&value) {
                            if let Some((old_kind, prior)) = old.get(&serial) {
                                if kind == *old_kind {
                                    if let Some(delta) = count.checked_sub(*prior) {
                                        self.rates.insert(serial, delta as f64 / seconds);
                                    }
                                }
                            }
                        }
                        for (serial, (kind, count)) in video_counters(&value) {
                            if let Some((old_kind, prior)) = old_video.get(&serial) {
                                if kind == *old_kind {
                                    if let Some(delta) = count.checked_sub(*prior) {
                                        self.video_rates.insert(serial, delta as f64 / seconds);
                                    }
                                }
                            }
                        }
                    }
                }
                self.previous = Some((now, value.clone()));
                self.status = Some(value);
                self.status_error = None;
            }
            Err(error) => {
                self.status = None;
                self.previous = None;
                self.rates.clear();
                self.video_rates.clear();
                self.hand_rates = [None; 2];
                self.status_error = Some(error);
            }
        }
    }

    pub fn doctor(&mut self, result: Result<Value, String>) {
        match result {
            Ok(value) => {
                self.doctor = Some(value);
                self.doctor_error = None;
            }
            Err(error) => {
                self.doctor = None;
                self.doctor_error = Some(error);
            }
        }
    }
}

fn video_counters(value: &Value) -> HashMap<String, (String, u64)> {
    value["sensors"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(|sensor| {
            let format = sensor["video_format"].as_str()?;
            let field = match format {
                "raw" => "frames_received",
                "jpeg" => "frames_decoded",
                _ => return None,
            };
            Some((
                sensor["serial"].as_str()?.to_ascii_uppercase(),
                (format.to_owned(), sensor[field].as_u64()?),
            ))
        })
        .collect()
}

fn counters(value: &Value) -> HashMap<String, (String, u64)> {
    value["sensors"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(|sensor| {
            let serial = sensor["serial"].as_str()?.to_ascii_uppercase();
            Some((
                serial,
                ("received".to_owned(), sensor["frames_received"].as_u64()?),
            ))
        })
        .collect()
}

/// A controller's state from its status flags: a short label, what it means, and its level.
pub fn controller(flags: u64) -> (&'static str, &'static str, Level) {
    if flags & 1 == 0 {
        ("Not heard from", "Press a button to wake it.", Level::Quiet)
    } else if flags & 16 != 0 {
        ("Asleep", "Press a button to wake it.", Level::Quiet)
    } else if flags & 2 == 0 {
        (
            "Out of range",
            "Out of radio range or turned off.",
            Level::Warning,
        )
    } else if flags & 8 != 0 {
        (
            "Radio gap",
            "Its radio signal dropped for a moment.",
            Level::Warning,
        )
    } else if flags & 4 != 0 {
        (
            "Tracking",
            "Position tracked: a sensor sees it.",
            Level::Good,
        )
    } else {
        (
            "Rotation only",
            "No sensor sees it: only its rotation is tracked.",
            Level::Warning,
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Level {
    Good,
    Warning,
    Bad,
    Quiet,
}

/// A sensor's state: a short label, what it means, and its level.
pub fn sensor(
    value: &Value,
    rate: Option<f64>,
    video_rate: Option<f64>,
) -> (&'static str, &'static str, Level) {
    match value["pose"]["state"].as_str() {
        Some("given_up") => (
            "Left out",
            "Left out of tracking: no pose fits its view.",
            Level::Bad,
        ),
        Some("moved") => (
            "Moved",
            "It moved and is being placed again.",
            Level::Warning,
        ),
        Some("released") => (
            "Rechecking",
            "Accepted for now as calibrated; a recheck is scheduled.",
            Level::Warning,
        ),
        _ if value["placed"].as_u64() == Some(0) => {
            ("Not placed", "Not placed in the room yet.", Level::Warning)
        }
        _ if value["placed"].as_u64() != Some(1) => ("Unknown", "Placement unknown.", Level::Quiet),
        _ if rate == Some(0.0) => (
            "No video",
            "No frames received in the last interval.",
            Level::Warning,
        ),
        _ if rate.is_none() => ("Checking", "Measuring its frame rate.", Level::Quiet),
        _ if video_rate == Some(0.0) && value["video_format"] == "jpeg" => {
            ("No video", "Frames arrive but none decode.", Level::Warning)
        }
        _ if video_rate.is_some_and(|rate| rate > 0.0) => {
            ("Streaming", "Video working.", Level::Good)
        }
        _ => (
            "Receiving",
            "Receiving frames; decoding not verified yet.",
            Level::Quiet,
        ),
    }
}

/// What the daemon is doing, as far as the GUI knows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Daemon {
    Checking,
    Running,
    Stopped,
    Busy(crate::actions::ServiceAction),
}

impl Daemon {
    /// The daemon's state from the latest status sample and the action in flight.
    pub fn from(snap: &Snapshot, busy: Option<crate::actions::ServiceAction>) -> Self {
        if let Some(action) = busy {
            Self::Busy(action)
        } else if snap.status.is_some() {
            Self::Running
        } else {
            match &snap.status_error {
                Some(error) if error.down => Self::Stopped,
                // It is there but not answering: restarting it is the useful action.
                Some(_) => Self::Running,
                None => Self::Checking,
            }
        }
    }

    /// Whether `action` makes sense now: start only a stopped daemon, restart or stop only a
    /// running one, and nothing while the state is unknown or another action is in flight.
    pub fn allows(self, action: crate::actions::ServiceAction) -> bool {
        use crate::actions::ServiceAction::*;
        matches!(
            (self, action),
            (Self::Running, Restart | Stop) | (Self::Stopped, Start)
        )
    }
}

pub fn problems(doctor: &Value) -> Vec<(Level, String)> {
    doctor["checks"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(|c| {
            let detail = c["detail"].as_str().unwrap_or("No details available");
            if c["ok"].as_bool() != Some(true) {
                Some((Level::Bad, detail.to_owned()))
            } else if let Some(rest) = detail.strip_prefix("todo: ") {
                Some((Level::Warning, rest.to_owned()))
            } else {
                detail
                    .strip_prefix("warning: ")
                    .map(|rest| (Level::Warning, rest.to_owned()))
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use std::time::Duration;
    #[test]
    fn rates_follow_identity_generation_and_counter_type() {
        let mut snap = Snapshot::default();
        let t = Instant::now();
        let status = |generation, format, n| json!({"generation": generation, "sensor_video_quality": true, "sensors": [{"serial":"abc", "video_format":format, "frames_received":n, "frames_decoded":n}]});
        snap.status(Ok(status(1, "raw", 100)), t);
        snap.status(Ok(status(1, "raw", 160)), t + Duration::from_secs(2));
        assert_eq!(snap.rates["ABC"], 30.0);
        snap.status(Ok(status(1, "jpeg", 200)), t + Duration::from_secs(4));
        assert_eq!(snap.rates["ABC"], 20.0);
        snap.status(Ok(status(2, "jpeg", 220)), t + Duration::from_secs(6));
        assert!(snap.rates.is_empty());
        snap.status(Err("offline".into()), t + Duration::from_secs(8));
        assert!(snap.status.is_none());
    }
    #[test]
    fn flags_and_checks() {
        assert_eq!(controller(0).0, "Not heard from");
        assert_eq!(controller(17).0, "Asleep");
        assert_eq!(controller(3).0, "Rotation only");
        assert_eq!(controller(11).0, "Radio gap");
        assert_eq!(
            controller(7),
            (
                "Tracking",
                "Position tracked: a sensor sees it.",
                Level::Good
            )
        );
        assert_eq!(problems(&json!({"checks":[{"ok":false,"detail":"failed"},{"ok":true,"detail":"warning: check"}]})).len(), 2);
    }
    #[test]
    fn video_proof_requires_matching_advanced_authoritative_counter() {
        let mut snap = Snapshot::default();
        let t = Instant::now();
        let status = |generation, format, received, decoded: Value| json!({"generation":generation,"sensors":[{"serial":"a","placed":1,"video_format":format,"frames_received":received,"frames_decoded":decoded}]});
        snap.status(Ok(status(1, "jpeg", 10, json!(0))), t);
        assert!(snap.video_rates.is_empty());
        snap.status(
            Ok(status(1, "jpeg", 20, json!(0))),
            t + Duration::from_secs(1),
        );
        assert!(
            sensor(
                &snap.status.as_ref().unwrap()["sensors"][0],
                snap.rates.get("A").copied(),
                snap.video_rates.get("A").copied()
            )
            .2 == Level::Warning
        );
        snap.status(
            Ok(status(1, "jpeg", 30, json!(5))),
            t + Duration::from_secs(2),
        );
        assert!(
            sensor(
                &snap.status.as_ref().unwrap()["sensors"][0],
                snap.rates.get("A").copied(),
                snap.video_rates.get("A").copied()
            )
            .2 == Level::Good
        );
        snap.status(
            Ok(status(1, "raw", 40, json!(5))),
            t + Duration::from_secs(3),
        );
        assert!(snap.video_rates.is_empty());
        snap.status(
            Ok(status(1, "raw", 50, json!(null))),
            t + Duration::from_secs(4),
        );
        assert_eq!(snap.video_rates["A"], 10.0);
        snap.status(
            Ok(status(2, "raw", 60, json!(null))),
            t + Duration::from_secs(5),
        );
        assert!(snap.video_rates.is_empty());
        snap.status(
            Ok(status(2, "jpeg", 70, json!(null))),
            t + Duration::from_secs(6),
        );
        snap.status(
            Ok(status(2, "jpeg", 80, json!(null))),
            t + Duration::from_secs(7),
        );
        assert!(snap.video_rates.is_empty());
        snap.status(
            Ok(status(2, "other", 90, json!(5))),
            t + Duration::from_secs(8),
        );
        assert!(snap.video_rates.is_empty());
    }
    #[test]
    fn sensor_states_and_rate_recovery() {
        for (state, expected) in [
            ("given_up", "Left out"),
            ("moved", "Moved"),
            ("released", "Rechecking"),
        ] {
            assert_eq!(
                sensor(&json!({"pose":{"state":state}}), Some(30.0), None).0,
                expected
            );
        }
        assert_eq!(sensor(&json!({}), Some(30.0), None).0, "Unknown");
        assert_eq!(sensor(&json!({"placed":1}), Some(0.0), None).0, "No video");
        assert_eq!(
            sensor(&json!({"placed":1}), Some(30.0), None).0,
            "Receiving"
        );
        assert_eq!(
            sensor(&json!({"placed":1}), Some(30.0), Some(30.0)).0,
            "Streaming"
        );
        let mut snap = Snapshot::default();
        let now = Instant::now();
        let status = |n| json!({"generation":1,"sensors":[{"serial":"a","frames_received":n}]});
        snap.status(Ok(status(100)), now);
        snap.status(Ok(status(1)), now + Duration::from_secs(1));
        assert!(snap.rates.is_empty());
        snap.status(Ok(status(10)), now + Duration::from_secs(2));
        assert_eq!(snap.rates["A"], 9.0);
        snap.status(Err("failed".into()), now + Duration::from_secs(3));
        snap.status(Ok(status(20)), now + Duration::from_secs(4));
        assert!(snap.rates.is_empty());
    }
    #[test]
    fn controller_update_rates_follow_tracker_updates() {
        let mut snap = Snapshot::default();
        let t = Instant::now();
        let status = |generation, l: u64, r: Value| {
            json!({"generation": generation, "sensors": [],
            "hands": [{"tracker": {"updates": l}}, {"tracker": {"updates": r}}]})
        };
        snap.status(Ok(status(1, 100, json!(50))), t);
        assert_eq!(snap.hand_rates, [None, None]);
        snap.status(
            Ok(status(1, 152, json!(null))),
            t + Duration::from_millis(500),
        );
        assert_eq!(snap.hand_rates, [Some(104.0), None]);
        // A restarted daemon's counters start again.
        snap.status(Ok(status(2, 10, json!(0))), t + Duration::from_secs(1));
        assert_eq!(snap.hand_rates, [None, None]);
        snap.status(Ok(status(2, 5, json!(30))), t + Duration::from_secs(2));
        assert_eq!(snap.hand_rates, [None, Some(30.0)]);
    }
}
