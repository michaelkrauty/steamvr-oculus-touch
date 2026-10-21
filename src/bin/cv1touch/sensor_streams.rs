// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Read-only camera diagnostics, shared by doctor, the status board and reports.

use super::*;
use std::collections::BTreeSet;

// Sensor counters are published every 250 ms. Match wait_for_mode's one-second samples.
pub(super) const SAMPLE_INTERVAL: Duration = Duration::from_secs(1);
const MAX_SAMPLES: usize = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Outcome {
    Pass,
    Fail,
    Pending,
    Skipped,
    NotVerified,
}

pub(super) struct Check {
    outcome: Outcome,
    detail: String,
    // A confirmed failure can coexist with cameras that need another sample.
    pending: bool,
    drop_notes: Vec<status::VideoDropNote>,
}

impl Check {
    fn new(outcome: Outcome, detail: impl Into<String>) -> Self {
        Self {
            outcome,
            detail: detail.into(),
            pending: outcome == Outcome::Pending,
            drop_notes: Vec::new(),
        }
    }

    fn has_warning(&self) -> bool {
        self.drop_notes.iter().any(status::VideoDropNote::warning)
    }

    fn description(self) -> String {
        let mut detail = self.detail;
        for note in self.drop_notes {
            detail.push_str("; ");
            detail.push_str(&note.text());
        }
        detail
    }

    fn board_line(self) -> String {
        let label = match self.outcome {
            Outcome::Pending => "checking",
            Outcome::Pass if self.has_warning() => "warning",
            Outcome::Pass | Outcome::Skipped => "ok",
            Outcome::Fail => "FAIL",
            Outcome::NotVerified => "not verified",
        };
        format!("{label} sensor_streams: {}", self.description())
    }

    pub(super) fn conclusion(self) -> (bool, String) {
        let outcome = self.outcome;
        let warning = self.has_warning();
        let detail = self.description();
        match outcome {
            Outcome::Pass if warning => (true, format!("warning: {detail}")),
            Outcome::Pass | Outcome::Skipped => (true, detail),
            Outcome::Fail | Outcome::NotVerified => (false, detail),
            Outcome::Pending => (
                false,
                format!(
                    "not verified: {detail}; run cv1touch doctor again after the daemon settles"
                ),
            ),
        }
    }
}

pub(super) fn doctor(
    calibration: Option<&file::CalibrationFile>,
) -> ((bool, String), io::Result<Option<serde_json::Value>>) {
    sample_status(
        || daemon_status(Duration::from_secs(5), DaemonPurpose::Check),
        thread::sleep,
        |first, second| from_snapshots(|| calibration.map(calibration_serials), first, second),
    )
}

// Preserve request errors alongside the sampled status, as the other CLI status helpers do.
fn sample_status(
    mut read: impl FnMut() -> io::Result<Option<serde_json::Value>>,
    wait: impl FnMut(Duration),
    inspect: impl FnMut(Option<&serde_json::Value>, Option<&serde_json::Value>) -> Check,
) -> ((bool, String), io::Result<Option<serde_json::Value>>) {
    let mut request_error = None;
    let (check, daemon) = sample(
        || match read() {
            Ok(status) => status,
            Err(error) => {
                request_error = Some(error);
                None
            }
        },
        wait,
        inspect,
    );
    match request_error {
        Some(error) => {
            let check = if matches!(
                error.kind(),
                io::ErrorKind::NotFound | io::ErrorKind::ConnectionRefused
            ) {
                check
            } else {
                (false, format!("video not verified: {error}"))
            };
            (check, Err(error))
        }
        None => (check, Ok(daemon)),
    }
}

// Keep the acquisition loop injectable: a reconnect or counter reset must get a later,
// spaced sample, and repeated changes must still finish without claiming verified video.
fn sample(
    mut read: impl FnMut() -> Option<serde_json::Value>,
    mut wait: impl FnMut(Duration),
    mut inspect: impl FnMut(Option<&serde_json::Value>, Option<&serde_json::Value>) -> Check,
) -> ((bool, String), Option<serde_json::Value>) {
    let mut current = read();
    let mut result = inspect(None, current.as_ref());
    let mut drop_notes = Vec::new();
    for _ in 1..MAX_SAMPLES {
        if !result.pending {
            break;
        }
        // Adjacent intervals do not overlap. Preserve only rejection observations,
        // not a camera's transient Pending/failure description or an assumed rate.
        drop_notes.append(&mut result.drop_notes);
        wait(SAMPLE_INTERVAL);
        let previous = current;
        current = read();
        result = inspect(previous.as_ref(), current.as_ref());
    }
    drop_notes.append(&mut result.drop_notes);
    result.drop_notes = drop_notes;
    (result.conclusion(), current)
}

pub(super) fn board_note(
    first: Option<&serde_json::Value>,
    second: Option<&serde_json::Value>,
) -> String {
    from_snapshots(
        || {
            file::load(&file::default_path())
                .ok()
                .as_ref()
                .map(calibration_serials)
        },
        first,
        second,
    )
    .board_line()
}

fn calibration_serials(calibration: &file::CalibrationFile) -> Vec<String> {
    calibration
        .sensors
        .iter()
        .map(|s| s.serial.clone())
        .collect()
}

fn from_snapshots(
    fallback: impl FnOnce() -> Option<Vec<String>>,
    first: Option<&serde_json::Value>,
    second: Option<&serde_json::Value>,
) -> Check {
    with_sources(fallback, sensors::connected_serials, first, second)
}

#[cfg(windows)]
const RESTART: &str =
    "then restart the daemon: choose Stop cv1touch, then Start cv1touch in the Start menu";
#[cfg(not(windows))]
const RESTART: &str = "then restart the daemon: systemctl --user restart cv1touchd";

fn stream_failure(current: &serde_json::Value, serial: &str) -> Option<String> {
    let stream = current["sensor_streams"].as_array()?.iter().find(|s| {
        s["serial"]
            .as_str()
            .is_some_and(|s| status::sensor_identity(s) == serial)
            && s["error_kind"].as_str() != Some("none")
    })?;
    match stream["error_kind"].as_str()? {
        "likely_usb_bandwidth" => Some(format!("most likely not enough USB bandwidth on this controller: plug it into a different USB controller, {RESTART}")),
        "other" => Some(format!("USB video couldn't start (error {}, system error {}): check USB access and daemon logs; after reconnecting, {RESTART}",
            stream["error_code"].as_i64().unwrap_or(0), stream["os_error"].as_i64().unwrap_or(0))),
        _ => None,
    }
}

fn identities(serials: &[String]) -> BTreeSet<String> {
    serials.iter().map(|s| status::sensor_identity(s)).collect()
}

fn with_sources(
    fallback: impl FnOnce() -> Option<Vec<String>>,
    connected: impl FnOnce() -> Result<Vec<String>, String>,
    previous: Option<&serde_json::Value>,
    current: Option<&serde_json::Value>,
) -> Check {
    let skip = |reason: &str| Check::new(Outcome::Skipped, format!("skipped: {reason}"));
    let pending = |reason: &str| Check::new(Outcome::Pending, reason);
    let Some(current) = current else {
        return skip("daemon isn't running");
    };
    // Absence alone means an older daemon. A present null means this daemon has no
    // loaded calibration; never replace an active (or empty) set with the default file.
    let active = current.get("calibrated_sensors");
    let declared = match active {
        Some(serde_json::Value::Null) => {
            return skip("no sensor calibration is loaded; run cv1touch setup")
        }
        Some(value) => match serde_json::from_value::<Vec<String>>(value.clone()) {
            Ok(serials) => Some(identities(&serials)),
            Err(_) => {
                return Check::new(
                    Outcome::Fail,
                    "invalid active sensor calibration in daemon status",
                )
            }
        },
        None => None,
    };
    let ignored: BTreeSet<_> = current["ignored_sensors"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(|v| v.as_str())
        .map(status::sensor_identity)
        .collect();
    if current["engine_ready"] == false {
        let Some(calibrated) = declared.or_else(|| fallback().map(|serials| identities(&serials)))
        else {
            return skip("older daemon: no valid default sensor calibration; run cv1touch setup");
        };
        if calibrated.difference(&ignored).next().is_none() {
            return skip("no calibrated sensors are enabled");
        }
        return pending("the tracking engine isn't running, so camera video couldn't be checked: check the headset's USB connection and power; check daemon logs for an engine open error");
    }
    if !status::has_received_frame_counters(current)
        && current["sensor_video_quality"] != true
        && !current["sensors"]
            .as_array()
            .is_some_and(|sensors| sensors.iter().any(|s| s.get("video_format").is_some()))
    {
        return Check::new(
            Outcome::NotVerified,
            format!(
                "video not verified: the daemon is older than this cv1touch; {}",
                RESTART.trim_start_matches("then ")
            ),
        );
    }
    let Some(previous) = previous else {
        return pending("waiting for a second video sample");
    };
    if previous["engine_ready"] == false {
        return pending("tracking engine was starting; wait for another video sample");
    }
    let previous_calibrated = previous
        .get("calibrated_sensors")
        .and_then(|v| serde_json::from_value::<Vec<String>>(v.clone()).ok())
        .map(|serials| identities(&serials));
    let calibration_changed = if active.is_some() {
        previous_calibrated.as_ref() != declared.as_ref()
    } else {
        previous.get("calibrated_sensors").is_some()
    };
    if previous["generation"] != current["generation"]
        || previous["mode"] != current["mode"]
        || calibration_changed
    {
        return pending(
            "daemon, tracking mode or calibration changed; wait for another video sample",
        );
    }
    let Some(calibrated) = declared.or_else(|| fallback().map(|serials| identities(&serials)))
    else {
        return skip("older daemon: no valid default sensor calibration; run cv1touch setup");
    };
    // USB enumeration can be expensive, especially on Windows. Only comparable
    // samples need it; per-camera decisions still need fresh USB presence so an
    // unplugged native object cannot pass on retained counters or errors.
    let connected = connected().map(|serials| identities(&serials));
    let old = status::video_frame_counters(previous);
    let new = status::video_frame_counters(current);
    let reported: BTreeSet<_> = current["sensors"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(|sensor| sensor["serial"].as_str())
        .map(status::sensor_identity)
        .collect();
    let mut failed = false;
    let mut unverified = false;
    let mut pending = false;
    let mut detail = Vec::new();
    for serial in calibrated.difference(&ignored) {
        // Native objects may outlive unplugging. OS absence beats retained counters.
        if connected.as_ref().is_ok_and(|list| !list.contains(serial)) {
            failed = true;
            detail.push(format!(
                "{serial} isn't connected: check its USB cable; after reconnecting, {RESTART}"
            ));
        } else if old
            .get(serial)
            .zip(new.get(serial))
            .is_some_and(|(&a, &b)| b.advanced_from(a))
        {
            detail.push(format!("{serial} is streaming"));
        } else if let Some(reason) = stream_failure(current, serial) {
            failed = true;
            if let Err(error) = &connected {
                detail.push(format!("{serial}'s USB connection couldn't be checked: {error}; last start attempt: {reason}"));
            } else {
                detail.push(format!("{serial} isn't sending video: {reason}"));
            }
        } else if reported.contains(serial) && !new.contains_key(serial) {
            unverified = true;
            let connection = connected.as_ref().err().map_or(String::new(), |error| {
                format!("; its USB connection couldn't be checked: {error}")
            });
            let counter = current["sensors"]
                .as_array()
                .into_iter()
                .flatten()
                .find(|s| {
                    s["serial"]
                        .as_str()
                        .is_some_and(|s| status::sensor_identity(s) == *serial)
                })
                .map_or(
                    "video format unavailable (older daemon or unknown input)",
                    |s| match s["video_format"].as_str() {
                        Some("raw") => "received-frame counter unavailable",
                        Some("jpeg") => "decoded-frame counter unavailable",
                        _ => "video format unavailable (older daemon or unknown input)",
                    },
                );
            detail.push(format!(
                "{serial}: video not verified ({counter}){connection}; check daemon logs; {}",
                RESTART.trim_start_matches("then ")
            ));
        } else if !old.contains_key(serial) && new.contains_key(serial) {
            pending = true;
            detail.push(match &connected {
                Ok(_) => format!("{serial} is newly connected; waiting for advancing video counters"),
                Err(error) => format!("{serial} is newly reported by the daemon; its USB connection couldn't be checked: {error}; check USB access and daemon logs; waiting for advancing video counters"),
            });
        } else if old
            .get(serial)
            .zip(new.get(serial))
            .is_some_and(|(&a, &b)| b.kind != a.kind || b.value < a.value)
        {
            pending = true;
            detail.push(format!(
                "{serial}'s video counter restarted; waiting for advancing video counters"
            ));
        } else {
            failed = true;
            if new
                .get(serial)
                .is_some_and(|c| c.kind == status::VideoCounterKind::JpegDecoded)
            {
                let received = status::received_frame_counters(current);
                let before = status::received_frame_counters(previous);
                let capture = if before
                    .get(serial)
                    .zip(received.get(serial))
                    .is_some_and(|(a, b)| b > a)
                {
                    "encoded video is arriving but no usable decoded image advanced"
                } else {
                    "no usable decoded image advanced"
                };
                let connection = connected.as_ref().err().map_or(String::new(), |error| {
                    format!("; its USB connection couldn't be checked: {error}")
                });
                detail.push(format!("{serial}: {capture}; check daemon logs for JPEG decode errors or delayed analysis{connection}"));
            } else if connected.as_ref().is_ok_and(|list| list.contains(serial)) {
                detail.push(format!("{serial} is connected but sending no video; USB bandwidth may be exhausted: plug it into a different USB controller, {RESTART}; also check USB access and daemon logs"));
            } else if new.contains_key(serial) {
                detail.push(format!("{serial}'s video counter is stalled; its USB connection couldn't be checked: {}; check its USB cable, USB access and daemon logs; after reconnecting, {RESTART}", connected.as_ref().unwrap_err()));
            } else {
                detail.push(format!("{serial} isn't reported by the daemon; its USB connection couldn't be checked: {}; check its USB cable and daemon logs; after reconnecting, {RESTART}", connected.as_ref().unwrap_err()));
            }
        }
    }
    let extra: BTreeSet<_> = reported
        .iter()
        .chain(connected.as_ref().ok().into_iter().flatten())
        .filter(|s| !calibrated.contains(*s) && !ignored.contains(*s))
        .collect();
    for serial in extra {
        detail.push(format!(
            "{serial} isn't calibrated: run cv1touch setup to use it for tracking"
        ));
    }
    if detail.is_empty() {
        detail.push("no calibrated sensors are enabled".into());
    }
    if active.is_none() {
        detail.push("older daemon: using the default calibration file".into());
    }
    let drop_notes = status::video_drop_notes(previous, current)
        .into_iter()
        .filter(|note| calibrated.contains(&note.serial) && !ignored.contains(&note.serial))
        .collect();
    Check {
        outcome: if failed {
            Outcome::Fail
        } else if pending {
            Outcome::Pending
        } else if unverified {
            Outcome::NotVerified
        } else {
            Outcome::Pass
        },
        detail: detail.join("; "),
        pending,
        drop_notes,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{json, Value};

    fn check(
        calibrated: Option<&[String]>,
        previous: Option<&Value>,
        current: Option<&Value>,
        connected: Result<&[String], &str>,
    ) -> (bool, String) {
        with_sources(
            || calibrated.map(<[String]>::to_vec),
            || connected.map(<[String]>::to_vec).map_err(str::to_owned),
            previous,
            current,
        )
        .conclusion()
    }

    fn snapshot(sensors: &[(&str, u32)]) -> Value {
        json!({"generation": 1, "mode": "calibrated", "engine_ready": true,
            "ignored_sensors": [], "sensor_frames_received": true, "sensor_total": sensors.len(),
            "sensors": sensors.iter().map(|(serial, frames)| json!({
                "serial": serial, "frames": frames, "frames_received": frames, "video_format": "raw", "frames_decoded": null, "frames_dropped": 0, "placed": 1, "fixes": [0, 0]
            })).collect::<Vec<_>>()})
    }

    fn cameras(names: &[&str]) -> Vec<String> {
        names.iter().map(|s| (*s).into()).collect()
    }

    #[test]
    fn sampling_skips_an_unreachable_daemon_but_keeps_the_connection_error() {
        for kind in [io::ErrorKind::NotFound, io::ErrorKind::ConnectionRefused] {
            let ((ok, detail), daemon) = sample_status(
                || Err(io::Error::new(kind, "fixture cannot connect")),
                |_| panic!("an unreachable daemon needs no further samples"),
                |first, second| {
                    with_sources(
                        || panic!("no calibration read"),
                        || panic!("no USB enumeration"),
                        first,
                        second,
                    )
                },
            );
            assert!(ok && detail == "skipped: daemon isn't running", "{detail}");
            let error = daemon.expect_err("the daemon check must keep the connection error");
            assert_eq!(error.kind(), kind);
            assert_eq!(error.to_string(), "fixture cannot connect");
        }
    }

    #[test]
    fn sampling_keeps_transport_errors_instead_of_skipping_video() {
        for fail_at in [0, 1] {
            let mut reads = 0;
            let mut waits = 0;
            let ((ok, detail), daemon) = sample_status(
                || {
                    let at = reads;
                    reads += 1;
                    if at == fail_at {
                        Err(io::Error::new(
                            io::ErrorKind::InvalidInput,
                            "the path of its control socket is too long: use a shorter XDG_RUNTIME_DIR",
                        ))
                    } else {
                        Ok(Some(json!({})))
                    }
                },
                |_| waits += 1,
                |_, current| {
                    if current.is_some() {
                        Check::new(Outcome::Pending, "waiting for a second video sample")
                    } else {
                        Check::new(Outcome::Skipped, "daemon isn't running")
                    }
                },
            );
            assert!(!ok, "a failed transport must not skip the video check");
            assert!(
                detail.contains("not verified") && detail.contains("XDG_RUNTIME_DIR"),
                "{detail}"
            );
            let error = daemon.expect_err("doctor's daemon check must keep the request error");
            assert_eq!(error.kind(), io::ErrorKind::InvalidInput);
            assert!(error.to_string().contains("XDG_RUNTIME_DIR"));
            assert_eq!((reads, waits), (fail_at + 1, fail_at));
        }
    }

    #[test]
    fn missing_calibrated_sensor_fails_even_when_no_sensor_is_reported() {
        let expected = cameras(&["CAMERA1"]);
        let empty = snapshot(&[]);
        let (ok, detail) = check(Some(&expected), Some(&empty), Some(&empty), Ok(&[]));
        assert!(!ok, "{detail}");
        assert!(
            detail.contains("CAMERA1 isn't connected") && detail.contains("USB cable"),
            "{detail}"
        );
    }

    #[test]
    fn a_connected_camera_with_stalled_or_absent_counters_is_silent() {
        let expected = cameras(&["CAMERA1"]);
        for current in [snapshot(&[("CAMERA1", 40)]), snapshot(&[])] {
            let old = current.clone();
            let (ok, detail) = check(Some(&expected), Some(&old), Some(&current), Ok(&expected));
            assert!(!ok && detail.contains("sending no video"), "{detail}");
            assert!(
                detail.contains("bandwidth may") && detail.contains("different USB controller"),
                "{detail}"
            );
        }
    }

    #[test]
    fn advancing_frames_pass_without_any_controller_pose_fixes() {
        let expected = cameras(&["CAMERA1"]);
        let old = snapshot(&[("CAMERA1", 40)]);
        let new = snapshot(&[("CAMERA1", 92)]);
        let (ok, detail) = check(Some(&expected), Some(&old), Some(&new), Ok(&expected));
        assert!(ok && detail.contains("CAMERA1 is streaming"), "{detail}");
    }

    #[test]
    fn daemon_down_is_skipped_with_the_reason() {
        let expected = cameras(&["CAMERA1"]);
        let (ok, detail) = check(Some(&expected), None, None, Ok(&[]));
        assert!(ok && detail == "skipped: daemon isn't running", "{detail}");
    }

    #[test]
    fn extra_uncalibrated_sensors_are_named_without_failing() {
        let expected = cameras(&["CAMERA1"]);
        let old = snapshot(&[("CAMERA1", 40), ("EXTRA", 0)]);
        let new = snapshot(&[("CAMERA1", 92), ("EXTRA", 0)]);
        let connected = cameras(&["CAMERA1", "EXTRA", "EXTRAUSB"]);
        let (ok, detail) = check(Some(&expected), Some(&old), Some(&new), Ok(&connected));
        assert!(
            ok && detail.contains("EXTRA isn't calibrated")
                && detail.contains("EXTRAUSB isn't calibrated"),
            "{detail}"
        );
        assert!(detail.contains("cv1touch setup"), "{detail}");
    }

    #[test]
    fn configured_ignored_cameras_are_not_required_or_reported_as_extra() {
        let expected = cameras(&["CAMERA1", "IGNORED"]);
        let old = snapshot(&[("CAMERA1", 40)]);
        let mut new = snapshot(&[("CAMERA1", 92)]);
        new["ignored_sensors"] = json!(["IGNORED", "OTHER"]);
        let connected = cameras(&["CAMERA1", "IGNORED", "OTHER"]);
        let (ok, detail) = check(Some(&expected), Some(&old), Some(&new), Ok(&connected));
        assert!(
            ok && !detail.contains("IGNORED") && !detail.contains("OTHER"),
            "{detail}"
        );
    }

    #[test]
    fn reconnects_restarts_and_counter_wrap_do_not_look_like_stopped_video() {
        let expected = cameras(&["CAMERA1"]);
        let old = snapshot(&[]);
        let new = snapshot(&[("CAMERA1", 52)]);
        let (ok, detail) = check(Some(&expected), Some(&old), Some(&new), Ok(&expected));
        assert!(
            !ok && detail.contains("not verified") && detail.contains("newly connected"),
            "{detail}"
        );
        let just_opened = snapshot(&[("CAMERA1", 0)]);
        let (ok, detail) = check(
            Some(&expected),
            Some(&old),
            Some(&just_opened),
            Ok(&expected),
        );
        assert!(
            !ok && detail.contains("not verified") && detail.contains("newly connected"),
            "{detail}"
        );
        let mut restarted = new.clone();
        restarted["generation"] = json!(2);
        let (ok, detail) = check(Some(&expected), Some(&new), Some(&restarted), Ok(&expected));
        assert!(
            !ok && detail.starts_with("not verified:") && detail.contains("changed"),
            "{detail}"
        );
        let old = snapshot(&[("CAMERA1", u32::MAX - 10)]);
        let new = snapshot(&[("CAMERA1", 40)]);
        let (ok, detail) = check(Some(&expected), Some(&old), Some(&new), Ok(&expected));
        assert!(!ok && detail.contains("restarted"), "{detail}");
        let later = snapshot(&[("CAMERA1", 92)]);
        let (ok, detail) = check(Some(&expected), Some(&new), Some(&later), Ok(&expected));
        assert!(ok && detail.contains("is streaming"), "{detail}");
    }

    #[test]
    fn older_daemons_and_unavailable_usb_listing_do_not_invent_a_disconnect() {
        let expected = cameras(&["CAMERA1"]);
        let mut old = snapshot(&[("CAMERA1", 40)]);
        old.as_object_mut().unwrap().remove("ignored_sensors");
        let (ok, detail) = check(
            Some(&expected),
            Some(&old),
            Some(&old),
            Err("permission denied"),
        );
        assert!(
            !ok && detail.contains("video counter is stalled"),
            "{detail}"
        );
        let empty = snapshot(&[]);
        let (ok, detail) = check(
            Some(&expected),
            Some(&empty),
            Some(&empty),
            Err("permission denied"),
        );
        assert!(
            !ok && detail.contains("USB connection couldn't be checked"),
            "{detail}"
        );
        assert!(!detail.contains("isn't connected"), "{detail}");
    }
    #[test]
    fn the_active_calibration_overrides_the_default_file_including_an_empty_set() {
        let default = cameras(&["CAMERA1"]);
        let mut current = snapshot(&[("CAMERA1", 40)]);
        current["calibrated_sensors"] = json!(["CAMERA1", "CAMERA2"]);
        let (ok, detail) = check(Some(&default), Some(&current), Some(&current), Ok(&default));
        assert!(
            !ok && detail.contains("CAMERA2 isn't connected"),
            "{detail}"
        );
        current["calibrated_sensors"] = json!([]);
        let (ok, detail) = check(Some(&default), Some(&current), Some(&current), Ok(&[]));
        assert!(ok && !detail.contains("isn't connected"), "{detail}");
        current["calibrated_sensors"] = json!(null);
        let (ok, detail) = check(Some(&default), Some(&current), Some(&current), Ok(&[]));
        assert!(
            ok && detail.contains("no sensor calibration is loaded"),
            "{detail}"
        );
    }

    #[test]
    fn an_older_daemon_uses_the_default_calibration_explicitly() {
        let expected = cameras(&["CAMERA1", "CAMERA2"]);
        let current = snapshot(&[]);
        let (ok, detail) = check(Some(&expected), Some(&current), Some(&current), Ok(&[]));
        assert!(
            !ok && detail.contains("CAMERA2 isn't connected"),
            "{detail}"
        );
        assert!(
            detail.contains("older daemon: using the default calibration file"),
            "{detail}"
        );
    }

    #[test]
    fn detached_native_sensors_are_missing_even_if_their_counter_advances() {
        let expected = cameras(&["CAMERA1"]);
        let old = snapshot(&[("CAMERA1", 40)]);
        let new = snapshot(&[("CAMERA1", 92)]);
        let (ok, detail) = check(Some(&expected), Some(&old), Some(&new), Ok(&[]));
        assert!(!ok && detail.contains("isn't connected"), "{detail}");
        assert!(
            !detail.contains("streaming") && !detail.contains("bandwidth"),
            "{detail}"
        );
    }

    #[test]
    fn serial_identity_is_case_insensitive_and_deduplicated_everywhere() {
        let expected = cameras(&["Camera1", "CAMERA1", "Ignored"]);
        let mut old = snapshot(&[("camera1", 40)]);
        old["calibrated_sensors"] = json!(["Camera1", "CAMERA1", "Ignored"]);
        let mut new = snapshot(&[("CAMERA1", 92), ("EXTRA", 0), ("ignored", 0)]);
        new["ignored_sensors"] = json!(["IGNORED", "AlsoIgnored"]);
        new["calibrated_sensors"] = json!(["IGNORED", "CAMERA1"]);
        let connected = cameras(&[
            "Camera1",
            "CAMERA1",
            "Extra",
            "extra",
            "IGNORED",
            "alsoignored",
        ]);
        let (ok, detail) = check(Some(&expected), Some(&old), Some(&new), Ok(&connected));
        assert!(ok && detail.contains("is streaming"), "{detail}");
        assert_eq!(detail.matches("is streaming").count(), 1, "{detail}");
        assert_eq!(detail.matches("isn't calibrated").count(), 1, "{detail}");
        assert!(!detail.to_ascii_lowercase().contains("ignored"), "{detail}");
    }

    #[test]
    fn a_camera_appearing_at_zero_has_not_verified_video() {
        let expected = cameras(&["CAMERA1"]);
        let old = snapshot(&[]);
        let new = snapshot(&[("CAMERA1", 0)]);
        let (ok, detail) = check(Some(&expected), Some(&old), Some(&new), Ok(&expected));
        assert!(!ok && detail.contains("not verified"), "{detail}");
    }

    #[test]
    fn cable_recovery_advice_includes_restarting_the_daemon() {
        let expected = cameras(&["CAMERA1"]);
        let current = snapshot(&[("CAMERA1", 40)]);
        for connected in [&[][..], expected.as_slice()] {
            let (_, detail) = check(
                Some(&expected),
                Some(&current),
                Some(&current),
                Ok(connected),
            );
            assert!(detail.contains("restart"), "{detail}");
        }
    }

    fn sampled(snapshots: &[Value]) -> ((bool, String), usize, Vec<Duration>) {
        let expected = cameras(&["CAMERA1"]);
        let mut reads = 0;
        let mut waits = Vec::new();
        let (result, _) = sample(
            || {
                let value = snapshots
                    .get(reads)
                    .expect("unexpected extra status read")
                    .clone();
                reads += 1;
                Some(value)
            },
            |interval| waits.push(interval),
            |previous, current| {
                with_sources(
                    || Some(expected.clone()),
                    || Ok(expected.clone()),
                    previous,
                    current,
                )
            },
        );
        (result, reads, waits)
    }

    #[test]
    fn a_camera_appearing_at_zero_then_stalling_gets_a_third_spaced_sample_and_fails() {
        let empty = snapshot(&[]);
        let zero = snapshot(&[("CAMERA1", 0)]);
        let ((ok, detail), reads, waits) = sampled(&[empty, zero.clone(), zero]);
        assert!(!ok && detail.contains("sending no video"), "{detail}");
        assert!(!detail.contains("is streaming"), "{detail}");
        assert_eq!(reads, 3);
        assert_eq!(waits, vec![SAMPLE_INTERVAL; 2]);
    }

    #[test]
    fn appearing_and_reset_counters_only_pass_after_a_later_sample_advances() {
        for first in [snapshot(&[]), snapshot(&[("CAMERA1", 100)])] {
            let ((ok, detail), reads, waits) = sampled(&[
                first,
                snapshot(&[("CAMERA1", 0)]),
                snapshot(&[("CAMERA1", 20)]),
            ]);
            assert!(ok && detail.contains("is streaming"), "{detail}");
            assert_eq!(reads, 3);
            assert_eq!(waits, vec![SAMPLE_INTERVAL; 2]);
        }
    }

    #[test]
    fn repeated_counter_resets_finish_as_unverified_with_a_fixed_sampling_bound() {
        let values: Vec<_> = [100, 90, 80, 70]
            .map(|count| snapshot(&[("CAMERA1", count)]))
            .into();
        let ((ok, detail), reads, waits) = sampled(&values);
        assert!(!ok && detail.starts_with("not verified:"), "{detail}");
        assert!(!detail.contains("is streaming"), "{detail}");
        assert_eq!(reads, MAX_SAMPLES);
        assert_eq!(waits, vec![SAMPLE_INTERVAL; MAX_SAMPLES - 1]);
    }

    #[test]
    fn an_active_calibration_does_not_require_a_valid_default_file() {
        let mut old = snapshot(&[("CAMERA1", 40)]);
        old["calibrated_sensors"] = json!(["Camera1"]);
        let mut new = old.clone();
        new["sensors"][0]["frames"] = json!(92);
        new["sensors"][0]["frames_received"] = json!(92);
        let connected = cameras(&["CAMERA1"]);
        let (ok, detail) = check(None, Some(&old), Some(&new), Ok(&connected));
        assert!(ok && detail.contains("is streaming"), "{detail}");
        assert!(!detail.contains("default calibration"), "{detail}");
    }
    fn probed(first: Option<&Value>, second: Option<&Value>) -> (Check, usize, usize) {
        let mut fallback_reads = 0;
        let mut usb_lists = 0;
        let result = with_sources(
            || {
                fallback_reads += 1;
                Some(cameras(&["CAMERA1"]))
            },
            || {
                usb_lists += 1;
                Ok(cameras(&["CAMERA1"]))
            },
            first,
            second,
        );
        (result, fallback_reads, usb_lists)
    }

    #[test]
    fn board_rendering_keeps_a_new_or_reset_camera_checking() {
        let current = snapshot(&[("CAMERA1", 0)]);
        for previous in [snapshot(&[]), snapshot(&[("CAMERA1", 100)])] {
            let (result, _, _) = probed(Some(&previous), Some(&current));
            let line = result.board_line();
            assert!(line.starts_with("checking sensor_streams:"), "{line}");
            assert!(
                !line.contains("FAIL") && !line.contains("run cv1touch doctor again"),
                "{line}"
            );
        }
        for (frames, label) in [(20, "ok"), (0, "FAIL")] {
            let new = snapshot(&[("CAMERA1", frames)]);
            let (result, _, _) = probed(Some(&current), Some(&new));
            assert!(result.board_line().starts_with(label));
        }
    }

    #[test]
    fn modern_status_never_reads_the_fallback_calibration() {
        for serials in [json!(["CAMERA1"]), json!([]), json!(null)] {
            let mut current = snapshot(&[("CAMERA1", 40)]);
            current["calibrated_sensors"] = serials;
            let (_, fallback_reads, usb_lists) = probed(Some(&current), Some(&current));
            assert_eq!(fallback_reads, 0, "{}", current["calibrated_sensors"]);
            assert_eq!(
                usb_lists,
                usize::from(!current["calibrated_sensors"].is_null())
            );
        }
    }

    #[test]
    fn skipped_or_not_yet_comparable_samples_do_not_read_files_or_list_usb() {
        let ready = snapshot(&[("CAMERA1", 40)]);
        let mut starting = ready.clone();
        starting["engine_ready"] = json!(false);
        starting["calibrated_sensors"] = json!(["CAMERA1"]);
        let mut changed = ready.clone();
        changed["generation"] = json!(2);
        let mut calibrated = ready.clone();
        calibrated["calibrated_sensors"] = json!(["CAMERA2"]);
        for (first, second) in [
            (None, None),
            (Some(&ready), Some(&starting)),
            (None, Some(&ready)),
            (Some(&starting), Some(&ready)),
            (Some(&ready), Some(&changed)),
            (Some(&ready), Some(&calibrated)),
        ] {
            let (_, fallback_reads, usb_lists) = probed(first, second);
            assert_eq!((fallback_reads, usb_lists), (0, 0));
        }
    }

    #[test]
    fn ready_older_daemons_read_the_fallback_only_when_samples_can_be_compared() {
        let old = snapshot(&[("CAMERA1", 40)]);
        let new = snapshot(&[("CAMERA1", 92)]);
        let (result, fallback_reads, usb_lists) = probed(Some(&old), Some(&new));
        assert_eq!((fallback_reads, usb_lists), (1, 1));
        assert_eq!(result.outcome, Outcome::Pass);
        let mut usb_lists = 0;
        let result = with_sources(
            || None,
            || {
                usb_lists += 1;
                Ok(Vec::new())
            },
            Some(&old),
            Some(&new),
        );
        assert_eq!(result.outcome, Outcome::Skipped);
        assert_eq!(usb_lists, 0);
    }
    #[test]
    fn known_usb_start_errors_explain_silent_video_without_guessing() {
        let expected = cameras(&["CAMERA1"]);
        let old = snapshot(&[]);
        for (kind, error_code, os_error, wanted) in [
            ("likely_usb_bandwidth", -99, 28, "not enough USB bandwidth"),
            ("other", -3, 13, "USB video couldn't start"),
        ] {
            let mut current = old.clone();
            current["sensor_streams"] = json!([{"serial": "Camera1", "error_kind": kind,
                "error_code": error_code, "os_error": os_error, "alt_setting": null}]);
            let (ok, detail) = check(Some(&expected), Some(&old), Some(&current), Ok(&expected));
            assert!(!ok && detail.contains(wanted), "{detail}");
            assert!(!detail.contains("bandwidth may"), "{detail}");
            if kind == "likely_usb_bandwidth" {
                assert!(detail.contains("different USB controller"), "{detail}");
            } else {
                assert!(
                    detail.contains("error -3") && detail.contains("USB access"),
                    "{detail}"
                );
            }
        }
    }

    #[test]
    fn current_video_and_usb_presence_take_precedence_over_old_start_errors() {
        let expected = cameras(&["CAMERA1"]);
        let old = snapshot(&[("CAMERA1", 40)]);
        let mut current = snapshot(&[("CAMERA1", 92)]);
        current["sensor_streams"] =
            json!([{"serial": "CAMERA1", "error_kind": "likely_usb_bandwidth"}]);
        let (ok, detail) = check(Some(&expected), Some(&old), Some(&current), Ok(&expected));
        assert!(ok && detail.contains("is streaming"), "{detail}");
        current["sensors"] = json!([]);
        current["sensor_total"] = json!(0);
        let (ok, detail) = check(Some(&expected), Some(&old), Some(&current), Ok(&[]));
        assert!(
            !ok && detail.contains("isn't connected") && !detail.contains("bandwidth"),
            "{detail}"
        );
    }

    #[test]
    fn a_detached_camera_is_missing_even_if_the_retained_native_object_advanced() {
        let expected = cameras(&["CAMERA1"]);
        let old = snapshot(&[("CAMERA1", 40)]);
        let mut current = snapshot(&[("CAMERA1", 92)]);
        current["sensor_streams"] =
            json!([{"serial": "CAMERA1", "error_kind": "likely_usb_bandwidth"}]);
        let (ok, detail) = check(Some(&expected), Some(&old), Some(&current), Ok(&[]));
        assert!(
            !ok && detail.contains("isn't connected") && detail.contains("USB cable"),
            "{detail}"
        );
        assert!(
            !detail.contains("bandwidth") && !detail.contains("streaming"),
            "{detail}"
        );
    }
    #[test]
    fn unverified_usb_presence_labels_retained_errors_as_last_start_attempts() {
        let expected = cameras(&["CAMERA1"]);
        for kind in ["likely_usb_bandwidth", "other"] {
            let mut value = snapshot(&[("CAMERA1", 0)]);
            value["sensor_streams"] = json!([{"serial": "CAMERA1", "error_kind": kind,
                "error_code": -99, "os_error": 28, "alt_setting": null}]);
            let result = with_sources(
                || Some(expected.clone()),
                || Err("USB enumeration failed".into()),
                Some(&value),
                Some(&value),
            );
            let line = result.board_line();
            assert!(
                line.contains("USB connection couldn't be checked")
                    && line.contains("last start attempt"),
                "{line}"
            );
            assert!(
                !line.contains("isn't sending video") && !line.contains("is connected"),
                "{line}"
            );
        }
    }

    #[test]
    fn errno_based_bandwidth_advice_always_qualifies_the_cause() {
        let expected = cameras(&["CAMERA1"]);
        let mut value = snapshot(&[]);
        value["sensor_streams"] = json!([{"serial": "CAMERA1", "error_kind": "likely_usb_bandwidth",
            "error_code": -99, "os_error": 28, "alt_setting": null}]);
        let result = with_sources(
            || Some(expected.clone()),
            || Ok(expected.clone()),
            Some(&value),
            Some(&value),
        );
        let (_, detail) = result.conclusion();
        assert!(
            detail.contains("most likely not enough USB bandwidth on this controller"),
            "{detail}"
        );
    }

    #[test]
    fn a_last_successful_start_does_not_present_a_detached_camera_as_current() {
        let expected = cameras(&["CAMERA1"]);
        let old = snapshot(&[("CAMERA1", 40)]);
        let mut current = snapshot(&[("CAMERA1", 92)]);
        current["sensor_streams"] = json!([{"serial": "CAMERA1", "error_kind": "none",
            "error_code": 0, "os_error": 0, "alt_setting": 2}]);
        let result = || {
            with_sources(
                || Some(expected.clone()),
                || Ok(Vec::new()),
                Some(&old),
                Some(&current),
            )
        };
        let board = result().board_line();
        let (ok, report_detail) = result().conclusion();
        assert!(
            board.starts_with("FAIL") && board.contains("isn't connected"),
            "{board}"
        );
        assert!(
            !ok && report_detail.contains("isn't connected"),
            "{report_detail}"
        );
        assert!(!board.contains("is streaming") && !report_detail.contains("is streaming"));
    }

    #[test]
    fn unavailable_counters_do_not_hide_known_start_failures() {
        let mut current = snapshot(&[("CAMERA1", 40), ("CAMERA2", 0)]);
        current["calibrated_sensors"] = json!(["CAMERA1", "CAMERA2", "CAMERA3"]);
        for row in current["sensors"].as_array_mut().unwrap() {
            row["frames_received"] = json!(null);
        }
        current["sensor_streams"] = json!([{"serial":"CAMERA2", "error_kind":"likely_usb_bandwidth",
            "error_code":-99, "os_error":28, "alt_setting":null}]);
        let result = with_sources(
            || unreachable!(),
            || Ok(cameras(&["CAMERA1", "CAMERA2"])),
            Some(&current),
            Some(&current),
        );
        let line = result.board_line();
        assert!(line.starts_with("FAIL sensor_streams:"), "{line}");
        assert!(
            line.contains("CAMERA1") && line.contains("received-frame counter unavailable"),
            "{line}"
        );
        assert!(
            line.contains("CAMERA2 isn't sending video: most likely not enough USB bandwidth"),
            "{line}"
        );
        assert!(line.contains("CAMERA3 isn't connected"), "{line}");
    }

    #[test]
    fn stalled_counters_do_not_prove_usb_presence_when_enumeration_fails() {
        let expected = cameras(&["CAMERA1"]);
        for successful_start in [false, true] {
            let mut value = snapshot(&[("CAMERA1", 40)]);
            if successful_start {
                value["sensor_streams"] = json!([{
                    "serial": "CAMERA1", "scope": "last_start_attempt",
                    "error_kind": "none", "error_code": 0, "os_error": 0,
                    "alt_setting": 2
                }]);
            }
            let inspect = || {
                with_sources(
                    || Some(expected.clone()),
                    || Err("USB enumeration denied".into()),
                    Some(&value),
                    Some(&value),
                )
            };
            let line = inspect().board_line();
            let (ok, detail) = inspect().conclusion();
            assert!(!ok && line.starts_with("FAIL"), "{line}");
            for text in [line, detail] {
                assert!(text.contains("video counter is stalled"), "{text}");
                assert!(
                    text.contains("USB connection couldn't be checked: USB enumeration denied"),
                    "{text}"
                );
                assert!(
                    !text.contains("is connected") && !text.contains("isn't connected"),
                    "{text}"
                );
                assert!(
                    text.contains("USB cable")
                        && text.contains("daemon logs")
                        && text.contains("restart"),
                    "{text}"
                );
            }
        }
    }

    #[test]
    fn confirmed_failures_take_precedence_over_pending_cameras_on_the_board() {
        let expected = cameras(&["CAMERA1", "CAMERA2"]);
        let previous = snapshot(&[("CAMERA2", 100)]);
        let current = snapshot(&[("CAMERA2", 0)]);
        let result = with_sources(
            || Some(expected),
            || Ok(cameras(&["CAMERA2"])),
            Some(&previous),
            Some(&current),
        );
        let line = result.board_line();
        assert!(line.starts_with("FAIL sensor_streams:"), "{line}");
        assert!(line.contains("CAMERA1 isn't connected"), "{line}");
        assert!(line.contains("CAMERA2's video counter restarted"), "{line}");
    }

    #[test]
    fn doctor_keeps_sampling_pending_cameras_while_another_camera_has_failed() {
        let expected = cameras(&["CAMERA1", "CAMERA2"]);
        for (counts, wanted) in [
            (vec![100, 0, 20], "CAMERA2 is streaming"),
            (vec![100, 90, 80, 70], "CAMERA2's video counter restarted"),
        ] {
            let mut reads = 0;
            let mut waits = Vec::new();
            let mut inspected = Vec::new();
            let ((ok, detail), _) = sample(
                || {
                    let value = snapshot(&[("CAMERA2", counts[reads])]);
                    reads += 1;
                    Some(value)
                },
                |interval| waits.push(interval),
                |previous, current| {
                    let result = with_sources(
                        || Some(expected.clone()),
                        || Ok(cameras(&["CAMERA2"])),
                        previous,
                        current,
                    );
                    inspected.push(result.outcome);
                    result
                },
            );
            assert!(
                !ok && detail.contains("CAMERA1 isn't connected"),
                "{detail}"
            );
            assert!(!detail.starts_with("not verified:"), "{detail}");
            assert!(detail.contains(wanted), "{detail}");
            assert_eq!(reads, counts.len());
            assert_eq!(waits, vec![SAMPLE_INTERVAL; counts.len() - 1]);
            assert!(inspected[1..]
                .iter()
                .all(|outcome| *outcome == Outcome::Fail));
        }
    }

    #[test]
    fn analysis_completing_without_received_video_cannot_pass() {
        let expected = cameras(&["CAMERA1"]);
        let mut old = snapshot(&[("CAMERA1", 40)]);
        let mut new = snapshot(&[("CAMERA1", 41)]);
        for value in [&mut old, &mut new] {
            value["sensor_frames_received"] = json!(true);
            value["sensors"][0]["frames_received"] = json!(40u64);
        }
        let (ok, detail) = check(Some(&expected), Some(&old), Some(&new), Ok(&expected));
        assert!(!ok && !detail.contains("is streaming"), "{detail}");
        assert!(detail.contains("sending no video"), "{detail}");
    }

    #[test]
    fn a_received_counter_reset_from_the_upper_u32_half_is_not_progress() {
        let expected = cameras(&["CAMERA1"]);
        let old = snapshot(&[("CAMERA1", 3_000_000_000)]);
        let new = snapshot(&[("CAMERA1", 0)]);
        let (ok, detail) = check(Some(&expected), Some(&old), Some(&new), Ok(&expected));
        assert!(
            !ok && detail.contains("not verified") && detail.contains("restarted"),
            "{detail}"
        );
    }

    #[test]
    fn received_counters_larger_than_u32_keep_advancing() {
        let expected = cameras(&["CAMERA1"]);
        let mut old = snapshot(&[("CAMERA1", 40)]);
        let mut new = old.clone();
        old["sensors"][0]["frames_received"] = json!(4_294_967_296u64);
        new["sensors"][0]["frames_received"] = json!(4_294_967_348u64);
        let (ok, detail) = check(Some(&expected), Some(&old), Some(&new), Ok(&expected));
        assert!(ok && detail.contains("is streaming"), "{detail}");
    }

    #[test]
    fn older_daemons_are_not_verified_without_blame_or_usb_queries() {
        let mut old = snapshot(&[("CAMERA1", 40)]);
        let mut new = snapshot(&[("CAMERA1", 92)]);
        for value in [&mut old, &mut new] {
            value
                .as_object_mut()
                .unwrap()
                .remove("sensor_frames_received");
            for sensor in value["sensors"].as_array_mut().unwrap() {
                for key in [
                    "frames_received",
                    "video_format",
                    "frames_decoded",
                    "frames_dropped",
                ] {
                    sensor.as_object_mut().unwrap().remove(key);
                }
            }
        }
        let (result, fallback_reads, usb_lists) = probed(Some(&old), Some(&new));
        let line = result.board_line();
        assert!(line.starts_with("not verified sensor_streams:"), "{line}");
        assert!(
            line.contains("video not verified: the daemon is older than this cv1touch"),
            "{line}"
        );
        assert!(
            line.contains("restart") && !line.contains("FAIL") && !line.contains("isn't connected"),
            "{line}"
        );
        assert_eq!((fallback_reads, usb_lists), (0, 0));
        let (ok, detail) =
            with_sources(|| None, || unreachable!(), Some(&old), Some(&new)).conclusion();
        assert!(!ok && detail.contains("the daemon is older"), "{detail}");
    }

    #[test]
    fn newly_reported_counters_keep_the_usb_enumeration_error() {
        let expected = cameras(&["CAMERA1"]);
        for successful_start in [false, true] {
            let old = snapshot(&[]);
            let mut new = snapshot(&[("CAMERA1", 0)]);
            if successful_start {
                new["sensor_streams"] = json!([{"serial":"CAMERA1", "scope":"last_start_attempt",
                    "error_kind":"none", "error_code":0, "os_error":0, "alt_setting":2}]);
            }
            let result = with_sources(
                || Some(expected.clone()),
                || Err("USB enumeration denied".into()),
                Some(&old),
                Some(&new),
            );
            let line = result.board_line();
            assert!(line.contains("newly reported by the daemon"), "{line}");
            assert!(
                line.contains("USB connection couldn't be checked: USB enumeration denied"),
                "{line}"
            );
            assert!(
                !line.contains("newly connected") && !line.contains("FAIL"),
                "{line}"
            );
        }
    }
    #[test]
    fn an_unavailable_received_counter_is_not_a_sensor_failure() {
        let mut value = snapshot(&[("CAMERA1", 40)]);
        value["sensors"][0]["frames_received"] = json!(null);
        value["calibrated_sensors"] = json!(["CAMERA1"]);
        let (result, fallback, usb) = probed(Some(&value), Some(&value));
        let line = result.board_line();
        assert!(line.starts_with("not verified sensor_streams:"), "{line}");
        assert!(
            line.contains("received-frame counter unavailable"),
            "{line}"
        );
        assert!(
            !line.contains("FAIL") && !line.contains("sending no video"),
            "{line}"
        );
        assert_eq!((fallback, usb), (0, 1));
    }
    #[test]
    fn unavailable_extra_and_ignored_counters_do_not_fail_required_video() {
        let mut old = snapshot(&[("CAMERA1", 40), ("CAMERA2", 0), ("CAMERA3", 0)]);
        let mut new = snapshot(&[("CAMERA1", 41), ("CAMERA2", 0), ("CAMERA3", 0)]);
        for value in [&mut old, &mut new] {
            value["calibrated_sensors"] = json!(["CAMERA1", "CAMERA3"]);
            value["ignored_sensors"] = json!(["camera3"]);
            value["sensors"][1]["frames_received"] = json!(null);
            value["sensors"][2]["frames_received"] = json!(null);
        }
        let result = with_sources(
            || unreachable!("active calibration"),
            || Ok(cameras(&["CAMERA1", "CAMERA2", "CAMERA3"])),
            Some(&old),
            Some(&new),
        );
        let (ok, detail) = result.conclusion();
        assert!(ok, "{detail}");
        assert!(
            detail.contains("CAMERA1 is streaming") && detail.contains("CAMERA2 isn't calibrated"),
            "{detail}"
        );
        assert!(!detail.contains("CAMERA3"), "{detail}");
    }
    #[test]
    fn an_unavailable_engine_is_sampled_to_the_budget_and_never_verifies_required_video() {
        for active in [true, false] {
            let mut current = snapshot(&[]);
            current["engine_ready"] = json!(false);
            if active {
                current["calibrated_sensors"] = json!(["CAMERA1"]);
            }
            let mut reads = 0;
            let mut waits = Vec::new();
            let mut outcomes = Vec::new();
            let ((ok, detail), _) = sample(
                || {
                    reads += 1;
                    Some(current.clone())
                },
                |interval| waits.push(interval),
                |old, new| {
                    let result = with_sources(
                        || Some(cameras(&["CAMERA1"])),
                        || panic!("engine unavailable: USB cannot establish video"),
                        old,
                        new,
                    );
                    outcomes.push(result.outcome);
                    result
                },
            );
            assert!(!ok && detail.contains("not verified"), "{detail}");
            assert!(
                detail.contains("tracking engine isn't running")
                    && detail.contains("headset")
                    && detail.contains("daemon log"),
                "{detail}"
            );
            assert!(
                !detail.contains("is streaming") && !detail.contains("sending no video"),
                "{detail}"
            );
            assert_eq!(reads, 4);
            assert_eq!(waits, [Duration::from_secs(1); 3]);
            assert_eq!(outcomes, [Outcome::Pending; 4]);
            let line = with_sources(
                || Some(cameras(&["CAMERA1"])),
                || unreachable!(),
                None,
                Some(&current),
            )
            .board_line();
            assert!(
                line.starts_with("checking sensor_streams:") && line.contains("headset"),
                "{line}"
            );
        }
    }

    #[test]
    fn engine_startup_or_reload_can_finish_with_verified_video() {
        for reloading in [false, true] {
            let mut starting = snapshot(&[]);
            starting["engine_ready"] = json!(false);
            starting["calibrated_sensors"] = json!(["CAMERA1"]);
            let mut ready = snapshot(&[("CAMERA1", 40)]);
            ready["calibrated_sensors"] = json!(["CAMERA1"]);
            let mut advancing = ready.clone();
            advancing["sensors"][0]["frames_received"] = json!(92);
            let snapshots = if reloading {
                vec![ready.clone(), starting, ready, advancing]
            } else {
                vec![starting.clone(), starting, ready, advancing]
            };
            let mut reads = 0;
            let mut waits = Vec::new();
            let ((ok, detail), _) = sample(
                || {
                    let next = snapshots[reads].clone();
                    reads += 1;
                    Some(next)
                },
                |interval| waits.push(interval),
                |old, new| with_sources(|| unreachable!(), || Ok(cameras(&["CAMERA1"])), old, new),
            );
            assert!(ok && detail == "CAMERA1 is streaming", "{detail}");
            assert_eq!(reads, 4);
            assert_eq!(waits, [Duration::from_secs(1); 3]);
        }
    }

    #[test]
    fn an_unavailable_engine_does_not_require_video_from_empty_or_ignored_calibrations() {
        for ignored in [false, true] {
            let mut current = snapshot(&[]);
            current["engine_ready"] = json!(false);
            current["calibrated_sensors"] = if ignored {
                json!(["CAMERA1"])
            } else {
                json!([])
            };
            current["ignored_sensors"] = json!(["camera1"]);
            let result = with_sources(
                || unreachable!("active calibration"),
                || unreachable!("no required cameras"),
                None,
                Some(&current),
            );
            assert_eq!(result.outcome, Outcome::Skipped);
            let (ok, detail) = result.conclusion();
            assert!(
                ok && detail.contains("no calibrated sensors are enabled"),
                "{detail}"
            );
        }
    }

    #[test]
    fn unavailable_counters_keep_other_missing_and_silent_cameras_visible() {
        let mut current = snapshot(&[("CAMERA1", 0), ("CAMERA2", 40), ("CAMERA3", 0)]);
        current["calibrated_sensors"] = json!(["CAMERA1", "CAMERA2", "CAMERA3"]);
        current["sensors"][0]["frames_received"] = json!(null);
        current["sensors"][1]["frames_received"] = json!(null);
        current["sensor_streams"] = json!([{"serial":"CAMERA3", "error_kind":"other",
            "error_code":-3, "os_error":13, "alt_setting":null}]);
        let result = with_sources(
            || unreachable!(),
            || Ok(cameras(&["CAMERA2", "CAMERA3"])),
            Some(&current),
            Some(&current),
        );
        assert_eq!(result.outcome, Outcome::Fail);
        assert!(!result.pending);
        let line = result.board_line();
        assert!(line.starts_with("FAIL sensor_streams:"), "{line}");
        assert!(line.contains("CAMERA1 isn't connected"), "{line}");
        assert!(
            line.contains("CAMERA2") && line.contains("received-frame counter unavailable"),
            "{line}"
        );
        assert!(
            line.contains("CAMERA3") && line.contains("video") && line.contains("daemon logs"),
            "{line}"
        );
        let (ok, detail) = with_sources(
            || unreachable!(),
            || Ok(cameras(&["CAMERA2", "CAMERA3"])),
            Some(&current),
            Some(&current),
        )
        .conclusion();
        assert!(
            !ok && detail.contains("CAMERA1 isn't connected") && detail.contains("CAMERA2"),
            "{detail}"
        );
    }

    #[test]
    fn an_unavailable_counter_preserves_the_usb_enumeration_error() {
        let mut current = snapshot(&[("CAMERA1", 40)]);
        current["calibrated_sensors"] = json!(["CAMERA1"]);
        current["sensors"][0]["frames_received"] = json!(null);
        let result = with_sources(
            || unreachable!(),
            || Err("USB enumeration denied".into()),
            Some(&current),
            Some(&current),
        );
        assert_eq!(result.outcome, Outcome::NotVerified);
        let line = result.board_line();
        assert!(
            line.contains("CAMERA1") && line.contains("received-frame counter unavailable"),
            "{line}"
        );
        assert!(
            line.contains("USB connection couldn't be checked: USB enumeration denied"),
            "{line}"
        );
        assert!(
            !line.contains("is connected") && !line.contains("sending no video"),
            "{line}"
        );
    }
    fn quality_sample(received: u64, decoded: Value, format: Value, dropped: Value) -> Value {
        let mut sample = snapshot(&[("CAMERA1", 0)]);
        sample["sensors"][0]["frames_received"] = received.into();
        sample["sensors"][0]["frames_decoded"] = decoded;
        sample["sensors"][0]["video_format"] = format;
        sample["sensors"][0]["frames_dropped"] = dropped;
        sample["sensor_video_quality"] = json!(true);
        sample
    }

    #[test]
    fn quality_jpeg_capture_without_decoding_cannot_pass() {
        let cameras = cameras(&["CAMERA1"]);
        let old = quality_sample(10, json!(0), json!("jpeg"), json!(0));
        let new = quality_sample(62, json!(0), json!("jpeg"), json!(7));
        let (ok, detail) = check(Some(&cameras), Some(&old), Some(&new), Ok(&cameras));
        assert!(!ok && detail.contains("decoded"), "{detail}");
        assert!(!detail.contains("bandwidth may be exhausted"), "{detail}");
    }

    #[test]
    fn quality_jpeg_decode_progress_is_independent_of_capture_and_released_buffers() {
        let cameras = cameras(&["CAMERA1"]);
        let old = quality_sample(100, json!(4_294_967_296u64), json!("jpeg"), json!(1000));
        let new = quality_sample(100, json!(4_294_967_348u64), json!("jpeg"), json!(1000));
        assert!(check(Some(&cameras), Some(&old), Some(&new), Ok(&cameras)).0);
    }

    #[test]
    fn quality_unavailable_decoding_and_unknown_format_are_not_received_fallbacks() {
        let cameras = cameras(&["CAMERA1"]);
        for format in [json!("jpeg"), Value::Null, json!("other")] {
            let old = quality_sample(10, Value::Null, format.clone(), json!(0));
            let new = quality_sample(62, Value::Null, format, json!(0));
            let result = with_sources(
                || Some(cameras.clone()),
                || Ok(cameras.clone()),
                Some(&old),
                Some(&new),
            );
            assert_eq!(result.outcome, Outcome::NotVerified, "{}", result.detail);
        }
        let mut old = snapshot(&[("CAMERA1", 10)]);
        let mut new = snapshot(&[("CAMERA1", 62)]);
        old["sensors"][0]
            .as_object_mut()
            .unwrap()
            .remove("video_format");
        new["sensors"][0]
            .as_object_mut()
            .unwrap()
            .remove("video_format");
        assert!(!check(Some(&cameras), Some(&old), Some(&new), Ok(&cameras)).0);
    }

    #[test]
    fn quality_format_switch_starts_a_new_baseline_even_when_values_rise() {
        let cameras = cameras(&["CAMERA1"]);
        let old = quality_sample(10, Value::Null, json!("raw"), json!(0));
        let new = quality_sample(62, json!(1000), json!("jpeg"), json!(0));
        let result = with_sources(
            || Some(cameras.clone()),
            || Ok(cameras.clone()),
            Some(&old),
            Some(&new),
        );
        assert_eq!(result.outcome, Outcome::Pending, "{}", result.detail);
        let later = quality_sample(62, json!(1052), json!("jpeg"), json!(0));
        assert!(check(Some(&cameras), Some(&new), Some(&later), Ok(&cameras)).0);
    }

    #[test]
    fn quality_rejections_warn_without_failing_and_never_use_received_minus_dropped() {
        let cameras = cameras(&["CAMERA1"]);
        for rejected in [0, 1, 2, 3, 7] {
            let old = quality_sample(10, Value::Null, json!("raw"), json!(1000));
            let new = quality_sample(62, Value::Null, json!("raw"), json!(1000 + rejected));
            let result = with_sources(
                || Some(cameras.clone()),
                || Ok(cameras.clone()),
                Some(&old),
                Some(&new),
            );
            assert_eq!(result.outcome, Outcome::Pass, "{}", result.detail);
            let line = result.board_line();
            assert_eq!(line.starts_with("warning "), rejected >= 3, "{line}");
            if rejected > 0 {
                assert!(line.contains(&format!("discarded {rejected} ")), "{line}");
            }
        }
        let old = quality_sample(10, Value::Null, json!("raw"), json!(1000));
        let reset = quality_sample(62, Value::Null, json!("raw"), json!(0));
        let (ok, detail) = check(Some(&cameras), Some(&old), Some(&reset), Ok(&cameras));
        assert!(ok && !detail.contains("discarded"), "{detail}");
    }

    #[test]
    fn quality_missing_jpeg_metadata_keeps_another_sensors_known_failure() {
        let cameras = cameras(&["CAMERA1", "CAMERA2"]);
        let old = quality_sample(10, Value::Null, json!("jpeg"), json!(0));
        let mut new = quality_sample(62, Value::Null, json!("jpeg"), json!(0));
        new["sensor_streams"] = json!([{"serial":"camera2", "error_kind":"likely_usb_bandwidth", "error_code":-99, "os_error":28}]);
        let (ok, detail) = check(Some(&cameras), Some(&old), Some(&new), Ok(&cameras));
        assert!(
            !ok && detail.contains("CAMERA1")
                && detail.contains("decoded-frame counter unavailable"),
            "{detail}"
        );
        assert!(
            detail.contains("CAMERA2") && detail.contains("most likely not enough USB bandwidth"),
            "{detail}"
        );
    }
    #[test]
    fn quality_mixed_video_is_matched_by_serial_and_uses_each_producing_stage() {
        let cameras = cameras(&["CAMERA1", "CAMERA2"]);
        let mut old = quality_sample(10, Value::Null, json!("raw"), json!(0));
        old["calibrated_sensors"] = json!(cameras);
        old["sensors"].as_array_mut().unwrap().push(json!({"serial":"camera2",
            "video_format":"jpeg", "frames_received":100, "frames_decoded":4294967296u64,"frames_dropped":0}));
        let mut new = old.clone();
        new["sensors"][0]["frames_received"] = json!(62);
        new["sensors"][1]["frames_decoded"] = json!(4294967348u64);
        new["sensors"].as_array_mut().unwrap().reverse();
        let (ok, detail) = check(None, Some(&old), Some(&new), Ok(&cameras));
        assert!(
            ok && detail.contains("CAMERA1 is streaming")
                && detail.contains("CAMERA2 is streaming"),
            "{detail}"
        );
        new["sensors"][0]["frames_decoded"] = json!(0);
        assert_eq!(
            with_sources(|| None, || Ok(cameras.clone()), Some(&old), Some(&new)).outcome,
            Outcome::Pending
        );
    }

    #[test]
    fn quality_decode_unavailability_keeps_connection_errors_and_requires_a_new_sample() {
        let cameras = cameras(&["CAMERA1"]);
        let old = quality_sample(10, Value::Null, json!("jpeg"), json!(0));
        let mut new = quality_sample(62, Value::Null, json!("jpeg"), json!(0));
        let result = with_sources(
            || Some(cameras.clone()),
            || Err("USB enumeration denied".into()),
            Some(&old),
            Some(&new),
        );
        assert_eq!(result.outcome, Outcome::NotVerified);
        assert!(result.detail.contains("USB enumeration denied"));
        new["sensors"][0]["frames_decoded"] = json!(0);
        assert_eq!(
            with_sources(
                || Some(cameras.clone()),
                || Ok(cameras.clone()),
                Some(&old),
                Some(&new)
            )
            .outcome,
            Outcome::Pending
        );
    }

    #[test]
    fn quality_warning_is_non_failing_but_never_hides_other_failures_or_ignored_sensors() {
        let cameras = cameras(&["CAMERA1", "CAMERA2"]);
        let old = quality_sample(10, Value::Null, json!("raw"), json!(0));
        let mut new = quality_sample(62, Value::Null, json!("raw"), json!(7));
        let (ok, detail) = check(Some(&cameras[..1]), Some(&old), Some(&new), Ok(&cameras));
        assert!(
            ok && detail.starts_with("warning: ") && detail.contains("discarded 7"),
            "{detail}"
        );
        let result = with_sources(
            || Some(cameras.clone()),
            || Ok(cameras[..1].to_vec()),
            Some(&old),
            Some(&new),
        );
        assert!(result.board_line().starts_with("FAIL "));
        let (ok, detail) = check(Some(&cameras), Some(&old), Some(&new), Ok(&cameras[..1]));
        assert!(
            !ok && detail.contains("CAMERA2 isn't connected") && detail.contains("discarded 7")
        );
        new["ignored_sensors"] = json!(["camera1"]);
        let (_, detail) = check(Some(&cameras), Some(&old), Some(&new), Ok(&cameras[..1]));
        assert!(!detail.contains("discarded"));
    }
    #[test]
    fn quality_sampling_retains_rejections_across_pending_then_pass() {
        let cameras = cameras(&["CAMERA1", "CAMERA2"]);
        let mut reads = 0;
        let mut waits = Vec::new();
        let mut outcomes = Vec::new();
        let ((ok, detail), _) = sample(
            || {
                let (a, b, dropped) = [(10, 100, 0), (62, 0, 7), (114, 52, 7)][reads];
                let mut value = snapshot(&[("CAMERA1", a), ("CAMERA2", b)]);
                value["calibrated_sensors"] = json!(cameras);
                value["sensors"][0]["frames_dropped"] = json!(dropped);
                reads += 1;
                Some(value)
            },
            |interval| waits.push(interval),
            |previous, current| {
                let result = with_sources(|| None, || Ok(cameras.clone()), previous, current);
                outcomes.push(result.outcome);
                result
            },
        );
        assert_eq!(
            outcomes,
            [Outcome::Pending, Outcome::Pending, Outcome::Pass]
        );
        assert_eq!(waits, [SAMPLE_INTERVAL; 2]);
        assert!(ok && detail.starts_with("warning: "), "{detail}");
        assert_eq!(
            detail.matches("CAMERA1 discarded 7 damaged images").count(),
            1,
            "{detail}"
        );
        assert!(
            detail.contains("CAMERA1 is streaming") && detail.contains("CAMERA2 is streaming"),
            "{detail}"
        );
        assert!(
            !detail.contains("since the previous video sample"),
            "an earlier interval must not be presented as the latest one: {detail}"
        );
        assert!(
            !detail.contains("counter restarted"),
            "only rejection observations survive: {detail}"
        );
    }
    #[test]
    fn quality_sampling_keeps_each_intervals_count_and_warning_threshold() {
        let cameras = cameras(&["CAMERA1", "CAMERA2"]);
        for drops in [vec![2u64, 2], vec![7, 2, 1]] {
            let mut reads = 0;
            let ((ok, detail), _) = sample(
                || {
                    let b = if reads == drops.len() {
                        100 - (reads as u32 - 1) * 20 + 1
                    } else {
                        100 - reads as u32 * 20
                    };
                    let mut value =
                        snapshot(&[("CAMERA1", 10 + reads as u32 * 52), ("CAMERA2", b)]);
                    value["calibrated_sensors"] = json!(cameras);
                    value["sensors"][0]["frames_dropped"] =
                        json!(drops[..reads].iter().sum::<u64>());
                    reads += 1;
                    Some(value)
                },
                |_| {},
                |previous, current| {
                    with_sources(|| None, || Ok(cameras.clone()), previous, current)
                },
            );
            assert!(ok, "{detail}");
            assert_eq!(reads, drops.len() + 1);
            assert_eq!(
                detail.starts_with("warning: "),
                drops.iter().any(|n| *n >= 3),
                "warnings use individual intervals, not their sum: {detail}"
            );
            assert_eq!(detail.matches("discarded").count(), drops.len(), "{detail}");
            for count in &drops {
                assert_eq!(
                    detail
                        .matches(&format!("discarded {count} damaged image"))
                        .count(),
                    drops.iter().filter(|n| *n == count).count(),
                    "{detail}"
                );
            }
        }
    }
}
