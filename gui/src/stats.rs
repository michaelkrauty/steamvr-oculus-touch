// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The Stats tab: every figure the daemon and the saved calibration report.

use super::*;

/// What the figure named `key` measures and what its values mean.
fn tip(key: &str) -> &'static str {
    match key {
        // Controllers.
        "Optical rate" => "How many times a second the sensors' video updates this controller's position. It follows the sensors' frame rate while a sensor sees the controller, and drops to 0 when none does or the controller sleeps.",
        "Flags" => "The controller's state bits from the daemon: 0x01 heard from, 0x02 in radio range, 0x04 position tracked, 0x08 radio gap, 0x10 asleep. 0x07 means awake, in range and tracked.",
        "Battery" => "The battery level the controller last reported. It is only known while the controller is awake; replace the AA battery when it gets low.",
        "Radio loss" => "The share of the controller's radio packets that did not arrive recently. A few percent is normal; above 10% its motion can stutter, so bring it closer to the headset or clear the way between them.",
        "Tracking" => "Whether the tracker holds the controller's position from the sensors' video right now. No while it is out of every sensor's view or asleep.",
        "updates" => "Sensor frames that updated the controller's position. It keeps rising while the controller is tracked.",
        "frames" => "Sensor frames the tracker looked at for this controller.",
        "acquisitions" => "How many times the tracker found the controller afresh, after a wake or a lost lock. One per wake is ideal; a steadily rising count means it keeps losing the controller.",
        "good" => "Acquisitions from a pose fitted to one sensor view, rated good rather than strong. Such a pose is used only once a second sensor or a strong pose confirms it.",
        "wrong_locks" => "Tracks dropped because the controller's position had locked on wrongly, shown by its accelerometer bias running away. This should stay 0.",
        "refused" => "Poses fitted to a sensor view, refused at acquisition because their orientation disagreed with the controller's gyroscope. This should stay at or near 0.",
        "unconfirmed" => "Acquisitions dropped because no second sensor or strong pose confirmed them in time. Games never saw these.",
        "rejected" => "Sensor frames whose LED fit did not match the controller well enough to use. Some are normal, more with LEDs partly hidden.",
        "out_of_order" => "Sensor frames that arrived after a newer measurement and were fitted in behind it. A few are normal.",
        "too_few" => "Sensor frames, while tracking, with too few LEDs matched to fit the position. It rises while the controller's LEDs are partly hidden.",
        "not_visible" => "Sensor frames in which no LED of the controller faced that sensor, so there was nothing to see.",
        "searched" => "Acquisitions found by the tracker's own search of the video, using the controller's known orientation.",
        "Accel bias" => "The size of the accelerometer error the tracker has estimated for this controller, which it corrects for. A value that keeps growing ends the track as a wrong lock.",
        "Bias x, y, z" => "The estimated accelerometer error along each of the controller's axes, in m/s².",
        "Verified bias" => "Whether this controller's accelerometer error has been confirmed and saved for later sessions, or is still being learned.",
        // Sensors.
        "Serial" => "The sensor's serial number, as printed on it and used in its calibration.",
        "Video" => "How the sensor sends video: raw is uncompressed, on a USB 3 port; jpeg is compressed, on a USB 2 port, and is decoded before tracking. The alt setting is the USB bandwidth mode its video uses.",
        "Frame rate" => "Video frames a second received from the sensor. All sensors should show the same steady rate; a lower one or 0 means its video is dropping.",
        "Dropped" => "Video frames from this sensor that were lost or could not be used. A few are normal; a rising count points to a USB bandwidth or cable problem.",
        "Fixes L / R" => "How many times this sensor's video fixed the left and the right controller's position. Both rise while the controllers are in its view.",
        "Agreement" => "How closely this sensor agrees with the other sensors on the controllers' positions, in pixels of its image: the median difference and the steady offset (bias). A median at or under 0.35 px is good, up to 0.60 px fair; more means the sensor may have moved.",
        "Checks" => "How often this sensor's view was compared with the other sensors', and how many comparisons disagreed. A small share of misses is normal.",
        "Calibration fit" => "How well the room calibration fits this sensor: the RMS error in pixels of the LEDs it saw during calibration, and how many views it used. Lower is better.",
        "Position" => "Where the room calibration puts this sensor, in metres from the room's origin (x, y, z).",
        "Pose" => "Whether this sensor's place in the room is in use: placed is calibrated and tracking, moved means it was bumped and is being placed again, released means it is accepted for now with a recheck to come, and given_up means no pose fits its view, so it is left out.",
        "Landmark shift" => "How far the fixed bright spots in this sensor's view have moved in its image since they were learned. Near 0 means the sensor has not moved; a growing shift suggests it was bumped.",
        // Room calibration.
        "Gravity spread" => "How much the controllers' measurements of which way is down disagreed during calibration, in degrees. Lower is better: the floor in VR is levelled from these.",
        "Gravity" => "The strength of gravity the calibration measured. Close to 9.81 m/s² means the controllers' accelerometers read true.",
        "Motion segments" => "Stretches of controller motion the room calibration used.",
        "Samples" => "LED sightings the room calibration used. More samples from more places in the room make a sturdier calibration.",
        "Rejected views" => "Sensor views the room calibration left out because they did not fit the others.",
        "Reference sensor" => "The sensor the room calibration measures the others from.",
        // Headset alignment.
        "Position RMS" => "How far, as an RMS in millimetres, the controllers' tracking and the headset's disagreed after alignment. Lower is better.",
        "Rotation RMS" => "How far, as an RMS in degrees, the controllers' and the headset's orientations disagreed after alignment. Lower is better.",
        "Pairs" => "Matched samples of controller and headset motion the alignment used. More pairs make a sturdier alignment.",
        "Reference system" => "The tracking the controllers are aligned with: lighthouse for base station tracking, or the headset's own tracking.",
        "Tracking base" => "The base station SteamVR tracked the headset from when it was aligned.",
        "Measured stations" => "How many of SteamVR's base stations the headset alignment measured, of those it saw.",
        "Universe" => "SteamVR's ID for the tracked space. If it changes, Room Setup was run again and the headset alignment may need redoing.",
        "Base pin" => "The base station cv1touch has SteamVR track the headset from, so that the alignment stays valid, and who set it.",
        // Controller calibration.
        "Left model offset" | "Right model offset" => "How well the controller's 3D model lines up with its tracked LEDs, as an RMS in millimetres, and how many LEDs the fit used. Lower is better.",
        "Left input" | "Right input" => "Whether the controller's trigger, grip and thumbstick ranges use your input calibration or the factory values.",
        // Daemon.
        "Mode" => "What the daemon tracks with: calibrated is the saved room calibration, provisional a rough placement of the sensors before there is one, and capture is setup recording calibration data.",
        "Tracker" => "Whose controller poses the daemon publishes: cv1touch's own tracker (Rust) or OpenHMD's.",
        "Tilt" => "The angle the controllers are tilted by as games see them (positive: front end up). Set with cv1touch tilt; tracking itself is unchanged.",
        "Shared memory" => "The shared memory name through which the daemon hands poses to the SteamVR driver.",
        "Generation" => "A random number the daemon picks each time it starts. It changes when the daemon restarts.",
        "Queued frames" => "Sensor frames waiting to be processed now. 0 to a few means tracking keeps up with the video.",
        "Dropped frames" => "Sensor frames dropped because tracking fell behind and newer ones replaced them. It should rise only rarely.",
        "Dropped clocks" => "Sensor timing samples dropped before they could be used. It should stay at or near 0.",
        "Unknown frames" => "Frames from a sensor the tracker had not admitted, such as one just unplugged. It should stay at or near 0.",
        "Pending dropped" => "Frames the tracking pipeline dropped while they waited. It should stay at or near 0.",
        "Recording" => "A recording of tracking data the daemon is making, for diagnosis.",
        _ => "",
    }
}

/// What the `doctor` check `name` looks at.
fn check_tip(name: &str) -> &'static str {
    match name {
        "steamvr_runtime" => "Whether SteamVR is installed where cv1touch looks for it.",
        "openvr_library" => "Whether SteamVR's OpenVR library, which cv1touch uses to talk to SteamVR, is there.",
        "input_profile" => "Whether cv1touch's controller input profile, made from SteamVR's Oculus Touch profile, is installed.",
        "render_models" => "Whether SteamVR's Rift CV1 controller models, which games show for your hands, are there.",
        "driver_assets" => "Whether cv1touch's SteamVR driver is installed completely.",
        "calibration" => "Whether the room calibration of the sensors exists and can be used.",
        "registration" => "Whether the alignment of cv1touch's tracking with the headset's exists and can be used.",
        "render_offsets" => "Whether the fit of the controller models to their tracked LEDs exists and can be used.",
        "setup" => "Which parts of setup have not been done yet.",
        "sensor_drivers" => "Whether the sensors have the USB driver cv1touch needs on Windows.",
        "daemon" => "Whether the cv1touchd daemon is running and answering.",
        "sensor_streams" => "Whether every sensor's video started and is running without trouble.",
        "sensor_poses" => "Whether every sensor has a usable place in the room.",
        "sensor_hotplug" => "Whether sensors plugged in or out have been picked up or let go.",
        "headset" => "Whether SteamVR uses the headset cv1touch was aligned with.",
        "headset_alignment" => "Whether SteamVR tracks the headset from a base station the headset alignment measured.",
        "base_station_pin" => "Whether SteamVR tracks the headset from the base station cv1touch asked for.",
        _ => "One of cv1touch doctor's checks of the installation and the daemon.",
    }
}

/// What a check's badge means.
fn badge_tip(label: &str) -> &'static str {
    match label {
        "FAIL" => "Failed: this needs fixing before cv1touch can work fully.",
        "TODO" => "Not done yet: run setup.",
        "WARN" => "Works, but something needs your attention.",
        _ => "Passed: nothing to do here.",
    }
}

/// One row of a figures grid: its name and value, both explaining the figure.
fn row(ui: &mut Ui, key: &str, value: RichText) {
    let tip = tip(key);
    debug_assert!(!tip.is_empty(), "no tooltip for {key}");
    ui.label(weak(key)).on_hover_text(tip);
    ui.label(value).on_hover_text(tip);
    ui.end_row();
}

fn kv(ui: &mut Ui, rows: &[(&str, String)]) {
    egui::Grid::new(ui.next_auto_id())
        .num_columns(2)
        .spacing([16.0, 2.0])
        .show(ui, |ui| {
            for (k, v) in rows {
                row(ui, k, mono(v.clone()));
            }
        });
}

fn num(value: &Value, digits: usize) -> String {
    match value.as_f64() {
        Some(v) if value.is_f64() => format!("{v:.digits$}"),
        Some(_) => value.to_string(),
        None => "–".to_owned(),
    }
}

fn hand_stats(ui: &mut Ui, snap: &Snapshot, hand: &Value, i: usize) {
    let t = &hand["tracker"];
    let mut rows = vec![
        (
            "Optical rate",
            snap.hand_rates[i].map_or("–".into(), |r| format!("{r:.1} Hz")),
        ),
        (
            "Flags",
            hand["flags"]
                .as_u64()
                .map_or("–".into(), |f| format!("{f:#04x}")),
        ),
        (
            "Battery",
            hand["battery_percent"]
                .as_i64()
                .map_or("–".into(), |b| format!("{b}%")),
        ),
        (
            "Radio loss",
            hand["radio_lost_percent"]
                .as_f64()
                .map_or("–".into(), |l| format!("{l:.2}%")),
        ),
        (
            "Tracking",
            t["tracking"]
                .as_bool()
                .map_or("–".into(), |b| if b { "yes" } else { "no" }.into()),
        ),
    ];
    for name in [
        "updates",
        "frames",
        "acquisitions",
        "good",
        "wrong_locks",
        "refused",
        "unconfirmed",
        "rejected",
        "out_of_order",
        "too_few",
        "not_visible",
        "searched",
    ] {
        rows.push((name, num(&t[name], 0)));
    }
    let bias = vec3(&t["accel_bias"]);
    rows.push((
        "Accel bias",
        bias.map_or("–".into(), |b| format!("{:.3} m/s²", norm(b))),
    ));
    rows.push((
        "Bias x, y, z",
        bias.map_or("–".into(), |b| {
            format!("{:.3}, {:.3}, {:.3}", b[0], b[1], b[2])
        }),
    ));
    rows.push((
        "Verified bias",
        vec3(&t["verified_accel_bias"]).map_or("Learning".into(), |b| {
            format!("Verified, {:.3} m/s²", norm(b))
        }),
    ));
    egui::Grid::new(("hand-stats", i))
        .num_columns(2)
        .spacing([12.0, 2.0])
        .show(ui, |ui| {
            for (k, v) in rows {
                let flagged =
                    matches!(k, "wrong_locks" | "refused") && t[k].as_u64().is_some_and(|n| n > 0);
                let text = mono(v);
                let text = if flagged {
                    text.color(color(
                        ui,
                        if k == "wrong_locks" {
                            Level::Bad
                        } else {
                            Level::Warning
                        },
                    ))
                } else {
                    text
                };
                row(ui, k, text);
            }
        });
}

fn sensor_stats(
    ui: &mut Ui,
    snap: &Snapshot,
    status: &Value,
    config: &ConfigStats,
    i: usize,
    sensor: &Value,
) {
    let serial = sensor["serial"].as_str().unwrap_or("unknown");
    let (state, hint, level) = sensor_state(snap, sensor);
    let alt = status["sensor_streams"]
        .as_array()
        .into_iter()
        .flatten()
        .find(|s| {
            s["serial"]
                .as_str()
                .is_some_and(|s| s.eq_ignore_ascii_case(serial))
        })
        .and_then(|s| s["alt_setting"].as_i64());
    let a = &sensor["agreement"];
    let miss = match (a["checks"].as_u64(), a["misses"].as_u64()) {
        (Some(c), Some(m)) if c > 0 => format!("{m} ({:.1}%)", m as f64 * 100.0 / c as f64),
        (_, Some(m)) => m.to_string(),
        _ => "–".into(),
    };
    let fit = config.sensor(serial);
    let position = vec3(&sensor["camera_pos"]).map_or("–".into(), |p| {
        format!("({:.3}, {:.3}, {:.3}) m", p[0], p[1], p[2])
    });
    let mut rows = vec![
        ("Serial", serial.to_owned()),
        (
            "Video",
            format!(
                "{}{}",
                sensor["video_format"].as_str().unwrap_or("–"),
                alt.map_or(String::new(), |a| format!(", alt setting {a}"))
            ),
        ),
        (
            "Frame rate",
            snap.rates
                .get(&key(sensor))
                .map_or("–".into(), |r| format!("{r:.1} fps")),
        ),
        ("Dropped", num(&sensor["frames_dropped"], 0)),
        (
            "Fixes L / R",
            format!(
                "{} / {}",
                num(&sensor["fixes"][0], 0),
                num(&sensor["fixes"][1], 0)
            ),
        ),
        (
            "Agreement",
            format!(
                "median {} px, bias {} px",
                num(&a["median_px"], 2),
                num(&a["bias_px"], 2)
            ),
        ),
        ("Checks", format!("{}, misses {miss}", num(&a["checks"], 0))),
        (
            "Calibration fit",
            fit.map_or("–".into(), |f| {
                format!(
                    "{} px over {} views",
                    num(&f["rms_px"], 3),
                    num(&f["observations"], 0)
                )
            }),
        ),
        ("Position", position),
        (
            "Pose",
            format!(
                "{} ({})",
                sensor["pose"]["state"].as_str().unwrap_or("placed"),
                if sensor["placed"] == 1 {
                    "placed"
                } else {
                    "not placed"
                }
            ),
        ),
    ];
    if let Some(shift) = sensor["landmark_shift"].as_object() {
        rows.push((
            "Landmark shift",
            format!(
                "dx {} dy {} px over {} landmarks",
                num(&shift["dx"], 2),
                num(&shift["dy"], 2),
                shift.get("landmarks").map_or("–".into(), Value::to_string)
            ),
        ));
    }
    card(ui, |ui| {
        ui.horizontal(|ui| {
            dot(ui, level);
            ui.label(RichText::new(format!("Sensor {}", i + 1)).strong());
            ui.label(RichText::new(state).color(color(ui, level)));
        })
        .response
        .on_hover_text(hint);
        kv(ui, &rows);
    });
}

pub(super) fn tab(ui: &mut Ui, view: &View) {
    let snap = view.snap;
    let config = view.config;
    if let Some(status) = &snap.status {
        ui.columns(2, |columns| {
            for (i, ui) in columns.iter_mut().enumerate() {
                card(ui, |ui| {
                    title(
                        ui,
                        if i == 0 {
                            "Left controller"
                        } else {
                            "Right controller"
                        },
                    );
                    hand_stats(ui, snap, &status["hands"][i], i);
                });
            }
        });
        for (i, sensor) in status["sensors"]
            .as_array()
            .into_iter()
            .flatten()
            .enumerate()
        {
            sensor_stats(ui, snap, status, config, i, sensor);
        }
    } else {
        card(ui, |ui| {
            ui.label(RichText::new("No live status from the daemon.").weak());
        });
    }
    card(ui, |ui| {
        title(ui, "Room calibration");
        let c = config.calibration.as_ref();
        let s = c.map(|c| &c["stats"]);
        let get = |name: &str, digits| s.map_or("–".into(), |s| num(&s[name], digits));
        kv(
            ui,
            &[
                (
                    "Gravity spread",
                    format!("{}°", get("gravity_spread_deg", 3)),
                ),
                ("Gravity", format!("{} m/s²", get("gravity_mps2", 4))),
                ("Motion segments", get("motion_segments", 0)),
                ("Samples", get("samples", 0)),
                ("Rejected views", get("rejected_views", 0)),
                (
                    "Reference sensor",
                    s.and_then(|s| s["reference"].as_str())
                        .unwrap_or("–")
                        .to_owned(),
                ),
            ],
        );
        title(ui, "Headset alignment");
        let r = config.registration.as_ref();
        let get = |name: &str, digits| r.map_or("–".into(), |r| num(&r[name], digits));
        let status = snap.status.as_ref();
        let pin = status.map(|s| &s["alignment"]["base_pin"]);
        kv(
            ui,
            &[
                ("Position RMS", format!("{} mm", get("pos_rms_mm", 2))),
                ("Rotation RMS", format!("{}°", get("rot_rms_deg", 2))),
                ("Pairs", get("pairs", 0)),
                (
                    "Reference system",
                    r.and_then(|r| r["reference_system"].as_str())
                        .unwrap_or("–")
                        .to_owned(),
                ),
                (
                    "Tracking base",
                    r.and_then(|r| r["tracking_base"].as_str())
                        .unwrap_or("–")
                        .to_owned(),
                ),
                (
                    "Measured stations",
                    r.map_or("–".into(), |r| {
                        format!(
                            "{} of {}",
                            r["measured"].as_array().map_or(0, Vec::len),
                            r["stations"].as_array().map_or(0, Vec::len)
                        )
                    }),
                ),
                (
                    "Universe",
                    status
                        .map(|s| &s["alignment"]["universe"])
                        .filter(|u| !u.is_null())
                        .or(r.map(|r| &r["universe_id"]))
                        .map_or("–".into(), |u| num(u, 0)),
                ),
                (
                    "Base pin",
                    pin.map_or("–".into(), |p| {
                        match (p["base"].as_str(), p["waiting_for"].as_str()) {
                            (_, Some(w)) => format!("waiting for {w}"),
                            (Some(b), None) => {
                                format!("{b} ({})", p["owner"].as_str().unwrap_or("owner unknown"))
                            }
                            (None, None) => "none".into(),
                        }
                    }),
                ),
            ],
        );
        title(ui, "Controllers");
        let fit = |hand: &str| {
            config.render_fit(hand).map_or("–".into(), |(mm, leds)| {
                format!("{mm:.3} mm, {leds} LEDs")
            })
        };
        let input = status.map(|s| &s["input_calibration"]["calibrated"]);
        let calibrated = |i: usize| {
            input
                .and_then(|c| c[i].as_bool())
                .map_or("–", |b| if b { "calibrated" } else { "factory" })
                .to_owned()
        };
        kv(
            ui,
            &[
                ("Left model offset", fit("left")),
                ("Right model offset", fit("right")),
                ("Left input", calibrated(0)),
                ("Right input", calibrated(1)),
            ],
        );
    });
    if let Some(status) = &snap.status {
        card(ui, |ui| {
            title(ui, "Daemon");
            let q = &status["camera_queue"];
            let mut rows = vec![
                ("Mode", status["mode"].as_str().unwrap_or("–").to_owned()),
                (
                    "Tracker",
                    status["tracker"].as_str().unwrap_or("–").to_owned(),
                ),
                ("Tilt", format!("{}°", num(&status["tilt_deg"], 3))),
                (
                    "Shared memory",
                    status["shm"].as_str().unwrap_or("–").to_owned(),
                ),
                ("Generation", num(&status["generation"], 0)),
                ("Queued frames", num(&q["queued_frames"], 0)),
                ("Dropped frames", num(&q["dropped_frames"], 0)),
                ("Dropped clocks", num(&q["dropped_clocks"], 0)),
                ("Unknown frames", num(&q["unknown_frames"], 0)),
                ("Pending dropped", num(&q["pending_frames_dropped"], 0)),
            ];
            let recording = &status["recording_status"];
            if recording["state"].as_str().is_some_and(|s| s != "idle") {
                rows.push((
                    "Recording",
                    recording["message"]
                        .as_str()
                        .or(recording["state"].as_str())
                        .unwrap_or("–")
                        .to_owned(),
                ));
            }
            kv(ui, &rows);
        });
    }
    card(ui, |ui| {
        title(ui, "Diagnostic checks");
        match (&snap.doctor, &snap.doctor_error) {
            (Some(doctor), _) => {
                egui::Grid::new("checks")
                    .num_columns(3)
                    .spacing([10.0, 4.0])
                    .show(ui, |ui| {
                        for check in doctor["checks"].as_array().into_iter().flatten() {
                            let detail = check["detail"].as_str().unwrap_or("");
                            let (label, level, detail) = if check["ok"] != true {
                                ("FAIL", Level::Bad, detail)
                            } else if let Some(rest) = detail.strip_prefix("todo: ") {
                                ("TODO", Level::Warning, rest)
                            } else if let Some(rest) = detail.strip_prefix("warning: ") {
                                ("WARN", Level::Warning, rest)
                            } else {
                                ("OK", Level::Good, detail)
                            };
                            let name = check["name"].as_str().unwrap_or("check");
                            badge(ui, level, label).on_hover_text(badge_tip(label));
                            ui.label(mono(name)).on_hover_text(check_tip(name));
                            ui.add(egui::Label::new(detail).wrap())
                                .on_hover_text(check_tip(name));
                            ui.end_row();
                        }
                    });
            }
            (None, Some(error)) => {
                ui.label(error);
            }
            (None, None) => {
                ui.label(RichText::new("Running checks…").weak());
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plain(text: &str) -> bool {
        text.ends_with('.') && text.split_whitespace().count() >= 4
    }

    #[test]
    fn every_doctor_check_and_badge_is_explained() {
        for name in [
            "steamvr_runtime",
            "openvr_library",
            "input_profile",
            "render_models",
            "driver_assets",
            "calibration",
            "registration",
            "render_offsets",
            "setup",
            "sensor_drivers",
            "daemon",
            "sensor_streams",
            "sensor_poses",
            "sensor_hotplug",
            "headset",
            "headset_alignment",
            "base_station_pin",
        ] {
            assert!(plain(check_tip(name)), "{name}");
            assert_ne!(check_tip(name), check_tip("a future check"), "{name}");
        }
        assert!(plain(check_tip("a future check")));
        for label in ["OK", "WARN", "TODO", "FAIL"] {
            assert!(plain(badge_tip(label)), "{label}");
        }
    }

    /// Every figure goes through `row`, which explains it; rendering the full Stats tab
    /// (`screenshots`) fails on a figure `tip` does not know.
    #[test]
    fn every_figure_goes_through_row() {
        let source = include_str!("stats.rs");
        let grid_labels = source.matches(&["ui.label(", "weak("].concat()).count();
        assert_eq!(grid_labels, 1, "a figure is labelled outside row()");
        for key in [
            "Optical rate",
            "Landmark shift",
            "Recording",
            "Pending dropped",
            "searched",
        ] {
            assert!(plain(tip(key)), "{key}");
        }
        assert_eq!(tip("not a figure"), "");
    }
}
