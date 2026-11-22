// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! What the guided setup shows a person: plain stage names, a short checklist, the checks only
//! when one fails, and one line per finished stage. Pure functions, so wording is testable.

use std::collections::{BTreeMap, BTreeSet};

use cv1touch::align::{Placement, Registration, StationWorld};
use cv1touch::setup::{Stage, StageReport, State};

/// Marks for the checklist and results; ASCII on a dumb terminal.
pub(super) struct Glyphs {
    pub done: &'static str,
    pub next: &'static str,
    pub later: &'static str,
    pub fail: &'static str,
}

impl Glyphs {
    pub(super) fn for_terminal(dumb: bool) -> Glyphs {
        if dumb {
            Glyphs {
                done: "[x]",
                next: "[>]",
                later: "[ ]",
                fail: "[!]",
            }
        } else {
            Glyphs {
                done: "✓",
                next: "●",
                later: "·",
                fail: "✗",
            }
        }
    }
}

pub(super) fn stage_name(stage: Stage) -> &'static str {
    match stage {
        Stage::Camera => "Sensor positions",
        Stage::Gravity => "Floor level",
        Stage::Render => "Controller model",
        Stage::Inputs => "Buttons and sticks",
        Stage::Registration => "Headset alignment",
    }
}

/// Roughly how long the stage takes a person, briefing included.
pub(super) fn stage_minutes(stage: Stage) -> &'static str {
    match stage {
        Stage::Camera => "~2 min",
        Stage::Gravity => "~2 min",
        Stage::Render => "~1 min",
        Stage::Inputs => "~1 min",
        Stage::Registration => "~3 min",
    }
}

fn state_words(report: &StageReport) -> String {
    match report.state {
        State::Ready => "done".into(),
        State::Missing => "not done".into(),
        State::Stale if report.reason.contains("did not complete") => "not finished".into(),
        // the planner's reasons are file fingerprints and dependencies: setup --check --json has them
        State::Stale => "needs redoing (its result or an earlier step changed)".into(),
        State::Unknown => "saved earlier, not verified".into(),
        State::Invalid => "saved file unusable: redo it".into(),
        // the report's reason is "what blocks it; its own state": the blocker is what to act on
        State::Blocked => format!(
            "waiting: {}",
            report.reason.split("; ").next().unwrap_or(&report.reason)
        ),
    }
}

/// The stage list, the first unfinished stage marked as next.
pub(super) fn checklist(reports: &[StageReport], g: &Glyphs) -> String {
    let next = reports.iter().position(|r| r.state != State::Ready);
    let width = reports
        .iter()
        .map(|r| stage_name(r.stage).chars().count())
        .max()
        .unwrap_or(0);
    reports
        .iter()
        .enumerate()
        .map(|(i, r)| {
            let mark = match r.state {
                State::Ready => g.done,
                State::Blocked | State::Invalid => g.fail,
                _ if Some(i) == next => g.next,
                _ => g.later,
            };
            format!(
                "  {mark} {:<width$}  {}",
                stage_name(r.stage),
                state_words(r)
            )
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// One check as the guide sees it.
pub(super) struct CheckView<'a> {
    pub name: &'a str,
    pub ok: bool,
    pub detail: &'a str,
}

/// Checks that only restate what the stage list already says (a stage not done yet, a new sensor
/// to place) are not shown on their own.
fn shown(name: &str) -> bool {
    !matches!(name, "tracking_configuration" | "sensor_coverage")
}

fn check_label(name: &str, sensors: usize) -> String {
    match name {
        "daemon" => "daemon".into(),
        "sensor_streams" => format!("{sensors} sensor{}", if sensors == 1 { "" } else { "s" }),
        "controllers_awake" => "controllers".into(),
        "steamvr_assets" => "SteamVR".into(),
        "idle" => "engine idle".into(),
        "render_geometry" => "controller models".into(),
        other => other.replace('_', " "),
    }
}

/// "Checks: daemon ✓  5 sensors ✓ …", then one line per failed check with what to do.
pub(super) fn checks(checks: &[CheckView], sensors: usize, g: &Glyphs) -> String {
    let shown: Vec<&CheckView> = checks.iter().filter(|c| shown(c.name)).collect();
    // the line lists what a person would check first; the rest appear only when they fail
    let core = |name: &str| {
        matches!(
            name,
            "daemon" | "sensor_streams" | "controllers_awake" | "steamvr_assets"
        )
    };
    let mut out = format!(
        "Checks: {}",
        shown
            .iter()
            .filter(|c| core(c.name) || !c.ok)
            .map(|c| format!(
                "{} {}",
                check_label(c.name, sensors),
                if c.ok { g.done } else { g.fail }
            ))
            .collect::<Vec<_>>()
            .join("  ")
    );
    for c in shown.iter().filter(|c| !c.ok) {
        out.push_str(&format!(
            "\n  {} {}: {}",
            g.fail,
            check_label(c.name, sensors),
            c.detail
        ));
    }
    out
}

/// Whether the checks that matter to the person all pass.
pub(super) fn checks_pass(checks: &[CheckView]) -> bool {
    checks.iter().filter(|c| shown(c.name)).all(|c| c.ok)
}

/// What a typed answer at a stage prompt means. Enter alone starts the stage.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub(super) enum Answer {
    Start,
    Keep,
    /// Try the headset alignment again without watching for a controller slipping.
    Unwatched,
    Quit,
}

impl Answer {
    /// What can be typed for it.
    pub(super) fn words(self) -> &'static [&'static str] {
        match self {
            Answer::Start => &["", "new", "start", "s", "y", "yes"],
            Answer::Keep => &["k", "keep"],
            Answer::Unwatched => &["d"],
            Answer::Quit => &["q", "quit", "exit"],
        }
    }
}

/// The `offered` answer `line` gives, if any.
pub(super) fn answer(line: &str, offered: &[Answer]) -> Option<Answer> {
    let line = line.trim().to_ascii_lowercase();
    offered
        .iter()
        .copied()
        .find(|a| a.words().contains(&line.as_str()))
}

/// What to type, for someone who typed something else at a prompt allowing `allowed`.
pub(super) fn hint(allowed: &[&str]) -> String {
    if allowed.contains(&"") {
        let mut extra = String::new();
        for key in ["k", "d"].iter().filter(|key| allowed.contains(key)) {
            extra.push_str(", ");
            extra.push_str(key);
        }
        format!("Press Enter{extra}, or q to quit.")
    } else {
        format!("Enter {}.", allowed.join(" / "))
    }
}

/// The prompt after a failed stage; `slipped`: a controller slipped during the headset alignment.
pub(super) fn retry_prompt(slipped: bool) -> &'static str {
    if slipped {
        "Enter: try again   d: try again without the slip check   q: quit"
    } else {
        "Enter: try again   q: quit"
    }
}

/// The prompt under the checklist for the next stage.
pub(super) fn stage_prompt(stage: Stage, can_keep: bool) -> String {
    let keys = if can_keep {
        "Enter: redo   k: keep (only if tracking looks right)   q: quit"
    } else {
        "Enter: start   q: quit"
    };
    format!("{} ({}). {keys}", stage_name(stage), stage_minutes(stage))
}

/// The one-line headline of a finished stage, from the file it installed (JSON text).
pub(super) fn result_summary(stage: Stage, installed: Option<&serde_json::Value>) -> String {
    let Some(v) = installed else {
        return "done".into();
    };
    let num = |v: &serde_json::Value| v.as_f64();
    match stage {
        Stage::Camera => {
            let rms: Vec<f64> = v["sensors"]
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(|s| num(&s["rms_px"]))
                .collect();
            match (
                rms.iter().cloned().reduce(f64::min),
                rms.iter().cloned().reduce(f64::max),
            ) {
                (Some(lo), Some(hi)) => format!("{} sensors, error {lo:.2}–{hi:.2} px", rms.len()),
                _ => "done".into(),
            }
        }
        Stage::Render => {
            let mm: Vec<String> = v["hands"]
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(|h| {
                    Some(format!(
                        "{} {:.1} mm",
                        h["hand"].as_str()?,
                        num(&h["rms_mm"])?
                    ))
                })
                .collect();
            if mm.is_empty() {
                "done".into()
            } else {
                format!("matches within {}", mm.join(", "))
            }
        }
        Stage::Registration => match (num(&v["pos_rms_mm"]), num(&v["rot_rms_deg"])) {
            (Some(mm), Some(deg)) => format!("lined up within {mm:.1} mm, {deg:.2}°"),
            _ => "done".into(),
        },
        Stage::Gravity => "level".into(),
        Stage::Inputs => "trigger, grip and stick ranges saved".into(),
    }
}

/// Before a headset recording: the base station SteamVR tracks the headset from (`base`, as its
/// log says), whether the registration on disk (`reg`) measured it, and that the recording
/// measures it.
pub(super) fn headset_base_plan(base: Option<&str>, reg: Option<&Registration>) -> String {
    let measured = |b: &str| reg.is_some_and(|r| r.measured.iter().any(|m| m.serial == b));
    match base {
        Some(b) if measured(b) => format!("SteamVR tracks the headset from base station {b}, which an earlier recording measured.\nThis recording measures it again."),
        Some(b) => format!("SteamVR tracks the headset from base station {b}, which no recording has measured yet.\nThis recording measures it."),
        None => "SteamVR's log names no base station it tracks the headset from, so this recording may measure none.".into(),
    }
}

/// After a headset registration `reg` is written: the stations its recording saw that no
/// recording measured yet, and how to measure one; None when every one is measured.
pub(super) fn stations_to_measure(reg: &Registration) -> Option<String> {
    let left: Vec<&str> = reg
        .stations
        .iter()
        .map(|s| s.serial.as_str())
        .filter(|s| !reg.measured.iter().any(|m| m.serial == *s))
        .collect();
    (!left.is_empty()).then(|| {
        format!(
            "Not measured yet: {}. To measure one, run `cv1touch setup --redo headset` while SteamVR\ntracks the headset from it (`cv1touch universe` names the station it tracks the headset from).",
            left.join(", ")
        )
    })
}

/// After a headset registration `reg` is written: the base station its recording measured, and
/// how it places each station its recording saw or a registration measured (`map`, from
/// `Registration::station_map`).
pub(super) fn station_summary(reg: &Registration, map: &BTreeMap<String, StationWorld>) -> String {
    let mut lines = vec![match &reg.tracking_base {
        Some(b) => format!("This recording measured base station {b}. Base stations now:"),
        None => "This recording measured no base station. Base stations now:".into(),
    }];
    let serials: BTreeSet<&String> = reg
        .stations
        .iter()
        .map(|s| &s.serial)
        .chain(reg.measured.iter().map(|m| &m.serial))
        .collect();
    for serial in serials {
        let how = if reg.tracking_base.as_ref() == Some(serial) {
            "measured by this recording"
        } else if reg.measured.iter().any(|m| &m.serial == serial) {
            "measured by an earlier recording"
        } else {
            match map.get(serial).map(|w| w.placement) {
                Some(Placement::Layouts(_)) => "placed by SteamVR's layouts",
                Some(_) => "placed where SteamVR had it during this recording",
                None => "not placed",
            }
        };
        lines.push(format!("  {serial}: {how}"));
    }
    lines.join("\n")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn report(stage: Stage, state: State, reason: &str) -> StageReport {
        StageReport {
            stage,
            state,
            reason: reason.into(),
        }
    }

    #[test]
    fn the_checklist_names_stages_plainly_and_marks_the_next_one() {
        let g = Glyphs::for_terminal(false);
        let list = checklist(
            &[
                report(Stage::Camera, State::Ready, "x"),
                report(Stage::Gravity, State::Ready, "x"),
                report(Stage::Render, State::Ready, "x"),
                report(Stage::Inputs, State::Missing, "artifact is missing"),
                report(
                    Stage::Registration,
                    State::Stale,
                    "previous stage attempt did not complete",
                ),
            ],
            &g,
        );
        assert_eq!(
            list,
            "  ✓ Sensor positions    done\n  ✓ Floor level         done\n  ✓ Controller model    done\n  ● Buttons and sticks  not done\n  · Headset alignment   not finished"
        );
        assert!(!list.contains("artifact"), "no internal jargon");
        let dumb = checklist(
            &[report(Stage::Camera, State::Blocked, "start the daemon")],
            &Glyphs::for_terminal(true),
        );
        assert_eq!(dumb, "  [!] Sensor positions  waiting: start the daemon");
        let waiting = checklist(
            &[report(
                Stage::Inputs,
                State::Blocked,
                "wake both controllers with a face button; artifact is missing",
            )],
            &g,
        );
        assert_eq!(
            waiting,
            "  ✗ Buttons and sticks  waiting: wake both controllers with a face button"
        );
        // the planner's reasons are for --check --json; the list says what it means to a person
        let changed = checklist(
            &[
                report(
                    Stage::Camera,
                    State::Stale,
                    "artifact or dependency fingerprint changed",
                ),
                report(
                    Stage::Gravity,
                    State::Invalid,
                    "invalid fingerprint or dependency",
                ),
            ],
            &g,
        );
        assert_eq!(
            changed,
            "  ● Sensor positions  needs redoing (its result or an earlier step changed)\n  ✗ Floor level       saved file unusable: redo it"
        );
    }

    #[test]
    fn checks_show_one_line_and_explain_only_failures() {
        let g = Glyphs::for_terminal(false);
        let all = [
            CheckView {
                name: "daemon",
                ok: true,
                detail: "start it",
            },
            CheckView {
                name: "sensor_streams",
                ok: true,
                detail: "usb",
            },
            CheckView {
                name: "controllers_awake",
                ok: true,
                detail: "wake them",
            },
            CheckView {
                name: "steamvr_assets",
                ok: true,
                detail: "install",
            },
            CheckView {
                name: "tracking_configuration",
                ok: false,
                detail: "jargon",
            },
            CheckView {
                name: "sensor_coverage",
                ok: false,
                detail: "jargon",
            },
        ];
        assert_eq!(
            checks(&all, 5, &g),
            "Checks: daemon ✓  5 sensors ✓  controllers ✓  SteamVR ✓"
        );
        assert!(
            checks_pass(&all),
            "stage-level conditions are not check failures"
        );
        let asleep = [
            CheckView {
                name: "daemon",
                ok: true,
                detail: "start it",
            },
            CheckView {
                name: "controllers_awake",
                ok: false,
                detail: "Press a button on each controller.",
            },
        ];
        assert_eq!(
            checks(&asleep, 1, &g),
            "Checks: daemon ✓  controllers ✗\n  ✗ controllers: Press a button on each controller."
        );
        assert!(!checks_pass(&asleep));
        let busy = [
            CheckView {
                name: "daemon",
                ok: true,
                detail: "start it",
            },
            CheckView {
                name: "idle",
                ok: true,
                detail: "wait",
            },
            CheckView {
                name: "render_geometry",
                ok: false,
                detail: "Install the SteamVR meshes.",
            },
        ];
        assert_eq!(
            checks(&busy, 1, &g),
            "Checks: daemon ✓  controller models ✗\n  ✗ controller models: Install the SteamVR meshes.",
            "secondary checks appear only when they fail"
        );
    }

    #[test]
    fn enter_starts_and_keep_is_offered_only_where_it_applies() {
        use Answer::*;
        assert_eq!(answer("\n", &[Start, Quit]), Some(Start));
        assert_eq!(answer("new", &[Start, Quit]), Some(Start));
        assert_eq!(answer("Q", &[Start, Keep, Quit]), Some(Quit));
        assert_eq!(answer("k", &[Start, Keep, Quit]), Some(Keep));
        assert_eq!(answer("keep", &[Start, Quit]), None);
        assert_eq!(answer("banana", &[Start, Keep, Quit]), None);
        assert_eq!(answer("d", &[Start, Unwatched, Quit]), Some(Unwatched));
        assert_eq!(answer("d", &[Start, Quit]), None);
        assert_eq!(retry_prompt(false), "Enter: try again   q: quit");
        // what a wrong key gets told
        assert_eq!(hint(&["", "s", "q"]), "Press Enter, or q to quit.");
        assert_eq!(hint(&["", "k", "q"]), "Press Enter, k, or q to quit.");
        assert_eq!(hint(&["", "d", "q"]), "Press Enter, d, or q to quit.");
        assert_eq!(hint(&["a", "b"]), "Enter a / b.");
        assert_eq!(
            retry_prompt(true),
            "Enter: try again   d: try again without the slip check   q: quit"
        );
        assert_eq!(
            stage_prompt(Stage::Registration, false),
            "Headset alignment (~3 min). Enter: start   q: quit"
        );
        assert_eq!(
            stage_prompt(Stage::Camera, true),
            "Sensor positions (~2 min). Enter: redo   k: keep (only if tracking looks right)   q: quit"
        );
    }

    #[test]
    fn a_finished_stage_reads_its_headline_from_the_installed_file() {
        let camera = serde_json::json!({"sensors": [{"rms_px": 0.182}, {"rms_px": 0.14}, {"rms_px": 0.169}]});
        assert_eq!(
            result_summary(Stage::Camera, Some(&camera)),
            "3 sensors, error 0.14–0.18 px"
        );
        let registration = serde_json::json!({"pos_rms_mm": 1.94, "rot_rms_deg": 0.234});
        assert_eq!(
            result_summary(Stage::Registration, Some(&registration)),
            "lined up within 1.9 mm, 0.23°"
        );
        let render = serde_json::json!({"hands": [{"hand": "left", "rms_mm": 0.41}, {"hand": "right", "rms_mm": 0.38}]});
        assert_eq!(
            result_summary(Stage::Render, Some(&render)),
            "matches within left 0.4 mm, right 0.4 mm"
        );
        assert_eq!(result_summary(Stage::Camera, None), "done");
        assert_eq!(
            result_summary(Stage::Registration, Some(&serde_json::json!({}))),
            "done"
        );
    }

    #[test]
    fn the_station_summary_says_how_each_station_is_placed() {
        use cv1touch::align::{MeasuredStation, PoseEntry, StationPose};
        let pose = PoseEntry::from_iso(&nalgebra::Isometry3::identity());
        let measured = |serial: &str| MeasuredStation {
            serial: serial.into(),
            world_from_station: pose,
            registered_unix_s: 1,
            calibration_unix_s: 1,
        };
        let placed = |placement| StationWorld {
            world_from_station: nalgebra::Isometry3::identity(),
            placement,
        };
        let mut reg = crate::initial_registration(7);
        reg.stations = ["LHB-A", "LHB-C", "LHB-D"]
            .map(|serial| StationPose {
                serial: serial.into(),
                pose,
            })
            .into();
        // B, measured before, is off during this recording; D no layout places
        reg.tracking_base = Some("LHB-A".into());
        reg.measured = vec![measured("LHB-A"), measured("LHB-B")];
        let map = BTreeMap::from([
            ("LHB-A".to_string(), placed(Placement::Measured)),
            ("LHB-B".to_string(), placed(Placement::Measured)),
            ("LHB-C".to_string(), placed(Placement::Layouts(3))),
        ]);
        assert_eq!(
            station_summary(&reg, &map),
            "This recording measured base station LHB-A. Base stations now:\n  \
             LHB-A: measured by this recording\n  \
             LHB-B: measured by an earlier recording\n  \
             LHB-C: placed by SteamVR's layouts\n  \
             LHB-D: not placed"
        );
        // nothing measured: each station is where this recording's layout puts it
        reg.tracking_base = None;
        reg.measured.clear();
        let map = BTreeMap::from([("LHB-A".to_string(), placed(Placement::Fit))]);
        assert_eq!(
            station_summary(&reg, &map),
            "This recording measured no base station. Base stations now:\n  \
             LHB-A: placed where SteamVR had it during this recording\n  \
             LHB-C: not placed\n  \
             LHB-D: not placed"
        );
    }

    #[test]
    fn the_headset_stage_names_the_station_it_measures_and_those_left() {
        use cv1touch::align::{MeasuredStation, PoseEntry, StationPose};
        let pose = PoseEntry::from_iso(&nalgebra::Isometry3::identity());
        let mut reg = crate::initial_registration(7);
        reg.stations = ["LHB-A", "LHB-B", "LHB-C"]
            .map(|serial| StationPose {
                serial: serial.into(),
                pose,
            })
            .into();
        reg.measured = vec![MeasuredStation {
            serial: "LHB-A".into(),
            world_from_station: pose,
            registered_unix_s: 1,
            calibration_unix_s: 1,
        }];
        assert_eq!(
            headset_base_plan(Some("LHB-A"), Some(&reg)),
            "SteamVR tracks the headset from base station LHB-A, which an earlier recording measured.\nThis recording measures it again."
        );
        for plan in [
            headset_base_plan(Some("LHB-B"), Some(&reg)),
            headset_base_plan(Some("LHB-B"), None),
        ] {
            assert_eq!(plan, "SteamVR tracks the headset from base station LHB-B, which no recording has measured yet.\nThis recording measures it.");
        }
        assert_eq!(
            headset_base_plan(None, Some(&reg)),
            "SteamVR's log names no base station it tracks the headset from, so this recording may measure none."
        );
        assert_eq!(
            stations_to_measure(&reg).as_deref(),
            Some("Not measured yet: LHB-B, LHB-C. To measure one, run `cv1touch setup --redo headset` while SteamVR\ntracks the headset from it (`cv1touch universe` names the station it tracks the headset from).")
        );
        reg.stations.truncate(1);
        assert_eq!(stations_to_measure(&reg), None);
    }
}
