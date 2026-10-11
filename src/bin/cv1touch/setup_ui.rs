// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! What the guided setup shows a person: plain stage names, a short checklist, the checks only
//! when one fails, and one line per finished stage. Pure functions, so wording is testable.

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
        State::Stale => format!("needs redoing ({})", report.reason),
        State::Unknown => "saved earlier, not verified".into(),
        State::Invalid => format!("saved file unusable ({})", report.reason),
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
        let extra: String = ["k", "d"]
            .iter()
            .filter(|key| allowed.contains(key))
            .map(|key| format!(", {key}"))
            .collect();
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
}
