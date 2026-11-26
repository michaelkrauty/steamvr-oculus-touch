// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The setup window (`cv1touch-gui --setup`): the terminal guide's stages, prompts and
//! progress, driven through `cv1touch setup --events`.

use crate::{
    actions::{self, ServiceAction},
    icon::Tint,
    model::Level,
    setup_client::{
        metric_hands, Guide, Hand, Hands, InputsTap, Prompt, Request, SetupClient, Stage,
    },
    window::{self, card, color, dot, title, weak},
};
use eframe::egui::{self, RichText, Ui};
use serde_json::Value;
use std::{
    sync::{mpsc, OnceLock},
    time::Duration,
};

pub const SIZE: [f32; 2] = [680.0, 540.0];
pub const MIN_SIZE: [f32; 2] = [560.0, 440.0];

static REQUEST: OnceLock<Request> = OnceLock::new();

/// What the window opens for; set before `run`.
pub fn request(request: Request) {
    let _ = REQUEST.set(request);
}

/// What one frame asks for.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SetupAction {
    /// A command line for the guide.
    Send(String),
    Sensors,
    StartDaemon,
    Close,
}

/// The terminal's text with its hard line breaks joined, paragraphs and list items kept.
pub fn unwrap(text: &str) -> String {
    text.split("\n\n")
        .map(|paragraph| {
            let mut out = String::new();
            for line in paragraph.lines().map(str::trim_end) {
                let item = line.trim_start().starts_with("- ")
                    || line.trim_start().starts_with("• ")
                    || line
                        .trim_start()
                        .chars()
                        .next()
                        .is_some_and(|c| c.is_ascii_digit())
                        && line.trim_start().contains(". ");
                if !out.is_empty() {
                    out.push(if item { '\n' } else { ' ' });
                }
                out.push_str(if item { line.trim_start() } else { line.trim() });
            }
            out
        })
        .collect::<Vec<_>>()
        .join("\n\n")
}

/// The buttons a prompt offers besides Cancel, with their commands: only what its `choices`
/// accept.
pub fn prompt_buttons(prompt: &Prompt) -> Vec<(&'static str, &'static str)> {
    let mut out = Vec::new();
    for choice in &prompt.choices {
        let button = match (choice.as_str(), prompt.id.as_str()) {
            ("go", "start") => ("Start capture", "go"),
            ("go", "rest" | "triggers" | "grips" | "sticks") => ("Measure now", "go"),
            ("go", "menu") => continue,
            ("go", _) => ("Continue", "go"),
            ("retry", _) => ("Try again", "retry"),
            ("unwatched", _) => ("Try without slip check", "unwatched"),
            ("skip", _) => ("Skip", "skip"),
            _ => continue,
        };
        out.push(button);
    }
    out
}

/// A stage's planner state as a colour.
fn state_level(state: &str) -> Level {
    match state {
        "ready" => Level::Good,
        "missing" => Level::Quiet,
        _ => Level::Warning,
    }
}

fn pill(ui: &mut Ui, level: Level, active: bool, text: &str) -> egui::Response {
    let tint = if active {
        ui.visuals().selection.bg_fill
    } else {
        color(ui, level)
    };
    egui::Frame::new()
        .fill(tint.gamma_multiply(if active { 0.55 } else { 0.18 }))
        .stroke(egui::Stroke::new(1.0, tint.gamma_multiply(0.7)))
        .corner_radius(10)
        .inner_margin(egui::Margin::symmetric(9, 2))
        .show(ui, |ui| {
            let text = RichText::new(text);
            ui.label(if active {
                text.strong().color(ui.visuals().strong_text_color())
            } else {
                text.color(tint)
            })
        })
        .response
}

fn header(ui: &mut Ui, guide: &Guide, action: &mut Option<SetupAction>) {
    ui.horizontal(|ui| {
        ui.label(RichText::new("cv1touch setup").size(18.0).strong());
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui
                .button("Sensor views")
                .on_hover_text("Show what each sensor sees, live, beside this window: useful to aim the sensors.")
                .clicked()
            {
                *action = Some(SetupAction::Sensors);
            }
        });
    });
    let active = guide.active_stage();
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 4.0;
        for (i, stage) in Stage::ALL.into_iter().enumerate() {
            let info = guide
                .state
                .as_ref()
                .and_then(|s| s.stages.iter().find(|s| s.id == stage.id()));
            let level = info.map_or(Level::Quiet, |s| state_level(&s.state));
            let response = pill(
                ui,
                level,
                active == Some(stage.id()),
                &format!("{} {}", i + 1, stage.short()),
            );
            if let Some(info) = info {
                response.on_hover_text(format!("{}: {}", info.title, info.summary));
            }
            if i + 1 < Stage::ALL.len() {
                ui.label(weak("·"));
            }
        }
    });
}

/// Draws the window; `live` is the inputs tap's latest reading.
pub fn draw(
    ui: &mut Ui,
    guide: &Guide,
    live: Option<&Hands>,
    starting_daemon: bool,
) -> Option<SetupAction> {
    let mut action = None;
    egui::Panel::top("setup-header")
        .frame(egui::Frame::side_top_panel(ui.style()).inner_margin(egui::Margin::symmetric(12, 8)))
        .show(ui, |ui| header(ui, guide, &mut action));
    egui::Panel::bottom("setup-footer")
        .frame(egui::Frame::side_top_panel(ui.style()).inner_margin(egui::Margin::symmetric(12, 8)))
        .show(ui, |ui| footer(ui, guide, &mut action));
    egui::CentralPanel::default().show(ui, |ui| {
        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                if guide.done.is_some() || guide.ended.is_some() {
                    completion(ui, guide);
                } else if guide.active_stage().is_some() {
                    stage_view(ui, guide, live);
                } else if guide.prompt.as_ref().is_some_and(|p| p.id == "menu") {
                    menu(ui, guide, starting_daemon, &mut action);
                } else {
                    ui.horizontal(|ui| {
                        ui.add(egui::Spinner::new());
                        ui.label("Looking at your setup…");
                    });
                }
            });
    });
    // Enter is the terminal's way to go on, as the prompts' text says.
    if guide
        .prompt
        .as_ref()
        .is_some_and(|p| prompt_buttons(p).iter().any(|(_, c)| *c == "go"))
        && ui.input(|i| i.key_pressed(egui::Key::Enter))
    {
        action = Some(SetupAction::Send("go".into()));
    }
    action
}

fn footer(ui: &mut Ui, guide: &Guide, action: &mut Option<SetupAction>) {
    if let Some((command, error)) = &guide.rejected {
        ui.label(
            RichText::new(format!("“{command}” was not accepted: {error}"))
                .color(color(ui, Level::Warning)),
        );
    }
    ui.horizontal(|ui| {
        let send = |action: &mut Option<SetupAction>, line: &str| {
            *action = Some(SetupAction::Send(line.to_owned()))
        };
        if guide.done.is_some() || guide.ended.is_some() {
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("Close").clicked() {
                    *action = Some(SetupAction::Close);
                }
            });
            return;
        }
        let prompt = guide.prompt.as_ref();
        if let Some(prompt) = prompt.filter(|p| p.id == "menu") {
            let state = guide.state.as_ref();
            let next = state.and_then(|s| {
                let next = s.next.as_deref()?;
                s.stages.iter().find(|st| st.id == next && st.can_run)
            });
            if let Some(next) = next.filter(|_| prompt.offers("go")) {
                if ui
                    .add(egui::Button::new(RichText::new("Continue setup").strong()))
                    .on_hover_text(format!("Run the next stage that is not done: {}.", next.title))
                    .clicked()
                {
                    send(action, "go");
                }
            }
            if prompt.offers("recheck")
                && ui
                    .button("Recheck")
                    .on_hover_text("Look at the sensors, controllers and saved files again.")
                    .clicked()
            {
                send(action, "recheck");
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui
                    .button("Close")
                    .on_hover_text("Close setup. Stages already finished stay saved.")
                    .clicked()
                {
                    *action = Some(SetupAction::Close);
                }
            });
            return;
        }
        if let Some(prompt) = prompt {
            for (label, command) in prompt_buttons(prompt) {
                let button = egui::Button::new(if command == "go" || command == "retry" {
                    RichText::new(label).strong()
                } else {
                    RichText::new(label)
                });
                if ui.add(button).clicked() {
                    send(action, command);
                }
            }
        }
        let logging = guide.active_stage() == Some("headset")
            && guide.progress.as_ref().is_some_and(|p| p.phase == "capture");
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui
                .button("Cancel")
                .on_hover_text("Stop setup here. Stages already finished stay saved, and tracking goes back to normal.")
                .clicked()
            {
                send(action, "quit");
            }
            // "finish" ends headset logging early and keeps the measurements so far.
            if logging
                && ui
                    .button("Finish early")
                    .on_hover_text("Stop logging now and align with what it has. It ends by itself once you have turned, nodded and tilted enough.")
                    .clicked()
            {
                send(action, "finish");
            }
        });
    });
}

fn menu(ui: &mut Ui, guide: &Guide, starting_daemon: bool, action: &mut Option<SetupAction>) {
    let Some(state) = &guide.state else { return };
    for (level, text) in state
        .left_out
        .iter()
        .map(|t| (Level::Warning, t))
        .chain(state.settling.iter().map(|t| (Level::Quiet, t)))
    {
        ui.horizontal_wrapped(|ui| {
            dot(ui, level);
            ui.label(text);
        });
    }
    let daemon_down = state.checks.iter().any(|c| c.name == "daemon" && !c.ok);
    let failing: Vec<_> = state
        .checks
        .iter()
        // With the daemon down, the other checks only say they were not made.
        .filter(|c| !c.ok && (!daemon_down || c.name == "daemon"))
        .collect();
    if !failing.is_empty() {
        let tint = color(ui, Level::Warning);
        egui::Frame::new()
            .fill(tint.gamma_multiply(0.14))
            .stroke(egui::Stroke::new(1.0, tint.gamma_multiply(0.6)))
            .corner_radius(8)
            .inner_margin(egui::Margin::symmetric(12, 8))
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.horizontal(|ui| {
                    dot(ui, Level::Warning);
                    ui.label(RichText::new("Before you start").strong().color(tint));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui
                            .small_button("Recheck")
                            .on_hover_text("Look again once you have done this.")
                            .clicked()
                        {
                            *action = Some(SetupAction::Send("recheck".into()));
                        }
                        if daemon_down {
                            let label = if starting_daemon {
                                ServiceAction::Start.progress()
                            } else {
                                ServiceAction::Start.label()
                            };
                            if ui
                                .add_enabled(!starting_daemon, egui::Button::new(label).small())
                                .on_hover_text(ServiceAction::Start.tooltip())
                                .clicked()
                            {
                                *action = Some(SetupAction::StartDaemon);
                            }
                        }
                    });
                });
                for check in failing {
                    ui.label(unwrap(&check.detail));
                }
            });
    }
    for (i, stage) in state.stages.iter().enumerate() {
        card(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(RichText::new(format!("{}  {}", i + 1, stage.title)).strong());
                ui.label(weak(&stage.minutes));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let run = if stage.state == "ready" { "Redo" } else { "Run" };
                    let tip = if stage.can_run {
                        format!("Run {} now.", stage.title)
                    } else {
                        format!("Not now: {}", stage.summary)
                    };
                    if ui
                        .add_enabled(stage.can_run, egui::Button::new(run))
                        .on_hover_text(&tip)
                        .on_disabled_hover_text(&tip)
                        .clicked()
                    {
                        *action = Some(SetupAction::Send(format!("run {}", stage.id)));
                    }
                    if stage.quick_available
                        && stage.can_run
                        && ui
                            .button("Quick (15 s)")
                            .on_hover_text("The short alignment: about 15 seconds of turning, with the controllers held to the headset. If it is not good enough, the full one follows.")
                            .clicked()
                    {
                        *action = Some(SetupAction::Send(format!("run {} quick", stage.id)));
                    }
                    if stage.can_keep
                        && ui
                            .button("Keep saved")
                            .on_hover_text("Keep the result saved earlier instead of doing this stage again.")
                            .clicked()
                    {
                        *action = Some(SetupAction::Send(format!("keep {}", stage.id)));
                    }
                });
            });
            ui.horizontal(|ui| {
                dot(ui, state_level(&stage.state));
                ui.label(RichText::new(&stage.summary).color(color(ui, state_level(&stage.state))))
                    .on_hover_text(state_tip(&stage.state));
            });
        });
    }
}

/// What a stage's planner state means, in a sentence.
fn state_tip(state: &str) -> &'static str {
    match state {
        "missing" => "Not done yet.",
        "invalid" => "The saved file cannot be used: redo this stage.",
        "unknown" => "Saved earlier but not verified: keep it or redo it.",
        "stale" => "Needs redoing, or did not complete.",
        "blocked" => "Waiting for something first.",
        _ => "Done.",
    }
}

fn stage_view(ui: &mut Ui, guide: &Guide, live: Option<&Hands>) {
    let id = guide.active_stage().unwrap_or_default();
    let title_text = guide
        .stage
        .as_ref()
        .map(|s| s.title.clone())
        .or_else(|| {
            guide
                .state
                .as_ref()?
                .stages
                .iter()
                .find(|s| s.id == id)
                .map(|s| s.title.clone())
        })
        .unwrap_or_else(|| id.to_owned());
    let prompt = guide.prompt.as_ref().filter(|p| p.id != "menu");
    card(ui, |ui| {
        ui.horizontal(|ui| {
            title(ui, &title_text);
            if guide.stage.as_ref().is_some_and(|s| s.quick) {
                ui.label(weak("quick"));
            }
        });
        if let Some(start) = &guide.stage {
            ui.label(unwrap(&start.instructions));
        }
    });
    if let Some(prompt) = prompt {
        match prompt.id.as_str() {
            "retry" => {
                let tint = color(ui, Level::Bad);
                action_card(ui, tint, |ui| {
                    ui.label(RichText::new(unwrap(&prompt.message)).strong().color(tint));
                    if let Some((_, error)) = &guide.failed {
                        ui.label(unwrap(error));
                    }
                });
            }
            "button_check" | "headset_start" => {
                let tint = ui.visuals().selection.bg_fill;
                action_card(ui, tint, |ui| {
                    ui.label(RichText::new("On your controllers").strong());
                    ui.label(unwrap(&prompt.message));
                });
            }
            _ => {
                card(ui, |ui| {
                    ui.label(unwrap(&prompt.message));
                    if prompt.accepts_controller_button && prompt.offers("go") {
                        ui.label(
                            RichText::new("You can also press A on your right controller")
                                .small()
                                .color(color(ui, Level::Good)),
                        );
                    }
                });
            }
        }
    }
    if let Some(status) = &guide.status {
        ui.label(RichText::new(status.trim()).strong());
    }
    if let Some(progress) = &guide.progress {
        let fraction = progress
            .elapsed_s
            .zip(progress.total_s)
            .filter(|(_, total)| *total > 0.0)
            .map(|(elapsed, total)| {
                (
                    (elapsed / total).clamp(0.0, 1.0),
                    (total - elapsed).max(0.0),
                )
            });
        match progress.phase.as_str() {
            "countdown" => {
                let left = progress.metrics["remaining_s"].as_u64().unwrap_or(0);
                ui.vertical_centered(|ui| {
                    ui.label(RichText::new(left.to_string()).size(64.0).strong());
                    ui.label(weak("Logging starts when the count ends"));
                });
            }
            "capture" if id == "headset" => headset_gauges(ui, &progress.metrics, fraction),
            "capture" => {
                if let Some((f, left)) = fraction {
                    ui.add(
                        egui::ProgressBar::new(f as f32)
                            .text(format!("{left:.0} s left"))
                            .corner_radius(4),
                    );
                }
            }
            "solve" | "reload" => {
                ui.horizontal(|ui| {
                    ui.add(egui::Spinner::new());
                    ui.label(if progress.detail.is_empty() {
                        "Tracking restarts…"
                    } else {
                        &progress.detail
                    });
                });
            }
            _ => {}
        }
    }
    if id == "buttons" {
        let metrics = guide
            .progress
            .as_ref()
            .filter(|p| p.phase == "inputs")
            .map(|p| &p.metrics);
        let hands = live.copied().or_else(|| metrics.and_then(metric_hands));
        if let Some(progress) = guide.progress.as_ref().filter(|p| p.phase == "inputs") {
            match progress.elapsed_s.zip(progress.total_s) {
                Some((elapsed, total)) if total > 0.0 => {
                    ui.add(
                        egui::ProgressBar::new((elapsed / total).clamp(0.0, 1.0) as f32)
                            .text("Logging…")
                            .corner_radius(4),
                    );
                }
                _ if progress.metrics["in_position"] == true => {
                    ui.label(RichText::new("In position: hold it").color(color(ui, Level::Good)));
                }
                _ => {}
            }
        }
        if let Some(hands) = hands {
            ui.columns(2, |columns| {
                for (i, ui) in columns.iter_mut().enumerate() {
                    hand_card(ui, if i == 0 { "Left" } else { "Right" }, &hands[i]);
                }
            });
        }
    }
}

fn action_card(ui: &mut Ui, tint: egui::Color32, add: impl FnOnce(&mut Ui)) {
    egui::Frame::new()
        .fill(tint.gamma_multiply(0.14))
        .stroke(egui::Stroke::new(1.0, tint.gamma_multiply(0.7)))
        .corner_radius(8)
        .inner_margin(egui::Margin::symmetric(12, 8))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            add(ui)
        });
}

/// The headset alignment's turning, nodding and tilting so far.
fn headset_gauges(ui: &mut Ui, m: &Value, fraction: Option<(f64, f64)>) {
    let needed = m["needed_deg"].as_f64().unwrap_or(0.0);
    let nod = m["nod_needed_deg"].as_f64().unwrap_or(15.0);
    let range = |key: &str| {
        let a = m[key].as_array()?;
        Some((a.first()?.as_f64()?, a.get(1)?.as_f64()?))
    };
    let gauge = |ui: &mut Ui, name: &str, done: f64, need: f64, text: String| {
        ui.horizontal(|ui| {
            ui.add_sized([96.0, 18.0], egui::Label::new(name));
            let f = if need > 0.0 {
                (done / need).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let mut bar = egui::ProgressBar::new(f as f32).text(text).corner_radius(4);
            if f >= 1.0 {
                bar = bar.fill(color(ui, Level::Good));
            }
            ui.add(bar);
        });
    };
    card(ui, |ui| {
        title(ui, "Logging");
        for (name, key) in [
            ("Turn left", "turn_left_deg"),
            ("Turn right", "turn_right_deg"),
        ] {
            let done = m[key].as_f64().unwrap_or(0.0);
            gauge(
                ui,
                name,
                done,
                needed,
                format!("{done:.0}° of {needed:.0}°"),
            );
        }
        let (down, up) = range("pitch_deg").unwrap_or_default();
        gauge(
            ui,
            "Nod down",
            -down,
            nod,
            format!("{:.0}° of {nod:.0}°", (-down).max(0.0)),
        );
        gauge(
            ui,
            "Nod up",
            up,
            nod,
            format!("{:.0}° of {nod:.0}°", up.max(0.0)),
        );
        let (left, right) = range("roll_deg").unwrap_or_default();
        gauge(
            ui,
            "Tilt one way",
            -left,
            nod,
            format!("{:.0}° of {nod:.0}°", (-left).max(0.0)),
        );
        gauge(
            ui,
            "Tilt the other",
            right,
            nod,
            format!("{:.0}° of {nod:.0}°", right.max(0.0)),
        );
        ui.label(weak(format!(
            "{} poses over {:.1} s{}",
            m["pairs"].as_u64().unwrap_or(0),
            m["seconds"].as_f64().unwrap_or(0.0),
            fraction.map_or(String::new(), |(_, left)| format!(
                " · stops by itself within {left:.0} s"
            ))
        )));
    });
}

fn hand_card(ui: &mut Ui, name: &str, hand: &Hand) {
    card(ui, |ui| {
        ui.horizontal(|ui| {
            dot(
                ui,
                if hand.connected {
                    Level::Good
                } else {
                    Level::Quiet
                },
            );
            ui.label(RichText::new(name).strong());
            if !hand.connected {
                ui.label(weak("not connected"));
            }
        });
        ui.horizontal(|ui| {
            let (rect, _) = ui.allocate_exact_size(egui::vec2(64.0, 64.0), egui::Sense::hover());
            let painter = ui.painter();
            let visuals = ui.visuals();
            let r = 28.0;
            painter.circle(
                rect.center(),
                r,
                visuals.extreme_bg_color,
                egui::Stroke::new(1.0, visuals.weak_text_color()),
            );
            painter.line_segment(
                [
                    rect.center() - egui::vec2(r, 0.0),
                    rect.center() + egui::vec2(r, 0.0),
                ],
                egui::Stroke::new(0.5, visuals.weak_text_color()),
            );
            painter.line_segment(
                [
                    rect.center() - egui::vec2(0.0, r),
                    rect.center() + egui::vec2(0.0, r),
                ],
                egui::Stroke::new(0.5, visuals.weak_text_color()),
            );
            let [x, y] = hand.stick.map(|v| v.clamp(-1.0, 1.0));
            // Stick up is positive; the screen's y grows downwards.
            painter.circle_filled(
                rect.center() + egui::vec2(x, -y) * r,
                5.0,
                color(ui, Level::Good),
            );
            ui.vertical(|ui| {
                for (label, value) in [("Trigger", hand.trigger), ("Grip", hand.grip)] {
                    ui.add(
                        egui::ProgressBar::new(value.clamp(0.0, 1.0))
                            .text(format!("{label} {:.0}%", value.clamp(0.0, 1.0) * 100.0))
                            .corner_radius(4),
                    );
                }
                ui.label(weak(format!("Stick {x:+.2}, {y:+.2}")));
            });
        });
    });
}

fn completion(ui: &mut Ui, guide: &Guide) {
    let (level, headline) = match (&guide.done, &guide.ended) {
        (Some((true, None)), _) => (Level::Good, "Setup complete — ready to play!".to_owned()),
        (Some((_, Some(error))), _) => (Level::Bad, unwrap(error)),
        (None, Some(ended)) => (Level::Bad, ended.clone()),
        _ => (Level::Warning, "Setup is not finished yet.".to_owned()),
    };
    card(ui, |ui| {
        ui.horizontal(|ui| {
            dot(ui, level);
            ui.label(
                RichText::new(headline)
                    .size(16.0)
                    .strong()
                    .color(color(ui, level)),
            );
        });
        for (stage, summary) in &guide.finished {
            let title = Stage::parse(stage).map_or(stage.as_str(), |s| s.short());
            ui.label(format!("{title}: {summary}"));
        }
        if level != Level::Good {
            if let Some(state) = &guide.state {
                for stage in state.stages.iter().filter(|s| s.state != "ready") {
                    ui.horizontal(|ui| {
                        dot(ui, state_level(&stage.state));
                        ui.label(format!("{}: {}", stage.title, stage.summary));
                    });
                }
            }
        }
    });
}

struct App {
    client: Option<SetupClient>,
    start_error: Option<String>,
    tap: Option<InputsTap>,
    starting: Option<mpsc::Receiver<std::io::Result<()>>>,
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        let guide = match (&self.client, &self.start_error) {
            (Some(client), _) => client.guide(),
            (None, error) => Guide {
                ended: Some(error.clone().unwrap_or_default()),
                ..Default::default()
            },
        };
        if guide.active_stage() == Some("buttons") {
            if self.tap.is_none() {
                let repaint = ctx.clone();
                self.tap = Some(InputsTap::start(
                    Box::new(crate::preview_client::connect),
                    move || repaint.request_repaint(),
                ));
            }
        } else {
            self.tap = None;
        }
        if let Some(result) = self.starting.as_ref().and_then(|rx| rx.try_recv().ok()) {
            self.starting = None;
            if let Err(error) = result {
                eprintln!("cv1touch-gui: could not start the daemon: {error}");
            }
            // The checks show the outcome either way.
            if let Some(client) = &mut self.client {
                client.send("recheck");
            }
        }
        let live = self.tap.as_ref().and_then(InputsTap::latest);
        let action = draw(ui, &guide, live.as_ref(), self.starting.is_some());
        match action {
            Some(SetupAction::Send(line)) => {
                if let Some(client) = &mut self.client {
                    client.send(&line);
                }
            }
            Some(SetupAction::Sensors) => {
                let _ = crate::tray::show(crate::tray::Kind::Sensors);
            }
            Some(SetupAction::StartDaemon) => {
                let (tx, rx) = mpsc::channel();
                let repaint = ctx.clone();
                if actions::control(ServiceAction::Start, move |result| {
                    let _ = tx.send(result);
                    repaint.request_repaint();
                })
                .is_ok()
                {
                    self.starting = Some(rx);
                }
            }
            Some(SetupAction::Close) => ctx.send_viewport_cmd(egui::ViewportCommand::Close),
            None => {}
        }
        ctx.request_repaint_after(Duration::from_secs(1));
    }

    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        // Stop the guide, which puts the daemon back to tracking, before the window goes.
        self.tap = None;
        self.client = None;
    }
}

pub fn run() {
    let request = REQUEST.get().copied().unwrap_or_default();
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_app_id(window::APP_ID)
            .with_inner_size(SIZE)
            .with_min_inner_size(MIN_SIZE)
            .with_icon(window::icon_data(Tint::Ready)),
        ..Default::default()
    };
    if let Err(error) = eframe::run_native(
        "cv1touch setup",
        options,
        Box::new(move |cc| {
            window::configure_style(&cc.egui_ctx);
            crate::tray::listen_for_activation(&cc.egui_ctx);
            let (client, start_error) = match SetupClient::start(request, cc.egui_ctx.clone()) {
                Ok(client) => (Some(client), None),
                Err(error) => (None, Some(format!("Could not run cv1touch setup: {error}"))),
            };
            Ok(Box::new(App {
                client,
                start_error,
                tap: None,
                starting: None,
            }))
        }),
    ) {
        eprintln!("cv1touch-gui: {error}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::setup_client::{parse_event, Event};

    fn prompt(line: &str) -> Prompt {
        match parse_event(line).unwrap() {
            Some(Event::Prompt(p)) => p,
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn hard_wrapped_text_joins_into_paragraphs() {
        assert_eq!(
            unwrap("This works out where each\nsensor is.\n\nPress A to\nstart."),
            "This works out where each sensor is.\n\nPress A to start."
        );
        assert_eq!(
            unwrap("Check:\n- the cable\n- the port"),
            "Check:\n- the cable\n- the port"
        );
        assert_eq!(
            unwrap("  2/4 Pull both triggers."),
            "2/4 Pull both triggers."
        );
    }

    #[test]
    fn prompts_offer_only_the_commands_they_accept() {
        use crate::setup_client::tests::*;
        assert_eq!(prompt(START).choices, ["go", "quit"]);
        assert_eq!(prompt_buttons(&prompt(START)), [("Start capture", "go")]);
        assert_eq!(prompt_buttons(&prompt(TRIGGERS)), [("Measure now", "go")]);
        // The headset waits for A on the controller: nothing to click but Cancel.
        assert!(prompt_buttons(&prompt(HEADSET_PROMPT)).is_empty());
        assert_eq!(
            prompt_buttons(&prompt(RETRY)),
            [
                ("Try again", "retry"),
                ("Try without slip check", "unwatched"),
                ("Skip", "skip")
            ]
        );
        let mut retry = prompt(RETRY);
        retry.choices.retain(|c| c != "unwatched");
        assert!(!prompt_buttons(&retry).iter().any(|b| b.1 == "unwatched"));
        // The stage list has its own buttons.
        assert!(prompt_buttons(&prompt(MENU)).is_empty());
    }
}
