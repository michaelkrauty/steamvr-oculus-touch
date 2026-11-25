// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

use crate::{
    actions::{self, ServiceAction},
    client::{Client, Update},
    config_stats::ConfigStats,
    icon::{self, Tint},
    model::{self, Daemon, Level, Snapshot},
};
use eframe::egui::{self, Color32, RichText, Ui};
use serde_json::Value;
use std::time::{Duration, Instant};

#[path = "stats.rs"]
mod stats;

/// The window's app ID, which Wayland compositors match to a desktop entry of that name.
pub const APP_ID: &str = "cv1touch-gui";

/// The default window fits a healthy setup's overview snugly; it grows to fit more.
pub const SIZE: [f32; 2] = [580.0, 410.0];
/// The heights between which the window follows its overview, and the Stats tab's least.
const FIT: (f32, f32) = (320.0, 620.0);
const STATS_HEIGHT: f32 = 560.0;

/// The window height at which the overview, `content` tall and seen through `visible` of a
/// window `window` tall, fits with a little room to spare.
pub fn fit_height(window: f32, (content, visible): (f32, f32)) -> f32 {
    (window - visible + content + 10.0).clamp(FIT.0, FIT.1)
}

fn color(ui: &Ui, level: Level) -> Color32 {
    let dark = ui.visuals().dark_mode;
    match (level, dark) {
        (Level::Good, true) => Color32::from_rgb(52, 199, 123),
        (Level::Good, false) => Color32::from_rgb(10, 135, 80),
        (Level::Warning, true) => Color32::from_rgb(240, 170, 40),
        (Level::Warning, false) => Color32::from_rgb(178, 105, 0),
        (Level::Bad, true) => Color32::from_rgb(248, 96, 88),
        (Level::Bad, false) => Color32::from_rgb(196, 40, 40),
        (Level::Quiet, _) => ui.visuals().weak_text_color(),
    }
}

fn dot(ui: &mut Ui, level: Level) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::hover());
    ui.painter()
        .circle_filled(rect.center(), 4.5, color(ui, level));
}

fn card<R>(ui: &mut Ui, add: impl FnOnce(&mut Ui) -> R) -> egui::InnerResponse<R> {
    let visuals = ui.visuals();
    egui::Frame::new()
        .fill(visuals.faint_bg_color)
        .stroke(visuals.widgets.noninteractive.bg_stroke)
        .corner_radius(8)
        .inner_margin(egui::Margin::symmetric(12, 7))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            add(ui)
        })
}

fn title(ui: &mut Ui, text: &str) {
    ui.label(RichText::new(text).strong());
}

fn weak(text: impl Into<String>) -> RichText {
    RichText::new(text.into()).small().weak()
}

fn mono(text: impl Into<String>) -> RichText {
    RichText::new(text.into()).monospace()
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Tab {
    #[default]
    Overview,
    Stats,
}

impl Tab {
    // A later tab is one more entry here.
    const ALL: [(Tab, &'static str); 2] = [(Tab::Overview, "Overview"), (Tab::Stats, "Stats")];
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    Service(ServiceAction),
    Setup,
    Report,
    Recheck,
}

/// The status icon in each colour, for the header.
pub struct Icons([egui::TextureHandle; 3]);

impl Icons {
    pub fn load(ctx: &egui::Context) -> Self {
        Self([Tint::Ready, Tint::Warning, Tint::Stopped].map(|tint| {
            let rgba = icon::rgba(tint, 64);
            ctx.load_texture(
                format!("icon-{tint:?}"),
                egui::ColorImage::from_rgba_unmultiplied([64, 64], &rgba),
                egui::TextureOptions::LINEAR,
            )
        }))
    }

    fn get(&self, tint: Tint) -> &egui::TextureHandle {
        &self.0[tint as usize]
    }
}

/// Everything one frame of the window shows.
pub struct View<'a> {
    pub snap: &'a Snapshot,
    pub config: &'a ConfigStats,
    pub daemon: Daemon,
    pub notice: Option<&'a (Level, String)>,
    pub icons: &'a Icons,
}

/// What one frame did: the button pressed, the icon's tint, and the tab's content and
/// visible heights.
pub struct Drawn {
    pub action: Option<Action>,
    pub tint: Tint,
    pub content: (f32, f32),
}

struct Pending {
    action: ServiceAction,
    done: Option<Instant>,
}

/// How long a finished service command may take to show in the status.
const CONFIRM: Duration = Duration::from_secs(15);

/// The window size last asked for, and whether the user has since sized it themselves.
struct Fit {
    requested: egui::Vec2,
    at: Instant,
    by_user: bool,
}

/// How long a requested size may take to apply before a different size counts as the user's.
const SETTLE: Duration = Duration::from_millis(500);

struct App {
    client: Client,
    snapshot: Snapshot,
    config: ConfigStats,
    tab: Tab,
    icons: Icons,
    pending: Option<Pending>,
    notice: Option<(Level, String)>,
    status_at: Option<Instant>,
    doctor_at: Option<Instant>,
    action_tx: std::sync::mpsc::Sender<Result<(), String>>,
    action_rx: std::sync::mpsc::Receiver<Result<(), String>>,
    tint: Tint,
    fit: Fit,
}

impl App {
    fn receive(&mut self) {
        while let Ok(result) = self.action_rx.try_recv() {
            match (result, &mut self.pending) {
                (Ok(()), Some(pending)) => {
                    pending.done = Some(Instant::now());
                    self.client.poll();
                }
                (Err(error), pending) => {
                    let label = pending
                        .as_ref()
                        .map_or("Service action", |p| p.action.label());
                    self.notice = Some((Level::Bad, format!("{label} failed: {error}")));
                    *pending = None;
                }
                (Ok(()), None) => {}
            }
        }
        while let Ok(update) = self.client.updates.try_recv() {
            match update {
                Update::Status(result, at) => {
                    self.status_at = Some(at);
                    self.snapshot.status(result, at);
                    if let Some(Pending {
                        action,
                        done: Some(done),
                    }) = self.pending
                    {
                        if at > done && confirmed(action, &self.snapshot) {
                            self.pending = None;
                        }
                    }
                }
                Update::Doctor(result) => {
                    self.doctor_at = Some(Instant::now());
                    self.snapshot.doctor(result);
                }
                Update::Config(config) => self.config = config,
            }
        }
        let now = Instant::now();
        if let Some(Pending {
            action,
            done: Some(done),
        }) = self.pending
        {
            if now.saturating_duration_since(done) >= CONFIRM {
                self.pending = None;
                self.notice = Some((
                    Level::Warning,
                    format!(
                        "{} finished, but the status did not confirm it within {} s",
                        action.label(),
                        CONFIRM.as_secs()
                    ),
                ));
            }
        }
        if self
            .status_at
            .is_some_and(|at| now.saturating_duration_since(at).as_secs() >= 6)
        {
            self.snapshot.status(Err("Status is stale".into()), now);
            self.status_at = None;
        }
        if self
            .doctor_at
            .is_some_and(|at| now.saturating_duration_since(at).as_secs() >= 45)
        {
            self.snapshot.doctor(Err("Checks are stale".into()));
            self.doctor_at = None;
        }
    }

    fn act(&mut self, action: Action, ctx: &egui::Context) {
        self.notice = None;
        let started = match action {
            Action::Service(service) => {
                let tx = self.action_tx.clone();
                let ctx = ctx.clone();
                let result = actions::control(service, move |result| {
                    let _ = tx.send(result.map_err(|e| e.to_string()));
                    ctx.request_repaint();
                });
                if result.is_ok() {
                    self.pending = Some(Pending {
                        action: service,
                        done: None,
                    });
                }
                result
            }
            Action::Setup => actions::setup(),
            Action::Report => actions::report(),
            Action::Recheck => {
                self.client.refresh();
                self.client.poll();
                Ok(())
            }
        };
        if let Err(error) = started {
            self.notice = Some((Level::Bad, format!("Could not start that: {error}")));
        }
    }
}

/// Whether the status shows a finished service action's effect.
fn confirmed(action: ServiceAction, snap: &Snapshot) -> bool {
    match action {
        ServiceAction::Start | ServiceAction::Restart => snap.status.is_some(),
        ServiceAction::Stop => snap.status_error.as_ref().is_some_and(|e| e.down),
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        self.receive();
        ui.ctx().request_repaint_after(Duration::from_secs(1));
        let view = View {
            snap: &self.snapshot,
            config: &self.config,
            daemon: Daemon::from(&self.snapshot, self.pending.as_ref().map(|p| p.action)),
            notice: self.notice.as_ref(),
            icons: &self.icons,
        };
        let checking = view.daemon == Daemon::Checking;
        let drawn = draw(ui, &view, &mut self.tab);
        let ctx = ui.ctx().clone();
        if drawn.tint != self.tint {
            // Windows and X11 show this icon; Wayland takes the desktop entry's.
            self.tint = drawn.tint;
            ctx.send_viewport_cmd(egui::ViewportCommand::Icon(Some(
                icon_data(self.tint).into(),
            )));
        }
        if !checking {
            self.fit(&ctx, drawn.content);
        }
        if let Some(action) = drawn.action {
            self.act(action, &ctx);
        }
    }
}

impl App {
    /// Sizes the window to its overview, and taller for Stats, until the user sizes it.
    fn fit(&mut self, ctx: &egui::Context, content: (f32, f32)) {
        let Some(size) = ctx.input(|i| i.viewport().inner_rect).map(|r| r.size()) else {
            return;
        };
        let now = Instant::now();
        if self.fit.by_user || now.saturating_duration_since(self.fit.at) < SETTLE {
            return;
        }
        if (size - self.fit.requested).abs().max_elem() > 6.0 {
            self.fit.by_user = true;
            return;
        }
        let height = match self.tab {
            Tab::Overview => fit_height(size.y, content),
            Tab::Stats => size.y.max(STATS_HEIGHT),
        };
        if (height - size.y).abs() > 6.0 {
            let requested = egui::vec2(size.x, height);
            ctx.send_viewport_cmd(egui::ViewportCommand::InnerSize(requested));
            self.fit = Fit {
                requested,
                at: now,
                by_user: false,
            };
        }
    }
}

fn icon_data(tint: Tint) -> egui::IconData {
    egui::IconData {
        rgba: icon::rgba(tint, 64),
        width: 64,
        height: 64,
    }
}

/// The icon's tint for the headline's level: grey while nothing is known or the daemon is
/// stopped, as in the tray.
fn tint(snap: &Snapshot, level: Level) -> Tint {
    match level {
        Level::Good => Tint::Ready,
        Level::Bad if snap.status_error.as_ref().is_some_and(|e| e.down) => Tint::Stopped,
        Level::Warning | Level::Bad => Tint::Warning,
        Level::Quiet => Tint::Stopped,
    }
}

/// The headline and its level.
fn headline(snap: &Snapshot, problems: &[(Level, String)]) -> (&'static str, Level) {
    if let Some(error) = &snap.status_error {
        if error.down {
            ("Daemon stopped", Level::Bad)
        } else {
            ("Daemon not responding", Level::Bad)
        }
    } else if snap.status.is_none() {
        ("Checking…", Level::Quiet)
    } else if snap.doctor.as_ref().is_some_and(|v| {
        v["checks"].as_array().is_some_and(|checks| {
            checks.iter().any(|c| {
                c["detail"]
                    .as_str()
                    .is_some_and(|d| d.starts_with("todo: "))
            })
        })
    }) {
        ("Not set up yet", Level::Warning)
    } else if snap.doctor_error.is_some() || !problems.is_empty() {
        ("Needs attention", Level::Warning)
    } else if snap.doctor.is_none() {
        ("Checking setup…", Level::Quiet)
    } else {
        match live_level(snap) {
            Level::Good => ("Ready to play", Level::Good),
            Level::Quiet => ("Checking tracking…", Level::Quiet),
            _ => ("Needs attention", Level::Warning),
        }
    }
}

fn live_level(snap: &Snapshot) -> Level {
    let Some(status) = &snap.status else {
        return Level::Quiet;
    };
    match status["engine_ready"].as_bool() {
        Some(false) => return Level::Warning,
        None => return Level::Quiet,
        Some(true) => {}
    }
    let Some(sensors) = status["sensors"].as_array() else {
        return Level::Quiet;
    };
    if sensors.is_empty() {
        return Level::Warning;
    }
    let Some(hands) = status["hands"].as_array() else {
        return Level::Quiet;
    };
    if hands.len() != 2 {
        return Level::Quiet;
    }
    let mut unknown = false;
    for hand in hands {
        match hand["flags"].as_u64() {
            Some(flags) if model::controller(flags).2 == Level::Good => {}
            Some(_) => return Level::Warning,
            None => unknown = true,
        }
    }
    for sensor in sensors {
        if sensor["placed"].as_u64().is_none() {
            unknown = true;
            continue;
        }
        match sensor_state(snap, sensor).2 {
            Level::Bad | Level::Warning => return Level::Warning,
            Level::Quiet => unknown = true,
            Level::Good => {}
        }
    }
    if unknown {
        Level::Quiet
    } else {
        Level::Good
    }
}

fn key(sensor: &Value) -> String {
    sensor["serial"].as_str().unwrap_or("").to_ascii_uppercase()
}

fn sensor_state(snap: &Snapshot, sensor: &Value) -> (&'static str, &'static str, Level) {
    let key = key(sensor);
    model::sensor(
        sensor,
        snap.rates.get(&key).copied(),
        snap.video_rates.get(&key).copied(),
    )
}

/// The alignment notes that need the user, from the live status.
fn alignment_warnings(status: &Value) -> Vec<(&'static str, String)> {
    let alignment = &status["alignment"];
    let mut out = Vec::new();
    if alignment["headset"] == "other" {
        out.push((
            "Other headset",
            "SteamVR uses another headset than the one cv1touch was aligned with. Run setup to redo the headset stage.".to_owned(),
        ));
    }
    if let Some(base) = alignment["unmeasured_base"].as_str() {
        out.push((
            "Base not measured",
            format!("SteamVR tracks against base station {base}, which the headset alignment did not measure. Run setup to redo the headset stage."),
        ));
    }
    if let Some(base) = alignment["base_pin"]["waiting_for"].as_str() {
        out.push((
            "Waiting for base",
            format!("Waiting for SteamVR to see base station {base}."),
        ));
    }
    out
}

pub fn draw(ui: &mut Ui, view: &View, tab: &mut Tab) -> Drawn {
    let problems = view
        .snap
        .doctor
        .as_ref()
        .map(model::problems)
        .unwrap_or_default();
    let mut action = None;
    egui::Panel::top("header")
        .frame(egui::Frame::side_top_panel(ui.style()).inner_margin(egui::Margin::symmetric(12, 8)))
        .show(ui, |ui| header(ui, view, &problems, tab));
    egui::Panel::bottom("footer")
        .frame(egui::Frame::side_top_panel(ui.style()).inner_margin(egui::Margin::symmetric(12, 8)))
        .show(ui, |ui| action = footer(ui, view));
    let content = egui::CentralPanel::default()
        .show(ui, |ui| {
            let scroll = egui::ScrollArea::vertical()
                .auto_shrink([false, false])
                .show(ui, |ui| match tab {
                    Tab::Overview => overview_tab(ui, view, &problems),
                    Tab::Stats => stats::tab(ui, view),
                });
            (scroll.content_size.y, scroll.inner_rect.height())
        })
        .inner;
    Drawn {
        action,
        tint: tint(view.snap, headline(view.snap, &problems).1),
        content,
    }
}

fn header(ui: &mut Ui, view: &View, problems: &[(Level, String)], tab: &mut Tab) {
    let (text, level) = headline(view.snap, problems);
    ui.horizontal(|ui| {
        ui.add(egui::Image::new((
            view.icons.get(tint(view.snap, level)).id(),
            egui::vec2(32.0, 32.0),
        )));
        ui.vertical(|ui| {
            ui.spacing_mut().item_spacing.y = 0.0;
            ui.label(
                RichText::new(text)
                    .size(18.0)
                    .strong()
                    .color(color(ui, level)),
            );
            ui.label(weak(subtitle(view)));
        });
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            egui::Frame::new()
                .fill(ui.visuals().extreme_bg_color)
                .corner_radius(8)
                .inner_margin(egui::Margin::same(3))
                .show(ui, |ui| {
                    ui.spacing_mut().item_spacing.x = 2.0;
                    // Right to left: add the tabs in reverse so they read left to right.
                    for (value, label) in Tab::ALL.iter().rev() {
                        ui.selectable_value(tab, *value, RichText::new(*label).size(14.0));
                    }
                });
        });
    });
}

fn subtitle(view: &View) -> String {
    match (&view.snap.status, view.daemon) {
        (_, Daemon::Busy(action)) => action.progress().to_owned(),
        (Some(status), _) => {
            let sensors = status["sensors"].as_array().map_or(0, Vec::len);
            let mode = match status["mode"].as_str() {
                Some("calibrated") => "calibrated",
                Some("capture") => "calibration capture",
                Some("provisional") => "provisional calibration",
                _ => "starting",
            };
            format!(
                "Daemon running · {mode} · {sensors} sensor{}",
                if sensors == 1 { "" } else { "s" }
            )
        }
        (None, Daemon::Stopped) => "cv1touchd is not running".to_owned(),
        (None, Daemon::Running) => "cv1touchd is not answering".to_owned(),
        (None, _) => "Looking for cv1touchd".to_owned(),
    }
}

fn footer(ui: &mut Ui, view: &View) -> Option<Action> {
    let mut action = None;
    if let Some((level, text)) = view.notice {
        ui.horizontal(|ui| {
            dot(ui, *level);
            ui.label(RichText::new(text).color(color(ui, *level)));
        });
    }
    ui.horizontal(|ui| {
        match view.daemon {
            Daemon::Checking => {
                ui.add_enabled(false, egui::Button::new("Checking daemon…"));
            }
            Daemon::Busy(service) => {
                ui.add(egui::Spinner::new());
                ui.label(service.progress());
            }
            daemon => {
                for service in [
                    ServiceAction::Start,
                    ServiceAction::Restart,
                    ServiceAction::Stop,
                ] {
                    if daemon.allows(service)
                        && ui
                            .button(service.label())
                            .on_hover_text(service.tooltip())
                            .clicked()
                    {
                        action = Some(Action::Service(service));
                    }
                }
            }
        }
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            for (label, value, tip) in [
                (
                    "Recheck",
                    Action::Recheck,
                    "Re-run cv1touch doctor diagnostics right now (normally runs every 30 s).",
                ),
                (
                    "Save report…",
                    Action::Report,
                    "Collect logs, config summary, and status into a diagnostic zip file for troubleshooting.",
                ),
                (
                    "Run setup…",
                    Action::Setup,
                    "Open cv1touch setup in a terminal to calibrate sensors, align your headset, or redo a stage.",
                ),
            ] {
                if ui.button(label).on_hover_text(tip).clicked() {
                    action = Some(value);
                }
            }
        });
    });
    action
}

fn overview_tab(ui: &mut Ui, view: &View, problems: &[(Level, String)]) {
    let snap = view.snap;
    let mut alerts: Vec<(Level, String)> = Vec::new();
    if let Some(error) = &snap.doctor_error {
        alerts.push((Level::Warning, format!("Checks unavailable: {error}")));
    }
    if let Some(error) = snap.status_error.as_ref().filter(|e| !e.down) {
        alerts.push((Level::Bad, error.message.clone()));
    }
    alerts.extend(problems.iter().cloned());
    if !alerts.is_empty() {
        let level = alerts[0].0;
        let tint = color(ui, level);
        egui::Frame::new()
            .fill(tint.gamma_multiply(0.14))
            .stroke(egui::Stroke::new(1.0, tint.gamma_multiply(0.6)))
            .corner_radius(8)
            .inner_margin(egui::Margin::symmetric(12, 8))
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                let id = ui.make_persistent_id("more-problems");
                let mut more = ui.data(|d| d.get_temp(id).unwrap_or(false));
                ui.horizontal(|ui| {
                    dot(ui, level);
                    ui.label(RichText::new("What to do").strong().color(tint));
                    if alerts.len() > 1 {
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            let label = if more {
                                "Show less".to_owned()
                            } else {
                                format!("{} more", alerts.len() - 1)
                            };
                            if ui.link(RichText::new(label).small()).clicked() {
                                more = !more;
                                ui.data_mut(|d| d.insert_temp(id, more));
                            }
                        });
                    }
                });
                ui.label(&alerts[0].1);
                if more {
                    for (level, text) in &alerts[1..] {
                        ui.horizontal_wrapped(|ui| {
                            dot(ui, *level);
                            ui.label(text);
                        });
                    }
                }
            });
    }
    match &snap.status {
        Some(status) => {
            ui.columns(2, |columns| {
                for (i, ui) in columns.iter_mut().enumerate() {
                    controller_card(ui, snap, &status["hands"][i], i);
                }
            });
            sensors_card(ui, snap, status);
        }
        None => {
            card(ui, |ui| {
                title(ui, "Controllers and sensors");
                ui.label(
                    RichText::new(match view.daemon {
                        Daemon::Stopped => {
                            "No live status while the daemon is stopped. Start it below."
                        }
                        Daemon::Checking => "Waiting for the first status…",
                        _ => "No live status from the daemon right now.",
                    })
                    .weak(),
                );
            });
        }
    }
    calibration_card(ui, view);
}

fn controller_card(ui: &mut Ui, snap: &Snapshot, hand: &Value, i: usize) {
    let name = if i == 0 { "Left" } else { "Right" };
    let flags = hand["flags"].as_u64();
    let (state, hint, level) = flags.map_or(
        ("Unknown", "No status yet.", Level::Quiet),
        model::controller,
    );
    let asleep = flags.is_some_and(|f| f & 1 == 0 || f & 16 != 0);
    card(ui, |ui| {
        ui.horizontal(|ui| {
            dot(ui, level);
            ui.label(RichText::new(name).strong());
            if let Some(rate) = snap.hand_rates[i].filter(|_| !asleep) {
                ui.label(weak(format!("{rate:.0} Hz")));
            }
            if let Some(loss) = hand["radio_lost_percent"].as_f64().filter(|_| !asleep) {
                let text = RichText::new(format!("{loss:.1}% loss")).small();
                ui.label(if loss > 10.0 {
                    text.color(color(ui, Level::Warning))
                } else {
                    text.weak()
                });
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(RichText::new(state).color(color(ui, level)));
            });
        });
        match hand["battery_percent"]
            .as_i64()
            .filter(|v| (0..=100).contains(v) && !asleep)
        {
            Some(battery) => {
                ui.horizontal(|ui| {
                    let fill = color(
                        ui,
                        if battery <= 15 {
                            Level::Warning
                        } else {
                            Level::Good
                        },
                    );
                    ui.add(
                        egui::ProgressBar::new(battery as f32 / 100.0)
                            .desired_width(ui.available_width() - 44.0)
                            .desired_height(8.0)
                            .corner_radius(4)
                            .fill(fill),
                    );
                    ui.label(mono(format!("{battery:>3}%")));
                });
            }
            None => {
                ui.label(weak(hint));
            }
        }
    })
    .response
    .on_hover_text(hand_tooltip(hand, hint));
}

fn hand_tooltip(hand: &Value, hint: &str) -> String {
    let t = &hand["tracker"];
    let mut text = format!(
        "{hint}\nTracker: {} updates, {} acquisitions, {} wrong locks, {} refused",
        t["updates"], t["acquisitions"], t["wrong_locks"], t["refused"]
    );
    if let Some(bias) = vec3(&t["accel_bias"]) {
        text += &format!("\nAccelerometer bias: {:.3} m/s²", norm(bias));
    }
    text
}

fn vec3(value: &Value) -> Option<[f64; 3]> {
    let a = value.as_array()?;
    Some([
        a.first()?.as_f64()?,
        a.get(1)?.as_f64()?,
        a.get(2)?.as_f64()?,
    ])
}

fn norm(v: [f64; 3]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

fn agreement_level(median_px: f64) -> Level {
    if median_px <= 0.35 {
        Level::Good
    } else if median_px <= 0.60 {
        Level::Warning
    } else {
        Level::Bad
    }
}

fn badge(ui: &mut Ui, level: Level, text: impl Into<String>) -> egui::Response {
    let tint = color(ui, level);
    egui::Frame::new()
        .fill(tint.gamma_multiply(0.16))
        .corner_radius(6)
        .inner_margin(egui::Margin::symmetric(6, 0))
        .show(ui, |ui| {
            ui.label(RichText::new(text.into()).monospace().color(tint))
        })
        .response
}

fn short_serial(serial: &str) -> String {
    let tail: String = serial
        .chars()
        .rev()
        .take(4)
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
        .collect();
    format!("…{tail}")
}

fn sensors_card(ui: &mut Ui, snap: &Snapshot, status: &Value) {
    card(ui, |ui| {
        title(ui, "Sensors");
        let sensors = status["sensors"].as_array().cloned().unwrap_or_default();
        if sensors.is_empty() {
            ui.label(RichText::new("No sensors found. Check that they are plugged in.").weak());
            return;
        }
        egui::Grid::new("sensors")
            .num_columns(4)
            .spacing([18.0, 3.0])
            .min_col_width(0.0)
            .show(ui, |ui| {
                for (i, sensor) in sensors.iter().enumerate() {
                    let serial = sensor["serial"].as_str().unwrap_or("unknown");
                    let (state, hint, level) = sensor_state(snap, sensor);
                    ui.horizontal(|ui| {
                        dot(ui, level);
                        ui.label(format!("Sensor {}", i + 1));
                        ui.label(weak(short_serial(serial)));
                    })
                    .response
                    .on_hover_text(format!("Serial {serial}"));
                    ui.label(RichText::new(state).color(color(ui, level)))
                        .on_hover_text(hint);
                    let fps = snap.rates.get(&key(sensor)).map_or("–".to_owned(), |r| format!("{r:.1} fps"));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(mono(format!("{fps:>9}")));
                    });
                    let a = &sensor["agreement"];
                    match a["median_px"].as_f64() {
                        Some(median) => {
                            badge(ui, agreement_level(median), format!("{median:.2} px")).on_hover_text(format!(
                                "How closely this sensor agrees with the others on the controllers' positions.\nMedian {median:.2} px, bias {:.2} px over {} checks.\nOptical fixes: left {}, right {}",
                                a["bias_px"].as_f64().unwrap_or(f64::NAN),
                                a["checks"],
                                sensor["fixes"][0],
                                sensor["fixes"][1],
                            ));
                        }
                        None => {
                            ui.label(weak("no checks yet"));
                        }
                    }
                    ui.end_row();
                }
            });
    });
}

fn chip(ui: &mut Ui, level: Level, name: &str, value: &str, tip: String) {
    egui::Frame::new()
        .fill(ui.visuals().extreme_bg_color)
        .stroke(ui.visuals().widgets.noninteractive.bg_stroke)
        .corner_radius(12)
        .inner_margin(egui::Margin::symmetric(8, 3))
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 5.0;
                dot(ui, level);
                ui.label(RichText::new(name).strong());
                ui.label(RichText::new(value).color(color(ui, level)));
            });
        })
        .response
        .on_hover_text(tip);
}

fn calibration_card(ui: &mut Ui, view: &View) {
    let status = view.snap.status.as_ref();
    let config = view.config;
    card(ui, |ui| {
        ui.horizontal(|ui| {
            title(ui, "Calibration and alignment");
            // One chip for any alignment notes keeps the card one row of chips tall.
            let warnings = status.map(alignment_warnings).unwrap_or_default();
            if let Some((name, _)) = warnings.first() {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let more = match warnings.len() {
                        1 => String::new(),
                        n => format!("+{}", n - 1),
                    };
                    let tip = warnings
                        .iter()
                        .map(|(_, tip)| tip.as_str())
                        .collect::<Vec<_>>()
                        .join("\n");
                    chip(ui, Level::Warning, name, &more, tip);
                });
            }
        });
        ui.horizontal(|ui| {
            // Sensors.
            let calibrated = status.map(|s| s["mode"] == "calibrated");
            match (config.sensor_fit(), calibrated) {
                (Some((median, n)), _) => {
                    let stats = &config.calibration.as_ref().unwrap()["stats"];
                    chip(ui, Level::Good, "Sensors", &format!("{median:.2} px"), format!(
                        "Room calibration of {n} sensors: median reprojection error {median:.2} px.\nGravity spread {:.2}°, {} samples.",
                        stats["gravity_spread_deg"].as_f64().unwrap_or(f64::NAN),
                        stats["samples"],
                    ));
                }
                (None, Some(true)) => chip(ui, Level::Good, "Sensors", "Calibrated", "The daemon tracks with a saved room calibration.".into()),
                (None, _) => chip(ui, Level::Warning, "Sensors", "Not calibrated", "Run setup to calibrate the sensors.".into()),
            }
            // Headset.
            let registered = status.and_then(|s| s["alignment"]["registration"].as_bool());
            match (config.headset_fit(), registered) {
                (Some((mm, deg)), _) => {
                    let r = config.registration.as_ref().unwrap();
                    let measured = r["measured"].as_array().map_or(0, Vec::len);
                    let stations = r["stations"].as_array().map_or(0, Vec::len);
                    chip(ui, Level::Good, "Headset", &format!("{mm:.1} mm · {deg:.1}°"), format!(
                        "Headset alignment: {mm:.1} mm and {deg:.1}° RMS over {} pairs, against {} tracking.\n{measured} of {stations} base stations measured.",
                        r["pairs"], r["reference_system"].as_str().unwrap_or("SteamVR"),
                    ));
                }
                (None, Some(true)) => chip(ui, Level::Good, "Headset", "Aligned", "cv1touch is aligned with SteamVR's headset tracking.".into()),
                (None, _) => chip(ui, Level::Warning, "Headset", "Not aligned", "Run setup to align cv1touch with the headset.".into()),
            }
            // Controllers.
            let fits = [config.render_fit("left"), config.render_fit("right")];
            let render = status.map(|s| [0, 1].map(|i| s["alignment"]["render"][i] == true));
            if let [Some((l, ln)), Some((r, rn))] = fits {
                chip(ui, Level::Good, "Controllers", &format!("{:.2} mm", l.max(r)), format!(
                    "Controller model offsets: left {l:.2} mm ({ln} LEDs), right {r:.2} mm ({rn} LEDs)."
                ));
            } else if render == Some([true, true]) {
                chip(ui, Level::Good, "Controllers", "Aligned", "Both controller models are aligned with their tracked LEDs.".into());
            } else {
                chip(ui, Level::Warning, "Controllers", "Not aligned", "Run setup to align the controller models.".into());
            }
        });
    });
}

pub(crate) fn configure_style(ctx: &egui::Context) {
    ctx.all_styles_mut(|s| {
        s.spacing.item_spacing = egui::vec2(8.0, 5.0);
        s.spacing.button_padding = egui::vec2(10.0, 4.0);
        s.text_styles
            .insert(egui::TextStyle::Body, egui::FontId::proportional(14.0));
        s.text_styles
            .insert(egui::TextStyle::Button, egui::FontId::proportional(14.0));
        s.text_styles
            .insert(egui::TextStyle::Monospace, egui::FontId::monospace(13.0));
    });
}

pub fn run() {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_app_id(APP_ID)
            .with_inner_size(SIZE)
            .with_min_inner_size([480.0, FIT.0])
            .with_icon(icon_data(Tint::Stopped)),
        ..Default::default()
    };
    if let Err(error) = eframe::run_native(
        "cv1touch",
        options,
        Box::new(|cc| {
            configure_style(&cc.egui_ctx);
            crate::tray::listen_for_activation(&cc.egui_ctx);
            let (action_tx, action_rx) = std::sync::mpsc::channel();
            Ok(Box::new(App {
                client: Client::start(cc.egui_ctx.clone()),
                snapshot: Snapshot::default(),
                config: ConfigStats::default(),
                tab: Tab::Overview,
                icons: Icons::load(&cc.egui_ctx),
                pending: None,
                notice: None,
                status_at: None,
                doctor_at: None,
                action_tx,
                action_rx,
                tint: Tint::Stopped,
                fit: Fit {
                    requested: SIZE.into(),
                    at: Instant::now(),
                    by_user: false,
                },
            }))
        }),
    ) {
        eprintln!("cv1touch-gui: {error}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::client::StatusError;
    use serde_json::json;

    #[test]
    fn live_summary_is_conservative() {
        let mut snap = Snapshot::default();
        snap.status =
            Some(json!({"engine_ready": true, "hands": [{"flags":7},{"flags":7}], "sensors": []}));
        assert!(live_level(&snap) == Level::Warning);
        snap.status.as_mut().unwrap()["engine_ready"] = json!(false);
        assert!(live_level(&snap) == Level::Warning);
        snap.status = Some(json!({}));
        assert!(live_level(&snap) == Level::Quiet);
        snap.status = Some(
            json!({"engine_ready":true,"hands":[{"flags":7},{"flags":7}],"sensors":[{"serial":"a","placed":1,"pose":{"state":"given_up"}}]}),
        );
        snap.doctor = Some(json!({"checks":[]}));
        assert!(live_level(&snap) == Level::Warning);
        snap.status.as_mut().unwrap()["sensors"][0]["pose"] = json!(null);
        snap.rates.insert("A".into(), 60.0);
        assert!(live_level(&snap) == Level::Quiet);
        snap.video_rates.insert("A".into(), 60.0);
        assert!(live_level(&snap) == Level::Good);
    }

    #[test]
    fn service_actions_follow_the_daemon_state() {
        use ServiceAction::*;
        let mut snap = Snapshot::default();
        let offered = |daemon: Daemon| {
            [Start, Restart, Stop]
                .into_iter()
                .filter(|a| daemon.allows(*a))
                .collect::<Vec<_>>()
        };
        assert_eq!(Daemon::from(&snap, None), Daemon::Checking);
        assert!(offered(Daemon::Checking).is_empty());
        snap.status(Ok(json!({"generation":1})), Instant::now());
        assert_eq!(Daemon::from(&snap, None), Daemon::Running);
        assert_eq!(offered(Daemon::Running), [Restart, Stop]);
        snap.status(
            Err(StatusError {
                down: true,
                message: "cv1touchd isn't running".into(),
            }),
            Instant::now(),
        );
        assert_eq!(Daemon::from(&snap, None), Daemon::Stopped);
        assert_eq!(offered(Daemon::Stopped), [Start]);
        // A daemon that does not answer can still be restarted or stopped.
        snap.status(Err("timed out".into()), Instant::now());
        assert_eq!(Daemon::from(&snap, None), Daemon::Running);
        let busy = Daemon::from(&snap, Some(Restart));
        assert_eq!(busy, Daemon::Busy(Restart));
        assert!(offered(busy).is_empty());
        assert!(!confirmed(Stop, &snap));
        snap.status(
            Err(StatusError {
                down: true,
                message: String::new(),
            }),
            Instant::now(),
        );
        assert!(confirmed(Stop, &snap) && !confirmed(Start, &snap));
    }

    #[test]
    fn alignment_warnings_only_when_needed() {
        assert!(
            alignment_warnings(&json!({"alignment":{"headset":null,"base_pin":{}}})).is_empty()
        );
        assert_eq!(
            alignment_warnings(&json!({"alignment":{"headset":"other","unmeasured_base":"LHB-1","base_pin":{"waiting_for":"LHB-2"}}})).len(),
            3
        );
    }

    #[test]
    fn agreement_badge_thresholds() {
        assert_eq!(agreement_level(0.35), Level::Good);
        assert_eq!(agreement_level(0.36), Level::Warning);
        assert_eq!(agreement_level(0.61), Level::Bad);
        assert_eq!(short_serial("SENSOR-0001"), "…0001");
    }
}
