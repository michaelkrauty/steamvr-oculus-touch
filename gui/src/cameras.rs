// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The sensor views window: every sensor's live infrared image with its blobs, to aim the sensors
//! and see which LEDs they match. Images flow only while the window is open.

use crate::{
    icon::Tint,
    preview_client::{Feed, Mode, Preview, PreviewError, SensorFrame},
    window,
};
use eframe::egui::{self, Align2, Color32, Pos2, Rect, RichText, Sense, Stroke, Ui, Vec2};
use std::collections::HashMap;

pub const SIZE: [f32; 2] = [960.0, 660.0];
/// Narrower, the toolbar of one sensor's view no longer fits.
pub const MIN_SIZE: [f32; 2] = [640.0, 400.0];
/// The canvas stays dark in either theme: infrared images read best on it.
pub const CANVAS: Color32 = Color32::from_rgb(0x12, 0x15, 0x1a);
const TILE: Color32 = Color32::from_rgb(0x0b, 0x0d, 0x11);
const CARD: Color32 = Color32::from_rgb(0x1e, 0x23, 0x2b);
pub const LEFT: Color32 = Color32::from_rgb(0x38, 0xbd, 0xf8);
pub const RIGHT: Color32 = Color32::from_rgb(0x34, 0xd3, 0x99);
pub const HEADSET: Color32 = Color32::from_rgb(0xa7, 0x8b, 0xfa);
const UNMATCHED: Color32 = Color32::from_rgba_unmultiplied_const(190, 190, 200, 150);
const AMBER: Color32 = Color32::from_rgb(0xf5, 0x9e, 0x0b);
const TEXT: Color32 = Color32::from_rgb(0xe2, 0xe8, 0xf0);
const HINT: Color32 = Color32::from_rgb(0x94, 0xa3, 0xb8);
/// Bright pixels per thousand from which a sensor is flooded by sunlight or a lamp.
const GLARE_PERMILLE: u32 = 15;
/// A CV1 sensor's image, for blobs from a sensor that sends none.
const SENSOR_SIZE: (f32, f32) = (1280.0, 960.0);

#[derive(Clone)]
pub struct Settings {
    pub blobs: bool,
    pub enhance: bool,
    pub hud: bool,
    /// The sensor shown alone, by serial.
    pub zoom: Option<String>,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            blobs: true,
            enhance: true,
            hud: true,
            zoom: None,
        }
    }
}

impl Settings {
    pub fn mode(&self) -> Mode {
        self.zoom.clone().map_or(Mode::All, Mode::One)
    }
}

/// GRAY8 to RGB. Raw is linear; enhanced lifts dim levels (a 0.42 gamma: 8 becomes 60) so
/// that the room shows, keeps 0 and 255, and tints toward slate so the overlays stand out.
pub fn tone_map(enhance: bool) -> [[u8; 3]; 256] {
    std::array::from_fn(|v| {
        let v = v as f32 / 255.0;
        if !enhance {
            return [(v * 255.0).round() as u8; 3];
        }
        let l = v.powf(0.42);
        [l * (0.86 + 0.14 * l), l * (0.94 + 0.06 * l), l].map(|c| (c * 255.0).round() as u8)
    })
}

/// Each sensor's image as a texture, redone only for a new frame or another tone mapping.
#[derive(Default)]
pub struct Textures(HashMap<String, (u64, bool, egui::TextureHandle)>);

impl Textures {
    pub fn update(&mut self, ctx: &egui::Context, feed: &Feed, enhance: bool) {
        let Feed::Live(snapshot) = feed else {
            self.0.clear();
            return;
        };
        let lut = std::cell::OnceCell::new();
        self.0.retain(|serial, _| {
            snapshot
                .sensors
                .iter()
                .any(|s| s.width > 0 && s.serial == *serial)
        });
        for sensor in snapshot.sensors.iter().filter(|s| s.width > 0) {
            if self
                .0
                .get(&sensor.serial)
                .is_some_and(|(frame, e, _)| (*frame, *e) == (sensor.frame_ns, enhance))
            {
                continue;
            }
            let lut = lut.get_or_init(|| tone_map(enhance));
            let rgb: Vec<u8> = sensor
                .pixels
                .iter()
                .flat_map(|&p| lut[p as usize])
                .collect();
            let image =
                egui::ColorImage::from_rgb([sensor.width as usize, sensor.height as usize], &rgb);
            let options = egui::TextureOptions::LINEAR;
            match self.0.get_mut(&sensor.serial) {
                Some((frame, e, texture)) => {
                    texture.set(image, options);
                    (*frame, *e) = (sensor.frame_ns, enhance);
                }
                None => {
                    let texture =
                        ctx.load_texture(format!("sensor-{}", sensor.serial), image, options);
                    self.0
                        .insert(sensor.serial.clone(), (sensor.frame_ns, enhance, texture));
                }
            }
        }
    }

    fn get(&self, serial: &str) -> Option<&egui::TextureHandle> {
        self.0.get(serial).map(|(_, _, texture)| texture)
    }
}

/// Grid columns for `n` sensors.
pub fn columns(n: usize) -> usize {
    match n {
        0..=1 => 1,
        2..=4 => 2,
        5..=9 => 3,
        n => n.isqrt() + usize::from(n.isqrt().pow(2) < n),
    }
}

fn short_serial(serial: &str) -> String {
    let skip = serial.chars().count().saturating_sub(4);
    format!("…{}", serial.chars().skip(skip).collect::<String>())
}

fn device_color(device: u8) -> Color32 {
    match device {
        0 => HEADSET,
        1 => RIGHT,
        2 => LEFT,
        _ => UNMATCHED,
    }
}

/// Draws the window and returns the previews it wants.
pub fn draw(ui: &mut Ui, feed: &Feed, textures: &Textures, settings: &mut Settings) -> Mode {
    let sensors: &[SensorFrame] = match feed {
        Feed::Live(snapshot) => &snapshot.sensors,
        _ => &[],
    };
    if settings.zoom.is_some() && ui.input(|i| i.key_pressed(egui::Key::Escape)) {
        settings.zoom = None;
    }
    egui::Panel::top("toolbar")
        .frame(egui::Frame::side_top_panel(ui.style()).inner_margin(egui::Margin::symmetric(12, 8)))
        .show(ui, |ui| toolbar(ui, feed, sensors, settings));
    egui::CentralPanel::default()
        .frame(
            egui::Frame::new()
                .fill(CANVAS)
                .inner_margin(egui::Margin::same(10)),
        )
        .show(ui, |ui| match feed {
            Feed::Connecting => message(ui, "Connecting to cv1touchd…", ""),
            Feed::Failed(PreviewError::DaemonStopped) => message(
                ui,
                "cv1touch daemon is not running.",
                "Start it from the status window or the tray menu.",
            ),
            Feed::Failed(PreviewError::UnsupportedDaemon) => message(
                ui,
                "The running daemon does not support live sensor views yet.",
                "Restart or update cv1touchd.",
            ),
            Feed::Failed(PreviewError::Other(error)) => {
                message(ui, "Could not show the sensor views.", error)
            }
            Feed::Live(_) if sensors.is_empty() => message(
                ui,
                "No sensors detected.",
                "Plug in a CV1 sensor to see what it sees.",
            ),
            Feed::Live(_) => match &settings.zoom {
                Some(serial) => {
                    let found = sensors
                        .iter()
                        .enumerate()
                        .find(|(_, s)| s.serial == *serial);
                    let rect = ui.available_rect_before_wrap();
                    match found {
                        Some((i, sensor)) => {
                            tile(
                                ui,
                                rect,
                                sensor_number(i, sensor),
                                sensor,
                                textures,
                                settings,
                                true,
                            );
                        }
                        None => message(ui, "Waiting for frames…", ""),
                    }
                }
                None => grid(ui, sensors, textures, settings),
            },
        });
    settings.mode()
}

fn toolbar(ui: &mut Ui, feed: &Feed, sensors: &[SensorFrame], settings: &mut Settings) {
    ui.horizontal(|ui| {
        match settings.zoom.clone() {
            Some(serial) => {
                if ui
                    .button("‹ All sensors")
                    .on_hover_text("Back to every sensor (Esc)")
                    .clicked()
                {
                    settings.zoom = None;
                }
                let label = match sensors
                    .iter()
                    .enumerate()
                    .find(|(_, s)| s.serial == serial)
                {
                    Some((i, sensor)) if sensor.width > 0 => format!(
                        "Sensor {} · {} · {}×{}",
                        sensor_number(i, sensor),
                        short_serial(&serial),
                        sensor.width,
                        sensor.height
                    ),
                    Some((i, sensor)) => format!(
                        "Sensor {} · {}",
                        sensor_number(i, sensor),
                        short_serial(&serial)
                    ),
                    None => format!("Sensor {}", short_serial(&serial)),
                };
                ui.label(RichText::new(label).size(16.0).strong())
                    .on_hover_text(format!("Serial {serial}"));
            }
            None => {
                ui.label(RichText::new("Sensor views").size(18.0).strong());
                let status = match feed {
                    Feed::Live(_) if sensors.is_empty() => "No sensors".to_owned(),
                    Feed::Live(_) if sensors.len() == 1 => "1 sensor active".to_owned(),
                    Feed::Live(_) => format!("{} sensors active", sensors.len()),
                    Feed::Connecting => "Connecting…".to_owned(),
                    Feed::Failed(_) => "Not active".to_owned(),
                };
                ui.label(RichText::new(status).weak());
            }
        }
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            // Right to left: added in reverse so that they read left to right.
            for (value, label, tip) in [
                (
                    &mut settings.hud,
                    "HUD",
                    "Show per-sensor frame rate, age, matched LED count, and infrared glare warnings.",
                ),
                (
                    &mut settings.enhance,
                    "Enhance room",
                    "Boost dim background levels so walls and furniture are visible while aiming sensors. Turn off for raw brightness.",
                ),
                (
                    &mut settings.blobs,
                    "Blobs & LEDs",
                    "Highlight detected infrared blobs (blue = left controller, green = right controller, gray = unmatched).",
                ),
            ] {
                ui.toggle_value(value, label).on_hover_text(tip);
            }
        });
    });
}

fn message(ui: &mut Ui, title: &str, hint: &str) {
    let rect = ui.available_rect_before_wrap();
    ui.scope_builder(egui::UiBuilder::new().max_rect(rect), |ui| {
        ui.vertical_centered(|ui| {
            ui.add_space((rect.height() / 2.0 - 40.0).max(0.0));
            egui::Frame::new()
                .fill(CARD)
                .corner_radius(10)
                .inner_margin(egui::Margin::symmetric(24, 16))
                .show(ui, |ui| {
                    ui.label(RichText::new(title).size(17.0).strong().color(TEXT));
                    if !hint.is_empty() {
                        ui.label(RichText::new(hint).color(HINT));
                    }
                });
        });
    });
}

fn grid(ui: &mut Ui, sensors: &[SensorFrame], textures: &Textures, settings: &mut Settings) {
    let area = ui.available_rect_before_wrap();
    let gap = 8.0;
    let cols = columns(sensors.len());
    let rows = sensors.len().div_ceil(cols);
    let cell = Vec2::new(
        (area.width() - gap * (cols - 1) as f32) / cols as f32,
        (area.height() - gap * (rows - 1) as f32) / rows as f32,
    );
    // One tile size for all, at the sensors' common aspect.
    let aspect = sensors
        .iter()
        .find(|s| s.raw_height > 0)
        .map_or(SENSOR_SIZE.0 / SENSOR_SIZE.1, |s| {
            s.raw_width as f32 / s.raw_height as f32
        });
    let size = fit(cell, aspect);
    let grid = Vec2::new(
        cols as f32 * cell.x + gap * (cols - 1) as f32,
        rows as f32 * size.y + gap * (rows - 1) as f32,
    );
    let top = area.center().y - grid.y / 2.0;
    for (i, sensor) in sensors.iter().enumerate() {
        let (row, col) = (i / cols, i % cols);
        // A short last row is centred.
        let in_row = (sensors.len() - row * cols).min(cols);
        let shift = (cols - in_row) as f32 * (cell.x + gap) / 2.0;
        let center = Pos2::new(
            area.left() + shift + col as f32 * (cell.x + gap) + cell.x / 2.0,
            top + row as f32 * (size.y + gap) + size.y / 2.0,
        );
        let rect = Rect::from_center_size(center, size);
        if tile(
            ui,
            rect,
            sensor_number(i, sensor),
            sensor,
            textures,
            settings,
            false,
        )
        .clicked()
        {
            settings.zoom = Some(sensor.serial.clone());
        }
    }
}

/// The largest size of this aspect within `cell`.
fn fit(cell: Vec2, aspect: f32) -> Vec2 {
    if cell.x / cell.y > aspect {
        Vec2::new(cell.y * aspect, cell.y)
    } else {
        Vec2::new(cell.x, cell.x / aspect)
    }
}

/// The 1-based sensor number from the daemon's full list, falling back to slice position.
fn sensor_number(index: usize, sensor: &SensorFrame) -> usize {
    sensor.number.unwrap_or(index + 1)
}

fn tile(
    ui: &mut Ui,
    area: Rect,
    number: usize,
    sensor: &SensorFrame,
    textures: &Textures,
    settings: &Settings,
    zoomed: bool,
) -> egui::Response {
    let (raw_w, raw_h) = if sensor.raw_width > 0 && sensor.raw_height > 0 {
        (sensor.raw_width as f32, sensor.raw_height as f32)
    } else {
        SENSOR_SIZE
    };
    let rect = Rect::from_center_size(area.center(), fit(area.size(), raw_w / raw_h));
    let sense = if zoomed {
        Sense::hover()
    } else {
        Sense::click()
    };
    let mut response = ui.interact(rect, ui.scope_id().with(("tile", &sensor.serial)), sense);
    if !zoomed {
        response = response
            .on_hover_cursor(egui::CursorIcon::PointingHand)
            .on_hover_text("Click to view this sensor alone");
    }
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 6, TILE);
    match textures.get(&sensor.serial) {
        Some(texture) => {
            let uv = Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0));
            painter.image(texture.id(), rect, uv, Color32::WHITE);
        }
        None => {
            let text = if sensor.blobs.is_empty() {
                "Waiting for frames…"
            } else {
                "No image from this sensor; showing its blobs"
            };
            painter.text(
                rect.center(),
                Align2::CENTER_CENTER,
                text,
                egui::FontId::proportional(14.0),
                HINT,
            );
        }
    }
    let border = if response.hovered() && !zoomed {
        Stroke::new(2.0, Color32::from_rgba_unmultiplied(255, 255, 255, 110))
    } else {
        Stroke::new(1.0, Color32::from_rgb(0x23, 0x29, 0x31))
    };
    painter.rect_stroke(rect, 6, border, egui::StrokeKind::Inside);
    if settings.blobs {
        let scale = rect.width() / raw_w;
        for blob in &sensor.blobs {
            let center = rect.min + Vec2::new(blob.x, blob.y) * scale;
            let radius = (blob.w.max(blob.h) * scale / 2.0 + 3.0).max(4.0);
            match blob.led {
                Some((device, led)) => {
                    let color = device_color(device);
                    painter.circle_stroke(center, radius, Stroke::new(2.0, color));
                    if zoomed {
                        painter.text(
                            center + Vec2::new(radius + 2.0, -radius),
                            Align2::LEFT_BOTTOM,
                            led.to_string(),
                            egui::FontId::monospace(11.0),
                            color,
                        );
                    }
                }
                None => {
                    painter.circle_stroke(center, radius, Stroke::new(1.2, UNMATCHED));
                }
            }
        }
    }
    if settings.hud {
        hud(ui, rect, number, sensor);
    }
    response
}

fn galley(ui: &Ui, text: impl Into<egui::WidgetText>) -> std::sync::Arc<egui::Galley> {
    text.into().into_galley(
        ui,
        Some(egui::TextWrapMode::Extend),
        f32::INFINITY,
        egui::TextStyle::Body,
    )
}

/// The padding around a pill's text.
const PILL: Vec2 = Vec2::new(14.0, 6.0);

/// A compact label on a pill, anchored at `at`.
fn pill(
    ui: &mut Ui,
    galley: std::sync::Arc<egui::Galley>,
    anchor: Align2,
    at: Pos2,
    fill: Color32,
) -> egui::Response {
    let rect = anchor.anchor_size(at, galley.size() + PILL);
    ui.painter().rect_filled(rect, rect.height() / 2.0, fill);
    ui.put(rect, egui::Label::new(galley).selectable(false))
}

fn hud(ui: &mut Ui, rect: Rect, number: usize, sensor: &SensorFrame) {
    let dark = Color32::from_rgba_unmultiplied(8, 10, 14, 190);
    let inset = rect.shrink(6.0);
    let glare = (sensor.bright_permille >= GLARE_PERMILLE).then(|| {
        let text = format!("IR glare {:.1}%", sensor.bright_permille as f32 / 10.0);
        galley(
            ui,
            RichText::new(text)
                .size(12.0)
                .strong()
                .color(Color32::from_rgb(0x1a, 0x12, 0x05)),
        )
    });
    // The longest label that fits beside the glare warning on a small tile.
    let room = inset.width() - PILL.x - glare.as_ref().map_or(0.0, |g| g.size().x + PILL.x + 6.0);
    let fps = sensor
        .fps
        .map_or("– fps".to_owned(), |fps| format!("{fps:.0} fps"));
    let number = format!("Sensor {number}");
    let labels = [
        format!(
            "{number} · {} · {fps} · {} ms",
            short_serial(&sensor.serial),
            sensor.age_ms
        ),
        format!("{number} · {fps}"),
        number,
    ]
    .map(|text| galley(ui, RichText::new(text).size(12.0).color(TEXT)));
    let [.., shortest] = &labels;
    let label = labels
        .iter()
        .find(|g| g.size().x <= room)
        .unwrap_or(shortest)
        .clone();
    pill(ui, label, Align2::LEFT_TOP, inset.left_top(), dark)
        .on_hover_text(format!("Serial {}", sensor.serial));
    if let Some(glare) = glare {
        pill(ui, glare, Align2::RIGHT_TOP, inset.right_top(), AMBER).on_hover_text(
            "A bright infrared source such as sunlight, a window, or a lamp is flooding this sensor.",
        );
    }
    let mut job = egui::text::LayoutJob::default();
    for (device, side, color) in [(2, "L", LEFT), (1, "R", RIGHT)] {
        if let Some(d) = sensor
            .devices
            .iter()
            .find(|d| d.device_id == device && d.matched_blobs > 0)
        {
            let text = format!(
                "{}{side} {}/{}",
                if job.is_empty() { "" } else { "  " },
                d.matched_blobs,
                d.visible_leds
            );
            RichText::new(text)
                .size(12.0)
                .strong()
                .color(color)
                .append_to(
                    &mut job,
                    ui.style(),
                    egui::FontSelection::Default,
                    egui::Align::Center,
                );
        }
    }
    if !job.is_empty() {
        let job = galley(ui, job);
        pill(ui, job, Align2::LEFT_BOTTOM, inset.left_bottom(), dark).on_hover_text(
            "LEDs matched to each controller in this frame, of those facing this sensor.",
        );
    }
}

struct App {
    preview: Option<Preview>,
    settings: Settings,
    textures: Textures,
    seen: u64,
    feed: Feed,
    mode: Mode,
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        let Some(preview) = &self.preview else {
            return;
        };
        if let Some((seen, feed)) = preview.latest(self.seen) {
            (self.seen, self.feed) = (seen, feed);
        }
        let ctx = ui.ctx().clone();
        self.textures
            .update(&ctx, &self.feed, self.settings.enhance);
        let mut mode = draw(ui, &self.feed, &self.textures, &mut self.settings);
        // Where the platform reports it (not Wayland), a minimized window takes no images.
        if ui.input(|i| i.viewport().minimized) == Some(true) {
            mode = Mode::Paused;
        }
        if mode != self.mode {
            preview.set_mode(mode.clone());
            self.mode = mode;
        }
    }

    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        // Stop the previews before the window goes, not whenever the app is dropped.
        self.preview = None;
    }
}

pub fn run() {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_app_id(window::APP_ID)
            .with_inner_size(SIZE)
            .with_min_inner_size(MIN_SIZE)
            .with_icon(window::icon_data(Tint::Ready)),
        ..Default::default()
    };
    if let Err(error) = eframe::run_native(
        "cv1touch sensors",
        options,
        Box::new(|cc| {
            window::configure_style(&cc.egui_ctx);
            crate::tray::listen_for_activation(&cc.egui_ctx);
            Ok(Box::new(App {
                preview: Some(Preview::start(cc.egui_ctx.clone())),
                settings: Settings::default(),
                textures: Textures::default(),
                seen: 0,
                feed: Feed::Connecting,
                mode: Mode::All,
            }))
        }),
    ) {
        eprintln!("cv1touch-gui: {error}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enhance_lifts_dim_levels_and_keeps_black_and_white() {
        let raw = tone_map(false);
        let lut = tone_map(true);
        assert_eq!((raw[8], raw[200]), ([8; 3], [200; 3]));
        assert_eq!((lut[0], lut[255]), ([0; 3], [255; 3]));
        for (v, pair) in lut.windows(2).enumerate() {
            assert!(
                (0..3).all(|c| pair[1][c] >= pair[0][c]),
                "not monotonic at {v}"
            );
        }
        // A dim wall at 8 becomes a visible mid-dark grey; 2..=30 all at least triple.
        assert!(lut[8].iter().all(|&c| c >= 45), "{:?}", lut[8]);
        for (v, rgb) in lut.iter().enumerate().take(31).skip(2) {
            assert!(rgb.iter().all(|&c| c as usize >= 3 * v), "{v}: {rgb:?}");
        }
        // Slate: never warmer than neutral.
        assert!(lut[60][0] <= lut[60][1] && lut[60][1] <= lut[60][2]);
    }

    #[test]
    fn grid_columns_follow_the_sensor_count() {
        let cols: Vec<usize> = (1..=10).map(columns).collect();
        assert_eq!(cols, [1, 2, 2, 2, 3, 3, 3, 3, 3, 4]);
        assert_eq!(columns(16), 4);
        assert_eq!(columns(17), 5);
    }

    #[test]
    fn short_serials_keep_the_last_four_characters() {
        assert_eq!(short_serial("TESTSENSOR0005"), "…0005");
        assert_eq!(short_serial("AB"), "…AB");
    }
}
