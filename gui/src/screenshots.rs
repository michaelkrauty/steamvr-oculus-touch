// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
// Headless CPU rendering of the actual window; no display server or GPU required.
// Set GUI_SCREENSHOTS_DIR to keep the PNGs.

use crate::{
    actions::Saved,
    actions::ServiceAction,
    cameras::{self, Settings, Textures},
    client::StatusError,
    config_stats::ConfigStats,
    model::{Daemon, Level, Snapshot},
    preview_client::{Blob, Device, Feed, Mode, PreviewError, PreviewSnapshot, SensorFrame},
    setup_client::{tests as events, Guide, Hand},
    setup_window,
    window::{self, Icons, Local, ReportState, Tab, View},
};
use eframe::egui::{self, Color32, Pos2, Rect, TextureId};
use serde_json::{json, Value};
use std::collections::HashMap;

const PPP: f32 = 1.5;

/// Runs `draw` for a few frames, as layout settles, and rasterizes the last one.
fn render<R>(
    theme: egui::Theme,
    size: [f32; 2],
    ppp: f32,
    mut draw: impl FnMut(&mut egui::Ui, &egui::Context) -> R,
) -> (Vec<u8>, (usize, usize), R, Vec<String>) {
    let (pixels, dims, result, labels) = render_boxes(theme, size, ppp, &mut draw);
    (
        pixels,
        dims,
        result,
        labels.into_iter().map(|(l, _)| l).collect(),
    )
}

type Boxes = Vec<(String, Option<egui::accesskit::Rect>)>;

/// As `render`, with each labelled node's bounds.
fn render_boxes<R>(
    theme: egui::Theme,
    size: [f32; 2],
    ppp: f32,
    mut draw: impl FnMut(&mut egui::Ui, &egui::Context) -> R,
) -> (Vec<u8>, (usize, usize), R, Boxes) {
    let ctx = egui::Context::default();
    ctx.set_theme(theme);
    ctx.set_pixels_per_point(ppp);
    ctx.enable_accesskit();
    window::configure_style(&ctx);
    let mut textures = HashMap::new();
    let mut last = None;
    for _ in 0..4 {
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, size.into())),
            ..Default::default()
        };
        let mut result = None;
        let mut frame = ctx.run_ui(input, |ui| result = Some(draw(ui, &ctx.clone())));
        apply(&mut textures, &frame.textures_delta);
        frame.textures_delta.clear();
        last = Some((frame, result.unwrap()));
    }
    let (out, result) = last.unwrap();
    let labels = out
        .platform_output
        .accesskit_update
        .as_ref()
        .expect("accessibility update")
        .nodes
        .iter()
        .filter_map(|(_, n)| n.label().or(n.value()).map(|l| (l.to_owned(), n.bounds())))
        .collect::<Boxes>();
    for (label, _) in &labels {
        assert_plain_words(label);
    }
    let prims = ctx.tessellate(out.shapes, out.pixels_per_point);
    let dims = ((size[0] * ppp) as usize, (size[1] * ppp) as usize);
    let pixels = rasterize(
        &prims,
        &textures,
        ctx.style_of(theme).visuals.panel_fill,
        dims,
        ppp,
    );
    (pixels, dims, result, labels)
}

/// The windows speak of sensors and say "active", never "camera" or "streaming": to people
/// who do not know the hardware, those read as surveillance.
fn assert_plain_words(text: &str) {
    let lower = text.to_lowercase();
    assert!(
        !lower.contains("camera")
            && !lower.contains("streaming")
            && !lower.contains("recording")
            && !lower.contains("record now"),
        "user-facing text says camera, streaming or recording: {text:?}"
    );
}

fn save(name: &str, pixels: &[u8], dims: (usize, usize)) {
    if let Some(dir) = std::env::var_os("GUI_SCREENSHOTS_DIR") {
        std::fs::create_dir_all(&dir).unwrap();
        crate::png::write(
            &std::path::PathBuf::from(dir).join(format!("{name}.png")),
            pixels,
            dims,
        )
        .unwrap();
    }
}

fn serial(i: usize) -> String {
    format!("TESTSENSOR000{}", i + 1)
}

/// A full live setup: five sensors, both controllers awake and tracked, calibrated.
fn playing() -> (Snapshot, ConfigStats) {
    let sensors: Vec<Value> = (0..5)
        .map(|i| {
            json!({
                "serial": serial(i), "placed": 1, "frames": 9000 + i,
                "video_format": if i == 4 { "raw" } else { "jpeg" },
                "frames_received": 9000, "frames_decoded": 9000, "frames_dropped": i,
                "fixes": [1200 + 40 * i, 1100 + 30 * i],
                "camera_pos": [1.2 - 0.6 * i as f64, 2.1, -1.4 + 0.7 * i as f64],
                "pose": null, "landmark_shift": null,
                "agreement": {"median_px": 0.14 + 0.03 * i as f64, "bias_px": 0.05 + 0.01 * i as f64,
                    "samples": 400, "checks": 3000 + i, "misses": 4 + i,
                    "observed_checks": 3000, "observed_misses": 4},
            })
        })
        .collect();
    let streams: Vec<Value> = (0..5)
        .map(|i| json!({"scope": "start", "serial": serial(i), "error_kind": "none", "error_code": 0, "os_error": 0, "alt_setting": if i == 4 { 1 } else { 2 }}))
        .collect();
    let hand = |name: &str, battery: i64, loss: f64, updates: u64| {
        json!({"hand": name, "flags": 7, "radio_lost_percent": loss, "battery_percent": battery,
            "tracker": {"tracking": true, "frames": updates + 40, "updates": updates, "not_visible": 12,
                "too_few": 30, "rejected": 2, "acquisitions": 1, "good": 1, "searched": 0,
                "unconfirmed": 0, "refused": 0, "out_of_order": 0, "wrong_locks": 0,
                "accel_bias": [0.012, -0.034, 0.021], "verified_accel_bias": [0.011, -0.033, 0.020]}})
    };
    let status = json!({
        "generation": 4, "shm": "/cv1touch", "tracker": "Repose", "mode": "calibrated",
        "tilt_deg": 0.0, "pin_base_station": true, "engine_ready": true,
        "input_calibration": {"configured": true, "calibrated": [true, true], "error": null},
        "alignment": {"registration": true, "render": [true, true], "universe": 1234567890u64,
            "unmeasured_base": null, "headset": null,
            "base_pin": {"base": "LHB-TEST0001", "owner": "cv1touch", "waiting_for": null}},
        "recording_status": {"state": "idle", "message": ""},
        "hands": [hand("left", 85, 0.4, 52_000), hand("right", 90, 0.6, 51_400)],
        "sensors": sensors, "sensors_unsure": 0,
        "sensor_hotplug": {"pending": 0, "scan_error": null},
        "camera_queue": {"sensors": 5, "queued_frames": 1, "dropped_frames": 3, "dropped_clocks": 0,
            "unknown_frames": 0, "pending_frames_dropped": 0},
        "ignored_sensors": [], "calibrated_sensors": (0..5).map(serial).collect::<Vec<_>>(),
        "sensor_streams": streams,
    });
    let checks = [
        "daemon",
        "headset",
        "sensors",
        "sensor_streams",
        "calibration",
        "registration",
        "steamvr_driver",
    ]
    .map(|name| json!({"name": name, "ok": true, "detail": format!("{name} looks right")}));
    let mut snap = Snapshot::default();
    for i in 0..5 {
        snap.rates.insert(serial(i), 52.0 - 0.1 * i as f64);
        snap.video_rates.insert(serial(i), 52.0 - 0.1 * i as f64);
    }
    snap.hand_rates = [Some(52.0), Some(51.0)];
    snap.status = Some(status);
    snap.doctor = Some(json!({"daemon": {}, "checks": checks}));
    let config = ConfigStats {
        calibration: Some(json!({"format": "cv1touch-calibration",
            "sensors": (0..5).map(|i| json!({"serial": serial(i), "rms_px": 0.17 + 0.02 * i as f64, "observations": 5000 + 100 * i})).collect::<Vec<_>>(),
            "stats": {"reference": serial(0), "gravity_mps2": 9.7999, "gravity_spread_deg": 0.087,
                "motion_segments": 1700, "samples": 35000, "rejected_views": 2000}})),
        registration: Some(
            json!({"format": "cv1touch-registration", "pos_rms_mm": 2.05, "rot_rms_deg": 0.92,
            "pairs": 1733, "reference_system": "lighthouse", "tracking_base": "LHB-TEST0001",
            "universe_id": 1234567890u64, "stations": [{}, {}, {}, {}], "measured": [{}, {}]}),
        ),
        render: Some(json!({"format": "cv1touch-render-offsets", "hands": [
            {"hand": "left", "rms_mm": 0.18, "leds": 22}, {"hand": "right", "rms_mm": 0.21, "leds": 22}]})),
        settings: None,
    };
    (snap, config)
}

/// One state of the status window.
struct Scene {
    snap: Snapshot,
    config: ConfigStats,
    tab: Tab,
    report: Option<ReportState>,
    local: Local,
}

fn scenario(name: &str) -> Scene {
    let (mut snap, mut config) = playing();
    let status = snap.status.as_mut().unwrap();
    let mut tab = Tab::Overview;
    let mut report = None;
    let mut local = Local::default();
    match name {
        "overview-playing" => {}
        "overview-report-saved" => {
            report = Some(ReportState::Saved(Saved {
                path: "/home/u/Desktop/cv1touch-report-2026-11-25-232800-UTC.zip".into(),
                bytes: 1_430_000,
                notice: "It holds:\n- cv1touch's logs, settings and calibration".into(),
            }))
        }
        "settings-tab" => {
            tab = Tab::Settings;
            status["tilt_deg"] = json!(7.5);
            local.autostart = Some(true);
            local.desktop = Some(true);
        }
        "stats-playing" => {
            tab = Tab::Stats;
            status["sensors"][0]["landmark_shift"] =
                json!({"dx": 0.12, "dy": -0.05, "landmarks": 14});
            status["recording_status"] =
                json!({"state": "recording", "message": "Logging to a file"});
        }
        "overview-asleep" => {
            status["hands"][0]["flags"] = json!(17);
            status["hands"][0]["battery_percent"] = json!(null);
            snap.hand_rates[0] = None;
        }
        "overview-warning-inline-fix" => {
            status["alignment"]["headset"] = json!("other");
            status["sensors"][2]["pose"] = json!({"state": "moved", "since_ns": 1, "attempts": 2});
            status["sensors"][3]["agreement"]["median_px"] = json!(0.48);
            status["hands"][1]["radio_lost_percent"] = json!(14.2);
            snap.doctor.as_mut().unwrap()["checks"][3] = json!({"name": "sensor_streams", "ok": true,
                "detail": "warning: Sensor 3 shares a USB 2 controller with the headset; move it to another port so its video does not drop frames."});
            snap.doctor.as_mut().unwrap()["checks"][6] = json!({"name": "steamvr_driver", "ok": true,
                "detail": "warning: SteamVR is using another headset than the one cv1touch was aligned with."});
        }
        "overview-daemon-down" => {
            snap.status(
                Err(StatusError {
                    down: true,
                    message: "cv1touchd isn't running".into(),
                }),
                std::time::Instant::now(),
            );
            snap.doctor = Some(json!({"daemon": null, "checks": [
                {"name": "daemon", "ok": false, "detail": "cv1touchd isn't running. Start it with Start daemon, or run setup."},
                {"name": "calibration", "ok": true, "detail": "calibration looks right"}]}));
        }
        "overview-not-setup" => {
            status["mode"] = json!("capture");
            status["alignment"]["registration"] = json!(false);
            status["alignment"]["render"] = json!([false, false]);
            for i in 0..5 {
                status["sensors"][i]["placed"] = json!(0);
                status["sensors"][i]["agreement"] = json!(null);
            }
            snap.doctor = Some(json!({"daemon": {}, "checks": [
                {"name": "daemon", "ok": true, "detail": "cv1touchd is running"},
                {"name": "calibration", "ok": true, "detail": "todo: Run cv1touch setup to calibrate the sensors and align the headset."}]}));
            config = ConfigStats::default();
        }
        _ => unreachable!("{name}"),
    }
    Scene {
        snap,
        config,
        tab,
        report,
        local,
    }
}

const SCENARIOS: [(&str, &[&str]); 8] = [
    (
        "overview-playing",
        &[
            "Ready to play",
            "Restart daemon",
            "Stop daemon",
            "52.0 fps",
            "0.14 px",
            "Tracking",
            "Sensor views",
            "Active",
        ],
    ),
    (
        "overview-report-saved",
        &[
            "Saved cv1touch-report-2026-11-25-232800-UTC.zip (1.4 MB)",
            "Review before sharing — contains calibration and logs.",
            "Open folder",
            "×",
        ],
    ),
    (
        "settings-tab",
        &[
            "Controller angle",
            "Reset to 0°",
            "Start tray icon when I log in",
            "Desktop shortcut and window icon",
            "Align controllers to headset (15 s)…",
            "Calibrate buttons & sticks…",
        ],
    ),
    ("overview-asleep", &["Asleep", "Press a button to wake it."]),
    (
        "overview-warning-inline-fix",
        &[
            "Needs attention",
            "What to do",
            "Align controllers to headset (15 s)",
            "Other headset",
            "Moved",
            "14.2% loss",
        ],
    ),
    ("overview-daemon-down", &["Daemon stopped", "Start daemon"]),
    (
        "overview-not-setup",
        &["Not set up yet", "Not placed", "Not calibrated"],
    ),
    (
        "stats-playing",
        &[
            "wrong_locks",
            "Verified, 0.040 m/s²",
            "Gravity spread",
            "alt setting 2",
            "OK",
            "Landmark shift",
            "Logging to a file",
        ],
    ),
];

fn view<'a>(scene: &'a Scene, icons: &'a Icons, notice: Option<&'a (Level, String)>) -> View<'a> {
    View {
        snap: &scene.snap,
        config: &scene.config,
        daemon: Daemon::from(&scene.snap, None),
        notice,
        icons,
        report: scene.report.as_ref(),
        local: &scene.local,
    }
}

/// The overview's window size: the default for a healthy setup, else the height the window
/// fits itself to; and the content and visible heights there.
fn fitted(
    theme: egui::Theme,
    scene: &Scene,
    notice: Option<&(Level, String)>,
) -> ([f32; 2], (f32, f32)) {
    let measure = |size: [f32; 2]| {
        let mut icons = None;
        render(theme, size, 1.0, |ui, ctx| {
            let icons = icons.get_or_insert_with(|| Icons::load(ctx));
            window::draw(ui, &view(scene, icons, notice), &mut Tab::Overview)
        })
        .2
        .content
    };
    let content = measure(window::SIZE);
    let height = window::fit_height(window::SIZE[1], content);
    if (height - window::SIZE[1]).abs() <= 6.0 {
        return (window::SIZE, content);
    }
    let size = [window::SIZE[0], height];
    (size, measure(size))
}

#[test]
fn screenshots_show_each_state_and_the_overview_fits_without_scrolling_or_waste() {
    let notice = (
        Level::Bad,
        "Restart daemon failed: Service restart failed (exit status: 1)".to_owned(),
    );
    for theme in [egui::Theme::Dark, egui::Theme::Light] {
        for (name, expected) in SCENARIOS {
            let scene = scenario(name);
            let tab = scene.tab;
            // The overview must fit even with an error line in the footer.
            for notice in [None, Some(&notice)] {
                let size = if tab == Tab::Overview {
                    let (size, (content, visible)) = fitted(theme, &scene, notice);
                    println!(
                        "{name}{} notice={}: window {} pt, content {content:.1} pt, visible {visible:.1} pt, spare {:.1} pt",
                        suffix(theme),
                        notice.is_some(),
                        size[1],
                        visible - content
                    );
                    assert!(
                        content <= visible,
                        "{name}: overview needs scrolling: {content} > {visible}"
                    );
                    assert!(
                        visible - content <= 36.0,
                        "{name}: {} pt of empty space below the overview",
                        visible - content
                    );
                    if matches!(name, "overview-playing" | "overview-asleep") && notice.is_none() {
                        assert_eq!(size, window::SIZE, "{name}: the default size does not fit");
                    }
                    size
                } else {
                    window::SIZE
                };
                let mut icons = None;
                let mut draw_tab = tab;
                let (pixels, dims, drawn, labels) = render(theme, size, PPP, |ui, ctx| {
                    let icons = icons.get_or_insert_with(|| Icons::load(ctx));
                    window::draw(ui, &view(&scene, icons, notice), &mut draw_tab)
                });
                for text in expected {
                    assert!(
                        labels.iter().any(|l| l.contains(text)),
                        "{name}: missing {text:?} in {labels:?}"
                    );
                }
                assert!(
                    labels.iter().any(|l| l.contains(window::VERSION)),
                    "{name}: missing version {} in {labels:?}",
                    window::VERSION
                );
                assert!(!labels.iter().any(|l| l.contains("Headset: not reported")));
                let running = !name.contains("daemon-down");
                assert_eq!(
                    labels.iter().any(|l| l == "Start daemon"),
                    !running,
                    "{name}"
                );
                assert_eq!(
                    labels.iter().any(|l| l == "Restart daemon"),
                    running,
                    "{name}"
                );
                assert!(
                    pixels.as_chunks::<4>().0.iter().any(|p| p != &pixels[..4]),
                    "{name}: blank image"
                );
                if notice.is_some() {
                    continue;
                }
                if tab != Tab::Overview {
                    // Stats and Settings scroll; render the whole tab so nothing clips.
                    let (content, visible) = drawn.content;
                    let tall = [size[0], size[1] + content - visible + 1.0];
                    let mut icons = None;
                    let mut draw_tab = tab;
                    let (pixels, dims, _, _) = render(theme, tall, PPP, |ui, ctx| {
                        let icons = icons.get_or_insert_with(|| Icons::load(ctx));
                        window::draw(ui, &view(&scene, icons, None), &mut draw_tab)
                    });
                    save(&format!("{name}{}", suffix(theme)), &pixels, dims);
                } else {
                    save(&format!("{name}{}", suffix(theme)), &pixels, dims);
                }
            }
        }
    }
}

fn suffix(theme: egui::Theme) -> &'static str {
    if theme == egui::Theme::Dark {
        ""
    } else {
        "-light"
    }
}

/// Labels whose boxes overlap: text drawn over other text.
fn overlaps(boxes: &Boxes) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for (i, (a, ra)) in boxes.iter().enumerate() {
        for (b, rb) in &boxes[i + 1..] {
            let (Some(ra), Some(rb)) = (ra, rb) else {
                continue;
            };
            let contains = |p: &egui::accesskit::Rect, q: &egui::accesskit::Rect| {
                p.x0 <= q.x0 + 0.5 && p.y0 <= q.y0 + 0.5 && p.x1 + 0.5 >= q.x1 && p.y1 + 0.5 >= q.y1
            };
            // A widget and the text inside it, or a group around its members.
            if contains(ra, rb) || contains(rb, ra) {
                continue;
            }
            let w = ra.x1.min(rb.x1) - ra.x0.max(rb.x0);
            let h = ra.y1.min(rb.y1) - ra.y0.max(rb.y0);
            if w > 1.0 && h > 1.0 {
                out.push((a.clone(), b.clone()));
            }
        }
    }
    out
}

#[test]
fn no_text_overlaps_in_any_state() {
    for theme in [egui::Theme::Dark, egui::Theme::Light] {
        for (name, _) in SCENARIOS {
            let scene = scenario(name);
            let tab = scene.tab;
            let mut icons = None;
            let mut draw_tab = tab;
            // Tall enough that nothing scrolls under the header or footer.
            let size = if tab != Tab::Overview {
                [window::SIZE[0], 3200.0]
            } else {
                fitted(theme, &scene, None).0
            };
            let (_, _, _, boxes) = render_boxes(
                theme,
                size,
                1.0,
                &mut |ui: &mut egui::Ui, ctx: &egui::Context| {
                    let icons = icons.get_or_insert_with(|| Icons::load(ctx));
                    window::draw(ui, &view(&scene, icons, None), &mut draw_tab)
                },
            );
            let overlaps = overlaps(&boxes);
            assert!(overlaps.is_empty(), "{name} {theme:?}: {overlaps:?}");
            for (label, r) in boxes.iter().filter_map(|(l, r)| Some((l, (*r)?))) {
                assert!(
                    r.x0 >= 0.0 && r.y0 >= 0.0 && r.x1 <= size[0] as f64 && r.y1 <= size[1] as f64,
                    "{name}: {label:?} is cut off at {r:?}"
                );
            }
        }
    }
    // Minimum window width must keep the versioned header subtitle clear of the tabs.
    let scene = scenario("overview-playing");
    let mut icons = None;
    let mut draw_tab = Tab::Overview;
    let min_size = [480.0, 620.0];
    let (_, _, _, boxes) = render_boxes(
        egui::Theme::Dark,
        min_size,
        1.0,
        &mut |ui: &mut egui::Ui, ctx: &egui::Context| {
            let icons = icons.get_or_insert_with(|| Icons::load(ctx));
            window::draw(ui, &view(&scene, icons, None), &mut draw_tab)
        },
    );
    let header_boxes: Boxes = boxes
        .into_iter()
        .filter(|(label, _)| {
            label.contains(window::VERSION)
                || matches!(
                    label.as_str(),
                    "Ready to play" | "Overview" | "Stats" | "Settings"
                )
        })
        .collect();
    assert!(
        header_boxes
            .iter()
            .any(|(l, _)| l.contains(window::VERSION)),
        "min width: missing version in header"
    );
    let header_overlaps = overlaps(&header_boxes);
    assert!(
        header_overlaps.is_empty(),
        "min width header overlaps: {header_overlaps:?}"
    );
    for (label, r) in header_boxes.iter().filter_map(|(l, r)| Some((l, (*r)?))) {
        assert!(
            r.x0 >= 0.0 && r.x1 <= min_size[0] as f64,
            "min width header: {label:?} is cut off at {r:?}"
        );
    }
}

#[test]
fn busy_and_checking_daemons_offer_no_service_buttons() {
    let scene = scenario("overview-playing");
    for (daemon, shown) in [
        (Daemon::Busy(ServiceAction::Restart), "Restarting daemon…"),
        (Daemon::Checking, "Checking daemon…"),
    ] {
        let mut icons = None;
        let (_, _, _, labels) = render(egui::Theme::Dark, window::SIZE, 1.0, |ui, ctx| {
            let icons = icons.get_or_insert_with(|| Icons::load(ctx));
            let view = View {
                daemon,
                ..view(&scene, icons, None)
            };
            window::draw(ui, &view, &mut Tab::Overview)
        });
        assert!(labels.iter().any(|l| l == shown), "{labels:?}");
        for label in ["Start daemon", "Restart daemon", "Stop daemon"] {
            assert!(
                !labels.iter().any(|l| l == label),
                "{daemon:?} offers {label}"
            );
        }
    }
}

type Texture = (usize, usize, Vec<Color32>);

fn apply(textures: &mut HashMap<TextureId, Texture>, delta: &egui::TexturesDelta) {
    for (id, d) in delta
        .set
        .iter()
        .flat_map(|(id, ds)| ds.iter().map(move |d| (id, d)))
    {
        let egui::ImageData::Color(img) = &d.image;
        let [w, h] = img.size;
        match d.pos {
            None => {
                textures.insert(*id, (w, h, img.pixels.clone()));
            }
            Some([x0, y0]) => {
                let t = textures.get_mut(id).unwrap();
                for y in 0..h {
                    for x in 0..w {
                        t.2[(y0 + y) * t.0 + x0 + x] = img.pixels[y * w + x];
                    }
                }
            }
        }
    }
    for id in &delta.free {
        textures.remove(id);
    }
}

/// Bilinear sample of a premultiplied texture, as floats 0..1.
fn sample(t: &Texture, u: f32, v: f32) -> [f32; 4] {
    let (w, h, px) = t;
    let x = (u * *w as f32 - 0.5).clamp(0.0, *w as f32 - 1.0);
    let y = (v * *h as f32 - 0.5).clamp(0.0, *h as f32 - 1.0);
    let (x0, y0) = (x.floor() as usize, y.floor() as usize);
    let (x1, y1) = ((x0 + 1).min(w - 1), (y0 + 1).min(h - 1));
    let (fx, fy) = (x - x0 as f32, y - y0 as f32);
    let c = |x: usize, y: usize| px[y * w + x].to_array().map(|v| v as f32 / 255.0);
    let (a, b, cc, d) = (c(x0, y0), c(x1, y0), c(x0, y1), c(x1, y1));
    std::array::from_fn(|i| {
        (a[i] * (1.0 - fx) + b[i] * fx) * (1.0 - fy) + (cc[i] * (1.0 - fx) + d[i] * fx) * fy
    })
}

fn rasterize(
    prims: &[egui::ClippedPrimitive],
    textures: &HashMap<TextureId, Texture>,
    background: Color32,
    (w, h): (usize, usize),
    ppp: f32,
) -> Vec<u8> {
    let bg = background.to_array().map(|v| v as f32 / 255.0);
    let mut buf = vec![bg; w * h];
    for p in prims {
        let egui::epaint::Primitive::Mesh(mesh) = &p.primitive else {
            continue;
        };
        let tex = &textures[&mesh.texture_id];
        let clip = p.clip_rect;
        let (cx0, cy0) = (
            (clip.min.x * ppp).max(0.0) as usize,
            (clip.min.y * ppp).max(0.0) as usize,
        );
        let (cx1, cy1) = (
            ((clip.max.x * ppp).ceil() as usize).min(w),
            ((clip.max.y * ppp).ceil() as usize).min(h),
        );
        for tri in mesh.indices.as_chunks::<3>().0 {
            let v = [tri[0], tri[1], tri[2]].map(|i| &mesh.vertices[i as usize]);
            let pt = v.map(|v| (v.pos.x * ppp, v.pos.y * ppp));
            let area = (pt[1].0 - pt[0].0) * (pt[2].1 - pt[0].1)
                - (pt[2].0 - pt[0].0) * (pt[1].1 - pt[0].1);
            if area.abs() < 1e-9 {
                continue;
            }
            let minx = pt
                .iter()
                .map(|p| p.0)
                .fold(f32::MAX, f32::min)
                .floor()
                .max(cx0 as f32) as usize;
            let maxx = (pt.iter().map(|p| p.0).fold(f32::MIN, f32::max).ceil() as usize).min(cx1);
            let miny = pt
                .iter()
                .map(|p| p.1)
                .fold(f32::MAX, f32::min)
                .floor()
                .max(cy0 as f32) as usize;
            let maxy = (pt.iter().map(|p| p.1).fold(f32::MIN, f32::max).ceil() as usize).min(cy1);
            for y in miny..maxy {
                for x in minx..maxx {
                    let (px, py) = (x as f32 + 0.5, y as f32 + 0.5);
                    let edge = |a: (f32, f32), b: (f32, f32)| {
                        (b.0 - a.0) * (py - a.1) - (b.1 - a.1) * (px - a.0)
                    };
                    let l = [
                        edge(pt[1], pt[2]) / area,
                        edge(pt[2], pt[0]) / area,
                        edge(pt[0], pt[1]) / area,
                    ];
                    if l.iter().any(|&l| l < 0.0) {
                        continue;
                    }
                    let u = l[0] * v[0].uv.x + l[1] * v[1].uv.x + l[2] * v[2].uv.x;
                    let vv = l[0] * v[0].uv.y + l[1] * v[1].uv.y + l[2] * v[2].uv.y;
                    let col: [f32; 4] = std::array::from_fn(|i| {
                        (0..3)
                            .map(|k| l[k] * v[k].color.to_array()[i] as f32 / 255.0)
                            .sum()
                    });
                    let t = sample(tex, u, vv);
                    let src: [f32; 4] = std::array::from_fn(|i| col[i] * t[i]);
                    let dst = &mut buf[y * w + x];
                    // premultiplied "over", in gamma space as egui's own painters blend
                    *dst = std::array::from_fn(|i| src[i] + dst[i] * (1.0 - src[3]));
                }
            }
        }
    }
    buf.iter()
        .flat_map(|c| c.map(|v| (v.clamp(0.0, 1.0) * 255.0).round() as u8))
        .collect()
}

/// A synthetic infrared view of sensor `i` pooled by `scale`: a dim room, both controllers'
/// LED rings as bright spots (sensor 2 sees only the right one), and on sensor 5 a sunlit
/// window in the corner.
fn camera_scene(i: usize, scale: u32) -> SensorFrame {
    let (raw_w, raw_h) = (1280.0f32, 960.0f32);
    let (w, h) = (1280 / scale, 960 / scale);
    let mut blobs = Vec::new();
    let mut ring = |device: u8, cx: f32, cy: f32, leds: u8, seen: u8| {
        for led in 0..leds {
            let a = led as f32 / leds as f32 * std::f32::consts::TAU + i as f32;
            let (x, y) = (cx + 58.0 * a.cos(), cy + 40.0 * a.sin());
            let matched = led < seen;
            blobs.push(Blob {
                x,
                y,
                w: 6.0,
                h: 5.0,
                led: matched.then_some((device, led * 2 + 1)),
            });
        }
    };
    let shift = i as f32 * 70.0;
    if i != 1 {
        ring(2, 360.0 + shift, 520.0 - shift / 2.0, 11, 9);
    }
    ring(1, 820.0 + shift / 2.0, 470.0 + shift / 3.0, 12, 10);
    for k in 0..3 {
        blobs.push(Blob {
            x: 200.0 + 380.0 * k as f32,
            y: 180.0 + 90.0 * k as f32,
            w: 4.0,
            h: 4.0,
            led: None,
        });
    }
    let mut seed = 0x2545_f491_u32.wrapping_mul(i as u32 + 1);
    let mut pixels = Vec::with_capacity((w * h) as usize);
    for y in 0..h {
        for x in 0..w {
            let (rx, ry) = ((x * scale) as f32, (y * scale) as f32);
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            // Back wall, a lighter floor and a doorway; a little sensor noise.
            let mut v = 4.0 + 10.0 * ry / raw_h + (seed >> 30) as f32;
            if ry > raw_h * 0.68 {
                v += 9.0;
            }
            if (raw_w * 0.12..raw_w * 0.27).contains(&rx) && ry > raw_h * 0.18 && ry < raw_h * 0.68
            {
                v += 7.0;
            }
            if i == 4 {
                let d = ((rx - raw_w).powi(2) + ry.powi(2)).sqrt();
                v = v.max(255.0 * (1.0 - d / 380.0).clamp(0.0, 1.0).powf(0.35));
            }
            for blob in &blobs {
                let d2 = (rx - blob.x).powi(2) + (ry - blob.y).powi(2);
                v = v.max(255.0 * (-d2 / 50.0).exp());
            }
            pixels.push(v.min(255.0) as u8);
        }
    }
    let bright = pixels.iter().filter(|&&p| p >= 180).count();
    let devices = [(2, 9, 11), (1, 10, 12)]
        .into_iter()
        .filter(|&(device, _, _)| !(i == 1 && device == 2))
        .map(|(device_id, matched_blobs, visible_leds)| Device {
            device_id,
            matched_blobs,
            visible_leds,
            pose_valid: true,
            match_flags: 3,
        })
        .collect();
    SensorFrame {
        serial: serial(i),
        frame_ns: 1_000_000 + i as u64,
        age_ms: 14 + i as u64 * 3,
        raw_width: 1280,
        raw_height: 960,
        width: w,
        height: h,
        max_pixel: *pixels.iter().max().unwrap(),
        // The daemon counts raw pixels; a pooled one stands for scale² of them.
        bright_permille: (bright * 1000 / pixels.len()) as u32,
        pixels,
        blobs,
        devices,
        fps: Some(if scale == 2 { 29.6 } else { 15.0 }),
        number: Some(i + 1),
    }
}

fn render_cameras(
    feed: &Feed,
    settings: &mut Settings,
    size: [f32; 2],
) -> (Vec<u8>, (usize, usize), Mode, Vec<String>) {
    let mut textures = Textures::default();
    render(egui::Theme::Dark, size, PPP, |ui, ctx| {
        textures.update(ctx, feed, settings.enhance);
        cameras::draw(ui, feed, &textures, settings)
    })
}

/// Whether any pixel is within 24 of `color` in every channel.
fn shows(pixels: &[u8], color: Color32) -> bool {
    let want = color.to_array();
    pixels
        .as_chunks::<4>()
        .0
        .iter()
        .any(|p| (0..3).all(|c| p[c].abs_diff(want[c]) <= 24))
}

#[test]
fn sensor_view_screenshots_show_the_grid_zoom_and_every_empty_state() {
    let grid = Feed::Live(PreviewSnapshot {
        sensors: (0..5).map(|i| camera_scene(i, 4)).collect(),
    });
    let glare = match &grid {
        Feed::Live(s) => s
            .sensors
            .iter()
            .map(|s| s.bright_permille)
            .collect::<Vec<_>>(),
        _ => unreachable!(),
    };
    assert!(
        glare[4] >= 15 && glare[..4].iter().all(|&g| g < 15),
        "{glare:?}"
    );
    let mut settings = Settings::default();
    let (pixels, dims, mode, labels) = render_cameras(&grid, &mut settings, cameras::SIZE);
    assert_eq!(mode, Mode::All);
    for text in [
        "Sensor views",
        "5 sensors active",
        "Blobs & LEDs",
        "Enhance room",
        "HUD",
        "Sensor 1 · …0001 · 15 fps · 14 ms",
        "Sensor 5 · …0005 · 15 fps · 26 ms",
        "L 9/11  R 10/12",
    ] {
        assert!(
            labels.iter().any(|l| l == text),
            "missing {text:?} in {labels:?}"
        );
    }
    // Sensor 2 sees only the right controller; only sensor 5 has glare.
    assert!(labels.iter().any(|l| l == "R 10/12"), "{labels:?}");
    assert_eq!(
        labels.iter().filter(|l| l.starts_with("IR glare")).count(),
        1
    );
    for color in [cameras::LEFT, cameras::RIGHT, cameras::CANVAS] {
        assert!(shows(&pixels, color), "no {color:?} in the grid");
    }
    save("sensors-grid", &pixels, dims);
    // The overlays and HUD go with their toggles.
    let mut plain = Settings {
        blobs: false,
        hud: false,
        ..Settings::default()
    };
    let (pixels, _, _, labels) = render_cameras(&grid, &mut plain, cameras::SIZE);
    assert!(!labels
        .iter()
        .any(|l| l.starts_with("Sensor 1") || l.starts_with("IR glare")));
    assert!(!shows(&pixels, cameras::LEFT) && !shows(&pixels, cameras::RIGHT));

    // Mode::One returns only the selected sensor; its number must stay 2, not reset to 1.
    let zoom = Feed::Live(PreviewSnapshot {
        sensors: vec![camera_scene(1, 2)],
    });
    let mut settings = Settings {
        zoom: Some(serial(1)),
        ..Settings::default()
    };
    let (pixels, dims, mode, labels) = render_cameras(&zoom, &mut settings, cameras::SIZE);
    assert_eq!(mode, Mode::One(serial(1)));
    for text in [
        "‹ All sensors",
        "Sensor 2 · …0002 · 640×480",
        "Sensor 2 · …0002 · 30 fps · 17 ms",
        "R 10/12",
    ] {
        assert!(
            labels.iter().any(|l| l == text),
            "missing {text:?} in {labels:?}"
        );
    }
    assert!(!labels.iter().any(|l| l.starts_with("Sensor 1")));
    assert!(shows(&pixels, cameras::RIGHT));
    save("sensors-zoom", &pixels, dims);

    let blobs_only = Feed::Live(PreviewSnapshot {
        sensors: vec![SensorFrame {
            width: 0,
            height: 0,
            raw_width: 0,
            raw_height: 0,
            pixels: Vec::new(),
            ..camera_scene(0, 4)
        }],
    });
    for (feed, name, text) in [
        (
            Feed::Failed(PreviewError::DaemonStopped),
            Some("sensors-offline"),
            "cv1touch daemon is not running.",
        ),
        (
            Feed::Failed(PreviewError::UnsupportedDaemon),
            None,
            "The running daemon does not support live sensor views yet.",
        ),
        (
            Feed::Live(PreviewSnapshot::default()),
            None,
            "No sensors detected.",
        ),
        (Feed::Connecting, None, "Connecting to cv1touchd…"),
        (blobs_only, None, "1 sensor active"),
    ] {
        let (pixels, dims, _, labels) =
            render_cameras(&feed, &mut Settings::default(), cameras::SIZE);
        assert!(
            labels.iter().any(|l| l == text),
            "missing {text:?} in {labels:?}"
        );
        if let Some(name) = name {
            save(name, &pixels, dims);
        }
    }
}

#[test]
fn clicking_a_tile_zooms_and_escape_returns_to_the_grid() {
    let feed = Feed::Live(PreviewSnapshot {
        sensors: (0..5).map(|i| camera_scene(i, 4)).collect(),
    });
    let ctx = egui::Context::default();
    window::configure_style(&ctx);
    let mut textures = Textures::default();
    let mut settings = Settings::default();
    let mut frame = |events: Vec<egui::Event>| {
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, cameras::SIZE.into())),
            events,
            ..Default::default()
        };
        let mut mode = None;
        ctx.run_ui(input, |ui| {
            textures.update(ui.ctx(), &feed, settings.enhance);
            mode = Some(cameras::draw(ui, &feed, &textures, &mut settings));
        })
        .textures_delta
        .clear();
        mode.unwrap()
    };
    frame(Vec::new());
    frame(Vec::new());
    // The window's symbols are in the bundled fonts, not drawn as missing glyphs.
    let font = egui::FontId::proportional(14.0);
    assert!(ctx.fonts_mut(|f| f.has_glyphs(&font, "‹…×·–")));
    // Inside the first tile, clear of its labels.
    let pos = Pos2::new(160.0, 230.0);
    let button = |pressed| egui::Event::PointerButton {
        pos,
        button: egui::PointerButton::Primary,
        pressed,
        modifiers: Default::default(),
    };
    frame(vec![egui::Event::PointerMoved(pos)]);
    frame(vec![button(true)]);
    frame(vec![button(false)]);
    assert_eq!(frame(Vec::new()), Mode::One(serial(0)));
    let escape = egui::Event::Key {
        key: egui::Key::Escape,
        physical_key: None,
        pressed: true,
        repeat: false,
        modifiers: Default::default(),
    };
    frame(vec![escape]);
    assert_eq!(frame(Vec::new()), Mode::All);
}

#[test]
fn the_sensor_views_have_no_overlapping_text_down_to_the_least_size() {
    let grid = Feed::Live(PreviewSnapshot {
        sensors: (0..5).map(|i| camera_scene(i, 4)).collect(),
    });
    let zoom = Settings {
        zoom: Some(serial(4)),
        ..Settings::default()
    };
    for size in [cameras::SIZE, cameras::MIN_SIZE] {
        for mut settings in [Settings::default(), zoom.clone()] {
            let mut textures = Textures::default();
            let (_, _, _, boxes) = render_boxes(
                egui::Theme::Dark,
                size,
                1.0,
                &mut |ui: &mut egui::Ui, ctx: &egui::Context| {
                    textures.update(ctx, &grid, settings.enhance);
                    cameras::draw(ui, &grid, &textures, &mut settings)
                },
            );
            let overlaps = overlaps(&boxes);
            assert!(overlaps.is_empty(), "{size:?}: {overlaps:?}");
            for (label, r) in boxes.iter().filter_map(|(l, r)| Some((l, (*r)?))) {
                assert!(
                    r.x0 >= 0.0 && r.y0 >= 0.0 && r.x1 <= size[0] as f64 && r.y1 <= size[1] as f64,
                    "{size:?}: {label:?} is cut off at {r:?}"
                );
            }
            assert!(boxes.iter().any(|(l, _)| l.starts_with("Sensor 5")));
        }
    }
}

/// The setup window's states, from the contract's event lines.
fn setup_scenes() -> Vec<(
    &'static str,
    Guide,
    &'static [&'static str],
    &'static [&'static str],
)> {
    use events::*;
    vec![
        (
            "setup-menu",
            guide(&[STATE, MENU]),
            &[
                "cv1touch setup",
                "1 Sensors",
                "5 Headset",
                "Sensor views",
                "Before you start",
                "a controller is asleep or out of range: press a button on each controller",
                "Sensor 2 moved and is being placed again by itself.",
                "1  Sensor positions",
                "Redo",
                "Run",
                "Keep saved",
                "needs redoing (the sensors moved)",
                "saved earlier, not verified",
                "waiting: press a button on each controller",
                "Continue setup",
                "Recheck",
                "Close",
            ],
            // The headset cannot run now, so its quick alignment is not offered.
            &["Quick (15 s)", "Cancel"],
        ),
        (
            "setup-start-prompt",
            guide(&[STATE, MENU, SENSORS_START, START]),
            &[
                "Start capture",
                "You can also press A on your right controller",
                "Stand where the sensors can see you. Press A on the right controller (or Enter) to start.",
                "Cancel",
            ],
            &["Measure now", "Continue setup"],
        ),
        (
            "setup-capture-progress",
            guide(&[STATE, MENU, SENSORS_START, START, STATUS, CAPTURE]),
            &[
                "Sensor positions",
                "This works out where each of your Rift sensors is. Hold both controllers and walk slowly around the play area for a minute, so that every sensor sees them.\n\nKeep the controllers awake: press a button now and then.",
                "Walk slowly around the play area with both controllers.",
                "36 s left",
                "Cancel",
            ],
            &["Start capture", "Finish early"],
        ),
        (
            "setup-headset-wait",
            guide(&[STATE, MENU, HEADSET_START, HEADSET_PROMPT]),
            &["On your controllers", "Cancel"],
            &["Start capture", "Continue", "Measure now", "Finish early"],
        ),
        (
            "setup-headset-countdown",
            guide(&[STATE, MENU, HEADSET_START, HEADSET_PROMPT, COUNTDOWN]),
            &["7", "Cancel"],
            &["Continue", "Finish early"],
        ),
        (
            "setup-headset-progress",
            guide(&[STATE, MENU, HEADSET_START, COUNTDOWN, HEADSET_CAPTURE]),
            &[
                "Logging",
                "512° of 660°",
                "95° of 660°",
                "22° of 15°",
                "812 poses over 18.7 s · stops by itself within 131 s",
                "Finish early",
                "Cancel",
            ],
            &["Start capture", "Continue"],
        ),
        (
            "setup-buttons-live",
            guide(&[STATE, MENU, BUTTONS_START, BUTTONS_STEP, TRIGGERS, INPUTS]),
            &[
                "Buttons and sticks",
                "2/4 Pull both triggers all the way and hold them there until the double beep.",
                "Measure now",
                "Trigger 91%",
                "Trigger 62%",
                "Left",
                "Right",
                "Cancel",
            ],
            &["Start capture", "You can also press A on your right controller"],
        ),
        (
            "setup-retry",
            guide(&[STATE, MENU, HEADSET_START, STAGE_FAILED, RETRY]),
            &[
                "Headset alignment did not finish.",
                "the left controller slid on the headset (it turned 4°); your previous calibration is unchanged",
                "Try again",
                "Try without slip check",
                "Skip",
                "Cancel",
            ],
            &["Start capture", "Continue"],
        ),
        (
            "setup-solve",
            guide(&[STATE, MENU, SENSORS_START, CAPTURE, SOLVE]),
            &["Working out where the sensors are", "Cancel"],
            &["Start capture"],
        ),
        (
            "setup-done",
            guide(&[STATE, MENU, STAGE_DONE, DONE]),
            &["Setup complete — ready to play!", "Floor: floor levelled: 0.4° correction", "Close"],
            &["Cancel", "Continue setup"],
        ),
        (
            "setup-done-error",
            guide(&[DONE_ERROR]),
            &["another setup guide is running; finish it first", "Close"],
            &["Cancel"],
        ),
    ]
}

/// Live hands from the inputs tap, which take the place of the event's slower ones.
const LIVE: [Hand; 2] = [
    Hand {
        connected: true,
        trigger: 0.62,
        grip: 0.1,
        stick: [-0.2, 0.1],
    },
    Hand {
        connected: true,
        trigger: 0.91,
        grip: 0.05,
        stick: [0.4, 0.55],
    },
];

#[test]
fn setup_screens_follow_the_guide_and_offer_only_what_it_accepts() {
    for (name, guide, shown, hidden) in setup_scenes() {
        let live = (name == "setup-buttons-live").then_some(&LIVE);
        let (pixels, dims, action, labels) =
            render(egui::Theme::Dark, setup_window::SIZE, PPP, |ui, _| {
                setup_window::draw(ui, &guide, live, false)
            });
        assert_eq!(action, None, "{name}");
        for text in shown {
            assert!(
                labels.iter().any(|l| l == text),
                "{name}: missing {text:?} in {labels:?}"
            );
        }
        for text in hidden {
            assert!(!labels.iter().any(|l| l == text), "{name}: offers {text:?}");
        }
        save(name, &pixels, dims);
        // Tall enough that nothing scrolls under the header or footer.
        for size in [setup_window::SIZE, setup_window::MIN_SIZE].map(|[w, _]| [w, 1400.0]) {
            let (_, _, _, boxes) = render_boxes(
                egui::Theme::Dark,
                size,
                1.0,
                &mut |ui: &mut egui::Ui, _: &egui::Context| {
                    setup_window::draw(ui, &guide, live, false)
                },
            );
            let overlaps = overlaps(&boxes);
            assert!(overlaps.is_empty(), "{name} {size:?}: {overlaps:?}");
            for (label, r) in boxes.iter().filter_map(|(l, r)| Some((l, (*r)?))) {
                assert!(
                    r.x0 >= 0.0 && r.x1 <= size[0] as f64,
                    "{name} {size:?}: {label:?} is cut off at {r:?}"
                );
            }
        }
    }
}

#[test]
fn enter_goes_on_only_where_the_prompt_takes_go() {
    use events::*;
    let enter = egui::Event::Key {
        key: egui::Key::Enter,
        physical_key: None,
        pressed: true,
        repeat: false,
        modifiers: Default::default(),
    };
    for (lines, expected) in [
        (
            &[SENSORS_START, START][..],
            Some(setup_window::SetupAction::Send("go".into())),
        ),
        (&[HEADSET_START, HEADSET_PROMPT], None),
        (&[STATE, MENU], None),
    ] {
        let guide = guide(lines);
        let ctx = egui::Context::default();
        let mut action = None;
        for events in [Vec::new(), vec![enter.clone()]] {
            let input = egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, setup_window::SIZE.into())),
                events,
                ..Default::default()
            };
            ctx.run_ui(input, |ui| {
                action = setup_window::draw(ui, &guide, None, false)
            })
            .textures_delta
            .clear();
        }
        assert_eq!(action, expected, "{lines:?}");
    }
}

/// Every window's text, rendered, and every text literal in the program's source: hover
/// texts are not in the accessibility tree until shown.
#[test]
fn no_user_text_says_camera_or_streaming() {
    let mut labels = Vec::new();
    for (name, _) in SCENARIOS {
        let scene = scenario(name);
        let mut icons = None;
        let mut tab = scene.tab;
        labels.extend(
            render(
                egui::Theme::Dark,
                [window::SIZE[0], 1600.0],
                1.0,
                |ui, ctx| {
                    let icons = icons.get_or_insert_with(|| Icons::load(ctx));
                    window::draw(ui, &view(&scene, icons, None), &mut tab)
                },
            )
            .3,
        );
    }
    let grid = Feed::Live(PreviewSnapshot {
        sensors: (0..5).map(|i| camera_scene(i, 4)).collect(),
    });
    for feed in [
        grid,
        Feed::Failed(PreviewError::DaemonStopped),
        Feed::Failed(PreviewError::UnsupportedDaemon),
        Feed::Failed(PreviewError::Other("x".into())),
        Feed::Connecting,
    ] {
        labels.extend(render_cameras(&feed, &mut Settings::default(), cameras::SIZE).3);
    }
    for (_, guide, _, _) in setup_scenes() {
        labels.extend(
            render(egui::Theme::Dark, setup_window::SIZE, 1.0, |ui, _| {
                setup_window::draw(ui, &guide, Some(&LIVE), false)
            })
            .3,
        );
    }
    assert!(labels.len() > 500, "{}", labels.len());
    labels.iter().for_each(|l| assert_plain_words(l));
    assert_plain_words(crate::HELP);
    // Names of the program's own parts and of the daemon's JSON fields, which users never see.
    let internal = [
        "cameras::",
        "mod cameras",
        "camera_pos",
        "camera_queue",
        "\"--cameras\"",
        "--cameras is the older name",
        "recording_status",
        "strip_prefix(\"recording\")",
    ];
    for (file, source) in [
        ("actions.rs", include_str!("actions.rs")),
        ("cameras.rs", include_str!("cameras.rs")),
        ("client.rs", include_str!("client.rs")),
        ("config_stats.rs", include_str!("config_stats.rs")),
        ("desktop.rs", include_str!("desktop.rs")),
        ("main.rs", include_str!("main.rs")),
        ("model.rs", include_str!("model.rs")),
        ("preview_client.rs", include_str!("preview_client.rs")),
        ("settings_tab.rs", include_str!("settings_tab.rs")),
        ("setup_client.rs", include_str!("setup_client.rs")),
        ("setup_window.rs", include_str!("setup_window.rs")),
        ("stats.rs", include_str!("stats.rs")),
        ("tray.rs", include_str!("tray.rs")),
        ("tray_windows.rs", include_str!("tray_windows.rs")),
        ("window.rs", include_str!("window.rs")),
    ] {
        for (n, line) in source.lines().enumerate() {
            let mut line = line.to_lowercase();
            for name in internal {
                line = line.replace(name, "");
            }
            assert!(
                !line.contains("camera")
                    && !line.contains("streaming")
                    && !line.contains("recording")
                    && !line.contains("record now"),
                "{file}:{}: {}",
                n + 1,
                source.lines().nth(n).unwrap().trim()
            );
        }
    }
}
