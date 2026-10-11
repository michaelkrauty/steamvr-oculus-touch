// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! frames_check BASE [--png DIR S...]: for a capture recorded with frames and SteamVR's Touch poses
//! (`eval-capture --free N --frames`), each frame our tracker had the Touch in: the LEDs projected
//! from (a) the tracker's pose at the exposure and (b) the pose SteamVR shows at that instant,
//! measured against the frame's blobs; and how far SteamVR's pose is from ours. With --png, overlays
//! of the frames nearest the given times: image grey, blobs blue, ours green, SteamVR red.
use std::collections::HashMap;

use cv1touch::{
    align,
    calib::{self, capture::Led, file},
    engine, eval,
    recorder::{self, Reader},
    track::{factory, replay, tracker::Outcome},
};
use nalgebra::{Isometry3, Point3, Vector2};

fn pct(v: &mut [f64], q: f64) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_by(f64::total_cmp);
    v[((v.len() - 1) as f64 * q) as usize]
}

/// Visible LEDs of `w` (world from LED model) projected into a sensor.
fn project(
    model: &calib::camera::CameraModel,
    world_from_cam: &Isometry3<f64>,
    leds: &[Led],
    w: &Isometry3<f64>,
) -> Vec<Vector2<f64>> {
    let cam_from_world = world_from_cam.inverse();
    leds.iter()
        .filter_map(|l| {
            let pc = cam_from_world * (w * Point3::from(l.pos));
            let n = cam_from_world.rotation * (w.rotation * l.dir);
            (pc.z > 0.0 && n.dot(&(-pc.coords)) / pc.coords.norm() > 0.2)
                .then(|| model.project(&pc.coords))
                .flatten()
        })
        .filter(|uv| {
            uv.x >= 0.0 && uv.y >= 0.0 && uv.x < model.width as f64 && uv.y < model.height as f64
        })
        .collect()
}

/// Median distance from each projected LED to its nearest blob within 12 px, and how many found one.
fn fit(proj: &[Vector2<f64>], blobs: &[Vector2<f64>]) -> Option<(f64, usize)> {
    let mut d: Vec<f64> = proj
        .iter()
        .filter_map(|p| {
            blobs
                .iter()
                .map(|b| (b - p).norm())
                .fold(None, |m: Option<f64>, x| Some(m.map_or(x, |m| m.min(x))))
                .filter(|&x| x < 12.0)
        })
        .collect();
    let n = d.len();
    (n >= 3).then(|| (pct(&mut d, 0.5), n))
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let base = std::path::PathBuf::from(&a[1]);
    let (png_dir, png_at): (Option<String>, Vec<f64>) = match a.iter().position(|x| x == "--png") {
        Some(i) => (
            Some(a[i + 1].clone()),
            a[i + 2..].iter().map(|s| s.parse().unwrap()).collect(),
        ),
        None => (None, vec![]),
    };
    let tf: eval::TruthFile =
        serde_json::from_str(&std::fs::read_to_string(base.with_extension("truth.json")).unwrap())
            .unwrap();
    let cal = file::load(&file::default_path()).unwrap();
    let reg = align::load_registration(&align::registration_path()).unwrap();
    let x = reg.lighthouse_from_world.iso();
    let render = align::load_render(&align::render_path()).ok();
    let render_of = |h: usize| {
        render
            .as_ref()
            .and_then(|r| r.hands.iter().find(|o| o.hand == ["left", "right"][h]))
            .map_or(Isometry3::identity(), |o| o.model_from_render.iso())
    };
    // SteamVR's Touch poses, in our world and the LED model frame
    let steam: Vec<Vec<eval::TruthSample>> = (0..2)
        .map(|h| {
            let serial = ["CV1T-LEFT", "CV1T-RIGHT"][h];
            let r_inv = render_of(h).inverse();
            tf.others
                .iter()
                .find(|o| o.serial == serial)
                .map_or(vec![], |o| {
                    eval::truth_samples(&o.samples)
                        .into_iter()
                        .map(|s| eval::TruthSample {
                            t_ns: s.t_ns,
                            pose: x.inverse() * s.pose * r_inv,
                        })
                        .collect()
                })
        })
        .collect();
    println!(
        "SteamVR Touch samples: left {}, right {}",
        steam[0].len(),
        steam[1].len()
    );
    let dir = factory::openhmd_config_dir();
    let rec = base.with_extension("rec");
    let mut o = replay::Options {
        trace: true,
        ..Default::default()
    };
    for (h, var) in ["SEED_L", "SEED_R"].iter().enumerate() {
        if let Ok(v) = std::env::var(var) {
            let x: Vec<f64> = v.split(',').map(|t| t.parse().unwrap()).collect();
            o.seed_biases[h] = Some((
                nalgebra::Vector3::new(x[0], x[1], x[2]),
                nalgebra::Vector3::new(x[3], x[4], x[5]),
            ));
        }
    }
    let r = replay::replay(&rec, &cal, &|l| factory::imu_position(&dir, l), &o).unwrap();
    for h in 0..2 {
        eprintln!(
            "stats {}: acquisitions {} wrong_locks {} refused {}",
            ["left", "right"][h],
            r.stats[h].acquisitions,
            r.stats[h].wrong_locks,
            r.stats[h].refused_acquisitions
        );
    }
    let (models, leds) = cv1touch::track::refine::recording_models(&rec).unwrap();
    let mut serials: Vec<String> = cal.sensors.iter().map(|s| s.serial.clone()).collect();
    serials.sort();
    let poses: HashMap<String, Isometry3<f64>> = cal
        .sensors
        .iter()
        .map(|s| (s.serial.clone(), calib::iso_from_xyzw(s.orient_xyzw, s.pos)))
        .collect();
    // blobs and images by (sensor, frame start)
    let mut blobs: HashMap<(String, u64), Vec<Vector2<f64>>> = HashMap::new();
    let mut images: HashMap<(String, u64), (usize, usize, Vec<u8>)> = HashMap::new();
    let mut want: Vec<(usize, f64)> = png_at.iter().copied().enumerate().collect();
    let t0 = tf.truth().first().map_or(0, |s| s.t_ns);
    for rr in Reader::open(&rec).unwrap() {
        match rr.rtype {
            recorder::FRAME_OBS => {
                if let Some((info, b)) = recorder::decode_frame_obs(&rr.payload) {
                    blobs.insert(
                        (engine::cstr_field(&info.serial), info.frame_start_host_ns),
                        b.iter()
                            .map(|b| Vector2::new(b.x as f64, b.y as f64))
                            .collect(),
                    );
                }
            }
            recorder::RAW_FRAME if png_dir.is_some() => {
                if let Some((info, w, h, img)) = recorder::decode_raw_frame(&rr.payload) {
                    let s = (info.frame_start_host_ns as f64 - t0 as f64) * 1e-9;
                    if want.iter().any(|&(_, t)| (s - t).abs() < 0.5) {
                        images.insert(
                            (engine::cstr_field(&info.serial), info.frame_start_host_ns),
                            (w, h, img),
                        );
                    }
                }
            }
            _ => {}
        }
    }
    for h in 0..2 {
        let Some(body) = leds[h].as_deref() else {
            continue;
        };
        let (mut ours, mut theirs, mut d_mm, mut d_deg) = (vec![], vec![], vec![], vec![]);
        let max_s: f64 = std::env::var("MAX_S")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(f64::INFINITY);
        for e in r.events.iter().filter(|e| {
            e.hand == h
                && matches!(e.outcome, Outcome::Updated(_))
                && (e.exposure_host_ns as f64 - t0 as f64) * 1e-9 < max_s
        }) {
            let (Some(tr), Some(serial)) = (e.tracked, serials.get(e.camera)) else {
                continue;
            };
            let (Some(model), Some(wc), Some(b)) = (
                models.get(serial),
                poses.get(serial),
                blobs.get(&(serial.clone(), e.frame_start_host_ns)),
            ) else {
                continue;
            };
            if let Some((m, _)) = fit(&project(model, wc, body, &tr), b) {
                ours.push(m);
            }
            let Some(sv) = eval::interpolate(&steam[h], e.exposure_host_ns) else {
                continue;
            };
            if let Some((m, _)) = fit(&project(model, wc, body, &sv), b) {
                theirs.push(m);
            }
            let d = tr.inverse() * sv;
            if std::env::var("TRACE").is_ok() && d.translation.vector.norm() > 0.05 {
                eprintln!("far {} {:.3} s cam {} {:.0} mm {:.0} deg ours ({:.2},{:.2},{:.2}) steam ({:.2},{:.2},{:.2})", ["L", "R"][h], (e.exposure_host_ns as f64 - t0 as f64) * 1e-9, e.camera, d.translation.vector.norm() * 1e3, d.rotation.angle().to_degrees(), tr.translation.vector.x, tr.translation.vector.y, tr.translation.vector.z, sv.translation.vector.x, sv.translation.vector.y, sv.translation.vector.z);
            }
            d_mm.push(d.translation.vector.norm() * 1e3);
            d_deg.push(d.rotation.angle().to_degrees());
            // overlays
            if let Some(dirp) = &png_dir {
                let s = (e.exposure_host_ns as f64 - t0 as f64) * 1e-9;
                if let Some(k) = want.iter().position(|&(_, t)| s >= t) {
                    if let Some((w, hh, img)) = images.get(&(serial.clone(), e.frame_start_host_ns))
                    {
                        let (ip, sp) =
                            (project(model, wc, body, &tr), project(model, wc, body, &sv));
                        let mut rgb: Vec<u8> = img.iter().flat_map(|&v| [v, v, v]).collect();
                        let mut dot = |p: &Vector2<f64>, c: [u8; 3], r: i32| {
                            for dy in -r..=r {
                                for dx in -r..=r {
                                    if dx * dx + dy * dy > r * r
                                        || dx * dx + dy * dy < (r - 1) * (r - 1)
                                    {
                                        continue;
                                    }
                                    let (px, py) =
                                        (p.x.round() as i32 + dx, p.y.round() as i32 + dy);
                                    if px >= 0
                                        && py >= 0
                                        && (px as usize) < *w
                                        && (py as usize) < *hh
                                    {
                                        let i = (py as usize * w + px as usize) * 3;
                                        rgb[i..i + 3].copy_from_slice(&c);
                                    }
                                }
                            }
                        };
                        for p in b {
                            dot(p, [60, 120, 255], 6);
                        }
                        for p in &ip {
                            dot(p, [0, 255, 0], 4);
                        }
                        for p in &sp {
                            dot(p, [255, 40, 40], 3);
                        }
                        let path =
                            format!("{dirp}/{}_{}_{:.2}s.ppm", ["left", "right"][h], serial, s);
                        let mut buf = format!("P6\n{w} {hh}\n255\n").into_bytes();
                        buf.extend_from_slice(&rgb);
                        std::fs::write(&path, buf).unwrap();
                        let gone = want[k].0;
                        want.retain(|&(kk, _)| kk != gone);
                    }
                }
            }
        }
        println!(
            "Touch {}: {} frames; LED-to-blob median px: ours median {:.2} p90 {:.2} | SteamVR's pose median {:.2} p90 {:.2} | SteamVR vs ours: {:.1} / {:.1} / {:.1} mm (median/p90/p99), {:.2} / {:.2} deg",
            ["left", "right"][h],
            ours.len(),
            pct(&mut ours.clone(), 0.5),
            pct(&mut ours, 0.9),
            pct(&mut theirs.clone(), 0.5),
            pct(&mut theirs, 0.9),
            pct(&mut d_mm.clone(), 0.5),
            pct(&mut d_mm.clone(), 0.9),
            pct(&mut d_mm, 0.99),
            pct(&mut d_deg.clone(), 0.5),
            pct(&mut d_deg, 0.9)
        );
    }
}
