// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! acquire_check BASE [--calibration PATH] [--min-ms MS]: each stretch a controller spent
//! without a position in a recording, from its wake (or a loss) to its next fix, replayed through
//! the tracker as the daemon runs it (remembered biases seeded): how long it lasted, when OpenHMD
//! first offered a strong pose, what the tracker made of the frames meanwhile, and how far the
//! published pose was from where the controller turned out to be. Stretches shorter than MS (100)
//! are only counted.
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use cv1touch::{
    calib::{
        self,
        capture::{Led, MATCH_GOOD, MATCH_STRONG},
        file,
    },
    engine::cstr_field,
    ffi,
    recorder::{self, Reader},
    track::{
        biases, factory,
        pipeline::{device_hand, Options, Pipeline},
        tracker::{Outcome, Stats},
    },
};
use nalgebra::{Isometry3, Vector3};

/// Radio samples further apart than this mark a wake (as the tracker's `max_imu_gap_ns`).
const WAKE_GAP_NS: u64 = 100_000_000;
/// A wrong lock dropped this soon after an acquisition counts against it.
const WRONG_LOCK_WINDOW_NS: u64 = 3_000_000_000;

#[derive(Default)]
struct Stretch {
    hand: usize,
    wake: bool,
    start: u64,
    end: Option<u64>,
    frames: u64,
    outcomes: BTreeMap<&'static str, u64>,
    /// Frames on which OpenHMD offered a strong pose for this controller, and the first one's
    /// host time and camera.
    engine_frames: u64,
    first_engine: Option<(u64, usize)>,
    /// Frames in which OpenHMD's own evaluation found a GOOD but not STRONG pose for this
    /// controller, and the most blobs such a pose matched.
    good_only: u64,
    good_matched: i32,
    first_good: Option<u64>,
    /// Frames with five or more blobs no other controller claims (it may be in view).
    blobby: u64,
    stats_at_start: Stats,
    stats_at_end: Stats,
    /// Published positions (live) while the stretch lasted.
    published: Vec<Vector3<f64>>,
    /// Where the tracker had the controller once it fixed it again.
    fixed_at: Option<Vector3<f64>>,
    wrong_lock_after: bool,
}

fn outcome_name(o: Outcome) -> &'static str {
    match o {
        Outcome::Updated(_) => "updated",
        Outcome::Acquired(_) => "acquired",
        Outcome::NotVisible => "not-visible",
        Outcome::Missed => "missed",
        Outcome::Waiting => "waiting",
        Outcome::TooOld => "too-old",
        _ => "other",
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(base) = args.first() else {
        eprintln!("usage: acquire_check BASE [--calibration PATH]");
        std::process::exit(2);
    };
    let cal_path = args
        .iter()
        .position(|a| a == "--calibration")
        .and_then(|i| args.get(i + 1))
        .map_or_else(file::default_path, PathBuf::from);
    let cal = file::load(&cal_path).expect("calibration");
    let poses: BTreeMap<String, Isometry3<f64>> = cal
        .sensors
        .iter()
        .map(|s| (s.serial.clone(), calib::iso_from_xyzw(s.orient_xyzw, s.pos)))
        .collect();
    let dir = factory::openhmd_config_dir();
    let bias_file = biases::load(&biases::default_path());
    let mut p = Pipeline::new(Options {
        trace: true,
        ..Options::default()
    });
    let path = Path::new(base).with_extension("rec");
    let mut first_t: Option<u64> = None;
    let mut last_sample: [Option<u64>; 2] = [None, None];
    let mut tracking = [false; 2];
    let mut wake_pending = [false; 2];
    let mut open: [Option<Stretch>; 2] = [None, None];
    let mut done: Vec<Stretch> = Vec::new();
    let mut seen_events = 0usize;
    // OpenHMD's poses while the tracker holds the controller, against the tracked pose at the
    // same exposure: (good-only or strong) -> (position errors m, turn errors rad)
    let mut agreement: BTreeMap<&str, (Vec<f64>, Vec<f64>)> = BTreeMap::new();
    for rec in Reader::open(&path).expect("recording") {
        first_t.get_or_insert(rec.host_ns);
        match rec.rtype {
            recorder::SENSOR_INFO => {
                if let Some(s) = recorder::decode_sensor_info(&rec.payload) {
                    if let Some(w) = poses.get(&cstr_field(&s.serial)) {
                        p.sensor(
                            &cstr_field(&s.serial),
                            calib::camera::CameraModel::from_sensor(&s),
                            *w,
                        );
                    }
                }
            }
            recorder::LED_MODEL => {
                let Some((device, model)) = recorder::decode_led_model(&rec.payload) else {
                    continue;
                };
                let Some(hand) = device_hand(device) else {
                    continue;
                };
                if p.has_body(hand) {
                    continue;
                }
                let leds: Vec<Led> = model
                    .iter()
                    .map(|l| Led {
                        pos: calib::vec3(l.pos),
                        dir: calib::vec3(l.dir),
                    })
                    .collect();
                let imu = factory::imu_position(&dir, &leds).expect("factory IMU position");
                p.body(device, leds.clone(), imu);
                if let Some(e) = bias_file.controllers.get(&biases::key(hand, &leds)) {
                    p.seed_biases(hand, e.gyro(), e.accel());
                }
            }
            recorder::RAW_REPORT => {
                if let Some((0, d)) = recorder::decode_raw(&rec.payload) {
                    p.hmd_report(rec.host_ns, d);
                }
            }
            recorder::CONTROLLER_STATE => {
                let Some((h, st)) = recorder::decode_state(&rec.payload) else {
                    continue;
                };
                if h > 1 || st.flags & ffi::CV1T_FLAG_CONNECTED == 0 {
                    continue;
                }
                wake_pending[h] |= last_sample[h]
                    .is_none_or(|t| st.received_host_ns.saturating_sub(t) > WAKE_GAP_NS);
                last_sample[h] = Some(st.received_host_ns);
                // the radio sample's arrival: a stretch is timed from the sample that began it
                let t = st.received_host_ns;
                p.imu(
                    h,
                    st.device_time_ns,
                    t,
                    calib::vec3(st.imu_gyro),
                    calib::vec3(st.imu_accel),
                );
                // the newest output, tracking or not: a repeated sample gives none of its own
                let out = p.latest(h);
                if std::mem::take(&mut wake_pending[h]) && open[h].is_none() {
                    open[h] = Some(Stretch {
                        hand: h,
                        wake: true,
                        start: t,
                        stats_at_start: p.stats(h),
                        ..Stretch::default()
                    });
                }
                if let Some(o) = open[h].as_mut() {
                    o.published.push(Vector3::new(
                        st.pos[0] as f64,
                        st.pos[1] as f64,
                        st.pos[2] as f64,
                    ));
                }
                // no output yet (nothing acquired) is no position either
                let now = out.as_ref().is_some_and(|s| s.output.tracking);
                if now {
                    if let Some(mut o) = open[h].take() {
                        o.end = Some(t);
                        o.stats_at_end = p.stats(h);
                        o.fixed_at = out.map(|s| s.output.world_from_model.translation.vector);
                        done.push(o);
                    }
                } else if tracking[h] && open[h].is_none() {
                    open[h] = Some(Stretch {
                        hand: h,
                        wake: false,
                        start: t,
                        stats_at_start: p.stats(h),
                        ..Stretch::default()
                    });
                }
                tracking[h] = now;
                // a wrong lock dropped soon after a stretch ended counts against its acquisition
                for d in done.iter_mut().filter(|d| d.hand == h) {
                    if d.end
                        .is_some_and(|e| t.saturating_sub(e) <= WRONG_LOCK_WINDOW_NS)
                        && p.stats(h).wrong_locks > d.stats_at_end.wrong_locks
                    {
                        d.wrong_lock_after = true;
                    }
                }
            }
            recorder::FRAME_OBS => {
                if let Some((info, blobs)) = recorder::decode_frame_obs(&rec.payload) {
                    for o in open.iter_mut().flatten() {
                        let device = [calib::DEVICE_LEFT, calib::DEVICE_RIGHT][o.hand];
                        let n = info.n_devices as usize;
                        if let Some(d) = info.devices[..n.min(info.devices.len())]
                            .iter()
                            .find(|d| d.device_id == device && d.pose_valid != 0)
                        {
                            if d.match_flags & MATCH_GOOD != 0 && d.match_flags & MATCH_STRONG == 0
                            {
                                o.good_only += 1;
                                o.good_matched = o.good_matched.max(d.matched_blobs);
                                o.first_good.get_or_insert(info.frame_start_host_ns);
                            }
                        }
                        let free = blobs
                            .iter()
                            .filter(|b| b.led_id < 0 || b.led_id >> 8 == device)
                            .count();
                        o.blobby += u64::from(free >= 5);
                    }
                    p.frame(&info, &blobs);
                    let serial = cstr_field(&info.serial);
                    let n = (info.n_devices as usize).min(info.devices.len());
                    for e in p.events[seen_events..]
                        .iter()
                        .filter(|e| e.frame_start_host_ns == info.frame_start_host_ns)
                    {
                        let (Some(tracked), Some(world_from_cam)) = (e.tracked, poses.get(&serial))
                        else {
                            continue;
                        };
                        let device = [calib::DEVICE_LEFT, calib::DEVICE_RIGHT][e.hand];
                        for d in info.devices[..n].iter().filter(|d| {
                            d.device_id == device
                                && d.pose_valid != 0
                                && d.match_flags & MATCH_GOOD != 0
                        }) {
                            let pose =
                                world_from_cam * calib::iso_from_xyzw(d.cam_orient, d.cam_pos);
                            let kind = if d.match_flags & MATCH_STRONG != 0 {
                                "strong"
                            } else {
                                "good-only"
                            };
                            let a = agreement.entry(kind).or_default();
                            a.0.push((pose.translation.vector - tracked.translation.vector).norm());
                            a.1.push(pose.rotation.angle_to(&tracked.rotation));
                        }
                    }
                }
                for e in &p.events[seen_events..] {
                    let Some(o) = open[e.hand].as_mut() else {
                        continue;
                    };
                    o.frames += 1;
                    *o.outcomes.entry(outcome_name(e.outcome)).or_default() += 1;
                    if e.engine.is_some_and(|p| p.strong) {
                        o.engine_frames += 1;
                        o.first_engine.get_or_insert((e.exposure_host_ns, e.camera));
                    }
                }
                seen_events = p.events.len();
            }
            _ => {}
        }
    }
    for (kind, (pos, rot)) in agreement.iter_mut() {
        pos.sort_by(f64::total_cmp);
        let near = pos
            .iter()
            .zip(rot.iter())
            .filter(|(p, r)| **p < 0.03 && **r < 10f64.to_radians())
            .count();
        let far = pos.iter().filter(|p| **p > 0.10).count();
        println!(
            "OpenHMD {kind} poses while tracked: {}, within 3 cm and 10 deg {:.1}%, over 10 cm {:.1}%, position median {:.1} mm",
            pos.len(),
            100.0 * near as f64 / pos.len().max(1) as f64,
            100.0 * far as f64 / pos.len().max(1) as f64,
            pos.get(pos.len() / 2).copied().unwrap_or(f64::NAN) * 1e3
        );
    }
    let t0 = first_t.unwrap_or(0);
    let ms = |a: u64, b: u64| b.saturating_sub(a) as f64 * 1e-6;
    for hand in 0..2 {
        let st = p.stats(hand);
        if st.frames > 0 {
            println!(
                "{}: {} acquisitions ({} from good poses, {} searched, {} unconfirmed), {} wrong locks dropped, {} updates",
                ["left", "right"][hand],
                st.acquisitions,
                st.good_acquisitions,
                st.searched_acquisitions,
                st.unconfirmed_acquisitions,
                st.wrong_locks,
                st.updates
            );
        }
    }
    let mut by_kind: BTreeMap<&str, Vec<f64>> = BTreeMap::new();
    for s in &done {
        let kind = if s.wake { "wake" } else { "loss" };
        by_kind
            .entry(kind)
            .or_default()
            .push(ms(s.start, s.end.unwrap_or(s.start)));
    }
    for (kind, mut v) in by_kind {
        v.sort_by(f64::total_cmp);
        let at = |q: f64| v[((v.len() - 1) as f64 * q) as usize];
        println!(
            "{kind}: {} stretches, fixed after median {:.0} ms, p90 {:.0}, max {:.0}",
            v.len(),
            at(0.5),
            at(0.9),
            at(1.0)
        );
    }
    let min_ms: f64 = args
        .iter()
        .position(|a| a == "--min-ms")
        .and_then(|i| args.get(i + 1))
        .map_or(100.0, |v| v.parse().expect("--min-ms"));
    for s in done
        .iter()
        .filter(|s| ms(s.start, s.end.unwrap_or(s.start)) >= min_ms)
        .chain(open.iter().flatten())
    {
        let (a, b) = (&s.stats_at_start, &s.stats_at_end);
        let float = s.fixed_at.map(|f| {
            s.published
                .iter()
                .map(|q| (q - f).norm())
                .fold(0.0, f64::max)
        });
        println!(
            "{:<5} {} at {:7.2} s: {}; frames {} {:?}, {} with 5+ free blobs; first strong OpenHMD pose {}; {} such frames; {} good-only (up to {} blobs, first after {}); refused {}, ambiguous {}, failed {}, relocks {}, alias turns {}, good {}, searched {}, unconfirmed {}{}; published pose up to {} from the fix",
            ["left", "right"][s.hand],
            if s.wake { "wake" } else { "loss" },
            ms(t0, s.start) * 1e-3,
            s.end.map_or("never fixed".into(), |e| format!("fixed after {:.0} ms", ms(s.start, e))),
            s.frames,
            s.outcomes,
            s.blobby,
            s.first_engine.map_or("none".into(), |(t, c)| format!("after {:.0} ms (camera {c})", ms(s.start, t))),
            s.engine_frames,
            s.good_only,
            s.good_matched,
            s.first_good.map_or("-".into(), |t| format!("{:.0} ms", ms(s.start, t))),
            b.refused_acquisitions.saturating_sub(a.refused_acquisitions),
            b.ambiguous_acquisitions.saturating_sub(a.ambiguous_acquisitions),
            b.failed_acquisitions.saturating_sub(a.failed_acquisitions),
            b.relocks.saturating_sub(a.relocks),
            b.alias_turns.saturating_sub(a.alias_turns),
            b.good_acquisitions.saturating_sub(a.good_acquisitions),
            b.searched_acquisitions.saturating_sub(a.searched_acquisitions),
            b.unconfirmed_acquisitions.saturating_sub(a.unconfirmed_acquisitions),
            if s.wrong_lock_after { "; WRONG LOCK dropped within 3 s" } else { "" },
            float.map_or("-".into(), |m| format!("{:.0} mm", m * 1e3)),
        );
    }
}
