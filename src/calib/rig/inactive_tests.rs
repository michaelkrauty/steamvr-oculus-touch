// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krautkramer

//! Offline synthetic replay coverage. CLI admission/mean and frame extraction are
//! source-matched transcriptions; replay, holds, association and calibration are production.
//! Factory IMU position is injected at the model origin; no hardware/cache is used.
use super::*;
use crate::calib::synth::{self, Rng};

/// Cleanup is armed only after exclusive directory creation succeeds. The file
/// may be created before Recorder::start reports a header/write failure, so own
/// the private container rather than inferring file ownership from that result.
struct RecordingDir(std::path::PathBuf);

impl RecordingDir {
    fn create(path: std::path::PathBuf) -> std::io::Result<Self> {
        let builder = std::fs::DirBuilder::new();
        #[cfg(unix)]
        let builder = {
            use std::os::unix::fs::DirBuilderExt;
            let mut builder = builder;
            builder.mode(0o700);
            builder
        };
        builder.create(&path)?;
        Ok(Self(path))
    }

    fn temporary(seed: u64) -> Self {
        static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let id = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        Self::create(std::env::temp_dir().join(format!(
            "cv1touch-rig-inactive-{}-{seed}-{id}",
            std::process::id()
        )))
        .unwrap()
    }
}

impl Drop for RecordingDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

#[test]
fn recording_cleanup_requires_exclusive_ownership() {
    let directory = RecordingDir::temporary(0);
    let path = directory.0.clone();
    let sentinel = path.join("synthetic.rec");
    std::fs::write(&sentinel, b"owned sentinel").unwrap();
    assert!(RecordingDir::create(path.clone()).is_err());
    assert_eq!(std::fs::read(&sentinel).unwrap(), b"owned sentinel");

    // A recording open collision cannot arm an independent file-removal guard.
    let rec = crate::recorder::Recorder::new(1024);
    assert!(rec.start(&sentinel, 0).is_err());
    assert_eq!(std::fs::read(&sentinel).unwrap(), b"owned sentinel");
    drop(rec);

    // Once the directory is ours, unwinding after any partial file creation must
    // still remove it. No failed directory creation above owns this cleanup.
    assert!(std::panic::catch_unwind(move || {
        let _directory = directory;
        panic!("synthetic failure after exclusive ownership");
    })
    .is_err());
    assert!(!path.exists());
}

fn replayed_problem(seed: u64) -> (Problem, State) {
    let mut rng = Rng::new(seed);
    let target = Vector3::new(0.0, 1.0, 0.0);
    let world_from_cam: Vec<Isometry3<f64>> = [
        (Vector3::new(-2.0, 2.3, 1.8), true),
        (Vector3::new(2.0, 2.3, 1.8), false),
        (Vector3::new(0.3, 2.4, -2.2), true),
    ]
    .iter()
    .map(|&(pos, flip)| synth::look_at(pos, target, flip))
    .collect();
    let x_true = Isometry3::from_parts(
        Translation3::new(0.1, -0.6, -2.3),
        UnitQuaternion::from_scaled_axis(Vector3::new(0.002, 1.2, -0.003)),
    );
    let y_true = [
        Isometry3::from_parts(
            Translation3::new(0.16, -0.05, 0.08),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.3, 0.1, 0.0)),
        ),
        Isometry3::from_parts(
            Translation3::new(-0.18, -0.04, 0.09),
            UnitQuaternion::from_scaled_axis(Vector3::new(-0.2, 2.9, 0.1)),
        ),
    ];
    let leds = synth::touch_leds();
    let cams = vec![synth::camera_model(); 3];
    let truth = State {
        cam_from_world: world_from_cam.iter().map(|w| w.inverse()).collect(),
        lighthouse_from_world: x_true,
        ref_from_touch: y_true.to_vec(),
    };
    let mut holds_v = Vec::new();
    for k in 0..30 {
        let world_from_ref = Isometry3::from_parts(
            Translation3::new(
                rng.range(-0.5, 0.5),
                rng.range(0.7, 1.5),
                rng.range(-0.5, 0.5),
            ),
            UnitQuaternion::from_scaled_axis(Vector3::new(
                rng.range(-1.2, 1.2),
                rng.range(-3.1, 3.1),
                rng.range(-1.2, 1.2),
            )),
        );
        let hold0 = Hold {
            lighthouse_from_ref: x_true * world_from_ref,
            frames: vec![],
        };
        let frames = (0..3)
            .map(|c| {
                (0..10)
                    .map(|_| {
                        let mut blobs = Vec::new();
                        for t in 0..2 {
                            if t == 1 && k % 2 == 0 {
                                continue;
                            }
                            let w = truth.world_from_touch(&hold0, t);
                            for l in &leds {
                                let pw = w * Point3::from(l.pos);
                                let cam_pos = world_from_cam[c].translation.vector;
                                if (w.rotation * l.dir).dot(&(cam_pos - pw.coords).normalize())
                                    < 0.3
                                {
                                    continue;
                                }
                                if let Some(uv) =
                                    cams[c].project(&(truth.cam_from_world[c] * pw).coords)
                                {
                                    blobs.push(uv + Vector2::new(rng.gauss(), rng.gauss()) * 0.3);
                                }
                            }
                        }
                        blobs
                    })
                    .collect()
            })
            .collect();
        holds_v.push(Hold {
            lighthouse_from_ref: hold0.lighthouse_from_ref,
            frames,
        });
    }
    let mut p = Problem {
        cameras: cams,
        fixed: 0,
        touches: vec![leds.clone(), leds],
        holds: holds_v,
    };
    let bump = |a: &Isometry3<f64>, mm: f64, deg: f64| {
        exp6(&[
            deg.to_radians(),
            -deg.to_radians() * 0.5,
            deg.to_radians() * 0.3,
            mm * 1e-3,
            -mm * 0.7e-3,
            mm * 0.4e-3,
        ]) * a
    };
    // the old calibration is right except that sensor 2 has since drifted 2 mm and 0.05
    // degrees (a pixel: larger moves are `refine`'s, which aligns the whole pattern first);
    // the registration and the rig offsets are only roughly known
    let mut initial = State {
        cam_from_world: vec![
            truth.cam_from_world[0],
            truth.cam_from_world[1],
            bump(&truth.cam_from_world[2], 2.0, 0.05),
        ],
        lighthouse_from_world: bump(&x_true, 2.0, 0.1),
        ref_from_touch: vec![
            y_true[0] * exp6(&[0.005, 0.0, 0.004, 0.003, -0.002, 0.004]),
            y_true[1] * exp6(&[-0.004, 0.003, 0.0, -0.003, 0.004, 0.002]),
        ],
    };

    use crate::calib::file::{CalibrationFile, SensorEntry};
    use crate::{
        recorder::{self, as_bytes, slice_bytes},
        track,
    };
    let directory = RecordingDir::temporary(seed);
    let path = directory.0.join("synthetic.rec");
    let mut cal = CalibrationFile::example();
    cal.sensors = initial
        .cam_from_world
        .iter()
        .enumerate()
        .map(|(c, cfw)| {
            let (orient_xyzw, pos) = crate::calib::iso_to_xyzw(&cfw.inverse());
            SensorEntry {
                serial: format!("SYNTH{c}"),
                orient_xyzw,
                pos,
                rms_px: 0.0,
                observations: 100,
            }
        })
        .collect();
    // Match CLI exactly: replay and optimization use the same serialized calibration.
    // Round trip float sensor fields before any replay/association, as the CLI does.
    initial.cam_from_world = cal
        .sensors
        .iter()
        .map(|e| crate::calib::iso_from_xyzw(e.orient_xyzw, e.pos).inverse())
        .collect();
    cal.stats.reference = "SYNTH0".into();
    for (c, e) in cal.sensors.iter().enumerate() {
        assert_eq!(
            initial.cam_from_world[c],
            crate::calib::iso_from_xyzw(e.orient_xyzw, e.pos).inverse()
        );
    }
    println!("CONSISTENCY replay_and_initial_cameras_identical=true");
    let serial = |c: usize| {
        let mut a = [0 as std::os::raw::c_char; 32];
        for (d, b) in a.iter_mut().zip(format!("SYNTH{c}").bytes()) {
            *d = b as _;
        }
        a
    };
    // No external caches, factory data or live native engine: the synthetic body's IMU
    // is at its model origin. All acquisition labels/poses below match generated geometry.
    let rec = recorder::Recorder::new(32 << 20);
    const H0: u64 = 5_000_000_000_000;
    rec.start(&path, H0).unwrap();
    for c in 0..3 {
        let m = p.cameras[c];
        let sensor = recorder::SensorInfo {
            serial: serial(c),
            width: m.width as i32,
            height: m.height as i32,
            fx: m.fx as f32,
            fy: m.fy as f32,
            cx: m.cx as f32,
            cy: m.cy as f32,
            k: m.k.map(|v| v as f32),
            ..Default::default()
        };
        rec.write(recorder::SENSOR_INFO, H0, &[as_bytes(&sensor)]);
    }
    for t in 0..2 {
        let model: Vec<_> = p.touches[t]
            .iter()
            .map(|l| recorder::LedEntry {
                pos: l.pos.map(|v| v as f32).into(),
                dir: l.dir.map(|v| v as f32).into(),
            })
            .collect();
        rec.write(
            recorder::LED_MODEL,
            H0,
            &[
                &recorder::led_model_prefix([2, 1][t], model.len()),
                slice_bytes(&model),
            ],
        );
    }
    let mut samples = Vec::new();
    for k in 0..30 {
        let h = &p.holds[k];
        // Two second stationary holds, with a one second unrecorded transition/radio gap.
        // No fabricated published poses are written. replay must acquire and publish.
        for tick in 0..1000u64 {
            let at = H0 + k as u64 * 3_000_000_000 + tick * 2_000_000;
            let us = ((at - H0) / 1000 + 77_000) as u32;
            let mut raw = [0u8; 64];
            raw[0] = 11;
            raw[8..12].copy_from_slice(&us.to_le_bytes());
            rec.write(recorder::RAW_REPORT, at, &[&0u32.to_le_bytes(), &raw]);
            samples.push(TruthSample {
                t_ns: at,
                pose: h.lighthouse_from_ref,
            });
            for t in 0..2 {
                let w = truth.world_from_touch(h, t);
                let accel = w.rotation.inverse() * Vector3::new(0.0, 9.80665, 0.0);
                let st = crate::shm::ControllerState {
                    device_time_ns: at - 4_000_000_000_000,
                    received_host_ns: at,
                    imu_accel: accel.map(|v| v as f32).into(),
                    ..Default::default()
                };
                rec.write(
                    recorder::CONTROLLER_STATE,
                    at,
                    &[
                        &(t as u32).to_le_bytes(),
                        &0u32.to_le_bytes(),
                        as_bytes(&st),
                    ],
                );
            }
            if tick % 10 != 0 {
                continue;
            }
            for c in 0..3 {
                let mut info = recorder::FrameInfo {
                    serial: serial(c),
                    frame_start_host_ns: at,
                    exposure_valid: 1,
                    exposure_hmd_ts: us,
                    exposure_count: tick as u32,
                    n_devices: 2,
                    ..Default::default()
                };
                let mut blobs = Vec::new();
                for t in 0..2 {
                    if t == 1 && k % 2 == 0 {
                        continue;
                    } // visibility at image generation, NOT sighting deletion
                    let w = truth.world_from_touch(h, t);
                    let cp = truth.cam_from_world[c] * w;
                    let (o, q) = crate::calib::iso_to_xyzw(&cp);
                    info.devices[t].device_id = [2, 1][t];
                    info.devices[t].pose_valid = 1;
                    info.devices[t].match_flags =
                        crate::calib::capture::MATCH_GOOD | crate::calib::capture::MATCH_STRONG;
                    info.devices[t].cam_orient = o;
                    info.devices[t].cam_pos = q;
                    for (i, l) in p.touches[t].iter().enumerate() {
                        let pc = cp * Point3::from(l.pos);
                        if (cp.rotation * l.dir).dot(&(-pc.coords).normalize()) < 0.3 {
                            continue;
                        }
                        let Some(uv) = p.cameras[c].project(&pc.coords) else {
                            continue;
                        };
                        blobs.push(recorder::Blob {
                            x: (uv.x + rng.gauss() * 0.2) as f32,
                            y: (uv.y + rng.gauss() * 0.2) as f32,
                            width: 3,
                            height: 3,
                            area: 9,
                            led_id: ([2, 1][t] << 8) | i as i32,
                        });
                    }
                }
                rec.write(
                    recorder::FRAME_OBS,
                    at,
                    &[as_bytes(&info), slice_bytes(&blobs)],
                );
            }
        }
    }
    let rs = rec.stop();
    assert_eq!(rs.dropped, 0);
    assert_eq!(rs.write_errors, 0);
    let replay = track::replay::replay(
        &path,
        &cal,
        &|_| Some(Vector3::zeros()),
        &Default::default(),
    )
    .unwrap();
    let spans = holds(&samples, 1_500_000_000);
    assert_eq!(spans.len(), 30);
    // Exact mean_pose admission rule from the CLI (>=20 replay outputs in a hold,
    // componentwise median position and chordal orientation mean; >=8 such holds).
    for t in 0..2 {
        let mut ys = Vec::new();
        for &(a, b, l) in &spans {
            let set: Vec<_> = replay.hands[t]
                .iter()
                .filter(|p| p.t_ns >= a && p.t_ns <= b)
                .collect();
            if set.len() < 20 {
                continue;
            }
            let med = |j: usize| {
                let mut v: Vec<_> = set.iter().map(|p| p.pose.translation.vector[j]).collect();
                v.sort_by(f64::total_cmp);
                v[v.len() / 2]
            };
            let r = crate::calib::motion::chordal_mean(
                &set.iter().map(|p| p.pose.rotation).collect::<Vec<_>>(),
            );
            let pose = Isometry3::from_parts(Translation3::new(med(0), med(1), med(2)), r);
            ys.push(l.inverse() * initial.lighthouse_from_world * pose);
        }
        println!(
            "ADMISSION touch={t} replay_outputs={} hold_means={}",
            replay.hands[t].len(),
            ys.len()
        );
        assert!(
            ys.len() >= 8,
            "touch {t} fails real replay admission: {:?}",
            replay.stats[t]
        );
        initial.ref_from_touch[t] = Isometry3::from_parts(
            Translation3::from(
                ys.iter()
                    .map(|y| y.translation.vector)
                    .sum::<Vector3<f64>>()
                    / ys.len() as f64,
            ),
            crate::calib::motion::chordal_mean(&ys.iter().map(|y| y.rotation).collect::<Vec<_>>()),
        );
    }
    for (t, y) in y_true.iter().enumerate() {
        let d = y.inverse() * initial.ref_from_touch[t];
        println!(
            "OFFSET t={t} error_mm={} error_deg={}",
            d.translation.vector.norm() * 1000.0,
            d.rotation.angle().to_degrees()
        );
    }
    // CLI frame extraction: only frames inside trimmed truth holds enter Problem.
    // Use the actual averaged poses returned by rig::holds, not the original
    // synthetic hold poses used only to generate the truth recording.
    p.holds = spans
        .iter()
        .map(|&(_, _, l)| Hold {
            lighthouse_from_ref: l,
            frames: vec![Vec::new(); 3],
        })
        .collect();
    for (h, span) in p.holds.iter().zip(&spans) {
        assert_eq!(h.lighthouse_from_ref, span.2);
    }
    println!(
        "CONSISTENCY problem_poses_from_rig_holds=true count={}",
        p.holds.len()
    );
    for r in recorder::Reader::open(&path).unwrap() {
        if r.rtype != recorder::FRAME_OBS {
            continue;
        }
        let (info, bl) = recorder::decode_frame_obs(&r.payload).unwrap();
        if let Some(k) = spans
            .iter()
            .position(|h| info.frame_start_host_ns >= h.0 && info.frame_start_host_ns <= h.1)
        {
            let c = (0..3).find(|&c| serial(c) == info.serial).unwrap();
            p.holds[k].frames[c].push(
                bl.iter()
                    .map(|b| Vector2::new(b.x as f64, b.y as f64))
                    .collect(),
            );
        }
    }
    let counts = |obs: &[Sighting]| -> (Vec<usize>, Vec<usize>) {
        (
            (0..2)
                .map(|t| obs.iter().filter(|o| o.touch == t).count())
                .collect(),
            (0..3)
                .map(|c| obs.iter().filter(|o| o.camera == c).count())
                .collect(),
        )
    };
    for (gate, margin) in [(4.0, 1.5), (2.5, 1.0), (1.5, 1.0), (1.0, 0.8)] {
        let all = associate(&p, &initial, gate, margin);
        let even: Vec<_> = all.iter().filter(|o| o.hold % 2 == 0).copied().collect();
        let (tc, cc) = counts(&even);
        println!(
            "ASSOC gate={gate} all={:?} even_touches={tc:?} even_cameras={cc:?}",
            counts(&all)
        );
        assert!(tc[0] > 0 && tc[1] == 0 && cc.iter().all(|&n| n > 0));
    }
    (p, initial)
}

fn rms(p: &Problem, s: &State, obs: &[Sighting]) -> f64 {
    let r = residuals(p, s, obs);
    (r.iter().map(|x| x * x).sum::<f64>() / r.len() as f64).sqrt()
}

#[test]
fn even_validation_preserves_inactive_touch() {
    check_even_validation(27);
}

#[test]
fn even_validation_fresh_seed() {
    check_even_validation(29);
}

fn check_even_validation(seed: u64) {
    let (p, initial) = replayed_problem(seed);
    let obs: Vec<_> = associate(&p, &initial, 4.0, 1.5)
        .into_iter()
        .filter(|o| o.hold % 2 == 0)
        .collect();
    let prior = Prior::around(&initial);
    let fitted = solve(&p, &initial, &obs, 0.5, 30, Some(&prior));
    let mut reduced_p = p.clone();
    reduced_p.touches.pop();
    let mut reduced_initial = initial.clone();
    reduced_initial.ref_from_touch.pop();
    assert_eq!(
        residuals(&p, &initial, &obs),
        residuals(&reduced_p, &reduced_initial, &obs)
    );
    let reduced = solve(&reduced_p, &reduced_initial, &obs, 0.5, 30, Some(&prior));
    println!(
        "seed={seed} initial_rms={} full_rms={} reduced_rms={}",
        rms(&p, &initial, &obs),
        rms(&p, &fitted, &obs),
        rms(&reduced_p, &reduced, &obs)
    );
    let half = calibrate(&p, &initial, &|k| k % 2 == 0);
    println!(
        "EVEN seed={seed} sightings={} initial_rms={} fitted_rms={}",
        half.sightings.len(),
        rms(&p, &initial, &obs),
        rms(&p, &half.state, &obs)
    );
    assert_eq!(
        half.state.cam_from_world[p.fixed],
        initial.cam_from_world[p.fixed]
    );
    assert_eq!(
        fitted.cam_from_world[p.fixed],
        initial.cam_from_world[p.fixed]
    );
    assert_eq!(half.state.ref_from_touch[1], initial.ref_from_touch[1]);
    assert_ne!(
        half.state.lighthouse_from_world,
        initial.lighthouse_from_world
    );
    assert_ne!(half.state.cam_from_world, initial.cam_from_world);
    assert_ne!(half.state.ref_from_touch[0], initial.ref_from_touch[0]);
    assert!(half
        .sightings
        .iter()
        .all(|o| o.hold % 2 == 0 && o.touch == 0));
    assert!(rms(&p, &half.state, &obs) < rms(&p, &initial, &obs) * 0.5);

    assert!(rms(&p, &fitted, &obs) < rms(&p, &initial, &obs) * 0.5);
    assert_eq!(fitted.ref_from_touch[1], initial.ref_from_touch[1]);
    assert_observed_equivalent(&fitted, &reduced, 0);

    // Touch indexing must not matter: swap models/state and remap only the identity,
    // keeping observation order, pixels, cameras, holds and priors identical.
    let mut swapped_p = p.clone();
    swapped_p.touches.swap(0, 1);
    let mut swapped_initial = initial.clone();
    swapped_initial.ref_from_touch.swap(0, 1);
    let swapped_obs: Vec<_> = obs
        .iter()
        .map(|o| Sighting {
            touch: 1 - o.touch,
            ..*o
        })
        .collect();
    let swapped = solve(
        &swapped_p,
        &swapped_initial,
        &swapped_obs,
        0.5,
        30,
        Some(&prior),
    );
    assert_eq!(swapped.ref_from_touch[0], swapped_initial.ref_from_touch[0]);
    assert_observed_equivalent(&swapped, &reduced, 1);
    let swapped_half = calibrate(&swapped_p, &swapped_initial, &|k| k % 2 == 0);
    assert_eq!(
        swapped_half.state.ref_from_touch[0],
        swapped_initial.ref_from_touch[0]
    );
    assert!(
        rms(&swapped_p, &swapped_half.state, &swapped_obs)
            < rms(&swapped_p, &swapped_initial, &swapped_obs) * 0.5
    );

    // Camera blocks are not inactive Touch blocks. Keep the ordinary sensor prior,
    // including for a camera with no sightings, even if its prior pose differs.
    let camera_obs: Vec<_> = obs.iter().filter(|o| o.camera != 2).copied().collect();
    let mut camera_prior = prior.clone();
    camera_prior.cam_from_world[2] =
        exp6(&[0.0, 0.0, 0.0, 0.001, 0.0, 0.0]) * initial.cam_from_world[2];
    let camera_fit = solve(&p, &initial, &camera_obs, 0.5, 30, Some(&camera_prior));
    assert!(rms(&p, &camera_fit, &camera_obs) < rms(&p, &initial, &camera_obs) * 0.5);
    assert!(
        (camera_fit.cam_from_world[2].translation.vector
            - camera_prior.cam_from_world[2].translation.vector)
            .norm()
            < 1e-9
    );
    assert_eq!(camera_fit.ref_from_touch[1], initial.ref_from_touch[1]);

    assert_eq!(solve(&p, &initial, &[], 0.5, 30, Some(&prior)), initial);
    assert_eq!(solve(&p, &initial, &[], 0.5, 30, None), initial);
    // Without that prior an unseen camera still leaves an unresolved null block.
    // This narrow fix must not silently regularize it or change rank policy.
    assert_eq!(solve(&p, &initial, &camera_obs, 0.5, 30, None), initial);

    let all_obs = associate(&p, &initial, 4.0, 1.5);
    assert!(all_obs.iter().any(|o| o.touch == 1));
    let all_fit = solve(&p, &initial, &all_obs, 0.5, 30, Some(&prior));
    assert_ne!(all_fit, initial);
    // An appended inactive block cannot alter any all-active numerical result.
    let mut appended_p = p.clone();
    appended_p.touches.push(p.touches[0].clone());
    let mut appended_initial = initial.clone();
    appended_initial
        .ref_from_touch
        .push(initial.ref_from_touch[0]);
    let appended = solve(
        &appended_p,
        &appended_initial,
        &all_obs,
        0.5,
        30,
        Some(&prior),
    );
    assert_eq!(
        appended.ref_from_touch[2],
        appended_initial.ref_from_touch[2]
    );
    for (a, b) in appended.cam_from_world.iter().zip(&all_fit.cam_from_world) {
        assert_pose_equivalent(a, b);
    }
    assert_pose_equivalent(
        &appended.lighthouse_from_world,
        &all_fit.lighthouse_from_world,
    );
    for (a, b) in appended.ref_from_touch.iter().zip(&all_fit.ref_from_touch) {
        assert_pose_equivalent(a, b);
    }
    let full = calibrate(&p, &initial, &|_| true);
    assert_ne!(
        full.state, initial,
        "all-hold fit already optimizes this scene"
    );
}

fn assert_observed_equivalent(full: &State, reduced: &State, active: usize) {
    // Cholesky on an identity-pinned zero block is equivalent to deleting that
    // block; allow only roundoff differences due to matrix dimension/order.
    for (a, b) in full.cam_from_world.iter().zip(&reduced.cam_from_world) {
        assert_pose_equivalent(a, b);
    }
    assert_pose_equivalent(&full.lighthouse_from_world, &reduced.lighthouse_from_world);
    assert_pose_equivalent(&full.ref_from_touch[active], &reduced.ref_from_touch[0]);
}

fn assert_pose_equivalent(a: &Isometry3<f64>, b: &Isometry3<f64>) {
    assert!((a.translation.vector - b.translation.vector).norm() < 1e-8);
    assert!(a.rotation.angle_to(&b.rotation) < 1e-8);
}

#[test]
fn all_active_numerical_control() {
    let (p, initial) = replayed_problem(27);
    let obs = associate(&p, &initial, 4.0, 1.5);
    let out = solve(&p, &initial, &obs, 0.5, 30, Some(&Prior::around(&initial)));
    let values: Vec<_> = out
        .cam_from_world
        .iter()
        .chain(std::iter::once(&out.lighthouse_from_world))
        .chain(&out.ref_from_touch)
        .flat_map(|p| {
            let q = p.rotation.quaternion();
            [
                p.translation.x,
                p.translation.y,
                p.translation.z,
                q.i,
                q.j,
                q.k,
                q.w,
            ]
        })
        .collect();
    // Captured from the original all-active solver, before inactive-block handling.
    // Compare every pose component, not merely the scalar objective. The small
    // tolerance permits platform roundoff; no active arithmetic changed.
    let expected = [
        7.201_710_983_117_948e-8,
        -0.9004167001324976,
        3.423338922034457,
        -0.09078212181615045,
        -0.8904673554477862,
        0.20383876784923663,
        0.3965807004407662,
        0.00013880698482780316,
        0.900498035017838,
        3.4234000830659186,
        -0.8904391437304504,
        -0.09078216752555582,
        0.3966446788754964,
        0.2038375041342954,
        9.097_325_427_255_708e-5,
        -0.8461669131529814,
        3.158008134232373,
        -0.2768831740517121,
        0.06492513667998931,
        -0.01876883893053263,
        0.9585239513122945,
        0.10010670755702845,
        -0.6000076422211211,
        -2.300123985440893,
        0.0009411002499961275,
        0.5645721547677617,
        -0.001388622571038408,
        0.8253820134450237,
        0.15997100573962877,
        -0.049962281495402945,
        0.08000607265936356,
        0.14954275217138321,
        0.05026835834180922,
        -0.00016442482118088765,
        0.9874765973870498,
        -0.18001199948257143,
        -0.039963020376788505,
        0.08999857446414307,
        -0.06845631633483336,
        0.9903174716919557,
        0.03380122785372578,
        0.115941860477992,
    ];
    for (got, expected) in values.iter().zip(expected) {
        assert!((got - expected).abs() < 1e-9, "{got} != {expected}");
    }
    assert_ne!(out, initial);
}
