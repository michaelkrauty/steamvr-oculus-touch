// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Fixed-storage differential regressions. Invoked by scripts/check.sh's cargo test.
use super::*;
use std::cell::Cell;
thread_local! {
    pub(super) static LEGACY: Cell<bool> = const { Cell::new(false) };
}

// Frozen pre-fixed-storage oracle: retain allocation and arithmetic order.
/// Central-difference Jacobian columns of `f(param, step)` over `n` parameters.
fn legacy_numeric<const R: usize>(
    n: usize,
    mut f: impl FnMut(usize, f64) -> Option<[f64; R]>,
) -> Option<Vec<[f64; R]>> {
    (0..n)
        .map(|a| {
            let (plus, minus) = (f(a, H)?, f(a, -H)?);
            Some(std::array::from_fn(|row| {
                (plus[row] - minus[row]) / (2.0 * H)
            }))
        })
        .collect()
}

pub(super) fn legacy_build(p: &Problem, s: &State, o: &Options, l: &Layout) -> Normal {
    let mut n = Normal::new(p.samples.len(), l.globals);
    let delta = o.huber_px / o.sigma_px;
    for (i, sample) in p.samples.iter().enumerate() {
        let (Some(w), Some(model)) = (s.ref_from_obj[i], p.leds.get(&sample.device_id)) else {
            continue;
        };
        for v in &sample.views {
            let (cam, t_cam) = (&p.cameras[v.camera], s.ref_from_cam[v.camera]);
            for ob in &v.obs {
                let Some(led) = model.get(ob.led) else {
                    continue;
                };
                let residual = |t_cam: &Isometry3<f64>, w: &Isometry3<f64>| {
                    reprojection(cam, t_cam, w, &led.pos, &ob.xy).map(|e| [e.x, e.y])
                };
                let Some(r) = residual(&t_cam, &w) else {
                    continue;
                };
                let Some(jw) =
                    legacy_numeric(6, |a, h| residual(&t_cam, &retract(&w, &unit::<6>(a, h))))
                else {
                    continue;
                };
                let (jg, cols): (Vec<[f64; 2]>, Vec<usize>) = match l.cam[v.camera] {
                    Some(off) => {
                        match legacy_numeric(6, |a, h| {
                            residual(&retract(&t_cam, &unit::<6>(a, h)), &w)
                        }) {
                            Some(j) => (j, (off..off + 6).collect()),
                            None => continue,
                        }
                    }
                    None => (Vec::new(), Vec::new()),
                };
                let norm = (r[0] * r[0] + r[1] * r[1]).sqrt() / o.sigma_px;
                n.add(
                    i,
                    &r,
                    &jw,
                    &jg,
                    &cols,
                    huber_weight(norm, delta) / (o.sigma_px * o.sigma_px),
                );
            }
        }
    }
    n
}

fn arithmetic_controls() -> String {
    use crate::calib::capture::{Observation, View};
    fn numeric<const R: usize>(
        n: usize,
        f: impl FnMut(usize, f64) -> Option<[f64; R]>,
    ) -> Option<Vec<[f64; R]>> {
        if LEGACY.get() {
            legacy_numeric(n, f)
        } else {
            super::numeric(f).map(|j| j.to_vec())
        }
    }
    let mut output = String::new();
    // Exact central-difference callback schedule, including short-circuit failures.
    for fail in 0..=12 {
        let mut calls = Vec::new();
        let j = numeric(6, |a, h| {
            calls.push((a, h.to_bits()));
            if calls.len() == fail + 1 {
                None
            } else {
                Some([a as f64 + h * 3.0, h * -7.0])
            }
        });
        assert_eq!(j.is_some(), fail == 12);
        assert_eq!(calls.len(), (fail + 1).min(12));
        for (k, &(a, h)) in calls.iter().enumerate() {
            assert_eq!(a, k / 2);
            assert_eq!(h, (if k % 2 == 0 { H } else { -H }).to_bits());
        }
        output += &format!(
            "numeric fail={fail} calls={calls:?} j={:?}\n",
            j.as_ref()
                .map(|x| x.iter().flatten().map(|v| v.to_bits()).collect::<Vec<_>>())
        );
    }
    let cameras = [crate::calib::synth::camera_model(); 2];
    let leds = BTreeMap::from([(
        1,
        vec![Led {
            pos: Vector3::zeros(),
            dir: Vector3::z(),
        }],
    )]);
    for reference in 0..2 {
        for (name, z, offset) in [
            ("inlier", 2.0, 0.2),
            ("outlier", 2.0, 40.0),
            ("behind", -1.0, 0.0),
            ("boundary", 1.5e-6, 0.0),
        ] {
            let samples = vec![WandSample {
                device_id: 1,
                device_time_ns: 0,
                host_ns: 0,
                views: (0..2)
                    .map(|camera| View {
                        camera,
                        strong: true,
                        cam_from_obj: None,
                        obs: vec![Observation {
                            led: 0,
                            xy: Vector2::new(640.0 + offset, 480.0),
                        }],
                    })
                    .collect(),
            }];
            let p = Problem {
                cameras: &cameras,
                leds: &leds,
                samples: &samples,
                reference,
            };
            let s = State {
                ref_from_cam: vec![Isometry3::identity(); 2],
                ref_from_obj: vec![Some(Isometry3::translation(0.0, 0.0, z))],
            };
            if name == "behind" || name == "boundary" {
                let w = s.ref_from_obj[0].unwrap();
                let project = |w: &Isometry3<f64>| {
                    reprojection(
                        &cameras[0],
                        &s.ref_from_cam[0],
                        w,
                        &Vector3::zeros(),
                        &Vector2::zeros(),
                    )
                };
                assert_eq!(project(&w).is_some(), name == "boundary");
                if name == "boundary" {
                    assert!(project(&retract(&w, &unit::<6>(2, -H))).is_none());
                }
            }
            let n = build(&p, &s, &Options::default(), &Layout::new(&p));
            if name == "behind" || name == "boundary" {
                assert!(n.hww[0].iter().all(|x| *x == 0.0));
                assert!(n.hgg.iter().all(|x| *x == 0.0));
            } else {
                assert!(n.hww[0].norm() > 0.0 && n.hgg.norm() > 0.0);
            }
            let bits: Vec<_> = n
                .hww
                .iter()
                .flat_map(|m| m.iter())
                .chain(n.bw.iter().flat_map(|v| v.iter()))
                .chain(n.hwg.iter().flat_map(|m| m.iter()))
                .chain(n.hgg.iter())
                .chain(n.bg.iter())
                .map(|v| v.to_bits())
                .collect();
            output += &format!("normal ref={reference} case={name} bits={bits:?}\n");
        }
    }
    // Nonzero translations/rotations and off-axis LEDs exercise every numeric pose column.
    for (camera, model) in cameras.iter().enumerate() {
        let tc = Isometry3::new(
            Vector3::new(0.1 * camera as f64, -0.03, 0.02),
            Vector3::new(0.03, -0.07, 0.1),
        );
        let w = Isometry3::new(
            Vector3::new(0.2, -0.15, 1.8),
            Vector3::new(-0.2, 0.15, 0.05),
        );
        let led = Vector3::new(0.035, -0.012, 0.023);
        let xy = Vector2::new(610.0, 455.0);
        for global in [false, true] {
            let j = numeric(6, |a, h| {
                let (t, obj) = if global {
                    (retract(&tc, &unit::<6>(a, h)), w)
                } else {
                    (tc, retract(&w, &unit::<6>(a, h)))
                };
                reprojection(model, &t, &obj, &led, &xy).map(|r| [r.x, r.y])
            })
            .unwrap();
            let bits: Vec<_> = j.iter().flatten().map(|v| v.to_bits()).collect();
            output += &format!("pose_jac camera={camera} global={global} bits={bits:?}\n");
        }
    }
    output
}

// Restore even if a differential assertion or solve panics.
fn with_legacy<T>(legacy: bool, f: impl FnOnce() -> T) -> T {
    struct Reset(bool);
    impl Drop for Reset {
        fn drop(&mut self) {
            LEGACY.set(self.0);
        }
    }
    let _reset = Reset(LEGACY.replace(legacy));
    f()
}

#[test]
fn fixed_storage_arithmetic() {
    assert_eq!(
        with_legacy(true, arithmetic_controls),
        with_legacy(false, arithmetic_controls)
    );
}

#[test]
fn fixed_storage_whole_solve() {
    use crate::calib::{solve, synth};
    use crate::track::tracker::stillness_tests::counted;
    for seed in [59, 83] {
        let tr = synth::trajectory(&synth::TrajectoryParams {
            duration_s: 12.0,
            seed,
            ..Default::default()
        });
        let run = |legacy| {
            let samples = tr.scene.samples.clone();
            with_legacy(legacy, || {
                counted(|| {
                    solve::solve_samples(
                        &tr.scene.serials,
                        &tr.scene.cameras,
                        &tr.scene.leds,
                        samples,
                        &tr.imu,
                        &Default::default(),
                    )
                })
            })
        };
        let (baseline, (old_requests, _, old_bytes)) = run(true);
        let (fixed, (requests, _, bytes)) = run(false);
        let baseline = baseline.unwrap();
        let fixed = fixed.unwrap();
        solve::check(&baseline, &Default::default()).unwrap();
        solve::check(&fixed, &Default::default()).unwrap();
        // Derived Debug includes every Solution field; f64 formatting round-trips.
        assert_eq!(format!("{baseline:?}"), format!("{fixed:?}"));
        assert!(requests < old_requests && bytes < old_bytes);
        println!("fixed_storage seed={seed} baseline_requests={old_requests} requests={requests} baseline_requested_bytes={old_bytes} requested_bytes={bytes}");
    }
}
