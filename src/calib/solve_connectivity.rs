// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Production solve regressions: only simultaneous surviving observations join cameras.
use crate::calib::{ba, capture, init, solve, synth};
use nalgebra::{Translation3, Vector2};

#[test]
fn rejects_disconnected_final_support() {
    for (seed, pixel_noise) in [(7, 0.05), (19, 0.15), (31, 0.15), (43, 0.10)] {
        for corrupt in [false, true] {
            let tr = synth::trajectory(&synth::TrajectoryParams {
                duration_s: 30.0,
                seed,
                pixel_noise,
                pnp_rot_noise_rad: 0.0,
                pnp_pos_noise_m: 0.0,
                ..Default::default()
            });
            let mut samples = tr.scene.samples.clone();
            let mut rng = synth::Rng::new(seed + 100);
            let mut bridges = 0;
            for s in &mut samples {
                let t = s.device_time_ns;
                if !(15_000_000_000..15_500_000_000).contains(&t) {
                    s.views.retain(|v| {
                        if t < 15_000_000_000 {
                            v.camera == 2
                        } else {
                            v.camera != 2
                        }
                    });
                } else {
                    // An indirect chain: camera 0 never shares a sample with camera 2.
                    s.views.retain(|v| v.camera != 0);
                    if s.views.len() >= 2 && s.views.iter().any(|v| v.camera == 2) {
                        bridges += 1;
                        for v in s.views.iter_mut().filter(|v| v.camera == 2) {
                            // Identical initialization perturbation in clean/corrupt controls.
                            v.cam_from_obj = v
                                .cam_from_obj
                                .map(|t| Translation3::new(0.12, 0.0, 0.0) * t);
                            if corrupt {
                                for ob in &mut v.obs {
                                    ob.xy += Vector2::new(rng.gauss(), rng.gauss()) * 12.0;
                                }
                            }
                        }
                    }
                }
            }
            samples.retain(|s| !s.views.is_empty());
            assert!(samples.iter().all(|s| s.views.iter().all(|v| v.strong
                && v.cam_from_obj.is_some()
                && v.obs.len() >= capture::MIN_VIEW_OBS)));
            assert!(samples.iter().any(|s| s.views.len() == 1));
            assert!(!init::pair_samples(&samples).contains_key(&(0, 2)));
            let mut no_bridges = samples.clone();
            for s in &mut no_bridges {
                if s.views.len() > 1 && s.views.iter().any(|v| v.camera == 2) {
                    s.views.retain(|v| v.camera != 2);
                }
            }
            let disconnected = capture::Capture {
                serials: tr.scene.serials.clone(),
                cameras: tr.scene.cameras.clone(),
                leds: tr.scene.leds.clone(),
                samples: no_bridges,
                imu: tr.imu.clone(),
            };
            let error = solve::solve_capture(&disconnected, &Default::default()).unwrap_err();
            assert!(
                error.contains("share too few confident controller sightings"),
                "{error}"
            );
            let initial = init::initialize(
                &samples,
                &tr.scene.serials,
                solve::SolveOptions::default().min_links,
            )
            .unwrap();
            println!("CASE seed={seed} pixel_noise={pixel_noise} corrupt={corrupt} bridges={bridges} initial_links={:?}",initial.links);
            assert!(bridges >= 30);
            let cap = capture::Capture {
                serials: tr.scene.serials.clone(),
                cameras: tr.scene.cameras.clone(),
                leds: tr.scene.leds.clone(),
                samples,
                imu: tr.imu.clone(),
            };
            let result = solve::solve_capture(&cap, &Default::default()).and_then(|sol| {
                solve::check(&sol, &Default::default())?;
                Ok(sol)
            });
            if corrupt {
                let error = result.expect_err("corrupted bridges must not yield a calibration");
                println!("REJECTED seed={seed}: {error}");
                assert!(
                    error.contains("disconnected after reprojection rejection"),
                    "{error}"
                );
                assert!(error.contains(&cap.serials[2]), "{error}");
                continue;
            }
            let sol = result.unwrap();
            let truth = tr.scene.world_from_cam[0].inverse() * tr.scene.world_from_cam[2];
            let got = sol.sensors[0].world_from_cam.inverse() * sol.sensors[2].world_from_cam;
            let mm = (got.translation.vector - truth.translation.vector).norm() * 1000.0;
            let deg = got.rotation.angle_to(&truth.rotation).to_degrees();
            println!("POSE_ERROR seed={seed} corrupt={corrupt} camera=2 mm={mm} deg={deg}");
            assert!(mm < 25.0 && deg < 0.5);
        }
    }
}

#[test]
fn one_camera_capture_needs_no_intercamera_link() {
    let tr = synth::trajectory(&synth::TrajectoryParams {
        duration_s: 30.0,
        seed: 7,
        pixel_noise: 0.05,
        pnp_rot_noise_rad: 0.0,
        pnp_pos_noise_m: 0.0,
        ..Default::default()
    });
    let mut samples = tr.scene.samples;
    for sample in &mut samples {
        sample.views.retain(|v| v.camera == 0);
    }
    samples.retain(|s| !s.views.is_empty());
    let cap = capture::Capture {
        serials: tr.scene.serials[..1].to_vec(),
        cameras: tr.scene.cameras[..1].to_vec(),
        leds: tr.scene.leds,
        samples,
        imu: tr.imu,
    };
    let sol = solve::solve_capture(&cap, &Default::default()).unwrap();
    solve::check(&sol, &Default::default()).unwrap();
}

// Test the graph criterion independently of initialization's robust min_links threshold. Unknown
// LEDs, missing poses and nonprojecting points must not manufacture a surviving bridge.
#[test]
fn only_fitted_observations_join_the_final_graph() {
    let sc = synth::trajectory(&synth::TrajectoryParams {
        duration_s: 2.0,
        pixel_noise: 0.0,
        ..Default::default()
    })
    .scene;
    let mut samples = vec![
        sc.samples
            .iter()
            .find(|s| s.views.len() == 3)
            .unwrap()
            .clone();
        2
    ];
    samples[0].views.retain(|v| v.camera != 2);
    samples[1].views.retain(|v| v.camera != 0);
    // One observation per view and one shared sample per link is enough for connectivity. This
    // deliberately says nothing about geometric rank, which is not this guard's responsibility.
    for view in samples.iter_mut().flat_map(|s| &mut s.views) {
        view.obs.truncate(1);
    }
    let mut state = ba::State {
        ref_from_cam: sc.world_from_cam.clone(),
        ref_from_obj: init::wand_poses(&samples, &sc.world_from_cam),
    };
    let connected = |samples: &[capture::WandSample], state: &ba::State| {
        super::ensure_connected(
            &ba::Problem {
                cameras: &sc.cameras,
                leds: &sc.leds,
                samples,
                reference: 0,
            },
            state,
            &sc.serials,
        )
    };
    connected(&samples, &state).unwrap();
    // The fitted graph does not consult engine confidence or initial per-view pose seeds.
    for view in samples.iter_mut().flat_map(|s| &mut s.views) {
        view.cam_from_obj = None;
        view.strong = false;
    }
    connected(&samples, &state).unwrap();
    let assert_disconnected = |samples: &[capture::WandSample], state: &ba::State| {
        let err = connected(samples, state).unwrap_err();
        assert!(err.contains(&sc.serials[2]), "{err}");
        assert!(err.contains(&sc.serials[0]), "{err}");
    };
    let pose = state.ref_from_obj[1].take();
    assert_disconnected(&samples, &state);
    state.ref_from_obj[1] = pose;
    let saved = samples[1].views[1].obs[0];
    samples[1].views[1].obs[0].led = usize::MAX;
    assert_disconnected(&samples, &state);
    samples[1].views[1].obs[0] = saved;
    state.ref_from_obj[1] = Some(nalgebra::Isometry3::translation(0.0, 0.0, -1000.0));
    state.ref_from_cam.fill(nalgebra::Isometry3::identity());
    assert_disconnected(&samples, &state);
}

/// At the projection plane, equivalent isometry compositions can round to opposite sides of the
/// clipping threshold. Compare with BA's actual final statistics, not a copied projection oracle.
#[test]
fn projection_plane_support_matches_ba_statistics() {
    use nalgebra::{Isometry3, UnitQuaternion, Vector3};
    let cameras = [synth::camera_model(); 2];
    let serials = ["reference".to_owned(), "near-plane".to_owned()];
    let led = synth::touch_leds()[0];
    let leds = std::collections::BTreeMap::from([(1, vec![led])]);
    let samples = [capture::WandSample {
        device_id: 1,
        device_time_ns: 0,
        host_ns: 0,
        views: (0..2)
            .map(|camera| capture::View {
                camera,
                cam_from_obj: None,
                strong: false,
                obs: vec![capture::Observation {
                    led: 0,
                    xy: Vector2::zeros(),
                }],
            })
            .collect(),
    }];
    let p = ba::Problem {
        cameras: &cameras,
        leds: &leds,
        samples: &samples,
        reference: 0,
    };
    let camera_pose = Isometry3::from_parts(
        Translation3::new(0.4, 0.2, 2.0),
        UnitQuaternion::from_euler_angles(0.3, -0.4, 0.2),
    );
    let object_rotation = UnitQuaternion::from_euler_angles(0.7, -0.2, 0.4);
    let mut supported = 0;
    let mut clipped = 0;
    for step in -128..=128 {
        let point = Vector3::new(0.1, -0.2, 1e-6 + f64::from(step) * (f64::EPSILON / 64.0));
        let camera_from_object = Isometry3::from_parts(
            Translation3::from(point - object_rotation * led.pos),
            object_rotation,
        );
        let mut state = ba::State {
            ref_from_cam: vec![Isometry3::identity(), camera_pose],
            ref_from_obj: vec![Some(camera_pose * camera_from_object)],
        };
        let report = ba::solve(
            &p,
            &mut state,
            &ba::Options {
                max_iterations: 0,
                ..Default::default()
            },
        );
        assert_eq!(report.iterations, 0);
        assert_eq!(report.camera_obs[0], 1);
        let ba_supported = report.camera_obs[1] == 1;
        assert_eq!(
            super::ensure_connected(&p, &state, &serials).is_ok(),
            ba_supported,
            "projection-plane step {step}, camera_obs={:?}",
            report.camera_obs
        );
        if ba_supported {
            supported += 1;
        } else {
            clipped += 1;
        }
    }
    assert!(
        supported > 0 && clipped > 0,
        "must exercise both sides of clipping"
    );
}
