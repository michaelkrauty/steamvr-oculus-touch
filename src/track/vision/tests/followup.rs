// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! Frozen baseline projection and focused follow-up performance regressions.
use super::*;

fn original_project(cam: &Camera, body: &Body, x: &Nominal, i: usize) -> Option<Projection> {
    let led = &body.leds[i];
    let arm = led.pos - body.imu_pos;
    let r = x.q.to_rotation_matrix().into_inner();
    let rcw = cam
        .world_from_cam
        .rotation
        .inverse()
        .to_rotation_matrix()
        .into_inner();
    let pc = rcw * (x.p + r * arm - cam.world_from_cam.translation.vector);
    let (uv, duv) = project_with_jacobian(&cam.model, &pc)?;
    let mut dpc = SMatrix::<f64, 3, 6>::zeros();
    dpc.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(rcw * (-r * skew(&arm))));
    dpc.fixed_view_mut::<3, 3>(0, 3).copy_from(&rcw);
    let facing = (rcw * (r * led.dir)).dot(&(-pc)) / pc.norm();
    Some(Projection {
        led: i,
        uv,
        j: duv * dpc,
        facing,
    })
}

fn projection_bits(p: Option<Projection>) -> Option<Vec<u64>> {
    p.map(|p| {
        std::iter::once(p.led as u64)
            .chain(
                p.uv.iter()
                    .chain(p.j.iter())
                    .chain(std::iter::once(&p.facing))
                    .map(|v| v.to_bits()),
            )
            .collect()
    })
}

#[test]
fn batched_projection_matches_baseline_bits() {
    let mut cam = camera();
    let b = body();
    for seed in 0..128 {
        let mut x = state();
        x.q = UnitQuaternion::from_euler_angles(
            seed as f64 * 0.13,
            seed as f64 * 0.23,
            seed as f64 * 0.31,
        );
        x.p.z += (seed as f64 - 64.0) * 0.1;
        cam.world_from_cam.rotation =
            UnitQuaternion::from_euler_angles(0.1, seed as f64 * 0.07, -0.2);
        let projector = Projector::new(&cam, &b, &x);
        for i in 0..b.leds.len() {
            assert_eq!(
                projection_bits(projector.project(i)),
                projection_bits(original_project(&cam, &b, &x, i))
            );
        }
    }
}

#[test]
#[ignore = "offline timing; run release, single threaded, with --nocapture"]
fn benchmark_followup_projection() {
    use std::{hint::black_box, time::Instant};
    let (cam, b, x) = (camera(), body(), state());
    let mut times = [Vec::new(), Vec::new()];
    for round in 0..12 {
        for which in [round % 2, 1 - round % 2] {
            let start = Instant::now();
            for _ in 0..5000 {
                let (cam, b, x) = (black_box(&cam), black_box(&b), black_box(&x));
                if which == 0 {
                    for i in 0..b.leds.len() {
                        black_box(original_project(cam, b, x, i));
                    }
                } else {
                    let projector = Projector::new(cam, b, x);
                    for i in 0..b.leds.len() {
                        black_box(projector.project(i));
                    }
                }
            }
            times[which].push(start.elapsed().as_secs_f64() * 1e6 / 5000.0);
        }
    }
    for (name, t) in ["baseline", "batched"].into_iter().zip(&mut times) {
        t.sort_by(f64::total_cmp);
        eprintln!(
            "{name} projection us/batch min/median/max: {:.3}/{:.3}/{:.3}",
            t[0], t[6], t[11]
        );
    }
}
