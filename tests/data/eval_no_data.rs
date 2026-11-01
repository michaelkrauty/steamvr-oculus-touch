// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
// Synthetic input for test_eval_no_data.py; the callers are extracted from production.
use cv1touch::{align, eval, register};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use std::path::Path;

struct RigDevice {
    name: String,
    poses: Vec<eval::Published>,
}

fn main() {
    let dir = std::path::PathBuf::from(std::env::args().nth(1).unwrap());
    let pubs: Vec<_> = (0..8001)
        .map(|i| {
            let t = i as f64 * 0.01;
            eval::Published {
                t_ns: 10_000_000_000 + i * 10_000_000,
                pose: Isometry3::from_parts(
                    Translation3::new(0.2 * t.sin(), 0.0, 0.0),
                    UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.4 * (t * t).sin(), 0.0)),
                ),
                vel: Vector3::zeros(),
                ang_vel_world: Vector3::zeros(),
            }
        })
        .collect();
    let reg = serde_json::json!({
        "format": align::REGISTRATION_FORMAT, "version": 1, "created_unix_s": 0,
        "lighthouse_from_world": {"orient_xyzw": [0,0,0,1], "pos": [0,0,0]},
        "universe_id": 1, "touch": "left", "reference": "synthetic", "pairs": 1000,
        "pos_rms_mm": 0, "rot_rms_deg": 0
    });
    let regpath = dir.join("registration.json");
    std::fs::write(&regpath, serde_json::to_vec(&reg).unwrap()).unwrap();
    for (name, start, count) in [
        ("short", 1000, 301),
        // End truth more than the full 150 ms lag search before the moving window.
        ("no-moving", 0, 950),
        ("continuous", 0, 8001),
        ("partial", 1000, 1001),
    ] {
        println!("CASE {name}");
        let tf = serde_json::json!({
            "format": eval::TRUTH_FORMAT, "version": 1, "reference": "synthetic",
            "universe_id": 1, "latency_adjustment_ms": 0.0,
            "windows": [
                ["still", pubs[0].t_ns, pubs[999].t_ns],
                ["move", pubs[1000].t_ns, pubs[6999].t_ns],
                ["still", pubs[7000].t_ns, pubs[8000].t_ns]
            ],
            "samples": pubs.iter().skip(start).take(count)
                .map(|p| eval::truth_row(p.t_ns, &p.pose)).collect::<Vec<_>>()
        });
        let base = dir.join(name);
        std::fs::write(
            base.with_extension("truth.json"),
            serde_json::to_vec(&tf).unwrap(),
        )
        .unwrap();
        assert_eq!(
            eval_report_with(&base, &[pubs.clone(), vec![]], &regpath, true),
            0
        );
    }
    let b = RigDevice {
        name: "published".into(),
        poses: pubs.clone(),
    };
    println!("CASE rig-missing");
    let a = RigDevice {
        name: "reference".into(),
        poses: pubs
            .iter()
            .enumerate()
            .filter(|(i, _)| *i <= 300 || *i >= 7700)
            .map(|(i, p)| {
                let mut p = *p;
                if i >= 7700 {
                    p.pose.rotation *=
                        UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.2, 0.0));
                }
                p
            })
            .collect(),
    };
    rig_pair(&a, &b);
    println!("CASE rig-valid");
    rig_pair(&b, &b);
}
