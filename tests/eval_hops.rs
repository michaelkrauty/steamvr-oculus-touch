use cv1touch::eval::{self, Published, Truth, TruthSample};
use nalgebra::{Isometry3, Vector3};
#[test]
fn smooth_error_drift_during_truth_loss_is_not_a_hop() {
    // 100 Hz publication, smooth motion and 30 mm/s error drift: 0.3 mm per frame.
    let published: Vec<_> = (0..=1000)
        .map(|i| {
            let t = i as f64 * 0.01;
            Published {
                t_ns: 1_000_000_000 + i * 10_000_000,
                pose: Isometry3::translation(0.1 * t + 0.03 * t, 0.0, 0.0),
                vel: Vector3::new(0.13, 0.0, 0.0),
                ang_vel_world: Vector3::zeros(),
            }
        })
        .collect();
    let full: Vec<_> = (0..=1000)
        .map(|i| TruthSample {
            t_ns: 1_000_000_000 + i * 10_000_000,
            pose: Isometry3::translation(0.1 * i as f64 * 0.01, 0.0, 0.0),
        })
        .collect();
    let missing: Vec<_> = full
        .iter()
        .copied()
        .enumerate()
        .filter(|(i, _)| !(400..=430).contains(i))
        .map(|(_, s)| s)
        .collect();
    let segments = eval::offset_segments(
        &published,
        &missing,
        &Isometry3::identity(),
        0,
        &eval::SegmentParams::default(),
    );
    assert_eq!(
        segments.len(),
        1,
        "normal translation drift does not invalidate jig segment"
    );
    let tr = Truth {
        samples: &missing,
        world_from_lh: Isometry3::identity(),
        segments: segments.clone(),
    };
    let continuous = Truth {
        samples: &full,
        world_from_lh: Isometry3::identity(),
        segments,
    };
    assert_eq!(eval::hops(&published, &continuous, 0, 0.005), (0, 1000));
    assert!(tr.at(published[415].t_ns as i64).is_none());
    let eligible = published
        .windows(2)
        .filter(|w| w.iter().all(|p| tr.at(p.t_ns as i64).is_some()))
        .count();
    assert_eq!(eligible, 967);
    let shifted: Vec<_> = published
        .iter()
        .map(|p| Published {
            t_ns: p.t_ns + 20_000_000,
            ..*p
        })
        .collect();
    assert_eq!(
        eval::hops(&shifted, &continuous, 20_000_000, 0.005),
        (0, 1000)
    );
    let got = eval::hops(&published, &tr, 0, 0.005);
    println!("continuous=(0,1000); tracking gap=310 ms; adjacent comparable pairs={eligible}; actual={got:?}");
    assert_eq!(
        got,
        (0, eligible),
        "smooth drift across absent truth must not become a consecutive-pose hop"
    );
    assert_eq!(eval::hops(&shifted, &tr, 20_000_000, 0.005), (0, eligible));
}

#[test]
fn hop_adjacency_controls() {
    let samples: Vec<_> = (0..3)
        .map(|i| TruthSample {
            t_ns: 1_000_000_000 + i * 10_000_000,
            pose: Isometry3::identity(),
        })
        .collect();
    let published: Vec<_> = samples
        .iter()
        .enumerate()
        .map(|(i, s)| Published {
            t_ns: s.t_ns,
            pose: Isometry3::translation(if i == 1 { 0.125 } else { 0.0 }, 0.0, 0.0),
            vel: Vector3::zeros(),
            ang_vel_world: Vector3::zeros(),
        })
        .collect();
    let truth = Truth::single(&samples, Isometry3::identity(), Isometry3::identity());
    assert_eq!(eval::hops(&published, &truth, 0, 0.1), (2, 2));
    assert_eq!(eval::hops(&published, &truth, 0, 0.125), (0, 2));
    assert_eq!(eval::hops(&[], &truth, 0, 0.1), (0, 0));
    assert_eq!(eval::hops(&published[..1], &truth, 0, 0.1), (0, 0));
    let absent = Truth::single(&[], Isometry3::identity(), Isometry3::identity());
    assert_eq!(eval::hops(&published, &absent, 0, 0.1), (0, 0));
    let one = Truth::single(&samples[1..2], Isometry3::identity(), Isometry3::identity());
    assert_eq!(eval::hops(&published, &one, 0, 0.1), (0, 0));
}
