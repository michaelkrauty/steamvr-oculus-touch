// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! Re-poses a sensor against the tracker (see `replay::Options::probe_sensor`). A sensor that has
//! moved since calibration disagrees with the others by a pixel or more; the controller poses
//! tracked with the others and the IMU pin its pose. The rest of the calibration, and with it the
//! world frame and the Lighthouse registration, stays as it is.

use nalgebra::{
    Isometry3, Matrix6, Point3, Translation3, UnitQuaternion, Vector2, Vector3, Vector6,
};

use super::eskf::skew;
use super::pipeline::Probe;
use super::vision::project_with_jacobian;
use crate::calib::camera::CameraModel;
use crate::calib::capture::Led;

#[derive(Debug, Clone, PartialEq)]
pub struct Refined {
    pub world_from_cam: Isometry3<f64>,
    /// Median residual of the final matches at the initial and the refined pose, pixels.
    pub before_px: f64,
    pub after_px: f64,
    pub pairs: usize,
    pub frames: usize,
}

/// A probe's LEDs that face the sensor, in the world, and its blobs.
struct Sighting {
    leds: Vec<Vector3<f64>>,
    blobs: Vec<Vector2<f64>>,
}

/// Mutual nearest LED/blob pairs within `gate` px whose runner-up (another blob for the LED,
/// another LED for the blob) is at least `margin` px further: (world point, blob).
fn matches(
    model: &CameraModel,
    cam_from_world: &Isometry3<f64>,
    s: &Sighting,
    gate: f64,
    margin: f64,
) -> Vec<(Vector3<f64>, Vector2<f64>)> {
    let uv: Vec<Option<Vector2<f64>>> = s
        .leds
        .iter()
        .map(|w| model.project(&(cam_from_world * Point3::from(*w)).coords))
        .collect();
    let mut out = Vec::new();
    for (i, u) in uv.iter().enumerate() {
        let Some(u) = u else { continue };
        let mut d: Vec<(f64, usize)> = s
            .blobs
            .iter()
            .enumerate()
            .map(|(b, x)| ((x - u).norm(), b))
            .collect();
        d.sort_by(|a, b| a.0.total_cmp(&b.0));
        let Some(&(d1, b)) = d.first() else { continue };
        if d1 > gate || d.get(1).is_some_and(|x| x.0 < d1 + margin) {
            continue;
        }
        let rival = uv
            .iter()
            .enumerate()
            .any(|(j, v)| j != i && v.is_some_and(|v| (s.blobs[b] - v).norm() < d1 + margin));
        if !rival {
            out.push((s.leds[i], s.blobs[b]));
        }
    }
    out
}

fn residual(
    model: &CameraModel,
    cam_from_world: &Isometry3<f64>,
    p: &(Vector3<f64>, Vector2<f64>),
) -> Option<f64> {
    Some((p.1 - model.project(&(cam_from_world * Point3::from(p.0)).coords)?).norm())
}

fn median(mut v: Vec<f64>) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_by(|a, b| a.total_cmp(b));
    v[v.len() / 2]
}

/// Gauss-Newton with Huber weights on cam_from_world, perturbed on the left in the camera frame.
fn solve(
    model: &CameraModel,
    mut cfw: Isometry3<f64>,
    pairs: &[(Vector3<f64>, Vector2<f64>)],
) -> Isometry3<f64> {
    let huber = 1.0;
    for _ in 0..15 {
        let (mut h, mut g) = (Matrix6::<f64>::zeros(), Vector6::<f64>::zeros());
        for (w, xy) in pairs {
            let pc = (cfw * Point3::from(*w)).coords;
            let Some((uv, duv)) = project_with_jacobian(model, &pc) else {
                continue;
            };
            let r = xy - uv;
            let mut dpc = nalgebra::SMatrix::<f64, 3, 6>::zeros();
            dpc.fixed_view_mut::<3, 3>(0, 0).copy_from(&(-skew(&pc)));
            dpc.fixed_view_mut::<3, 3>(0, 3)
                .copy_from(&nalgebra::Matrix3::identity());
            let j = duv * dpc;
            let wt = if r.norm() <= huber {
                1.0
            } else {
                huber / r.norm()
            };
            h += j.transpose() * j * wt;
            g += j.transpose() * r * wt;
        }
        let Some(step) = h.cholesky().map(|c| c.solve(&g)) else {
            break;
        };
        let rot = UnitQuaternion::from_scaled_axis(Vector3::new(step[0], step[1], step[2]));
        cfw = Isometry3::from_parts(
            Translation3::from(Vector3::new(step[3], step[4], step[5])),
            rot,
        ) * cfw;
        if step.norm() < 1e-9 {
            break;
        }
    }
    cfw
}

/// `bodies` are the controllers' LEDs by hand; probes whose pose is less certain than
/// `max_pos_sd` (metres) are not used.
pub fn refine_sensor(
    model: &CameraModel,
    initial: &Isometry3<f64>,
    bodies: [Option<&[Led]>; 2],
    probes: &[Probe],
    max_pos_sd: f64,
) -> Result<Refined, String> {
    let cam_pos = initial.translation.vector;
    let sightings: Vec<Sighting> = probes
        .iter()
        .filter(|p| p.pos_sd <= max_pos_sd && !p.blobs.is_empty())
        .filter_map(|p| {
            let leds = bodies[p.hand]?;
            let world: Vec<Vector3<f64>> = leds
                .iter()
                .filter_map(|l| {
                    let w = p.world_from_model * Point3::from(l.pos);
                    let n = p.world_from_model.rotation * l.dir;
                    let to_cam = (cam_pos - w.coords).normalize();
                    (n.dot(&to_cam) > 0.2).then_some(w.coords)
                })
                .collect();
            (!world.is_empty()).then(|| Sighting {
                leds: world,
                blobs: p.blobs.clone(),
            })
        })
        .collect();
    let mut cfw = initial.inverse();
    let mut pairs = Vec::new();
    for (gate, margin) in [(4.0, 1.5), (2.5, 1.0), (1.5, 1.0), (1.0, 1.0)] {
        pairs = sightings
            .iter()
            .flat_map(|s| matches(model, &cfw, s, gate, margin))
            .collect();
        if pairs.len() < 100 {
            return Err(format!(
                "only {} LED matches in {} usable frames",
                pairs.len(),
                sightings.len()
            ));
        }
        cfw = solve(model, cfw, &pairs);
    }
    let initial_cfw = initial.inverse();
    let before_px = median(
        pairs
            .iter()
            .filter_map(|p| residual(model, &initial_cfw, p))
            .collect(),
    );
    let after_px = median(
        pairs
            .iter()
            .filter_map(|p| residual(model, &cfw, p))
            .collect(),
    );
    Ok(Refined {
        world_from_cam: cfw.inverse(),
        before_px,
        after_px,
        pairs: pairs.len(),
        frames: sightings.len(),
    })
}

#[derive(Debug, Clone, PartialEq)]
pub struct Placed {
    pub world_from_cam: Isometry3<f64>,
    /// Labelled LED sightings used, of those offered, and their median residual (px).
    pub inliers: usize,
    pub offered: usize,
    pub median_px: f64,
}

/// A sensor with no pose yet, placed from the engine's LED labels on its blobs: each labelled
/// blob of a probe gives the LED's world position (the controller as the other sensors track it)
/// and the ray it was seen along. A robust direct linear fit on those rays, then Gauss-Newton on
/// the reprojection, gives a pose good enough for `refine_sensor`.
pub fn place_sensor(
    model: &CameraModel,
    bodies: [Option<&[Led]>; 2],
    probes: &[Probe],
    max_pos_sd: f64,
) -> Result<Placed, String> {
    // (LED in the world, blob, ray in the camera)
    let mut seen: Vec<(Vector3<f64>, Vector2<f64>, Vector3<f64>)> = Vec::new();
    for p in probes.iter().filter(|p| p.pos_sd <= max_pos_sd) {
        let Some(leds) = bodies[p.hand] else { continue };
        for (xy, &label) in p.blobs.iter().zip(&p.labels) {
            if label < 0 || label >> 8 != p.device_id {
                continue;
            }
            let (Some(l), Some(ray)) = (leds.get((label & 0xff) as usize), model.unproject(xy))
            else {
                continue;
            };
            seen.push((
                (p.world_from_model * Point3::from(l.pos)).coords,
                *xy,
                ray.normalize(),
            ));
        }
    }
    if seen.len() < 50 {
        return Err(format!("only {} labelled LED sightings", seen.len()));
    }
    let offered = seen.len();
    let reproj = |cfw: &Isometry3<f64>, s: &(Vector3<f64>, Vector2<f64>, Vector3<f64>)| -> f64 {
        let pc = cfw * Point3::from(s.0);
        if pc.z <= 0.0 {
            return f64::INFINITY;
        }
        model
            .project(&pc.coords)
            .map_or(f64::INFINITY, |uv| (uv - s.1).norm())
    };
    let inliers = |cfw: &Isometry3<f64>, gate: f64| -> Vec<usize> {
        (0..seen.len())
            .filter(|&i| reproj(cfw, &seen[i]) < gate)
            .collect()
    };
    // RANSAC over six-point fits (a fixed generator: the same recording places the same way)
    let mut state: u64 = 0x9e37_79b9_7f4a_7c15;
    let mut next = |n: usize| {
        state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((state >> 33) as usize) % n
    };
    let mut best: Option<(Vec<usize>, Isometry3<f64>)> = None;
    for _ in 0..2000 {
        let mut pick = [0usize; 6];
        for index in &mut pick {
            *index = next(seen.len());
        }
        let Some(cfw) = dlt(&pick
            .iter()
            .map(|&i| (seen[i].0, seen[i].2))
            .collect::<Vec<_>>())
        else {
            continue;
        };
        let inl = inliers(&cfw, 4.0);
        if best.as_ref().is_none_or(|b| inl.len() > b.0.len()) {
            best = Some((inl, cfw));
        }
    }
    let (mut inl, mut cfw) = best.ok_or("no pose fits the labelled sightings")?;
    if inl.len() < 30 {
        return Err(format!(
            "the best pose explains only {} of {} labelled sightings",
            inl.len(),
            offered
        ));
    }
    for gate in [4.0, 2.0, 2.0] {
        let pairs: Vec<(Vector3<f64>, Vector2<f64>)> =
            inl.iter().map(|&i| (seen[i].0, seen[i].1)).collect();
        cfw = solve(model, cfw, &pairs);
        inl = inliers(&cfw, gate);
    }
    let median_px = median(inl.iter().map(|&i| reproj(&cfw, &seen[i])).collect());
    Ok(Placed {
        world_from_cam: cfw.inverse(),
        inliers: inl.len(),
        offered,
        median_px,
    })
}

/// cam_from_world from (world point, unit ray in the camera) pairs, six or more: the rays must be
/// parallel to R X + t. Points normalised for conditioning; the rotation is the nearest proper one.
fn dlt(pairs: &[(Vector3<f64>, Vector3<f64>)]) -> Option<Isometry3<f64>> {
    use nalgebra::{Matrix3, SMatrix, SVector};
    let n = pairs.len() as f64;
    let c = pairs.iter().map(|p| p.0).sum::<Vector3<f64>>() / n;
    let sc = (pairs.iter().map(|p| (p.0 - c).norm()).sum::<f64>() / n).max(1e-9);
    let mut ata = SMatrix::<f64, 12, 12>::zeros();
    for (x, b) in pairs {
        let xs = (x - c) / sc;
        let xh = [xs.x, xs.y, xs.z, 1.0];
        let mut r1 = SVector::<f64, 12>::zeros();
        let mut r2 = SVector::<f64, 12>::zeros();
        for k in 0..4 {
            // b x (P X) = 0, rows one and two
            r1[4 + k] = -b.z * xh[k];
            r1[8 + k] = b.y * xh[k];
            r2[k] = b.z * xh[k];
            r2[8 + k] = -b.x * xh[k];
        }
        ata += r1 * r1.transpose() + r2 * r2.transpose();
    }
    let eig = ata.symmetric_eigen();
    let k = (0..12).min_by(|&a, &b| eig.eigenvalues[a].total_cmp(&eig.eigenvalues[b]))?;
    let h = eig.eigenvectors.column(k);
    let mut m = Matrix3::new(h[0], h[1], h[2], h[4], h[5], h[6], h[8], h[9], h[10]);
    let mut t = Vector3::new(h[3], h[7], h[11]);
    if m.determinant() < 0.0 {
        m = -m;
        t = -t;
    }
    let svd = m.svd(true, true);
    let (u, vt) = (svd.u?, svd.v_t?);
    let s = svd.singular_values.mean();
    if s <= 0.0 {
        return None;
    }
    let r = u * vt;
    if r.determinant() <= 0.0 {
        return None;
    }
    // undo the normalisation: R (X - c) / sc + t'/s = (R/sc) X + ...; scale back to metres
    let rot = UnitQuaternion::from_matrix(&r);
    let t_m = (t / s) * sc - r * c;
    Some(Isometry3::from_parts(Translation3::from(t_m), rot))
}

pub type RecordingModels = (
    std::collections::BTreeMap<String, CameraModel>,
    [Option<Vec<Led>>; 2],
);

/// Each sensor's camera model by serial and each hand's LEDs, from a recording.
pub fn recording_models(path: &std::path::Path) -> Result<RecordingModels, String> {
    use crate::recorder::{self, Reader};
    let mut cams = std::collections::BTreeMap::new();
    let mut leds: [Option<Vec<Led>>; 2] = [None, None];
    for rec in Reader::open(path)? {
        match rec.rtype {
            recorder::SENSOR_INFO => {
                if let Some(s) = recorder::decode_sensor_info(&rec.payload) {
                    cams.insert(
                        crate::engine::cstr_field(&s.serial),
                        CameraModel::from_sensor(&s),
                    );
                }
            }
            recorder::LED_MODEL => {
                if let Some((device, model)) = recorder::decode_led_model(&rec.payload) {
                    if let Some(hand) = super::pipeline::device_hand(device) {
                        leds[hand].get_or_insert_with(|| {
                            model
                                .iter()
                                .map(|l| Led {
                                    pos: crate::calib::vec3(l.pos),
                                    dir: crate::calib::vec3(l.dir),
                                })
                                .collect()
                        });
                    }
                }
            }
            _ => {}
        }
    }
    Ok((cams, leds))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::synth::{self, Rng};

    /// A sensor nudged 4 mm and 0.15 degrees: probes of a controller seen through its true pose
    /// bring it back.
    #[test]
    fn a_nudged_sensor_is_recovered() {
        let model = synth::camera_model();
        let truth = synth::look_at(
            Vector3::new(-2.0, 2.3, 1.8),
            Vector3::new(0.0, 1.0, 0.0),
            true,
        );
        let nudged = Isometry3::from_parts(
            Translation3::new(0.003, -0.002, 0.002),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.0015, -0.002, 0.001)),
        ) * truth;
        let leds = synth::touch_leds();
        let mut rng = Rng::new(3);
        let motion = synth::Motion::random(&mut rng, Vector3::new(0.0, 1.0, 0.0));
        let probes: Vec<Probe> = (0..600)
            .map(|k| {
                let w = motion.pose(k as f64 * 0.05);
                let blobs = leds
                    .iter()
                    .filter_map(|l| {
                        let pc = truth.inverse() * (w * Point3::from(l.pos));
                        let n = truth.inverse().rotation * (w.rotation * l.dir);
                        (n.dot(&(-pc.coords)) / pc.coords.norm() > 0.3)
                            .then(|| {
                                model
                                    .project(&pc.coords)
                                    .map(|uv| uv + Vector2::new(rng.gauss(), rng.gauss()) * 0.2)
                            })
                            .flatten()
                    })
                    .collect();
                Probe {
                    hand: 1,
                    device_id: 1,
                    world_from_model: w,
                    pos_sd: 5e-4,
                    blobs,
                    labels: Vec::new(),
                }
            })
            .collect();
        let r = refine_sensor(&model, &nudged, [None, Some(&leds)], &probes, 1e-3).unwrap();
        let d = r.world_from_cam.inverse() * truth;
        assert!(
            d.translation.vector.norm() < 5e-4 && d.rotation.angle() < 1e-4,
            "{:?} {:?}",
            d,
            r
        );
        assert!(r.after_px < 0.25 && r.before_px > 0.8, "{r:?}");
    }

    /// A sensor with no pose at all is placed from the engine's labels, though some are wrong and
    /// some blobs carry none, and then refined to well under a millimetre.
    #[test]
    fn an_unposed_sensor_is_placed_from_labels() {
        let model = synth::camera_model();
        let truth = synth::look_at(
            Vector3::new(1.9, 2.2, -1.6),
            Vector3::new(0.0, 1.0, 0.2),
            false,
        );
        let leds = synth::touch_leds();
        let mut rng = Rng::new(4);
        let motion = synth::Motion::random(&mut rng, Vector3::new(0.0, 1.0, 0.0));
        let probes: Vec<Probe> = (0..600)
            .map(|k| {
                let w = motion.pose(k as f64 * 0.05);
                let (mut blobs, mut labels) = (Vec::new(), Vec::new());
                for (i, l) in leds.iter().enumerate() {
                    let pc = truth.inverse() * (w * Point3::from(l.pos));
                    let n = truth.inverse().rotation * (w.rotation * l.dir);
                    if n.dot(&(-pc.coords)) / pc.coords.norm() <= 0.3 {
                        continue;
                    }
                    let Some(uv) = model.project(&pc.coords) else {
                        continue;
                    };
                    blobs.push(uv + Vector2::new(rng.gauss(), rng.gauss()) * 0.2);
                    let u = rng.uniform();
                    labels.push(if u < 0.05 {
                        (1 << 8) | ((i + 3) % leds.len()) as i32
                    } else if u < 0.25 {
                        -1
                    } else {
                        (1 << 8) | i as i32
                    });
                }
                Probe {
                    hand: 1,
                    device_id: 1,
                    world_from_model: w,
                    pos_sd: 5e-4,
                    blobs,
                    labels,
                }
            })
            .collect();
        let placed = place_sensor(&model, [None, Some(&leds)], &probes, 1e-3).unwrap();
        let d = placed.world_from_cam.inverse() * truth;
        assert!(
            d.translation.vector.norm() < 5e-3 && d.rotation.angle() < 0.3f64.to_radians(),
            "placed off by {:?}",
            d
        );
        let r = refine_sensor(
            &model,
            &placed.world_from_cam,
            [None, Some(&leds)],
            &probes,
            1e-3,
        )
        .unwrap();
        let d = r.world_from_cam.inverse() * truth;
        assert!(
            d.translation.vector.norm() < 5e-4 && d.rotation.angle() < 2e-4,
            "refined off by {:?}",
            d
        );
    }
}
