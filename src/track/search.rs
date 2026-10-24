// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! A controller's position in one frame from its orientation alone, for acquisition without the
//! engine. After a loss the gyro still knows how the controller is turned long after its position
//! is gone, and with the turn known two blobs seen as two LEDs place it: each LED lies on its
//! blob's ray, and the two lie the turned LED offset apart. Every such pairing is tried, and each
//! position found is scored by how many of the LEDs it shows the camera land on a blob. The
//! engine's brute-force search, which also has to find the turn, can take seconds to offer a pose.

use std::collections::HashSet;

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

use super::vision::{Blob, Body, Camera};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SearchParams {
    /// A blob this close to a predicted LED explains it (px).
    pub inlier_px: f64,
    /// Fewest LEDs a found position must explain, and the smallest share of the LEDs it shows the
    /// camera.
    pub min_leds: usize,
    pub min_share: f64,
    /// The most candidates returned, each pairing the blobs differently (sharing under half of
    /// its pairs with any before it).
    pub max_candidates: usize,
    /// Fitted, a candidate is taken only with this many LEDs paired, this share of the LEDs it
    /// shows the camera, its pairs reprojected this closely (RMS px), and at most this many blobs
    /// in its footprint unexplained (see `Tracker::explains_frame`): nothing else backs a
    /// position found this way.
    pub min_fit_leds: usize,
    pub min_fit_share: f64,
    pub max_fit_rms_px: f64,
    pub max_unexplained: usize,
    /// A position found this way is published once a second sensor's frame confirms it, and
    /// dropped if none has within this long (ns): one view can be fitted by a wrong pose. A
    /// strong engine pose confirms it too when within `acquire_max_turn_rad` and this distance
    /// (m) of it, and replaces it otherwise.
    pub confirm_within_ns: u64,
    pub engine_agree_m: f64,
    /// The search runs on the radio thread, so its cost is bounded: each sensor's frames are
    /// searched at most this often (ns), and a frame with more blobs than this not at all (the
    /// work grows with the square of the blobs).
    pub every_ns: u64,
    pub max_blobs: usize,
    /// And each search scores at most this many positions; it ends at the first that explains
    /// the frame nearly whole and closely (`min_fit_leds`, `min_fit_share`, twice
    /// `max_fit_rms_px` before any fit), which the tracker still fits and a second sensor still
    /// has to confirm.
    pub max_scored: usize,
    /// LEDs facing the camera less than this (cosine) are not expected to show.
    pub min_facing: f64,
    /// Depths along a blob's ray the controller can be at (m).
    pub depth_m: (f64, f64),
    /// How far apart the two LEDs' rays may leave them from the turned offset between them: this
    /// much, plus this share of the offset (an orientation off by a few degrees).
    pub slack_m: f64,
    pub slack_share: f64,
}

impl Default for SearchParams {
    fn default() -> Self {
        SearchParams {
            inlier_px: 4.0,
            min_leds: 5,
            min_share: 0.34,
            max_candidates: 4,
            min_fit_leds: 6,
            min_fit_share: 0.8,
            max_fit_rms_px: 0.6,
            max_unexplained: 1,
            confirm_within_ns: 100_000_000,
            engine_agree_m: 0.05,
            every_ns: 60_000_000,
            max_blobs: 24,
            max_scored: 200,
            min_facing: 0.2,
            depth_m: (0.15, 6.0),
            slack_m: 0.004,
            slack_share: 0.12,
        }
    }
}

/// A position the frame supports: the controller's pose, with its centre placed closest to the
/// rays of the blobs it explains, and the (LED, blob index) pairs.
#[derive(Debug, Clone, PartialEq)]
pub struct Candidate {
    pub world_from_model: Isometry3<f64>,
    pub pairs: Vec<(usize, usize)>,
}

/// One scored position.
struct Scored {
    pairs: Vec<(usize, usize)>,
    /// The LEDs it shows the camera.
    shown: usize,
    /// Sum of squared pixel distances of the pairs, to tell equal counts apart.
    sq_px: f64,
}

/// Where the controller turned by `q` (world from model) may be in this frame, from `blobs[i]`
/// for each `i` in `candidates` (the blobs that may be its LEDs): the positions explaining most
/// LEDs first. Telling the right one from a near miss takes residuals after a fit that also
/// corrects the orientation (see `Tracker::explains_frame`); none means no position explains
/// enough.
pub fn search_position(
    cam: &Camera,
    body: &Body,
    q: &UnitQuaternion<f64>,
    blobs: &[Blob],
    candidates: &[usize],
    p: &SearchParams,
) -> Vec<Candidate> {
    search(cam, body, q, blobs, candidates, p).0
}

/// `search_position`, and how many positions it scored.
fn search(
    cam: &Camera,
    body: &Body,
    q: &UnitQuaternion<f64>,
    blobs: &[Blob],
    candidates: &[usize],
    p: &SearchParams,
) -> (Vec<Candidate>, usize) {
    if candidates.len() < p.min_leds {
        return (Vec::new(), 0);
    }
    let centre = cam.world_from_cam.translation.vector;
    let rays: Vec<(usize, Vector3<f64>)> = candidates
        .iter()
        .filter_map(|&b| {
            let d = cam.model.unproject(&blobs[b].xy)?;
            Some((b, cam.world_from_cam.rotation * d))
        })
        .collect();
    let offsets: Vec<Vector3<f64>> = body.leds.iter().map(|l| q * l.pos).collect();
    let normals: Vec<Vector3<f64>> = body.leds.iter().map(|l| q * l.dir).collect();
    // per ray, the LEDs that could be at its far end: those facing back along it
    let facing: Vec<Vec<usize>> = rays
        .iter()
        .map(|(_, d)| {
            (0..offsets.len())
                .filter(|&i| normals[i].dot(&-d) > p.min_facing)
                .collect()
        })
        .collect();
    let mut tried: HashSet<[i64; 3]> = HashSet::new();
    let mut scored: Vec<Scored> = Vec::new();
    let mut positions = 0;
    'search: for a in 0..rays.len() {
        for b in a + 1..rays.len() {
            let (da, db) = (rays[a].1, rays[b].1);
            let c = da.dot(&db);
            let det = 1.0 - c * c;
            if det < 1e-12 {
                continue;
            }
            for &i in &facing[a] {
                for &j in &facing[b] {
                    if i == j {
                        continue;
                    }
                    // LED i at centre + la da, LED j at centre + lb db: la da - lb db = e
                    let e = offsets[i] - offsets[j];
                    let (ae, be) = (da.dot(&e), db.dot(&e));
                    let la = (ae - c * be) / det;
                    let lb = (c * ae - be) / det;
                    if la < p.depth_m.0 || la > p.depth_m.1 || lb < p.depth_m.0 || lb > p.depth_m.1
                    {
                        continue;
                    }
                    let miss = (da * la - db * lb - e).norm();
                    if miss > p.slack_m + p.slack_share * e.norm() {
                        continue;
                    }
                    let t = (centre + da * la - offsets[i] + centre + db * lb - offsets[j]) / 2.0;
                    // a centimetre grid: the true position comes from many pairings
                    let cell = [
                        (t.x * 100.0).round() as i64,
                        (t.y * 100.0).round() as i64,
                        (t.z * 100.0).round() as i64,
                    ];
                    if !tried.insert(cell) {
                        continue;
                    }
                    if tried.len() > p.max_scored {
                        break 'search;
                    }
                    positions += 1;
                    if let Some(s) = score(cam, &offsets, &normals, &t, blobs, &rays, p) {
                        // a position that explains the frame nearly whole ends the search
                        let n = s.pairs.len() as f64;
                        let whole = s.pairs.len() >= p.min_fit_leds
                            && n >= p.min_fit_share * s.shown as f64
                            && s.sq_px / n <= (2.0 * p.max_fit_rms_px).powi(2);
                        scored.push(s);
                        if whole {
                            break 'search;
                        }
                    }
                }
            }
        }
    }
    // most LEDs explained first; the same pairs at another depth are the same answer (one camera
    // barely sees depth), so each candidate pairs the blobs differently from those before it
    scored.sort_by(|x, y| {
        y.pairs
            .len()
            .cmp(&x.pairs.len())
            .then(x.sq_px.total_cmp(&y.sq_px))
    });
    let mut out: Vec<Candidate> = Vec::new();
    for s in scored {
        if out.len() == p.max_candidates {
            break;
        }
        if out
            .iter()
            .any(|c| shared(&c.pairs, &s.pairs) * 2 >= s.pairs.len())
        {
            continue;
        }
        let on_rays: Option<Vec<(Vector3<f64>, Vector3<f64>)>> = s
            .pairs
            .iter()
            .map(|&(k, b)| Some((offsets[k], rays.iter().find(|r| r.0 == b)?.1)))
            .collect();
        if let Some(t) = on_rays.and_then(|r| closest_to_rays(&centre, &r)) {
            out.push(Candidate {
                world_from_model: Isometry3::from_parts(Translation3::from(t), *q),
                pairs: s.pairs,
            });
        }
    }
    (out, positions)
}

/// How many (LED, blob) pairs two pairings share.
fn shared(a: &[(usize, usize)], b: &[(usize, usize)]) -> usize {
    a.iter().filter(|x| b.contains(x)).count()
}

/// The model origin closest, in the least-squares sense, to putting each LED offset `o` (world
/// frame) on its ray from `centre` along the unit `d`: sum (I - d d') (t + o - centre) = 0.
fn closest_to_rays(
    centre: &Vector3<f64>,
    rays: &[(Vector3<f64>, Vector3<f64>)],
) -> Option<Vector3<f64>> {
    let mut a = nalgebra::Matrix3::zeros();
    let mut rhs = Vector3::zeros();
    for (o, d) in rays {
        let across = nalgebra::Matrix3::identity() - d * d.transpose();
        a += across;
        rhs += across * (centre - o);
    }
    a.try_inverse().map(|inv| inv * rhs)
}

/// The LEDs a position `t` shows the camera, each paired with the nearest unclaimed blob within
/// `inlier_px` (nearest pairings first). None when too few land on blobs, or too small a share.
fn score(
    cam: &Camera,
    offsets: &[Vector3<f64>],
    normals: &[Vector3<f64>],
    t: &Vector3<f64>,
    blobs: &[Blob],
    rays: &[(usize, Vector3<f64>)],
    p: &SearchParams,
) -> Option<Scored> {
    let centre = cam.world_from_cam.translation.vector;
    let cam_from_world = cam.world_from_cam.rotation.inverse();
    let mut near: Vec<(f64, usize, usize)> = Vec::new();
    let mut shown = 0;
    for (k, (o, n)) in offsets.iter().zip(normals).enumerate() {
        let w = t + o;
        let to_cam = centre - w;
        if n.dot(&to_cam) <= p.min_facing * to_cam.norm() {
            continue;
        }
        let Some(uv) = cam.model.project(&(cam_from_world * -to_cam)) else {
            continue;
        };
        if uv.x < 0.0
            || uv.y < 0.0
            || uv.x >= cam.model.width as f64
            || uv.y >= cam.model.height as f64
        {
            continue;
        }
        shown += 1;
        let nearest = rays
            .iter()
            .map(|&(b, _)| ((blobs[b].xy - uv).norm_squared(), b))
            .min_by(|x, y| x.0.total_cmp(&y.0));
        if let Some((d2, b)) = nearest.filter(|(d2, _)| *d2 <= p.inlier_px * p.inlier_px) {
            near.push((d2, k, b));
        }
    }
    if near.len() < p.min_leds {
        return None;
    }
    near.sort_by(|x, y| x.0.total_cmp(&y.0));
    let (mut leds, mut used) = (HashSet::new(), HashSet::new());
    let mut pairs = Vec::new();
    let mut sq_px = 0.0;
    for (d2, k, b) in near {
        if leds.insert(k) && used.insert(b) {
            pairs.push((k, b));
            sq_px += d2;
        }
    }
    (pairs.len() >= p.min_leds && pairs.len() as f64 >= p.min_share * shown as f64).then_some(
        Scored {
            pairs,
            shown,
            sq_px,
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::synth::{self, Rng};
    use nalgebra::Vector2;

    fn camera() -> Camera {
        Camera {
            model: synth::camera_model(),
            world_from_cam: synth::look_at(
                Vector3::new(0.0, 2.0, 2.5),
                Vector3::new(0.0, 1.0, 0.0),
                false,
            ),
        }
    }

    fn body() -> Body {
        let (leds, imu_pos) = synth::cv1_touch_left();
        Body { leds, imu_pos }
    }

    /// The blobs a controller at `pose` makes (facing LEDs, with pixel noise), then `clutter`
    /// stray blobs anywhere, shuffled; and each blob's LED (None for clutter).
    fn sight(
        cam: &Camera,
        body: &Body,
        pose: &Isometry3<f64>,
        clutter: usize,
        rng: &mut Rng,
    ) -> (Vec<Blob>, Vec<Option<usize>>) {
        let cam_from_world = cam.world_from_cam.inverse();
        let mut out: Vec<(Blob, Option<usize>)> = Vec::new();
        for (k, l) in body.leds.iter().enumerate() {
            let w = pose * nalgebra::Point3::from(l.pos);
            let n = pose.rotation * l.dir;
            let to_cam = cam.world_from_cam.translation.vector - w.coords;
            if n.dot(&to_cam) <= 0.3 * to_cam.norm() {
                continue;
            }
            if let Some(uv) = cam.model.project(&(cam_from_world * w).coords) {
                let xy = uv + Vector2::new(rng.gauss(), rng.gauss()) * 0.3;
                out.push((Blob { xy, led_id: -1 }, Some(k)));
            }
        }
        for _ in 0..clutter {
            let xy = Vector2::new(rng.range(0.0, 1280.0), rng.range(0.0, 960.0));
            out.push((Blob { xy, led_id: -1 }, None));
        }
        for i in (1..out.len()).rev() {
            out.swap(i, (rng.uniform() * (i + 1) as f64) as usize % (i + 1));
        }
        out.into_iter().unzip()
    }

    /// How far `found` is from `truth` across the camera's line of sight, and along it (one
    /// camera measures depth from the LEDs' spread alone: centimetres at a few metres).
    fn error(cam: &Camera, found: &Isometry3<f64>, truth: &Isometry3<f64>) -> (f64, f64) {
        let sight = (truth.translation.vector - cam.world_from_cam.translation.vector).normalize();
        let e = found.translation.vector - truth.translation.vector;
        let along = e.dot(&sight);
        ((e - sight * along).norm(), along.abs())
    }

    fn pose(seed: u64) -> Isometry3<f64> {
        let mut rng = Rng::new(seed);
        Isometry3::from_parts(
            Translation3::new(
                rng.range(-0.4, 0.4),
                rng.range(0.8, 1.5),
                rng.range(-0.3, 0.4),
            ),
            UnitQuaternion::from_scaled_axis(rng.gauss3(1.2)),
        )
    }

    /// The candidate for the truth: within `across` m of it across the line of sight, and with
    /// at least nine in ten of its pairs right. Its index among the candidates.
    fn truth_among(
        cam: &Camera,
        found: &[Candidate],
        truth: &Isometry3<f64>,
        leds: &[Option<usize>],
        across: f64,
    ) -> Option<usize> {
        found.iter().position(|c| {
            let right = c
                .pairs
                .iter()
                .filter(|(led, b)| leds[*b] == Some(*led))
                .count();
            error(cam, &c.world_from_model, truth).0 < across && right * 10 >= c.pairs.len() * 9
        })
    }

    #[test]
    fn the_orientation_places_the_controller_and_names_its_leds() {
        let (cam, body) = (camera(), body());
        let (mut tried, mut first) = (0, 0);
        for seed in 0..60 {
            let truth = pose(seed);
            let mut rng = Rng::new(1000 + seed);
            let (blobs, leds) = sight(&cam, &body, &truth, 8, &mut rng);
            if leds.iter().flatten().count() < 6 {
                continue;
            }
            tried += 1;
            let all: Vec<usize> = (0..blobs.len()).collect();
            let found = search_position(
                &cam,
                &body,
                &truth.rotation,
                &blobs,
                &all,
                &SearchParams::default(),
            );
            let at = truth_among(&cam, &found, &truth, &leds, 0.005);
            let c = &found[at.unwrap_or_else(|| panic!("seed {seed}: not among {found:?}"))];
            // depth from one camera 2.7 m away and a few LEDs on one side: centimetres, which the
            // other sensors settle once the track runs
            let along = error(&cam, &c.world_from_model, &truth).1;
            assert!(
                along < 0.1,
                "seed {seed}: {along} m along, {} pairs",
                c.pairs.len()
            );
            first += usize::from(at == Some(0));
        }
        assert!(
            tried >= 30 && first * 10 >= tried * 9,
            "first in {first} of {tried}"
        );
    }

    /// The gyro's orientation a few degrees off still offers the truth, close enough for a fit
    /// that also corrects the orientation to finish.
    #[test]
    fn an_orientation_a_few_degrees_off_still_offers_the_truth() {
        let (cam, body) = (camera(), body());
        let (mut tried, mut offered) = (0, 0);
        for seed in 0..80 {
            let truth = pose(seed);
            let mut rng = Rng::new(2000 + seed);
            let (blobs, leds) = sight(&cam, &body, &truth, 8, &mut rng);
            if leds.iter().flatten().count() < 8 {
                continue;
            }
            tried += 1;
            let off =
                UnitQuaternion::from_scaled_axis(rng.gauss3(1.0).normalize() * 4f64.to_radians());
            let all: Vec<usize> = (0..blobs.len()).collect();
            let found = search_position(
                &cam,
                &body,
                &(off * truth.rotation),
                &blobs,
                &all,
                &SearchParams::default(),
            );
            offered += usize::from(truth_among(&cam, &found, &truth, &leds, 0.02).is_some());
        }
        assert!(
            tried >= 25 && offered * 10 >= tried * 9,
            "offered in {offered} of {tried}"
        );
    }

    /// A search's cost is bounded: it ends at the first position that explains the frame nearly
    /// whole, and scores no more than `max_scored` positions in a crowded frame.
    #[test]
    fn a_search_scores_a_bounded_number_of_positions() {
        let (cam, body) = (camera(), body());
        let p = SearchParams::default();
        let truth = pose(0);
        let mut rng = Rng::new(0);
        let (blobs, _) = sight(&cam, &body, &truth, 0, &mut rng);
        let all: Vec<usize> = (0..blobs.len()).collect();
        let (found, scored) = search(&cam, &body, &truth.rotation, &blobs, &all, &p);
        assert!(!found.is_empty() && scored < 50, "{scored} scored");
        // 40 stray blobs: some hundreds of positions to score without the bound
        let (strays, _) = sight(
            &cam,
            &body,
            &Isometry3::translation(0.0, -50.0, 0.0),
            40,
            &mut rng,
        );
        let all: Vec<usize> = (0..strays.len()).collect();
        let (_, scored) = search(&cam, &body, &truth.rotation, &strays, &all, &p);
        assert_eq!(scored, p.max_scored);
    }

    /// Stray blobs alone, or too few of the controller's, offer nothing.
    #[test]
    fn clutter_or_too_few_leds_offer_nothing() {
        let (cam, body) = (camera(), body());
        let mut rng = Rng::new(7);
        let q = pose(3).rotation;
        let (clutter, _) = sight(
            &cam,
            &body,
            &Isometry3::translation(0.0, -50.0, 0.0),
            30,
            &mut rng,
        );
        let all: Vec<usize> = (0..clutter.len()).collect();
        assert_eq!(
            search_position(&cam, &body, &q, &clutter, &all, &SearchParams::default()),
            vec![]
        );

        let truth = pose(3);
        let (blobs, leds) = sight(&cam, &body, &truth, 0, &mut rng);
        let four: Vec<usize> = (0..blobs.len())
            .filter(|&b| leds[b].is_some())
            .take(4)
            .collect();
        assert_eq!(
            search_position(
                &cam,
                &body,
                &truth.rotation,
                &blobs,
                &four,
                &SearchParams::default()
            ),
            vec![]
        );
    }
}
