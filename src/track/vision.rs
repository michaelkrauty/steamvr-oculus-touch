// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! One sensor frame against one controller: LED projection with Jacobians, blob association by
//! Mahalanobis gating under the prior, and the robust update that fuses the matched blobs.

use nalgebra::{
    Isometry3, Matrix2, Matrix3, Point3, SMatrix, Translation3, Unit, UnitQuaternion, Vector2,
    Vector3,
};

use super::eskf::{skew, Filter, Nominal, PixelResidual, PoseJac, UpdateParams, Updated};
use crate::calib::camera::CameraModel;
use crate::calib::capture::Led;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Camera {
    pub model: CameraModel,
    pub world_from_cam: Isometry3<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Blob {
    pub xy: Vector2<f64>,
    /// The engine's label: (OpenHMD device id << 8) | LED index, or negative.
    pub led_id: i32,
}

/// A controller's LEDs and its IMU's position, in the LED model frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Body {
    pub leds: Vec<Led>,
    pub imu_pos: Vector3<f64>,
}

impl Body {
    /// world_from_model for a nominal state (whose p is the IMU's position).
    pub fn world_from_model(&self, x: &Nominal) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::from(x.p - x.q * self.imu_pos), x.q)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Projection {
    pub led: usize,
    pub uv: Vector2<f64>,
    pub j: PoseJac,
    /// Cosine between the LED's normal and the direction to the camera.
    pub facing: f64,
}

/// A camera-frame point's pixel position and its derivative (central differences).
pub fn project_with_jacobian(
    model: &CameraModel,
    pc: &Vector3<f64>,
) -> Option<(Vector2<f64>, SMatrix<f64, 2, 3>)> {
    let uv = model.project(pc)?;
    let h = 1e-6 * pc.norm().max(1e-3);
    let mut duv = SMatrix::<f64, 2, 3>::zeros();
    for k in 0..3 {
        let (mut a, mut b) = (*pc, *pc);
        a[k] += h;
        b[k] -= h;
        duv.set_column(k, &((model.project(&a)? - model.project(&b)?) / (2.0 * h)));
    }
    Some((uv, duv))
}

/// LED `i` in the image, with its Jacobian. None at or behind the camera plane.
pub fn project(cam: &Camera, body: &Body, x: &Nominal, i: usize) -> Option<Projection> {
    Projector::new(cam, body, x).project(i)
}

/// Invariant rotations for one state evaluation. Preserve per-LED multiplication
/// order rather than pre-multiplying the rotations and changing rounding.
struct Projector<'a> {
    cam: &'a Camera,
    body: &'a Body,
    x: &'a Nominal,
    r: Matrix3<f64>,
    rcw: Matrix3<f64>,
}

impl<'a> Projector<'a> {
    fn new(cam: &'a Camera, body: &'a Body, x: &'a Nominal) -> Self {
        Self {
            cam,
            body,
            x,
            r: x.q.to_rotation_matrix().into_inner(),
            rcw: cam
                .world_from_cam
                .rotation
                .inverse()
                .to_rotation_matrix()
                .into_inner(),
        }
    }
    /// Pixel-only callers do not need six extra fisheye projections or a Jacobian.
    /// Preserve the finite-difference routine's camera-plane rejection, including
    /// NaNs/infinities: CameraModel::project rejects only z <= 1e-6.
    fn pixel(&self, i: usize) -> Option<(Vector2<f64>, f64)> {
        let led = &self.body.leds[i];
        let arm = led.pos - self.body.imu_pos;
        let pc = self.rcw * (self.x.p + self.r * arm - self.cam.world_from_cam.translation.vector);
        let uv = self.cam.model.project(&pc)?;
        let h = 1e-6 * pc.norm().max(1e-3);
        if pc.z + h <= 1e-6 || pc.z - h <= 1e-6 {
            return None;
        }
        let facing = (self.rcw * (self.r * led.dir)).dot(&(-pc)) / pc.norm();
        Some((uv, facing))
    }

    fn project(&self, i: usize) -> Option<Projection> {
        let Self {
            cam,
            body,
            x,
            r,
            rcw,
        } = self;
        let led = &body.leds[i];
        let arm = led.pos - body.imu_pos;
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
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AssocParams {
    /// LEDs facing the camera less than this (cosine) are not expected to show.
    pub min_facing: f64,
    /// Mahalanobis^2 gate (2 dof).
    pub gate: f64,
    /// And a hard pixel cap.
    pub max_px: f64,
    /// LEDs predicted closer than this to another visible LED are skipped: their blobs merge.
    pub merge_px: f64,
    /// A pairing must beat any rival (another LED for its blob, another blob for its LED) by this
    /// much Mahalanobis^2.
    pub margin: f64,
}

impl Default for AssocParams {
    fn default() -> Self {
        AssocParams {
            min_facing: 0.2,
            gate: 13.8,
            max_px: 30.0,
            merge_px: 0.0,
            margin: 6.0,
        }
    }
}

/// Where the state puts its LEDs that face the camera, pixels.
pub fn predicted_pixels(
    cam: &Camera,
    body: &Body,
    x: &Nominal,
    min_facing: f64,
) -> Vec<Vector2<f64>> {
    let projector = Projector::new(cam, body, x);
    (0..body.leds.len())
        .filter_map(|i| projector.pixel(i))
        .filter(|(_, facing)| *facing > min_facing)
        .map(|(uv, _)| uv)
        .collect()
}

/// How many LEDs the state predicts in the image, facing the camera.
pub fn visible_leds(cam: &Camera, body: &Body, x: &Nominal, p: &AssocParams) -> usize {
    let projector = Projector::new(cam, body, x);
    let (w, h) = (cam.model.width as f64, cam.model.height as f64);
    (0..body.leds.len())
        .filter_map(|i| projector.pixel(i))
        .filter(|(uv, facing)| {
            *facing > p.min_facing && uv.x >= 0.0 && uv.y >= 0.0 && uv.x < w && uv.y < h
        })
        .count()
}

/// Diagnostics of one association: LEDs predicted visible, those with a blob within `near_px`,
/// and how many of them the association accepted.
pub fn association_report(
    cam: &Camera,
    body: &Body,
    f: &Filter,
    blobs: &[Blob],
    sigma_px: f64,
    p: &AssocParams,
    near_px: f64,
) -> (usize, usize, usize) {
    let projector = Projector::new(cam, body, &f.x);
    let (w, h) = (cam.model.width as f64, cam.model.height as f64);
    let visible: Vec<Projection> = (0..body.leds.len())
        .filter_map(|i| projector.project(i))
        .filter(|pr| {
            pr.facing > p.min_facing
                && pr.uv.x >= 0.0
                && pr.uv.y >= 0.0
                && pr.uv.x < w
                && pr.uv.y < h
        })
        .collect();
    let near = visible
        .iter()
        .filter(|pr| blobs.iter().any(|b| (b.xy - pr.uv).norm() <= near_px))
        .count();
    (
        visible.len(),
        near,
        associate_visible(&visible, f, blobs, sigma_px, p).len(),
    )
}

/// Visible LEDs and those with a nearby blob, without computing a matching.
pub fn visibility_counts(
    cam: &Camera,
    body: &Body,
    x: &Nominal,
    blobs: &[Blob],
    p: &AssocParams,
    near_px: f64,
) -> (usize, usize) {
    let projector = Projector::new(cam, body, x);
    let (w, h) = (cam.model.width as f64, cam.model.height as f64);
    (0..body.leds.len())
        .filter_map(|i| projector.pixel(i))
        .filter(|(uv, facing)| {
            *facing > p.min_facing && uv.x >= 0.0 && uv.y >= 0.0 && uv.x < w && uv.y < h
        })
        .fold((0, 0), |(visible, near), (uv, _)| {
            let nearby = blobs.iter().any(|b| (b.xy - uv).norm() <= near_px);
            (visible + 1, near + usize::from(nearby))
        })
}

/// Matches blobs to the LEDs the prior predicts, one to one: (LED, blob index) pairs.
pub fn associate(
    cam: &Camera,
    body: &Body,
    f: &Filter,
    blobs: &[Blob],
    sigma_px: f64,
    p: &AssocParams,
) -> Vec<(usize, usize)> {
    let projector = Projector::new(cam, body, &f.x);
    let (w, h) = (cam.model.width as f64, cam.model.height as f64);
    let visible: Vec<Projection> = (0..body.leds.len())
        .filter_map(|i| projector.project(i))
        .filter(|pr| {
            pr.facing > p.min_facing
                && pr.uv.x >= 0.0
                && pr.uv.y >= 0.0
                && pr.uv.x < w
                && pr.uv.y < h
        })
        .collect();
    associate_visible(&visible, f, blobs, sigma_px, p)
}

fn associate_visible(
    visible: &[Projection],
    f: &Filter,
    blobs: &[Blob],
    sigma_px: f64,
    p: &AssocParams,
) -> Vec<(usize, usize)> {
    let p6 = f.cov.fixed_view::<6, 6>(0, 0).into_owned();
    let var = Matrix2::identity() * (sigma_px * sigma_px);
    // (d2, visible index, blob)
    let mut cand: Vec<(f64, usize, usize)> = Vec::new();
    for (k, pr) in visible.iter().enumerate() {
        let Some(s_inv) = (pr.j * p6 * pr.j.transpose() + var).try_inverse() else {
            continue;
        };
        for (b, blob) in blobs.iter().enumerate() {
            let d = blob.xy - pr.uv;
            if d.norm() > p.max_px {
                continue;
            }
            let d2 = (d.transpose() * s_inv * d)[0];
            if d2 < p.gate {
                cand.push((d2, k, b));
            }
        }
    }
    cand.sort_by(|a, b| a.0.total_cmp(&b.0));
    let lone = |k: usize| {
        visible
            .iter()
            .enumerate()
            .all(|(m, o)| m == k || (o.uv - visible[k].uv).norm() > p.merge_px)
    };
    let (mut led_done, mut blob_done) = (vec![false; visible.len()], vec![false; blobs.len()]);
    let mut out = Vec::new();
    for &(d2, k, b) in &cand {
        if led_done[k] || blob_done[b] {
            continue;
        }
        led_done[k] = true;
        blob_done[b] = true;
        let rival = cand.iter().any(|&(e2, k2, b2)| {
            ((b2 == b && k2 != k) || (k2 == k && b2 != b)) && e2 < d2 + p.margin
        });
        if !rival && lone(k) {
            out.push((visible[k].led, b));
        }
    }
    out.sort();
    out
}

/// The given (LED, blob) pairs that the prior finds plausible: the LED faces the camera and its
/// blob lies within the gate.
pub fn gate_pairs(
    cam: &Camera,
    body: &Body,
    f: &Filter,
    blobs: &[Blob],
    pairs: &[(usize, usize)],
    sigma_px: f64,
    p: &AssocParams,
) -> Vec<(usize, usize)> {
    let projector = Projector::new(cam, body, &f.x);
    let p6 = f.cov.fixed_view::<6, 6>(0, 0).into_owned();
    let var = Matrix2::identity() * (sigma_px * sigma_px);
    pairs
        .iter()
        .copied()
        .filter(|&(led, b)| {
            let Some(pr) = projector.project(led) else {
                return false;
            };
            let Some(s_inv) = (pr.j * p6 * pr.j.transpose() + var).try_inverse() else {
                return false;
            };
            let d = blobs[b].xy - pr.uv;
            pr.facing > p.min_facing
                && d.norm() <= p.max_px
                && (d.transpose() * s_inv * d)[0] < p.gate
        })
        .collect()
}

/// `ours` plus every hint whose LED and blob `ours` leaves free.
pub fn merge_pairs(mut ours: Vec<(usize, usize)>, hints: &[(usize, usize)]) -> Vec<(usize, usize)> {
    for &(led, b) in hints {
        if ours.iter().all(|&(l, o)| l != led && o != b) {
            ours.push((led, b));
        }
    }
    ours.sort();
    ours
}

/// Pixel residuals of matched pairs at a nominal state.
pub fn residuals(
    cam: &Camera,
    body: &Body,
    blobs: &[Blob],
    pairs: &[(usize, usize)],
    x: &Nominal,
) -> Vec<PixelResidual> {
    let projector = Projector::new(cam, body, x);
    pairs
        .iter()
        .filter_map(|&(led, b)| {
            projector.project(led).map(|pr| PixelResidual {
                r: blobs[b].xy - pr.uv,
                j: pr.j,
            })
        })
        .collect()
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FitParams {
    pub update: UpdateParams,
    pub assoc: AssocParams,
    /// Fewest matched LEDs for an update.
    pub min_leds: usize,
    /// Post-fit residuals above this are outliers, refitted without.
    pub inlier_px: f64,
    /// Fraction of the matches that must be inliers.
    pub min_inlier_fraction: f64,
    /// The pose correction's Mahalanobis^2 under the prior (6 dof) may not exceed this: a fit
    /// that moves the state much further than the prior allows has matched the wrong LEDs.
    pub max_correction: f64,
    /// A fit leaving at least `min_unexplained_reject` unexplained blobs in the controller's
    /// footprint, and at least `unexplained_ratio` times as many as it matched, is a wrong pose
    /// that explains part of the constellation.
    pub min_unexplained_reject: usize,
    pub unexplained_ratio: f64,
    /// Shift search (see `associate_shifted`): snap radius, fewest LEDs, lead over the runner-up.
    pub snap_px: f64,
    pub shift_min_leds: usize,
    pub shift_margin: usize,
}

impl Default for FitParams {
    fn default() -> Self {
        FitParams {
            update: UpdateParams::default(),
            assoc: AssocParams::default(),
            min_leds: 3,
            inlier_px: 2.0,
            min_inlier_fraction: 0.7,
            max_correction: 50.0,
            min_unexplained_reject: 3,
            unexplained_ratio: 0.5,
            snap_px: 1.5,
            shift_min_leds: 4,
            shift_margin: 2,
        }
    }
}

/// Why a frame gave no update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Miss {
    /// Fewer matches than needed.
    TooFew,
    /// Too many matches disagree with the fit, or the filter could not update.
    Rejected,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Fit {
    pub updated: Updated,
    pub pairs: Vec<(usize, usize)>,
    /// Blobs inside the controller's projected footprint that no matched LED explains.
    pub unexplained: usize,
}

/// Blobs within `margin` px of the bounding box of the LEDs the state predicts facing the camera,
/// other than `used`.
pub fn unexplained_blobs(
    cam: &Camera,
    body: &Body,
    x: &Nominal,
    blobs: &[Blob],
    used: &[(usize, usize)],
    min_facing: f64,
    margin: f64,
) -> usize {
    let projector = Projector::new(cam, body, x);
    let mut uv = (0..body.leds.len())
        .filter_map(|i| projector.pixel(i))
        .filter(|(_, facing)| *facing > min_facing)
        .map(|(uv, _)| uv);
    let Some(first) = uv.next() else {
        return 0;
    };
    let (lo, hi) = std::iter::once(first)
        .chain(uv)
        .fold((first, first), |(lo, hi), p| (lo.inf(&p), hi.sup(&p)));
    blobs
        .iter()
        .enumerate()
        .filter(|(i, b)| {
            !used.iter().any(|u| u.1 == *i)
                && b.xy.x >= lo.x - margin
                && b.xy.y >= lo.y - margin
                && b.xy.x <= hi.x + margin
                && b.xy.y <= hi.y + margin
        })
        .count()
}

/// Association for an uncertain prior (just acquired from one view, or after an occlusion): its
/// error shows in this camera mostly as a shift of the whole predicted pattern, which a per-LED
/// gate cannot tell from a neighbour. Each (LED, blob) pair within the prior's 3-sigma region
/// proposes a shift; the shift that lands the most predictions within `snap_px` of a blob wins if
/// it clearly beats any other (by `margin` LEDs and at least `min_leds` of them).
// These inputs describe the prediction, observations and independent association gates.
#[allow(clippy::too_many_arguments)]
pub fn associate_shifted(
    cam: &Camera,
    body: &Body,
    f: &Filter,
    blobs: &[Blob],
    sigma_px: f64,
    p: &AssocParams,
    snap_px: f64,
    min_leds: usize,
    margin: usize,
) -> Vec<(usize, usize)> {
    let projector = Projector::new(cam, body, &f.x);
    let (w, h) = (cam.model.width as f64, cam.model.height as f64);
    let visible: Vec<Projection> = (0..body.leds.len())
        .filter_map(|i| projector.project(i))
        .filter(|pr| {
            pr.facing > p.min_facing
                && pr.uv.x >= -50.0
                && pr.uv.y >= -50.0
                && pr.uv.x < w + 50.0
                && pr.uv.y < h + 50.0
        })
        .collect();
    if visible.len() < min_leds || blobs.len() < min_leds {
        return Vec::new();
    }
    let p6 = f.cov.fixed_view::<6, 6>(0, 0).into_owned();
    // the pattern moves as a whole: the largest per-LED spread bounds the search
    let reach = visible
        .iter()
        .map(|pr| (pr.j * p6 * pr.j.transpose()).trace().sqrt())
        .fold(0.0, f64::max)
        * 3.0
        + 3.0 * sigma_px;
    let place = |d: &Vector2<f64>, out: &mut Vec<(usize, usize)>| {
        out.clear();
        for pr in &visible {
            let at = pr.uv + d;
            // Only a unique blob strictly inside snap_px is usable. A second
            // match proves ambiguity; sorting a wider neighbourhood adds nothing.
            let mut near = blobs
                .iter()
                .enumerate()
                .filter(|(_, x)| (x.xy - at).norm() < snap_px);
            if let Some((b, _)) = near.next() {
                if near.next().is_none() && !out.iter().any(|o: &(usize, usize)| o.1 == b) {
                    out.push((pr.led, b));
                }
            }
        }
    };
    let mut scratch = Vec::with_capacity(visible.len());
    let mut scored: Vec<(usize, Vector2<f64>)> = Vec::new();
    for pr in &visible {
        for b in blobs {
            let d = b.xy - pr.uv;
            if d.norm() <= reach.min(p.max_px * 3.0) {
                place(&d, &mut scratch);
                scored.push((scratch.len(), d));
            }
        }
    }
    // min_by_key returns the first tie, just like the old stable descending sort.
    let Some(&(best, d)) = scored.iter().min_by_key(|s| std::cmp::Reverse(s.0)) else {
        return Vec::new();
    };
    let rival = scored
        .iter()
        .filter(|s| (s.1 - d).norm() > 2.0 * snap_px)
        .map(|s| s.0)
        .max()
        .unwrap_or(0);
    if best < min_leds || best < rival + margin {
        return Vec::new();
    }
    place(&d, &mut scratch);
    scratch.sort();
    scratch
}

/// Associates, updates, re-associates under the tightened posterior, and updates again from the
/// prior with the final matches; outliers after the fit are dropped and the fit redone. `hints`
/// (the engine's labels) fill in what our own association leaves out, where the prior allows them:
/// they come from a whole-constellation fit, which tells apart neighbouring LEDs that a single
/// uncertain prediction cannot. None when too few LEDs match or too many disagree with the fit.
pub fn fit_frame(
    cam: &Camera,
    body: &Body,
    prior: &Filter,
    blobs: &[Blob],
    hints: &[(usize, usize)],
    p: &FitParams,
) -> Result<Fit, Miss> {
    let sigma = p.update.sigma_px;
    let hints = gate_pairs(cam, body, prior, blobs, hints, sigma, &p.assoc);
    let mut pairs = merge_pairs(associate(cam, body, prior, blobs, sigma, &p.assoc), &hints);
    if pairs.len() < p.min_leds {
        pairs = merge_pairs(
            associate_shifted(
                cam,
                body,
                prior,
                blobs,
                sigma,
                &p.assoc,
                p.snap_px,
                p.shift_min_leds,
                p.shift_margin,
            ),
            &hints,
        );
    }
    if pairs.len() < p.min_leds {
        return Err(Miss::TooFew);
    }
    let first = prior
        .update(|x| residuals(cam, body, blobs, &pairs, x), &p.update)
        .ok_or(Miss::Rejected)?;
    let again = merge_pairs(
        associate(cam, body, &first.filter, blobs, sigma, &p.assoc),
        &hints,
    );
    if again.len() > pairs.len() {
        pairs = again;
    }
    fit_pairs(cam, body, prior, blobs, pairs, p)
}

/// The update for given matches, with the outlier refit and the acceptance test.
pub fn fit_pairs(
    cam: &Camera,
    body: &Body,
    prior: &Filter,
    blobs: &[Blob],
    mut pairs: Vec<(usize, usize)>,
    p: &FitParams,
) -> Result<Fit, Miss> {
    if pairs.len() < p.min_leds {
        return Err(Miss::TooFew);
    }
    let mut updated = prior
        .update(|x| residuals(cam, body, blobs, &pairs, x), &p.update)
        .ok_or(Miss::Rejected)?;
    // per pair (a pair that no longer projects counts as an outlier, keeping pairs and residuals aligned)
    let pair_residual = |x: &Nominal, &(led, b): &(usize, usize)| {
        project(cam, body, x, led).map_or(f64::INFINITY, |pr| (blobs[b].xy - pr.uv).norm())
    };
    let original_len = pairs.len();
    pairs.retain(|pr| pair_residual(&updated.filter.x, pr) <= p.inlier_px);
    if pairs.len() < p.min_leds
        || (pairs.len() as f64) < p.min_inlier_fraction * original_len as f64
    {
        return Err(Miss::Rejected);
    }
    if pairs.len() < original_len {
        updated = prior
            .update(|x| residuals(cam, body, blobs, &pairs, x), &p.update)
            .ok_or(Miss::Rejected)?;
        if pairs
            .iter()
            .any(|pr| pair_residual(&updated.filter.x, pr) > p.inlier_px)
        {
            return Err(Miss::Rejected);
        }
    }
    let e = updated
        .filter
        .x
        .minus(&prior.x)
        .fixed_rows::<6>(0)
        .into_owned();
    let p6 = prior.cov.fixed_view::<6, 6>(0, 0).into_owned();
    if p6
        .cholesky()
        .is_none_or(|c| (e.transpose() * c.solve(&e))[0] > p.max_correction)
    {
        return Err(Miss::Rejected);
    }
    let unexplained = unexplained_blobs(
        cam,
        body,
        &updated.filter.x,
        blobs,
        &pairs,
        p.assoc.min_facing,
        3.0,
    );
    if unexplained >= p.min_unexplained_reject
        && unexplained as f64 >= p.unexplained_ratio * pairs.len() as f64
    {
        return Err(Miss::Rejected);
    }
    Ok(Fit {
        updated,
        pairs,
        unexplained,
    })
}

/// A turn of a controller's LED constellation about its ring axis that lands most of its LEDs on
/// others' spots: a pose turned by it explains most of the same blobs, and a single sensor's view
/// (or a labelling off by one around the ring) can take one for the other.
#[derive(Debug, Clone)]
pub struct Alias {
    /// The turn, in the LED model frame.
    pub turn: Isometry3<f64>,
    /// For each LED i, the other LED j the turn puts on i's spot, if any.
    pub led_at: Vec<Option<usize>>,
}

/// LED j lands on LED i's spot when the turn puts it within this distance, facing within
/// `ALIAS_FACING_RAD`.
const ALIAS_SPOT_M: f64 = 0.006;
const ALIAS_FACING_RAD: f64 = 0.52;

/// The constellation's aliases (see `Alias`): about the axis of the ring the LEDs lie on, the
/// turn each way (up to 90 degrees) that lands the most LEDs on others' spots, when that is at
/// least 40% of them. An exact symmetry (90% or more) is left out: nothing in a frame tells it
/// apart, so testing it gains nothing.
pub fn ring_aliases(leds: &[Led]) -> Vec<Alias> {
    let n = leds.len();
    if n < 5 {
        return Vec::new();
    }
    let c = leds.iter().map(|l| l.pos).sum::<Vector3<f64>>() / n as f64;
    let cov: Matrix3<f64> = leds
        .iter()
        .map(|l| (l.pos - c) * (l.pos - c).transpose())
        .sum();
    let e = cov.symmetric_eigen();
    let k = (0..3)
        .min_by(|&a, &b| e.eigenvalues[a].total_cmp(&e.eigenvalues[b]))
        .unwrap_or(0);
    let axis = Unit::new_normalize(e.eigenvectors.column(k).into_owned());
    let turn = |deg: f64| {
        Translation3::from(c)
            * UnitQuaternion::from_axis_angle(&axis, deg.to_radians())
            * Translation3::from(-c)
    };
    // each LED's spot and the LED landing nearest it, with the distance
    let landing = |t: &Isometry3<f64>| -> Vec<Option<(usize, f64)>> {
        leds.iter()
            .enumerate()
            .map(|(i, li)| {
                leds.iter()
                    .enumerate()
                    .filter(|&(j, lj)| {
                        j != i && (t.rotation * lj.dir).angle(&li.dir) < ALIAS_FACING_RAD
                    })
                    .map(|(j, lj)| (j, (t * Point3::from(lj.pos) - Point3::from(li.pos)).norm()))
                    .filter(|&(_, d)| d < ALIAS_SPOT_M)
                    .min_by(|a, b| a.1.total_cmp(&b.1))
            })
            .collect()
    };
    let mut out = Vec::new();
    for sign in [-1.0, 1.0] {
        // the most LEDs landed, then the closest landing
        let best = (5..=90)
            .map(|d| {
                let t = turn(sign * d as f64);
                let l = landing(&t);
                let count = l.iter().flatten().count();
                let spread = l.iter().flatten().map(|x| x.1).sum::<f64>() / count.max(1) as f64;
                (count, spread, t, l)
            })
            .max_by(|a, b| a.0.cmp(&b.0).then(b.1.total_cmp(&a.1)));
        if let Some((count, _, t, l)) = best {
            if count * 10 >= n * 4 && count * 10 < n * 9 {
                out.push(Alias {
                    turn: t,
                    led_at: l.iter().map(|x| x.map(|y| y.0)).collect(),
                });
            }
        }
    }
    out
}

#[cfg(test)]
pub(crate) mod tests {
    mod followup;
    pub(crate) mod hot_paths;
    mod shift;

    use super::*;
    use crate::calib::synth;
    use crate::track::eskf::{Cov, Nominal};
    use nalgebra::{Point3, UnitQuaternion};

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
        Body {
            leds: synth::touch_leds(),
            imu_pos: Vector3::new(0.01, -0.012, 0.004),
        }
    }

    fn state() -> Nominal {
        Nominal {
            q: UnitQuaternion::from_euler_angles(0.3, 0.5, -0.2),
            p: Vector3::new(0.05, 1.0, 0.1),
            v: Vector3::zeros(),
            bg: Vector3::zeros(),
            ba: Vector3::zeros(),
        }
    }

    fn filter(x: Nominal, rot_sd: f64, pos_sd: f64) -> Filter {
        let mut cov = Cov::identity() * 1e-4;
        for k in 0..3 {
            cov[(k, k)] = rot_sd * rot_sd;
            cov[(3 + k, 3 + k)] = pos_sd * pos_sd;
        }
        Filter { x, cov }
    }

    /// The blobs a camera sees of the body at `x`, LEDs facing it, in LED order.
    fn blobs(cam: &Camera, b: &Body, x: &Nominal) -> Vec<(usize, Blob)> {
        let w = b.world_from_model(x);
        (0..b.leds.len())
            .filter_map(|i| {
                let pc = cam.world_from_cam.inverse() * (w * Point3::from(b.leds[i].pos));
                let n = cam.world_from_cam.inverse().rotation * (w.rotation * b.leds[i].dir);
                (n.dot(&(-pc.coords)) / pc.coords.norm() > 0.2).then(|| {
                    (
                        i,
                        Blob {
                            xy: cam.model.project(&pc.coords).unwrap(),
                            led_id: -1,
                        },
                    )
                })
            })
            .collect()
    }

    #[test]
    fn the_jacobian_matches_finite_differences() {
        let (cam, b, x) = (camera(), body(), state());
        let pr = project(&cam, &b, &x, 3).unwrap();
        for k in 0..6 {
            let mut e = crate::track::eskf::ErrVec::zeros();
            e[k] = 1e-6;
            let up = project(&cam, &b, &x.plus(&e), 3).unwrap().uv;
            let num = (up - pr.uv) / 1e-6;
            assert!(
                (num - pr.j.column(k)).norm() < 1e-3 * (1.0 + num.norm()),
                "column {k}: {num:?} vs {:?}",
                pr.j.column(k)
            );
        }
    }

    #[test]
    fn blobs_of_a_perturbed_prior_match_their_leds_and_the_fit_recovers_the_pose() {
        let (cam, b, truth) = (camera(), body(), state());
        let seen = blobs(&cam, &b, &truth);
        assert!(seen.len() >= 6, "only {} LEDs visible", seen.len());
        let mut list: Vec<Blob> = seen.iter().map(|s| s.1).collect();
        list.push(Blob {
            xy: Vector2::new(100.0, 100.0),
            led_id: -1,
        }); // a stray light
            // a tracking prior: a millimetre and a few tenths of a degree off
        let prior_x = truth.plus(&crate::track::eskf::ErrVec::from_iterator([
            0.004, -0.003, 0.002, 0.001, -0.0008, 0.0012, 0., 0., 0., 0., 0., 0., 0., 0., 0.,
        ]));
        let prior = filter(prior_x, 0.005, 0.002);
        let pairs = associate(&cam, &b, &prior, &list, 0.5, &AssocParams::default());
        assert!(pairs.len() >= 5, "{pairs:?}");
        for &(led, blob) in &pairs {
            assert_eq!(
                seen.get(blob).map(|s| s.0),
                Some(led),
                "blob {blob} matched to the wrong LED"
            );
        }
        let fit = fit_frame(&cam, &b, &prior, &list, &[], &FitParams::default()).unwrap();
        // one view pins the position across the image well, depth and rotation (45 mm lever) less:
        // the error shrinks from the prior's without growing anywhere
        let (e, e0) = (fit.updated.filter.x.minus(&truth), prior_x.minus(&truth));
        assert!(
            e.fixed_rows::<3>(3).norm() < 0.6 * e0.fixed_rows::<3>(3).norm(),
            "{e:?}"
        );
        assert!(
            e.fixed_rows::<3>(0).norm() <= e0.fixed_rows::<3>(0).norm() * 1.05,
            "{e:?}"
        );
    }

    /// Centimetre uncertainty (just after acquisition) makes neighbouring LEDs ambiguous on their
    /// own; the engine's labels resolve them, and a wrong label the prior rules out is ignored.
    #[test]
    fn hints_resolve_an_uncertain_prior() {
        let (cam, b, truth) = (camera(), body(), state());
        let seen = blobs(&cam, &b, &truth);
        let list: Vec<Blob> = seen.iter().map(|s| s.1).collect();
        let prior_x = truth.plus(&crate::track::eskf::ErrVec::from_iterator([
            0.01, -0.008, 0.005, 0.006, -0.004, 0.005, 0., 0., 0., 0., 0., 0., 0., 0., 0.,
        ]));
        let prior = filter(prior_x, 0.02, 0.01);
        assert!(
            associate(&cam, &b, &prior, &list, 0.5, &AssocParams::default()).len() < 3,
            "expected ambiguity without hints"
        );
        let mut hints: Vec<(usize, usize)> =
            seen.iter().enumerate().map(|(i, s)| (s.0, i)).collect();
        let far = (0..b.leds.len())
            .find(|l| {
                seen.iter().all(|s| s.0 != *l)
                    && (b.leds[*l].pos - b.leds[seen[0].0].pos).norm() > 0.07
            })
            .unwrap();
        hints[0].0 = far; // the engine mislabels one blob as an LED on the far side
        let fit = fit_frame(&cam, &b, &prior, &list, &hints, &FitParams::default()).unwrap();
        assert!(
            fit.pairs.iter().all(|&(led, blob)| seen[blob].0 == led),
            "{:?}",
            fit.pairs
        );
        // one view fixes depth only to centimetres (the ring is ~25 px across), so check agreement
        // with every blob rather than the depth
        assert!(
            fit.pairs.len() >= seen.len() - 1 && fit.updated.residuals_px.iter().all(|&r| r < 0.3),
            "{:?}",
            fit.updated.residuals_px
        );
    }

    /// A prior 3 cm off along the view of another camera (a one-view acquisition): every LED is
    /// ambiguous on its own, the shift search finds the pattern and the fit pulls the state in.
    #[test]
    fn the_shift_search_relocks_a_prior_off_in_depth() {
        let (cam, b, truth) = (camera(), body(), state());
        let seen = blobs(&cam, &b, &truth);
        let list: Vec<Blob> = seen.iter().map(|s| s.1).collect();
        let lateral = cam.world_from_cam.rotation * Vector3::new(0.02, -0.015, 0.0);
        let prior_x = Nominal {
            p: truth.p + lateral,
            ..truth
        };
        let prior = filter(prior_x, 0.01, 0.03);
        assert!(associate(&cam, &b, &prior, &list, 0.5, &AssocParams::default()).len() < 3);
        let pairs = associate_shifted(
            &cam,
            &b,
            &prior,
            &list,
            0.5,
            &AssocParams::default(),
            1.5,
            4,
            2,
        );
        assert!(
            pairs.len() >= 5 && pairs.iter().all(|&(led, blob)| seen[blob].0 == led),
            "{pairs:?}"
        );
        let fit = fit_frame(&cam, &b, &prior, &list, &[], &FitParams::default()).unwrap();
        assert!(
            (fit.updated.filter.x.p - truth.p).norm() < 0.005,
            "{:?}",
            fit.updated.filter.x.p - truth.p
        );
    }

    #[test]
    fn the_touch_ring_aliases_one_led_step_either_way() {
        let (leds, _) = crate::calib::synth::cv1_touch_left();
        let a = ring_aliases(&leds);
        assert_eq!(
            a.len(),
            2,
            "{:?}",
            a.iter()
                .map(|a| a.turn.rotation.angle().to_degrees())
                .collect::<Vec<_>>()
        );
        for alias in &a {
            let deg = alias.turn.rotation.angle().to_degrees();
            assert!((22.0..34.0).contains(&deg), "{deg}");
            let landed = alias.led_at.iter().flatten().count();
            assert!((10..22).contains(&landed), "{landed}");
            // led_at[i] = j: the turn puts LED j on LED i's spot
            for (i, j) in alias
                .led_at
                .iter()
                .enumerate()
                .filter_map(|(i, j)| Some((i, (*j)?)))
            {
                assert!(
                    (alias.turn * nalgebra::Point3::from(leds[j].pos)
                        - nalgebra::Point3::from(leds[i].pos))
                    .norm()
                        < 0.006
                );
            }
        }
        assert!(
            (a[0].turn.rotation * a[1].turn.rotation).angle() < 3f64.to_radians(),
            "one each way"
        );
        // an exactly symmetric ring has no alias worth testing: nothing could tell the turns apart
        assert!(ring_aliases(&crate::calib::synth::touch_leds()).is_empty());
    }

    #[test]
    fn a_wrong_prior_matches_nothing_rather_than_the_wrong_leds() {
        let (cam, b, truth) = (camera(), body(), state());
        let list: Vec<Blob> = blobs(&cam, &b, &truth).iter().map(|s| s.1).collect();
        // rotated 15 degrees about the ring axis: every LED predicted near a neighbour's blob
        let wrong = Nominal {
            q: truth.q
                * UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 15f64.to_radians(), 0.0)),
            ..truth
        };
        let fit = fit_frame(
            &cam,
            &b,
            &filter(wrong, 0.003, 0.002),
            &list,
            &[],
            &FitParams::default(),
        )
        .ok();
        assert!(
            fit.is_none()
                || fit
                    .unwrap()
                    .updated
                    .filter
                    .x
                    .minus(&truth)
                    .fixed_rows::<3>(0)
                    .norm()
                    < 0.01
        );
    }
}
