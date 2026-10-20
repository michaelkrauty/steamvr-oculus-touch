// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Accuracy and latency against Lighthouse truth. A Touch is fixed to an Index controller (a jig)
//! and the Index pose is sampled from SteamVR at high rate while the daemon records. The poses the
//! daemon published are compared with that truth carried into the cv1touch world:
//! touch_truth(t) = world_from_lh * index_raw(t) * ref_from_touch.

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

use crate::calib::iso_from_xyzw;
use crate::shm::ControllerState;

#[derive(Debug, Clone, Copy)]
pub struct TruthSample {
    pub t_ns: u64,
    /// The reference controller's raw-universe pose.
    pub pose: Isometry3<f64>,
}

/// A pose the daemon published, at the time it refers to (`sample_host_ns`), with its velocities
/// in the world frame.
#[derive(Debug, Clone, Copy)]
pub struct Published {
    pub t_ns: u64,
    pub pose: Isometry3<f64>,
    pub vel: Vector3<f64>,
    pub ang_vel_world: Vector3<f64>,
}

pub fn published_from_state(st: &ControllerState) -> Published {
    let pose = iso_from_xyzw(st.orient, st.pos);
    let v = |a: [f32; 3]| Vector3::new(a[0] as f64, a[1] as f64, a[2] as f64);
    Published {
        t_ns: st.sample_host_ns,
        pose,
        vel: v(st.lin_vel),
        ang_vel_world: pose.rotation * (v(st.ang_vel) - v(st.gyro_bias)),
    }
}

/// Truth pose at `t_ns`: linear in position, slerp in rotation between the bracketing samples.
pub fn interpolate(truth: &[TruthSample], t_ns: u64) -> Option<Isometry3<f64>> {
    let k = truth.partition_point(|s| s.t_ns < t_ns);
    if k == 0 || k == truth.len() {
        return (k < truth.len() && truth[k].t_ns == t_ns).then(|| truth[k].pose);
    }
    let (a, b) = (&truth[k - 1], &truth[k]);
    if b.t_ns - a.t_ns > 50_000_000 {
        return None; // a tracking gap: no truth
    }
    let f = (t_ns - a.t_ns) as f64 / (b.t_ns - a.t_ns) as f64;
    let p = a
        .pose
        .translation
        .vector
        .lerp(&b.pose.translation.vector, f);
    let r = a.pose.rotation.slerp(&b.pose.rotation, f);
    Some(Isometry3::from_parts(Translation3::from(p), r))
}

/// A stretch of time over which the Touch sat still on the jig: the held offset then.
#[derive(Debug, Clone, Copy)]
pub struct Segment {
    pub start_ns: u64,
    pub end_ns: u64,
    pub ref_from_touch: Isometry3<f64>,
}

pub struct Truth<'a> {
    pub samples: &'a [TruthSample],
    pub world_from_lh: Isometry3<f64>,
    /// The held offset over time (a jig can slip); no truth outside them.
    pub segments: Vec<Segment>,
}

impl<'a> Truth<'a> {
    pub fn single(
        samples: &'a [TruthSample],
        world_from_lh: Isometry3<f64>,
        ref_from_touch: Isometry3<f64>,
    ) -> Truth<'a> {
        Truth {
            samples,
            world_from_lh,
            segments: vec![Segment {
                start_ns: 0,
                end_ns: u64::MAX,
                ref_from_touch,
            }],
        }
    }

    /// The Touch's true world pose at `t_ns` (signed, for shifted comparisons).
    pub fn at(&self, t_ns: i64) -> Option<Isometry3<f64>> {
        let t = u64::try_from(t_ns).ok()?;
        let seg = self
            .segments
            .iter()
            .find(|s| t >= s.start_ns && t <= s.end_ns)?;
        Some(self.world_from_lh * interpolate(self.samples, t)? * seg.ref_from_touch)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SegmentParams {
    /// Offsets are compared as medians over blocks of this length.
    pub block_ns: u64,
    /// A block whose rotation offset is this far from its segment's, followed by another that
    /// agrees with it, starts a new segment. Position is not used: centimetre-level tracking error
    /// wanders the translation of every block, while the orientation is precise.
    pub tol_rad: f64,
    /// Shorter segments (a slip in progress, a re-seat) are dropped.
    pub min_ns: u64,
}

impl Default for SegmentParams {
    fn default() -> Self {
        SegmentParams {
            block_ns: 1_000_000_000,
            tol_rad: 3f64.to_radians(),
            min_ns: 4_000_000_000,
        }
    }
}

fn median_translation(ys: &[Isometry3<f64>]) -> Vector3<f64> {
    let mut c = [Vec::new(), Vec::new(), Vec::new()];
    for y in ys {
        for (i, component) in c.iter_mut().enumerate() {
            component.push(y.translation.vector[i]);
        }
    }
    Vector3::new(
        quantile(&mut c[0], 0.5),
        quantile(&mut c[1], 0.5),
        quantile(&mut c[2], 0.5),
    )
}

fn robust_offset(ys: &[Isometry3<f64>]) -> Isometry3<f64> {
    let r = crate::calib::motion::chordal_mean(&ys.iter().map(|y| y.rotation).collect::<Vec<_>>());
    Isometry3::from_parts(Translation3::from(median_translation(ys)), r)
}

/// The held offset Y_i = index_raw(t - lag)^-1 * lighthouse_from_world * touch_i, split wherever
/// it jumps (a jig slipping or being re-seated).
pub fn offset_segments(
    published: &[Published],
    truth: &[TruthSample],
    lighthouse_from_world: &Isometry3<f64>,
    lag_ns: i64,
    p: &SegmentParams,
) -> Vec<Segment> {
    let ys: Vec<(u64, Isometry3<f64>)> = published
        .iter()
        .filter_map(|q| {
            let l = interpolate(truth, u64::try_from(q.t_ns as i64 - lag_ns).ok()?)?;
            Some((q.t_ns, l.inverse() * lighthouse_from_world * q.pose))
        })
        .collect();
    // block medians
    type OffsetBlock = (u64, u64, Isometry3<f64>, Vec<Isometry3<f64>>);
    let mut blocks: Vec<OffsetBlock> = Vec::new();
    let mut i = 0;
    while i < ys.len() {
        let b = ys[i].0 / p.block_ns;
        let mut j = i;
        while j < ys.len() && ys[j].0 / p.block_ns == b {
            j += 1;
        }
        let members: Vec<Isometry3<f64>> = ys[i..j].iter().map(|y| y.1).collect();
        blocks.push((ys[i].0, ys[j - 1].0, robust_offset(&members), members));
        i = j;
    }
    let mut segments = Vec::new();
    let mut cur: Vec<usize> = Vec::new();
    let close = |cur: &[usize], segments: &mut Vec<Segment>| {
        let (Some(&a), Some(&b)) = (cur.first(), cur.last()) else {
            return;
        };
        if blocks[b].1 - blocks[a].0 < p.min_ns {
            return;
        }
        let all: Vec<Isometry3<f64>> = cur
            .iter()
            .flat_map(|&k| blocks[k].3.iter().copied())
            .collect();
        segments.push(Segment {
            start_ns: blocks[a].0,
            end_ns: blocks[b].1,
            ref_from_touch: robust_offset(&all),
        });
    };
    let far = |a: &Isometry3<f64>, b: &Isometry3<f64>| a.rotation.angle_to(&b.rotation) > p.tol_rad;
    for k in 0..blocks.len() {
        if !cur.is_empty() {
            let reference = robust_offset(&cur.iter().map(|&c| blocks[c].2).collect::<Vec<_>>());
            if far(&blocks[k].2, &reference) {
                // a slip persists: the next block agrees with this one, not with the segment
                let persists = blocks
                    .get(k + 1)
                    .is_some_and(|next| !far(&next.2, &blocks[k].2) && far(&next.2, &reference));
                if !persists {
                    continue; // a glitch: leave the block out
                }
                close(&cur, &mut segments);
                cur.clear();
            }
        }
        cur.push(k);
    }
    close(&cur, &mut segments);
    segments
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ErrorStats {
    pub n: usize,
    pub pos_median_mm: f64,
    pub pos_p90_mm: f64,
    pub pos_p99_mm: f64,
    pub pos_max_mm: f64,
    pub pos_rms_mm: f64,
    pub rot_median_deg: f64,
    pub rot_p90_deg: f64,
}

fn quantile(v: &mut [f64], q: f64) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_by(f64::total_cmp);
    v[((v.len() - 1) as f64 * q).round() as usize]
}

fn stats(pairs: impl Iterator<Item = (Isometry3<f64>, Isometry3<f64>)>) -> ErrorStats {
    let (mut pe, mut re) = (Vec::new(), Vec::new());
    for (a, b) in pairs {
        pe.push((a.translation.vector - b.translation.vector).norm() * 1e3);
        re.push(a.rotation.angle_to(&b.rotation).to_degrees());
    }
    let rms = (pe.iter().map(|x| x * x).sum::<f64>() / pe.len().max(1) as f64).sqrt();
    ErrorStats {
        n: pe.len(),
        pos_median_mm: quantile(&mut pe, 0.5),
        pos_p90_mm: quantile(&mut pe, 0.9),
        pos_p99_mm: quantile(&mut pe, 0.99),
        pos_max_mm: quantile(&mut pe, 1.0),
        pos_rms_mm: rms,
        rot_median_deg: quantile(&mut re, 0.5),
        rot_p90_deg: quantile(&mut re, 0.9),
    }
}

/// Published(t) against truth(t - shift_ns).
pub fn errors(published: &[Published], truth: &Truth, shift_ns: i64) -> ErrorStats {
    stats(
        published
            .iter()
            .filter_map(|p| Some((p.pose, truth.at(p.t_ns as i64 - shift_ns)?))),
    )
}

/// The lag (ms) that minimises the RMS position error of published(t) against truth(t - lag):
/// 1 ms steps over `range_ms`, refined by a parabola through the best three. Returns (lag, rms mm).
pub fn lag_ms(published: &[Published], truth: &Truth, range_ms: (i64, i64)) -> (f64, f64) {
    let rms = |ms: i64| errors(published, truth, ms * 1_000_000).pos_rms_mm;
    let costs: Vec<(i64, f64)> = (range_ms.0..=range_ms.1)
        .map(|ms| (ms, rms(ms)))
        .filter(|c| c.1.is_finite())
        .collect();
    let Some(k) = (0..costs.len()).min_by(|&a, &b| costs[a].1.total_cmp(&costs[b].1)) else {
        return (f64::NAN, f64::NAN);
    };
    if k == 0 || k + 1 == costs.len() {
        return (costs[k].0 as f64, costs[k].1);
    }
    let (y0, y1, y2) = (costs[k - 1].1, costs[k].1, costs[k + 1].1);
    let d = y0 - 2.0 * y1 + y2;
    let off = if d > 0.0 { 0.5 * (y0 - y2) / d } else { 0.0 };
    (costs[k].0 as f64 + off, y1)
}

/// Jumps of the error vector between consecutive published poses larger than `threshold_m`:
/// (count, comparisons). Smooth noise and lag do not jump; mislabels and fusion switches do.
pub fn hops(
    published: &[Published],
    truth: &Truth,
    shift_ns: i64,
    threshold_m: f64,
) -> (usize, usize) {
    let errs: Vec<Vector3<f64>> = published
        .iter()
        .filter_map(|p| {
            Some(p.pose.translation.vector - truth.at(p.t_ns as i64 - shift_ns)?.translation.vector)
        })
        .collect();
    let n = errs.len().saturating_sub(1);
    (
        errs.windows(2)
            .filter(|w| (w[1] - w[0]).norm() > threshold_m)
            .count(),
        n,
    )
}

/// What SteamVR's prediction gets: each published pose carried `horizon_ms` ahead by its
/// velocities, against the truth at that time.
pub fn prediction(published: &[Published], truth: &Truth, horizon_ms: f64) -> ErrorStats {
    let h = horizon_ms * 1e-3;
    stats(published.iter().filter_map(|p| {
        let pos = p.pose.translation.vector + p.vel * h;
        let rot = UnitQuaternion::from_scaled_axis(p.ang_vel_world * h) * p.pose.rotation;
        Some((
            Isometry3::from_parts(Translation3::from(pos), rot),
            truth.at(p.t_ns as i64 + (horizon_ms * 1e6) as i64)?,
        ))
    }))
}

/// Spread of a still controller's published position: (rms about the mean, max), mm.
pub fn jitter(published: &[Published]) -> (f64, f64) {
    if published.is_empty() {
        return (f64::NAN, f64::NAN);
    }
    let mean = published
        .iter()
        .map(|p| p.pose.translation.vector)
        .sum::<Vector3<f64>>()
        / published.len() as f64;
    let d: Vec<f64> = published
        .iter()
        .map(|p| (p.pose.translation.vector - mean).norm() * 1e3)
        .collect();
    (
        (d.iter().map(|x| x * x).sum::<f64>() / d.len() as f64).sqrt(),
        d.iter().copied().fold(0.0, f64::max),
    )
}

/// Shake of a still controller: rms deviation (mm) of each published position from the mean over
/// the `window_ns` centred on it. Slow wander, invisible in a headset, does not count.
pub fn jitter_hf(published: &[Published], window_ns: u64) -> f64 {
    let (mut sum, mut n) = (0.0, 0usize);
    let (mut a, mut b) = (0usize, 0usize);
    let mut acc = Vector3::zeros();
    for p in published {
        while b < published.len() && published[b].t_ns <= p.t_ns + window_ns / 2 {
            acc += published[b].pose.translation.vector;
            b += 1;
        }
        while published[a].t_ns + window_ns / 2 < p.t_ns {
            acc -= published[a].pose.translation.vector;
            a += 1;
        }
        let mean = acc / (b - a) as f64;
        sum += (p.pose.translation.vector - mean).norm_squared();
        n += 1;
    }
    if n == 0 {
        f64::NAN
    } else {
        (sum / n as f64).sqrt() * 1e3
    }
}

pub const TRUTH_FORMAT: &str = "cv1touch-eval-truth";

#[derive(serde::Serialize, serde::Deserialize)]
pub struct TruthFile {
    pub format: String,
    pub version: u32,
    pub reference: String,
    pub universe_id: u64,
    /// (phase name, start ns, end ns)
    pub windows: Vec<(String, u64, u64)>,
    /// t_ns, qx, qy, qz, qw, px, py, pz
    pub samples: Vec<[f64; 8]>,
    /// Further tracked devices recorded beside the reference, in paced and free captures.
    #[serde(default)]
    pub others: Vec<RefTrack>,
    /// The daemon's latency adjustment (ms) when this was captured, which the stamps of its
    /// published poses carry (a replay's do not); none in files from before it was recorded.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub latency_adjustment_ms: Option<f64>,
    /// The base station SteamVR tracked each Lighthouse device from throughout the capture, by
    /// device serial (`steamvr_log::Span`); a device whose tracking restarted meanwhile, or every
    /// device when SteamVR's log could not be read, is not in it.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub tracking_bases: std::collections::BTreeMap<String, String>,
}

/// A lag smaller than this (ms) is within what `lag_ms` resolves: nothing to adjust.
const LAG_WORTH_ADJUSTING_MS: f64 = 0.5;

/// What to do about a measured lag of the published stamps against the truth (negative: the Touch
/// runs ahead of the reference): the `cv1touch latency` adjustment that puts the Touch on the
/// reference's timeline, on top of the one the capture was made with (`lag` moves the adjustment
/// by itself: a Touch ahead is stamped later); or how to apply it when the capture did not record
/// that adjustment. Which timeline is the true one the capture cannot tell. None when the lag is
/// not worth it.
pub fn latency_advice(lag_ms: f64, adjustment_at_capture_ms: Option<f64>) -> Option<String> {
    if !lag_ms.is_finite() || lag_ms.abs() < LAG_WORTH_ADJUSTING_MS {
        return None;
    }
    let max = crate::settings::MAX_LATENCY_ADJUSTMENT_MS;
    let Some(at_capture) = adjustment_at_capture_ms else {
        return Some(format!(
            "to put the Touch on the reference's timeline, add {lag_ms:+.1} ms to the latency \
             adjustment (`cv1touch latency` shows it)"
        ));
    };
    let wanted = at_capture + lag_ms;
    if wanted.abs() > max {
        return Some(format!(
            "a lag this large is not a stamp adjustment (+-{max:.0} ms at most): check that the \
             Touch stayed fixed to the reference"
        ));
    }
    let was = if at_capture == 0.0 {
        "none was set".to_string()
    } else {
        format!("{at_capture:+.1} ms was set")
    };
    Some(format!(
        "to put the Touch on the reference's timeline: cv1touch latency {wanted:+.1} ({was} when \
         this was captured)"
    ))
}

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct RefTrack {
    pub serial: String,
    pub samples: Vec<[f64; 8]>,
    /// "controller", "hmd" or "base" (empty in captures from before the field: controllers).
    #[serde(default)]
    pub kind: String,
}

impl RefTrack {
    pub fn is_controller(&self) -> bool {
        self.kind.is_empty() || self.kind == "controller"
    }
}

pub fn truth_samples(rows: &[[f64; 8]]) -> Vec<TruthSample> {
    rows.iter()
        .map(|s| TruthSample {
            t_ns: s[0] as u64,
            pose: Isometry3::from_parts(
                Translation3::new(s[5], s[6], s[7]),
                UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(s[4], s[1], s[2], s[3])),
            ),
        })
        .collect()
}

/// How far a track strays from its middle sample (m, radians, the farthest of each): a base
/// station's track stays at zero unless SteamVR moved the station during the capture.
pub fn spread(track: &[TruthSample]) -> Option<(f64, f64)> {
    let mid = track.get(track.len() / 2)?.pose;
    Some(track.iter().fold((0.0f64, 0.0f64), |(m, r), s| {
        let d = mid.inverse() * s.pose;
        (
            m.max(d.translation.vector.norm()),
            r.max(d.rotation.angle()),
        )
    }))
}

impl TruthFile {
    pub fn truth(&self) -> Vec<TruthSample> {
        truth_samples(&self.samples)
    }
}

pub fn truth_row(t_ns: u64, pose: &Isometry3<f64>) -> [f64; 8] {
    let q = pose.rotation.quaternion();
    let p = pose.translation.vector;
    [t_ns as f64, q.i, q.j, q.k, q.w, p.x, p.y, p.z]
}

/// Which published stream moves with the truth: the hand whose angular speed correlates best
/// (>= 0.7) with the reference's.
pub fn pick_hand(hands: [&[Published]; 2], truth: &[TruthSample]) -> Option<(usize, f64)> {
    let mut best: Option<(usize, f64)> = None;
    for (h, pubs) in hands.iter().enumerate() {
        let (mut a, mut b) = (Vec::new(), Vec::new());
        for w in pubs.windows(2) {
            let dt = (w[1].t_ns.saturating_sub(w[0].t_ns)) as f64 * 1e-9;
            let (Some(r0), Some(r1)) =
                (interpolate(truth, w[0].t_ns), interpolate(truth, w[1].t_ns))
            else {
                continue;
            };
            if dt <= 0.0 || dt > 0.05 {
                continue;
            }
            a.push(w[0].pose.rotation.angle_to(&w[1].pose.rotation) / dt);
            b.push(r0.rotation.angle_to(&r1.rotation) / dt);
        }
        let n = a.len() as f64;
        if n < 100.0 {
            continue;
        }
        let (ma, mb) = (a.iter().sum::<f64>() / n, b.iter().sum::<f64>() / n);
        let (mut sab, mut saa, mut sbb) = (0.0, 0.0, 0.0);
        for (x, y) in a.iter().zip(&b) {
            sab += (x - ma) * (y - mb);
            saa += (x - ma).powi(2);
            sbb += (y - mb).powi(2);
        }
        let c = if saa > 0.0 && sbb > 0.0 {
            sab / (saa * sbb).sqrt()
        } else {
            0.0
        };
        if c >= 0.7 && best.is_none_or(|(_, bc)| c > bc) {
            best = Some((h, c));
        }
    }
    best
}

#[cfg(test)]
mod spread_tests {
    use super::*;

    #[test]
    fn a_track_strays_by_its_farthest_sample_from_the_middle() {
        let at = |t: u64, x: f64, a: f64| TruthSample {
            t_ns: t,
            pose: Isometry3::from_parts(
                Translation3::new(x, 1.0, -2.0),
                UnitQuaternion::from_scaled_axis(Vector3::new(0.0, a, 0.0)),
            ),
        };
        assert_eq!(spread(&[]), None);
        let still = [at(0, 0.5, 0.2), at(1, 0.5, 0.2), at(2, 0.5, 0.2)];
        assert_eq!(spread(&still), Some((0.0, 0.0)));
        // moved once, late: 3 mm and 0.5 degrees from where it sat halfway through
        let moved = [
            at(0, 0.5, 0.2),
            at(1, 0.5, 0.2),
            at(2, 0.503, 0.2 + 0.5f64.to_radians()),
        ];
        let (m, r) = spread(&moved).unwrap();
        assert!((m - 0.003).abs() < 1e-9 && (r.to_degrees() - 0.5).abs() < 1e-6);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::synth::Rng;
    use nalgebra::{Translation3, UnitQuaternion};

    /// A smooth path: position and orientation as sums of sines.
    fn path(t: f64) -> Isometry3<f64> {
        let p = Vector3::new(
            0.4 * (0.9 * t).sin(),
            1.1 + 0.2 * (1.3 * t).sin(),
            0.3 * (0.7 * t).cos(),
        );
        let r = UnitQuaternion::from_scaled_axis(Vector3::new(
            0.8 * (0.6 * t).sin(),
            1.1 * (0.4 * t).cos(),
            0.5 * (1.1 * t).sin(),
        ));
        Isometry3::from_parts(Translation3::from(p), r)
    }

    fn ns(t: f64) -> u64 {
        (t * 1e9) as u64
    }

    /// Truth at 500 Hz for `secs`, as the reference pose in Lighthouse space with identity transforms.
    fn truth(secs: f64) -> Vec<TruthSample> {
        (0..(secs * 500.0) as usize)
            .map(|k| TruthSample {
                t_ns: ns(k as f64 / 500.0),
                pose: path(k as f64 / 500.0),
            })
            .collect()
    }

    /// Published at 250 Hz, `lag_s` behind the truth, with Gaussian position noise and occasional hops.
    fn published(secs: f64, lag_s: f64, noise_m: f64, hop_every: usize) -> Vec<Published> {
        let mut rng = Rng::new(5);
        (25..(secs * 250.0) as usize - 25)
            .map(|k| {
                let t = k as f64 / 250.0;
                let mut pose = path(t - lag_s);
                pose.translation.vector += rng.gauss3(noise_m);
                if hop_every > 0 && k % hop_every == 0 {
                    pose.translation.vector += Vector3::new(0.03, 0.0, 0.0);
                }
                let h = 1e-4;
                let vel = (path(t - lag_s + h).translation.vector
                    - path(t - lag_s - h).translation.vector)
                    / (2.0 * h);
                let ang = (path(t - lag_s - h).rotation.inverse() * path(t - lag_s + h).rotation)
                    .scaled_axis()
                    / (2.0 * h);
                Published {
                    t_ns: ns(t),
                    pose,
                    vel,
                    ang_vel_world: path(t - lag_s).rotation * ang,
                }
            })
            .collect()
    }

    fn identity_truth(samples: &[TruthSample]) -> Truth<'_> {
        Truth::single(samples, Isometry3::identity(), Isometry3::identity())
    }

    #[test]
    fn interpolates_between_truth_samples() {
        let t = truth(2.0);
        let mid = interpolate(&t, ns(1.001)).unwrap();
        assert!((mid.translation.vector - path(1.001).translation.vector).norm() < 1e-5);
        assert!(mid.rotation.angle_to(&path(1.001).rotation) < 1e-5);
        assert!(interpolate(&t, ns(5.0)).is_none());
    }

    #[test]
    fn finds_a_known_lag() {
        let t = truth(20.0);
        let p = published(20.0, 0.025, 0.0005, 0);
        let (lag, rms) = lag_ms(&p, &identity_truth(&t), (-50, 100));
        assert!((lag - 25.0).abs() < 1.0, "{lag}");
        assert!(rms < 1.0, "{rms} mm");
        let at_zero = errors(&p, &identity_truth(&t), 0);
        let at_lag = errors(&p, &identity_truth(&t), (lag * 1e6) as i64);
        assert!(
            at_lag.pos_median_mm < at_zero.pos_median_mm / 3.0,
            "{} vs {}",
            at_lag.pos_median_mm,
            at_zero.pos_median_mm
        );
    }

    #[test]
    fn counts_hops_not_noise() {
        let t = truth(10.0);
        let p = published(10.0, 0.0, 0.0005, 100);
        let (hops, n) = hops(&p, &identity_truth(&t), 0, 0.005);
        // each 3 cm hop shows as a jump in and a jump back out
        let injected = p
            .iter()
            .enumerate()
            .filter(|(k, _)| (k + 25) % 100 == 0)
            .count();
        assert!(
            hops >= 2 * injected - 1 && hops <= 2 * injected + 1,
            "{hops} hops for {injected} injected of {n}"
        );
        assert_eq!(
            super::hops(
                &published(10.0, 0.0, 0.0005, 0),
                &identity_truth(&t),
                0,
                0.005
            )
            .0,
            0
        );
    }

    #[test]
    fn prediction_with_exact_velocities_is_good() {
        let t = truth(10.0);
        let p = published(10.0, 0.0, 0.0, 0);
        let e = prediction(&p, &identity_truth(&t), 30.0);
        assert!(e.pos_p90_mm < 1.0, "{}", e.pos_p90_mm); // second-order error only over 30 ms
        let still: Vec<Published> = p
            .iter()
            .map(|x| Published {
                vel: Vector3::zeros(),
                ang_vel_world: Vector3::zeros(),
                ..*x
            })
            .collect();
        assert!(prediction(&still, &identity_truth(&t), 30.0).pos_median_mm > 5.0);
    }

    #[test]
    fn jitter_of_a_still_controller() {
        let mut rng = Rng::new(9);
        let still: Vec<Published> = (0..1000)
            .map(|k| Published {
                t_ns: k * 4_000_000,
                pose: Isometry3::from_parts(
                    Translation3::from(Vector3::new(0.1, 1.0, 0.2) + rng.gauss3(0.001)),
                    UnitQuaternion::identity(),
                ),
                vel: Vector3::zeros(),
                ang_vel_world: Vector3::zeros(),
            })
            .collect();
        let (rms, max) = jitter(&still);
        // 3D rms of 1 mm per axis is sqrt(3) mm
        assert!((rms - 3f64.sqrt()).abs() < 0.15, "{rms}");
        assert!(max > rms && max < 6.0, "{max}");
    }

    #[test]
    fn high_frequency_jitter_ignores_slow_wander() {
        let at = |k: u64, x: f64| Published {
            t_ns: k * 2_000_000,
            pose: Isometry3::translation(x, 0.0, 0.0),
            vel: Vector3::zeros(),
            ang_vel_world: Vector3::zeros(),
        };
        let wander: Vec<Published> = (0..5000)
            .map(|k| at(k, 0.01 * (k as f64 * 0.002 * 0.5).sin()))
            .collect();
        assert!(
            jitter_hf(&wander, 100_000_000) < 0.01,
            "{}",
            jitter_hf(&wander, 100_000_000)
        );
        let shake: Vec<Published> = (0..5000)
            .map(|k| at(k, if k % 2 == 0 { 0.001 } else { -0.001 }))
            .collect();
        assert!((jitter_hf(&shake, 100_000_000) - 1.0).abs() < 0.02);
    }

    /// A jig that slips at 5 s: the held offset is found per segment, and errors stay small.
    /// Tracking error (centimetre-level position wander, a one-second orientation glitch) must not
    /// split segments; only a persistent rotation of the hold does.
    #[test]
    fn a_slipping_jig_splits_into_segments() {
        let samples = truth(12.0);
        let y1 = Isometry3::from_parts(
            Translation3::new(0.1, 0.04, 0.16),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.3, 0.0, 0.2)),
        );
        let y2 = y1
            * Isometry3::from_parts(
                Translation3::new(0.01, 0.0, -0.004),
                UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.09, 0.0)),
            );
        let mut rng = Rng::new(3);
        let pubs: Vec<Published> = (0..3000)
            .map(|k| {
                let t = k as f64 / 250.0;
                let y = if t < 5.0 { y1 } else { y2 };
                let mut pose = path(t) * y;
                pose.translation.vector +=
                    rng.gauss3(0.0005) + Vector3::new(0.015 * (2.0 * t).sin(), 0.0, 0.0); // slow wander
                if (8.0..9.0).contains(&t) {
                    pose.rotation *= UnitQuaternion::from_scaled_axis(Vector3::new(0.1, 0.0, 0.0));
                    // a glitch
                }
                Published {
                    t_ns: ns(t),
                    pose,
                    vel: Vector3::zeros(),
                    ang_vel_world: Vector3::zeros(),
                }
            })
            .collect();
        let segs = offset_segments(
            &pubs,
            &samples,
            &Isometry3::identity(),
            0,
            &SegmentParams::default(),
        );
        assert_eq!(
            segs.len(),
            2,
            "{:?}",
            segs.iter()
                .map(|s| (s.start_ns, s.end_ns))
                .collect::<Vec<_>>()
        );
        assert!(
            (segs[0].end_ns as f64 * 1e-9 - 5.0).abs() < 0.6
                && (segs[1].start_ns as f64 * 1e-9 - 5.0).abs() < 0.6
        );
        for (seg, y) in segs.iter().zip([y1, y2]) {
            assert!(
                (seg.ref_from_touch.translation.vector - y.translation.vector).norm() < 0.01
                    && seg.ref_from_touch.rotation.angle_to(&y.rotation) < 0.02
            );
        }
        assert!(
            segs[1].end_ns as f64 * 1e-9 > 11.0,
            "the glitch did not end the second segment"
        );
    }

    #[test]
    fn truth_carries_through_registration_and_jig() {
        let samples = truth(2.0);
        let x = Isometry3::from_parts(
            Translation3::new(1.0, 0.0, -2.0),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.7, 0.0)),
        );
        let y = Isometry3::from_parts(
            Translation3::new(0.1, 0.04, 0.16),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.3, 0.0, 0.2)),
        );
        let tr = Truth::single(&samples, x, y);
        let got = tr.at(ns(1.0) as i64).unwrap();
        let want = x * path(1.0) * y;
        assert!(
            (got.translation.vector - want.translation.vector).norm() < 1e-9
                && got.rotation.angle_to(&want.rotation) < 1e-9
        );
    }

    /// What `eval` says to do about a lag: the adjustment that puts the Touch on the reference's
    /// timeline, on top of the one the capture was made with. A lag within the measurement's
    /// resolution is left alone, and one beyond what an adjustment can be is not a stamp problem.
    #[test]
    fn advises_the_latency_adjustment_that_removes_a_lag() {
        // the Touch 5.4 ms ahead with no adjustment: stamp it 5.4 ms later
        assert_eq!(
            latency_advice(-5.4, Some(0.0)).as_deref(),
            Some(
                "to put the Touch on the reference's timeline: cv1touch latency -5.4 (none was \
                 set when this was captured)"
            )
        );
        // on top of the one set when the capture was made
        assert_eq!(
            latency_advice(-5.4, Some(-5.4)).as_deref(),
            Some(
                "to put the Touch on the reference's timeline: cv1touch latency -10.8 (-5.4 ms \
                 was set when this was captured)"
            )
        );
        assert_eq!(
            latency_advice(3.0, Some(-1.0)).as_deref(),
            Some(
                "to put the Touch on the reference's timeline: cv1touch latency +2.0 (-1.0 ms \
                 was set when this was captured)"
            )
        );
        // a capture from before the adjustment was recorded with it
        assert_eq!(
            latency_advice(-5.4, None).as_deref(),
            Some(
                "to put the Touch on the reference's timeline, add -5.4 ms to the latency \
                 adjustment (`cv1touch latency` shows it)"
            )
        );
        assert_eq!(latency_advice(0.4, Some(0.0)), None);
        assert_eq!(latency_advice(-0.4, None), None);
        assert_eq!(latency_advice(f64::NAN, None), None);
        assert_eq!(
            latency_advice(40.0, Some(0.0)).as_deref(),
            Some(
                "a lag this large is not a stamp adjustment (+-30 ms at most): check that the \
                 Touch stayed fixed to the reference"
            )
        );
    }

    /// A truth file from before the adjustment was recorded loads with none, and none is left
    /// out when written.
    #[test]
    fn a_truth_file_without_the_adjustment_loads() {
        let tf: TruthFile = serde_json::from_str(
            r#"{"format":"cv1touch-eval-truth","version":1,"reference":"LHR-0","universe_id":1,"windows":[],"samples":[]}"#,
        )
        .unwrap();
        assert_eq!(tf.latency_adjustment_ms, None);
        assert!(!serde_json::to_string(&tf)
            .unwrap()
            .contains("latency_adjustment_ms"));
    }
}
