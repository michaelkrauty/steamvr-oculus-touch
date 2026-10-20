// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Landmarks: static infrared sources each sensor sees at fixed pixels (the Lighthouse base
//! stations' laser sweeps, any other emitter that stays put). A sensor that is knocked moves all
//! of its landmarks together: watching them catches a moved sensor at a fraction of a pixel,
//! long before tracking degrades.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use nalgebra::Vector2;
use serde::{Deserialize, Serialize};

use crate::status::same_sensor;

pub const FORMAT: &str = "cv1touch-landmarks";

/// Radius used to learn one sharp spot. Displacement matching searches independently.
pub const MODE_RADIUS: f64 = 0.8;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Landmark {
    pub serial: String,
    pub x: f64,
    pub y: f64,
    /// Fraction of the sensor's frames it showed in when learned.
    pub seen: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LandmarkFile {
    pub format: String,
    pub version: u32,
    pub created_unix_s: u64,
    pub source: String,
    pub landmarks: Vec<Landmark>,
}

pub fn default_path() -> PathBuf {
    crate::align::registration_path().with_file_name("landmarks.json")
}

/// Writes `f` to `path`, replacing it whole: written beside it first, then renamed over it.
pub fn save(path: &Path, f: &LandmarkFile) -> Result<(), String> {
    let text = serde_json::to_string_pretty(f).map_err(|e| e.to_string())?;
    let tmp = path.with_extension("json.tmp");
    std::fs::write(&tmp, text + "\n").map_err(|e| format!("{}: {e}", tmp.display()))?;
    crate::platform::fs::rename_replacing(&tmp, path)
        .map_err(|e| format!("{}: {e}", path.display()))
}

/// Drops the landmarks of the sensor `serial` from the file at `path`, which was learned where
/// the sensor stood: how many went (0 when the file does not exist).
pub fn forget_in_file(path: &Path, serial: &str) -> Result<usize, String> {
    if !path.exists() {
        return Ok(0);
    }
    let mut f = load(path)?;
    let before = f.landmarks.len();
    f.landmarks.retain(|l| !same_sensor(&l.serial, serial));
    let gone = before - f.landmarks.len();
    if gone > 0 {
        save(path, &f)?;
    }
    Ok(gone)
}

pub fn load(path: &Path) -> Result<LandmarkFile, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let f: LandmarkFile =
        serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    if f.format != FORMAT {
        return Err(format!("{}: not a landmarks file", path.display()));
    }
    Ok(f)
}

/// The centroid of the blobs within `tight` px of the median of those within `radius` of `near`.
fn centroid(
    blobs: &[Vector2<f64>],
    near: Vector2<f64>,
    radius: f64,
    tight: f64,
) -> Option<(Vector2<f64>, usize)> {
    let set: Vec<Vector2<f64>> = blobs
        .iter()
        .filter(|b| (*b - near).norm() <= radius)
        .copied()
        .collect();
    if set.is_empty() {
        return None;
    }
    let med = |k: usize| {
        let mut v: Vec<f64> = set.iter().map(|b| b[k]).collect();
        v.sort_by(f64::total_cmp);
        v[v.len() / 2]
    };
    let m = Vector2::new(med(0), med(1));
    let core: Vec<Vector2<f64>> = set
        .iter()
        .filter(|b| (*b - m).norm() <= tight)
        .copied()
        .collect();
    (!core.is_empty()).then(|| {
        (
            core.iter().sum::<Vector2<f64>>() / core.len() as f64,
            core.len(),
        )
    })
}

/// Recurring spots of one sensor: `slices` holds its unlabelled blobs in consecutive time slices
/// (each with its frame count). A landmark shows in at least `min_fraction` of the frames of
/// nearly every slice, so a controller held still for a while does not become one.
pub fn learn(
    serial: &str,
    slices: &[(usize, Vec<Vector2<f64>>)],
    min_fraction: f64,
) -> Vec<Landmark> {
    let frames: usize = slices.iter().map(|s| s.0).sum();
    if frames == 0 || slices.is_empty() {
        return Vec::new();
    }
    // candidate spots: 4 px bins over the whole recording, merged within 15 px
    let mut bins: BTreeMap<(i64, i64), (Vector2<f64>, usize)> = BTreeMap::new();
    for (_, blobs) in slices {
        for b in blobs {
            let e = bins
                .entry(((b.x / 4.0) as i64, (b.y / 4.0) as i64))
                .or_insert((Vector2::zeros(), 0));
            e.0 += b;
            e.1 += 1;
        }
    }
    let mut cands: Vec<(Vector2<f64>, usize)> = bins
        .into_values()
        .filter(|b| b.1 as f64 >= min_fraction * frames as f64 * 0.25)
        .map(|(s, n)| (s / n as f64, n))
        .collect();
    cands.sort_by_key(|c| std::cmp::Reverse(c.1));
    let mut spots: Vec<Vector2<f64>> = Vec::new();
    for (c, _) in cands {
        if spots.iter().all(|s| (s - c).norm() > 15.0) {
            spots.push(c);
        }
    }
    let all: Vec<Vector2<f64>> = slices.iter().flat_map(|s| s.1.iter().copied()).collect();
    // a station's sweeps land on several sharp spots a few pixels apart whose mix changes from
    // minute to minute: each sharp spot (mode) is a landmark of its own
    let mut modes: Vec<Vector2<f64>> = Vec::new();
    for s in spots {
        let near: Vec<Vector2<f64>> = all
            .iter()
            .filter(|b| (*b - s).norm() <= 15.0)
            .copied()
            .collect();
        let mut cells: BTreeMap<(i64, i64), usize> = BTreeMap::new();
        for b in &near {
            *cells
                .entry(((b.x * 2.0).floor() as i64, (b.y * 2.0).floor() as i64))
                .or_default() += 1;
        }
        let mut peaks: Vec<((i64, i64), usize)> = cells.into_iter().collect();
        peaks.sort_by_key(|p| std::cmp::Reverse(p.1));
        for (cell, n) in peaks {
            if (n as f64) < 0.05 * near.len() as f64 {
                break;
            }
            let c = Vector2::new(cell.0 as f64 * 0.5 + 0.25, cell.1 as f64 * 0.5 + 0.25);
            if modes.iter().all(|m| (m - c).norm() > MODE_RADIUS * 2.0) {
                modes.push(c);
            }
        }
    }
    let mut out = Vec::new();
    for s in modes {
        let Some((xy, n)) = centroid(&all, s, MODE_RADIUS, MODE_RADIUS) else {
            continue;
        };
        if (n as f64) < 0.25 * min_fraction * frames as f64 {
            continue;
        }
        let present = slices
            .iter()
            .filter(|(f, blobs)| {
                blobs
                    .iter()
                    .filter(|b| (*b - xy).norm() <= MODE_RADIUS)
                    .count() as f64
                    >= 0.05 * min_fraction * *f as f64
            })
            .count();
        // the same spot reached from two candidates
        let dup = out
            .iter()
            .any(|l: &Landmark| (Vector2::new(l.x, l.y) - xy).norm() < MODE_RADIUS * 2.0);
        if present * 10 >= slices.len() * 9 && !dup {
            out.push(Landmark {
                serial: serial.to_string(),
                x: xy.x,
                y: xy.y,
                seen: n as f64 / frames as f64,
            });
        }
    }
    out
}

// Learned modes can be only 1.6 px apart. A 2 px search covers the single-spot alert
// with 1 px to spare, but overlaps neighbouring modes: only a coherent assignment is a
// measurement. Beyond it, or without a unique assignment, the displacement is unknown.
const MAX_SHIFT_PX: f64 = 2.0;
const SPOT_RADIUS_PX: f64 = 0.6;
// A conditional bound on a supporting mode's centroid error, not an RMS estimate.
const CENTROID_ERROR_PX: f64 = 0.4;

mod background;
pub use background::BackgroundMonitor;
#[cfg(test)]
mod property;
const COLLECTION_RADIUS_PX: f64 = MAX_SHIFT_PX + SPOT_RADIUS_PX;

#[derive(Debug)]
struct Spot {
    xy: Vector2<f64>,
    noise: f64,
}

/// Find recurring modes without reference-centred clipping. Cells seed the search; samples
/// within a mode determine its centre and uncertainty, independently of its displacement.
fn observed_spots(blobs: &[Vector2<f64>], min_hits: usize) -> Vec<Spot> {
    let mut remaining = blobs.to_vec();
    let mut spots: Vec<Spot> = Vec::new();
    loop {
        let mut cells = BTreeMap::<(i64, i64), usize>::new();
        for b in &remaining {
            *cells
                .entry(((b.x * 2.0).floor() as i64, (b.y * 2.0).floor() as i64))
                .or_default() += 1;
        }
        let mut seeds: Vec<_> = cells.into_iter().collect();
        seeds.sort_by_key(|(_, n)| std::cmp::Reverse(*n));
        let mut found = false;
        for ((x, y), _) in seeds {
            let seed = Vector2::new(x as f64 * 0.5 + 0.25, y as f64 * 0.5 + 0.25);
            let Some((xy, _)) = centroid(&remaining, seed, SPOT_RADIUS_PX, SPOT_RADIUS_PX) else {
                continue;
            };
            let core: Vec<_> = remaining
                .iter()
                .filter(|b| (*b - xy).norm() <= SPOT_RADIUS_PX)
                .collect();
            if core.len() < min_hits {
                continue;
            }
            let scatter =
                core.iter().map(|b| (*b - xy).norm_squared()).sum::<f64>() / core.len() as f64;
            let noise = scatter.sqrt().clamp(0.12, 0.4);
            spots.push(Spot { xy, noise });
            // Suppress only the samples this mode explains, not neighbouring seed cells.
            // A recurring mode trimmed out of this centroid must still be considered.
            remaining.retain(|b| (b - xy).norm() > SPOT_RADIUS_PX);
            found = true;
            break;
        }
        if !found {
            break;
        }
    }
    spots
}

/// References whose search regions overlap must be matched together. Otherwise a move
/// beyond the search could leave only a neighbouring mode in range and look like a small one.
fn neighbourhoods(reference: &[Vector2<f64>]) -> Vec<Vec<usize>> {
    let mut remaining = vec![true; reference.len()];
    let mut groups = Vec::new();
    for first in 0..reference.len() {
        if !remaining[first] {
            continue;
        }
        remaining[first] = false;
        let mut group = vec![first];
        let mut next = 0;
        while next < group.len() {
            for (i, r) in reference.iter().enumerate() {
                if remaining[i] && (r - reference[group[next]]).norm() <= 2.0 * COLLECTION_RADIUS_PX
                {
                    remaining[i] = false;
                    group.push(i);
                }
            }
            next += 1;
        }
        groups.push(group);
    }
    groups
}

/// Enumerate assignments, retaining alternatives rather than following a nearest-mode
/// refinement. Pairwise uncertainty is a necessary condition for a coherent fitted mean.
/// A crowded or pathological pattern that exhausts the bound is unknown.
// The uniqueness proof must retain a genuine boundary correspondence despite
// subtraction, norm and mean roundoff. Scale from the original coordinates: a
// small difference of large coordinates can lose bits before we compare shifts.
// The absolute floor covers near-zero inputs; extra retained mappings fail closed.
fn roundoff_px(scale: f64) -> f64 {
    1e-12 + 128.0 * f64::EPSILON * scale.max(1.0)
}

#[derive(Clone, Copy)]
struct Candidate {
    index: usize,
    shift: Vector2<f64>,
    noise: f64,
    scale: f64,
}

impl Candidate {
    fn new(index: usize, spot: Vector2<f64>, reference: Vector2<f64>, noise: f64) -> Self {
        let scale = spot
            .iter()
            .chain(reference.iter())
            .fold(1.0_f64, |scale, x| scale.max(x.abs()));
        Self {
            index,
            shift: spot - reference,
            noise,
            scale,
        }
    }
    fn in_range(&self) -> bool {
        self.shift.norm() <= MAX_SHIFT_PX + self.noise + roundoff_px(self.scale)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SearchKind {
    Fit,
    Correspondence,
}

struct Domain {
    row: usize,
    choices: Vec<Candidate>,
}

fn search_assignments(
    domains: &[Domain],
    selected: &mut [Option<Candidate>],
    chosen: &mut Vec<Candidate>,
    budget: &mut usize,
    kind: SearchKind,
    visit: &mut dyn FnMut(&[Candidate], Vector2<f64>) -> bool,
) -> bool {
    if *budget == 0 {
        return false;
    }
    *budget -= 1;
    if domains.is_empty() {
        chosen.clear();
        chosen.extend(selected.iter().map(|c| c.expect("every row is assigned")));
        let shift = chosen.iter().map(|c| c.shift).sum::<Vector2<f64>>() / chosen.len() as f64;
        return (kind == SearchKind::Fit
            && chosen
                .iter()
                .any(|c| (c.shift - shift).norm() > c.noise + roundoff_px(c.scale)))
            || visit(chosen, shift);
    }
    // Recompute the most constrained row after propagation. A precise anchor must
    // constrain the broad modes before exploring their exponential combinations.
    let tightness = |d: &Domain| d.choices.iter().map(|c| c.noise).fold(0.0, f64::max);
    let (next, domain) = domains
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            a.choices
                .len()
                .cmp(&b.choices.len())
                .then_with(|| tightness(a).total_cmp(&tightness(b)))
                .then_with(|| a.row.cmp(&b.row))
        })
        .expect("a row remains");
    for c in &domain.choices {
        let mut remaining = Vec::new();
        let mut impossible = false;
        for (i, d) in domains.iter().enumerate() {
            if i == next {
                continue;
            }
            // Fit rows are references and require distinct observed spots. In the
            // uniqueness check rows are observations: one emitter may explain two
            // modes. Retain those repeated indices whenever their bounds agree.
            let choices: Vec<_> = d
                .choices
                .iter()
                .filter(|p| {
                    (kind == SearchKind::Correspondence || p.index != c.index)
                        && (p.shift - c.shift).norm()
                            <= p.noise + c.noise + roundoff_px(p.scale.max(c.scale))
                })
                .copied()
                .collect();
            if choices.is_empty() {
                impossible = true;
                break;
            }
            remaining.push(Domain {
                row: d.row,
                choices,
            });
        }
        if impossible {
            continue;
        }
        selected[domain.row] = Some(*c);
        let complete = search_assignments(&remaining, selected, chosen, budget, kind, visit);
        selected[domain.row] = None;
        if !complete {
            return false;
        }
    }
    true
}

fn assignments(
    choices: &[Vec<Candidate>],
    chosen: &mut Vec<Candidate>,
    budget: &mut usize,
    visit: &mut dyn FnMut(&[Candidate], Vector2<f64>) -> bool,
) -> bool {
    enumerate_assignments(choices, chosen, budget, SearchKind::Fit, visit)
}

fn enumerate_assignments(
    choices: &[Vec<Candidate>],
    chosen: &mut Vec<Candidate>,
    budget: &mut usize,
    kind: SearchKind,
    visit: &mut dyn FnMut(&[Candidate], Vector2<f64>) -> bool,
) -> bool {
    if choices.is_empty() {
        return true;
    }
    let domains: Vec<_> = choices
        .iter()
        .enumerate()
        .map(|(row, choices)| Domain {
            row,
            choices: choices.clone(),
        })
        .collect();
    let complete = search_assignments(
        &domains,
        &mut vec![None; choices.len()],
        chosen,
        budget,
        kind,
        visit,
    );
    chosen.clear();
    complete
}

/// Positive-evidence invariant: every supporting spot must be an observation of a genuine
/// learned emitter with centroid error <= 0.4 px. Two spots may come from one emitter.
/// A published mean is within 0.4 px of the true translation
/// up to numerical roundoff. Proof-critical comparisons retain a scaled roundoff margin.
/// Spurious sources must not counterfeit the relative geometry of a learned pattern (two
/// or more emitters); that case cannot be distinguished from a move using pixels alone.
///
/// Map ALL supporting spots to references over ALL translations, allowing repeated emitter
/// indices. The true mapping passes the pairwise bound even if one emitter split into modes.
/// The triangle inequality gives a necessary condition; we deliberately retain this
/// superset, without testing residuals about the fitted mean: bounded individual errors
/// need not be bounded about their mean. Any other mapping, or an exhausted search, is
/// unknown. Thus a unique mapping is the true one and averaging preserves the error bound.
/// No missing spot, clipped support, or min_hits decision can refute a correspondence.
fn correspondence_is_unique(
    reference: &[Vector2<f64>],
    spots: &[Spot],
    pairs: &[Option<usize>],
) -> bool {
    let support: Vec<_> = pairs
        .iter()
        .enumerate()
        .filter_map(|(r, i)| i.map(|i| (r, i)))
        .collect();
    let choices: Vec<Vec<Candidate>> = support
        .iter()
        .map(|&(_, i)| {
            reference
                .iter()
                .enumerate()
                .map(|(index, r)| Candidate::new(index, spots[i].xy, *r, CENTROID_ERROR_PX))
                .collect()
        })
        .collect();
    enumerate_assignments(
        &choices,
        &mut Vec::new(),
        &mut 4096,
        SearchKind::Correspondence,
        &mut |chosen, _| chosen.iter().zip(&support).all(|(c, (r, _))| c.index == *r),
    )
}

fn common_shift(
    reference: &[Vector2<f64>],
    spots: &[Spot],
) -> Option<(Vector2<f64>, Vec<Option<usize>>)> {
    common_shift_with_budget(reference, spots, 4096)
}

fn common_shift_with_budget(
    reference: &[Vector2<f64>],
    spots: &[Spot],
    mut node_budget: usize,
) -> Option<(Vector2<f64>, Vec<Option<usize>>)> {
    if reference.is_empty() || reference.len() > 64 {
        return None;
    }
    let groups = neighbourhoods(reference);
    let mut required = Vec::new();
    for group in &groups {
        let visible = group.iter().any(|&i| {
            spots
                .iter()
                .any(|s| Candidate::new(0, s.xy, reference[i], s.noise).in_range())
        });
        if visible {
            required.extend(group.iter().copied());
        }
    }
    if required.len() < reference.len().min(2) {
        return None;
    }
    let choices: Vec<Vec<Candidate>> = required
        .iter()
        .map(|&r| {
            spots
                .iter()
                .enumerate()
                .filter_map(|(index, s)| {
                    let candidate = Candidate::new(index, s.xy, reference[r], s.noise);
                    // A spot can lie outside the range while its final fitted mean lies inside.
                    candidate.in_range().then_some(candidate)
                })
                .collect()
        })
        .collect();
    let mut solution = None;
    let complete = assignments(
        &choices,
        &mut Vec::new(),
        &mut node_budget,
        &mut |chosen, shift| {
            if shift.norm() > MAX_SHIFT_PX {
                return true;
            }
            let mut pairs = vec![None; reference.len()];
            for (&r, c) in required.iter().zip(chosen) {
                pairs[r] = Some(c.index);
            }
            // Every reference has exactly its proposed match at the fitted shift, or no
            // match for an entirely hidden group. Alternatives at this fit are ambiguous.
            for (r, assigned) in reference.iter().zip(&pairs) {
                let mut matches = spots
                    .iter()
                    .enumerate()
                    .filter(|(_, s)| {
                        let candidate = Candidate::new(0, s.xy, *r, s.noise);
                        (candidate.shift - shift).norm() <= s.noise + roundoff_px(candidate.scale)
                    })
                    .map(|(i, _)| i);
                // A competing match at a coherent fit makes the whole window unknown.
                if matches.next() != *assigned || matches.next().is_some() {
                    return false;
                }
            }
            if let Some((_, previous)) = &solution {
                if previous != &pairs {
                    return false;
                }
            }
            solution = Some((shift, pairs));
            true
        },
    );
    if !complete {
        return None;
    }
    let (shift, pairs) = solution?;
    correspondence_is_unique(reference, spots, &pairs).then_some((shift, pairs))
}

/// One sensor's coherent displacement in pixels. An ambiguous window has no shift.
#[derive(Debug, Clone, PartialEq)]
pub struct Shift {
    pub serial: String,
    pub shift: Vector2<f64>,
    pub landmarks: usize,
    /// Each measured landmark: (reference pixel, where it is now).
    pub pairs: Vec<(Vector2<f64>, Vector2<f64>)>,
}

/// Watches the landmarks in windows of `window_ns`.
#[derive(Debug, Clone)]
pub struct Monitor {
    refs: Vec<Landmark>,
    window_ns: u64,
    start: u64,
    /// Nearby unlabelled samples, collected once even where search regions overlap.
    seen: BTreeMap<String, Vec<Vector2<f64>>>,
    /// Only the newest window's known displacements; unknown removes an older result.
    pub latest: BTreeMap<String, Shift>,
}

impl Monitor {
    /// Stops watching the sensor `serial`'s landmarks: they were learned where it stood.
    pub fn forget(&mut self, serial: &str) {
        self.refs.retain(|l| !same_sensor(&l.serial, serial));
        self.seen.retain(|s, _| !same_sensor(s, serial));
        self.latest.retain(|s, _| !same_sensor(s, serial));
    }

    pub fn new(refs: Vec<Landmark>, window_ns: u64) -> Monitor {
        Monitor {
            refs,
            window_ns,
            start: 0,
            seen: BTreeMap::new(),
            latest: BTreeMap::new(),
        }
    }

    /// One frame's unlabelled blobs. A closed window returns only uniquely measured sensors.
    /// Missing modes, competing assignments and inconsistent shifts remain unknown.
    pub fn frame(
        &mut self,
        serial: &str,
        host_ns: u64,
        blobs: &[Vector2<f64>],
        min_hits: usize,
    ) -> Option<Vec<Shift>> {
        if self.start == 0 {
            self.start = host_ns;
        }
        // Use the landmark file's key for collection too, without allocating a folded
        // serial on each frame. A file and device may spell the same camera differently.
        let key = self
            .refs
            .iter()
            .find(|l| same_sensor(&l.serial, serial))
            .map(|l| l.serial.as_str());
        for b in blobs {
            if self.refs.iter().any(|l| {
                same_sensor(&l.serial, serial)
                    && (*b - Vector2::new(l.x, l.y)).norm() <= COLLECTION_RADIUS_PX
            }) {
                if let Some(bucket) = self.seen.get_mut(key.expect("matched landmark")) {
                    bucket.push(*b);
                } else {
                    self.seen
                        .insert(key.expect("matched landmark").to_owned(), vec![*b]);
                }
            }
        }
        if host_ns < self.start + self.window_ns {
            return None;
        }
        self.start = host_ns;
        let mut per = BTreeMap::<String, Vec<Vector2<f64>>>::new();
        for l in &self.refs {
            let point = Vector2::new(l.x, l.y);
            if let Some((_, points)) = per.iter_mut().find(|(s, _)| same_sensor(s, &l.serial)) {
                points.push(point);
            } else {
                per.insert(l.serial.clone(), vec![point]);
            }
        }
        self.latest.clear();
        let mut shifts = Vec::new();
        for (serial, reference) in per {
            let blobs = self.seen.remove(&serial).unwrap_or_default();
            let Some(s) = analyse(&serial, &reference, &blobs, min_hits) else {
                continue;
            };
            self.latest.insert(s.serial.clone(), s.clone());
            shifts.push(s);
        }
        Some(shifts)
    }
}

fn analyse(
    serial: &str,
    reference: &[Vector2<f64>],
    blobs: &[Vector2<f64>],
    min_hits: usize,
) -> Option<Shift> {
    let spots = observed_spots(blobs, min_hits);
    let (shift, assigned) = common_shift(reference, &spots)?;
    let pairs: Vec<_> = reference
        .iter()
        .zip(assigned)
        .filter_map(|(r, i)| i.map(|i| (*r, spots[i].xy)))
        .collect();
    Some(Shift {
        serial: serial.to_owned(),
        shift,
        landmarks: pairs.len(),
        pairs,
    })
}

/// The small rotation of a sensor (camera frame, axis * angle; world_from_cam is post-multiplied
/// by it) that carries its landmarks from their reference pixels to where they are now. One
/// landmark fixes pan and tilt only (the minimum-norm answer leaves roll alone).
pub fn rotation_correction(
    model: &crate::calib::camera::CameraModel,
    reference: &[Vector2<f64>],
    now: &[Vector2<f64>],
) -> Option<nalgebra::Vector3<f64>> {
    use nalgebra::{DMatrix, DVector, UnitQuaternion, Vector3};
    let rays: Vec<Vector3<f64>> = reference
        .iter()
        .map(|r| model.unproject(r))
        .collect::<Option<_>>()?;
    let predict = |w: &Vector3<f64>, d: &Vector3<f64>| {
        model.project(&(UnitQuaternion::from_scaled_axis(*w).inverse() * d))
    };
    let mut w = Vector3::zeros();
    for _ in 0..10 {
        let mut j = DMatrix::<f64>::zeros(2 * rays.len(), 3);
        let mut r = DVector::<f64>::zeros(2 * rays.len());
        for (i, (d, obs)) in rays.iter().zip(now).enumerate() {
            let p0 = predict(&w, d)?;
            let e = obs - p0;
            r[2 * i] = e.x;
            r[2 * i + 1] = e.y;
            for k in 0..3 {
                let mut wk = w;
                wk[k] += 1e-6;
                let dk = (predict(&wk, d)? - p0) / 1e-6;
                j[(2 * i, k)] = dk.x;
                j[(2 * i + 1, k)] = dk.y;
            }
        }
        // minimum norm (pseudo-inverse) for one landmark
        let svd = j.clone().svd(true, true);
        let step = svd.solve(&r, 1e-9).ok()?;
        w += Vector3::new(step[0], step[1], step[2]);
        if step.norm() < 1e-12 {
            break;
        }
    }
    Some(w)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn references(xs: &[f64]) -> Vec<Landmark> {
        xs.iter()
            .map(|&x| Landmark {
                serial: "A".into(),
                x,
                y: 300.0,
                seen: 0.1,
            })
            .collect()
    }
    fn window(m: &mut Monitor, start: u64, xs: &[f64]) -> Vec<Shift> {
        let mut out = None;
        for k in 0..=100u64 {
            let noise = (k % 5) as f64 * 0.02 - 0.04;
            let blobs: Vec<_> = xs.iter().map(|&x| Vector2::new(x + noise, 300.0)).collect();
            if let Some(s) = m.frame("A", start + k * 10, &blobs, 20) {
                out = Some(s);
            }
        }
        out.unwrap()
    }
    fn exact_window(refs: Vec<Landmark>, samples: &[Vec<Vector2<f64>>]) -> Vec<Shift> {
        let mut monitor = Monitor::new(refs, 1000);
        let mut result = None;
        for k in 0..=100 {
            if let Some(out) =
                monitor.frame("A", 1 + k * 10, &samples[k as usize % samples.len()], 20)
            {
                result = Some(out);
            }
        }
        result.unwrap()
    }

    fn scattered_samples(modes: &[(f64, f64)]) -> Vec<Vec<Vector2<f64>>> {
        (0..=100)
            .map(|k| {
                modes
                    .iter()
                    .map(|&(x, noise)| {
                        let tail = if noise < 0.35 { 18 } else { 32 };
                        let amplitude = noise * (101.0 / (2 * tail) as f64).sqrt();
                        let dy = if k < tail {
                            -amplitude
                        } else if k < 2 * tail {
                            amplitude
                        } else {
                            0.0
                        };
                        Vector2::new(x, 300.25 + dy)
                    })
                    .collect()
            })
            .collect()
    }

    fn scattered_references(xs: &[f64]) -> Vec<Landmark> {
        let mut refs = references(xs);
        for r in &mut refs {
            r.y = 300.25;
        }
        refs
    }

    fn assert_mode_scatter(samples: &[Vec<Vector2<f64>>], modes: &[(f64, f64)]) -> Vec<Spot> {
        let all: Vec<_> = samples.iter().flatten().copied().collect();
        let spots = observed_spots(&all, 20);
        assert_eq!(spots.len(), modes.len(), "{spots:?}");
        for &(x, noise) in modes {
            let s = spots
                .iter()
                .find(|s| (s.xy.x - x).abs() < 1e-8)
                .expect("the requested mode was extracted");
            assert!(
                (s.xy.y - 300.25).abs() < 1e-8 && (s.noise - noise).abs() < 1e-8,
                "{s:?}"
            );
        }
        spots
    }

    #[test]
    fn two_modes_of_one_emitter_cannot_prove_a_small_displacement() {
        let modes = [(1.1000000000001, 0.4), (1.8999999999999, 0.4)];
        let samples = scattered_samples(&modes);
        let spots = assert_mode_scatter(&samples, &modes);
        let truth = Vector2::new(1.5, 300.25);
        assert!(spots
            .iter()
            .all(|s| (s.xy - truth).norm() <= CENTROID_ERROR_PX));
        // One genuine emitter alternates between two recurring modes. Its second
        // learned neighbour is hidden; these are not two different emitters.
        let mut monitor = Monitor::new(scattered_references(&[0.0, 1.6]), 2020);
        for (k, blob) in samples.into_iter().flatten().enumerate() {
            assert!(monitor.frame("A", 1 + k as u64 * 10, &[blob], 20).is_none());
        }
        let out = monitor.frame("A", 2021, &[], 20).unwrap();
        assert!(out.is_empty(), "true shift +1.5, measured {out:?}");
    }

    #[test]
    fn a_split_emitter_does_not_hide_a_disambiguated_supporting_pattern() {
        let modes = [
            (0.7000000000001, 0.4),
            (1.4999999999999, 0.4),
            (101.4999999999999, 0.12),
        ];
        let samples = scattered_samples(&modes);
        assert_mode_scatter(&samples, &modes);
        let mut monitor = Monitor::new(scattered_references(&[0.0, 100.0]), 3030);
        for (k, blob) in samples.into_iter().flatten().enumerate() {
            assert!(monitor.frame("A", 1 + k as u64 * 10, &[blob], 20).is_none());
        }
        let out = monitor.frame("A", 3031, &[], 20).unwrap();
        assert_eq!(out.len(), 1, "{out:?}");
        assert_eq!(out[0].landmarks, 2);
        assert!((out[0].shift.x - 1.5).abs() < 1e-8, "{out:?}");
    }

    #[test]
    fn another_observed_emitter_can_disambiguate_two_close_supporting_spots() {
        let modes = [
            (0.3999999999999, 0.4),
            (1.2000000000001, 0.4),
            (100.0, 0.12),
        ];
        let samples = scattered_samples(&modes);
        assert_mode_scatter(&samples, &modes);
        // The first two spots alone could be one emitter shifted by 0.8 px. The
        // observed third emitter disagrees with that explanation, fixing the shift.
        let out = exact_window(scattered_references(&[0.0, 1.6, 100.0]), &samples);
        assert_eq!(out.len(), 1, "{out:?}");
        assert_eq!(out[0].landmarks, 3);
        assert!(out[0].shift.norm() < 1e-8, "{out:?}");
    }

    #[test]
    fn roundoff_cannot_hide_a_1024_pixel_correspondence() {
        let out = exact_window(
            references(&[100.39999999999999, 105.60000000000001, 1124.0, 1130.0]),
            &[vec![
                Vector2::new(1124.0, 300.0),
                Vector2::new(1130.0, 300.0),
            ]],
        );
        assert!(out.is_empty(), "{out:?}");
    }
    #[test]
    fn roundoff_cannot_hide_an_1100_pixel_correspondence() {
        // Repeated-sample averaging happens to perturb these fractional centres
        // enough to retain the alternative. A one-sample Monitor window preserves
        // the exact witness and isolates correspondence arithmetic (min_hits=1).
        let mut m = Monitor::new(
            references(&[100.0000000000001, 102.9999999999999, 1200.4, 1202.6]),
            1000,
        );
        assert!(m
            .frame(
                "A",
                1,
                &[Vector2::new(1200.4, 300.0), Vector2::new(1202.6, 300.0)],
                1
            )
            .is_none());
        let out = m.frame("A", 1001, &[], 1).unwrap();
        assert!(out.is_empty(), "{out:?}");
    }

    #[test]
    fn roundoff_allowance_scales_with_large_displacements() {
        let large = 1_073_741_824.0;
        let out = exact_window(
            references(&[
                100.39999999999999,
                105.60000000000001,
                large + 100.0,
                large + 106.0,
            ]),
            &[vec![
                Vector2::new(large + 100.0, 300.0),
                Vector2::new(large + 106.0, 300.0),
            ]],
        );
        assert!(out.is_empty(), "{out:?}");
    }

    #[test]
    fn an_exhausted_budget_cannot_publish_the_candidate_found_first() {
        let reference = [Vector2::new(100.0, 300.0), Vector2::new(500.0, 300.0)];
        let spots: Vec<_> = [100.0, 100.7, 500.0, 500.7]
            .iter()
            .map(|x| Spot {
                xy: Vector2::new(*x, 300.0),
                noise: 0.12,
            })
            .collect();
        let choices: Vec<Vec<_>> = reference
            .iter()
            .map(|r| {
                spots
                    .iter()
                    .enumerate()
                    .filter_map(|(index, s)| {
                        let candidate = Candidate::new(index, s.xy, *r, s.noise);
                        candidate.in_range().then_some(candidate)
                    })
                    .collect()
            })
            .collect();
        // Four nodes find zero, but stop before visiting the alternative +0.7 leaf.
        // Exercise the same production consumer with that reduced deterministic budget.
        let mut budget = 4;
        let mut found = Vec::new();
        let complete = assignments(&choices, &mut Vec::new(), &mut budget, &mut |_, shift| {
            found.push(shift);
            true
        });
        assert!(!complete && budget == 0);
        assert_eq!(found, vec![Vector2::zeros()]);
        assert!(common_shift_with_budget(&reference, &spots, 4).is_none());
    }

    #[test]
    fn sixty_five_references_are_unknown() {
        let xs: Vec<_> = (0..65).map(|i| 100.0 + 20.0 * i as f64).collect();
        let out = exact_window(
            references(&xs),
            &[xs.iter().map(|x| Vector2::new(*x, 300.0)).collect()],
        );
        assert!(out.is_empty(), "{out:?}");
    }

    #[test]
    fn an_outside_hypothesis_uses_modes_beyond_the_in_range_assignment() {
        let xs = [100.0, 101.8, 104.3, 110.0, 111.8];
        let out = exact_window(
            references(&xs),
            &[xs.iter().map(|x| Vector2::new(x + 10.0, 300.0)).collect()],
        );
        assert!(out.is_empty(), "{out:?}");
    }

    #[test]
    fn a_clipped_mode_below_min_hits_cannot_refute_an_outside_hypothesis() {
        let mut m = Monitor::new(references(&[100.0, 101.8, 104.38, 110.0, 111.8]), 1000);
        for k in 0..100 {
            let third = if k < 10 { 113.84 } else { 114.44 };
            let blobs = [
                Vector2::new(110.0, 300.0),
                Vector2::new(111.8, 300.0),
                Vector2::new(third, 300.0),
            ];
            assert!(m.frame("A", 1 + k * 10, &blobs, 20).is_none());
        }
        let out = m.frame("A", 1001, &[], 20).unwrap();
        assert!(out.is_empty(), "{out:?}");
    }

    #[test]
    fn a_competing_fit_cannot_be_skipped_to_publish_a_small_shift() {
        let modes = [(100.1, 0.4), (100.8, 0.4), (500.15, 0.4)];
        let samples = scattered_samples(&modes);
        assert_mode_scatter(&samples, &modes);
        let out = exact_window(scattered_references(&[100.0, 500.0]), &samples);
        assert!(out.is_empty(), "{out:?}");
    }

    #[test]
    fn a_competing_fit_cannot_be_skipped_to_publish_a_warning() {
        let modes = [(100.6, 0.3), (500.1, 0.3), (500.74, 0.4), (900.35, 0.3)];
        let samples = scattered_samples(&modes);
        assert_mode_scatter(&samples, &modes);
        let out = exact_window(scattered_references(&[100.0, 500.0, 900.0]), &samples);
        assert!(out.is_empty(), "{out:?}");
    }

    fn clipped_group_alias(xs: &[f64], dx: f64) {
        let samples = vec![xs.iter().map(|x| Vector2::new(x + dx, 300.0)).collect()];
        let out = exact_window(references(xs), &samples);
        assert!(out.is_empty(), "references {xs:?}: {out:?}");
    }

    #[test]
    fn a_fifteen_pixel_alias_can_lose_the_rest_of_its_group_outside_collection() {
        clipped_group_alias(&[100.0, 101.8, 106.8, 115.0, 116.8], 15.0);
    }

    #[test]
    fn a_ten_pixel_alias_can_lose_the_rest_of_its_group_outside_collection() {
        clipped_group_alias(&[100.0, 101.8, 104.5, 110.0, 111.8], 10.0);
    }

    #[test]
    fn repeated_emitters_are_measured_when_the_observations_disambiguate_them() {
        // Both complete repeated patterns are visible; neither can stand in for the other.
        for dx in [0.0, 0.6, 1.1, 1.5, 1.9] {
            let xs = [100.0, 101.8, 110.0, 111.8];
            let out = exact_window(
                references(&xs),
                &[xs.iter().map(|x| Vector2::new(x + dx, 300.0)).collect()],
            );
            assert_eq!(out.len(), 1, "{dx}: {out:?}");
            assert!((out[0].shift.x - dx).abs() < 1e-8, "{out:?}");
        }
        // An absent third emitter cannot disambiguate a repeated pair, even if its
        // predicted centre would be inside the collection footprint.
        for dx in [0.0, 0.6] {
            let out = exact_window(
                references(&[100.0, 101.8, 103.5, 110.0, 111.8]),
                &[vec![
                    Vector2::new(110.0 + dx, 300.0),
                    Vector2::new(111.8 + dx, 300.0),
                ]],
            );
            assert!(out.is_empty(), "{dx}: {out:?}");
        }
    }

    #[test]
    fn an_observed_third_emitter_disambiguates_a_repeated_pair() {
        let out = exact_window(
            references(&[100.0, 101.8, 110.0, 111.8, 114.0]),
            &[vec![
                Vector2::new(110.6, 300.0),
                Vector2::new(112.4, 300.0),
                Vector2::new(114.6, 300.0),
            ]],
        );
        assert_eq!(out.len(), 1, "{out:?}");
        assert!((out[0].shift.x - 0.6).abs() < 1e-8);
    }

    #[test]
    fn bounded_individual_errors_cannot_be_pruned_by_their_fitted_mean() {
        // The alternative correspondence has errors -0.4,+0.4,+0.4 around its true
        // translation. Its first residual about the mean is 0.533, so a fitted-mean
        // cutoff would wrongly discard it and publish zero from the other group.
        let out = exact_window(
            references(&[100.0, 103.0, 106.0, 500.4, 502.6, 505.6]),
            &[vec![
                Vector2::new(100.0, 300.0),
                Vector2::new(103.0, 300.0),
                Vector2::new(106.0, 300.0),
            ]],
        );
        assert!(out.is_empty(), "{out:?}");
    }

    #[test]
    fn a_counterfeit_source_is_indistinguishable_from_a_genuine_lone_emitter() {
        // World A: genuine emitter moved +1.5. World B: genuine emitter moved +10,
        // outside collection, with a persistent spurious emitter at the same pixel.
        // The identical input documents the genuine-support assumption, not a veto.
        let a = exact_window(references(&[100.0]), &[vec![Vector2::new(101.5, 300.0)]]);
        let b = exact_window(
            references(&[100.0]),
            &[vec![Vector2::new(101.5, 300.0), Vector2::new(110.0, 300.0)]],
        );
        assert_eq!(a, b);
        assert_eq!(a.len(), 1);
        assert!((a[0].shift.x - 1.5).abs() < 1e-8);
    }

    #[test]
    fn a_partial_pattern_within_the_centroid_bound_of_another_is_unknown() {
        let out = exact_window(
            references(&[100.0, 101.8, 500.0, 502.5]),
            &[vec![Vector2::new(100.6, 300.0), Vector2::new(102.4, 300.0)]],
        );
        assert!(out.is_empty(), "{out:?}");
    }

    fn anchor_scene(anchor_shift: f64) {
        let mut xs: Vec<_> = (0..12).map(|i| 100.0 + i as f64 * 20.0).collect();
        let mut modes: Vec<_> = xs
            .iter()
            .flat_map(|&x| [(x, 0.4), (x + 0.7, 0.4)])
            .collect();
        xs.push(500.0);
        modes.push((500.0 + anchor_shift, 0.12));
        let samples = scattered_samples(&modes);
        let spots = assert_mode_scatter(&samples, &modes);
        let out = exact_window(scattered_references(&xs), &samples);
        // Count the production search on the very same extracted mode centres/noise.
        let choices: Vec<Vec<_>> = xs
            .iter()
            .map(|x| {
                spots
                    .iter()
                    .enumerate()
                    .filter_map(|(index, s)| {
                        let candidate =
                            Candidate::new(index, s.xy, Vector2::new(*x, 300.25), s.noise);
                        candidate.in_range().then_some(candidate)
                    })
                    .collect()
            })
            .collect();
        let mut remaining = 4096;
        let mut fits = 0;
        let complete = assignments(&choices, &mut Vec::new(), &mut remaining, &mut |_, _| {
            fits += 1;
            true
        });
        let nodes = 4096 - remaining;
        println!(
            "anchor {anchor_shift}: {nodes} search nodes, {fits} fits, {} Monitor results",
            out.len()
        );
        assert!(complete && nodes < 64 && fits == 1, "anchor {anchor_shift}: {nodes} nodes, {fits} fits, complete={complete}, Monitor={out:?}");
        assert_eq!(out.len(), 1, "anchor {anchor_shift}: {out:?}");
        let expected = if anchor_shift == 0.0 {
            0.0
        } else {
            (12.0 * 0.7 + anchor_shift) / 13.0
        };
        assert!((out[0].shift.x - expected).abs() < 1e-8, "{out:?}");
    }

    #[test]
    fn a_precise_zero_anchor_resolves_many_two_mode_references() {
        anchor_scene(0.0);
    }

    #[test]
    fn a_precise_shifted_anchor_resolves_many_two_mode_references() {
        anchor_scene(0.65);
    }

    #[test]
    fn a_fit_at_the_search_boundary_keeps_competing_assignments() {
        let spots = vec![
            Vector2::new(100.199, 300.0),
            Vector2::new(101.999, 300.1),
            Vector2::new(103.799, 299.9),
        ];
        let out = exact_window(references(&[100.0, 101.8]), &[spots]);
        assert!(out.is_empty(), "{out:?}");
    }

    #[test]
    fn a_fit_at_the_search_boundary_can_start_outside_it() {
        let spots = vec![Vector2::new(101.999, 300.1), Vector2::new(501.999, 299.9)];
        let out = exact_window(references(&[100.0, 500.0]), &[spots]);
        assert_eq!(out.len(), 1, "{out:?}");
        assert!((out[0].shift.x - 1.999).abs() < 1e-9, "{out:?}");
    }

    #[test]
    fn alternative_noisy_assignments_are_not_lost_during_refinement() {
        let modes = [
            (100.0, -0.45, 0.30),
            (100.0, 0.55, 0.40),
            (500.0, -0.17, 0.40),
            (500.0, 0.55, 0.30),
            (900.0, 0.20, 0.40),
        ];
        // One blob per mode per frame: symmetric tails and a central peak give
        // exactly the stated RMS across the 101-frame window.
        let samples: Vec<Vec<_>> = (0..=100)
            .map(|k| {
                modes
                    .iter()
                    .map(|&(r, dx, sigma)| {
                        let tail = if sigma == 0.3 { 18 } else { 32 };
                        let amplitude = 0.5 * (101.0_f64 / 100.0).sqrt();
                        let dy = if k < tail {
                            -amplitude
                        } else if k < 2 * tail {
                            amplitude
                        } else {
                            0.0
                        };
                        Vector2::new(r + dx, 300.25 + dy)
                    })
                    .collect()
            })
            .collect();
        let mut refs = references(&[100.0, 500.0, 900.0]);
        for r in &mut refs {
            r.y = 300.25;
        }
        let out = exact_window(refs, &samples);
        assert!(out.is_empty(), "{out:?}");
    }

    #[test]
    fn recurring_samples_trimmed_from_a_mode_are_reconsidered() {
        let out = exact_window(
            references(&[100.0]),
            &[
                vec![Vector2::new(100.7, 300.0)],
                vec![Vector2::new(100.0, 300.0)],
            ],
        );
        assert!(out.is_empty(), "{out:?}");
    }

    #[test]
    fn repeated_groups_cannot_hide_a_move_beyond_the_search() {
        let out = exact_window(
            references(&[100.0, 101.8, 110.0, 111.8]),
            &[vec![
                Vector2::new(110.0, 300.0),
                Vector2::new(111.8, 300.0),
                Vector2::new(120.0, 300.0),
                Vector2::new(121.8, 300.0),
            ]],
        );
        assert!(out.is_empty(), "{out:?}");
    }

    #[test]
    fn measures_displacements_beyond_the_learning_radius() {
        for count in [1, 2, 12] {
            let xs: Vec<_> = (0..count).map(|i| 100.0 + i as f64 * 40.0).collect();
            for dx in [1.1, 1.5, 1.9, -1.5] {
                let mut m = Monitor::new(references(&xs), 1000);
                let moved: Vec<_> = xs.iter().map(|x| x + dx).collect();
                let out = window(&mut m, 1, &moved);
                assert_eq!(out.len(), 1, "{count} landmarks, {dx} px: {out:?}");
                assert!((out[0].shift.x - dx).abs() < 0.02, "{out:?}");
                assert_eq!(out[0].landmarks, count);
            }
        }
    }
    #[test]
    fn neighbouring_modes_cannot_stand_in_for_missing_landmarks() {
        for xs in [[100.0, 101.8, 500.0, 501.8], [100.0, 101.8, 500.0, 502.5]] {
            let mut m = Monitor::new(references(&xs), 1000);
            assert!(window(&mut m, 1, &[101.1, 501.1]).is_empty());
        }
    }

    #[test]
    fn a_move_beyond_the_search_cannot_alias_the_neighbouring_mode() {
        let mut m = Monitor::new(references(&[100.0, 101.8, 500.0, 501.8]), 1000);
        assert!(window(&mut m, 1, &[103.5, 503.5]).is_empty());
    }

    #[test]
    fn a_lone_landmark_with_two_possible_modes_is_unknown() {
        for xs in [[99.4, 101.1], [99.6, 100.6]] {
            let mut m = Monitor::new(references(&[100.0]), 1000);
            assert!(window(&mut m, 1, &xs).is_empty());
        }
    }

    #[test]
    fn a_noisy_modes_changing_mix_does_not_hide_the_other_spots() {
        let mut m = Monitor::new(references(&[100.0, 500.0, 900.0]), 1000);
        let mut measured = None;
        for k in 0..=100 {
            let blobs = [
                Vector2::new(99.8 + (k % 2) as f64 * 0.6 - 0.3, 300.0),
                Vector2::new(500.0, 300.0),
                Vector2::new(900.0, 300.0),
            ];
            if let Some(s) = m.frame("A", 1 + k * 10, &blobs, 20) {
                measured = Some(s);
            }
        }
        let measured = measured.unwrap();
        assert_eq!(measured.len(), 1);
        assert_eq!(measured[0].landmarks, 3);
        assert!(measured[0].shift.norm() < 0.1, "{measured:?}");
    }
    #[test]
    fn landmarks_must_agree_on_one_displacement() {
        let mut m = Monitor::new(references(&[100.0, 500.0, 900.0]), 1000);
        assert!(window(&mut m, 1, &[100.6, 499.4, 900.6]).is_empty());
    }
    #[test]
    fn an_unknown_window_does_not_keep_a_stale_measurement() {
        let mut m = Monitor::new(references(&[100.0]), 1000);
        assert_eq!(window(&mut m, 1, &[100.0]).len(), 1);
        assert!(window(&mut m, 1001, &[99.4, 101.1]).is_empty());
        assert!(!m.latest.contains_key("A"));
    }
    #[test]
    fn identical_hidden_groups_cannot_prove_a_partial_recovery() {
        let mut m = Monitor::new(references(&[100.0, 101.8, 500.0, 501.8]), 1000);
        assert!(window(&mut m, 1, &[100.6, 102.4]).is_empty());
    }
    #[test]
    fn an_occluded_station_does_not_hide_another_stations_complete_pattern() {
        let mut m = Monitor::new(references(&[100.0, 101.8, 500.0, 503.1]), 1000);
        let out = window(&mut m, 1, &[100.6, 102.4]);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].landmarks, 2);
        assert!((out[0].shift.x - 0.6).abs() < 0.02);
    }

    #[test]
    fn mixed_case_landmark_aliases_share_one_window() {
        let refs = vec![
            Landmark {
                serial: "camera1".into(),
                x: 100.0,
                y: 300.0,
                seen: 1.0,
            },
            Landmark {
                serial: "CAMERA1".into(),
                x: 500.0,
                y: 300.0,
                seen: 1.0,
            },
        ];
        let mut m = Monitor::new(refs, 1000);
        for k in 0..100 {
            m.frame(
                "Camera1",
                1 + k * 10,
                &[Vector2::new(100.5, 300.0), Vector2::new(500.5, 300.0)],
                20,
            );
        }
        let shifts = m.frame("Camera1", 1001, &[], 20).unwrap();
        assert_eq!(shifts.len(), 1);
        assert_eq!(shifts[0].landmarks, 2);
        assert!((shifts[0].shift.x - 0.5).abs() < 1e-8);
    }

    #[test]
    fn mixed_case_landmarks_follow_frames_and_forget_the_same_sensor() {
        let path =
            std::env::temp_dir().join(format!("cv1t-landmark-case-{}.json", std::process::id()));
        let refs = vec![Landmark {
            serial: "camera1".into(),
            x: 100.0,
            y: 300.0,
            seen: 1.0,
        }];
        save(
            &path,
            &LandmarkFile {
                format: FORMAT.into(),
                version: 1,
                created_unix_s: 0,
                source: "test".into(),
                landmarks: refs.clone(),
            },
        )
        .unwrap();
        let mut m = Monitor::new(refs, 1000);
        for k in 0..100 {
            m.frame("CAMERA1", 1 + k * 10, &[Vector2::new(100.5, 300.0)], 20);
        }
        let measured = m.frame("CAMERA1", 1001, &[], 20).unwrap();
        assert_eq!(measured.len(), 1);
        assert!((measured[0].shift.x - 0.5).abs() < 1e-8);
        m.forget("CaMeRa1");
        assert!(m.refs.is_empty() && m.seen.is_empty() && m.latest.is_empty());
        assert_eq!(forget_in_file(&path, "CAMERA1").unwrap(), 1);
        assert!(load(&path).unwrap().landmarks.is_empty());
        std::fs::remove_file(path).unwrap();
    }

    fn jitter(k: usize) -> Vector2<f64> {
        Vector2::new(
            ((k * 37) % 7) as f64 * 0.05 - 0.15,
            ((k * 53) % 5) as f64 * 0.05 - 0.1,
        )
    }

    #[test]
    fn a_rotated_sensor_is_turned_back_by_its_landmarks() {
        use crate::calib::synth;
        use nalgebra::{UnitQuaternion, Vector3};
        let m = synth::camera_model();
        let reference = vec![
            Vector2::new(293.0, 288.0),
            Vector2::new(1155.0, 278.0),
            Vector2::new(679.0, 273.0),
        ];
        let turn = Vector3::new(0.0004, -0.0011, 0.0006);
        let now: Vec<Vector2<f64>> = reference
            .iter()
            .map(|r| {
                m.project(
                    &(UnitQuaternion::from_scaled_axis(turn).inverse() * m.unproject(r).unwrap()),
                )
                .unwrap()
            })
            .collect();
        let w = rotation_correction(&m, &reference, &now).unwrap();
        assert!((w - turn).norm() < 1e-8, "{w:?}");
        // one landmark: pan and tilt only, and it lands exactly
        let w1 = rotation_correction(&m, &reference[..1], &now[..1]).unwrap();
        let p = m
            .project(
                &(UnitQuaternion::from_scaled_axis(w1).inverse()
                    * m.unproject(&reference[0]).unwrap()),
            )
            .unwrap();
        assert!((p - now[0]).norm() < 1e-6);
    }

    #[test]
    fn learns_spots_that_stay_and_not_a_controller_held_still_once() {
        let mut slices = Vec::new();
        for s in 0..10 {
            let mut blobs = Vec::new();
            for k in 0..100 {
                if k % 10 == 0 {
                    blobs.push(Vector2::new(293.0, 288.0) + jitter(k)); // a station: 10% of frames
                }
                blobs.push(Vector2::new(544.0, 885.0) + jitter(k + 3)); // a steady emitter
                if s == 4 {
                    blobs.push(Vector2::new(700.0, 500.0)); // a controller resting for one slice
                }
                blobs.push(Vector2::new(100.0 + (s * 100 + k) as f64 * 1.1, 400.0));
                // moving
            }
            slices.push((100, blobs));
        }
        let l = learn("A", &slices, 0.03);
        let mut xy: Vec<(i64, i64)> = l
            .iter()
            .map(|l| (l.x.round() as i64, l.y.round() as i64))
            .collect();
        xy.sort();
        assert_eq!(xy, vec![(293, 288), (544, 885)], "{l:?}");
    }

    #[test]
    fn a_knocked_sensor_shows_its_shift_and_a_steady_one_none() {
        let refs = vec![
            Landmark {
                serial: "A".into(),
                x: 293.0,
                y: 288.0,
                seen: 0.1,
            },
            Landmark {
                serial: "A".into(),
                x: 1155.0,
                y: 278.0,
                seen: 0.1,
            },
            Landmark {
                serial: "B".into(),
                x: 261.0,
                y: 806.0,
                seen: 0.1,
            },
        ];
        let mut m = Monitor::new(refs, 30_000_000_000);
        let mut out = None;
        for k in 0..2000u64 {
            let t = 1_000_000_000 + k * 19_000_000;
            let a = [
                Vector2::new(293.7, 287.8) + jitter(k as usize),
                Vector2::new(1155.7, 277.8) + jitter(k as usize + 1),
            ];
            for s in [
                m.frame(
                    "B",
                    t,
                    &[Vector2::new(261.0, 806.0) + jitter(k as usize + 2)],
                    50,
                ),
                m.frame("A", t, &a, 50),
            ]
            .into_iter()
            .flatten()
            {
                out = Some(s);
            }
        }
        let _ = out.expect("a window closed");
        let a = m.latest.get("A").unwrap();
        assert!(
            (a.shift - Vector2::new(0.7, -0.2)).norm() < 0.1 && a.landmarks == 2,
            "{a:?}"
        );
        let b = m.latest.get("B").unwrap();
        assert!(b.shift.norm() < 0.1, "{b:?}");
    }
}
