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

pub const FORMAT: &str = "cv1touch-landmarks";

/// A landmark is one sharp spot: blobs within this many pixels of it.
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

/// A sensor whose landmarks have moved: the median shift over them, px.
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
    /// Per landmark: blobs near it this window.
    seen: Vec<Vec<Vector2<f64>>>,
    /// Newest per-sensor shift (all sensors with a measured landmark).
    pub latest: BTreeMap<String, Shift>,
}

impl Monitor {
    pub fn new(refs: Vec<Landmark>, window_ns: u64) -> Monitor {
        let n = refs.len();
        Monitor {
            refs,
            window_ns,
            start: 0,
            seen: vec![Vec::new(); n],
            latest: BTreeMap::new(),
        }
    }

    /// One frame's unlabelled blobs. At the end of a window, returns each sensor's shift (median
    /// over its landmarks seen at least `min_hits` times).
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
        for (i, l) in self.refs.iter().enumerate() {
            if l.serial == serial {
                let r = Vector2::new(l.x, l.y);
                self.seen[i].extend(
                    blobs
                        .iter()
                        .filter(|b| (*b - r).norm() <= 2.0 * MODE_RADIUS)
                        .copied(),
                );
            }
        }
        if host_ns < self.start + self.window_ns {
            return None;
        }
        self.start = host_ns;
        type LandmarkPairs = Vec<(Vector2<f64>, Vector2<f64>)>;
        let mut per: BTreeMap<String, LandmarkPairs> = BTreeMap::new();
        for (i, l) in self.refs.iter().enumerate() {
            let r = Vector2::new(l.x, l.y);
            // in place: the mean of what lands within the mode's radius (re-centring could hop to
            // a neighbouring mode a pixel away)
            let near: Vec<Vector2<f64>> = self.seen[i]
                .iter()
                .filter(|b| (*b - r).norm() <= MODE_RADIUS)
                .copied()
                .collect();
            let (c, n) = (
                near.iter().sum::<Vector2<f64>>() / near.len().max(1) as f64,
                near.len(),
            );
            {
                if n >= min_hits {
                    per.entry(l.serial.clone()).or_default().push((r, c));
                }
            }
            self.seen[i].clear();
        }
        let shifts: Vec<Shift> = per
            .into_iter()
            .map(|(serial, pairs)| {
                let mut v: Vec<Vector2<f64>> = pairs.iter().map(|(r, c)| c - r).collect();
                let med = |k: usize, v: &mut Vec<Vector2<f64>>| {
                    v.sort_by(|a, b| a[k].total_cmp(&b[k]));
                    v[v.len() / 2][k]
                };
                let shift = Vector2::new(med(0, &mut v), med(1, &mut v));
                Shift {
                    serial,
                    shift,
                    landmarks: v.len(),
                    pairs,
                }
            })
            .collect();
        for s in &shifts {
            self.latest.insert(s.serial.clone(), s.clone());
        }
        Some(shifts)
    }
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
