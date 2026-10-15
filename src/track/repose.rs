// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Moved sensors, found while the controllers are tracked and placed again from the sensors that
//! stayed, with nothing for the person to do. How each sensor agrees with the others
//! (`tracker::CameraAgreement`) says which one moved: a lot, and it misses nearly every check (the
//! controller is in its frame, by the engine's labels or by a frame full of blobs where the track
//! puts it, and nothing matches under the pose the others set); a little, and its residual
//! carries an offset the others do not. Such a sensor is then probed
//! rather than tracked with, and its probes, the controller as the other sensors track it with
//! the LEDs the engine labels in its frames, place it anew (`refine::place_sensor`,
//! `refine::refine_sensor`), as `cv1touch calibrate place` does from a recording.

use std::collections::{BTreeMap, BTreeSet};

use nalgebra::Isometry3;

use super::pipeline::Probe;
use super::refine::{place_sensor, refine_sensor};
use super::tracker::CameraAgreement;
use crate::calib::camera::CameraModel;
use crate::calib::capture::Led;

/// Sensors are judged by what their checks said over this long.
pub const WINDOW_NS: u64 = 5_000_000_000;
/// A sensor checked fewer times than this in a window is not judged by it.
pub const MIN_CHECKS: u64 = 40;
/// A sensor missing this share of its checks moved (a lot); one missing no more than
/// `STAYED_MISS_SHARE` with a residual under `STAYED_MEDIAN_PX` stayed, unless its offset says
/// otherwise.
pub const MOVED_MISS_SHARE: f64 = 0.6;
pub const STAYED_MISS_SHARE: f64 = 0.2;
pub const STAYED_MEDIAN_PX: f64 = 1.0;
/// A sensor whose residual offset is at least this, and this many times the others', moved (a
/// little). Needs two other sensors that stayed to compare with.
pub const MOVED_BIAS_PX: f64 = 0.6;
pub const MOVED_BIAS_RATIO: f64 = 2.5;
/// Windows in a row a sensor must be judged moved before it is.
pub const CONSECUTIVE: u32 = 2;
/// A sensor placed again within this of where it was had not moved: a false alarm, and nothing
/// is written.
pub const FALSE_ALARM_M: f64 = 0.003;
pub const FALSE_ALARM_RAD: f64 = 0.2 * std::f64::consts::PI / 180.0;
/// A probed sensor whose probes bring no pose in this long (no controller in its view, say) is
/// released, and left alone for `COOLDOWN_NS` after a false alarm or a timeout.
pub const PATIENCE_NS: u64 = 180_000_000_000;
pub const COOLDOWN_NS: u64 = 600_000_000_000;

/// What a window of checks said about a sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Standing {
    /// Too few checks, or no two other sensors to compare with.
    Unknown,
    Stayed,
    Moved,
}

/// What the monitor found at the end of a window.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Change {
    /// The sensor moved: probe it, and place it again.
    Moved(String),
    /// Fewer than two sensors agree with each other, so no sensor can be judged against the
    /// others; said once per window while it lasts.
    Unsure,
}

/// Watches the sensors' agreement and says which moved. One sensor at most is judged per window
/// end, the worst; sensors handed over (`Change::Moved`) are left alone until `released`.
#[derive(Debug, Default)]
pub struct Monitor {
    window_ns: u64,
    window_end: u64,
    /// Each sensor's (checks, misses) at the last window end.
    last: BTreeMap<String, (u64, u64)>,
    /// Windows in a row each sensor was judged moved.
    over: BTreeMap<String, u32>,
    handed_over: BTreeSet<String>,
    /// Sensors left alone until then (after a false alarm or a timeout).
    cooling: BTreeMap<String, u64>,
}

impl Monitor {
    pub fn new(window_ns: u64) -> Monitor {
        Monitor {
            window_ns,
            ..Default::default()
        }
    }

    /// The sensor `serial` is tracked with again: judge it afresh from now on, from the next
    /// full window of its checks.
    pub fn released(&mut self, serial: &str) {
        self.handed_over.remove(serial);
        self.over.remove(serial);
        self.last.remove(serial);
    }

    /// As `released`, and the sensor is left alone until `until_ns`.
    pub fn released_until(&mut self, serial: &str, until_ns: u64) {
        self.released(serial);
        self.cooling.insert(serial.to_string(), until_ns);
    }

    /// Whether `serial` was handed over and not released.
    pub fn handed_over(&self, serial: &str) -> bool {
        self.handed_over.contains(serial)
    }

    /// Called as often as convenient with every sensor's agreement; judges at the end of each
    /// window, from what the checks since the last one said.
    pub fn update(&mut self, now_ns: u64, agreement: &[(String, CameraAgreement)]) -> Vec<Change> {
        if self.window_end == 0 {
            self.window_end = now_ns + self.window_ns;
            self.remember(agreement);
            return Vec::new();
        }
        if now_ns < self.window_end {
            return Vec::new();
        }
        self.window_end = now_ns + self.window_ns;
        self.cooling.retain(|_, until| *until > now_ns);
        let (standings, judgeable) = self.judge(agreement);
        self.remember(agreement);
        let mut changes = Vec::new();
        if judgeable
            && standings
                .iter()
                .filter(|(_, s, _)| *s == Standing::Stayed)
                .count()
                < 2
        {
            changes.push(Change::Unsure);
        }
        let mut worst: Option<(&str, f64)> = None;
        for (serial, standing, badness) in &standings {
            let over = self.over.entry(serial.to_string()).or_default();
            if *standing == Standing::Moved {
                *over += 1;
                if *over >= CONSECUTIVE && worst.is_none_or(|(_, b)| *badness > b) {
                    worst = Some((serial, *badness));
                }
            } else {
                *over = 0;
            }
        }
        if let Some((serial, _)) = worst {
            self.over.remove(serial);
            self.handed_over.insert(serial.to_string());
            changes.push(Change::Moved(serial.to_string()));
        }
        changes
    }

    fn remember(&mut self, agreement: &[(String, CameraAgreement)]) {
        for (serial, a) in agreement {
            self.last.insert(serial.clone(), (a.checks, a.misses));
        }
    }

    /// Each sensor not handed over: its standing over the window, and how badly it disagrees
    /// (the share of checks missed, or its offset in pixels); and whether any sensor was checked
    /// enough to be judged at all.
    fn judge<'a>(
        &self,
        agreement: &'a [(String, CameraAgreement)],
    ) -> (Vec<(&'a str, Standing, f64)>, bool) {
        // the window's checks and misses, and the agreement, per sensor judged
        let seen: Vec<(&str, &CameraAgreement, u64, u64)> = agreement
            .iter()
            .filter(|(serial, _)| {
                !self.handed_over.contains(serial) && !self.cooling.contains_key(serial)
            })
            .map(|(serial, a)| {
                let (checks, misses) = match self.last.get(serial) {
                    // a tracker started over resets its counters: this window says nothing, and
                    // neither does the first window after a sensor is released
                    Some(&(c, m)) if a.checks >= c && a.misses >= m => (a.checks - c, a.misses - m),
                    _ => (0, 0),
                };
                (serial.as_str(), a, checks, misses)
            })
            .collect();
        let judgeable = seen
            .iter()
            .filter(|(_, _, checks, _)| *checks >= MIN_CHECKS)
            .count()
            >= 2;
        let share = |checks: u64, misses: u64| misses as f64 / checks.max(1) as f64;
        let stayed: Vec<&(&str, &CameraAgreement, u64, u64)> = seen
            .iter()
            .filter(|(_, a, checks, misses)| {
                *checks >= MIN_CHECKS
                    && share(*checks, *misses) <= STAYED_MISS_SHARE
                    && a.median_px <= STAYED_MEDIAN_PX
            })
            .collect();
        let standings = seen
            .iter()
            .map(|&(serial, a, checks, misses)| {
                if checks < MIN_CHECKS || stayed.len() < 2 {
                    return (serial, Standing::Unknown, 0.0);
                }
                let missed = share(checks, misses);
                if missed >= MOVED_MISS_SHARE {
                    return (serial, Standing::Moved, 1.0 + missed);
                }
                // a little: an offset the sensors that stayed, this one aside, do not have
                let others: Vec<f64> = stayed
                    .iter()
                    .filter(|s| s.0 != serial)
                    .map(|s| s.1.bias_px.norm())
                    .collect();
                let bias = a.bias_px.norm();
                if others.len() >= 2 && missed <= STAYED_MISS_SHARE {
                    let mut sorted = others;
                    sorted.sort_by(|x, y| x.total_cmp(y));
                    let typical = sorted[sorted.len() / 2];
                    if bias >= MOVED_BIAS_PX && bias >= MOVED_BIAS_RATIO * typical {
                        return (serial, Standing::Moved, bias);
                    }
                }
                let stayed_here = stayed.iter().any(|s| s.0 == serial);
                (
                    serial,
                    if stayed_here {
                        Standing::Stayed
                    } else {
                        Standing::Unknown
                    },
                    0.0,
                )
            })
            .collect();
        (standings, judgeable)
    }
}

/// Probes with labelled LEDs that make a batch worth solving, once the controller has moved
/// `BATCH_SPREAD_M` across them.
pub const BATCH_PROBES: usize = 300;
pub const BATCH_SPREAD_M: f64 = 0.3;
/// Probes kept per sensor at most; the oldest go first. A probe within `THIN_M` and `THIN_RAD` of
/// the last one kept adds nothing (a controller at rest) and is not kept.
pub const KEEP_PROBES: usize = 1200;
pub const THIN_M: f64 = 0.01;
pub const THIN_RAD: f64 = std::f64::consts::PI / 180.0;
/// A batch's probes must also span `BATCH_SIDE_M` across a second direction and turn by
/// `BATCH_TURN_RAD`, so that the fit is not one line of positions.
pub const BATCH_SIDE_M: f64 = 0.1;
pub const BATCH_TURN_RAD: f64 = 20.0 * std::f64::consts::PI / 180.0;
/// A fit is taken when the sensor's labelled sightings mostly fit it, too: the share of the
/// labelled LED sightings that lie within the placing gate (`refine::Placed::inliers`).
pub const ACCEPT_LABEL_SHARE: f64 = 0.5;
/// Probes whose pose is less certain than this (metres) are not used.
pub const MAX_POS_SD: f64 = 0.002;
/// A fit is taken when its median residual is at most this, on at least this many LED matches.
pub const ACCEPT_MEDIAN_PX: f64 = 1.0;
pub const ACCEPT_PAIRS: usize = 200;
/// Two fits from independent batches confirm each other within these.
pub const CONFIRM_M: f64 = 0.01;
pub const CONFIRM_RAD: f64 = 0.5 * std::f64::consts::PI / 180.0;
/// Batches that failed, were loose or disagreed before the sensor is given up on.
pub const MAX_ATTEMPTS: u32 = 6;

/// A sensor pose fitted to probes.
#[derive(Debug, Clone, PartialEq)]
pub struct Fit {
    pub world_from_cam: Isometry3<f64>,
    /// Median residual (px) of the LED matches, and how many, in how many frames.
    pub median_px: f64,
    pub pairs: usize,
    pub frames: usize,
    /// Of the labelled LED sightings the pose was placed from, the share that fit it.
    pub label_share: f64,
}

/// A batch of probes to solve, off the tracking thread: the fresh probes, and when a first fit
/// awaits confirmation, the probes it came from, for one fit on all of them.
#[derive(Debug, Clone, PartialEq)]
pub struct Job {
    pub camera: usize,
    pub fresh: Vec<Probe>,
    pub earlier: Vec<Probe>,
}

/// What a solved job gave: the fit on the fresh probes alone, and on all of them (the same when
/// there were no earlier ones).
#[derive(Debug, Clone, PartialEq)]
pub struct Solved {
    pub fresh: Fit,
    pub all: Fit,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Outcome {
    Nothing,
    /// The sensor's pose, confirmed by two independent batches: fitted on both.
    Placed {
        camera: usize,
        fit: Fit,
    },
    /// No pose fits, or none holds up, batch after batch: the sensor is left out of tracking.
    GaveUp {
        camera: usize,
        attempts: u32,
        reason: String,
    },
}

#[derive(Debug, Default)]
struct Gathering {
    probes: Vec<Probe>,
    /// Probes kept since the sensor was watched, batches included.
    kept: usize,
    /// The pose of the last probe kept, for thinning.
    last_kept: Option<Isometry3<f64>>,
    /// A fit awaiting confirmation, with the probes it was fitted on.
    first: Option<(Fit, Vec<Probe>)>,
    attempts: u32,
    solving: bool,
    given_up: bool,
}

/// Gathers a probed sensor's probes and, batch by batch, the fits that place it again: a fit is
/// taken when tight, and the pose is confirmed when a second batch lands within tolerance.
#[derive(Debug, Default)]
pub struct Reposer {
    gathering: BTreeMap<usize, Gathering>,
}

impl Reposer {
    /// Starts gathering the sensor `camera`'s probes (and starts over, if it was watched).
    pub fn watch(&mut self, camera: usize) {
        self.gathering.insert(camera, Gathering::default());
    }

    pub fn forget(&mut self, camera: usize) {
        self.gathering.remove(&camera);
    }

    pub fn watching(&self, camera: usize) -> bool {
        self.gathering.contains_key(&camera)
    }

    /// Whether the sensor was given up on (`Outcome::GaveUp`).
    pub fn given_up(&self, camera: usize) -> bool {
        self.gathering.get(&camera).is_some_and(|g| g.given_up)
    }

    /// A probe of a watched sensor, with labelled LEDs and a certain enough pose, is kept.
    pub fn add(&mut self, probe: Probe) {
        let Some(g) = self.gathering.get_mut(&probe.camera) else {
            return;
        };
        if g.given_up
            || probe.pos_sd > MAX_POS_SD
            || !probe
                .labels
                .iter()
                .any(|&l| l >= 0 && l >> 8 == probe.device_id)
        {
            return;
        }
        if let Some(last) = &g.last_kept {
            let d = last.inverse() * probe.world_from_model;
            if d.translation.vector.norm() < THIN_M && d.rotation.angle() < THIN_RAD {
                return;
            }
        }
        g.last_kept = Some(probe.world_from_model);
        g.kept += 1;
        if g.probes.len() >= KEEP_PROBES {
            g.probes.remove(0);
        }
        g.probes.push(probe);
    }

    /// How many probes a watched sensor has gathered since it was watched, batches included.
    pub fn gathered(&self, camera: usize) -> usize {
        self.gathering.get(&camera).map_or(0, |g| g.kept)
    }

    /// The next batch worth solving, if any: a sensor with enough probes, spread enough, and no
    /// solve in flight. Its probes are taken.
    pub fn batch(&mut self) -> Option<Job> {
        let camera = *self
            .gathering
            .iter()
            .find(|(_, g)| {
                !g.solving && !g.given_up && g.probes.len() >= BATCH_PROBES && varied(&g.probes)
            })?
            .0;
        let g = self.gathering.get_mut(&camera)?;
        g.solving = true;
        Some(Job {
            camera,
            fresh: std::mem::take(&mut g.probes),
            // the first fit's probes go along for one fit on all of them; the fit awaiting
            // confirmation keeps its pose, which is all that is compared
            earlier: g
                .first
                .as_mut()
                .map(|(_, p)| std::mem::take(p))
                .unwrap_or_default(),
        })
    }

    /// What a job's result means for its sensor.
    pub fn solved(&mut self, job: Job, result: Result<Solved, String>) -> Outcome {
        let Some(g) = self.gathering.get_mut(&job.camera) else {
            return Outcome::Nothing;
        };
        g.solving = false;
        let failed = |g: &mut Gathering, reason: String| {
            g.attempts += 1;
            if g.attempts >= MAX_ATTEMPTS {
                g.given_up = true;
                g.first = None;
                Outcome::GaveUp {
                    camera: job.camera,
                    attempts: g.attempts,
                    reason,
                }
            } else {
                Outcome::Nothing
            }
        };
        let solved = match result {
            Ok(s) => s,
            Err(e) => return failed(g, e),
        };
        if solved.fresh.median_px > ACCEPT_MEDIAN_PX
            || solved.fresh.pairs < ACCEPT_PAIRS
            || solved.fresh.label_share < ACCEPT_LABEL_SHARE
        {
            return failed(
                g,
                format!(
                    "a loose fit: {:.2} px median on {} matches, {:.0}% of the labelled sightings",
                    solved.fresh.median_px,
                    solved.fresh.pairs,
                    solved.fresh.label_share * 100.0
                ),
            );
        }
        let Some((first, _)) = g.first.take() else {
            g.first = Some((solved.fresh, job.fresh));
            return Outcome::Nothing;
        };
        let d = first.world_from_cam.inverse() * solved.fresh.world_from_cam;
        let (moved, turned) = (d.translation.vector.norm(), d.rotation.angle());
        if moved <= CONFIRM_M && turned <= CONFIRM_RAD {
            g.attempts = 0;
            Outcome::Placed {
                camera: job.camera,
                fit: solved.all,
            }
        } else {
            // the newer fit is the one to confirm next
            g.first = Some((solved.fresh, job.fresh));
            failed(
                g,
                format!(
                    "two fits {:.1} mm and {:.2} degrees apart",
                    moved * 1e3,
                    turned.to_degrees()
                ),
            )
        }
    }
}

/// Whether a pose placed again is where the sensor was calibrated, within `FALSE_ALARM_M` and
/// `FALSE_ALARM_RAD`: the sensor had not moved.
pub fn false_alarm(calibrated: &Isometry3<f64>, placed: &Isometry3<f64>) -> bool {
    let d = calibrated.inverse() * placed;
    d.translation.vector.norm() <= FALSE_ALARM_M && d.rotation.angle() <= FALSE_ALARM_RAD
}

/// Whether the controller moved enough across `probes` for a pose to be well determined: the
/// bounding box of its positions spans `BATCH_SPREAD_M`, and `BATCH_SIDE_M` across its second
/// widest side, and it turned by `BATCH_TURN_RAD` between some two probes.
fn varied(probes: &[Probe]) -> bool {
    let Some(first) = probes.first() else {
        return false;
    };
    let mut lo = nalgebra::Vector3::repeat(f64::INFINITY);
    let mut hi = nalgebra::Vector3::repeat(f64::NEG_INFINITY);
    let mut turned: f64 = 0.0;
    for p in probes {
        let t = p.world_from_model.translation.vector;
        lo = lo.inf(&t);
        hi = hi.sup(&t);
        turned = turned.max(
            first
                .world_from_model
                .rotation
                .angle_to(&p.world_from_model.rotation),
        );
    }
    let mut sides: Vec<f64> = (hi - lo).iter().copied().collect();
    sides.sort_by(|a, b| b.total_cmp(a));
    (hi - lo).norm() >= BATCH_SPREAD_M && sides[1] >= BATCH_SIDE_M && turned >= BATCH_TURN_RAD
}

/// Solves a job: the sensor placed from the fresh probes' labels and refined on them, and, with
/// earlier probes, refined on all of them too. `bodies` are the controllers' LEDs by hand.
pub fn solve(
    model: &CameraModel,
    bodies: [Option<&[Led]>; 2],
    job: &Job,
) -> Result<Solved, String> {
    let placed = place_sensor(model, bodies, &job.fresh, MAX_POS_SD)?;
    let label_share = placed.inliers as f64 / placed.offered.max(1) as f64;
    let fit = |initial: &Isometry3<f64>, probes: &[Probe]| -> Result<Fit, String> {
        let r = refine_sensor(model, initial, bodies, probes, MAX_POS_SD)?;
        Ok(Fit {
            world_from_cam: r.world_from_cam,
            median_px: r.after_px,
            pairs: r.pairs,
            frames: r.frames,
            label_share,
        })
    };
    let fresh = fit(&placed.world_from_cam, &job.fresh)?;
    let all = if job.earlier.is_empty() {
        fresh.clone()
    } else {
        let mut probes = job.earlier.clone();
        probes.extend(job.fresh.iter().cloned());
        fit(&fresh.world_from_cam, &probes)?
    };
    Ok(Solved { fresh, all })
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector2;

    fn sensor(
        serial: &str,
        checks: u64,
        misses: u64,
        median: f64,
        bias: f64,
    ) -> (String, CameraAgreement) {
        (
            serial.to_string(),
            CameraAgreement {
                median_px: median,
                bias_px: Vector2::new(bias, 0.0),
                samples: checks - misses,
                checks,
                misses,
            },
        )
    }

    /// The agreement after `windows` windows: three sensors that stayed, and D as `d` makes it
    /// from the window's checks and misses.
    fn room(
        windows: u64,
        d: impl Fn(u64) -> (u64, u64, f64, f64),
    ) -> Vec<(String, CameraAgreement)> {
        let (dc, dm, median, bias) = d(windows);
        vec![
            sensor("A", 100 * windows, 5 * windows, 0.3, 0.2),
            sensor("B", 100 * windows, 4 * windows, 0.3, 0.25),
            sensor("C", 90 * windows, 6 * windows, 0.4, 0.15),
            sensor("D", dc, dm, median, bias),
        ]
    }

    fn run(
        m: &mut Monitor,
        windows: u64,
        d: impl Fn(u64) -> (u64, u64, f64, f64),
    ) -> Vec<Vec<Change>> {
        (0..=windows)
            .map(|w| m.update(1_000 + w * WINDOW_NS, &room(w, &d)))
            .collect()
    }

    /// A sensor that misses nearly every check moved a lot: judged after two windows in a row,
    /// then left to the reposer.
    #[test]
    fn a_sensor_missing_its_checks_moved_a_lot() {
        let mut m = Monitor::new(WINDOW_NS);
        let out = run(&mut m, 4, |w| (100 * w, 80 * w, 0.3, 0.1));
        assert_eq!(out[0], []);
        assert_eq!(out[1], []);
        assert_eq!(out[2], [Change::Moved("D".into())]);
        assert_eq!(out[3], []);
        assert_eq!(out[4], []);
        assert!(m.handed_over("D") && !m.handed_over("A"));
    }

    /// A sensor whose residual carries an offset the others do not moved a little.
    #[test]
    fn a_nudged_sensor_stands_out_by_its_offset() {
        let mut m = Monitor::new(WINDOW_NS);
        let out = run(&mut m, 2, |w| (100 * w, 0, 0.9, 0.9));
        assert_eq!(out[2], [Change::Moved("D".into())]);
        // a smaller offset is noise; one the others share as much is no move either
        let mut m = Monitor::new(WINDOW_NS);
        let out = run(&mut m, 3, |w| (100 * w, 0, 0.5, 0.45));
        assert!(out.iter().all(|c| c.is_empty()), "{out:?}");
        let mut m = Monitor::new(WINDOW_NS);
        let shared = |w: u64| {
            vec![
                sensor("A", 100 * w, 5 * w, 0.8, 0.7),
                sensor("B", 100 * w, 4 * w, 0.8, 0.75),
                sensor("C", 90 * w, 6 * w, 0.9, 0.65),
                sensor("D", 100 * w, 0, 0.9, 0.9),
            ]
        };
        let out: Vec<_> = (0..=3)
            .map(|w| m.update(1_000 + w * WINDOW_NS, &shared(w)))
            .collect();
        assert!(out.iter().all(|c| c.is_empty()), "{out:?}");
    }

    /// A sensor checked too little, or whose counters started over, is not judged; one that
    /// misses only now and then stayed.
    #[test]
    fn too_few_checks_or_a_restart_say_nothing() {
        let mut m = Monitor::new(WINDOW_NS);
        let out = run(&mut m, 3, |w| (10 * w, 9 * w, 0.3, 0.1));
        assert!(out.iter().all(|c| c.is_empty()), "{out:?}");
        let mut m = Monitor::new(WINDOW_NS);
        // the counters fall back at window 2: a tracker started over
        let out = run(&mut m, 3, |w| {
            if w == 2 {
                (50, 45, 0.3, 0.1)
            } else {
                (100 * w, 80 * w, 0.3, 0.1)
            }
        });
        assert!(out[2].is_empty(), "{out:?}");
        let mut m = Monitor::new(WINDOW_NS);
        let out = run(&mut m, 3, |w| (100 * w, 10 * w, 0.4, 0.2));
        assert!(out.iter().all(|c| c.is_empty()), "{out:?}");
    }

    /// Without two sensors that agree with each other there is no one to judge against: unsure,
    /// said each window, and no sensor is judged moved.
    #[test]
    fn two_agreeing_sensors_are_needed_to_judge() {
        let mut m = Monitor::new(WINDOW_NS);
        let two = |w: u64| {
            vec![
                sensor("A", 100 * w, 5 * w, 0.3, 0.2),
                sensor("D", 100 * w, 80 * w, 0.3, 0.1),
            ]
        };
        let out: Vec<_> = (0..=2)
            .map(|w| m.update(1_000 + w * WINDOW_NS, &two(w)))
            .collect();
        assert_eq!(out[1], [Change::Unsure]);
        assert_eq!(out[2], [Change::Unsure]);
        assert!(!m.handed_over("D"));
    }

    /// Once released, a sensor is judged again, from the next full window of its checks: the
    /// counters it carries from before say nothing. Released with a cooldown, it waits.
    #[test]
    fn a_released_sensor_is_judged_again_from_fresh_checks() {
        let mut m = Monitor::new(WINDOW_NS);
        let moved = |w: u64| (100 * w, 80 * w, 0.3, 0.1);
        run(&mut m, 2, moved);
        assert!(m.handed_over("D"));
        m.released("D");
        assert!(!m.handed_over("D"));
        // D agrees from now on: its old misses must not count against it
        let fine = |w: u64| (100 * w, 160, 0.3, 0.1);
        let out: Vec<_> = (3..=6)
            .map(|w| m.update(1_000 + w * WINDOW_NS, &room(w, fine)))
            .collect();
        assert!(out.iter().all(|c| c.is_empty()), "{out:?}");
        // moved again: judged from the second full window on
        let out: Vec<_> = (7..=9)
            .map(|w| {
                m.update(
                    1_000 + w * WINDOW_NS,
                    &room(w, |w| (100 * w, 160 + 80 * (w - 6), 0.3, 0.1)),
                )
            })
            .collect();
        assert_eq!(
            out,
            [vec![], vec![Change::Moved("D".into())], vec![]],
            "{out:?}"
        );
        m.released_until("D", 1_000 + 12 * WINDOW_NS);
        let out: Vec<_> = (10..=13)
            .map(|w| {
                m.update(
                    1_000 + w * WINDOW_NS,
                    &room(w, |w| (100 * w, 80 * w, 0.3, 0.1)),
                )
            })
            .collect();
        assert_eq!(
            out,
            [vec![], vec![], vec![], vec![Change::Moved("D".into())]],
            "{out:?}"
        );
    }

    /// A pose placed again where the sensor was calibrated is a false alarm.
    #[test]
    fn a_pose_near_the_calibrated_one_is_a_false_alarm() {
        let pose = Isometry3::translation(1.0, 2.0, 3.0);
        assert!(false_alarm(&pose, &pose));
        assert!(false_alarm(
            &pose,
            &(Translation3::new(0.002, 0.0, 0.0) * pose)
        ));
        assert!(!false_alarm(
            &pose,
            &(Translation3::new(0.005, 0.0, 0.0) * pose)
        ));
        let turned = Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.01, 0.0)),
        ) * pose;
        assert!(!false_alarm(&pose, &turned));
    }

    use crate::calib::synth::{self, Rng};
    use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

    /// `n` probes of a Touch moving through its `motion` from `k0` on, seen by a sensor at
    /// `truth` with the engine's labels (some wrong, some missing).
    fn probes(
        truth: &Isometry3<f64>,
        motion: &synth::Motion,
        k0: usize,
        n: usize,
        rng: &mut Rng,
    ) -> Vec<Probe> {
        let model = synth::camera_model();
        let leds = synth::touch_leds();
        (k0..k0 + n)
            .map(|k| {
                let w = motion.pose(k as f64 * 0.05);
                let (mut blobs, mut labels) = (Vec::new(), Vec::new());
                for (i, l) in leds.iter().enumerate() {
                    let pc = truth.inverse() * (w * Point3::from(l.pos));
                    let nrm = truth.inverse().rotation * (w.rotation * l.dir);
                    if nrm.dot(&(-pc.coords)) / pc.coords.norm() <= 0.3 {
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
                    camera: 7,
                    hand: 1,
                    device_id: 1,
                    world_from_model: w,
                    pos_sd: 5e-4,
                    blobs,
                    labels,
                }
            })
            .collect()
    }

    fn fit_at(world_from_cam: Isometry3<f64>, median_px: f64) -> Fit {
        Fit {
            world_from_cam,
            median_px,
            pairs: 400,
            frames: 300,
            label_share: 0.9,
        }
    }

    /// A sensor that moved is placed from two batches of probes that agree: the first fit waits,
    /// the second confirms it, and the pose fitted on both is within a few millimetres.
    #[test]
    fn two_agreeing_batches_place_the_sensor() {
        let truth = synth::look_at(
            Vector3::new(1.9, 2.2, -1.6),
            Vector3::new(0.0, 1.0, 0.2),
            false,
        );
        let model = synth::camera_model();
        let leds = synth::touch_leds();
        let mut rng = Rng::new(4);
        let motion = synth::Motion::random(&mut rng, Vector3::new(0.0, 1.0, 0.0));
        let mut r = Reposer::default();
        r.watch(7);
        assert!(r.batch().is_none());
        // frames in which the sensor sees no LED carry no labels and are not kept
        let mut k = 0;
        while r.gathering[&7].probes.len() < BATCH_PROBES {
            for p in probes(&truth, &motion, k, 50, &mut rng) {
                r.add(p);
            }
            k += 50;
        }
        let job = r.batch().expect("a first batch");
        assert!(
            job.earlier.is_empty() && r.batch().is_none(),
            "one solve at a time"
        );
        let first = job.fresh.len();
        assert_eq!(
            r.gathered(7),
            first,
            "a batch taken still counts as gathered"
        );
        let solved = solve(&model, [None, Some(&leds)], &job).unwrap();
        assert_eq!(r.solved(job, Ok(solved)), Outcome::Nothing);
        let mut k = 1000;
        while r.gathering[&7].probes.len() < BATCH_PROBES {
            for p in probes(&truth, &motion, k, 50, &mut rng) {
                r.add(p);
            }
            k += 50;
        }
        let job = r.batch().expect("a second batch");
        assert_eq!(job.earlier.len(), first);
        let solved = solve(&model, [None, Some(&leds)], &job).unwrap();
        match r.solved(job, Ok(solved)) {
            Outcome::Placed { camera: 7, fit } => {
                let d = fit.world_from_cam.inverse() * truth;
                assert!(
                    d.translation.vector.norm() < 2e-3 && d.rotation.angle() < 0.1f64.to_radians(),
                    "{d:?}"
                );
                assert!(
                    fit.median_px < 0.5 && fit.frames > 500 && fit.label_share > 0.6,
                    "{fit:?}"
                );
            }
            other => panic!("{other:?}"),
        }
    }

    /// A loose fit, a failed solve, or two fits that disagree are not taken, and enough of them
    /// give the sensor up; a sensor not watched, a probe without labels, and one of an uncertain
    /// pose are dropped.
    #[test]
    fn loose_or_disagreeing_fits_are_not_taken() {
        let pose = Isometry3::translation(1.0, 2.0, 3.0);
        let job = |camera| Job {
            camera,
            fresh: Vec::new(),
            earlier: Vec::new(),
        };
        let mut r = Reposer::default();
        r.watch(1);
        assert_eq!(r.solved(job(1), Err("no pose".into())), Outcome::Nothing);
        let loose = Solved {
            fresh: fit_at(pose, 2.0),
            all: fit_at(pose, 2.0),
        };
        assert_eq!(r.solved(job(1), Ok(loose)), Outcome::Nothing);
        let mut unlabelled = Solved {
            fresh: fit_at(pose, 0.3),
            all: fit_at(pose, 0.3),
        };
        unlabelled.fresh.label_share = 0.3;
        assert_eq!(
            r.solved(job(1), Ok(unlabelled)),
            Outcome::Nothing,
            "few labels fit"
        );
        assert_eq!(r.gathering[&1].attempts, 3);
        let tight = |p: Isometry3<f64>| Solved {
            fresh: fit_at(p, 0.3),
            all: fit_at(p, 0.3),
        };
        assert_eq!(
            r.solved(job(1), Ok(tight(pose))),
            Outcome::Nothing,
            "a first fit waits"
        );
        let far = Translation3::new(0.05, 0.0, 0.0) * pose;
        assert_eq!(
            r.solved(job(1), Ok(tight(far))),
            Outcome::Nothing,
            "disagreeing fits"
        );
        let turned = Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.02, 0.0)),
        ) * far;
        assert_eq!(r.solved(job(1), Ok(tight(turned))), Outcome::Nothing);
        // the sixth failure: given up
        match r.solved(job(1), Err("no pose".into())) {
            Outcome::GaveUp {
                camera: 1,
                attempts: 6,
                ..
            } => {}
            other => panic!("{other:?}"),
        }
        assert!(r.given_up(1));
        let mut p = Probe {
            camera: 1,
            hand: 1,
            device_id: 1,
            world_from_model: pose,
            pos_sd: 5e-4,
            blobs: vec![Vector2::new(1.0, 1.0)],
            labels: vec![1 << 8],
        };
        r.add(p.clone());
        r.watch(2);
        p.camera = 2;
        r.add(p.clone());
        p.labels = vec![-1];
        r.add(p.clone());
        p.labels = vec![1 << 8];
        p.pos_sd = 0.01;
        r.add(p.clone());
        p.camera = 3;
        p.pos_sd = 5e-4;
        r.add(p);
        assert_eq!(r.gathering[&1].probes.len(), 0);
        assert_eq!(r.gathering[&2].probes.len(), 1);
        assert!(!r.watching(3));
    }

    /// A controller at rest adds one probe, however long it rests; a batch needs it to have moved
    /// across two directions and turned; and the oldest probes go once too many are kept.
    #[test]
    fn probes_are_thinned_and_a_batch_needs_variety() {
        let mut r = Reposer::default();
        r.watch(1);
        let at = |pose: Isometry3<f64>| Probe {
            camera: 1,
            hand: 1,
            device_id: 1,
            world_from_model: pose,
            pos_sd: 5e-4,
            blobs: vec![Vector2::new(1.0, 1.0)],
            labels: vec![1 << 8],
        };
        for _ in 0..50 {
            r.add(at(Isometry3::translation(0.0, 1.0, 0.0)));
        }
        assert_eq!(r.gathered(1), 1, "a controller at rest");
        // along one line (each step more than the thinning), turning: not varied enough
        for k in 1..=BATCH_PROBES {
            let turn = UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.002 * k as f64, 0.0));
            r.add(at(Isometry3::from_parts(
                Translation3::new(0.011 * k as f64, 1.0, 0.0),
                turn,
            )));
        }
        assert!(
            r.gathered(1) > BATCH_PROBES && r.batch().is_none(),
            "one line of positions"
        );
        // across a second direction: a batch
        for k in 1..=60 {
            r.add(at(Isometry3::translation(0.3, 1.0 + 0.003 * k as f64, 0.0)));
        }
        let job = r.batch().expect("varied now");
        assert!(job.fresh.len() > BATCH_PROBES);
        // the store is capped
        r.watch(2);
        for k in 0..KEEP_PROBES + 50 {
            let mut p = at(Isometry3::translation(0.02 * k as f64, 1.0, 0.0));
            p.camera = 2;
            r.add(p);
        }
        assert_eq!(r.gathering[&2].probes.len(), KEEP_PROBES);
        assert_eq!(r.gathered(2), KEEP_PROBES + 50, "kept counts them all");
    }
}
