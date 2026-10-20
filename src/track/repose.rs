// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Moved sensors, found while the controllers are tracked and placed again from the sensors that
//! stayed, with nothing for the person to do. Matched LEDs and engine-labelled sightings check
//! each sensor against the pose the others set for the same hand (`tracker::CameraAgreement`).
//! Unlabelled misses may be an occluded hand, so they request a bounded inspection without
//! removing the sensor from tracking. Only observed disagreement over two windows hands it to
//! the reposer, whose labelled probes place it anew (`refine::place_sensor`, `refine::refine_sensor`).

use std::collections::{BTreeMap, BTreeSet};

use nalgebra::Isometry3;

use super::epochs::{Checks, Evidence};
pub use super::pipeline::IdentityEvidence;
use super::pipeline::Probe;
use super::refine::{place_sensor, refine_sensor};
use super::tracker::CameraAgreement;
use crate::calib::camera::CameraModel;
use crate::calib::capture::Led;

/// Sensors are judged by what their checks said over this long.
pub const WINDOW_NS: u64 = 5_000_000_000;
/// A hand checked fewer times than this in a window cannot judge a sensor's pose.
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
/// A non-demoting label inspection requests restoration after this long; its native search also
/// has this deadline. The slot stays occupied until the native side acknowledges restoration.
/// The next inspection of that sensor waits `COOLDOWN_NS` after an agreeing or inconclusive
/// outcome, or the shorter starved retry (`STARVED_RETRY_NS`) after a starved one. One stopped
/// for another sensor's movement verdict has no outcome and waits only for that sensor's
/// placement search; each such stop takes a new verdict. Native cold searches can take longer
/// than a frame; their nominal duty is at most 30 / 630 per sensor when its one long-analysis
/// thread is saturated, without cooling fresh matched evidence, and at most 30 / 90 at a
/// starved run's first retry, whose wait then doubles up to `COOLDOWN_NS`.
pub const INSPECTION_NS: u64 = 30_000_000_000;
/// An inspection, or a moved sensor's probation, that found the controllers but never saw one
/// turn enough on its own to be identified is no verdict: the sensor is looked at again after
/// this, doubling while that lasts up to `COOLDOWN_NS`. A long still stretch then costs a
/// sensor's long search no more than an ordinary cooldown does.
pub const STARVED_RETRY_NS: u64 = 60_000_000_000;

/// The advice while fewer than two sensors agree with each other (`Change::Unsure`), for the
/// daemon's log and `cv1touch doctor` alike.
pub const UNSURE_ADVICE: &str = "cv1touch is still checking whether a sensor moved: fewer than two sensors agree on where the controllers are. Keep the controllers where the sensors can see them; if it stays unsure for several minutes, run cv1touch setup";

/// What a window of checks said about a sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Standing {
    /// Too few checks, or no two other sensors to compare with.
    Unknown,
    Stayed,
    Moved,
}

/// What a movement verdict rested on, for each hand that decided it (`None`: that hand did
/// not). The tracker's counters run on from its last reset and are not this evidence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MovedBy {
    /// Two windows in a row judged it moved: the last window's evidence.
    Windows([Option<LastWindow>; 2]),
    /// Its label inspection accumulated these eligible observed checks and misses, over this
    /// many windows.
    Inspection([Option<(Checks, usize)>; 2]),
}

/// One hand's last window, when two windows in a row judged a sensor moved.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LastWindow {
    /// Its observed checks and misses.
    pub observed: Checks,
    /// The median residual and the length of the offset at its end, in pixels; `None` while
    /// none of that hand's LEDs has matched.
    pub residual_px: Option<(f64, f64)>,
}

/// What the monitor found at the end of a window.
#[derive(Debug, Clone, PartialEq)]
pub enum Change {
    /// Ask the engine for labels without removing this sensor from tracking.
    Inspect(String),
    /// An inspected sensor's observed LEDs agree with its current pose.
    Agreed(String),
    /// The inspection did not establish movement in time: stop searching, keeping the pose.
    Unconfirmed(String),
    /// The inspection found the controllers too still, or moving together, to identify them:
    /// stop searching, keeping the pose, and look again after `retry_ns`.
    Starved { serial: String, retry_ns: u64 },
    /// Sensor `moved` moved while `serial` was inspected: stop searching, keeping the pose.
    /// That is no outcome for `serial`, and no wait follows. `moved` is handed over once the
    /// engine acknowledges the restoration, unless that verdict is dropped first.
    Interrupted { serial: String, moved: String },
    /// The verdict that this sensor moved, waiting for another sensor's restoration, was
    /// dropped before it was handed over: an invalidation or overflow, of any sensor or hand,
    /// took back controller sightings, so every sensor's agreement is judged afresh. Fresh
    /// windows judge it again; said once.
    Dropped(String),
    /// The sensor moved: probe it, and place it again.
    Moved { serial: String, by: MovedBy },
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
    /// Each sensor's counters at the last window end.
    last: BTreeMap<String, [CameraAgreement; 2]>,
    /// Tombstones outlive inspections, until their asynchronous restoration is acknowledged.
    lifetimes: BTreeMap<String, [Evidence; 2]>,
    /// Windows of unlabelled misses in a row: reasons to inspect, never to demote.
    unseen: BTreeMap<String, [u32; 2]>,
    /// At most one sensor requests cold labels for its suspect hands, until then.
    inspecting: Option<Inspection>,
    /// Restoring a pose is asynchronous: no new search until native acknowledgement.
    stopping: BTreeSet<String>,
    /// Only label requests wait after an inspection; fresh matched evidence can still judge it.
    inspection_cooling: BTreeMap<String, u64>,
    /// Windows in a row each sensor was judged moved.
    over: BTreeMap<String, u32>,
    /// A movement verdict that stopped another sensor's inspection, handed over once that
    /// restoration is acknowledged; judged no further meanwhile.
    deferred: Option<(String, MovedBy)>,
    /// The sensor whose deferred verdict was dropped since the last `update`.
    dropped: Option<String>,
    /// Handed over sensors, and whether their engine search is still running. A given-up sensor
    /// stays excluded after its engine pose is restored, without blocking other inspections.
    handed_over: BTreeMap<String, bool>,
    /// Sensors left alone until then (after a false alarm or a timeout).
    cooling: BTreeMap<String, u64>,
    /// Starved inspections or probations of each sensor since one could judge its identity.
    starved: BTreeMap<String, u32>,
}

#[derive(Debug, Clone)]
struct Inspection {
    serial: String,
    hands: [bool; 2],
    until_ns: u64,
    evidence: [IdentityEvidence; 2],
    observed: [InspectionChecks; 2],
}

/// Keep window membership as well as counts: retracting the sole contributor to a second
/// window must also retract that window from the movement decision.
#[derive(Debug, Clone, Default)]
struct InspectionChecks {
    current: Evidence,
    previous: BTreeMap<u64, Checks>,
    windows: BTreeMap<u64, BTreeMap<u64, Checks>>,
}

impl InspectionChecks {
    fn refresh(&mut self, evidence: &Evidence) {
        self.current = evidence.clone();
        for window in self.windows.values_mut() {
            window.retain(|&epoch, _| evidence.permits(epoch));
        }
        self.windows.retain(|_, entries| !entries.is_empty());
    }

    fn window(&mut self, now: u64, peers: bool) -> (Checks, usize) {
        for (&epoch, item) in &self.current.epochs {
            let old = self
                .previous
                .insert(epoch, item.observed)
                .unwrap_or_default();
            let new = item.observed.since(old);
            if peers && self.current.permits(epoch) && new.checks != 0 {
                self.windows
                    .entry(now)
                    .or_default()
                    .entry(epoch)
                    .or_default()
                    .add(new);
            }
        }
        self.previous
            .retain(|&epoch, _| self.current.permits(epoch));
        let mut total = Checks::default();
        for window in self.windows.values() {
            for checks in window.values() {
                total.add(*checks);
            }
        }
        (total, self.windows.len())
    }
}

impl Monitor {
    pub fn new(window_ns: u64) -> Monitor {
        Monitor {
            window_ns,
            ..Default::default()
        }
    }

    /// The sensor under label inspection, if any.
    pub fn inspecting(&self) -> Option<&str> {
        self.inspecting.as_ref().map(|i| i.serial.as_str())
    }

    /// What the engine reported about `serial`'s hands since its inspection or probing began.
    /// Evidence that could judge identity ends a run of starved retries.
    pub fn identity_evidence(&mut self, serial: &str, evidence: [IdentityEvidence; 2]) {
        if let Some(i) = self.inspecting.as_mut().filter(|i| i.serial == serial) {
            i.evidence = evidence;
        }
        if evidence.iter().any(|e| e.judgeable) {
            self.starved.remove(serial);
        }
    }

    /// Apply lifecycle changes or state even after inspection ends; tombstones already seen
    /// change nothing. Mixed residuals and their window baselines cannot be separated by epoch.
    /// A changed peer can also have supported another sensor's judgment, so pending streaks
    /// restart together, and a deferred verdict goes with them (`Change::Dropped`). Applied
    /// verdicts stay final.
    pub fn lifecycle(&mut self, serial: &str, states: &[Evidence; 2]) {
        let lifetimes = self.lifetimes.entry(serial.to_string()).or_default();
        let mut invalidated = false;
        for (old, new) in lifetimes.iter_mut().zip(states) {
            invalidated |= new.invalidates(old);
            let before = old.forgotten_through;
            old.lifecycle(new);
            invalidated |= old.forgotten_through > before;
        }
        if invalidated {
            self.over.clear();
            self.unseen.clear();
            self.last.clear();
            if let Some((serial, _)) = self.deferred.take() {
                self.dropped = Some(serial);
            }
        }
        if let Some(i) = self.inspecting.as_mut() {
            if i.serial == serial {
                for (observed, hand) in i.observed.iter_mut().zip(lifetimes) {
                    let mut current = observed.current.clone();
                    current.lifecycle(hand);
                    observed.refresh(&current);
                }
            } else if invalidated {
                // The invalidated sensor may have been a confirming peer. Its old eligibility
                // cannot support retained windows, nor can their checks be counted again.
                for observed in &mut i.observed {
                    observed.windows.clear();
                    observed.previous = observed
                        .current
                        .epochs
                        .iter()
                        .map(|(&epoch, item)| (epoch, item.observed))
                        .collect();
                }
            }
        }
    }

    /// Refresh epoch-tagged counts before judging a window. Normal ends keep their checks;
    /// invalidations remove them even when no new camera frame reaches this consumer.
    pub fn observed_evidence(&mut self, serial: &str, evidence: &[Evidence; 2]) {
        self.lifecycle(serial, evidence);
        if let Some(i) = self.inspecting.as_mut().filter(|i| i.serial == serial) {
            for (hand, observed) in i.observed.iter_mut().enumerate() {
                let mut current = evidence[hand].clone();
                current.lifecycle(&self.lifetimes[serial][hand]);
                observed.refresh(&current);
            }
        }
    }

    /// How long a starved `serial` waits before it is looked at again; each starved retry in a
    /// row doubles it, up to `COOLDOWN_NS`.
    pub fn starved_retry(&mut self, serial: &str) -> u64 {
        let run = self.starved.entry(serial.to_string()).or_default();
        let retry = (STARVED_RETRY_NS << (*run).min(4)).min(COOLDOWN_NS);
        *run = run.saturating_add(1);
        retry
    }

    /// The hands whose missing sightings prompted this label inspection.
    pub fn inspection_hands(&self, serial: &str) -> Option<[bool; 2]> {
        self.inspecting
            .as_ref()
            .filter(|i| i.serial == serial)
            .map(|i| i.hands)
    }

    /// The sensor `serial` is tracked with again: judge it afresh from now on, from the next
    /// full window of its checks.
    pub fn released(&mut self, serial: &str) {
        if self.handed_over.remove(serial) == Some(true) {
            self.stopping.insert(serial.to_string());
        }
        self.over.remove(serial);
        self.last.remove(serial);
        self.unseen.remove(serial);
        self.inspection_cooling.remove(serial);
        // A waiting verdict's sensor is never handed over, so never released: only a
        // retraction drops that verdict.
        let waiting = self.deferred.as_ref().is_some_and(|(s, _)| s == serial);
        debug_assert!(!waiting, "{serial} released while its verdict waits");
        if waiting {
            self.deferred = None;
        }
        if self.inspecting.as_ref().is_some_and(|i| i.serial == serial) {
            self.inspecting = None;
            self.stopping.insert(serial.to_string());
        }
    }

    /// As `released`, and the sensor is left alone until `until_ns`.
    pub fn released_until(&mut self, serial: &str, until_ns: u64) {
        self.released(serial);
        self.cooling.insert(serial.to_string(), until_ns);
    }

    /// Whether `serial` was handed over and not released.
    pub fn handed_over(&self, serial: &str) -> bool {
        self.handed_over.contains_key(serial)
    }

    /// Restoration was requested after giving up. Keep the sensor out of judgments, and hold
    /// the search slot until the native side acknowledges its restored pose.
    pub fn search_finished(&mut self, serial: &str) {
        if let Some(searching) = self.handed_over.get_mut(serial) {
            *searching = false;
        }
        self.stopping.insert(serial.to_string());
    }

    /// The engine has applied its restored pose and stopped the preceding long search. Its
    /// inspection is over and every notice of it delivered: its tombstones retire with the
    /// pipeline's (`Pipeline::retire_identities`).
    pub fn engine_restored(&mut self, serial: &str) {
        self.stopping.remove(serial);
        if let Some(lifetimes) = self.lifetimes.get_mut(serial) {
            for hand in lifetimes {
                hand.retire();
            }
        }
    }

    /// Sensors whose restored engine pose is not acknowledged yet: no search starts until then.
    pub fn restoring(&self) -> &BTreeSet<String> {
        &self.stopping
    }

    /// `serial`'s tombstones and watermark as the monitor holds them.
    pub fn identity_lifetimes(&self, serial: &str) -> [Evidence; 2] {
        self.lifetimes.get(serial).cloned().unwrap_or_default()
    }

    /// Called as often as convenient with every sensor's agreement; judges at the end of each
    /// window, from what the checks since the last one said. A deferred verdict is handed over
    /// at the first call after the restoration it waited for is acknowledged; one dropped
    /// meanwhile is reported at the next call.
    pub fn update(
        &mut self,
        now_ns: u64,
        agreement: &[(String, [CameraAgreement; 2])],
    ) -> Vec<Change> {
        if self.window_end == 0 {
            self.window_end = now_ns + self.window_ns;
            self.remember(agreement);
            return Vec::new();
        }
        let mut changes = Vec::new();
        if let Some(serial) = self.dropped.take() {
            changes.push(Change::Dropped(serial));
        }
        // The restoration it waited for held the search slot, so nothing else has taken it.
        if self.stopping.is_empty() {
            if let Some((serial, by)) = self.deferred.take() {
                self.handed_over.insert(serial.clone(), true);
                changes.push(Change::Moved { serial, by });
            }
        }
        if now_ns < self.window_end {
            if let Some(serial) = self
                .inspecting
                .as_ref()
                .filter(|i| now_ns >= i.until_ns)
                .map(|i| i.serial.clone())
            {
                changes.push(self.inspected(&serial, now_ns, false));
            }
            return changes;
        }
        self.window_end = now_ns + self.window_ns;
        self.cooling.retain(|_, until| *until > now_ns);
        self.inspection_cooling.retain(|_, until| *until > now_ns);
        let windows = self.windows(agreement);
        let per_hand = [Self::judge(&windows, 0), Self::judge(&windows, 1)];
        let peers = per_hand.each_ref().map(|(standings, _)| {
            standings
                .iter()
                .filter(|(_, standing, _)| *standing == Standing::Stayed)
                .count()
        });
        let unsure = (0..2).any(|hand| per_hand[hand].1 && peers[hand] < 2);
        let standings: Vec<_> = windows
            .iter()
            .enumerate()
            .map(|(i, (serial, _))| {
                let a = per_hand[0].0[i];
                let b = per_hand[1].0[i];
                let standing = if a.1 == Standing::Moved || b.1 == Standing::Moved {
                    Standing::Moved
                } else if a.1 == Standing::Stayed || b.1 == Standing::Stayed {
                    Standing::Stayed
                } else {
                    Standing::Unknown
                };
                (serial.as_str(), standing, a.2.max(b.2))
            })
            .collect();
        self.remember(agreement);
        if unsure {
            changes.push(Change::Unsure);
        }
        let mut worst: Option<(&str, f64, MovedBy)> = None;
        for (index, (serial, _, badness)) in standings.iter().enumerate() {
            let over = self.over.entry(serial.to_string()).or_default();
            let inspected = self
                .inspecting
                .as_ref()
                .is_some_and(|i| i.serial == *serial);
            // The hands this window judged moved, with the evidence it judged by. For
            // the inspected sensor, the miss rule uses retained epochs below; the small-bias
            // rule still needs two windows and cannot use misses as a shortcut.
            let deciding = [0, 1].map(|hand| {
                let a = windows[index].1[hand];
                (per_hand[hand].0[index].1 == Standing::Moved
                    && (!inspected
                        || a.observed_misses as f64 / a.observed_checks.max(1) as f64
                            <= STAYED_MISS_SHARE))
                    .then(|| LastWindow {
                        observed: Checks {
                            checks: a.observed_checks,
                            misses: a.observed_misses,
                        },
                        residual_px: (a.samples != 0).then(|| (a.median_px, a.bias_px.norm())),
                    })
            });
            if deciding.iter().any(Option::is_some) {
                *over += 1;
                if *over >= CONSECUTIVE && worst.is_none_or(|(_, b, _)| *badness > b) {
                    worst = Some((serial, *badness, MovedBy::Windows(deciding)));
                }
            } else {
                *over = 0;
            }
        }
        if let Some(inspection) = self.inspecting.as_mut() {
            let mut deciding = [None; 2];
            for (hand, (standings, _)) in per_hand.iter().enumerate() {
                let other_peers = standings
                    .iter()
                    .filter(|(serial, standing, _)| {
                        *serial != inspection.serial && *standing == Standing::Stayed
                    })
                    .count();
                let (total, count) = inspection.observed[hand].window(now_ns, other_peers >= 2);
                if other_peers >= 2
                    && total.checks >= MIN_CHECKS * u64::from(CONSECUTIVE)
                    && count >= CONSECUTIVE as usize
                    && total.misses as f64 / total.checks as f64 >= MOVED_MISS_SHARE
                {
                    deciding[hand] = Some((total, count));
                }
            }
            let badness = deciding
                .iter()
                .flatten()
                .map(|(total, _)| 1.0 + total.misses as f64 / total.checks as f64)
                .reduce(f64::max);
            if let Some(badness) = badness.filter(|b| worst.is_none_or(|(_, w, _)| *b > w)) {
                if let Some((serial, _)) = windows.iter().find(|(s, _)| *s == inspection.serial) {
                    worst = Some((serial.as_str(), badness, MovedBy::Inspection(deciding)));
                }
            }
        }
        // While a restoration is unacknowledged, a running inspection goes on and the verdict
        // waits. A verdict on another sensor than the inspected one stops that inspection,
        // without an outcome for it, and waits for its restoration.
        if let Some((serial, _, by)) = worst.filter(|_| {
            self.stopping.is_empty() && !self.handed_over.values().any(|active| *active)
        }) {
            self.over.remove(serial);
            if let Some(other) = self.inspecting.take_if(|i| i.serial != serial) {
                self.stopping.insert(other.serial.clone());
                self.deferred = Some((serial.to_string(), by));
                changes.push(Change::Interrupted {
                    serial: other.serial,
                    moved: serial.to_string(),
                });
            } else {
                self.inspecting = None;
                self.handed_over.insert(serial.to_string(), true);
                changes.push(Change::Moved {
                    serial: serial.to_string(),
                    by,
                });
            }
        }
        if let Some(i) = self.inspecting.as_ref() {
            let agreed = (0..2).all(|hand| {
                !i.hands[hand]
                    || per_hand[hand]
                        .0
                        .iter()
                        .any(|(s, standing, _)| *s == i.serial && *standing == Standing::Stayed)
            });
            if agreed || now_ns >= i.until_ns {
                let serial = i.serial.clone();
                changes.push(self.inspected(&serial, now_ns, agreed));
            }
        }
        let mut inspect = None;
        for (i, (serial, hands)) in windows.iter().enumerate() {
            if self.inspection_cooling.contains_key(serial) {
                self.unseen.remove(serial);
                continue;
            }
            let over = self.unseen.entry(serial.clone()).or_default();
            let requested = [0, 1].map(|hand| {
                let a = hands[hand];
                let missing = per_hand[hand].0[i].1 == Standing::Unknown
                    && peers[hand] >= 2
                    && a.checks >= MIN_CHECKS
                    && a.observed_checks < MIN_CHECKS
                    && a.misses as f64 >= MOVED_MISS_SHARE * a.checks as f64;
                over[hand] = if missing {
                    over[hand].saturating_add(1)
                } else {
                    0
                };
                over[hand] >= CONSECUTIVE
            });
            if requested.iter().any(|r| *r)
                && !self.cooling.contains_key(serial)
                && inspect.is_none()
            {
                inspect = Some((serial.clone(), requested));
            }
        }
        if self.inspecting.is_none()
            && self.stopping.is_empty()
            && !self.handed_over.values().any(|searching| *searching)
        {
            if let Some((serial, hands)) = inspect {
                self.inspecting = Some(Inspection {
                    serial: serial.clone(),
                    hands,
                    until_ns: now_ns + INSPECTION_NS,
                    evidence: Default::default(),
                    observed: Default::default(),
                });
                changes.push(Change::Inspect(serial));
            }
        }
        changes
    }

    /// Ends the inspection of `serial`: its search stops and its label requests cool down, for
    /// a short retry only when the hands were seen but never identifiable.
    fn inspected(&mut self, serial: &str, now_ns: u64, agreed: bool) -> Change {
        let inspection = self.inspecting.take();
        let starved =
            !agreed && inspection.is_some_and(|i| IdentityEvidence::starved(&i.evidence, i.hands));
        self.stopping.insert(serial.to_string());
        self.unseen.remove(serial);
        let change = if agreed {
            Change::Agreed(serial.to_string())
        } else if starved {
            Change::Starved {
                serial: serial.to_string(),
                retry_ns: self.starved_retry(serial),
            }
        } else {
            Change::Unconfirmed(serial.to_string())
        };
        let cooldown = match &change {
            Change::Starved { retry_ns, .. } => *retry_ns,
            _ => {
                self.starved.remove(serial);
                COOLDOWN_NS
            }
        };
        self.inspection_cooling
            .insert(serial.to_string(), now_ns + cooldown);
        change
    }

    fn remember(&mut self, agreement: &[(String, [CameraAgreement; 2])]) {
        for (serial, a) in agreement {
            self.last.insert(serial.clone(), *a);
        }
    }

    fn windows(
        &self,
        agreement: &[(String, [CameraAgreement; 2])],
    ) -> Vec<(String, [CameraAgreement; 2])> {
        agreement
            .iter()
            .filter(|(serial, _)| {
                !self.handed_over.contains_key(serial)
                    && !self.cooling.contains_key(serial)
                    && self.deferred.as_ref().is_none_or(|(s, _)| s != serial)
            })
            .map(|(serial, a)| {
                let window = [0, 1].map(|hand| {
                    let a = &a[hand];
                    let mut window = *a;
                    let previous = self.last.get(serial).map(|old| &old[hand]).filter(|old| {
                        a.checks >= old.checks
                            && a.misses >= old.misses
                            && a.observed_checks >= old.observed_checks
                            && a.observed_misses >= old.observed_misses
                    });
                    if let Some(old) = previous {
                        window.checks -= old.checks;
                        window.misses -= old.misses;
                        window.observed_checks -= old.observed_checks;
                        window.observed_misses -= old.observed_misses;
                    } else {
                        // A tracker reset, or the first window after a release, says nothing.
                        window.checks = 0;
                        window.misses = 0;
                        window.observed_checks = 0;
                        window.observed_misses = 0;
                    }
                    window
                });
                (serial.clone(), window)
            })
            .collect()
    }

    /// Only actual correspondences judge a sensor. An absent hand can leave arbitrarily many
    /// unlabelled blobs without contradicting the pose the other sensors set.
    fn judge(
        windows: &[(String, [CameraAgreement; 2])],
        hand: usize,
    ) -> (Vec<(&str, Standing, f64)>, bool) {
        let seen: Vec<(&str, &CameraAgreement, u64, u64)> = windows
            .iter()
            .map(|(serial, a)| {
                let a = &a[hand];
                (serial.as_str(), a, a.observed_checks, a.observed_misses)
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
    /// The watch it was taken from: a job that outlives `Reposer::forget` cannot place, or
    /// count against, a later watch of the same sensor.
    watch: u64,
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
    watch: u64,
    probes: Vec<Probe>,
    /// Probes kept since the sensor was watched, batches included.
    kept: BTreeMap<(usize, u64), usize>,
    /// The pose of the last probe kept, for thinning.
    last_kept: Option<Isometry3<f64>>,
    /// A fit awaiting confirmation, with the probes it was fitted on.
    first: Option<Confirmation>,
    lifetimes: [Evidence; 2],
    /// Failed jobs retain every contributing epoch, including both disagreeing fits.
    attempts: Vec<BTreeSet<(usize, u64)>>,
    solving: bool,
    given_up: bool,
}

#[derive(Debug)]
struct Confirmation {
    fit: Fit,
    probes: Vec<Probe>,
    /// Keep provenance even while the confirmation job owns the earlier probes.
    epochs: BTreeSet<(usize, u64)>,
}

impl Confirmation {
    fn new(fit: Fit, probes: Vec<Probe>) -> Self {
        let epochs = probes.iter().map(|p| (p.hand, p.identity_epoch)).collect();
        Self {
            fit,
            probes,
            epochs,
        }
    }
}

/// Gathers a probed sensor's probes and, batch by batch, the fits that place it again: a fit is
/// taken when tight, and the pose is confirmed when a second batch lands within tolerance.
#[derive(Debug, Default)]
pub struct Reposer {
    gathering: BTreeMap<usize, Gathering>,
    watches: u64,
}

impl Reposer {
    /// Starts gathering the sensor `camera`'s probes (and starts over, if it was watched).
    pub fn watch(&mut self, camera: usize) {
        self.watches += 1;
        self.gathering.insert(
            camera,
            Gathering {
                watch: self.watches,
                ..Default::default()
            },
        );
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
        if !g.lifetimes[probe.hand].permits(probe.identity_epoch)
            || g.given_up
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
        *g.kept
            .entry((probe.hand, probe.identity_epoch))
            .or_default() += 1;
        if g.probes.len() >= KEEP_PROBES {
            g.probes.remove(0);
        }
        g.probes.push(probe);
    }

    /// How many probes a watched sensor has gathered since it was watched, batches included.
    pub fn gathered(&self, camera: usize) -> usize {
        self.gathering
            .get(&camera)
            .map_or(0, |g| g.kept.values().sum())
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
            watch: g.watch,
            fresh: std::mem::take(&mut g.probes),
            // the first fit's probes go along for one fit on all of them; the fit awaiting
            // confirmation keeps its pose and epoch provenance while the job owns its probes
            earlier: g
                .first
                .as_mut()
                .map(|first| std::mem::take(&mut first.probes))
                .unwrap_or_default(),
        })
    }

    /// Retract queued probes and saved fits; jobs keep their own provenance until completion.
    pub fn lifecycle(&mut self, camera: usize, states: &[Evidence; 2]) {
        let Some(g) = self.gathering.get_mut(&camera) else {
            return;
        };
        for (old, new) in g.lifetimes.iter_mut().zip(states) {
            old.lifecycle(new);
        }
        let before = g.kept.len();
        g.kept
            .retain(|&(hand, epoch), _| g.lifetimes[hand].permits(epoch));
        g.probes
            .retain(|p| g.lifetimes[p.hand].permits(p.identity_epoch));
        if g.kept.len() != before {
            g.last_kept = None;
        }
        g.attempts.retain(|epochs| {
            epochs
                .iter()
                .all(|&(hand, epoch)| g.lifetimes[hand].permits(epoch))
        });
        if g.first.as_ref().is_some_and(|first| {
            first
                .epochs
                .iter()
                .any(|&(hand, epoch)| !g.lifetimes[hand].permits(epoch))
        }) {
            g.first = None;
        }
    }

    /// What a job's result means for its sensor.
    pub fn solved(&mut self, job: Job, result: Result<Solved, String>) -> Outcome {
        let Some(g) = self
            .gathering
            .get_mut(&job.camera)
            .filter(|g| g.watch == job.watch)
        else {
            return Outcome::Nothing;
        };
        g.solving = false;
        if job
            .fresh
            .iter()
            .chain(&job.earlier)
            .any(|p| !g.lifetimes[p.hand].permits(p.identity_epoch))
        {
            if let Some(first) = g.first.as_mut() {
                first.probes = job.earlier;
            }
            return Outcome::Nothing;
        }
        // A failed earlier job may have consumed the saved fit's probe vector. The fit
        // still participates in confirmation, so retain its explicit provenance as well.
        let epochs = job
            .fresh
            .iter()
            .chain(&job.earlier)
            .map(|p| (p.hand, p.identity_epoch))
            .chain(
                g.first
                    .iter()
                    .flat_map(|first| first.epochs.iter().copied()),
            )
            .collect();
        let failed = |g: &mut Gathering, reason: String| {
            g.attempts.push(epochs);
            if g.attempts.len() >= MAX_ATTEMPTS as usize {
                g.given_up = true;
                g.first = None;
                Outcome::GaveUp {
                    camera: job.camera,
                    attempts: g.attempts.len() as u32,
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
        let Some(first) = g.first.take() else {
            g.first = Some(Confirmation::new(solved.fresh, job.fresh));
            return Outcome::Nothing;
        };
        let d = first.fit.world_from_cam.inverse() * solved.fresh.world_from_cam;
        let (moved, turned) = (d.translation.vector.norm(), d.rotation.angle());
        if moved <= CONFIRM_M && turned <= CONFIRM_RAD {
            g.attempts.clear();
            Outcome::Placed {
                camera: job.camera,
                fit: solved.all,
            }
        } else {
            // the newer fit is the one to confirm next
            g.first = Some(Confirmation::new(solved.fresh, job.fresh));
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
    ) -> (String, [CameraAgreement; 2]) {
        (
            serial.to_string(),
            [
                CameraAgreement {
                    median_px: median,
                    bias_px: Vector2::new(bias, 0.0),
                    samples: checks - misses,
                    checks,
                    misses,
                    observed_checks: checks,
                    observed_misses: misses,
                },
                CameraAgreement::default(),
            ],
        )
    }

    fn visibility(
        serial: &str,
        raw: (u64, u64),
        observed: (u64, u64),
        median: f64,
        bias: f64,
    ) -> (String, [CameraAgreement; 2]) {
        let mut item = sensor(serial, raw.0, raw.1, median, bias);
        item.1[0].observed_checks = observed.0;
        item.1[0].observed_misses = observed.1;
        item
    }

    /// `value` for `hand`, nothing for the other.
    fn on_hand<T>(hand: usize, value: T) -> [Option<T>; 2] {
        let mut hands = [None, None];
        hands[hand] = Some(value);
        hands
    }

    /// One hand's last window: observed checks, misses, and the residual and offset unless no
    /// LED matched.
    fn last_window(checks: u64, misses: u64, residual_px: Option<(f64, f64)>) -> LastWindow {
        LastWindow {
            observed: Checks { checks, misses },
            residual_px,
        }
    }

    /// A two-window verdict on these hands' last window.
    fn moved_in_windows(serial: &str, hands: [Option<LastWindow>; 2]) -> Change {
        Change::Moved {
            serial: serial.into(),
            by: MovedBy::Windows(hands),
        }
    }

    /// An inspection's verdict on these hands: (observed checks, misses, windows).
    fn moved_on_inspection(serial: &str, hands: [Option<(u64, u64, usize)>; 2]) -> Change {
        Change::Moved {
            serial: serial.into(),
            by: MovedBy::Inspection(
                hands.map(|h| {
                    h.map(|(checks, misses, windows)| (Checks { checks, misses }, windows))
                }),
            ),
        }
    }

    /// The agreement after `windows` windows: three sensors that stayed, and D as `d` makes it
    /// from the window's checks and misses.
    fn room(
        windows: u64,
        d: impl Fn(u64) -> (u64, u64, f64, f64),
    ) -> Vec<(String, [CameraAgreement; 2])> {
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
        assert_eq!(
            out[2],
            [moved_in_windows(
                "D",
                on_hand(0, last_window(100, 80, Some((0.3, 0.1))))
            )]
        );
        assert_eq!(out[3], []);
        assert_eq!(out[4], []);
        assert!(m.handed_over("D") && !m.handed_over("A"));
    }

    /// A sensor whose residual carries an offset the others do not moved a little.
    #[test]
    fn a_nudged_sensor_stands_out_by_its_offset() {
        let mut m = Monitor::new(WINDOW_NS);
        let out = run(&mut m, 2, |w| (100 * w, 0, 0.9, 0.9));
        assert_eq!(
            out[2],
            [moved_in_windows(
                "D",
                on_hand(0, last_window(100, 0, Some((0.9, 0.9))))
            )]
        );
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

    /// Without two agreeing sensors there is no one to judge against. Actual labelled sightings
    /// disagree, so the uncertainty is reported, without judging which sensor moved.
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

    #[test]
    fn an_unseen_hand_is_inspected_even_when_the_other_hand_agrees() {
        for hand in 0..2 {
            let mut m = Monitor::new(WINDOW_NS);
            for w in 0..=2 {
                let mut agreement = vec![
                    sensor("A", 100 * w, 0, 0.3, 0.1),
                    sensor("B", 100 * w, 0, 0.3, 0.1),
                    sensor("C", 100 * w, 0, 0.3, 0.1),
                ];
                for (_, hands) in &mut agreement {
                    hands[1] = hands[0];
                }
                agreement[2].1[hand] = visibility("C", (100 * w, 100 * w), (0, 0), 0.0, 0.0).1[0];
                let want = if w == 2 {
                    vec![Change::Inspect("C".into())]
                } else {
                    vec![]
                };
                assert_eq!(
                    m.update(1_000 + w * WINDOW_NS, &agreement),
                    want,
                    "hand {hand}"
                );
            }
        }
    }

    #[test]
    fn another_hands_matches_do_not_finish_an_inspection() {
        for hand in 0..2 {
            let mut m = Monitor::new(WINDOW_NS);
            for w in 0u64..=4 {
                let mut agreement = vec![
                    sensor("A", 100 * w, 0, 0.3, 0.1),
                    sensor("B", 100 * w, 0, 0.3, 0.1),
                    visibility("C", (100 * w, 100 * w), (0, 0), 0.0, 0.0),
                ];
                for (_, hands) in &mut agreement {
                    hands[1] = hands[0];
                }
                agreement[2].1[1 - hand] = sensor("C", 100 * w.saturating_sub(2), 0, 0.3, 0.1).1[0];
                if w == 4 {
                    agreement[2].1[hand] = visibility("C", (400, 300), (100, 0), 0.3, 0.1).1[0];
                }
                let want = match w {
                    2 => vec![Change::Inspect("C".into())],
                    4 => vec![Change::Agreed("C".into())],
                    _ => vec![],
                };
                assert_eq!(
                    m.update(1_000 + w * WINDOW_NS, &agreement),
                    want,
                    "hand {hand}, window {w}"
                );
            }
        }
    }

    /// A five-second replay window: B tracked one hand to 0.21 px while missing the other;
    /// C matched nothing. Their missing sightings do not contradict A and B's matched LEDs.
    #[test]
    fn occluded_views_do_not_disagree_with_matched_leds() {
        let mut m = Monitor::new(WINDOW_NS);
        for w in 0u64..=4 {
            let agreement = vec![
                sensor("A", 222 * w, 0, 0.23, 0.15),
                visibility("B", (416 * w, 183 * w), (233 * w, 0), 0.21, 0.14),
                visibility("C", (428 * w, 428 * w), (0, 0), 0.45, 0.38),
            ];
            let want = if w == 2 {
                vec![Change::Inspect("C".into())]
            } else {
                vec![]
            };
            assert_eq!(m.update(1_000 + w * WINDOW_NS, &agreement), want);
            assert!(!m.handed_over("C"));
        }
    }

    /// Two adjacent replay windows: A and B saw the right hand; C's labelled misses concerned
    /// the left. Those right-hand matches cannot establish C's left-hand prior.
    #[test]
    fn agreement_on_one_hand_cannot_judge_another_hands_misses() {
        let mut m = Monitor::new(WINDOW_NS);
        let mut agreement: Vec<_> = ["A", "B", "C"].map(|s| sensor(s, 0, 0, 0.23, 0.15)).into();
        assert_eq!(m.update(1_000, &agreement), []);
        for (i, (a, right, hidden_left, labelled_left, hidden_right)) in
            [(187, 194, 186, 167, 141), (221, 233, 183, 233, 195)]
                .into_iter()
                .enumerate()
        {
            let checks = [
                [
                    visibility("A", (0, 0), (0, 0), 0.23, 0.15).1[0],
                    sensor("A", a, 0, 0.23, 0.15).1[0],
                ],
                [
                    visibility("B", (hidden_left, hidden_left), (0, 0), 0.21, 0.14).1[0],
                    sensor("B", right, 0, 0.21, 0.14).1[0],
                ],
                [
                    sensor("C", labelled_left, labelled_left, 0.45, 0.38).1[0],
                    visibility("C", (hidden_right, hidden_right), (0, 0), 0.45, 0.38).1[0],
                ],
            ];
            for (item, hands) in agreement.iter_mut().zip(checks) {
                for (old, new) in item.1.iter_mut().zip(hands) {
                    old.checks += new.checks;
                    old.misses += new.misses;
                    old.observed_checks += new.observed_checks;
                    old.observed_misses += new.observed_misses;
                }
            }
            let want = if i == 1 {
                vec![Change::Inspect("C".into())]
            } else {
                vec![]
            };
            assert_eq!(
                m.update(1_000 + (i as u64 + 1) * WINDOW_NS, &agreement),
                want
            );
            assert!(!m.handed_over("C"));
        }
    }

    /// The residual is a moving average of earlier matched frames. New misses alone cannot
    /// make that old residual evidence of disagreement in the current window.
    #[test]
    fn old_residuals_and_new_misses_do_not_mean_new_disagreement() {
        let mut m = Monitor::new(WINDOW_NS);
        for w in 0..=3 {
            let agreement = vec![
                visibility("A", (1_000 + 100 * w, 100 * w), (1_000, 0), 1.5, 1.2),
                visibility("B", (1_000 + 100 * w, 100 * w), (1_000, 0), 1.8, 1.4),
            ];
            assert_eq!(m.update(1_000 + w * WINDOW_NS, &agreement), []);
        }
    }

    #[test]
    fn fresh_matches_can_show_disagreement_without_two_agreeing_sensors() {
        let mut m = Monitor::new(WINDOW_NS);
        for w in 0..=3 {
            let agreement = vec![
                sensor("A", 100 * w, 0, 1.5, 1.2),
                sensor("B", 100 * w, 2 * w, 1.8, 1.4),
                sensor("C", 100 * w, 5 * w, 0.3, 0.1),
            ];
            let want = if w == 0 { vec![] } else { vec![Change::Unsure] };
            assert_eq!(m.update(1_000 + w * WINDOW_NS, &agreement), want);
        }
    }

    /// The matched residuals from the occlusion case remain a useful comparison for a real
    /// offset: with two steadily observed peers, D is still found after two windows.
    #[test]
    fn a_moved_sensor_still_stands_out_among_measured_residuals() {
        let mut m = Monitor::new(WINDOW_NS);
        for w in 0..=2 {
            let agreement = vec![
                sensor("A", 222 * w, 0, 0.23, 0.15),
                sensor("B", 233 * w, 0, 0.21, 0.14),
                sensor("D", 100 * w, 5 * w, 1.1, 0.95),
            ];
            let want = if w == 2 {
                vec![moved_in_windows(
                    "D",
                    on_hand(0, last_window(100, 5, Some((1.1, 0.95)))),
                )]
            } else {
                vec![]
            };
            assert_eq!(m.update(1_000 + w * WINDOW_NS, &agreement), want);
        }
    }

    #[test]
    fn unlabelled_misses_inspect_before_two_windows_of_observed_disagreement() {
        let mut m = Monitor::new(WINDOW_NS);
        for w in 0u64..=4 {
            let observed = 100 * w.saturating_sub(2);
            let agreement = vec![
                sensor("A", 222 * w, 0, 0.23, 0.15),
                sensor("B", 233 * w, 0, 0.21, 0.14),
                visibility("C", (100 * w, 100 * w), (observed, observed), 0.0, 0.0),
            ];
            let want = match w {
                2 => vec![Change::Inspect("C".into())],
                4 => vec![moved_on_inspection("C", on_hand(0, (200, 200, 2)))],
                _ => vec![],
            };
            let mut epochs: [Evidence; 2] = Default::default();
            epochs[0].count(
                1,
                Checks {
                    checks: observed,
                    misses: observed,
                },
            );
            m.observed_evidence("C", &epochs);
            assert_eq!(m.update(1_000 + w * WINDOW_NS, &agreement), want);
            assert_eq!(m.handed_over("C"), w == 4);
        }
    }

    /// Recorded LED frames with one camera's configured pose displaced by 0.4 m: two windows
    /// ask for labels, then two windows of actual labelled misses establish the movement.
    #[test]
    fn an_inspection_keeps_observed_checks_from_sparse_windows() {
        for hand in 0..2 {
            let mut monitor = Monitor::new(WINDOW_NS);
            for window in 0..=4 {
                let checks = window * 100;
                let observed = match window {
                    3 => 50,
                    4 => 80,
                    _ => 0,
                };
                let mut evidence = vec![
                    sensor("A", checks, 0, 0.1, 0.0),
                    sensor("B", checks, 0, 0.1, 0.0),
                    visibility("C", (checks, checks), (observed, observed), 0.0, 0.0),
                ];
                if hand == 1 {
                    for (_, hands) in &mut evidence {
                        hands.swap(0, 1);
                    }
                }
                let mut epochs: [Evidence; 2] = Default::default();
                epochs[hand].count(
                    1,
                    Checks {
                        checks: observed,
                        misses: observed,
                    },
                );
                monitor.observed_evidence("C", &epochs);
                let changes = monitor.update(1 + window * WINDOW_NS, &evidence);
                assert_eq!(
                    changes,
                    match window {
                        2 => vec![Change::Inspect("C".into())],
                        4 => vec![moved_on_inspection("C", on_hand(hand, (80, 80, 2)))],
                        _ => Vec::new(),
                    },
                    "hand {hand}, window {window}"
                );
            }
        }
    }

    #[test]
    fn only_invalidated_epochs_retract_an_inspections_evidence() {
        use super::super::epochs::State;
        for hand in 0..2 {
            for state in [State::Ended, State::Invalidated] {
                for (first, second) in [(50, 30), (80, 1)] {
                    let mut monitor = Monitor::new(WINDOW_NS);
                    let mut epochs: [Evidence; 2] = Default::default();
                    for window in 0..=5 {
                        let raw = window * 100;
                        let observed = match window {
                            0..=2 => 0,
                            3 => first,
                            4 => first + second,
                            _ => first + second + 30,
                        };
                        let mut agreement = vec![
                            sensor("A", raw, 0, 0.1, 0.0),
                            sensor("B", raw, 0, 0.1, 0.0),
                            visibility("C", (raw, raw), (observed, observed), 0.0, 0.0),
                        ];
                        if hand == 1 {
                            for (_, hands) in &mut agreement {
                                hands.swap(0, 1);
                            }
                        }
                        if window == 3 {
                            epochs[hand].count(
                                1,
                                Checks {
                                    checks: first,
                                    misses: first,
                                },
                            );
                            epochs[hand].note(1, State::Ended);
                        } else if window == 4 {
                            epochs[hand].count(
                                2,
                                Checks {
                                    checks: second,
                                    misses: second,
                                },
                            );
                            // The second window's checks have arrived, but the one-second
                            // consumer tick has not reached the window's decision yet.
                            monitor.observed_evidence("C", &epochs);
                            assert!(monitor.update(window * WINDOW_NS, &agreement).is_empty());
                            epochs[hand].note(2, state);
                            if state == State::Invalidated {
                                // A delayed frame or lifecycle notice cannot revive it.
                                epochs[hand].count(
                                    2,
                                    Checks {
                                        checks: 100,
                                        misses: 100,
                                    },
                                );
                                epochs[hand].note(2, State::Ended);
                            }
                        } else if window == 5 {
                            epochs[hand].count(
                                3,
                                Checks {
                                    checks: 30,
                                    misses: 30,
                                },
                            );
                        }
                        monitor.observed_evidence("C", &epochs);
                        let changes = monitor.update(1 + window * WINDOW_NS, &agreement);
                        let moved = window == 4 && state == State::Ended
                            || window == 5 && state == State::Invalidated;
                        assert_eq!(
                            changes,
                            if window == 2 {
                                vec![Change::Inspect("C".into())]
                            } else if moved {
                                // a normal end keeps both windows; an invalidation leaves the
                                // first, and the third epoch's window completes the evidence
                                let total = if state == State::Ended {
                                    first + second
                                } else {
                                    first + 30
                                };
                                vec![moved_on_inspection("C", on_hand(hand, (total, total, 2)))]
                            } else {
                                Vec::new()
                            },
                            "hand {hand}, {state:?}, {first}+{second}, window {window}"
                        );
                    }
                    assert!(monitor.handed_over("C"));
                    // An applied verdict is final; a later invalidation cannot undo it.
                    epochs[hand].note(1, State::Invalidated);
                    monitor.observed_evidence("C", &epochs);
                    assert!(monitor.handed_over("C"));
                }
            }
        }
    }

    #[test]
    fn invalidation_also_breaks_the_small_bias_streak() {
        use super::super::epochs::State;
        for hand in 0..2 {
            let mut monitor = Monitor::new(WINDOW_NS);
            let mut epochs: [Evidence; 2] = Default::default();
            for w in 0u64..=6 {
                let seen = 100 * w.saturating_sub(2);
                let mut agreement = vec![
                    sensor("A", 100 * w, 0, 0.1, 0.1),
                    sensor("B", 100 * w, 0, 0.1, 0.1),
                    visibility("C", (100 * w, 200.min(100 * w)), (seen, 0), 0.9, 0.9),
                ];
                if hand == 1 {
                    for (_, hands) in &mut agreement {
                        hands.swap(0, 1);
                    }
                }
                if w == 3 {
                    epochs[hand].count(
                        1,
                        Checks {
                            checks: 100,
                            misses: 0,
                        },
                    );
                }
                if w == 4 {
                    epochs[hand].note(1, State::Invalidated);
                }
                if w >= 4 {
                    epochs[hand].count(
                        2,
                        Checks {
                            checks: 100,
                            misses: 0,
                        },
                    );
                }
                monitor.observed_evidence("C", &epochs);
                assert_eq!(
                    monitor.update(1 + w * WINDOW_NS, &agreement),
                    match w {
                        2 => vec![Change::Inspect("C".into())],
                        6 => vec![moved_in_windows(
                            "C",
                            on_hand(hand, last_window(100, 0, Some((0.9, 0.9))))
                        )],
                        _ => vec![],
                    },
                    "hand {hand}, window {w}"
                );
            }
        }
    }

    #[test]
    fn lost_lifecycle_events_break_the_small_bias_streak() {
        use super::super::epochs::State;
        for hand in 0..2 {
            let mut monitor = Monitor::new(WINDOW_NS);
            let mut epochs: [Evidence; 2] = Default::default();
            for w in 0u64..=6 {
                let seen = 100 * w.saturating_sub(2);
                let mut agreement = vec![
                    sensor("A", 100 * w, 0, 0.1, 0.1),
                    sensor("B", 100 * w, 0, 0.1, 0.1),
                    visibility("C", (100 * w, 200.min(100 * w)), (seen, 0), 0.9, 0.9),
                ];
                if hand == 1 {
                    for (_, hands) in &mut agreement {
                        hands.swap(0, 1);
                    }
                }
                if w == 3 {
                    epochs[hand].count(
                        1,
                        Checks {
                            checks: 100,
                            misses: 0,
                        },
                    );
                }
                if w == 4 {
                    // The invalidation is lost: overflow leaves only a watermark behind it.
                    epochs[hand].note(1, State::Invalidated);
                    for epoch in 2..=1025 {
                        epochs[hand].count(
                            epoch,
                            Checks {
                                checks: 1,
                                misses: 0,
                            },
                        );
                    }
                    assert_eq!(epochs[hand].forgotten_through, 1);
                    assert!(epochs[hand]
                        .epochs
                        .values()
                        .all(|e| e.state != State::Invalidated));
                }
                if w >= 4 {
                    epochs[hand].count(
                        2049,
                        Checks {
                            checks: 100,
                            misses: 0,
                        },
                    );
                }
                monitor.observed_evidence("C", &epochs);
                assert_eq!(
                    monitor.update(1 + w * WINDOW_NS, &agreement),
                    match w {
                        2 => vec![Change::Inspect("C".into())],
                        6 => vec![moved_in_windows(
                            "C",
                            on_hand(hand, last_window(100, 0, Some((0.9, 0.9))))
                        )],
                        _ => vec![],
                    },
                    "hand {hand}, window {w}"
                );
            }
        }
    }

    #[test]
    fn late_invalidation_after_timeout_resets_window_state() {
        use super::super::epochs::State;
        for hand in 0..2 {
            for ending in 0..3 {
                let mut m = Monitor::new(WINDOW_NS);
                let mut epochs: [Evidence; 2] = Default::default();
                let agreement = |w: u64| {
                    let mut a = vec![
                        sensor("A", 100 * w, 0, 0.1, 0.1),
                        sensor("B", 100 * w, 0, 0.1, 0.1),
                        visibility(
                            "C",
                            (100 * w, 200.min(100 * w)),
                            (100 * w.saturating_sub(2), 0),
                            0.9,
                            0.9,
                        ),
                    ];
                    if hand == 1 {
                        for (_, hands) in &mut a {
                            hands.swap(0, 1);
                        }
                    }
                    a
                };
                for w in 0..=3 {
                    if w == 3 {
                        epochs[hand].count(
                            1,
                            Checks {
                                checks: 100,
                                misses: 0,
                            },
                        );
                    }
                    m.observed_evidence("C", &epochs);
                    assert_eq!(
                        m.update(1 + w * WINDOW_NS, &agreement(w)),
                        if w == 2 {
                            vec![Change::Inspect("C".into())]
                        } else {
                            vec![]
                        }
                    );
                }
                let timeout = 1 + 3 * WINDOW_NS + 1_000_000_000;
                m.inspecting.as_mut().unwrap().until_ns = timeout;
                assert_eq!(
                    m.update(timeout, &agreement(3)),
                    vec![Change::Unconfirmed("C".into())]
                );
                assert!(m.inspecting().is_none());
                assert_eq!(m.over["C"], 1);
                epochs[hand].note(
                    1,
                    if ending == 0 {
                        State::Ended
                    } else {
                        State::Invalidated
                    },
                );
                if ending == 2 {
                    // Overflow forgets the invalidation, leaving only a watermark.
                    for epoch in 2..=1025 {
                        epochs[hand].count(
                            epoch,
                            Checks {
                                checks: 1,
                                misses: 0,
                            },
                        );
                    }
                    assert_eq!(epochs[hand].forgotten_through, 1);
                }
                // Lifecycle delivery must work while restoration is pending, without an inspection.
                m.observed_evidence("C", &epochs);
                m.engine_restored("C");
                for w in 4..=6 {
                    // Fresh counters have already overtaken the old baseline. A numerical
                    // counter-reset heuristic cannot identify this invalidated predecessor.
                    let moved = if ending == 0 { w == 4 } else { w == 6 };
                    assert_eq!(
                        m.update(1 + w * WINDOW_NS, &agreement(w + 10)),
                        if moved {
                            // a normal end keeps the old baseline, so the window spans the
                            // counters' whole advance since window 3
                            let checks = if ending == 0 { 1100 } else { 100 };
                            vec![moved_in_windows(
                                "C",
                                on_hand(hand, last_window(checks, 0, Some((0.9, 0.9)))),
                            )]
                        } else {
                            vec![]
                        },
                        "hand {hand}, ending {ending}, window {w}"
                    );
                }
            }
        }
    }

    #[test]
    fn invalidated_peer_cannot_complete_another_sensors_streak() {
        use super::super::epochs::State;
        let mut m = Monitor::new(WINDOW_NS);
        let mut epochs: [Evidence; 2] = Default::default();
        epochs[0].note(1, State::Started);
        m.observed_evidence("A", &epochs);
        for w in 0..=1 {
            assert!(m
                .update(1 + w * WINDOW_NS, &room(w, |w| (100 * w, 0, 0.9, 0.9)))
                .is_empty());
        }
        assert_eq!(m.over["D"], 1);
        epochs[0].note(1, State::Invalidated);
        m.observed_evidence("A", &epochs);
        for w in 2..=4 {
            assert_eq!(
                m.update(1 + w * WINDOW_NS, &room(w, |w| (100 * w, 0, 0.9, 0.9))),
                if w == 4 {
                    vec![moved_in_windows(
                        "D",
                        on_hand(0, last_window(100, 0, Some((0.9, 0.9)))),
                    )]
                } else {
                    vec![]
                }
            );
        }
    }

    #[test]
    fn invalidated_peer_retracts_inspection_window_eligibility() {
        use super::super::epochs::State;
        let mut m = Monitor::new(WINDOW_NS);
        let mut observed: [Evidence; 2] = Default::default();
        let mut peer: [Evidence; 2] = Default::default();
        peer[0].note(1, State::Started);
        m.observed_evidence("A", &peer);
        for w in 0..=6 {
            if w == 3 {
                observed[0].count(
                    2,
                    Checks {
                        checks: 50,
                        misses: 50,
                    },
                );
            }
            if w == 4 {
                peer[0].note(1, State::Invalidated);
                m.observed_evidence("A", &peer);
            }
            if w >= 4 {
                observed[0].count(
                    2,
                    Checks {
                        checks: 30,
                        misses: 30,
                    },
                );
            }
            let n = observed[0].epochs.get(&2).map_or(0, |e| e.observed.checks);
            m.observed_evidence("C", &observed);
            let changes = m.update(
                1 + w * WINDOW_NS,
                &[
                    sensor("A", 100 * w, 0, 0.1, 0.1),
                    sensor("B", 100 * w, 0, 0.1, 0.1),
                    visibility("C", (100 * w, 100 * w), (n, n), 0.0, 0.0),
                ],
            );
            assert_eq!(
                changes,
                if w == 2 {
                    vec![Change::Inspect("C".into())]
                } else {
                    vec![]
                },
                "the old window depended on the invalidated peer, window {w}"
            );
        }
    }

    #[test]
    fn a_revoked_epoch_removes_earlier_windows() {
        use super::super::epochs::State;
        for hand in 0..2 {
            let mut monitor = Monitor::new(WINDOW_NS);
            let mut epochs: [Evidence; 2] = Default::default();
            for window in 0..=6 {
                let n = 100 * window;
                let mut agreement = vec![
                    sensor("A", n, 0, 0.1, 0.0),
                    sensor("B", n, 0, 0.1, 0.0),
                    visibility(
                        "C",
                        (n, n),
                        (n.saturating_sub(200), n.saturating_sub(200)),
                        0.0,
                        0.0,
                    ),
                ];
                if hand == 1 {
                    for (_, hands) in &mut agreement {
                        hands.swap(0, 1);
                    }
                }
                match window {
                    3 => epochs[hand].count(
                        1,
                        Checks {
                            checks: 30,
                            misses: 30,
                        },
                    ),
                    4 => {
                        epochs[hand].note(1, State::Invalidated);
                        epochs[hand].count(
                            2,
                            Checks {
                                checks: 80,
                                misses: 80,
                            },
                        );
                    }
                    5 => epochs[hand].count(
                        3,
                        Checks {
                            checks: 1,
                            misses: 1,
                        },
                    ),
                    // Retraction also resets the mixed peer/window baselines. Window 4
                    // cannot qualify; fresh windows 5 and 6 must supply their own 80 checks.
                    6 => epochs[hand].count(
                        4,
                        Checks {
                            checks: 79,
                            misses: 79,
                        },
                    ),
                    _ => {}
                }
                monitor.observed_evidence("C", &epochs);
                assert_eq!(
                    monitor.update(1 + window * WINDOW_NS, &agreement),
                    match window {
                        2 => vec![Change::Inspect("C".into())],
                        6 => vec![moved_on_inspection("C", on_hand(hand, (80, 80, 2)))],
                        _ => vec![],
                    },
                    "hand {hand}, window {window}"
                );
            }
        }
    }

    #[test]
    fn a_replayed_large_move_needs_observed_disagreement_after_inspection() {
        let fixture: serde_json::Value =
            serde_json::from_str(include_str!("../../tests/data/repose-moved-windows.json"))
                .unwrap();
        let mut m = Monitor::new(WINDOW_NS);
        let mut totals: BTreeMap<String, [CameraAgreement; 2]> = BTreeMap::new();
        let mut changes = Vec::new();
        for window in fixture["windows"].as_array().unwrap() {
            for row in window["sightings"].as_array().unwrap() {
                let a = &mut totals
                    .entry(row["sensor"].as_str().unwrap().to_string())
                    .or_default()[row["hand"].as_u64().unwrap() as usize];
                a.checks += row["checks"].as_u64().unwrap();
                a.misses += row["misses"].as_u64().unwrap();
                a.observed_checks += row["observed_checks"].as_u64().unwrap();
                a.observed_misses += row["observed_misses"].as_u64().unwrap();
                a.samples += row["samples"].as_u64().unwrap();
                a.median_px = row["median_px"].as_f64().unwrap();
                a.bias_px = Vector2::new(
                    row["bias_px"][0].as_f64().unwrap(),
                    row["bias_px"][1].as_f64().unwrap(),
                );
            }
            let now = window["t_ns"].as_u64().unwrap();
            let agreement: Vec<_> = totals.iter().map(|(s, a)| (s.clone(), *a)).collect();
            if let Some(hands) = totals.get("C") {
                let epochs = hands.map(|a| {
                    let mut evidence = Evidence::default();
                    evidence.count(
                        1,
                        Checks {
                            checks: a.observed_checks,
                            misses: a.observed_misses,
                        },
                    );
                    evidence
                });
                m.observed_evidence("C", &epochs);
            }
            for change in m.update(1_000 + now, &agreement) {
                changes.push((now, change));
            }
            if now < 25_000_000_000 {
                assert!(!m.handed_over("C"));
            }
        }
        assert_eq!(
            changes,
            [
                (15_001_763_884, Change::Inspect("C".into())),
                (
                    25_002_586_567,
                    moved_on_inspection("C", [Some((182, 182, 2)), Some((118, 118, 2))])
                ),
            ]
        );
        assert!(m.handed_over("C"));
    }

    #[test]
    fn a_sensor_being_placed_holds_the_search_slot_against_another_move() {
        let mut m = Monitor::new(WINDOW_NS);
        for w in 0u64..=4 {
            let agreement = vec![
                sensor("A", 100 * w, 0, 0.2, 0.1),
                sensor("B", 100 * w, 0, 0.2, 0.1),
                sensor("C", 100 * w, 100 * w, 0.0, 0.0),
                sensor("D", 100 * w, 100 * w.saturating_sub(2), 0.0, 0.0),
            ];
            let changes = m.update(1_000 + w * WINDOW_NS, &agreement);
            assert_eq!(
                changes,
                if w == 2 {
                    vec![moved_in_windows(
                        "C",
                        on_hand(0, last_window(100, 100, None)),
                    )]
                } else {
                    vec![]
                }
            );
        }
        assert!(m.handed_over("C"));
        assert!(!m.handed_over("D"));
        m.search_finished("C");
        assert!(m
            .update(
                1_000 + 5 * WINDOW_NS,
                &[
                    sensor("A", 500, 0, 0.2, 0.1),
                    sensor("B", 500, 0, 0.2, 0.1),
                    sensor("D", 500, 300, 0.0, 0.0),
                ]
            )
            .is_empty());
        m.engine_restored("C");
        assert_eq!(
            m.update(
                1_000 + 6 * WINDOW_NS,
                &[
                    sensor("A", 600, 0, 0.2, 0.1),
                    sensor("B", 600, 0, 0.2, 0.1),
                    sensor("D", 600, 400, 0.0, 0.0),
                ]
            ),
            [moved_in_windows(
                "D",
                on_hand(0, last_window(100, 100, Some((0.0, 0.0))))
            )]
        );
    }

    /// D is judged moved while C is inspected. C's inspection stops without an outcome, so no
    /// wait follows it. D's verdict waits for C's restoration, D judged no further, and is
    /// handed over once the engine acknowledges it: mid-window, whatever D shows meanwhile.
    /// Retracted evidence drops the waiting verdict, said once. While another restoration is
    /// unacknowledged, the inspection goes on and D's streak waits.
    #[test]
    fn a_move_during_another_sensors_inspection_waits_for_its_restoration() {
        use super::super::epochs::State;
        let at = |w: u64| 1_000 + w * WINDOW_NS;
        // A and B agree; C sees only unlabelled blobs; B and D missed this many in all.
        let room = |w: u64, b_misses: u64, d_misses: u64| {
            vec![
                sensor("A", 100 * w, 0, 0.3, 0.1),
                sensor("B", 100 * w, b_misses, 0.3, 0.1),
                visibility("C", (100 * w, 100 * w), (0, 0), 0.0, 0.0),
                sensor("D", 100 * w, d_misses, 0.3, 0.1),
            ]
        };
        // D misses 80 of its 100 observed checks in windows 3 and 4 only.
        let d_misses = |w: u64| 80 * w.min(4).saturating_sub(2);
        let interrupted = || Change::Interrupted {
            serial: "C".into(),
            moved: "D".into(),
        };
        let until_interrupted = |m: &mut Monitor| {
            for w in 0..=4 {
                let want = match w {
                    2 => vec![Change::Inspect("C".into())],
                    4 => vec![interrupted()],
                    _ => vec![],
                };
                assert_eq!(
                    m.update(at(w), &room(w, 0, d_misses(w))),
                    want,
                    "window {w}"
                );
            }
            assert_eq!(m.inspecting(), None);
            assert!(m.restoring().contains("C"));
            assert!(!m.handed_over("D"));
        };
        let mut m = Monitor::new(WINDOW_NS);
        until_interrupted(&mut m);
        // D, already judged, is no peer of the others: B disagrees with A alone.
        assert_eq!(m.update(at(5), &room(5, 80, d_misses(5))), [Change::Unsure]);
        assert!(!m.handed_over("D"));
        m.engine_restored("C");
        assert_eq!(
            m.update(at(5) + 1_000_000_000, &room(5, 80, d_misses(5))),
            [moved_in_windows(
                "D",
                on_hand(0, last_window(100, 80, Some((0.3, 0.1))))
            )]
        );
        assert!(m.handed_over("D"));
        // D is placed; C is inspected again once two more windows miss its hand.
        m.released("D");
        m.engine_restored("D");
        for w in 6..=7 {
            assert_eq!(
                m.update(at(w), &room(w, 80, d_misses(w))),
                if w == 7 {
                    vec![Change::Inspect("C".into())]
                } else {
                    vec![]
                },
                "window {w}"
            );
        }
        // A peer's normal end keeps the waiting verdict. Its invalidation or overflow drops it,
        // and the next update says so once.
        let peer = |state: State, forgotten_through: u64| {
            let mut states: [Evidence; 2] = Default::default();
            states[0].note(1, state);
            states[1].forgotten_through = forgotten_through;
            states
        };
        for (case, lifecycle, dropped) in [
            ("ended", peer(State::Ended, 0), false),
            ("invalidated", peer(State::Invalidated, 0), true),
            ("overflowed", peer(State::Ended, 1), true),
        ] {
            let mut m = Monitor::new(WINDOW_NS);
            until_interrupted(&mut m);
            m.lifecycle("A", &lifecycle);
            m.engine_restored("C");
            let want = if dropped {
                Change::Dropped("D".into())
            } else {
                moved_in_windows("D", on_hand(0, last_window(100, 80, Some((0.3, 0.1)))))
            };
            assert_eq!(
                m.update(at(4) + 1_000_000_000, &room(4, 0, d_misses(4))),
                [want],
                "{case}"
            );
            assert_eq!(m.handed_over("D"), !dropped, "{case}");
            assert_eq!(
                m.update(at(4) + 2_000_000_000, &room(4, 0, d_misses(4))),
                [],
                "{case}, said once"
            );
        }
        let mut m = Monitor::new(WINDOW_NS);
        for w in 0..=4 {
            if w == 3 {
                m.stopping.insert("E".into());
            }
            assert_eq!(
                m.update(at(w), &room(w, 0, 80 * w.saturating_sub(2))),
                if w == 2 {
                    vec![Change::Inspect("C".into())]
                } else {
                    vec![]
                },
                "window {w}"
            );
        }
        assert_eq!(m.inspecting(), Some("C"));
        m.engine_restored("E");
        assert_eq!(m.update(at(5), &room(5, 0, 240)), [interrupted()]);
    }

    #[test]
    fn only_one_unlabelled_sensor_is_inspected_at_a_time() {
        let mut m = Monitor::new(WINDOW_NS);
        for w in 0u64..=4 {
            let agreement = vec![
                sensor("A", 100 * w, 0, 0.3, 0.1),
                sensor("B", 100 * w, 0, 0.3, 0.1),
                visibility("C", (100 * w, 100 * w), (0, 0), 0.0, 0.0),
                visibility("D", (100 * w, 100 * w), (0, 0), 0.0, 0.0),
            ];
            let want = if w == 2 {
                vec![Change::Inspect("C".into())]
            } else {
                vec![]
            };
            assert_eq!(m.update(1_000 + w * WINDOW_NS, &agreement), want);
        }
    }

    #[test]
    fn an_inspected_sensor_can_agree_without_having_been_demoted() {
        let mut m = Monitor::new(WINDOW_NS);
        for w in 0u64..=4 {
            let matched = 100 * w.saturating_sub(2);
            let agreement = vec![
                sensor("A", 100 * w, 0, 0.3, 0.1),
                sensor("B", 100 * w, 0, 0.3, 0.1),
                visibility("C", (100 * w, 200.min(100 * w)), (matched, 0), 0.3, 0.1),
            ];
            let want = match w {
                2 => vec![Change::Inspect("C".into())],
                3 => vec![Change::Agreed("C".into())],
                _ => vec![],
            };
            assert_eq!(m.update(1_000 + w * WINDOW_NS, &agreement), want);
            assert!(!m.handed_over("C"));
        }
    }

    #[test]
    fn an_inspection_times_out_without_demotion_and_waits_before_searching_again() {
        let mut m = Monitor::new(WINDOW_NS);
        let restart = (INSPECTION_NS + COOLDOWN_NS) / WINDOW_NS + 3;
        for w in 0..=restart {
            let agreement = vec![
                sensor("A", 100 * w, 0, 0.3, 0.1),
                sensor("B", 100 * w, 0, 0.3, 0.1),
                visibility("C", (100 * w, 100 * w), (0, 0), 0.0, 0.0),
            ];
            let want = if w == 2 || w == restart {
                vec![Change::Inspect("C".into())]
            } else if w == 2 + INSPECTION_NS / WINDOW_NS {
                vec![Change::Unconfirmed("C".into())]
            } else {
                vec![]
            };
            assert_eq!(
                m.update(1_000 + w * WINDOW_NS, &agreement),
                want,
                "window {w}"
            );
            assert!(!m.handed_over("C"));
            if w == 2 + INSPECTION_NS / WINDOW_NS {
                m.engine_restored("C");
            }
        }
    }

    #[test]
    fn a_timed_out_inspection_holds_its_slot_until_native_restoration() {
        let mut m = Monitor::new(WINDOW_NS);
        let agreement = |w| {
            vec![
                sensor("A", 100 * w, 0, 0.3, 0.1),
                sensor("B", 100 * w, 0, 0.3, 0.1),
                visibility("C", (100 * w, 100 * w), (0, 0), 0.0, 0.0),
                visibility("D", (100 * w, 100 * w), (0, 0), 0.0, 0.0),
            ]
        };
        for w in 0..=9 {
            let want = match w {
                2 => vec![Change::Inspect("C".into())],
                8 => vec![Change::Unconfirmed("C".into())],
                _ => vec![],
            };
            assert_eq!(
                m.update(1_000 + w * WINDOW_NS, &agreement(w)),
                want,
                "window {w}"
            );
        }
        m.engine_restored("C");
        assert_eq!(
            m.update(1_000 + 10 * WINDOW_NS, &agreement(10)),
            [Change::Inspect("D".into())]
        );
    }

    #[test]
    fn an_inspection_deadline_does_not_wait_for_another_full_window() {
        let mut m = Monitor::new(WINDOW_NS);
        let agreement = |w| {
            vec![
                sensor("A", 100 * w, 0, 0.3, 0.1),
                sensor("B", 100 * w, 0, 0.3, 0.1),
                visibility("C", (100 * w, 100 * w), (0, 0), 0.0, 0.0),
            ]
        };
        for w in 0..=2 {
            m.update(1_000 + w * WINDOW_NS, &agreement(w));
        }
        let deadline = 1_000 + 2 * WINDOW_NS + INSPECTION_NS;
        assert_eq!(m.update(deadline - 1, &agreement(2)), []);
        assert_eq!(
            m.update(deadline, &agreement(2)),
            [Change::Unconfirmed("C".into())]
        );
        assert!(!m.handed_over("C"));
    }

    /// Still hands, or a two-handed grip, starve an inspection: the controllers were found but
    /// never turned enough on their own to be identified. That is no verdict, so the sensor is
    /// looked at again after a minute, doubling while it keeps starving, up to the cooldown a
    /// verdict gets. A sensor whose hand was never found at all (an occlusion) gets the
    /// cooldown, which ends the run; so does evidence that could judge identity.
    #[test]
    fn a_starved_inspection_is_retried_soon_and_backs_off_to_the_cooldown() {
        let mut m = Monitor::new(WINDOW_NS);
        let still = IdentityEvidence {
            seen: true,
            judgeable: false,
        };
        let judged = IdentityEvidence {
            seen: true,
            judgeable: true,
        };
        let mut starts = Vec::new();
        let mut outcomes = Vec::new();
        let mut w = 0u64;
        while outcomes.len() < 7 {
            let now = 1_000 + w * WINDOW_NS;
            let agreement = vec![
                sensor("A", 100 * w, 0, 0.3, 0.1),
                sensor("B", 100 * w, 0, 0.3, 0.1),
                visibility("C", (100 * w, 100 * w), (0, 0), 0.0, 0.0),
            ];
            if let Some(serial) = m.inspecting().map(str::to_string) {
                // The sixth inspection never finds the hand.
                let hand = if outcomes.len() == 5 {
                    IdentityEvidence::default()
                } else {
                    still
                };
                m.identity_evidence(&serial, [hand, still]);
            }
            for change in m.update(now, &agreement) {
                match change {
                    Change::Inspect(serial) => {
                        assert_eq!(serial, "C");
                        starts.push(now);
                    }
                    Change::Starved { serial, retry_ns } => {
                        assert_eq!(serial, "C");
                        outcomes.push((now, Some(retry_ns)));
                        m.engine_restored("C");
                    }
                    Change::Unconfirmed(serial) => {
                        assert_eq!(serial, "C");
                        outcomes.push((now, None));
                        m.engine_restored("C");
                    }
                    other => panic!("window {w}: {other:?}"),
                }
            }
            assert!(!m.handed_over("C"));
            w += 1;
        }
        let retries: Vec<Option<u64>> = outcomes.iter().map(|o| o.1).collect();
        let s = 1_000_000_000;
        assert_eq!(
            retries,
            [
                Some(60 * s),
                Some(120 * s),
                Some(240 * s),
                Some(480 * s),
                Some(600 * s),
                None,
                Some(60 * s)
            ]
        );
        // Each inspection starts once its retry (or cooldown) is over and two more windows of
        // missing sightings have passed.
        for (k, (ended, retry)) in outcomes.iter().enumerate().take(6) {
            let wait = retry.unwrap_or(COOLDOWN_NS);
            assert_eq!(
                starts[k + 1],
                ended + wait + WINDOW_NS,
                "inspection {}",
                k + 1
            );
        }
        assert_eq!(m.starved_retry("C"), 120 * s);
        m.identity_evidence("C", [judged, still]);
        assert_eq!(m.starved_retry("C"), 60 * s);
    }

    #[test]
    fn a_finished_placing_search_does_not_block_other_sensors_inspections() {
        let mut m = Monitor::new(WINDOW_NS);
        run(&mut m, 2, |w| (100 * w, 80 * w, 0.3, 0.1));
        let missing = |w| {
            let mut agreement = room(w, |w| (100 * w, 80 * w, 0.3, 0.1));
            agreement[2] = visibility(
                "C",
                (180 + 100 * (w - 2), 12 + 100 * (w - 2)),
                (180, 12),
                0.4,
                0.15,
            );
            agreement
        };
        for w in 3..=4 {
            assert_eq!(m.update(1_000 + w * WINDOW_NS, &missing(w)), []);
        }
        m.search_finished("D");
        m.engine_restored("D");
        assert_eq!(
            m.update(1_000 + 5 * WINDOW_NS, &missing(5)),
            [Change::Inspect("C".into())]
        );
        assert!(m.handed_over("D"));
        assert!(!m.handed_over("C"));
    }

    #[test]
    fn an_inspection_cooldown_does_not_ignore_new_observed_movement() {
        let mut m = Monitor::new(WINDOW_NS);
        for w in 0u64..=5 {
            let matched = 100 * w.saturating_sub(2);
            let agreement = vec![
                sensor("A", 100 * w, 0, 0.3, 0.1),
                sensor("B", 100 * w, 0, 0.3, 0.1),
                visibility(
                    "C",
                    (100 * w, 200.min(100 * w)),
                    (matched, 0),
                    if w >= 4 { 1.2 } else { 0.3 },
                    if w >= 4 { 1.0 } else { 0.1 },
                ),
            ];
            let want = match w {
                2 => vec![Change::Inspect("C".into())],
                3 => vec![Change::Agreed("C".into())],
                5 => vec![moved_in_windows(
                    "C",
                    on_hand(0, last_window(100, 0, Some((1.2, 1.0)))),
                )],
                _ => vec![],
            };
            assert_eq!(m.update(1_000 + w * WINDOW_NS, &agreement), want);
            if w == 3 {
                m.engine_restored("C");
            }
        }
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
        m.engine_restored("D");
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
            [
                vec![],
                vec![moved_in_windows(
                    "D",
                    on_hand(0, last_window(100, 80, Some((0.3, 0.1))))
                )],
                vec![]
            ],
            "{out:?}"
        );
        m.released_until("D", 1_000 + 12 * WINDOW_NS);
        m.engine_restored("D");
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
            [
                vec![],
                vec![],
                vec![],
                vec![moved_in_windows(
                    "D",
                    on_hand(0, last_window(100, 80, Some((0.3, 0.1))))
                )]
            ],
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
                    identity_epoch: 0,
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

    #[test]
    fn invalidation_retracts_gathered_solving_and_confirming_placements() {
        for stage in 0..4 {
            for hand in 0..2 {
                let truth = synth::look_at(
                    Vector3::new(1.9, 2.2, -1.6),
                    Vector3::new(0.0, 1.0, 0.2),
                    false,
                );
                let mut rng = Rng::new(4);
                let motion = synth::Motion::random(&mut rng, Vector3::new(0.0, 1.0, 0.0));
                let mut r = Reposer::default();
                r.watch(7);
                // Use real, varied probe geometry; control only completion of the off-thread fit.
                let mut batch = probes(&truth, &motion, 0, 500, &mut rng);
                for p in &mut batch {
                    p.hand = hand;
                    p.identity_epoch = 1;
                }
                let result = || {
                    Ok(Solved {
                        fresh: fit_at(truth, 0.1),
                        all: fit_at(truth, 0.1),
                    })
                };
                for p in &batch {
                    r.add(p.clone());
                }
                let first = if stage > 0 { r.batch() } else { None };
                if stage >= 2 {
                    assert_eq!(r.solved(first.clone().unwrap(), result()), Outcome::Nothing);
                    for p in &batch {
                        let mut fresh = p.clone();
                        if stage == 3 {
                            fresh.identity_epoch = 2;
                        }
                        r.add(fresh);
                    }
                }
                let second = if stage == 3 {
                    Some(r.batch().unwrap())
                } else {
                    None
                };
                let mut states: [Evidence; 2] = Default::default();
                states[hand].note(1, super::super::epochs::State::Invalidated);
                r.lifecycle(7, &states);
                // Delayed probes cannot restore invalidated gathering or its saved fit.
                for p in &batch {
                    r.add(p.clone());
                }
                assert!(r.gathering[&7].probes.is_empty(), "gathering stage {stage}");
                if stage < 3 {
                    assert_eq!(r.gathered(7), 0);
                }
                assert!(
                    r.gathering[&7].first.is_none(),
                    "confirmation stage {stage}"
                );
                if let Some(job) = second.or(if stage == 1 { first } else { None }) {
                    assert_eq!(
                        r.solved(job, result()),
                        Outcome::Nothing,
                        "solving stage {stage}"
                    );
                    assert!(r.gathering[&7].first.is_none());
                }
                assert!(r.batch().is_none());
                // A normal end preserves both independent batches; a fresh identity can place.
                states[hand].note(2, super::super::epochs::State::Ended);
                r.lifecycle(7, &states);
                for p in &mut batch {
                    p.identity_epoch = 2;
                }
                for n in 0..2 {
                    for p in &batch {
                        r.add(p.clone());
                    }
                    let job = r.batch().unwrap();
                    assert_eq!(
                        matches!(r.solved(job, result()), Outcome::Placed { .. }),
                        n == 1
                    );
                }
            }
        }
    }

    #[test]
    fn invalidated_failures_cannot_give_up_a_sensor() {
        use super::super::epochs::State;
        let truth = synth::look_at(
            Vector3::new(1.9, 2.2, -1.6),
            Vector3::new(0.0, 1.0, 0.2),
            false,
        );
        let far = Translation3::new(0.05, 0.0, 0.0) * truth;
        let mut rng = Rng::new(4);
        let motion = synth::Motion::random(&mut rng, Vector3::new(0.0, 1.0, 0.0));
        let probes = probes(&truth, &motion, 0, 500, &mut rng);
        for hand in 0..2 {
            for mode in 0..3 {
                for ending in 0..3 {
                    let mut r = Reposer::default();
                    r.watch(7);
                    let batch = |r: &mut Reposer, epoch| {
                        for p in &probes {
                            let mut p = p.clone();
                            p.hand = hand;
                            p.identity_epoch = epoch;
                            r.add(p);
                        }
                        r.batch().expect("a real varied batch")
                    };
                    let tight = |pose| {
                        Ok(Solved {
                            fresh: fit_at(pose, 0.1),
                            all: fit_at(pose, 0.1),
                        })
                    };
                    if mode == 2 {
                        let job = batch(&mut r, 1);
                        assert_eq!(r.solved(job, tight(truth)), Outcome::Nothing);
                    }
                    for n in 0..5 {
                        // Every disagreement includes epoch 1, alternating between the earlier
                        // and fresh fit. Neither side may lose its failure provenance.
                        let epoch = if mode == 2 && n % 2 == 0 { 2 } else { 1 };
                        let job = batch(&mut r, epoch);
                        let result = match mode {
                            0 => Err("no pose".into()),
                            1 => Ok(Solved {
                                fresh: fit_at(truth, 2.0),
                                all: fit_at(truth, 2.0),
                            }),
                            _ => tight(if n % 2 == 0 { far } else { truth }),
                        };
                        assert_eq!(r.solved(job, result), Outcome::Nothing);
                    }
                    let mut states: [Evidence; 2] = Default::default();
                    states[hand].note(
                        1,
                        if ending == 0 {
                            State::Ended
                        } else {
                            State::Invalidated
                        },
                    );
                    if ending == 2 {
                        // Overflow forgets the invalidation, leaving only a watermark.
                        for epoch in 2..=1025 {
                            states[hand].count(
                                epoch,
                                Checks {
                                    checks: 1,
                                    misses: 0,
                                },
                            );
                        }
                        assert_eq!(states[hand].forgotten_through, 1);
                    }
                    r.lifecycle(7, &states);
                    assert_eq!(r.gathering[&7].attempts.len(), if ending == 0 { 5 } else { 0 },
                        "every failed comparison includes the revoked epoch: mode {mode}, ending {ending}");
                    let job = batch(&mut r, 3000);
                    let outcome = r.solved(job, Err("one fresh failure".into()));
                    assert_eq!(
                        matches!(outcome, Outcome::GaveUp { .. }),
                        ending == 0,
                        "hand {hand}, mode {mode}, ending {ending}: {outcome:?}"
                    );
                    assert_eq!(r.given_up(7), ending == 0);
                }
            }
        }
    }

    /// A solver job that outlives its sensor's release cannot place, or count against, the
    /// sensor's next watch, whatever epochs its probes carry: their tombstones have retired.
    #[test]
    fn a_job_outliving_its_watch_cannot_affect_the_next() {
        let truth = synth::look_at(
            Vector3::new(1.9, 2.2, -1.6),
            Vector3::new(0.0, 1.0, 0.2),
            false,
        );
        let mut rng = Rng::new(4);
        let motion = synth::Motion::random(&mut rng, Vector3::new(0.0, 1.0, 0.0));
        let probes = probes(&truth, &motion, 0, 500, &mut rng);
        let batch = |r: &mut Reposer| {
            for p in &probes {
                r.add(p.clone());
            }
            r.batch().unwrap()
        };
        let fit = || {
            Ok(Solved {
                fresh: fit_at(truth, 0.1),
                all: fit_at(truth, 0.1),
            })
        };
        let mut r = Reposer::default();
        r.watch(7);
        let stale = batch(&mut r);
        r.forget(7);
        r.watch(7);
        let current = batch(&mut r);
        assert_eq!(r.solved(stale.clone(), fit()), Outcome::Nothing);
        assert!(
            r.gathering[&7].first.is_none(),
            "a stale fit cannot await confirmation"
        );
        assert!(
            r.gathering[&7].solving,
            "the current job is still outstanding"
        );
        assert_eq!(r.solved(stale, Err("no pose".into())), Outcome::Nothing);
        assert!(r.gathering[&7].attempts.is_empty());
        assert_eq!(r.solved(current, fit()), Outcome::Nothing);
        assert!(r.gathering[&7].first.is_some());
    }

    /// The engine has restored the sensor: its search's tombstones retire, so a long session
    /// of searches, each with an invalidation, never fills the bound.
    #[test]
    fn restored_sensors_retire_their_tombstones() {
        use super::super::epochs::State;
        let mut m = Monitor::new(WINDOW_NS);
        for search in 1..=1100 {
            let mut states: [Evidence; 2] = Default::default();
            states[search as usize % 2].note(search, State::Invalidated);
            m.lifecycle("C", &states);
            m.search_finished("C");
            m.engine_restored("C");
        }
        assert_eq!(m.lifetimes["C"], <[Evidence; 2]>::default());
        assert!(m.stopping.is_empty());
    }

    #[test]
    fn a_failed_job_keeps_the_saved_fits_failure_provenance() {
        use super::super::epochs::State;
        let truth = synth::look_at(
            Vector3::new(1.9, 2.2, -1.6),
            Vector3::new(0.0, 1.0, 0.2),
            false,
        );
        let mut rng = Rng::new(4);
        let motion = synth::Motion::random(&mut rng, Vector3::new(0.0, 1.0, 0.0));
        let probes = probes(&truth, &motion, 0, 500, &mut rng);
        for hand in 0..2 {
            let mut r = Reposer::default();
            r.watch(7);
            let batch = |r: &mut Reposer, epoch| {
                for p in &probes {
                    let mut p = p.clone();
                    p.hand = hand;
                    p.identity_epoch = epoch;
                    r.add(p);
                }
                r.batch().unwrap()
            };
            let fit = |pose| {
                Ok(Solved {
                    fresh: fit_at(pose, 0.1),
                    all: fit_at(pose, 0.1),
                })
            };
            let first = batch(&mut r, 1);
            assert_eq!(r.solved(first, fit(truth)), Outcome::Nothing);
            let failed = batch(&mut r, 2);
            assert!(!failed.earlier.is_empty());
            assert_eq!(r.solved(failed, Err("no pose".into())), Outcome::Nothing);
            // The saved fit is still compared after a failed job consumed its probe vector.
            let comparison = batch(&mut r, 3);
            assert_eq!(
                r.solved(comparison, fit(Translation3::new(0.05, 0.0, 0.0) * truth)),
                Outcome::Nothing
            );
            assert_eq!(r.gathering[&7].attempts.len(), 2);
            let mut states: [Evidence; 2] = Default::default();
            states[hand].note(1, State::Invalidated);
            r.lifecycle(7, &states);
            assert!(
                r.gathering[&7].attempts.is_empty(),
                "both failures depended on the original saved fit, hand {hand}"
            );
        }
    }

    /// A physical hand can inherit the other's blob IDs at an image overlap. Once its motion
    /// diverges the identity check revokes it, but an exposure already queued for a running
    /// placement can arrive first. Identical motion can keep the alias for several seconds.
    /// Exercise the actual labelled fit, refinement, batch confirmation and combined fit.
    #[test]
    fn swapped_exposures_cannot_confirm_a_wrong_placement() {
        let truth = synth::look_at(
            Vector3::new(1.9, 2.2, -1.6),
            Vector3::new(0.0, 1.0, 0.2),
            false,
        );
        let model = synth::camera_model();
        let (leds, _) = synth::cv1_touch_left();
        // The idealised alias used by the native search regression: the other hand's model
        // fits this one at a fixed 120-degree offset. Making that fit exact is more favourable
        // to the wrong correspondence than the real controllers' approximate mirror symmetry.
        let alias = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 120f64.to_radians());
        let other_leds: Vec<Led> = leds
            .iter()
            .map(|led| Led {
                pos: alias * led.pos,
                dir: alias * led.dir,
            })
            .collect();
        // Front-to-back hand separation at an overlap, in the camera's viewing direction.
        let separation = (Vector3::new(0.0, 1.0, 0.0) - truth.translation.vector).normalize() * 0.3;
        for hand in 0..2 {
            let device_id = if hand == 0 { 2 } else { 1 };
            // A clean control; one queued exposure in a confirmation batch; four seconds at
            // 50 Hz within one batch and across a batch boundary. No observations are retracted.
            for (start, count) in [(400, 0), (400, 1), (50, 200), (250, 200), (350, 200)] {
                let mut rng = Rng::new(4);
                let motion = synth::Motion::random(&mut rng, Vector3::new(0.0, 1.0, 0.0));
                let mut r = Reposer::default();
                r.watch(7);
                let mut contaminated = 0;
                let mut accepted = false;
                let mut batches = 0;
                for k in 0..2500 {
                    let own = motion.pose(k as f64 * 0.02);
                    let swapped = (start..start + count).contains(&k);
                    let (source, physical) = if swapped {
                        // Both hands have identical world-relative turns, so this alias can
                        // survive the contradiction backstop. Its position belongs to the
                        // other physical hand, but the probe's pose belongs to the requested one.
                        (
                            &other_leds,
                            Isometry3::from_parts(
                                Translation3::from(own.translation.vector + separation),
                                own.rotation * alias.inverse(),
                            ),
                        )
                    } else {
                        (&leds, own)
                    };
                    let (mut blobs, mut labels) = (Vec::new(), Vec::new());
                    for (i, led) in source.iter().enumerate() {
                        let pc = truth.inverse() * (physical * Point3::from(led.pos));
                        let normal = truth.inverse().rotation * (physical.rotation * led.dir);
                        if normal.dot(&(-pc.coords)) / pc.coords.norm() <= 0.3 {
                            continue;
                        }
                        if let Some(uv) = model.project(&pc.coords) {
                            blobs.push(uv + Vector2::new(rng.gauss(), rng.gauss()) * 0.2);
                            labels.push((device_id << 8) | i as i32);
                        }
                    }
                    let before = r.gathered(7);
                    r.add(Probe {
                        camera: 7,
                        hand,
                        device_id,
                        identity_epoch: 0,
                        world_from_model: own,
                        pos_sd: 5e-4,
                        blobs,
                        labels,
                    });
                    contaminated += usize::from(swapped && r.gathered(7) > before);
                    let Some(job) = r.batch() else { continue };
                    batches += 1;
                    let result = solve(&model, [Some(&leds), Some(&leds)], &job);
                    if let Ok(s) = &result {
                        for (name, fit) in [("fresh", &s.fresh), ("combined", &s.all)] {
                            let error = fit.world_from_cam.inverse() * truth;
                            eprintln!(
                                "overlap hand={hand} start={start} count={count} batch={batches} \
                                 {name} mm={:.4} deg={:.5} median={:.4} share={:.3}",
                                error.translation.vector.norm() * 1e3,
                                error.rotation.angle().to_degrees(),
                                fit.median_px,
                                fit.label_share,
                            );
                        }
                    }
                    match r.solved(job, result) {
                        Outcome::Placed { fit, .. } => {
                            let error = fit.world_from_cam.inverse() * truth;
                            assert!(
                                error.translation.vector.norm() < 2e-3
                                    && error.rotation.angle() < 0.1f64.to_radians(),
                                "hand={hand} start={start} count={count}: {error:?}"
                            );
                            accepted = true;
                            break;
                        }
                        Outcome::Nothing => {}
                        outcome => panic!("{outcome:?}"),
                    }
                }
                assert!(accepted, "later clean batches must confirm the true pose");
                assert!(batches >= 2);
                assert!(
                    if count < 2 {
                        contaminated == count
                    } else {
                        contaminated >= 100
                    },
                    "the swap must actually enter placement: {contaminated}/{count}"
                );
            }
        }
    }

    /// A loose fit, a failed solve, or two fits that disagree are not taken, and enough of them
    /// give the sensor up; a sensor not watched, a probe without labels, and one of an uncertain
    /// pose are dropped.
    #[test]
    fn loose_or_disagreeing_fits_are_not_taken() {
        let pose = Isometry3::translation(1.0, 2.0, 3.0);
        let job = |r: &Reposer, camera| Job {
            camera,
            fresh: Vec::new(),
            earlier: Vec::new(),
            watch: r.gathering[&camera].watch,
        };
        let mut r = Reposer::default();
        r.watch(1);
        assert_eq!(
            r.solved(job(&r, 1), Err("no pose".into())),
            Outcome::Nothing
        );
        let loose = Solved {
            fresh: fit_at(pose, 2.0),
            all: fit_at(pose, 2.0),
        };
        assert_eq!(r.solved(job(&r, 1), Ok(loose)), Outcome::Nothing);
        let mut unlabelled = Solved {
            fresh: fit_at(pose, 0.3),
            all: fit_at(pose, 0.3),
        };
        unlabelled.fresh.label_share = 0.3;
        assert_eq!(
            r.solved(job(&r, 1), Ok(unlabelled)),
            Outcome::Nothing,
            "few labels fit"
        );
        assert_eq!(r.gathering[&1].attempts.len(), 3);
        let tight = |p: Isometry3<f64>| Solved {
            fresh: fit_at(p, 0.3),
            all: fit_at(p, 0.3),
        };
        assert_eq!(
            r.solved(job(&r, 1), Ok(tight(pose))),
            Outcome::Nothing,
            "a first fit waits"
        );
        let far = Translation3::new(0.05, 0.0, 0.0) * pose;
        assert_eq!(
            r.solved(job(&r, 1), Ok(tight(far))),
            Outcome::Nothing,
            "disagreeing fits"
        );
        let turned = Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.02, 0.0)),
        ) * far;
        assert_eq!(r.solved(job(&r, 1), Ok(tight(turned))), Outcome::Nothing);
        // the sixth failure: given up
        match r.solved(job(&r, 1), Err("no pose".into())) {
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
            identity_epoch: 0,
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
            identity_epoch: 0,
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
