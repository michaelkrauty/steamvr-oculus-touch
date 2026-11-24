// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Camera exposures timed from a FreeTouch dongle, which, unlike the headset, reports no exposure
//! stamps: only its clock (`poll_us` in every controller record and in the status record sent
//! every 10 ms) and the exposure period. The grid is anchored to frame arrivals: on the dongle
//! clock a sensor's frames arrive a short, varying USB delay after their exposure, and the lower
//! edge of those arrivals, folded on the period, sits `LOWER_EDGE_NS` after it. The clock fit
//! supplies only the stable period, so which poll starts an exposure never enters. What remains is
//! the difference between the delay floors of the two USB paths, which the stored exposure offset
//! (`Settings::dongle_exposure_offset_ns`) absorbs as the pipeline applies it.
//!
//! The anchor is constant between changes and depends only on the order of the input, so a replay
//! gives the same exposures. It changes when the clock fit has spent a full window in a new regime
//! (its start, a reset, controller records starting or stopping), and when the earliest lower edge
//! leaves the grid (a jump).

use std::collections::{BTreeMap, VecDeque};

use serde::Serialize;

use super::clock::{LinearClock, Unwrap32};

/// Frame arrivals per sensor whose lower edge anchors the grid: about 2.5 s of a sensor.
pub const WINDOW: usize = 128;
/// The lower edge: this quantile of a window's folded arrivals.
pub const EDGE_QUANTILE: f64 = 0.02;
/// Where the lower edge of frame arrivals sits after the exposure.
pub const LOWER_EDGE_NS: i64 = 70_000;
/// An earliest lower edge this far from the anchored grid is a jump of the grid.
pub const JUMP_NS: i64 = 500_000;
/// Controller records this recent put the clock fit in its dense regime.
const DENSE_GAP_NS: u64 = 100_000_000;
/// How long the clock fit takes to reflect a new regime: its window of one-second bins.
const SETTLE_NS: u64 = 20_000_000_000;
/// A sensor with no frame for this long is dropped from anchoring so it cannot pin the grid.
const STALE_NS: u64 = 1_000_000_000;

/// One exposure: its index on the unwrapped dongle clock and when it happened there.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Exposure {
    pub count: u64,
    pub device_ns: u64,
}

impl Exposure {
    /// The stamp as the engine and `Pipeline::frame` take it: the dongle clock's microseconds.
    pub fn stamp_us(&self) -> u32 {
        (self.device_ns / 1000) as u32
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum Why {
    /// The first anchor, or one taken after the clock fit settled in a new regime.
    Settled,
    /// The earliest lower edge left the grid.
    Jump,
}

/// A change of the anchor, for the log.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct Anchor {
    pub why: Why,
    /// The dongle clock at the arrival that set it.
    pub device_ns: u64,
    /// How far the grid moved, within half a period (0 for the first).
    pub moved_ns: i64,
}

/// For `cv1touch report`, through the daemon's status.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Report {
    pub period_us: u64,
    pub jumps: u64,
    pub anchors: u64,
    /// Per sensor: its lower edge behind the earliest one at the last anchor or check, us.
    pub sensors: BTreeMap<String, Option<f64>>,
}

#[derive(Debug, Clone, Default)]
struct Window {
    /// Arrivals on the dongle clock minus the reference, folded into one period.
    lags: VecDeque<i64>,
    /// Dongle clock of the newest frame in `lags`, for excluding stopped sensors.
    last_ns: Option<u64>,
    behind_us: Option<f64>,
}

#[derive(Debug, Clone, Default)]
pub struct DongleExposures {
    clock: LinearClock,
    stamps: Unwrap32,
    newest_ns: u64,
    period_ns: u64,
    dense_until: Option<u64>,
    in_dense: bool,
    settle_at: Option<u64>,
    /// Arrivals are folded against this: the grid once anchored, before that the first arrival.
    reference: Option<i64>,
    /// An exposure on the dongle clock, once anchored.
    t0: Option<i64>,
    /// The next full window replaces the anchor.
    fresh: bool,
    /// The last full window's edge left the grid.
    suspect: bool,
    sensors: BTreeMap<String, Window>,
    jumps: u64,
    anchor_count: u64,
    anchors: Vec<Anchor>,
}

impl DongleExposures {
    /// A status record: a poll's stamp and the exposure period, both microseconds.
    pub fn status(&mut self, poll_us: u32, period_us: u16, host_ns: u64) {
        let period_ns = period_us as u64 * 1000;
        if period_ns != self.period_ns {
            self.period_ns = period_ns;
            self.clear();
            self.reference = None;
            self.t0 = None;
        }
        self.sample(poll_us, host_ns, false);
    }

    /// A controller record's `poll_us`.
    pub fn poll(&mut self, poll_us: u32, host_ns: u64) {
        self.sample(poll_us, host_ns, true);
    }

    fn sample(&mut self, us: u32, host_ns: u64, dense: bool) {
        // the two kinds of record interleave, so a stamp can be a little older than the newest
        let device_ns = match self.stamps.relative(us) {
            Some(r) if r < self.newest_ns => r,
            _ => self.stamps.extend(us),
        };
        self.newest_ns = self.newest_ns.max(device_ns);
        let was_dense = self.in_dense;
        if dense {
            let until = device_ns + DENSE_GAP_NS;
            self.dense_until = Some(self.dense_until.map_or(until, |u| u.max(until)));
        }
        let reset = self.clock.observe(device_ns, host_ns);
        if reset {
            self.clear();
        }
        let now_dense = self.dense_until.is_some_and(|u| self.newest_ns < u);
        self.in_dense = now_dense;
        if reset || was_dense != now_dense || self.settle_at.is_none() {
            self.settle_at = Some(device_ns + SETTLE_NS);
        } else if self.settle_at.is_some_and(|s| device_ns >= s) {
            self.settle_at = Some(u64::MAX);
            self.clear();
            self.fresh = true;
        }
    }

    fn clear(&mut self) {
        for w in self.sensors.values_mut() {
            w.lags.clear();
        }
    }

    /// A sensor's frame arrived at `host_ns`: the exposure it shows, once the grid is anchored.
    pub fn frame(&mut self, serial: &str, host_ns: u64) -> Option<Exposure> {
        if self.period_ns == 0 {
            return None;
        }
        let p = self.period_ns as i64;
        let d = self.clock.to_device(host_ns)? as i64;
        let reference = *self.reference.get_or_insert(d);
        let w = self.sensors.entry(serial.to_owned()).or_default();
        let d_u64 = d as u64;
        if w.last_ns
            .is_some_and(|t| d_u64.saturating_sub(t) > STALE_NS)
        {
            w.lags.clear();
            w.behind_us = None;
        }
        w.last_ns = Some(d_u64);
        w.lags.push_back((d - reference).rem_euclid(p));
        if w.lags.len() > WINDOW {
            w.lags.pop_front();
        }
        if w.lags.len() == WINDOW {
            self.check(d as u64);
        }
        let t0 = self.t0?;
        // a frame arrives well within half a period of its exposure
        let e = (t0 + (d - t0 + p / 2).div_euclid(p) * p) as u64;
        Some(Exposure {
            count: e / self.period_ns,
            device_ns: e,
        })
    }

    /// The earliest full window's lower edge places the grid: anchor on it when there is no
    /// anchor, when the fit has settled, or when the grid has left it.
    fn check(&mut self, device_ns: u64) {
        for w in self.sensors.values_mut() {
            if w.last_ns
                .is_none_or(|t| device_ns.saturating_sub(t) > STALE_NS)
            {
                w.lags.clear();
                w.behind_us = None;
            }
        }
        let p = self.period_ns as i64;
        let edges: Vec<(&String, i64)> = self
            .sensors
            .iter()
            .filter(|(_, w)| w.lags.len() == WINDOW)
            .map(|(s, w)| {
                let mut v: Vec<i64> = w.lags.iter().copied().collect();
                v.sort_unstable();
                // the arrivals fill a few ms of the period wherever the fold cuts them: they
                // start after the widest gap
                let start = (0..v.len())
                    .max_by_key(|&i| v[i] - if i == 0 { v[v.len() - 1] - p } else { v[i - 1] })
                    .unwrap_or(0);
                let q = start + (EDGE_QUANTILE * (WINDOW - 1) as f64).round() as usize;
                (s, v[q % v.len()] + if q >= v.len() { p } else { 0 })
            })
            .collect();
        let first = edges.first().map_or(0, |e| e.1);
        let fold = |x: i64| (x + p / 2).rem_euclid(p) - p / 2;
        let earliest = first + edges.iter().map(|e| fold(e.1 - first)).min().unwrap_or(0);
        let behind: Vec<(String, f64)> = edges
            .iter()
            .map(|(s, e)| ((*s).clone(), fold(e - earliest) as f64 / 1000.0))
            .collect();
        for (s, b) in behind {
            if let Some(w) = self.sensors.get_mut(&s) {
                w.behind_us = Some(b);
            }
        }
        let candidate = self.reference.unwrap_or(0) + earliest - LOWER_EDGE_NS;
        let moved = self.t0.map_or(0, |t0| fold(candidate - t0));
        let why = match self.t0 {
            Some(_) if !self.fresh && moved.abs() <= JUMP_NS => {
                self.suspect = false;
                return;
            }
            // a window that straddles the jump shows part of it: confirm on one wholly after it
            Some(_) if !self.fresh && !self.suspect => {
                self.suspect = true;
                self.clear();
                return;
            }
            Some(_) if !self.fresh => Why::Jump,
            _ => Why::Settled,
        };
        self.jumps += (why == Why::Jump) as u64;
        self.anchor_count += 1;
        self.anchors.push(Anchor {
            why,
            device_ns,
            moved_ns: moved,
        });
        self.fresh = false;
        self.suspect = false;
        self.t0 = Some(candidate);
        self.reference = Some(candidate);
        self.clear();
    }

    /// Anchor changes since the last call, for the log.
    pub fn take_anchors(&mut self) -> Vec<Anchor> {
        std::mem::take(&mut self.anchors)
    }

    pub fn report(&self) -> Report {
        Report {
            period_us: self.period_ns / 1000,
            jumps: self.jumps,
            anchors: self.anchor_count,
            sensors: self
                .sensors
                .iter()
                .map(|(s, w)| (s.clone(), w.behind_us))
                .collect(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PERIOD_US: u16 = 19_200;
    /// Host = dongle clock + this, with every record delayed 0.2 to 1.2 ms.
    const HOST_FROM_DEVICE_NS: u64 = 7_000_000_000;

    struct Rig {
        ex: DongleExposures,
        seed: u64,
        /// Dongle clock, microseconds, unwrapped.
        now_us: u64,
        /// The sensors' true exposure grid: an exposure on the dongle clock, us.
        grid_us: u64,
        controllers: bool,
        sensor_b: bool,
    }

    impl Rig {
        fn new(start_us: u64) -> Rig {
            Rig {
                ex: DongleExposures::default(),
                seed: 9,
                now_us: start_us,
                grid_us: start_us - 5_123,
                controllers: true,
                sensor_b: true,
            }
        }

        fn delay_ns(&mut self, max_ns: u64) -> u64 {
            self.seed = self
                .seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (self.seed >> 33) % max_ns
        }

        fn host(&mut self, us: u64) -> u64 {
            us * 1000 + HOST_FROM_DEVICE_NS + 200_000 + self.delay_ns(1_000_000)
        }

        /// Runs for `seconds`; returns each frame's error against the true exposure, ns.
        fn run(&mut self, seconds: u64) -> Vec<(u64, i64)> {
            let mut errors = vec![];
            let end = self.now_us + seconds * 1_000_000;
            while self.now_us < end {
                self.now_us += 2_000;
                let us = self.now_us;
                if self.controllers {
                    let h = self.host(us);
                    self.ex.poll(us as u32, h);
                }
                if us % 10_000 == 0 {
                    let h = self.host(us);
                    self.ex.status(us as u32, PERIOD_US, h);
                }
                let p = PERIOD_US as u64;
                let next = self.grid_us + (us - self.grid_us).div_ceil(p) * p;
                if next < us + 2_000 && next >= us {
                    for sensor in ["A", "B"] {
                        if sensor == "B" && !self.sensor_b {
                            continue;
                        }
                        // a frame arrives its lower edge plus up to 3 ms after its exposure
                        let lag = LOWER_EDGE_NS as u64 + self.delay_ns(3_000_000);
                        let host = next * 1000 + HOST_FROM_DEVICE_NS + lag;
                        if let Some(e) = self.ex.frame(sensor, host) {
                            // the stream starts a wrap above zero (`Unwrap32`)
                            let truth = (next + (1 << 32)) as i64 * 1000;
                            errors.push((us, e.device_ns as i64 - truth));
                        }
                    }
                }
            }
            errors
        }
    }

    fn worst(errors: &[(u64, i64)]) -> i64 {
        errors.iter().map(|e| e.1.abs()).max().unwrap()
    }

    #[test]
    fn the_grid_is_anchored_to_the_lower_edge_of_frame_arrivals() {
        let mut rig = Rig::new(1_000_000_000);
        let errors = rig.run(60);
        assert!(!errors.is_empty());
        // the host's clock reads every record late: the mapping has the delay floor
        let settled: Vec<_> = errors
            .iter()
            .filter(|e| e.0 > 1_025_000_000)
            .copied()
            .collect();
        assert!(worst(&settled) < 300_000, "{}", worst(&settled));
        let r = rig.ex.report();
        assert_eq!((r.period_us, r.jumps), (19_200, 0));
        assert_eq!(r.sensors.len(), 2);
        assert!(
            r.sensors.values().all(|b| b.is_some_and(|b| b < 100.0)),
            "{r:?}"
        );
    }

    #[test]
    fn a_replay_gives_the_same_exposures_and_the_anchor_holds_between_changes() {
        let (mut a, mut b) = (Rig::new(1_000_000_000), Rig::new(1_000_000_000));
        let ea = a.run(90);
        assert_eq!(ea, b.run(90));
        let anchors = a.ex.take_anchors();
        // the first, then the one after the fit's window: nothing moves it in steady state
        assert_eq!(anchors.len(), 2, "{anchors:?}");
        assert!(anchors.iter().all(|a| a.why == Why::Settled));
        let t0 = a.ex.t0;
        a.run(30);
        assert_eq!(a.ex.t0, t0);
    }

    #[test]
    fn a_jump_of_the_grid_is_followed_within_seconds_and_counted() {
        let mut rig = Rig::new(1_000_000_000);
        rig.run(40);
        rig.grid_us += 7_000;
        let start = rig.now_us;
        let errors = rig.run(20);
        let late: Vec<_> = errors
            .iter()
            .filter(|e| e.0 > start + 5_000_000)
            .copied()
            .collect();
        assert!(worst(&late) < 300_000, "{}", worst(&late));
        let r = rig.ex.report();
        assert_eq!(r.jumps, 1, "{:?}", rig.ex.anchors);
        let anchors = rig.ex.take_anchors();
        let jump = anchors.last().unwrap();
        assert_eq!(jump.why, Why::Jump);
        assert!((jump.moved_ns - 7_000_000).abs() < 300_000, "{jump:?}");
    }

    #[test]
    fn the_grid_continues_across_the_dongle_clocks_32_bit_wrap() {
        // 40 s before the wrap, on a status record's 10 ms
        let mut rig = Rig::new(4_254_960_000);
        let errors = rig.run(80);
        assert!(errors.iter().any(|e| e.0 > 1 << 32));
        assert!(worst(&errors[errors.len() / 2..]) < 300_000);
        assert_eq!(rig.ex.report().jumps, 0);
    }

    #[test]
    fn controller_records_starting_and_stopping_reanchor_after_the_fits_window() {
        let mut rig = Rig::new(1_000_000_000);
        rig.controllers = false;
        rig.run(30);
        let before = rig.ex.take_anchors().len();
        rig.controllers = true;
        rig.run(25);
        let anchors = rig.ex.take_anchors();
        assert_eq!(anchors.len(), 1, "{before} then {anchors:?}");
        assert_eq!(anchors[0].why, Why::Settled);
        // stopping controller records must also trigger a reanchor after the fit's window
        rig.controllers = false;
        rig.run(25);
        let after_stop = rig.ex.take_anchors();
        assert_eq!(after_stop.len(), 1, "{after_stop:?}");
        assert_eq!(after_stop[0].why, Why::Settled);
    }

    #[test]
    fn a_stopped_sensor_does_not_pin_the_grid_across_a_jump() {
        let mut rig = Rig::new(1_000_000_000);
        rig.run(40);
        rig.sensor_b = false;
        rig.grid_us += 3_000;
        let start = rig.now_us;
        let errors = rig.run(30);
        let late: Vec<_> = errors
            .iter()
            .filter(|e| e.0 > start + 8_000_000)
            .copied()
            .collect();
        assert!(worst(&late) < 300_000, "{}", worst(&late));
        let r = rig.ex.report();
        assert_eq!(r.jumps, 1, "{:?}", rig.ex.anchors);
        assert_eq!(r.sensors.get("B"), Some(&None), "{r:?}");
        let jump = rig.ex.take_anchors().pop().unwrap();
        assert_eq!(jump.why, Why::Jump);
        assert!((jump.moved_ns - 3_000_000).abs() < 300_000, "{jump:?}");
    }
}
