// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Device clock -> host CLOCK_MONOTONIC as a line fitted to the lower envelope of (device stamp,
//! host receive time) pairs. Every report arrives after a variable transport delay, so the fastest
//! report of each bin bounds the mapping from above; a least-squares line through the bin minima
//! of a trailing window follows crystal drift (tens of ppm) without the steps of a sliding minimum.

use std::collections::VecDeque;

#[derive(Debug, Clone)]
pub struct LinearClock {
    bin_ns: u64,
    window: usize,
    /// A report this far off the fitted line can indicate a clock discontinuity.
    reset_ns: f64,
    last_device_ns: Option<u64>,
    positive_since: Option<u64>,
    /// (device ns, host - device ns) of the first report; everything else is relative to it.
    reference: Option<(u64, i64)>,
    /// Closed bins: (x, y) of each bin's fastest report, x = device - reference, y = offset - reference.
    bins: VecDeque<(f64, f64)>,
    current: Option<(u64, f64, f64)>,
    /// host - device = reference offset + a + b x
    a: f64,
    b: f64,
}

impl Default for LinearClock {
    fn default() -> Self {
        LinearClock::new(1_000_000_000, 20)
    }
}

impl LinearClock {
    pub fn new(bin_ns: u64, window: usize) -> LinearClock {
        LinearClock {
            bin_ns,
            window,
            reset_ns: 50e6,
            last_device_ns: None,
            positive_since: None,
            reference: None,
            bins: VecDeque::new(),
            current: None,
            a: 0.0,
            b: 0.0,
        }
    }

    /// Returns true when an existing mapping was discarded and re-locked. This can be
    /// a clock discontinuity or a temporary transport delay, not first initialization.
    pub fn observe(&mut self, device_ns: u64, host_ns: u64) -> bool {
        self.observe_inner(device_ns, host_ns, false)
    }

    /// Within an independently checked continuity epoch, positive residuals can be FIFO
    /// receive delay. Require a full device-time bin without an on-time report before
    /// accepting a positive discontinuity; negative corrections and restarts stay immediate.
    pub(crate) fn observe_continuous(&mut self, device_ns: u64, host_ns: u64) -> bool {
        self.observe_inner(device_ns, host_ns, true)
    }

    fn observe_inner(&mut self, device_ns: u64, host_ns: u64, continuous: bool) -> bool {
        let restarted = continuous && self.last_device_ns.is_some_and(|last| device_ns < last);
        self.last_device_ns = Some(device_ns);
        let offset = host_ns as i64 - device_ns as i64;
        let (d0, y0) = *self.reference.get_or_insert((device_ns, offset));
        let (x, y) = (device_ns as f64 - d0 as f64, (offset - y0) as f64);
        let residual = y - self.a - self.b * x;
        if continuous && !restarted && residual > self.reset_ns {
            let since = *self.positive_since.get_or_insert(device_ns);
            if device_ns.saturating_sub(since) < self.bin_ns {
                return false;
            }
        } else {
            self.positive_since = None;
        }
        if restarted || residual.abs() > self.reset_ns {
            *self = LinearClock::new(self.bin_ns, self.window);
            self.observe(device_ns, host_ns);
            return true;
        }
        let bin = device_ns / self.bin_ns;
        match self.current {
            Some((b, _, cy)) if b == bin => {
                if y >= cy {
                    return false;
                }
                self.current = Some((bin, x, y));
                // The warm fit uses only closed bins. Save the open minimum for rollover.
                if self.bins.len() >= 4 {
                    return false;
                }
            }
            Some((b, cx, cy)) if bin > b => {
                self.bins.push_back((cx, cy));
                while self.bins.len() >= self.window {
                    self.bins.pop_front();
                }
                self.current = Some((bin, x, y));
            }
            Some(_) => return false, // stamps going backwards: ignore
            None => self.current = Some((bin, x, y)),
        }
        self.refit();
        false
    }

    fn refit(&mut self) {
        // a slope needs a few seconds of closed bins (the open one may hold only slow reports so
        // far, and at the newest x it would lever the line); until then, a constant at the lowest
        if self.bins.len() < 4 {
            self.a = self
                .bins
                .iter()
                .map(|p| p.1)
                .chain(self.current.map(|c| c.2))
                .fold(f64::INFINITY, f64::min);
            self.b = 0.0;
            return;
        }
        let points = &self.bins;
        let n = points.len() as f64;
        let (mx, my) = (
            points.iter().map(|p| p.0).sum::<f64>() / n,
            points.iter().map(|p| p.1).sum::<f64>() / n,
        );
        let sxx: f64 = points.iter().map(|p| (p.0 - mx).powi(2)).sum();
        let sxy: f64 = points.iter().map(|p| (p.0 - mx) * (p.1 - my)).sum();
        self.b = if sxx > 0.0 { sxy / sxx } else { 0.0 };
        self.a = my - self.b * mx;
    }

    pub fn valid(&self) -> bool {
        self.reference.is_some()
    }

    pub fn to_host(&self, device_ns: u64) -> Option<u64> {
        let (d0, y0) = self.reference?;
        let x = device_ns as f64 - d0 as f64;
        Some((device_ns as i64 + y0 + (self.a + self.b * x).round() as i64) as u64)
    }

    pub fn to_device(&self, host_ns: u64) -> Option<u64> {
        let (d0, y0) = self.reference?;
        // host = d0 + x + y0 + a + b x  =>  x = (host - d0 - y0 - a) / (1 + b)
        let h = (host_ns as i64 - d0 as i64 - y0) as f64;
        Some((d0 as i64 + ((h - self.a) / (1.0 + self.b)).round() as i64) as u64)
    }
}

/// Extends a wrapping 32-bit microsecond stamp to 64-bit nanoseconds.
#[derive(Debug, Clone, Default)]
pub struct Unwrap32 {
    last: Option<(u32, u64)>,
}

impl Unwrap32 {
    /// The next stamp of a monotonic stream.
    pub fn extend(&mut self, us: u32) -> u64 {
        let ext = match self.last {
            // start a wrap above zero so that slightly older stamps stay positive
            None => (1u64 << 32) + us as u64,
            Some((l, e)) => e + us.wrapping_sub(l) as u64,
        };
        self.last = Some((us, ext));
        ext * 1000
    }

    /// A stamp within +-35 minutes of the newest one, without advancing the stream.
    pub fn relative(&self, us: u32) -> Option<u64> {
        let (l, e) = self.last?;
        Some((e as i64 + us.wrapping_sub(l) as i32 as i64) as u64 * 1000)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A device clock running 20 ppm fast against the host, reports delayed 0.3 to 2.3 ms.
    fn drifting(n: usize, every_ns: u64) -> Vec<(u64, u64)> {
        let mut seed = 5u64;
        (0..n)
            .map(|k| {
                seed = seed
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                let delay = 300_000 + (seed >> 33) % 2_000_000;
                let host = 5_000_000_000_000 + k as u64 * every_ns;
                let device = 1_000_000 + ((host - 5_000_000_000_000) as f64 * (1.0 + 20e-6)) as u64;
                (device, host + delay)
            })
            .collect()
    }

    #[test]
    fn refit_matches_vec_oracle_for_every_deque_split() {
        for n in [4, 5, 19, 20, 64] {
            for split in 0..n {
                for constant_x in [false, true] {
                    let mut c = LinearClock {
                        bins: VecDeque::with_capacity(n),
                        ..LinearClock::default()
                    };
                    for i in 0..n {
                        c.bins
                            .push_back((i as f64 * 1e9, (i as f64 * 0.71).sin() * 1e6));
                    }
                    for _ in 0..split {
                        let p = c.bins.pop_front().unwrap();
                        c.bins.push_back(p);
                    }
                    if constant_x {
                        for p in &mut c.bins {
                            p.0 = 1.0;
                        }
                    }
                    // Original contiguous-copy algorithm, retaining reduction order.
                    let points: Vec<(f64, f64)> = c.bins.iter().copied().collect();
                    let count = points.len() as f64;
                    let mx = points.iter().map(|p| p.0).sum::<f64>() / count;
                    let my = points.iter().map(|p| p.1).sum::<f64>() / count;
                    let sxx: f64 = points.iter().map(|p| (p.0 - mx).powi(2)).sum();
                    let sxy: f64 = points.iter().map(|p| (p.0 - mx) * (p.1 - my)).sum();
                    let b = if sxx > 0.0 { sxy / sxx } else { 0.0 };
                    let a = my - b * mx;
                    c.refit();
                    assert_eq!((c.a.to_bits(), c.b.to_bits()), (a.to_bits(), b.to_bits()));
                }
            }
        }
    }

    #[test]
    fn the_fit_follows_drift_to_the_fastest_reports() {
        let mut c = LinearClock::default();
        let mut continuous = c.clone();
        for &(d, h) in &drifting(60_000, 1_000_000) {
            c.observe(d, h);
            assert!(!continuous.observe_continuous(d, h));
            assert_eq!(continuous.to_host(d), c.to_host(d));
            assert_eq!(
                (continuous.a.to_bits(), continuous.b.to_bits()),
                (c.a.to_bits(), c.b.to_bits())
            );
        }
        // truth at the newest report: host = (device - 1e6) / (1 + 20e-6) + 5e12, plus the 0.3 ms minimum delay
        let d = 1_000_000 + (59_999_000_000f64 * (1.0 + 20e-6)) as u64;
        let truth = 5_000_000_000_000 + ((d - 1_000_000) as f64 / (1.0 + 20e-6)) as u64 + 300_000;
        let err = c.to_host(d).unwrap() as i64 - truth as i64;
        assert!(err.abs() < 20_000, "error {err} ns");
        assert!((c.to_device(c.to_host(d).unwrap()).unwrap() as i64 - d as i64).abs() <= 1);
    }

    #[test]
    fn continuous_fifo_delay_does_not_change_the_fit_across_bins() {
        for warm in [2, 10] {
            for delay in [
                20_000_000, 49_000_000, 51_000_000, 60_000_000, 80_000_000, 99_000_000,
            ] {
                let mut c = LinearClock::default();
                for tick in 0..warm * 500 {
                    let dev = 1_000_000_000 + tick * 2_000_000;
                    assert!(!c.observe_continuous(dev, dev + 10_000_000_000));
                }
                let dev = 1_000_000_000 + warm * 1_000_000_000;
                let old_fix = c.to_host(dev - 100_000_000);
                assert!(!c.observe_continuous(dev, dev + 10_000_000_000 + delay));
                assert_eq!(c.to_host(dev - 100_000_000), old_fix);
                assert!(!c.observe_continuous(dev + 2_000_000, dev + 10_002_000_000));
                assert_eq!(c.to_host(dev - 100_000_000), old_fix);
                assert_eq!(c.positive_since, None);
            }
        }
    }

    #[test]
    fn continuous_positive_discontinuity_requires_a_full_bin_but_negative_and_restart_do_not() {
        let mut c = LinearClock::default();
        c.observe_continuous(1_000_000_000, 11_000_000_000);
        assert!(!c.observe_continuous(1_002_000_000, 11_062_000_000));
        assert!(!c.observe_continuous(2_001_999_999, 12_061_999_999));
        assert!(c.observe_continuous(2_002_000_000, 12_062_000_000));
        assert_eq!(c.to_host(2_002_000_000), Some(12_062_000_000));
        assert!(c.observe_continuous(2_004_000_000, 12_004_000_000));
        assert_eq!(c.to_host(2_004_000_000), Some(12_004_000_000));
        // A small backwards epoch need not produce a 50 ms residual.
        assert!(c.observe_continuous(2_003_000_000, 12_006_000_000));
        assert_eq!(c.to_host(2_003_000_000), Some(12_006_000_000));
    }

    #[test]
    fn a_clock_restart_relocks() {
        let mut c = LinearClock::default();
        for k in 0..5000u64 {
            assert!(!c.observe(k * 2_000_000, 9_000_000_000 + k * 2_000_000));
        }
        assert!(c.observe(1_000, 9_000_000_000 + 5000 * 2_000_000));
        assert_eq!(c.to_host(1_000), Some(9_000_000_000 + 5000 * 2_000_000));
        assert!(!c.observe(2_001_000, 9_000_000_000 + 5001 * 2_000_000));
    }

    #[test]
    fn unwrap_crosses_the_32_bit_boundary_and_reads_back_older_stamps() {
        let mut u = Unwrap32::default();
        let a = u.extend(u32::MAX - 10);
        let b = u.extend(5);
        assert_eq!(b - a, 16_000);
        assert_eq!(u.relative(u32::MAX - 2), Some(a + 8_000));
    }
}

#[cfg(test)]
impl LinearClock {
    fn original_observe(&mut self, device_ns: u64, host_ns: u64) {
        // Observation metadata only; keep the original fitting/reset oracle below intact.
        self.last_device_ns = Some(device_ns);
        let offset = host_ns as i64 - device_ns as i64;
        let (d0, y0) = *self.reference.get_or_insert((device_ns, offset));
        let (x, y) = (device_ns as f64 - d0 as f64, (offset - y0) as f64);
        if (y - self.a - self.b * x).abs() > self.reset_ns {
            *self = LinearClock::new(self.bin_ns, self.window);
            return self.original_observe(device_ns, host_ns);
        }
        let bin = device_ns / self.bin_ns;
        match self.current {
            Some((b, _, cy)) if b == bin => {
                if y >= cy {
                    return;
                }
                self.current = Some((bin, x, y));
            }
            Some((b, cx, cy)) if bin > b => {
                self.bins.push_back((cx, cy));
                while self.bins.len() >= self.window {
                    self.bins.pop_front();
                }
                self.current = Some((bin, x, y));
            }
            Some(_) => return, // stamps going backwards: ignore
            None => self.current = Some((bin, x, y)),
        }
        self.refit();
    }
}

#[cfg(test)]
#[test]
fn skipped_refits_match_original_for_every_observation() {
    for window in [1, 4, 5, 20] {
        let mut a = LinearClock::new(100_000_000, window);
        let mut b = a.clone();
        for k in 0..5000u64 {
            let d = k * 1_000_000;
            let delay = 1_000_000 - (k % 100) * 1000;
            let h = 5_000_000_000 + d + delay + if k % 1703 == 0 { 100_000_000 } else { 0 };
            a.original_observe(d, h);
            b.observe(d, h);
            assert_eq!(format!("{a:?}"), format!("{b:?}"));
            assert_eq!(
                (a.a.to_bits(), a.b.to_bits()),
                (b.a.to_bits(), b.b.to_bits())
            );
            assert_eq!(a.to_host(d), b.to_host(d));
            assert_eq!(a.to_device(h), b.to_device(h));
        }
    }
}

#[cfg(test)]
#[test]
fn direct_observe_keeps_original_policy_for_small_backward_and_reordered_stamps() {
    for warm_seconds in [2, 10] {
        let mut original = LinearClock::default();
        let mut direct = original.clone();
        for tick in 0..warm_seconds * 500 {
            let device = 1_000_000_000 + tick * 2_000_000;
            original.original_observe(device, device + 10_000_000_000);
            assert!(!direct.observe(device, device + 10_000_000_000));
        }
        let edge = 1_000_000_000 + warm_seconds * 1_000_000_000;
        for (device, received) in [
            (edge + 2_000_000, edge + 10_002_000_000),
            // Small reorder within the current bin, then a report from the preceding bin.
            (edge + 1_000_000, edge + 10_004_000_000),
            (edge - 1_000_000, edge + 10_006_000_000),
            (edge + 2_000_000, edge + 10_008_000_000),
            (edge + 4_000_000, edge + 10_010_000_000),
        ] {
            original.original_observe(device, received);
            assert!(
                !direct.observe(device, received),
                "legacy reorder must not relock"
            );
            assert_eq!(format!("{original:?}"), format!("{direct:?}"));
            for query in [edge - 10_000_000, edge, device] {
                assert_eq!(direct.to_host(query), original.to_host(query));
            }
            assert_eq!(direct.to_device(received), original.to_device(received));
        }
    }
}
