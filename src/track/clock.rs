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
    /// A report this far off the fitted line re-locks (a device clock restart).
    reset_ns: f64,
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
            reference: None,
            bins: VecDeque::new(),
            current: None,
            a: 0.0,
            b: 0.0,
        }
    }

    pub fn observe(&mut self, device_ns: u64, host_ns: u64) {
        let offset = host_ns as i64 - device_ns as i64;
        let (d0, y0) = *self.reference.get_or_insert((device_ns, offset));
        let (x, y) = (device_ns as f64 - d0 as f64, (offset - y0) as f64);
        if (y - self.a - self.b * x).abs() > self.reset_ns {
            *self = LinearClock::new(self.bin_ns, self.window);
            return self.observe(device_ns, host_ns);
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
                    let mut c = LinearClock::default();
                    c.bins = VecDeque::with_capacity(n);
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
        for &(d, h) in &drifting(60_000, 1_000_000) {
            c.observe(d, h);
        }
        // truth at the newest report: host = (device - 1e6) / (1 + 20e-6) + 5e12, plus the 0.3 ms minimum delay
        let d = 1_000_000 + (59_999_000_000f64 * (1.0 + 20e-6)) as u64;
        let truth = 5_000_000_000_000 + ((d - 1_000_000) as f64 / (1.0 + 20e-6)) as u64 + 300_000;
        let err = c.to_host(d).unwrap() as i64 - truth as i64;
        assert!(err.abs() < 20_000, "error {err} ns");
        assert!((c.to_device(c.to_host(d).unwrap()).unwrap() as i64 - d as i64).abs() <= 1);
    }

    #[test]
    fn a_clock_restart_relocks() {
        let mut c = LinearClock::default();
        for k in 0..5000u64 {
            c.observe(k * 2_000_000, 9_000_000_000 + k * 2_000_000);
        }
        c.observe(1_000, 9_000_000_000 + 5000 * 2_000_000);
        assert_eq!(c.to_host(1_000), Some(9_000_000_000 + 5000 * 2_000_000));
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
