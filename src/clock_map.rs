// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Controller clock → host CLOCK_MONOTONIC. Each observation satisfies host = device + offset + delay
//! with delay ≥ 0, so the minimum of (host − device) over a sliding two-window span estimates the
//! offset plus the minimum transport delay and follows crystal drift.

#[derive(Debug, Clone)]
pub struct ClockMap {
    valid: bool,
    offset_ns: i64,
    cur_min_ns: i64,
    prev_min_ns: i64,
    window_start_host_ns: u64,
    /// Window length (default 1 s).
    pub window_ns: u64,
    /// A jump larger than this re-locks, e.g. after a controller clock restart (default 50 ms).
    pub reset_threshold_ns: i64,
}

impl Default for ClockMap {
    fn default() -> Self {
        Self {
            valid: false,
            offset_ns: 0,
            cur_min_ns: 0,
            prev_min_ns: 0,
            window_start_host_ns: 0,
            window_ns: 1_000_000_000,
            reset_threshold_ns: 50_000_000,
        }
    }
}

impl ClockMap {
    pub fn observe(&mut self, device_ns: u64, host_ns: u64) {
        let d = host_ns.wrapping_sub(device_ns) as i64;
        if !self.valid || (d - self.offset_ns).abs() > self.reset_threshold_ns {
            self.valid = true;
            self.offset_ns = d;
            self.cur_min_ns = d;
            self.prev_min_ns = d;
            self.window_start_host_ns = host_ns;
            return;
        }
        if host_ns.saturating_sub(self.window_start_host_ns) >= self.window_ns {
            self.prev_min_ns = self.cur_min_ns;
            self.cur_min_ns = d;
            self.window_start_host_ns = host_ns;
        } else if d < self.cur_min_ns {
            self.cur_min_ns = d;
        }
        self.offset_ns = self.cur_min_ns.min(self.prev_min_ns);
    }

    /// None before the first observation, and for device_ns == 0 ("never").
    pub fn to_host(&self, device_ns: u64) -> Option<u64> {
        if !self.valid || device_ns == 0 {
            return None;
        }
        Some((device_ns as i64 + self.offset_ns) as u64)
    }

    pub fn offset_ns(&self) -> Option<i64> {
        self.valid.then_some(self.offset_ns)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constant_delay() {
        let mut m = ClockMap::default();
        assert_eq!(m.to_host(123), None);
        for i in 0..100u64 {
            let dev = 5_000_000_000 + i * 2_000_000;
            m.observe(dev, dev + 7_000_000_000 + 1_500_000);
        }
        assert_eq!(
            m.to_host(6_000_000_000),
            Some(6_000_000_000 + 7_001_500_000)
        );
        assert_eq!(m.to_host(0), None);
    }

    #[test]
    fn jitter_takes_minimum() {
        let mut m = ClockMap::default();
        for i in 0..2000u64 {
            let dev = 1_000_000_000 + i * 2_000_000;
            let delay = 1_500_000 + ((i * 7919) % 2500) * 1000; // 1.5–4.0 ms
            m.observe(dev, dev + 3_000_000_000 + delay);
        }
        let off = m.offset_ns().unwrap();
        assert!(
            (3_001_500_000..=3_001_550_000).contains(&off),
            "offset {off}"
        );
    }

    #[test]
    fn drift_tracking() {
        let mut m = ClockMap::default();
        let (mut dev, mut host) = (0u64, 0u64);
        for i in 0..5000u64 {
            // 10 s at 500 Hz, host clock 100 ppm fast
            dev = 1_000_000_000 + i * 2_000_000;
            host = 9_000_000_000 + (dev as f64 * (1.0 + 1e-4)) as u64 + 1_500_000;
            m.observe(dev, host);
        }
        let err = m.to_host(dev).unwrap() as i64 - host as i64;
        assert!(err <= 0 && err > -250_000, "err {err}");
    }

    #[test]
    fn device_clock_restart_relocks() {
        let mut m = ClockMap::default();
        for i in 0..10u64 {
            m.observe(1_000_000 * i, 5_000_000_000 + 1_000_000 * i);
        }
        m.observe(2_000_000, 20_000_000_000);
        assert_eq!(m.to_host(2_000_000), Some(20_000_000_000));
    }
}
