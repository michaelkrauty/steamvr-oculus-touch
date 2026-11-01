// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Controller clock → host CLOCK_MONOTONIC. Each observation satisfies host = device + offset + delay
//! with delay ≥ 0, so the minimum of (host − device) over a sliding two-window span estimates the
//! offset plus the minimum transport delay and follows crystal drift.

#[derive(Debug, Clone)]
pub struct ClockMap {
    valid: bool,
    offset_ns: i64,
    last_device_ns: u64,
    cur_min_ns: i64,
    prev_min_ns: i64,
    window_start_host_ns: u64,
    /// Window length (default 1 s).
    pub window_ns: u64,
    /// A negative residual larger than this re-locks immediately (default 50 ms).
    pub reset_threshold_ns: i64,
}

impl Default for ClockMap {
    fn default() -> Self {
        Self {
            valid: false,
            offset_ns: 0,
            last_device_ns: 0,
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
        // A late host read only increases the residual: let the existing two-window minimum
        // age it in, rather than moving every historical event on a single delayed report.
        // Sustained positive shifts still take effect once both old minima expire (up to 2
        // windows). Backward device epochs and large negative corrections re-lock at once.
        // Native counter restarts can instead extend time FORWARD by a large modular delta;
        // if that advance exceeds host elapsed time, the negative residual re-locks here.
        // Native clears optical evidence independently on its >100 ms continuity reset.
        let restarted = self.valid && device_ns < self.last_device_ns;
        self.last_device_ns = device_ns;
        if !self.valid || restarted || d < self.offset_ns - self.reset_threshold_ns {
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

#[cfg(test)]
mod recovery_tests {
    use super::*;

    #[test]
    fn positive_outliers_use_the_same_minimum_as_small_jitter() {
        for delay in [40_000_000, 50_000_001, 60_000_000, 500_000_000] {
            let mut m = ClockMap::default();
            m.observe(1_000_000_000, 11_000_000_000);
            m.observe(1_002_000_000, 11_002_000_000 + delay);
            assert_eq!(m.to_host(800_000_000), Some(10_800_000_000));
            assert_eq!(m.to_host(1_002_000_000), Some(11_002_000_000));
            // A normal following report cancels the excursion without a re-lock.
            m.observe(2_000_000_000, 12_000_000_000);
            assert_eq!(m.offset_ns(), Some(10_000_000_000));
        }
    }

    #[test]
    fn sustained_positive_offset_adapts_after_old_minima_expire() {
        let mut m = ClockMap::default();
        m.observe(1_000_000_000, 11_000_000_000);
        for i in 1..=1000u64 {
            let dev = 1_000_000_000 + i * 2_000_000;
            m.observe(dev, dev + 10_060_000_000);
            if i == 1 {
                assert_eq!(m.offset_ns(), Some(10_000_000_000));
            }
        }
        assert_eq!(m.offset_ns(), Some(10_060_000_000));
        assert_eq!(m.to_host(0), None);
    }

    #[test]
    fn negative_correction_relocks_and_negative_drift_tracks() {
        let mut m = ClockMap::default();
        m.observe(1_000_000_000, 11_060_000_000);
        m.observe(1_002_000_000, 11_002_000_000);
        assert_eq!(m.offset_ns(), Some(10_000_000_000));
        for i in 1..=5000u64 {
            let dev = 1_002_000_000 + i * 2_000_000;
            let host = dev + 10_000_000_000 - i * 200; // -100 ppm
            m.observe(dev, host);
            assert_eq!(m.to_host(dev), Some(host));
        }
    }

    #[test]
    fn backward_epoch_relocks_even_for_a_small_offset_change() {
        let mut m = ClockMap::default();
        m.observe(1_000_000_000, 11_000_000_000);
        m.observe(998_000_000, 11_002_000_000);
        assert_eq!(m.to_host(998_000_000), Some(11_002_000_000));
        assert_eq!(m.to_host(0), None);
    }
}
