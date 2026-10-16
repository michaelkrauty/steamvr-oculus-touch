// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Recording analysis for live verification: per-window connection/position fractions,
//! receive→publish latency, and how soon a controller's silence is noticed (reporting→silent).
//! A controller held through a radio gap is not reporting.

use crate::hand::{reporting, FLAG_POSITION};
use crate::shm::ControllerState;

#[derive(Debug, Clone, PartialEq)]
pub struct Window {
    pub name: String,
    pub start_ns: u64,
    pub end_ns: u64,
}

pub fn parse_window(s: &str) -> Result<Window, String> {
    let parts: Vec<&str> = s.split(':').collect();
    let [name, start, end] = parts.as_slice() else {
        return Err(format!("window '{s}': expected NAME:START_NS:END_NS"));
    };
    let start_ns: u64 = start
        .parse()
        .map_err(|_| format!("window '{s}': bad start"))?;
    let end_ns: u64 = end.parse().map_err(|_| format!("window '{s}': bad end"))?;
    if end_ns <= start_ns {
        return Err(format!("window '{s}': end must be after start"));
    }
    Ok(Window {
        name: name.to_string(),
        start_ns,
        end_ns,
    })
}

#[derive(Debug, Clone, PartialEq)]
pub struct HandWindowStats {
    pub n: usize,
    pub connected: f64,
    pub position: f64,
    pub latency_p50_ns: Option<u64>,
    pub latency_p99_ns: Option<u64>,
}

/// `states`: (record host_ns, hand, state) in recording order.
pub fn window_stats(
    states: &[(u64, usize, ControllerState)],
    hand: usize,
    w: &Window,
) -> HandWindowStats {
    let sel: Vec<&ControllerState> = states
        .iter()
        .filter(|(t, h, _)| *h == hand && *t >= w.start_ns && *t <= w.end_ns)
        .map(|(_, _, s)| s)
        .collect();
    let n = sel.len();
    let frac = |has: &dyn Fn(u32) -> bool| {
        if n == 0 {
            0.0
        } else {
            sel.iter().filter(|s| has(s.flags)).count() as f64 / n as f64
        }
    };
    // Only fresh samples measure receive->publish latency: the daemon republishes both hands on
    // every engine iteration, so a republished state carries an older sample.
    let mut last_device_ns = None;
    let mut lat: Vec<u64> = Vec::new();
    for (t, h, s) in states {
        if *h != hand {
            continue;
        }
        let fresh = last_device_ns != Some(s.device_time_ns);
        last_device_ns = Some(s.device_time_ns);
        if fresh && *t >= w.start_ns && *t <= w.end_ns && reporting(s.flags) {
            lat.push(s.publish_host_ns.saturating_sub(s.received_host_ns));
        }
    }
    lat.sort_unstable();
    let pct =
        |p: f64| (!lat.is_empty()).then(|| lat[((lat.len() - 1) as f64 * p).round() as usize]);
    HandWindowStats {
        n,
        connected: frac(&reporting),
        position: frac(&|f| f & FLAG_POSITION != 0),
        latency_p50_ns: pct(0.5),
        latency_p99_ns: pct(0.99),
    }
}

/// For each flip from reporting to silent (a radio gap or standby): (host time of the flip, time
/// since the last received report).
pub fn disconnect_delays(states: &[(u64, usize, ControllerState)], hand: usize) -> Vec<(u64, u64)> {
    let mut out = Vec::new();
    let mut prev_connected = false;
    let mut last_received = 0u64;
    for (t, h, s) in states {
        if *h != hand {
            continue;
        }
        let connected = reporting(s.flags);
        if prev_connected && !connected {
            out.push((*t, t.saturating_sub(last_received)));
        }
        if connected {
            last_received = s.received_host_ns;
        }
        prev_connected = connected;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hand::{FLAG_CALIBRATED, FLAG_CONNECTED, FLAG_POSITION};

    fn st(publish: u64, received: u64, flags: u32) -> ControllerState {
        ControllerState {
            publish_host_ns: publish,
            received_host_ns: received,
            flags,
            ..Default::default()
        }
    }

    /// Republished states carry an older sample; only fresh samples measure receive->publish latency.
    #[test]
    fn latency_ignores_republished_samples() {
        let live = FLAG_CALIBRATED | FLAG_CONNECTED | FLAG_POSITION;
        let mut states = Vec::new();
        for i in 0..50u64 {
            let rx = 1_000_000_000 + i * 2_000_000;
            let fresh = ControllerState {
                device_time_ns: i + 1,
                received_host_ns: rx,
                publish_host_ns: rx + 200_000,
                flags: live,
                ..Default::default()
            };
            let stale = ControllerState {
                publish_host_ns: rx + 1_900_000,
                ..fresh
            };
            states.push((rx + 200_000, 0usize, fresh));
            states.push((rx + 1_900_000, 0usize, stale));
        }
        let w = Window {
            name: "all".into(),
            start_ns: 0,
            end_ns: u64::MAX,
        };
        let s = window_stats(&states, 0, &w);
        assert_eq!(s.latency_p99_ns, Some(200_000));
        assert_eq!(s.n, 100);
    }

    #[test]
    fn parses_window() {
        assert_eq!(
            parse_window("move:10:20"),
            Ok(Window {
                name: "move".into(),
                start_ns: 10,
                end_ns: 20
            })
        );
        assert!(parse_window("move:20").is_err());
        assert!(parse_window("move:30:20").is_err());
    }

    #[test]
    fn window_and_disconnect_stats() {
        let live = FLAG_CALIBRATED | FLAG_CONNECTED | FLAG_POSITION;
        let mut states = Vec::new();
        for i in 0..100u64 {
            let t = 1_000_000_000 + i * 2_000_000;
            states.push((t, 0usize, st(t, t - 300_000, live))); // 0.3 ms receive->publish
        }
        let last_rx = 1_000_000_000 + 99 * 2_000_000 - 300_000;
        states.push((
            last_rx + 100_000_000,
            0,
            st(last_rx + 100_000_000, last_rx, FLAG_CALIBRATED),
        ));
        let w = Window {
            name: "all".into(),
            start_ns: 0,
            end_ns: u64::MAX,
        };
        let s = window_stats(&states, 0, &w);
        assert_eq!(s.n, 101);
        assert!((s.connected - 100.0 / 101.0).abs() < 1e-9);
        assert_eq!(s.latency_p50_ns, Some(300_000));
        assert_eq!(
            disconnect_delays(&states, 0),
            vec![(last_rx + 100_000_000, 100_000_000)]
        );
        assert_eq!(window_stats(&states, 1, &w).n, 0);
    }

    /// A controller held through a radio gap is published connected, but is silent.
    #[test]
    fn a_radio_gap_is_silence() {
        let live = FLAG_CALIBRATED | FLAG_CONNECTED | FLAG_POSITION;
        let held = FLAG_CALIBRATED | FLAG_CONNECTED | crate::hand::FLAG_RADIO_GAP;
        let states = vec![
            (1_000_000_000, 1usize, st(1_000_000_000, 999_000_000, live)),
            (1_101_000_000, 1, st(1_101_000_000, 999_000_000, held)),
            (1_200_000_000, 1, st(1_200_000_000, 999_000_000, held)),
            (1_300_000_000, 1, st(1_300_000_000, 1_299_000_000, live)),
        ];
        let w = Window {
            name: "all".into(),
            start_ns: 0,
            end_ns: u64::MAX,
        };
        assert!((window_stats(&states, 1, &w).connected - 0.5).abs() < 1e-9);
        assert_eq!(
            disconnect_delays(&states, 1),
            vec![(1_101_000_000, 102_000_000)]
        );
    }
}
