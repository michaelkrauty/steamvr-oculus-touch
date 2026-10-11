// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Bounded camera ingress. Each admitted sensor keeps its two newest frames, and
//! round-robin service prevents a fast camera from displacing a slower camera.

use std::collections::{BTreeMap, VecDeque};

const FRAMES_PER_SENSOR: usize = 2;
const CLOCK_SAMPLES: usize = 32;

#[derive(Clone, Copy, Default, serde::Serialize)]
pub struct QueueStats {
    pub sensors: usize,
    pub queued_frames: usize,
    pub dropped_frames: u64,
    pub dropped_clocks: u64,
    pub unknown_frames: u64,
}

pub struct FrameQueue<F> {
    frames: BTreeMap<String, VecDeque<(u64, F)>>,
    roster: Vec<String>,
    next: usize,
    clocks: VecDeque<(u64, u32)>,
    stats: QueueStats,
}

impl<F> Default for FrameQueue<F> {
    fn default() -> Self {
        Self {
            frames: BTreeMap::new(),
            roster: Vec::new(),
            next: 0,
            clocks: VecDeque::new(),
            stats: QueueStats::default(),
        }
    }
}

impl<F> FrameQueue<F> {
    /// Replace the admitted roster on engine startup/reopen. Callback-provided
    /// names never grow the registry, so an inactive consumer cannot grow memory.
    pub fn register(&mut self, serials: impl IntoIterator<Item = String>) {
        self.frames.clear();
        self.roster.clear();
        self.clocks.clear();
        self.next = 0;
        for serial in serials {
            if let std::collections::btree_map::Entry::Vacant(entry) =
                self.frames.entry(serial.clone())
            {
                entry.insert(VecDeque::new());
                self.roster.push(serial);
            }
        }
    }

    pub fn clear(&mut self) {
        for frames in self.frames.values_mut() {
            frames.clear();
        }
        self.clocks.clear();
    }

    pub fn push_frame(&mut self, serial: &str, timestamp_ns: u64, frame: F) {
        let Some(queue) = self.frames.get_mut(serial) else {
            self.stats.unknown_frames = self.stats.unknown_frames.saturating_add(1);
            return;
        };
        // Fast and long analysis workers can finish out of order. Keep the newest
        // exposures, not the newest callback arrivals, and consume them in order.
        let at = queue.partition_point(|(time, _)| *time < timestamp_ns);
        if queue.get(at).is_some_and(|(time, _)| *time == timestamp_ns) {
            queue[at] = (timestamp_ns, frame);
            self.stats.dropped_frames = self.stats.dropped_frames.saturating_add(1);
            return;
        }
        queue.insert(at, (timestamp_ns, frame));
        if queue.len() > FRAMES_PER_SENSOR {
            queue.pop_front();
            self.stats.dropped_frames = self.stats.dropped_frames.saturating_add(1);
        }
    }

    pub fn push_clock(&mut self, host_ns: u64, device_us: u32) {
        if self.clocks.len() == CLOCK_SAMPLES {
            self.clocks.pop_front();
            self.stats.dropped_clocks = self.stats.dropped_clocks.saturating_add(1);
        }
        self.clocks.push_back((host_ns, device_us));
    }

    pub fn take_clocks(&mut self) -> Vec<(u64, u32)> {
        self.clocks.drain(..).collect()
    }

    pub fn pop_frame(&mut self) -> Option<F> {
        for _ in 0..self.roster.len() {
            let serial = &self.roster[self.next];
            self.next = (self.next + 1) % self.roster.len();
            if let Some(frame) = self.frames.get_mut(serial).unwrap().pop_front() {
                return Some(frame.1);
            }
        }
        None
    }

    pub fn is_empty(&self) -> bool {
        self.clocks.is_empty() && self.frames.values().all(VecDeque::is_empty)
    }

    pub fn stats(&self) -> QueueStats {
        QueueStats {
            sensors: self.frames.len(),
            queued_frames: self.frames.values().map(VecDeque::len).sum(),
            ..self.stats
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn late_analysis_keeps_the_two_newest_exposures_in_order() {
        for times in [[30, 40, 20], [30, 20, 40]] {
            let mut q = FrameQueue::default();
            q.register(["camera".into()]);
            for t in times {
                q.push_frame("camera", t, t);
            }
            assert_eq!(q.pop_frame(), Some(30));
            assert_eq!(q.pop_frame(), Some(40));
            assert_eq!(q.pop_frame(), None);
        }
    }

    #[test]
    fn a_busy_sensor_cannot_displace_a_quiet_sensors_pending_frames() {
        let mut q = FrameQueue::default();
        q.register(["slow".into(), "fast".into()]);
        for t in 0..2 {
            q.push_frame("slow", t, ("slow", t));
        }
        for t in 0..100 {
            q.push_frame("fast", t, ("fast", t));
        }
        assert_eq!(q.stats().queued_frames, 4);
        assert_eq!(q.stats().dropped_frames, 98);
        assert_eq!(q.pop_frame(), Some(("slow", 0)));
        assert_eq!(q.pop_frame(), Some(("fast", 98)));
        assert_eq!(q.pop_frame(), Some(("slow", 1)));
        assert_eq!(q.pop_frame(), Some(("fast", 99)));
        assert_eq!(q.pop_frame(), None);
    }

    #[test]
    fn a_stalled_consumer_has_a_bounded_clock_backlog() {
        let mut q: FrameQueue<()> = FrameQueue::default();
        for t in 0..10_000 {
            q.push_clock(t, t as u32);
        }
        let clocks = q.take_clocks();
        assert_eq!(clocks.len(), 32);
        assert_eq!(clocks.first(), Some(&(9_968, 9_968)));
        assert_eq!(clocks.last(), Some(&(9_999, 9_999)));
        assert_eq!(q.stats().dropped_clocks, 9_968);
        assert!(q.is_empty());
    }

    #[test]
    fn all_sixty_five_sensors_receive_fair_service_and_reopen_replaces_the_roster() {
        let mut q = FrameQueue::default();
        q.register((0..65).map(|i| format!("camera-{i}")));
        for i in 0..65 {
            for t in 0..100 {
                q.push_frame(&format!("camera-{i}"), t, (i, t));
            }
        }
        for round in [98, 99] {
            for i in 0..65 {
                assert_eq!(q.pop_frame(), Some((i, round)));
            }
        }
        assert!(q.is_empty());
        q.push_frame("unregistered", 0, (100, 0));
        assert_eq!(q.stats().unknown_frames, 1);
        assert_eq!(q.stats().sensors, 65);
        q.push_frame("camera-0", 101, (0, 101));
        q.register(["replacement".into()]);
        assert_eq!(q.pop_frame(), None);
        q.push_frame("camera-0", 102, (0, 102));
        q.push_frame("replacement", 0, (200, 0));
        assert_eq!(q.pop_frame(), Some((200, 0)));
        assert_eq!(q.stats().sensors, 1);
        q.register([]);
        assert_eq!(q.pop_frame(), None);
    }
}
