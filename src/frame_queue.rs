// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Bounded camera ingress. Each admitted sensor keeps its two newest frames, and
//! round-robin service prevents a fast camera from displacing a slower camera. A sensor is
//! admitted as one native object (its callback lifetime): frames of any other object of its
//! serial, such as the one a replug replaced, are never queued for it.

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
    /// Per admitted serial: its object's lifetime, and its frames by exposure time.
    frames: BTreeMap<String, (u64, VecDeque<(u64, F)>)>,
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
    /// names never grow the registry, so an inactive consumer cannot grow memory. Each serial
    /// comes with the lifetime of its open native object.
    pub fn register(&mut self, sensors: impl IntoIterator<Item = (String, u64)>) {
        self.frames.clear();
        self.roster.clear();
        self.clocks.clear();
        self.next = 0;
        for (serial, lifetime) in sensors {
            if let std::collections::btree_map::Entry::Vacant(entry) =
                self.frames.entry(serial.clone())
            {
                // Allocate before callbacks start, and never temporarily exceed this bound.
                entry.insert((lifetime, VecDeque::with_capacity(FRAMES_PER_SENSOR)));
                self.roster.push(serial);
            }
        }
    }

    /// Admit the native object `lifetime` a running engine added, without disturbing other
    /// queues: its frames are queued from now on. It replaces another object of its serial and
    /// discards that one's frames; adding the object already admitted changes nothing.
    pub fn add(&mut self, serial: &str, lifetime: u64) {
        if let Some(admitted) = self
            .roster
            .iter()
            .find(|s| crate::status::same_sensor(s, serial))
        {
            let (admitted, frames) = self.frames.get_mut(admitted).unwrap();
            if *admitted != lifetime {
                *admitted = lifetime;
                frames.clear();
            }
            return;
        }
        self.frames.insert(
            serial.to_string(),
            (lifetime, VecDeque::with_capacity(FRAMES_PER_SENSOR)),
        );
        self.roster.push(serial.to_string());
    }

    /// Retire one sensor, rejecting its later callbacks without disturbing other queues.
    pub fn remove(&mut self, serial: &str) {
        if let Some(index) = self
            .roster
            .iter()
            .position(|s| crate::status::same_sensor(s, serial))
        {
            let serial = self.roster.remove(index);
            self.frames.remove(&serial);
            if index < self.next {
                self.next -= 1;
            }
            if self.next >= self.roster.len() {
                self.next = 0;
            }
        }
    }

    pub fn clear(&mut self) {
        for (_, frames) in self.frames.values_mut() {
            frames.clear();
        }
        self.clocks.clear();
    }

    /// Queues a frame of the native object `lifetime`, if that object is the one admitted.
    pub fn push_frame(&mut self, serial: &str, lifetime: u64, timestamp_ns: u64, frame: F) {
        let Some((_, queue)) = self
            .frames
            .get_mut(serial)
            .filter(|(admitted, _)| *admitted == lifetime)
        else {
            self.stats.unknown_frames = self.stats.unknown_frames.saturating_add(1);
            return;
        };
        // Fast and long analysis workers can finish out of order. Keep the newest
        // exposures, not the newest callback arrivals, and consume them in order.
        let mut at = queue.partition_point(|(time, _)| *time < timestamp_ns);
        if queue.get(at).is_some_and(|(time, _)| *time == timestamp_ns) {
            queue[at] = (timestamp_ns, frame);
            self.stats.dropped_frames = self.stats.dropped_frames.saturating_add(1);
            return;
        }
        if queue.len() == FRAMES_PER_SENSOR {
            self.stats.dropped_frames = self.stats.dropped_frames.saturating_add(1);
            if at == 0 {
                return; // older than both retained exposures
            }
            queue.pop_front();
            at -= 1;
        }
        queue.insert(at, (timestamp_ns, frame));
    }

    pub fn push_clock(&mut self, host_ns: u64, device_us: u32) {
        if self.clocks.len() == CLOCK_SAMPLES {
            self.clocks.pop_front();
            self.stats.dropped_clocks = self.stats.dropped_clocks.saturating_add(1);
        }
        self.clocks.push_back((host_ns, device_us));
    }

    pub fn take_clocks(&mut self) -> Vec<(u64, u32)> {
        let mut clocks = Vec::new();
        self.take_clocks_into(&mut clocks);
        clocks
    }

    /// Replace the consumer's batch while keeping both buffers' allocations for the next pass.
    /// The caller can release the ingress lock before fitting clocks from this batch.
    pub fn take_clocks_into(&mut self, clocks: &mut Vec<(u64, u32)>) {
        clocks.clear();
        clocks.extend(self.clocks.drain(..));
    }

    /// Takes the admitted sensor's queued frames that `take` selects, oldest exposure first;
    /// the others stay queued in order.
    pub fn take_sensor_frames(&mut self, serial: &str, mut take: impl FnMut(&F) -> bool) -> Vec<F> {
        let Some((_, queue)) = self.frames.get_mut(serial) else {
            return Vec::new();
        };
        let mut taken = Vec::new();
        let mut i = 0;
        while i < queue.len() {
            if take(&queue[i].1) {
                taken.push(queue.remove(i).unwrap().1);
            } else {
                i += 1;
            }
        }
        taken
    }

    pub fn pop_frame(&mut self) -> Option<F> {
        for _ in 0..self.roster.len() {
            let serial = &self.roster[self.next];
            self.next = (self.next + 1) % self.roster.len();
            if let Some(frame) = self.frames.get_mut(serial).unwrap().1.pop_front() {
                return Some(frame.1);
            }
        }
        None
    }

    pub fn is_empty(&self) -> bool {
        self.clocks.is_empty() && self.frames.values().all(|(_, f)| f.is_empty())
    }

    pub fn stats(&self) -> QueueStats {
        QueueStats {
            sensors: self.frames.len(),
            queued_frames: self.frames.values().map(|(_, f)| f.len()).sum(),
            ..self.stats
        }
    }
}

#[cfg(test)]
#[path = "frame_queue/perf_tests.rs"]
mod perf_tests;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hotplug_removal_keeps_other_sensors_frames_and_rejects_late_callbacks() {
        let mut q = FrameQueue::default();
        q.register([("SENSORALPHA".into(), 1), ("SENSORBETA".into(), 1)]);
        q.push_frame("SENSORALPHA", 1, 1, 1);
        q.push_frame("SENSORBETA", 1, 2, 2);
        q.push_clock(3, 4);
        q.remove("SENSORALPHA");
        q.push_frame("SENSORALPHA", 1, 5, 5);
        assert_eq!(q.pop_frame(), Some(2));
        assert_eq!(q.pop_frame(), None);
        assert_eq!(q.take_clocks(), [(3, 4)]);
        assert_eq!(q.stats().unknown_frames, 1);
        assert_eq!(q.stats().sensors, 1);
        q.remove("SENSORBETA");
        q.remove("SENSORBETA");
        assert_eq!(q.pop_frame(), None);
    }

    #[test]
    fn hotplug_an_added_sensor_queues_its_frames_from_then_on() {
        let mut q = FrameQueue::default();
        q.register([("SENSORALPHA".into(), 1)]);
        q.push_frame("SENSORALPHA", 1, 1, 1);
        q.push_frame("SENSORBETA", 1, 2, 2);
        assert_eq!(q.stats().unknown_frames, 1, "not admitted yet");
        q.add("SENSORBETA", 1);
        q.add("sensorbeta", 1);
        assert_eq!(q.stats().sensors, 2);
        q.push_frame("SENSORBETA", 1, 3, 3);
        q.push_frame("SENSORALPHA", 1, 4, 4);
        assert_eq!(q.pop_frame(), Some(1));
        assert_eq!(q.pop_frame(), Some(3));
        assert_eq!(q.pop_frame(), Some(4));
        assert_eq!(q.pop_frame(), None);
        // replugged: removed, then added again as another object
        q.remove("SENSORBETA");
        q.push_frame("SENSORBETA", 1, 5, 5);
        q.add("SENSORBETA", 2);
        q.push_frame("SENSORBETA", 2, 6, 6);
        assert_eq!(q.pop_frame(), Some(6));
        assert_eq!(q.stats().unknown_frames, 2);
    }

    #[test]
    fn late_analysis_keeps_the_two_newest_exposures_in_order() {
        for times in [[30, 40, 20], [30, 20, 40]] {
            let mut q = FrameQueue::default();
            q.register([("camera".into(), 1)]);
            for t in times {
                q.push_frame("camera", 1, t, t);
            }
            assert_eq!(q.pop_frame(), Some(30));
            assert_eq!(q.pop_frame(), Some(40));
            assert_eq!(q.pop_frame(), None);
        }
    }

    #[test]
    fn a_busy_sensor_cannot_displace_a_quiet_sensors_pending_frames() {
        let mut q = FrameQueue::default();
        q.register([("slow".into(), 1), ("fast".into(), 1)]);
        for t in 0..2 {
            q.push_frame("slow", 1, t, ("slow", t));
        }
        for t in 0..100 {
            q.push_frame("fast", 1, t, ("fast", t));
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
    fn a_sensors_selected_frames_are_taken_and_the_rest_keep_their_turn() {
        let mut q = FrameQueue::default();
        q.register([("a".into(), 1), ("b".into(), 1)]);
        q.push_frame("a", 1, 20, ("a", 20));
        q.push_frame("a", 1, 10, ("a", 10));
        q.push_frame("b", 1, 15, ("b", 15));
        assert_eq!(q.take_sensor_frames("a", |f| f.1 == 10), [("a", 10)]);
        assert!(q.take_sensor_frames("unregistered", |_| true).is_empty());
        assert_eq!(q.stats().queued_frames, 2);
        assert_eq!(q.stats().dropped_frames, 0);
        assert_eq!(q.pop_frame(), Some(("a", 20)));
        assert_eq!(q.pop_frame(), Some(("b", 15)));
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
        q.register((0..65).map(|i| (format!("camera-{i}"), 1)));
        for i in 0..65 {
            for t in 0..100 {
                q.push_frame(&format!("camera-{i}"), 1, t, (i, t));
            }
        }
        for round in [98, 99] {
            for i in 0..65 {
                assert_eq!(q.pop_frame(), Some((i, round)));
            }
        }
        assert!(q.is_empty());
        q.push_frame("unregistered", 1, 0, (100, 0));
        assert_eq!(q.stats().unknown_frames, 1);
        assert_eq!(q.stats().sensors, 65);
        q.push_frame("camera-0", 1, 101, (0, 101));
        q.register([("replacement".into(), 1)]);
        assert_eq!(q.pop_frame(), None);
        q.push_frame("camera-0", 1, 102, (0, 102));
        q.push_frame("replacement", 1, 0, (200, 0));
        assert_eq!(q.pop_frame(), Some((200, 0)));
        assert_eq!(q.stats().sensors, 1);
        q.register([]);
        assert_eq!(q.pop_frame(), None);
    }

    /// Admission is by native object, not serial: a callback of an earlier object of a serial
    /// is never queued for its successor, and replacing the object discards what it queued.
    #[test]
    fn hotplug_frames_are_admitted_only_for_the_rostered_lifetime() {
        let mut q = FrameQueue::default();
        q.register([("SENSORALPHA".to_string(), 1)]);
        q.push_frame("SENSORALPHA", 1, 1, 1);
        q.push_frame("SENSORALPHA", 2, 2, 2);
        assert_eq!(q.stats().unknown_frames, 1, "a lifetime not yet rostered");
        q.push_frame("SENSORALPHA", 0, 3, 3);
        assert_eq!(
            q.stats().unknown_frames,
            2,
            "lifetime zero is never admitted"
        );
        // a replacing add drops the old object's queued frame and rejects its later ones
        q.add("SENSORALPHA", 2);
        assert_eq!(q.stats().sensors, 1);
        assert_eq!(q.stats().queued_frames, 0);
        q.push_frame("SENSORALPHA", 1, 4, 4);
        q.push_frame("SENSORALPHA", 2, 5, 5);
        assert_eq!(q.pop_frame(), Some(5));
        assert_eq!(q.pop_frame(), None);
        assert_eq!(q.stats().unknown_frames, 3);
        // re-adding the rostered lifetime keeps its frames
        q.push_frame("SENSORALPHA", 2, 6, 6);
        q.add("sensoralpha", 2);
        assert_eq!(q.pop_frame(), Some(6));
        // after removal no lifetime of the serial is admitted
        q.remove("SENSORALPHA");
        q.push_frame("SENSORALPHA", 2, 7, 7);
        q.push_frame("SENSORALPHA", 3, 8, 8);
        assert_eq!(q.pop_frame(), None);
        assert_eq!(q.stats().unknown_frames, 5);
        // a reopened engine's roster admits only its own lifetimes
        q.register([("SENSORALPHA".to_string(), 9)]);
        q.push_frame("SENSORALPHA", 2, 9, 9);
        q.push_frame("SENSORALPHA", 9, 10, 10);
        assert_eq!(q.pop_frame(), Some(10));
        assert_eq!(q.pop_frame(), None);
    }
}
