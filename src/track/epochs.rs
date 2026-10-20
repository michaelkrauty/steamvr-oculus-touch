// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Inspection evidence follows a correspondence's lifetime. Losing its blobs ends that
//! correspondence; discovering a wrong identity invalidates its earlier evidence as well.
//!
//! Only an invalidation restricts evidence. Native code reports an invalidation only for the
//! live epoch and clears it on any end, so a normally ended epoch can never be retracted
//! (`end_inspection_identity`, rift-sensor.c). An epoch absent above the watermark is
//! permitted: starts and normal ends leave no lifecycle state, however long the session.

use std::collections::BTreeMap;

/// Tombstones and counted epochs per sensor and hand, excluding epoch zero. A consumer retires
/// a sensor's tombstones once its engine search is over (`Pipeline::retire_identities`), and
/// counts live for one inspection, so only a stalled consumer, or one search with more
/// invalidations than this, reaches it. Forgetting old evidence at the bound is safe;
/// forgetting its invalidation and accepting it again is not, so overflow leaves a watermark
/// that rejects everything at or below it.
const MAX_EPOCHS: usize = 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Started,
    Ended,
    Invalidated,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Checks {
    pub checks: u64,
    pub misses: u64,
}

impl Checks {
    pub fn since(self, old: Self) -> Self {
        Self {
            checks: self.checks.saturating_sub(old.checks),
            misses: self.misses.saturating_sub(old.misses),
        }
    }

    pub fn add(&mut self, other: Self) {
        self.checks += other.checks;
        self.misses += other.misses;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Epoch {
    pub state: State,
    pub observed: Checks,
}

/// One sensor and hand's tombstones, with an inspection's observed counts by epoch when it is
/// that inspection's copy. Epoch zero is ordinary matching, which did not use an inspection
/// identity. A labelled inspection result must have its nonzero native epoch; an old result
/// without that provenance cannot judge movement.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Evidence {
    pub epochs: BTreeMap<u64, Epoch>,
    pub forgotten_through: u64,
}

impl Evidence {
    /// Invalidation is retrospective and sticky, even if a delayed end or start arrives. A
    /// start or a normal end restricts nothing, so it is not kept: keeping it would only count
    /// toward the bound, and a late notice or count could revive it.
    pub fn note(&mut self, epoch: u64, state: State) {
        if epoch == 0 || epoch <= self.forgotten_through || state != State::Invalidated {
            return;
        }
        self.epochs.insert(
            epoch,
            Epoch {
                state,
                observed: Checks::default(),
            },
        );
        self.bound();
    }

    pub fn count(&mut self, epoch: u64, observed: Checks) {
        if epoch != 0 && epoch <= self.forgotten_through || observed.checks == 0 {
            return;
        }
        let item = self.epochs.entry(epoch).or_insert(Epoch {
            state: State::Started,
            observed: Checks::default(),
        });
        if item.state != State::Invalidated {
            item.observed.add(observed);
        }
        self.bound();
    }

    pub fn permits(&self, epoch: u64) -> bool {
        (epoch == 0 || epoch > self.forgotten_through)
            && self
                .epochs
                .get(&epoch)
                .is_none_or(|e| e.state != State::Invalidated)
    }

    /// A newly invalidated or forgotten lifetime can no longer support residual statistics.
    pub fn invalidates(&self, previous: &Self) -> bool {
        self.forgotten_through > previous.forgotten_through
            || self
                .epochs
                .iter()
                .any(|(&epoch, item)| item.state == State::Invalidated && previous.permits(epoch))
    }

    /// Merge another lifecycle state or delta: its tombstones and watermark, never its counts.
    /// Coalescing preserves invalidations even when the consumer's camera-frame queue drops
    /// frames, and never revives forgotten epochs.
    pub fn lifecycle(&mut self, other: &Self) {
        self.forgotten_through = self.forgotten_through.max(other.forgotten_through);
        self.epochs
            .retain(|&epoch, _| epoch == 0 || epoch > self.forgotten_through);
        for (&epoch, item) in &other.epochs {
            self.note(epoch, item.state);
        }
    }

    /// Nothing can carry these epochs any more: drop the tombstones. The watermark stays.
    pub fn retire(&mut self) {
        self.epochs.retain(|_, e| e.state != State::Invalidated);
    }

    /// Epoch zero is ordinary matching, not an identity lifetime: it never counts here.
    fn bound(&mut self) {
        while self.epochs.len() - usize::from(self.epochs.contains_key(&0)) > MAX_EPOCHS {
            let oldest = *self.epochs.range(1..).next().unwrap().0;
            self.epochs.remove(&oldest);
            self.forgotten_through = self.forgotten_through.max(oldest);
        }
    }
}

impl State {
    pub fn from_native(value: u32) -> Option<Self> {
        match value {
            crate::ffi::OHMD_CV1TOUCH_IDENTITY_STARTED => Some(Self::Started),
            crate::ffi::OHMD_CV1TOUCH_IDENTITY_ENDED => Some(Self::Ended),
            crate::ffi::OHMD_CV1TOUCH_IDENTITY_INVALIDATED => Some(Self::Invalidated),
            _ => None,
        }
    }
}

/// Lifecycle notices cannot share the dropping camera-frame queue. Coalesce the invalidations
/// by epoch, bounded per registered sensor and hand, and transfer ownership before doing
/// tracking work.
#[derive(Default)]
pub struct Mailbox {
    sensors: BTreeMap<String, [Evidence; 2]>,
}

impl Mailbox {
    pub fn register(&mut self, serials: impl IntoIterator<Item = String>) {
        self.sensors = serials
            .into_iter()
            .map(|s| (s, Default::default()))
            .collect();
    }

    pub fn clear(&mut self) {
        for evidence in self.sensors.values_mut() {
            *evidence = Default::default();
        }
    }

    pub fn note(&mut self, event: &crate::engine::InspectionIdentity) {
        let Some(hand) = super::pipeline::device_hand(event.device_id) else {
            return;
        };
        let Some(state) = State::from_native(event.state) else {
            return;
        };
        let serial = crate::engine::cstr_field(&event.serial);
        if let Some((_, evidence)) = self
            .sensors
            .iter_mut()
            .find(|(s, _)| crate::status::same_sensor(s, &serial))
        {
            evidence[hand].note(event.epoch, state);
        }
    }

    pub fn take(&mut self) -> Vec<(String, [Evidence; 2])> {
        self.sensors
            .iter_mut()
            .filter_map(|(serial, evidence)| {
                evidence
                    .iter()
                    .any(|e| !e.epochs.is_empty() || e.forgotten_through != 0)
                    .then(|| (serial.clone(), std::mem::take(evidence)))
            })
            .collect()
    }
}

/// Commit a movement or placement decision against the latest native lifecycle notices.
/// Tracking and solving run outside this ordering boundary.
pub fn commit<T>(
    mailbox: &std::sync::Mutex<Mailbox>,
    pipe: &mut super::pipeline::Pipeline,
    decide: impl FnOnce(&mut super::pipeline::Pipeline) -> T,
) -> T {
    let mut pending = mailbox.lock().unwrap();
    for (serial, states) in pending.take() {
        pipe.inspection_lifecycle(&serial, &states);
    }
    let result = decide(pipe);
    drop(pending);
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normal_ends_keep_counts_and_invalidations_cannot_be_undone() {
        let mut evidence = Evidence::default();
        evidence.count(
            1,
            Checks {
                checks: 50,
                misses: 49,
            },
        );
        evidence.count(
            2,
            Checks {
                checks: 30,
                misses: 30,
            },
        );
        evidence.note(1, State::Ended);
        evidence.note(2, State::Invalidated);
        evidence.note(2, State::Ended);
        evidence.note(2, State::Started);
        evidence.count(
            2,
            Checks {
                checks: 100,
                misses: 100,
            },
        );
        assert_eq!(
            evidence.epochs[&1].observed,
            Checks {
                checks: 50,
                misses: 49
            }
        );
        assert_eq!(evidence.epochs[&2].observed, Checks::default());
        assert_eq!(evidence.epochs[&2].state, State::Invalidated);
    }

    #[test]
    fn lifecycle_delivery_is_bounded_and_independent_of_dropped_frames() {
        let mut mailbox = Mailbox::default();
        mailbox.register(["CAMERA".into()]);
        let mut event = crate::engine::InspectionIdentity {
            device_id: crate::calib::DEVICE_LEFT,
            state: crate::ffi::OHMD_CV1TOUCH_IDENTITY_INVALIDATED,
            ..Default::default()
        };
        for (d, b) in event.serial.iter_mut().zip(b"camera") {
            *d = *b as _;
        }
        let mut frames = crate::frame_queue::FrameQueue::default();
        frames.register(["CAMERA".into()]);
        for epoch in 1..=2 * MAX_EPOCHS as u64 {
            event.epoch = epoch;
            mailbox.note(&event);
            event.state = crate::ffi::OHMD_CV1TOUCH_IDENTITY_ENDED;
            mailbox.note(&event);
            event.state = crate::ffi::OHMD_CV1TOUCH_IDENTITY_INVALIDATED;
            frames.push_frame("CAMERA", epoch, ());
        }
        frames.clear();
        let delivery = mailbox.take();
        assert_eq!(delivery.len(), 1);
        let mut consumer = Evidence::default();
        consumer.lifecycle(&delivery[0].1[crate::shm::HAND_LEFT]);
        assert_eq!(consumer.epochs.len(), MAX_EPOCHS);
        for epoch in 1..=2 * MAX_EPOCHS as u64 {
            consumer.count(
                epoch,
                Checks {
                    checks: 1,
                    misses: 1,
                },
            );
            assert!(!consumer.permits(epoch));
        }
        assert!(consumer
            .epochs
            .values()
            .all(|e| e.observed == Checks::default()));
        assert!(delivery[0].1[crate::shm::HAND_RIGHT].epochs.is_empty());
        assert!(mailbox.take().is_empty());
        // A start and a normal end restrict nothing: there is nothing to deliver, and the
        // epoch's checks stay with the consumer.
        event.epoch += 1;
        consumer.count(
            event.epoch,
            Checks {
                checks: 7,
                misses: 6,
            },
        );
        for state in [
            crate::ffi::OHMD_CV1TOUCH_IDENTITY_STARTED,
            crate::ffi::OHMD_CV1TOUCH_IDENTITY_ENDED,
        ] {
            event.state = state;
            mailbox.note(&event);
        }
        assert!(mailbox.take().is_empty());
        assert_eq!(consumer.epochs[&event.epoch].observed.checks, 7);
        mailbox.register([]);
        event.state = crate::ffi::OHMD_CV1TOUCH_IDENTITY_INVALIDATED;
        mailbox.note(&event);
        assert!(mailbox.take().is_empty());
    }

    /// Native code invalidates only the live epoch and clears it on any end, so an ended epoch
    /// can never be retracted. However long the session, only invalidations are kept: nothing
    /// overflows, a late frame of an ended epoch still counts, and the invalidated one stays
    /// rejected until the consumer retires it, without moving the watermark.
    #[test]
    fn only_invalidations_are_kept_however_long_the_session() {
        let invalid = 2500;
        let mut lifetimes = Evidence::default();
        for epoch in 1..=5000 {
            lifetimes.note(epoch, State::Started);
            lifetimes.note(
                epoch,
                if epoch == invalid {
                    State::Invalidated
                } else {
                    State::Ended
                },
            );
        }
        assert_eq!(lifetimes.forgotten_through, 0);
        assert_eq!(
            lifetimes.epochs.keys().copied().collect::<Vec<_>>(),
            [invalid]
        );
        assert!((1..=5000).all(|epoch| lifetimes.permits(epoch) == (epoch != invalid)));
        // An inspection's counts: a late frame of an ended epoch counts, the invalidated
        // epoch's do not, and merging the counts back cannot revive anything.
        let mut observed = lifetimes.clone();
        let late = Checks {
            checks: 3,
            misses: 3,
        };
        observed.count(invalid - 1, late);
        observed.count(invalid, late);
        assert_eq!(observed.epochs[&(invalid - 1)].observed, late);
        assert_eq!(observed.epochs[&invalid].observed, Checks::default());
        let before = lifetimes.clone();
        lifetimes.lifecycle(&observed);
        assert_eq!(lifetimes, before, "counts carry no lifecycle state");
        lifetimes.note(invalid, State::Started);
        lifetimes.note(invalid, State::Ended);
        assert!(!lifetimes.permits(invalid), "invalidation is sticky");
        lifetimes.retire();
        assert_eq!(lifetimes, Evidence::default());
        assert!(lifetimes.permits(invalid - 1) && lifetimes.permits(invalid));
    }

    /// Epoch zero is ordinary matching, not an identity lifetime: an inspection's copy of a
    /// full set of lifetimes can still count ordinary checks without overflowing.
    #[test]
    fn epoch_zero_does_not_count_toward_the_bound() {
        let mut evidence = Evidence::default();
        for epoch in 1..=MAX_EPOCHS as u64 {
            evidence.note(epoch, State::Invalidated);
        }
        evidence.count(
            0,
            Checks {
                checks: 5,
                misses: 0,
            },
        );
        assert_eq!(evidence.forgotten_through, 0);
        assert_eq!(evidence.epochs.len(), MAX_EPOCHS + 1);
        assert!(!evidence.permits(1));
        evidence.note(MAX_EPOCHS as u64 + 1, State::Invalidated);
        assert_eq!(
            evidence.forgotten_through, 1,
            "a real overflow still bounds it"
        );
        assert_eq!(evidence.epochs[&0].observed.checks, 5);
    }
}
