// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Converts one engine controller snapshot into the published shared-memory state: connection and
//! standby detection, radio gaps, position-hold window, per-hand button mapping, capacitive touch
//! hysteresis and device→host time mapping.

use crate::clock_map::ClockMap;
use crate::ffi;
use crate::shm::{ControllerState, HAND_LEFT, HAND_RIGHT};

pub const FLAG_CALIBRATED: u32 = ffi::CV1T_FLAG_CALIBRATED;
pub const FLAG_CONNECTED: u32 = ffi::CV1T_FLAG_CONNECTED;
pub const FLAG_POSITION: u32 = ffi::CV1T_FLAG_POSITION;
pub const FLAG_RADIO_GAP: u32 = ffi::CV1T_FLAG_RADIO_GAP;
pub const FLAG_ASLEEP: u32 = ffi::CV1T_FLAG_ASLEEP;
pub const BTN_A: u32 = ffi::CV1T_BTN_A;
pub const BTN_B: u32 = ffi::CV1T_BTN_B;
pub const BTN_X: u32 = ffi::CV1T_BTN_X;
pub const BTN_Y: u32 = ffi::CV1T_BTN_Y;
pub const BTN_SYSTEM: u32 = ffi::CV1T_BTN_SYSTEM;
pub const BTN_STICK: u32 = ffi::CV1T_BTN_STICK;
pub const TOUCH_A: u32 = ffi::CV1T_TOUCH_A;
pub const TOUCH_B: u32 = ffi::CV1T_TOUCH_B;
pub const TOUCH_X: u32 = ffi::CV1T_TOUCH_X;
pub const TOUCH_Y: u32 = ffi::CV1T_TOUCH_Y;
pub const TOUCH_TRIGGER: u32 = ffi::CV1T_TOUCH_TRIGGER;
pub const TOUCH_STICK: u32 = ffi::CV1T_TOUCH_STICK;
pub const TOUCH_THUMBREST: u32 = ffi::CV1T_TOUCH_THUMBREST;
pub const CAP_STICK: usize = ffi::CV1T_CAP_STICK as usize;
pub const CAP_BY: usize = ffi::CV1T_CAP_BY as usize;
pub const CAP_TRIGGER: usize = ffi::CV1T_CAP_TRIGGER as usize;
pub const CAP_AX: usize = ffi::CV1T_CAP_AX as usize;
pub const CAP_THUMBREST: usize = ffi::CV1T_CAP_THUMBREST as usize;

/// Battery reports in a radio report from the headset (64 bytes, id 0x0c, two 28-byte messages):
/// (hand, percent) for each Touch message whose multiplexed ADC channel is the battery (0x21).
pub fn battery_from_radio(d: &[u8]) -> Vec<(usize, i32)> {
    if d.len() != 64 || d[0] != 0x0c {
        return Vec::new();
    }
    (0..2)
        .filter_map(|i| {
            let m = &d[3 + 28 * i..3 + 28 * (i + 1)];
            let hand = match m[2] {
                2 => HAND_LEFT,
                3 => HAND_RIGHT,
                _ => return None,
            };
            (u16::from_le_bytes([m[0], m[1]]) == 0x1c && m[25] == 0x21)
                .then(|| (hand, u16::from_le_bytes([m[26], m[27]]).min(100) as i32))
        })
        .collect()
}

/// Whether a published state is of a controller whose samples are arriving: connected and not in
/// a radio gap.
pub fn reporting(flags: u32) -> bool {
    flags & FLAG_CONNECTED != 0 && flags & FLAG_RADIO_GAP == 0
}

/// One reader's decoded activity, in message order. Beacons never count as samples.
#[derive(Clone, Copy, Debug)]
pub struct RadioActivity {
    pub device_time_ns: u64,
    pub received_host_ns: u64,
    pub asleep: bool,
    pub moving: bool,
}

#[derive(Default)]
struct ActivitySlot {
    device_ns: std::sync::atomic::AtomicU64,
    host_ns: std::sync::atomic::AtomicU64,
    asleep: std::sync::atomic::AtomicBool,
    moving: std::sync::atomic::AtomicBool,
}

const ACTIVITY_SLOTS: usize = 256;

/// A bounded single-producer, single-consumer queue for one hand on one reader. Each reader
/// writes its own queues; only the thread publishing that reader's hand consumes them. A full
/// queue loses evidence, never blocks the reader or lets uncertain evidence hide a dropout.
///
/// Each reader's callback runs on the thread that also drains its queues: the engine reader
/// inside `Engine::update` on the engine thread, the radio reader inside `RadioDecoder::decode`
/// on the radio thread. No push can fall between a drain's head load and its loss checks, so
/// every loss a drain sees came after every entry it retained, which is where it applies them.
/// A producer running concurrently with its drain could have a loss after the head load applied
/// before an older entry the next drain retains; this daemon has no such producer.
pub struct RadioActivityQueue {
    slots: [ActivitySlot; ACTIVITY_SLOTS],
    head: std::sync::atomic::AtomicUsize,
    tail: std::sync::atomic::AtomicUsize,
    lost: std::sync::atomic::AtomicBool,
    lost_motion: std::sync::atomic::AtomicBool,
    lost_samples: std::sync::atomic::AtomicU64,
    lost_device_ns: std::sync::atomic::AtomicU64,
    /// What this reader discarded while another reader published the hand (`discard`), for its
    /// first drain after taking the hand over, 0: none: the newest sample and matching device
    /// counter, the newest motion it decoded, and a time no lost sample was later than.
    unpublished_sample_ns: std::sync::atomic::AtomicU64,
    unpublished_device_ns: std::sync::atomic::AtomicU64,
    unpublished_motion_ns: std::sync::atomic::AtomicU64,
    // A range preserves possible earlier use when a later moving return replaces the newest
    // motion time. Intersect it conservatively with the inherited gap's own start window.
    unpublished_motion_from_ns: std::sync::atomic::AtomicU64,
    unpublished_lost_ns: std::sync::atomic::AtomicU64,
}

impl Default for RadioActivityQueue {
    fn default() -> Self {
        Self {
            slots: std::array::from_fn(|_| ActivitySlot::default()),
            head: Default::default(),
            tail: Default::default(),
            lost: Default::default(),
            lost_motion: Default::default(),
            lost_samples: Default::default(),
            lost_device_ns: Default::default(),
            unpublished_sample_ns: Default::default(),
            unpublished_device_ns: Default::default(),
            unpublished_motion_ns: Default::default(),
            unpublished_motion_from_ns: Default::default(),
            unpublished_lost_ns: Default::default(),
        }
    }
}

impl RadioActivityQueue {
    pub fn push(&self, activity: RadioActivity) {
        use std::sync::atomic::Ordering;
        let head = self.head.load(Ordering::Relaxed);
        if head.wrapping_sub(self.tail.load(Ordering::Acquire)) == ACTIVITY_SLOTS {
            if activity.moving {
                self.lost_motion.store(true, Ordering::Release);
            }
            if !activity.asleep {
                self.lost_device_ns
                    .store(activity.device_time_ns, Ordering::Relaxed);
                self.lost_samples
                    .store(activity.received_host_ns, Ordering::Release);
            }
            self.lost.store(true, Ordering::Release);
            return;
        }
        let slot = &self.slots[head % ACTIVITY_SLOTS];
        slot.device_ns
            .store(activity.device_time_ns, Ordering::Relaxed);
        slot.host_ns
            .store(activity.received_host_ns, Ordering::Relaxed);
        slot.asleep.store(activity.asleep, Ordering::Relaxed);
        slot.moving.store(activity.moving, Ordering::Relaxed);
        self.head.store(head.wrapping_add(1), Ordering::Release);
    }

    /// The consuming thread, while another reader publishes this hand: drops what is queued, so
    /// that a takeover finds only activity from after its last discard and no stale losses. A
    /// takeover keeps the newest sample and a range of motion times on this reader's clock.
    /// An omitted sample's time bounds the first return without borrowing another reader's clock.
    pub fn discard(&self) {
        use std::sync::atomic::Ordering;
        let mut tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.load(Ordering::Acquire);
        let (mut sample, mut sample_device, mut motion, mut lost) = (0, 0, 0, 0);
        let (mut motion_from, mut last_message) = (0, 0);
        while tail != head {
            let slot = &self.slots[tail % ACTIVITY_SLOTS];
            last_message = slot.host_ns.load(Ordering::Relaxed);
            if !slot.asleep.load(Ordering::Relaxed) {
                sample = slot.host_ns.load(Ordering::Relaxed);
                sample_device = slot.device_ns.load(Ordering::Relaxed);
                if slot.moving.load(Ordering::Relaxed) {
                    if motion_from == 0 {
                        motion_from = sample;
                    }
                    motion = sample;
                }
            }
            tail = tail.wrapping_add(1);
        }
        self.tail.store(tail, Ordering::Release);
        // Lost beacons cannot matter: a takeover ignores beacons from before it.
        self.lost.store(false, Ordering::Relaxed);
        let last_lost = self.lost_samples.swap(0, Ordering::AcqRel);
        if last_lost != 0 {
            lost = last_lost;
            if last_lost >= sample {
                sample = last_lost;
                sample_device = self.lost_device_ns.load(Ordering::Relaxed);
            }
        }
        // Motion's exact position in an overflow is unknown: its latest sample is a safe
        // upper bound. No timing or device counter here belongs to the other reader.
        if self.lost_motion.swap(false, Ordering::AcqRel) {
            motion = motion.max(last_lost);
            // Every omitted report followed the retained prefix on this same reader.
            motion_from = if motion_from == 0 {
                last_message
            } else {
                motion_from.min(last_message)
            };
        }
        if motion_from != 0 {
            let from = self.unpublished_motion_from_ns.load(Ordering::Relaxed);
            self.unpublished_motion_from_ns.store(
                if from == 0 {
                    motion_from
                } else {
                    from.min(motion_from)
                },
                Ordering::Relaxed,
            );
        }
        if sample != 0 && sample >= self.unpublished_sample_ns.load(Ordering::Relaxed) {
            self.unpublished_device_ns
                .store(sample_device, Ordering::Relaxed);
            self.unpublished_sample_ns.store(sample, Ordering::Relaxed);
        }
        self.unpublished_motion_ns
            .fetch_max(motion, Ordering::Relaxed);
        self.unpublished_lost_ns.fetch_max(lost, Ordering::Relaxed);
    }

    /// Forgets everything, when no reader runs: an engine session ends.
    pub fn reset(&self) {
        use std::sync::atomic::Ordering;
        self.tail
            .store(self.head.load(Ordering::Acquire), Ordering::Release);
        self.lost_samples.store(0, Ordering::Relaxed);
        self.lost_device_ns.store(0, Ordering::Relaxed);
        for flag in [&self.lost, &self.lost_motion] {
            flag.store(false, Ordering::Relaxed);
        }
        for time in [
            &self.unpublished_sample_ns,
            &self.unpublished_motion_ns,
            &self.unpublished_lost_ns,
            &self.unpublished_device_ns,
            &self.unpublished_motion_from_ns,
        ] {
            time.store(0, Ordering::Relaxed);
        }
    }

    /// Deliver each completed gap before a later drained report can replace its event. Missing
    /// activity vetoes sleep both before processing retained entries and after their last sample.
    pub fn drain_events(&self, hand: &mut Hand, mut event: impl FnMut(GapEvent)) {
        use std::sync::atomic::Ordering;
        let unpublished = [
            &self.unpublished_sample_ns,
            &self.unpublished_motion_ns,
            &self.unpublished_lost_ns,
            &self.unpublished_device_ns,
            &self.unpublished_motion_from_ns,
        ]
        .map(|time| time.swap(0, Ordering::Relaxed));
        hand.classification
            .unpublished(unpublished, hand.cfg, &mut event);
        let mut tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.load(Ordering::Acquire);
        let lost = self.lost.swap(false, Ordering::AcqRel);
        if lost {
            hand.lose_sleep_evidence();
        }
        while tail != head {
            let slot = &self.slots[tail % ACTIVITY_SLOTS];
            let host = slot.host_ns.load(Ordering::Relaxed);
            if slot.asleep.load(Ordering::Relaxed) {
                hand.note_sleep_beacon(host);
            } else {
                hand.note_sample_activity(
                    slot.device_ns.load(Ordering::Relaxed),
                    host,
                    slot.moving.load(Ordering::Relaxed),
                    false,
                );
                if let Some(e) = hand.take_event() {
                    event(e);
                }
                // Every actual decoded sample was live on its reader, even if publication waits.
                for e in hand
                    .classification
                    .tick(host, hand.cfg)
                    .into_iter()
                    .flatten()
                {
                    event(e);
                }
            }
            tail = tail.wrapping_add(1);
        }
        self.tail.store(tail, Ordering::Release);
        if lost || self.lost.swap(false, Ordering::AcqRel) {
            hand.lose_sleep_evidence();
        }
        let lost_sample = self.lost_samples.swap(0, Ordering::AcqRel);
        if self.lost_motion.swap(false, Ordering::AcqRel) {
            hand.classification.note_motion(lost_sample, hand.cfg);
        }
        if lost_sample != 0 {
            if let Some(e) = hand.classification.unknown_return(
                self.lost_device_ns.load(Ordering::Relaxed),
                lost_sample,
                hand.cfg,
            ) {
                event(e);
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct HandConfig {
    /// No new IMU sample for this long means the controller's samples stopped.
    pub connect_timeout_ns: u64,
    /// A controller whose samples stop stays connected this long after its last sample, as it
    /// was then, inputs held (a radio gap); past it, it is in standby or out of range.
    pub ride_through_ns: u64,
    /// Position stays valid this long after the last optical fix (engine POSE_LOST_THRESHOLD).
    pub position_hold_ns: u64,
    /// Capacitive hysteresis thresholds on the normalized values.
    pub touch_on: f32,
    pub touch_off: f32,
}

impl Default for HandConfig {
    fn default() -> Self {
        Self {
            connect_timeout_ns: 100_000_000,
            ride_through_ns: 10_000_000_000,
            position_hold_ns: 500_000_000,
            touch_on: 0.6,
            touch_off: 0.4,
        }
    }
}

/// A controller that turned faster than this within `MOTION_WINDOW_NS` of its last sample was in
/// use when its samples stopped, rad/s: a controller put down has been still for longer.
const MOTION_RAD_S: f32 = 0.5;
/// Whether one calibrated gyro sample establishes recent use (squared rad/s).
pub fn in_use_motion(gyro_squared: f32) -> bool {
    gyro_squared > MOTION_RAD_S * MOTION_RAD_S
}

const MOTION_WINDOW_NS: u64 = 3_000_000_000;
/// First recorded beacons in four sleep gaps arrived 2-952 ms after the last sample. Leave
/// about 300 ms beyond that before logging a still gap as radio silence.
pub const SLEEP_BEACON_WAIT_NS: u64 = 1_250_000_000;

/// Why a controller published live is held through a gap. A gap keeps the cause it began with.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum GapCause {
    /// Its samples stopped arriving: a radio gap.
    #[default]
    Radio,
    /// The tracking engine is closing or opening and has no sample to give.
    Reopen,
    /// The thread publishing it published nothing for longer than the connect timeout.
    Stall,
}

/// A change in a controller's radio gap (see `HandConfig::ride_through_ns`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GapEvent {
    /// Its samples stopped: no sample since `last_sample_host_ns`. `moving`: it turned within a
    /// few seconds before.
    Began {
        last_sample_host_ns: u64,
        moving: bool,
        asleep: bool,
        cause: GapCause,
    },
    /// A beacon arrived after a still gap began: it is sleep, not a radio dropout.
    Asleep,
    /// An open gap acquired use evidence: capture it with its original anchor.
    InUse {
        last_sample_host_ns: u64,
        cause: GapCause,
    },
    /// A still gap has received no sleep beacon for the confirmation window.
    StillSilent,
    /// Its samples came back after `silent_ns` without, the last before at `last_sample_host_ns`.
    Ended {
        last_sample_host_ns: u64,
        silent_ns: u64,
        asleep: bool,
        /// Missing samples or an ownership change left the exact return boundary uncertain.
        uncertain: bool,
        /// Complete recording decision, including blips, standby and missing return boundaries.
        recordable: bool,
        /// This identity submitted its capture: accepted/covered, retained for retry, or saved.
        capture_requested: bool,
        cause: GapCause,
    },
    /// No sample for the whole ride-through: published as in standby from now.
    Expired {
        silent_ns: u64,
        asleep: bool,
        cause: GapCause,
    },
}

/// The inputs a controller last reported while its samples arrived, held through a radio gap.
#[derive(Debug, Clone, Copy, Default)]
struct HeldInputs {
    trigger: f32,
    grip: f32,
    stick: [f32; 2],
    cap: [f32; 5],
    buttons: u32,
    touches: u32,
}

/// Both acquisition threads use the same host-monotonic clock. Absolute sample and motion
/// times survive publication delays; reader-local beacon evidence never crosses ownership.
#[derive(Clone, Copy, Debug, Default)]
pub struct ClassificationHistory {
    pub gap_in_use: bool,
    pub motion_host_ns: u64,
    pub gap: Option<GapHistory>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct GapHistory {
    pub recording_anchor_ns: u64,
    pub cause: GapCause,
    pub confirmed_sleep_before_ns: u64,
    pub open: bool,
    pub expired: bool,
    pub silence_logged: bool,
    pub spans_handoff: bool,
    pub capture_requested: bool,
}

/// Classification invariants, independent of pose/input holding:
/// * silence and motion use absolute host-monotonic times, never a publication-time duration;
/// * local motion counts relative to the gap start, never the handoff time;
/// * imported use is only a bool: it vetoes sleep for 3 s from receiver-local takeover, even
///   across still samples. This over-approximates source use; after that only local use counts;
/// * only one reader that saw a gap begin can prove sleep from its beacons before first return;
///   an ordinary handoff cannot prove a nap, even if ownership returns to the original reader;
/// * only Radio gaps can request capture. Reopen/Stall hold until a fresh accepted publication;
///   raw decoded returns during teardown cannot end those holds;
/// * native session reset preserves a Radio identity, anchor, capture lifecycle and an already
///   settled nap verdict, but drops reader-local evidence. New reader beacons prove nothing
///   about that older interval; contradictory returns or activity loss veto the settled nap;
/// * in-use Radio gaps request capture at their start, or explicitly when promoted to in-use;
///   capture-requested belongs to that identity, independent of expiry ticks or saving.
///   The daemon retains rejected recorder requests across returns until accepted or covered;
/// * only a sample received strictly after takeover clears the handoff-spanning identity;
///   a pre-takeover return establishes another handoff-spanning interval;
/// * a still handoff gap records a return at >=0.5 s, or an unknown/lost first return;
/// * other still gaps without sleep proof record a return at >=0.5 s and before 10 s;
/// * a still gap's return, known or lost, is standby from `flight::LATEST_NS` after its last
///   sample, when the flight ring may no longer hold its window;
/// * every observed return ends a Radio gap, including discarded samples and relevant loss bounds;
/// * a reader's own copy of an inherited anchor's report is not a return: in an opened interval, a
///   sample less than the connect timeout after another reader's anchor never ends it;
/// * known blips cancel their own request; ambiguity may add a capture, never hide a dropout.
#[derive(Default)]
struct GapTracker {
    cause: GapCause,
    device_ns: u64,
    anchor_ns: u64,
    takeover_host_ns: u64,
    // These two times belong to THIS reader, and survive it losing and regaining ownership.
    local_sample_ns: u64,
    local_motion_ns: u64,
    // A plain imported use bool buys a full local motion window at takeover. A still sample
    // cannot clear it; after it expires, only this reader's own evidence can establish use.
    imported_use_until_ns: u64,
    live: bool,
    open: bool,
    in_use: bool,
    promoted: bool,
    spans_handoff: bool,
    first_beacon_ns: u64,
    confirmed_sleep_before_ns: u64,
    last_beacon_ns: u64,
    uncertain: bool,
    asleep: bool,
    expired: bool,
    silence_logged: bool,
    capture_requested: bool,
}

impl GapTracker {
    fn silent_at(&self, now: u64) -> u64 {
        now.saturating_sub(self.anchor_ns)
    }

    fn start(&self) -> Option<u64> {
        (self.anchor_ns != 0).then_some(self.anchor_ns)
    }

    fn moving(&self) -> bool {
        self.start().map_or(
            self.local_motion_ns != 0 || self.imported_use_until_ns != 0,
            |start| {
                (self.imported_use_until_ns != 0 && start <= self.imported_use_until_ns)
                    || (self.local_motion_ns != 0
                        && self.local_motion_ns <= start
                        && start - self.local_motion_ns <= MOTION_WINDOW_NS)
            },
        )
    }

    fn mark_in_use(&mut self) {
        if !self.in_use {
            self.in_use = true;
            self.asleep = false;
            self.confirmed_sleep_before_ns = 0;
            self.promoted = self.open;
        }
    }

    fn promote(&mut self) {
        if self.moving() {
            self.mark_in_use();
        }
    }

    fn promotion(&mut self) -> Option<GapEvent> {
        if std::mem::take(&mut self.promoted)
            && self.cause == GapCause::Radio
            && !self.capture_requested
        {
            self.capture_requested = true;
            Some(GapEvent::InUse {
                last_sample_host_ns: self.anchor_ns,
                cause: self.cause,
            })
        } else {
            None
        }
    }

    fn sleep_before(&self, end: u64) -> bool {
        self.cause == GapCause::Radio
            && !self.in_use
            && ((self.confirmed_sleep_before_ns != 0 && self.confirmed_sleep_before_ns < end)
                || (!self.spans_handoff
                    && !self.uncertain
                    && self.first_beacon_ns != 0
                    && self.first_beacon_ns < end))
    }

    /// Both readers read every report and stamp it at their own read, up to their read
    /// latencies apart, so a handoff-spanning anchor can be another reader's stamp of a report
    /// this reader also has. Its copy is never a return: not one stamped at the anchor, and not
    /// one stamped within the connect timeout after it once the interval has opened. A gap opens
    /// only after that much silence, so an actual return comes later. (A nap settled for a reopen
    /// has opened: teardown opens the gap first.)
    fn copy_of_anchor(&self, host: u64, cfg: HandConfig) -> bool {
        self.spans_handoff
            && self.start().is_some_and(|start| {
                host == start
                    || (self.open && host > start && host - start < cfg.connect_timeout_ns)
            })
    }

    /// Motion this reader decoded at `host`. On a copy of the inherited anchor's report, or of
    /// an earlier one, it was motion before the gap, however late this reader stamped it.
    fn note_motion(&mut self, host: u64, cfg: HandConfig) {
        let at = if self.copy_of_anchor(host, cfg) {
            self.anchor_ns
        } else {
            host
        };
        self.local_motion_ns = self.local_motion_ns.max(at);
    }

    fn sample(
        &mut self,
        device: u64,
        host: u64,
        moving: bool,
        cfg: HandConfig,
        published: bool,
    ) -> Option<GapEvent> {
        let native_return = self.open && self.cause != GapCause::Radio;
        if native_return && !published {
            // Only a fresh, accepted publication can end an engine/stall hold. Raw reports
            // may arrive during teardown or while tracker work is still blocked.
            self.local_sample_ns = self.local_sample_ns.max(host);
            if moving {
                self.local_motion_ns = self.local_motion_ns.max(host);
                self.mark_in_use();
            }
            return self.promotion();
        }
        if self.start().is_some_and(|start| host < start)
            || (!native_return && self.copy_of_anchor(host, cfg))
        {
            if moving {
                self.note_motion(host, cfg);
                self.promote();
            }
            self.device_ns = device;
            self.local_sample_ns = self.local_sample_ns.max(host);
            return self.promotion();
        }
        if device == self.device_ns && !(native_return && host > self.anchor_ns) {
            return None;
        }
        let elapsed = self.silent_at(host);
        let event = if self.live && (self.open || elapsed >= cfg.connect_timeout_ns) {
            if !self.open {
                self.in_use |= self.moving();
            }
            let asleep = self.sleep_before(host);
            // A handoff can hide an earlier first return, so a spanning gap's return counts
            // after the hold too, but only while the flight ring can still hold its window.
            let spanning = self.spans_handoff && elapsed < crate::flight::LATEST_NS;
            let within_hold = self.in_use || spanning || elapsed < cfg.ride_through_ns;
            let recordable = self.cause == GapCause::Radio
                && !asleep
                && elapsed >= crate::flight::SHORTEST_NS
                && within_hold;
            // Preserve the existing wake/standby event policy. Capture-requested below is
            // independent of this timer: saving can finish before any expiry tick runs.
            if asleep
                || (within_hold
                    && (!self.expired
                        || (self.cause == GapCause::Radio && spanning && !self.in_use)))
            {
                Some(GapEvent::Ended {
                    last_sample_host_ns: self.anchor_ns,
                    silent_ns: elapsed,
                    asleep,
                    uncertain: self.spans_handoff,
                    recordable,
                    capture_requested: self.capture_requested,
                    cause: self.cause,
                })
            } else {
                None
            }
        } else {
            None
        };
        if moving {
            self.local_motion_ns = self.local_motion_ns.max(host);
        }
        self.advance(host);
        self.device_ns = device;
        self.local_sample_ns = self.local_sample_ns.max(host);
        event
    }

    /// The latest decoded sample establishes the next local interval. Classification never
    /// advances the separate pose/input sample clock.
    fn advance(&mut self, host: u64) {
        self.anchor_ns = host;
        self.cause = GapCause::Radio;
        self.confirmed_sleep_before_ns = 0;
        self.live = false;
        self.open = false;
        // Snapshot use at the last advancing sample; a later moving return cannot erase it.
        self.in_use = self.moving();
        self.promoted = false;
        // A return received before (or at) takeover ends its inherited interval, but cannot
        // establish uninterrupted ownership of the successor. Only a later receive can.
        self.spans_handoff = self.takeover_host_ns != 0 && host <= self.takeover_host_ns;
        self.first_beacon_ns = if self.last_beacon_ns > host {
            self.last_beacon_ns
        } else {
            0
        };
        self.last_beacon_ns = self.first_beacon_ns;
        self.uncertain = self.spans_handoff;
        self.asleep = false;
        self.expired = false;
        self.silence_logged = false;
        self.capture_requested = false;
    }

    // The first return was lost or discarded; record it now, unless the flight ring may no
    // longer hold its window: a still gap is then standby. Its latest actual sample and matching
    // local device counter establish the next interval without replaying the snapshot.
    fn unknown_return(&mut self, device: u64, host: u64, cfg: HandConfig) -> Option<GapEvent> {
        if self.open && self.cause != GapCause::Radio {
            self.local_sample_ns = self.local_sample_ns.max(host);
            if self.local_motion_ns != 0
                && self.local_motion_ns <= host
                && host - self.local_motion_ns <= MOTION_WINDOW_NS
            {
                self.mark_in_use();
            }
            return self.promotion();
        }
        if !self.live
            || self.start().is_some_and(|start| host <= start)
            || self.copy_of_anchor(host, cfg)
        {
            return None;
        }
        let elapsed = self.silent_at(host);
        let event = ((self.open || elapsed >= cfg.connect_timeout_ns)
            && !(self.in_use && self.expired)
            && (self.in_use || elapsed < crate::flight::LATEST_NS))
            .then_some(GapEvent::Ended {
                last_sample_host_ns: self.anchor_ns,
                silent_ns: elapsed,
                asleep: false,
                uncertain: true,
                recordable: true,
                capture_requested: self.capture_requested,
                cause: self.cause,
            });
        self.advance(host);
        self.live = true;
        self.device_ns = device;
        self.local_sample_ns = self.local_sample_ns.max(host);
        event
    }

    /// Discarded reports still end an inherited gap. Their first return may be unknown, and a
    /// loss bound may include it before the handoff. Neither needs a later retained sample.
    fn unpublished(
        &mut self,
        [sample, motion, lost, device, motion_from]: [u64; 5],
        cfg: HandConfig,
        mut event: impl FnMut(GapEvent),
    ) {
        // Preserve already-known local use before the newest discarded motion replaces it.
        self.promote();
        if self.spans_handoff
            && motion_from != 0
            && self.start().is_none_or(|start| {
                (motion_from <= start || self.copy_of_anchor(motion_from, cfg))
                    && motion >= start.saturating_sub(MOTION_WINDOW_NS)
            })
        {
            // Only a range is retained. Its intersection may add use, never exclude earlier
            // motion because a later moving return overwrote the most recent timestamp.
            self.mark_in_use();
        }
        self.note_motion(motion, cfg);
        self.promote();
        if let Some(e) = self.promotion() {
            event(e);
        }
        if !self.spans_handoff {
            return;
        }
        let device = if sample >= self.local_sample_ns {
            device
        } else {
            self.device_ns
        };
        let sample = sample.max(self.local_sample_ns);
        if sample != 0 && self.start().is_none_or(|start| sample > start) {
            if let Some(e) = self.unknown_return(device, sample, cfg) {
                event(e);
            }
            self.local_sample_ns = sample;
        }
        if lost != 0 && self.start().is_none_or(|start| lost > start) {
            if let Some(e) = self.unknown_return(device, lost, cfg) {
                event(e);
            }
        }
    }

    fn beacon(&mut self, host: u64) {
        if self.anchor_ns != 0 && self.silent_at(host) > 0 && !self.uncertain && !self.spans_handoff
        {
            if self.first_beacon_ns == 0 {
                self.first_beacon_ns = host;
            }
            self.last_beacon_ns = self.last_beacon_ns.max(host);
        }
    }

    fn lose_evidence(&mut self) {
        self.confirmed_sleep_before_ns = 0;
        self.first_beacon_ns = 0;
        self.last_beacon_ns = 0;
        self.uncertain = true;
        self.asleep = false;
    }

    fn tick(&mut self, now: u64, cfg: HandConfig) -> [Option<GapEvent>; 2] {
        self.tick_missing(now, cfg, None)
    }

    fn tick_missing(
        &mut self,
        now: u64,
        cfg: HandConfig,
        missing: Option<GapCause>,
    ) -> [Option<GapEvent>; 2] {
        if self.anchor_ns == 0 {
            return [None, None];
        }
        let silent = self.silent_at(now);
        if missing.is_none() && silent < cfg.connect_timeout_ns {
            self.live = true;
            return [self.promotion(), None];
        }
        if !self.live {
            return [None, None];
        }
        let first = if self.promoted {
            self.promotion()
        } else if !self.open && !self.expired {
            self.open = true;
            self.cause = missing.unwrap_or(GapCause::Radio);
            self.in_use |= self.moving();
            self.asleep = self.sleep_before(u64::MAX);
            self.capture_requested |= self.in_use && self.cause == GapCause::Radio;
            Some(GapEvent::Began {
                last_sample_host_ns: self.anchor_ns,
                moving: self.in_use,
                asleep: self.asleep,
                cause: self.cause,
            })
        } else if self.open && !self.asleep && self.sleep_before(u64::MAX) {
            self.asleep = true;
            Some(GapEvent::Asleep)
        } else {
            None
        };
        let second = if self.open && !self.expired && silent >= cfg.ride_through_ns {
            self.expired = true;
            Some(GapEvent::Expired {
                silent_ns: silent,
                asleep: self.asleep,
                cause: self.cause,
            })
        } else if self.open
            && self.cause == GapCause::Radio
            && !self.expired
            && !self.in_use
            && !self.asleep
            && !self.silence_logged
            && silent >= SLEEP_BEACON_WAIT_NS
        {
            self.silence_logged = true;
            Some(GapEvent::StillSilent)
        } else {
            None
        };
        [first, second]
    }

    fn history(&self) -> ClassificationHistory {
        ClassificationHistory {
            gap_in_use: self.in_use || self.moving(),
            motion_host_ns: self.local_motion_ns,
            gap: (self.live || self.open).then_some(GapHistory {
                recording_anchor_ns: self.anchor_ns,
                cause: self.cause,
                confirmed_sleep_before_ns: self.confirmed_sleep_before_ns,
                open: self.open,
                expired: self.expired,
                silence_logged: self.silence_logged,
                spans_handoff: self.spans_handoff,
                capture_requested: self.capture_requested,
            }),
        }
    }

    fn inherit(&mut self, history: ClassificationHistory, takeover_host_ns: u64) {
        let (local_sample_ns, local_motion_ns, device_ns) =
            (self.local_sample_ns, self.local_motion_ns, self.device_ns);
        let imported_use_until_ns = self.imported_use_until_ns.max(if history.gap_in_use {
            takeover_host_ns.saturating_add(MOTION_WINDOW_NS)
        } else {
            0
        });
        *self = Self {
            device_ns,
            local_sample_ns,
            local_motion_ns,
            imported_use_until_ns,
            takeover_host_ns,
            ..Self::default()
        };
        if let Some(gap) = history.gap {
            self.anchor_ns = gap.recording_anchor_ns;
            self.cause = gap.cause;
            self.confirmed_sleep_before_ns = gap.confirmed_sleep_before_ns;
            self.live = true;
            self.open = gap.open;
            self.in_use = history.gap_in_use
                || (history.motion_host_ns != 0
                    && history.motion_host_ns <= self.anchor_ns
                    && self.anchor_ns - history.motion_host_ns <= MOTION_WINDOW_NS);
            self.asleep = self.confirmed_sleep_before_ns != 0 && !self.in_use;
            self.capture_requested = gap.capture_requested;
            self.promoted = self.open && self.in_use && !self.capture_requested;
            self.expired = gap.expired;
            self.silence_logged = gap.silence_logged;
            self.spans_handoff = true;
            self.uncertain = true;
            self.promote();
        }
    }
}

pub struct Hand {
    hand: usize,
    cfg: HandConfig,
    clock: ClockMap,
    last_device_ns: u64,
    last_advance_host_ns: u64,
    touches: u32,
    held: HeldInputs,
    /// Published live since its samples last stopped for good: what a radio gap holds.
    seen_live: bool,
    classification: GapTracker,
    classification_reader: Option<usize>,
    parked: [GapTracker; 2],
    event: Option<GapEvent>,
}

fn hysteresis(cur: u32, bit: u32, value: f32, on: f32, off: f32) -> u32 {
    if value >= on {
        cur | bit
    } else if value <= off {
        cur & !bit
    } else {
        cur
    }
}

impl Hand {
    pub fn new(hand: usize, cfg: HandConfig) -> Self {
        assert!(hand == HAND_LEFT || hand == HAND_RIGHT, "hand index");
        Self {
            hand,
            cfg,
            clock: ClockMap::default(),
            last_device_ns: 0,
            last_advance_host_ns: 0,
            touches: 0,
            held: HeldInputs::default(),
            seen_live: false,
            classification: GapTracker::default(),
            classification_reader: None,
            parked: Default::default(),
            event: None,
        }
    }

    /// The radio-gap change the last `update` found, if any (taken once).
    pub fn take_event(&mut self) -> Option<GapEvent> {
        self.event.take()
    }

    pub fn classification_reader(&self) -> Option<usize> {
        self.classification_reader
    }

    /// Seal an already-proven nap before discarding its reader. New reader beacons cannot
    /// establish this verdict. A contradictory old return or any lost activity vetoes it.
    pub fn preserve_reopen_sleep(&mut self) {
        let g = &mut self.classification;
        if g.sleep_before(u64::MAX) && g.confirmed_sleep_before_ns == 0 {
            g.confirmed_sleep_before_ns = g.first_beacon_ns;
            g.asleep = true;
        }
    }

    /// Reset acquisition-local evidence after both readers stop. A Radio identity, its capture
    /// request and a settled sleep verdict survive; the new reader starts its own use window.
    pub fn reset_reader_evidence(&mut self) {
        self.preserve_reopen_sleep();
        let g = &mut self.classification;
        g.device_ns = 0;
        g.local_sample_ns = 0;
        g.local_motion_ns = 0;
        g.imported_use_until_ns = 0;
        g.takeover_host_ns = 0;
        g.first_beacon_ns = 0;
        g.last_beacon_ns = 0;
        g.promoted = false;
        g.spans_handoff = true;
        g.uncertain = true;
        self.classification_reader = None;
        self.parked = Default::default();
        self.event = None;
    }

    /// Select only this reader's classification context; held poses and inputs never move.
    pub fn select_classification_reader(&mut self, reader: usize) {
        assert!(reader < 2);
        if self.classification_reader == Some(reader) {
            return;
        }
        if let Some(old) = self.classification_reader {
            std::mem::swap(&mut self.classification, &mut self.parked[old]);
            std::mem::swap(&mut self.classification, &mut self.parked[reader]);
        }
        self.classification_reader = Some(reader);
        self.event = None;
    }

    pub fn classification_history(&self) -> ClassificationHistory {
        self.classification.history()
    }

    /// Both readers share the process's host-monotonic clock. Takeover uses a fresh timestamp,
    /// never the caller's earlier publication tick. Sleep evidence remains reader-local.
    pub fn inherit_classification_at(
        &mut self,
        history: ClassificationHistory,
        takeover_host_ns: u64,
    ) {
        self.classification.inherit(history, takeover_host_ns);
        self.event = None;
    }

    pub fn note_sleep_beacon(&mut self, host_ns: u64) {
        self.classification.beacon(host_ns);
    }

    fn note_sample_activity(
        &mut self,
        device_ns: u64,
        host_ns: u64,
        moving: bool,
        published: bool,
    ) {
        if let Some(event) = self
            .classification
            .sample(device_ns, host_ns, moving, self.cfg, published)
        {
            self.event = Some(event);
        }
    }

    fn lose_sleep_evidence(&mut self) {
        self.classification.lose_evidence();
    }

    /// Classification alone, also used before releasing ownership with a drained engine batch.
    /// It does not update poses, touches or held inputs.
    pub fn classify(&mut self, c: &ffi::ohmd_cv1touch_controller, now_ns: u64) {
        let mut latest = self.event.take();
        self.classify_events(c, now_ns, |event| latest = Some(event));
        self.event = latest;
    }

    /// Deliver a returning sample's end before a delayed publication can begin another gap.
    pub fn classify_events(
        &mut self,
        c: &ffi::ohmd_cv1touch_controller,
        now_ns: u64,
        event: impl FnMut(GapEvent),
    ) {
        let fresh = c.received_host_ns > self.last_advance_host_ns
            || c.device_time_ns != self.last_device_ns;
        self.classify_snapshot_events(c, now_ns, fresh, event);
    }

    /// Pre-handoff snapshots carry raw activity, never permission to end a native hold. Only
    /// the daemon's accepted-publication path calls `classify_events` with that permission.
    pub fn classify_unpublished_events(
        &mut self,
        c: &ffi::ohmd_cv1touch_controller,
        now_ns: u64,
        event: impl FnMut(GapEvent),
    ) {
        self.classify_snapshot_events(c, now_ns, false, event);
    }

    fn classify_snapshot_events(
        &mut self,
        c: &ffi::ohmd_cv1touch_controller,
        now_ns: u64,
        published: bool,
        mut event: impl FnMut(GapEvent),
    ) {
        if !c.calibrated {
            return;
        }
        self.classification
            .unpublished([0; 5], self.cfg, &mut event);
        let g = c.imu_gyro;
        self.note_sample_activity(
            c.device_time_ns,
            c.received_host_ns,
            in_use_motion(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]),
            published,
        );
        if let Some(e) = self.take_event() {
            event(e);
        }
        self.tick_events(now_ns, event);
    }

    /// Classify a missing publication without changing the pose/input deadline.
    pub fn unavailable_events(
        &mut self,
        now_ns: u64,
        cause: GapCause,
        mut event: impl FnMut(GapEvent),
    ) {
        self.classification
            .unpublished([0; 5], self.cfg, &mut event);
        if cause != GapCause::Radio && !self.classification.open && self.last_advance_host_ns != 0 {
            self.classification.anchor_ns = self.last_advance_host_ns;
        }
        for e in self
            .classification
            .tick_missing(now_ns, self.cfg, Some(cause))
            .into_iter()
            .flatten()
        {
            event(e);
        }
    }

    /// Classification alone for a publisher without a sample of its own yet: a gap it took over
    /// still begins, logs and expires on time.
    pub fn tick_events(&mut self, now_ns: u64, mut event: impl FnMut(GapEvent)) {
        self.classification
            .unpublished([0; 5], self.cfg, &mut event);
        for e in self
            .classification
            .tick(now_ns, self.cfg)
            .into_iter()
            .flatten()
        {
            event(e);
        }
    }

    fn map_buttons(&self, b: u8) -> u32 {
        let right = self.hand == HAND_RIGHT;
        let mut out = 0;
        if b & 0x1 != 0 {
            out |= if right { BTN_A } else { BTN_X };
        }
        if b & 0x2 != 0 {
            out |= if right { BTN_B } else { BTN_Y };
        }
        if b & 0x4 != 0 {
            out |= BTN_SYSTEM;
        }
        if b & 0x8 != 0 {
            out |= BTN_STICK;
        }
        out
    }

    fn update_touches(&mut self, cap: &[f32; 5]) {
        let right = self.hand == HAND_RIGHT;
        let (on, off) = (self.cfg.touch_on, self.cfg.touch_off);
        let mut t = self.touches;
        t = hysteresis(
            t,
            if right { TOUCH_A } else { TOUCH_X },
            cap[CAP_AX],
            on,
            off,
        );
        t = hysteresis(
            t,
            if right { TOUCH_B } else { TOUCH_Y },
            cap[CAP_BY],
            on,
            off,
        );
        t = hysteresis(t, TOUCH_TRIGGER, cap[CAP_TRIGGER], on, off);
        t = hysteresis(t, TOUCH_STICK, cap[CAP_STICK], on, off);
        t = hysteresis(t, TOUCH_THUMBREST, cap[CAP_THUMBREST], on, off);
        self.touches = t;
    }

    pub fn update(&mut self, c: &ffi::ohmd_cv1touch_controller, now_ns: u64) -> ControllerState {
        if !c.calibrated {
            // a new engine has no calibrated controller until it reads the controller's setup
            return self.unavailable(now_ns, GapCause::Reopen);
        }
        if c.received_host_ns != 0 {
            self.classify(c, now_ns);
        }
        if c.received_host_ns < self.last_advance_host_ns {
            if c.received_host_ns == 0 {
                // a new engine's controller before its first sample
                return self.unavailable(now_ns, GapCause::Reopen);
            }
            // The engine and the radio thread stamp their own reads of the same reports, so the
            // first samples after a hand passes between them can be stamped before the newest
            // one published: the controller still reporting. Its inputs count; its times, on the
            // other reader's clocks, do not, so it does not advance.
            let reread = ffi::ohmd_cv1touch_controller {
                device_time_ns: self.last_device_ns,
                received_host_ns: self.last_advance_host_ns,
                last_pose_fix_device_ns: 0,
                ..*c
            };
            return self.update_state(&reread, now_ns, None);
        }
        self.update_state(c, now_ns, None)
    }

    /// There is no sample to offer, for `cause` (a gap this starts keeps it). Keep the same gap
    /// and its deadline, even when the last sample was less than 100 ms ago.
    pub fn unavailable(&mut self, now_ns: u64, cause: GapCause) -> ControllerState {
        let mut latest = self.event.take();
        self.unavailable_events(now_ns, cause, |event| latest = Some(event));
        self.event = latest;
        let c = ffi::ohmd_cv1touch_controller {
            calibrated: self.last_advance_host_ns != 0,
            device_time_ns: self.last_device_ns,
            received_host_ns: self.last_advance_host_ns,
            ..Default::default()
        };
        self.update_state(&c, now_ns, Some(cause))
    }

    /// `missing`: no sample, and why (see `unavailable`).
    fn update_state(
        &mut self,
        c: &ffi::ohmd_cv1touch_controller,
        now_ns: u64,
        missing: Option<GapCause>,
    ) -> ControllerState {
        let mut out = ControllerState {
            publish_host_ns: now_ns,
            battery_percent: -1,
            ..Default::default()
        };
        if !c.calibrated {
            return out;
        }
        out.flags |= FLAG_CALIBRATED;

        if c.device_time_ns != self.last_device_ns || c.received_host_ns > self.last_advance_host_ns
        {
            self.last_device_ns = c.device_time_ns;
            self.last_advance_host_ns = c.received_host_ns;
            self.clock.observe(c.device_time_ns, c.received_host_ns);
        }
        let silent_ns = now_ns.saturating_sub(self.last_advance_host_ns);
        let connected = missing.is_none()
            && self.last_advance_host_ns != 0
            && silent_ns < self.cfg.connect_timeout_ns;
        // Pose and inputs retain exactly the existing 10 s ride-through, regardless of classification.
        self.seen_live |= connected;
        let gap = self.seen_live && !connected && silent_ns < self.cfg.ride_through_ns;
        if !gap && !connected {
            self.seen_live = false;
        }
        if self.classification.asleep {
            out.flags |= FLAG_ASLEEP;
        }
        if gap {
            out.flags |= FLAG_CONNECTED | FLAG_RADIO_GAP;
        } else if connected {
            out.flags |= FLAG_CONNECTED;
            if c.last_pose_fix_device_ns != 0
                && c.device_time_ns.saturating_sub(c.last_pose_fix_device_ns)
                    < self.cfg.position_hold_ns
            {
                out.flags |= FLAG_POSITION;
            }
        }

        out.device_time_ns = c.device_time_ns;
        out.received_host_ns = c.received_host_ns;
        out.sample_host_ns = self.clock.to_host(c.device_time_ns).unwrap_or(0);
        out.last_position_fix_host_ns = self.clock.to_host(c.last_pose_fix_device_ns).unwrap_or(0);
        out.orient = c.view_orient;
        out.pos = c.view_pos;
        out.raw_orient = c.raw_orient;
        out.raw_pos = c.raw_pos;
        out.lin_vel = c.view_lin_vel;
        out.ang_vel = c.view_ang_vel;
        out.lin_acc = c.view_lin_acc;
        out.gyro_raw = c.gyro_raw;
        out.gyro_bias = c.gyro_bias;
        out.imu_accel = c.imu_accel;
        out.imu_gyro = c.imu_gyro;
        out.pos_stddev = c.pos_stddev;
        out.rot_stddev = c.rot_stddev;
        if missing.is_none() {
            self.update_touches(&c.cap);
        }
        if connected {
            self.held = HeldInputs {
                trigger: c.trigger,
                grip: c.grip,
                stick: c.stick,
                cap: c.cap,
                buttons: self.map_buttons(c.buttons),
                touches: self.touches,
            };
        }
        // Through a radio gap the inputs stay as the controller last reported them, whatever the
        // engine keeps; in standby they are released (the engine keeps the last report).
        if connected || gap {
            let h = self.held;
            out.trigger = h.trigger;
            out.grip = h.grip;
            out.stick = h.stick;
            out.cap = h.cap;
            out.buttons = h.buttons;
            out.touches = h.touches;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input() -> ffi::ohmd_cv1touch_controller {
        ffi::ohmd_cv1touch_controller {
            calibrated: true,
            ..Default::default()
        }
    }

    #[test]
    fn a_still_gap_waits_for_a_beacon_before_logging_radio_silence() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        let host = 5_000_000_000;
        live_sample(&mut h, &mut c, host);
        h.update(&c, host + 150_000_000);
        assert!(matches!(
            h.take_event(),
            Some(GapEvent::Began {
                moving: false,
                asleep: false,
                ..
            })
        ));
        h.update(&c, host + 1_249_999_999);
        assert_eq!(h.take_event(), None);
        let held = h.update(&c, host + 1_250_000_000);
        assert_eq!(h.take_event(), Some(GapEvent::StillSilent));
        assert_ne!(held.flags & FLAG_RADIO_GAP, 0);
        h.update(&c, host + 1_350_000_000);
        assert_eq!(h.take_event(), None);
        h.note_sleep_beacon(host + 1_450_000_000);
        h.update(&c, host + 1_450_000_000);
        assert_eq!(h.take_event(), Some(GapEvent::Asleep));
    }

    #[test]
    fn early_and_late_sleep_beacons_avoid_the_radio_silent_log() {
        for delay in [2_000_000, 750_000_000, 953_000_000] {
            let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
            let mut c = input();
            let host = 5_000_000_000;
            live_sample(&mut h, &mut c, host);
            if delay < 150_000_000 {
                h.note_sleep_beacon(host + delay);
            }
            h.update(&c, host + 150_000_000);
            assert_eq!(
                h.take_event(),
                Some(GapEvent::Began {
                    last_sample_host_ns: host,
                    moving: false,
                    asleep: delay < 150_000_000,
                    cause: GapCause::Radio,
                })
            );
            if delay >= 150_000_000 {
                h.note_sleep_beacon(host + delay);
                h.update(&c, host + delay);
                assert_eq!(h.take_event(), Some(GapEvent::Asleep));
            }
            h.update(&c, host + SLEEP_BEACON_WAIT_NS);
            assert_eq!(h.take_event(), None);
        }
    }

    #[test]
    fn sleep_evidence_must_precede_the_sample_that_ends_the_gap() {
        for (beacon_ns, earlier_beacon_ns, published_before_wake, asleep) in [
            (750_000_000, None, false, true),
            (800_000_000, None, false, false),
            (802_000_000, None, false, false),
            (802_000_000, None, true, false),
            (802_000_000, Some(750_000_000), false, true),
        ] {
            let mut h = Hand::new(HAND_LEFT, HandConfig::default());
            let mut c = input();
            let host = 5_000_000_000;
            live_sample(&mut h, &mut c, host);
            h.update(&c, host + 150_000_000);
            h.take_event();
            if let Some(earlier) = earlier_beacon_ns {
                h.note_sleep_beacon(host + earlier);
                h.update(&c, host + earlier);
                h.take_event();
            }
            h.note_sleep_beacon(host + beacon_ns);
            if published_before_wake {
                h.update(&c, host + beacon_ns);
                h.take_event();
            }
            live_sample(&mut h, &mut c, host + 800_000_000);
            assert_eq!(
                h.take_event(),
                Some(GapEvent::Ended {
                    last_sample_host_ns: host,
                    silent_ns: 800_000_000,
                    asleep,
                    uncertain: false,
                    recordable: !asleep,
                    capture_requested: false,
                    cause: GapCause::Radio,
                }),
                "beacon {beacon_ns}, published before wake {published_before_wake}"
            );
            h.update(&c, host + 950_000_000);
            assert!(matches!(
                h.take_event(),
                Some(GapEvent::Began { asleep, .. }) if asleep == (beacon_ns > 800_000_000)
            ));
        }
    }

    #[test]
    fn lost_first_returns_do_not_turn_a_pre_expiry_dropout_into_standby() {
        for (duration, recordable) in [
            (200_000_000, true),
            (9_998_000_000, true),
            (10_002_000_000, true),
        ] {
            let mut h = Hand::new(HAND_LEFT, HandConfig::default());
            let mut c = input();
            let start = 5_000_000_000;
            live_sample(&mut h, &mut c, start);
            h.update(&c, start + 100_000_000);
            h.take_event();
            let queue = RadioActivityQueue::default();
            for i in 0..ACTIVITY_SLOTS {
                queue.push(RadioActivity {
                    device_time_ns: c.device_time_ns,
                    received_host_ns: start + duration / 2 + 1 + i as u64,
                    asleep: true,
                    moving: false,
                });
            }
            // Both returns are omitted. The newest snapshot cannot locate the first return:
            // in the middle case it is exactly at expiry, but the gap ended one sample before.
            for i in 1..=2 {
                queue.push(RadioActivity {
                    device_time_ns: c.device_time_ns + i * 2_000_000,
                    received_host_ns: start + duration + (i - 1) * 2_000_000,
                    asleep: false,
                    moving: false,
                });
            }
            let mut events = Vec::new();
            queue.drain_events(&mut h, |e| events.push(e));
            c.device_time_ns += 4_000_000;
            c.received_host_ns = start + duration + 2_000_000;
            h.classify_events(&c, c.received_host_ns, |e| events.push(e));
            assert!(
                matches!(events.as_slice(), [GapEvent::Ended { uncertain: true, recordable: r, asleep: false, .. }] if *r == recordable),
                "duration {duration}: {events:?}"
            );
            // A late newest snapshot alone cannot prove that a lost first return was late.
        }
    }

    #[test]
    fn losing_only_beacons_keeps_a_known_late_return_in_standby() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        let start = 5_000_000_000;
        live_sample(&mut h, &mut c, start);
        h.update(&c, start + 150_000_000);
        h.take_event();
        let queue = RadioActivityQueue::default();
        for i in 0..=ACTIVITY_SLOTS {
            queue.push(RadioActivity {
                device_time_ns: c.device_time_ns,
                received_host_ns: start + 750_000_000 + i as u64,
                asleep: true,
                moving: false,
            });
        }
        queue.drain_events(&mut h, |_| {});
        c.device_time_ns += 2_000_000;
        c.received_host_ns = start + 10_002_000_000;
        queue.push(RadioActivity {
            device_time_ns: c.device_time_ns,
            received_host_ns: c.received_host_ns,
            asleep: false,
            moving: false,
        });
        let mut events = Vec::new();
        queue.drain_events(&mut h, |e| events.push(e));
        h.classify_events(&c, c.received_host_ns, |e| events.push(e));
        assert!(
            !events.iter().any(|e| matches!(
                e,
                GapEvent::Ended {
                    recordable: true,
                    ..
                }
            )),
            "{events:?}"
        );
    }

    #[test]
    fn still_returns_on_both_sides_of_hold_expiry_ignore_publication_timing() {
        for duration in [9_998_000_000, 10_000_000_000, 10_002_000_000] {
            for timer in [false, true] {
                let mut h = Hand::new(HAND_LEFT, HandConfig::default());
                let mut c = input();
                let start = 5_000_000_000;
                live_sample(&mut h, &mut c, start);
                let mut events = Vec::new();
                if timer && duration >= 10_000_000_000 {
                    h.classify_events(&c, start + 10_000_000_000, |e| events.push(e));
                }
                c.device_time_ns += 2_000_000;
                c.received_host_ns = start + duration;
                h.classify_events(&c, c.received_host_ns, |e| events.push(e));
                let ended: Vec<_> = events
                    .into_iter()
                    .filter(|e| matches!(e, GapEvent::Ended { .. }))
                    .collect();
                if duration < 10_000_000_000 {
                    assert_eq!(
                        ended,
                        [GapEvent::Ended {
                            last_sample_host_ns: start,
                            silent_ns: duration,
                            asleep: false,
                            uncertain: false,
                            recordable: true,
                            capture_requested: false,
                            cause: GapCause::Radio,
                        }]
                    );
                } else {
                    assert!(ended.is_empty(), "duration {duration}, timer {timer}");
                }
            }
        }
    }

    #[test]
    fn a_delayed_first_gap_tick_keeps_the_in_use_request_without_a_return() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        c.imu_gyro = [1.0, 0.0, 0.0];
        let start = 5_000_000_000;
        live_sample(&mut h, &mut c, start);
        let mut events = Vec::new();
        h.classify_events(&c, start + 10_002_000_000, |e| events.push(e));
        assert_eq!(
            events,
            [
                GapEvent::Began {
                    last_sample_host_ns: start,
                    moving: true,
                    asleep: false,
                    cause: GapCause::Radio,
                },
                GapEvent::Expired {
                    silent_ns: 10_002_000_000,
                    asleep: false,
                    cause: GapCause::Radio,
                }
            ]
        );
    }

    #[test]
    fn a_handoff_before_the_gap_tick_preserves_the_last_advancing_sample() {
        for (old_clock, new_clock) in [
            (50_000_000_000, 50_000_000_000),
            (1_000_000_000, 1_000_000_000),
        ] {
            let mut outgoing = Hand::new(HAND_LEFT, HandConfig::default());
            let mut incoming = Hand::new(HAND_LEFT, HandConfig::default());
            let mut c = input();
            live_sample(&mut outgoing, &mut c, old_clock);
            // Ownership changes before either publisher has ticked the 100 ms deadline. The
            // receiving reader starts during the silence and sees no earlier sample at all.
            incoming.inherit_classification_at(
                outgoing.classification_history(),
                new_clock + 800_000_000,
            );
            incoming.note_sleep_beacon(new_clock + 750_000_000);
            live_sample(&mut incoming, &mut c, new_clock + 800_000_000);
            assert_eq!(
                incoming.take_event(),
                Some(GapEvent::Ended {
                    last_sample_host_ns: old_clock,
                    silent_ns: 800_000_000,
                    asleep: false,
                    uncertain: true,
                    recordable: true,
                    capture_requested: false,
                    cause: GapCause::Radio,
                })
            );
        }
    }

    #[test]
    fn an_unfinished_gap_survives_both_handoff_directions_until_its_first_return() {
        for (outgoing_clock, incoming_clock) in [
            (50_000_000_000, 50_000_000_000),
            (1_000_000_000, 1_000_000_000),
        ] {
            let mut outgoing = Hand::new(HAND_LEFT, HandConfig::default());
            let mut incoming = Hand::new(HAND_LEFT, HandConfig::default());
            let mut c = input();
            live_sample(&mut outgoing, &mut c, outgoing_clock);
            outgoing.update(&c, outgoing_clock + 150_000_000);
            outgoing.take_event();
            let history = outgoing.classification_history();
            incoming.inherit_classification_at(history, incoming_clock + 800_000_000);
            // The receiving reader may have an older backlog. It must not end the inherited gap.
            assert_eq!(
                decoded(&mut incoming, &[(2_000_000, incoming_clock, false)]),
                []
            );
            live_sample(&mut incoming, &mut c, incoming_clock + 800_000_000);
            assert_eq!(
                incoming.take_event(),
                Some(GapEvent::Ended {
                    last_sample_host_ns: outgoing_clock,
                    silent_ns: 800_000_000,
                    asleep: false,
                    uncertain: true,
                    recordable: true,
                    capture_requested: false,
                    cause: GapCause::Radio,
                })
            );
            assert!(!incoming.classification_history().gap.unwrap().open);
            // No subsequent gap is needed to request this recording.
            assert_eq!(incoming.take_event(), None);
        }
    }

    /// The outgoing reader logged a still gap asleep, then handed the hand over during the nap.
    /// `seen` are the receiving reader's own samples from before the handoff, in ms from the
    /// gap's last sample (negative: before it); it gets a beacon of its own 1.5 s in, and the
    /// controller wakes at `wake_ms`.
    fn nap_across_a_handoff(
        outgoing_clock: u64,
        incoming_clock: u64,
        seen: &[i64],
        wake_ms: u64,
    ) -> Vec<GapEvent> {
        const MS: u64 = 1_000_000;
        let mut outgoing = Hand::new(HAND_LEFT, HandConfig::default());
        let mut incoming = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        live_sample(&mut outgoing, &mut c, outgoing_clock);
        outgoing.update(&c, outgoing_clock + 150 * MS);
        outgoing.take_event();
        outgoing.note_sleep_beacon(outgoing_clock + 750 * MS);
        outgoing.update(&c, outgoing_clock + 750 * MS);
        assert_eq!(outgoing.take_event(), Some(GapEvent::Asleep));
        // Published at 750 ms, taken over 250 ms later on the receiving reader's own clock.
        incoming.inherit_classification_at(
            outgoing.classification_history(),
            incoming_clock + 1_000 * MS,
        );
        let mut events = Vec::new();
        for (i, at) in seen.iter().enumerate() {
            let host = incoming_clock.checked_add_signed(at * MS as i64).unwrap();
            events.extend(decoded(
                &mut incoming,
                &[(c.device_time_ns + i as u64, host, false)],
            ));
        }
        incoming.tick_events(incoming_clock + 1_300 * MS, |e| events.push(e));
        incoming.note_sleep_beacon(incoming_clock + 1_500 * MS);
        incoming.tick_events(incoming_clock + 1_500 * MS, |e| events.push(e));
        incoming.tick_events(incoming_clock + wake_ms.min(10_000) * MS, |e| {
            events.push(e)
        });
        c.device_time_ns += 1_000 * MS;
        c.received_host_ns = incoming_clock + wake_ms * MS;
        incoming.classify_events(&c, c.received_host_ns, |e| events.push(e));
        events
    }

    #[test]
    fn known_handoff_returns_keep_the_half_second_boundary_even_after_expiry() {
        for (duration, keep) in [
            (499_999_999, false),
            (500_000_000, true),
            (10_002_000_000, true),
        ] {
            let (a, b) = (5_000_000_000, 5_000_000_000);
            let mut outgoing = Hand::new(HAND_LEFT, HandConfig::default());
            let mut incoming = Hand::new(HAND_LEFT, HandConfig::default());
            let mut c = input();
            live_sample(&mut outgoing, &mut c, a);
            outgoing.update(&c, a + 150_000_000);
            outgoing.take_event();
            incoming.inherit_classification_at(outgoing.classification_history(), b + 200_000_000);
            let mut events = Vec::new();
            incoming.tick_events(b + duration.min(10_000_000_000), |e| events.push(e));
            c.device_time_ns += 2_000_000;
            c.received_host_ns = b + duration;
            incoming.classify_events(&c, c.received_host_ns, |e| events.push(e));
            assert!(
                events.iter().any(|e| matches!(e, GapEvent::Ended {
                last_sample_host_ns, recordable, asleep: false, ..
            } if *last_sample_host_ns == a && *recordable == keep)),
                "duration {duration}: {events:?}"
            );
        }
    }

    /// However its first return is seen, a still gap that expired is standby once the flight
    /// ring may no longer hold its window: a return `flight::LATEST_NS` after its last sample
    /// is not recorded, one just before it still is.
    #[test]
    fn a_return_too_late_for_the_flight_ring_is_standby() {
        use crate::flight::LATEST_NS;
        const MS: u64 = 1_000_000;
        let start = 5_000_000_000;
        for (duration, recorded) in [
            (LATEST_NS - 1, true),
            (LATEST_NS, false),
            (120_000 * MS, false),
        ] {
            for how in [
                "known after a handoff",
                "discarded before a takeover",
                "lost",
            ] {
                let mut h = Hand::new(HAND_LEFT, HandConfig::default());
                let mut c = input();
                live_sample(&mut h, &mut c, start);
                h.update(&c, start + 150 * MS);
                h.update(&c, start + 10_000 * MS);
                assert!(matches!(h.take_event(), Some(GapEvent::Expired { .. })));
                let returned = start + duration;
                let return_sample = RadioActivity {
                    device_time_ns: c.device_time_ns + 2 * MS,
                    received_host_ns: returned,
                    asleep: false,
                    moving: false,
                };
                let mut incoming = Hand::new(HAND_LEFT, HandConfig::default());
                let queue = RadioActivityQueue::default();
                let mut events = Vec::new();
                match how {
                    "known after a handoff" => {
                        incoming.inherit_classification_at(
                            h.classification_history(),
                            start + 10_100 * MS,
                        );
                        queue.push(return_sample);
                        queue.drain_events(&mut incoming, |e| events.push(e));
                    }
                    "discarded before a takeover" => {
                        queue.push(return_sample);
                        queue.discard();
                        incoming
                            .inherit_classification_at(h.classification_history(), returned + MS);
                        queue.drain_events(&mut incoming, |e| events.push(e));
                    }
                    _ => {
                        for i in 0..ACTIVITY_SLOTS as u64 {
                            queue.push(RadioActivity {
                                received_host_ns: start + 10_100 * MS + i,
                                asleep: true,
                                ..return_sample
                            });
                        }
                        queue.push(return_sample);
                        queue.drain_events(&mut h, |e| events.push(e));
                    }
                }
                let ended: Vec<_> = events
                    .into_iter()
                    .filter(|e| matches!(e, GapEvent::Ended { last_sample_host_ns, .. } if *last_sample_host_ns == start))
                    .collect();
                assert_eq!(
                    ended,
                    if recorded {
                        vec![GapEvent::Ended {
                            last_sample_host_ns: start,
                            silent_ns: duration,
                            asleep: false,
                            uncertain: true,
                            recordable: true,
                            capture_requested: false,
                            cause: GapCause::Radio,
                        }]
                    } else {
                        vec![]
                    },
                    "{how} after {duration} ns"
                );
            }
        }
    }

    #[test]
    fn a_handoff_spanning_gap_is_never_proven_a_nap() {
        let events = nap_across_a_handoff(50_000_000_000, 50_000_000_000, &[0], 12_000);
        assert!(
            events.iter().any(|e| matches!(
                e,
                GapEvent::Ended {
                    asleep: false,
                    recordable: true,
                    ..
                }
            )),
            "{events:?}"
        );
        assert!(
            !events.iter().any(|e| matches!(
                e,
                GapEvent::Asleep
                    | GapEvent::Began { asleep: true, .. }
                    | GapEvent::Ended { asleep: true, .. }
            )),
            "{events:?}"
        );
    }

    #[test]
    fn a_handoff_clears_sleep_even_through_expiry() {
        for (a, b) in [
            (50_000_000_000, 50_000_000_000),
            (1_000_000_000, 1_000_000_000),
        ] {
            let events = nap_across_a_handoff(a, b, &[0], 12_000);
            assert!(
                events
                    .iter()
                    .any(|e| matches!(e, GapEvent::Expired { asleep: false, .. })),
                "{events:?}"
            );
            assert!(
                matches!(
                    events.last(),
                    Some(GapEvent::Ended {
                        asleep: false,
                        recordable: true,
                        ..
                    })
                ),
                "{events:?}"
            );
        }
    }

    #[test]
    fn a_discarded_moving_return_starts_its_own_in_use_gap_without_more_samples() {
        let (a, b) = (5_000_000_000, 5_000_000_000);
        let mut outgoing = Hand::new(HAND_LEFT, HandConfig::default());
        let mut incoming = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        live_sample(&mut outgoing, &mut c, a);
        outgoing.update(&c, a + 150_000_000);
        outgoing.take_event();
        let queue = RadioActivityQueue::default();
        c.device_time_ns += 2_000_000;
        c.received_host_ns = b + 600_000_000;
        c.imu_gyro = [1.0, 0.0, 0.0];
        queue.push(RadioActivity {
            device_time_ns: c.device_time_ns,
            received_host_ns: c.received_host_ns,
            asleep: false,
            moving: true,
        });
        queue.discard();
        incoming.inherit_classification_at(outgoing.classification_history(), b + 1_000_000_000);
        let mut events = Vec::new();
        queue.drain_events(&mut incoming, |e| events.push(e));
        incoming.classify_events(&c, b + 1_000_000_000, |e| events.push(e));
        assert!(
            events
                .iter()
                .any(|e| matches!(e, GapEvent::Began { last_sample_host_ns,
            moving: true, asleep: false , cause: GapCause::Radio,} if *last_sample_host_ns == b + 600_000_000)),
            "{events:?}"
        );
    }

    #[test]
    fn receiver_beacons_never_prove_sleep_for_the_original_handoff_gap() {
        let anchor = 50_000_000_000;
        for seen in [&[][..], &[0][..], &[0, 300][..], &[-2_000][..]] {
            for wake in [5_000, 12_000] {
                let events = nap_across_a_handoff(anchor, anchor, seen, wake);
                assert!(
                    events.iter().any(|e| matches!(e, GapEvent::Ended {
                    last_sample_host_ns, asleep: false, recordable, ..
                } if *last_sample_host_ns == anchor && *recordable != seen.contains(&300))),
                    "{seen:?}, {wake}: {events:?}"
                );
                assert!(
                    !events.iter().any(|e| matches!(e, GapEvent::Ended {
                    last_sample_host_ns, asleep: true, ..
                } if *last_sample_host_ns == anchor)),
                    "{events:?}"
                );
            }
        }
    }

    /// A reader that lost or discarded samples while it did not publish cannot tell which was
    /// the first return during the gap it takes over: its own beacons after the handoff prove no
    /// sleep then, and that return may have come before the hold expired. Its own copy of the
    /// gap's last sample, stamped at its own read, is not a return.
    #[test]
    fn discarded_or_lost_returns_end_only_the_original_handoff_gap() {
        const MS: u64 = 1_000_000;
        // (a return, ms on the shared clock; the gap's last sample at 1 s, its start seen by the
        // outgoing reader at 1.15 s and the handoff at 1.4 s; whether this reader's queue was
        // full, so that the return was lost rather than discarded; when the controller wakes)
        for (lost_ms, full, wake_ms) in [
            (None, true, 3_000),
            (None, false, 12_000),
            (Some(1_120), true, 3_000),
            (Some(1_300), true, 12_000),
            (Some(1_120), false, 3_000),
            (Some(1_120), false, 12_000),
        ] {
            // This reader stamped its copy of the gap's last sample before, with or after the
            // outgoing reader, by up to tens of ms.
            for copy_ms in [-3, 0, 3, 60] {
                let mut outgoing = Hand::new(HAND_LEFT, HandConfig::default());
                let mut incoming = Hand::new(HAND_LEFT, HandConfig::default());
                let queue = RadioActivityQueue::default();
                let clock = 50_000_000_000;
                let anchor = clock + 1_000 * MS;
                let mut c = input();
                live_sample(&mut outgoing, &mut c, anchor);
                outgoing.update(&c, clock + 1_150 * MS);
                assert!(matches!(
                    outgoing.take_event(),
                    Some(GapEvent::Began { .. })
                ));
                // This reader's queue ends at its copy of the gap's last sample.
                let copy = anchor.checked_add_signed(copy_ms * MS as i64).unwrap();
                let n = if full { ACTIVITY_SLOTS as u64 } else { 1 };
                for i in 0..n {
                    queue.push(RadioActivity {
                        device_time_ns: i,
                        received_host_ns: copy - (n - 1 - i) * 2 * MS,
                        asleep: false,
                        moving: false,
                    });
                }
                if let Some(lost) = lost_ms {
                    // A return it read, then lost or discarded; the outgoing reader had stopped
                    // reading.
                    queue.push(RadioActivity {
                        device_time_ns: 1_000,
                        received_host_ns: clock + lost * MS,
                        asleep: false,
                        moving: false,
                    });
                }
                queue.discard();
                incoming.inherit_classification_at(
                    outgoing.classification_history(),
                    clock + 1_400 * MS,
                );
                let mut events = Vec::new();
                queue.drain_events(&mut incoming, |e| events.push(e));
                queue.push(RadioActivity {
                    device_time_ns: 1_000,
                    received_host_ns: clock + 1_500 * MS,
                    asleep: true,
                    moving: false,
                });
                queue.drain_events(&mut incoming, |e| events.push(e));
                c.device_time_ns += 1_000 * MS;
                c.received_host_ns = clock + wake_ms * MS;
                incoming.classify_events(&c, c.received_host_ns, |e| events.push(e));
                // The original gap ends once, at its first actual return.
                let ended: Vec<_> = events
                    .iter()
                    .filter(|e| matches!(e, GapEvent::Ended { last_sample_host_ns, .. } if *last_sample_host_ns == anchor))
                    .collect();
                assert_eq!(
                    ended,
                    [&GapEvent::Ended {
                        last_sample_host_ns: anchor,
                        silent_ns: (lost_ms.unwrap_or(wake_ms) - 1_000) * MS,
                        asleep: false,
                        uncertain: true,
                        recordable: true,
                        capture_requested: false,
                        cause: GapCause::Radio,
                    }],
                    "lost at {lost_ms:?}, copy {copy_ms} ms: {events:?}"
                );
            }
        }
    }

    /// Both readers read every report and stamp it at their own read: the receiving reader's
    /// copy of the gap's last sample can be stamped either side of the outgoing reader's anchor.
    /// Whether it was discarded before the takeover, read after it, or lost to a full queue, that
    /// copy never ends a gap that had opened, and never undoes a nap settled before a reopen.
    #[test]
    fn another_readers_copy_of_the_last_sample_never_ends_an_opened_gap() {
        const MS: u64 = 1_000_000;
        for reopen in [false, true] {
            for how in [
                "discarded",
                "discarded after overflow",
                "retained",
                "retained after overflow",
            ] {
                for copy_ms in [-3, 0, 1, 3, 20, 60, 99] {
                    let mut outgoing = Hand::new(HAND_LEFT, HandConfig::default());
                    let mut incoming = Hand::new(HAND_LEFT, HandConfig::default());
                    let mut c = input();
                    let anchor = 50_000_000_000;
                    live_sample(&mut outgoing, &mut c, anchor);
                    outgoing.update(&c, anchor + 150 * MS);
                    outgoing.take_event();
                    outgoing.note_sleep_beacon(anchor + 750 * MS);
                    outgoing.update(&c, anchor + 750 * MS);
                    assert_eq!(outgoing.take_event(), Some(GapEvent::Asleep));
                    if reopen {
                        outgoing.preserve_reopen_sleep();
                    }
                    // This reader's copies of the last few samples, the newest the gap's last.
                    let copy = anchor.checked_add_signed(copy_ms * MS as i64).unwrap();
                    let queue = RadioActivityQueue::default();
                    let n = if how.ends_with("overflow") {
                        ACTIVITY_SLOTS as u64 + 1
                    } else {
                        4
                    };
                    for i in 0..n {
                        queue.push(RadioActivity {
                            device_time_ns: i,
                            received_host_ns: copy - (n - 1 - i) * 2 * MS,
                            asleep: false,
                            moving: false,
                        });
                    }
                    if how.starts_with("discarded") {
                        queue.discard();
                    }
                    incoming.inherit_classification_at(
                        outgoing.classification_history(),
                        anchor + 1_000 * MS,
                    );
                    let mut events = Vec::new();
                    queue.drain_events(&mut incoming, |e| events.push(e));
                    assert_eq!(events, [], "{how} copy {copy_ms} ms, reopen {reopen}");
                    // Only an overflow after the takeover vetoes a settled nap.
                    let asleep = reopen && how != "retained after overflow";
                    assert_eq!(
                        incoming
                            .classification_history()
                            .gap
                            .unwrap()
                            .confirmed_sleep_before_ns
                            != 0,
                        asleep,
                        "{how} copy {copy_ms} ms, reopen {reopen}"
                    );
                    if reopen {
                        incoming.reset_reader_evidence();
                    }
                    c.device_time_ns += 1_000 * MS;
                    c.received_host_ns = anchor + 5_000 * MS;
                    incoming.classify_events(&c, c.received_host_ns, |e| events.push(e));
                    assert_eq!(
                        events,
                        [GapEvent::Ended {
                            last_sample_host_ns: anchor,
                            silent_ns: 5_000 * MS,
                            asleep,
                            uncertain: true,
                            recordable: !asleep,
                            capture_requested: false,
                            cause: GapCause::Radio,
                        }],
                        "{how} copy {copy_ms} ms, reopen {reopen}"
                    );
                }
            }
        }
    }

    /// Only the receiving reader decoded the motion just before a still gap, and it stamped its
    /// copy after the outgoing reader's anchor. That is still use before the gap began, even when
    /// a later moving return it also discarded replaced its newest motion time.
    #[test]
    fn a_copys_motion_is_use_before_the_gap_however_late_it_was_stamped() {
        const MS: u64 = 1_000_000;
        for (discarded, moving_return) in [(false, false), (true, false), (true, true)] {
            for copy_ms in [3, 60] {
                let mut outgoing = Hand::new(HAND_LEFT, HandConfig::default());
                let mut incoming = Hand::new(HAND_LEFT, HandConfig::default());
                let mut c = input();
                let anchor = 50_000_000_000;
                live_sample(&mut outgoing, &mut c, anchor);
                outgoing.update(&c, anchor + 150 * MS);
                assert!(matches!(
                    outgoing.take_event(),
                    Some(GapEvent::Began { moving: false, .. })
                ));
                let queue = RadioActivityQueue::default();
                queue.push(RadioActivity {
                    device_time_ns: 7,
                    received_host_ns: anchor + copy_ms * MS,
                    asleep: false,
                    moving: true,
                });
                if moving_return {
                    queue.push(RadioActivity {
                        device_time_ns: 9,
                        received_host_ns: anchor + 300 * MS,
                        asleep: false,
                        moving: true,
                    });
                }
                if discarded {
                    queue.discard();
                }
                incoming.inherit_classification_at(
                    outgoing.classification_history(),
                    anchor + 400 * MS,
                );
                let mut events = Vec::new();
                queue.drain_events(&mut incoming, |e| events.push(e));
                assert_eq!(
                    events.first(),
                    Some(&GapEvent::InUse {
                        last_sample_host_ns: anchor,
                        cause: GapCause::Radio,
                    }),
                    "discarded {discarded}, moving return {moving_return}, copy {copy_ms} ms: {events:?}"
                );
                assert_eq!(events.len(), 1 + usize::from(moving_return), "{events:?}");
            }
        }
    }

    /// A gap opens only after the connect timeout without a sample, so another reader's sample
    /// from then on is a return, even when it was discarded before the takeover.
    #[test]
    fn a_discarded_sample_from_the_connect_timeout_on_ends_the_inherited_gap() {
        const MS: u64 = 1_000_000;
        let timeout = HandConfig::default().connect_timeout_ns;
        for (after, ends) in [(timeout - 1, false), (timeout, true)] {
            let mut outgoing = Hand::new(HAND_LEFT, HandConfig::default());
            let mut incoming = Hand::new(HAND_LEFT, HandConfig::default());
            let mut c = input();
            let anchor = 50_000_000_000;
            live_sample(&mut outgoing, &mut c, anchor);
            outgoing.update(&c, anchor + 150 * MS);
            outgoing.take_event();
            let queue = RadioActivityQueue::default();
            queue.push(RadioActivity {
                device_time_ns: 7,
                received_host_ns: anchor + after,
                asleep: false,
                moving: false,
            });
            queue.discard();
            incoming
                .inherit_classification_at(outgoing.classification_history(), anchor + 400 * MS);
            let mut events = Vec::new();
            queue.drain_events(&mut incoming, |e| events.push(e));
            let ended = GapEvent::Ended {
                last_sample_host_ns: anchor,
                silent_ns: after,
                asleep: false,
                uncertain: true,
                recordable: true,
                capture_requested: false,
                cause: GapCause::Radio,
            };
            assert_eq!(events, if ends { vec![ended] } else { vec![] });
        }
    }

    /// Returning ownership to the original reader cannot restore sleep proof to the interval.
    #[test]
    fn a_round_trip_handoff_keeps_the_original_identity_and_never_proves_sleep() {
        const MS: u64 = 1_000_000;
        for wake_ms in [5_000, 12_000] {
            let mut a = Hand::new(HAND_LEFT, HandConfig::default());
            let mut b = Hand::new(HAND_LEFT, HandConfig::default());
            let mut c = input();
            let (a_clock, b_clock) = (1_000_000_000, 1_000_000_000);
            live_sample(&mut a, &mut c, a_clock);
            a.update(&c, a_clock + 150 * MS);
            a.take_event();
            b.inherit_classification_at(a.classification_history(), b_clock + 200 * MS);
            b.tick_events(b_clock + 300 * MS, |_| {});
            a.inherit_classification_at(b.classification_history(), a_clock + 400 * MS);
            a.note_sleep_beacon(a_clock + 1_500 * MS);
            let mut events = Vec::new();
            a.tick_events(a_clock + wake_ms.min(10_000) * MS, |e| events.push(e));
            c.device_time_ns += 1_000 * MS;
            c.received_host_ns = a_clock + wake_ms * MS;
            a.classify_events(&c, c.received_host_ns, |e| events.push(e));
            assert!(
                events.iter().any(|e| matches!(e, GapEvent::Ended {
                last_sample_host_ns, asleep: false, recordable: true, ..
            } if *last_sample_host_ns == a_clock)),
                "{events:?}"
            );
        }
    }

    /// Activity a reader discards while the other reader publishes the hand cannot set off loss
    /// flags at its takeover. Only when it happened counts: motion before the handoff, whether
    /// this reader read the gap begin, and whether it saw the controller during it.
    #[test]
    fn discarded_activity_keeps_only_its_timing_for_the_takeover() {
        const MS: u64 = 1_000_000;
        // (when the overflow was, ms after the reader started; motion decoded but discarded, ms
        // before the handoff at 8 s; whether the gap after the handoff begins in use), with this
        // reader's copy of the last sample stamped before, with or after the outgoing reader's
        for ((overflow_ms, motion_before_handoff, in_use), copy_ms) in [
            (0, None, false),
            (0, Some(2_900), true),
            (0, Some(3_100), false),
            // The lost motion is no older than the discard after it, 500 ms before the handoff.
            (6_900, None, true),
        ]
        .into_iter()
        .flat_map(|case| [-3, 0, 2].map(|copy_ms| (case, copy_ms)))
        {
            let mut outgoing = Hand::new(HAND_LEFT, HandConfig::default());
            let mut incoming = Hand::new(HAND_LEFT, HandConfig::default());
            let queue = RadioActivityQueue::default();
            let (out_clock, in_clock) = (50_000_000_000, 50_000_000_000);
            // This reader's queue overflowed while it did not publish, losing motion among its
            // entries, and was discarded 600 ms after the overflow began.
            for i in 0..ACTIVITY_SLOTS as u64 + 10 {
                queue.push(RadioActivity {
                    device_time_ns: i,
                    received_host_ns: in_clock + (overflow_ms + i * 2) * MS,
                    asleep: false,
                    moving: i > ACTIVITY_SLOTS as u64,
                });
            }
            queue.discard();
            if let Some(before) = motion_before_handoff {
                queue.push(RadioActivity {
                    device_time_ns: 1_000,
                    received_host_ns: in_clock + (8_000 - before) * MS,
                    asleep: false,
                    moving: true,
                });
            }
            // Its own copy of the outgoing reader's last sample, just before the handoff. Read
            // before the gap began, that copy is this reader's own stamp of its last sample.
            let copy = (in_clock + 7_997 * MS)
                .checked_add_signed(copy_ms * MS as i64)
                .unwrap();
            queue.push(RadioActivity {
                device_time_ns: 2_000,
                received_host_ns: copy,
                asleep: false,
                moving: false,
            });
            queue.discard();
            let mut c = input();
            live_sample(&mut outgoing, &mut c, out_clock + 7_997 * MS);
            incoming.inherit_classification_at(
                outgoing.classification_history(),
                in_clock + 8_000 * MS,
            );
            let mut events = Vec::new();
            queue.drain_events(&mut incoming, |e| events.push(e));
            queue.push(RadioActivity {
                device_time_ns: c.device_time_ns,
                received_host_ns: in_clock + 8_002 * MS,
                asleep: true,
                moving: false,
            });
            queue.drain_events(&mut incoming, |e| events.push(e));
            incoming.tick_events(in_clock + 8_150 * MS, |e| events.push(e));
            assert_eq!(
                events,
                [GapEvent::Began {
                    last_sample_host_ns: copy.max(out_clock + 7_997 * MS),
                    moving: in_use,
                    asleep: false,
                    cause: GapCause::Radio,
                }],
                "overflow at {overflow_ms} ms, motion {motion_before_handoff:?} ms before the handoff, copy {copy_ms} ms"
            );
        }
    }

    #[test]
    fn a_drain_keeps_every_completed_gap_before_the_newest_snapshot() {
        let mut hand = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        let start = 5_000_000_000;
        live_sample(&mut hand, &mut c, start);
        let queue = RadioActivityQueue::default();
        for i in 1..=3 {
            queue.push(RadioActivity {
                device_time_ns: c.device_time_ns + i * 2_000_000,
                received_host_ns: start + i * 800_000_000,
                asleep: false,
                moving: false,
            });
        }
        let mut ended = Vec::new();
        queue.drain_events(&mut hand, |e| {
            if let GapEvent::Ended {
                last_sample_host_ns,
                silent_ns,
                asleep,
                ..
            } = e
            {
                assert!(!asleep);
                assert_eq!(silent_ns, 800_000_000);
                ended.push(last_sample_host_ns);
            }
        });
        assert_eq!(ended, [start, start + 800_000_000, start + 1_600_000_000]);
    }

    #[test]
    fn motion_in_an_overflowed_sample_cannot_become_a_nap() {
        let mut hand = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        let start = 5_000_000_000;
        live_sample(&mut hand, &mut c, start);
        let queue = RadioActivityQueue::default();
        for i in 1..=ACTIVITY_SLOTS + 2 {
            queue.push(RadioActivity {
                device_time_ns: c.device_time_ns + i as u64 * 2_000_000,
                received_host_ns: start + i as u64 * 2_000_000,
                asleep: false,
                moving: i == ACTIVITY_SLOTS + 1,
            });
        }
        queue.drain_events(&mut hand, |_| {});
        c.device_time_ns += (ACTIVITY_SLOTS as u64 + 2) * 2_000_000;
        c.received_host_ns = start + (ACTIVITY_SLOTS as u64 + 2) * 2_000_000;
        hand.update(&c, c.received_host_ns);
        hand.take_event();
        hand.note_sleep_beacon(c.received_host_ns + 2_000_000);
        hand.update(&c, c.received_host_ns + 150_000_000);
        assert!(matches!(
            hand.take_event(),
            Some(GapEvent::Began {
                moving: true,
                asleep: false,
                ..
            })
        ));
    }

    #[test]
    fn imported_use_survives_still_samples_for_three_receiver_seconds() {
        let mut outgoing = Hand::new(HAND_LEFT, HandConfig::default());
        let mut old = input();
        old.imu_gyro = [1.0, 0.0, 0.0];
        live_sample(&mut outgoing, &mut old, 50_000_000_000);
        old.imu_gyro = [0.0; 3];
        live_sample(&mut outgoing, &mut old, 52_900_000_000);
        let history = outgoing.classification_history();
        assert!(history.gap_in_use);
        for (elapsed, in_use) in [
            (2_999_999_999, true),
            (3_000_000_000, true),
            (3_000_000_001, false),
        ] {
            let mut incoming = Hand::new(HAND_LEFT, HandConfig::default());
            let mut c = input();
            let start = 52_900_000_000;
            incoming.inherit_classification_at(history, start);
            live_sample(&mut incoming, &mut c, start + 2_000_000);
            live_sample(&mut incoming, &mut c, start + elapsed);
            incoming.take_event();
            incoming.note_sleep_beacon(start + elapsed + 2_000_000);
            incoming.update(&c, start + elapsed + 150_000_000);
            assert!(
                matches!(incoming.take_event(), Some(GapEvent::Began { moving, asleep, .. })
                if moving == in_use && asleep != in_use),
                "elapsed {elapsed}"
            );
        }
    }

    #[test]
    fn ordered_samples_keep_the_first_return_and_ignore_duplicate_device_times() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        let start = 5_000_000_000;
        live_sample(&mut h, &mut c, start);
        h.update(&c, start + 150_000_000);
        h.take_event();
        let events = decoded(
            &mut h,
            &[
                (c.device_time_ns, start + 700_000_000, false),
                (c.device_time_ns, start + 750_000_000, true),
                (c.device_time_ns + 2_000_000, start + 800_000_000, false),
                (c.device_time_ns, start + 802_000_000, true),
                (c.device_time_ns + 4_000_000, start + 804_000_000, false),
            ],
        );
        assert_eq!(
            events,
            [GapEvent::Ended {
                last_sample_host_ns: start,
                silent_ns: 800_000_000,
                asleep: true,
                uncertain: false,
                recordable: false,
                capture_requested: false,
                cause: GapCause::Radio,
            }]
        );
        c.device_time_ns += 4_000_000;
        c.received_host_ns = start + 804_000_000;
        h.update(&c, c.received_host_ns);
        assert_eq!(h.take_event(), None);
        h.update(&c, start + 954_000_000);
        assert!(matches!(
            h.take_event(),
            Some(GapEvent::Began { asleep: false, .. })
        ));
    }

    #[test]
    fn lost_ordered_activity_cannot_identify_sleep() {
        let queue = RadioActivityQueue::default();
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        let start = 5_000_000_000;
        live_sample(&mut h, &mut c, start);
        h.update(&c, start + 150_000_000);
        h.take_event();
        for i in 0..=ACTIVITY_SLOTS {
            queue.push(RadioActivity {
                device_time_ns: c.device_time_ns,
                received_host_ns: start + 750_000_000 + i as u64,
                asleep: true,
                moving: false,
            });
        }
        queue.drain_events(&mut h, |_| {});
        live_sample(&mut h, &mut c, start + 800_000_000);
        assert!(matches!(
            h.take_event(),
            Some(GapEvent::Ended { asleep: false, .. })
        ));
        queue.push(RadioActivity {
            device_time_ns: c.device_time_ns,
            received_host_ns: start + 900_000_000,
            asleep: true,
            moving: false,
        });
        queue.drain_events(&mut h, |_| {});
        h.update(&c, start + 950_000_000);
        assert!(matches!(
            h.take_event(),
            Some(GapEvent::Began { asleep: true, .. })
        ));
    }

    #[test]
    fn a_sleep_beacon_preserves_the_hold_and_the_wake_after_expiry() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        c.trigger = 0.8;
        c.buttons = 1;
        let host = 5_000_000_000;
        let live = live_sample(&mut h, &mut c, host);
        h.update(&c, host + 150_000_000);
        h.take_event();
        h.note_sleep_beacon(host + 750_000_000);
        let held = h.update(&c, host + 750_000_000);
        assert_eq!(h.take_event(), Some(GapEvent::Asleep));
        assert_eq!(
            held.flags,
            FLAG_CALIBRATED | FLAG_CONNECTED | FLAG_RADIO_GAP | FLAG_ASLEEP
        );
        assert!(!reporting(held.flags));
        assert_eq!((held.buttons, held.trigger), (live.buttons, live.trigger));
        assert_eq!(
            (held.device_time_ns, held.received_host_ns),
            (live.device_time_ns, live.received_host_ns)
        );
        assert_eq!(h.update(&c, host + 9_999_999_999).trigger, live.trigger);
        assert_eq!(h.take_event(), None);
        let expired = h.update(&c, host + 10_000_000_000);
        assert_eq!(expired.flags, FLAG_CALIBRATED | FLAG_ASLEEP);
        assert_eq!((expired.buttons, expired.trigger), (0, 0.0));
        assert_eq!(
            h.take_event(),
            Some(GapEvent::Expired {
                silent_ns: 10_000_000_000,
                asleep: true,
                cause: GapCause::Radio,
            })
        );
        h.note_sleep_beacon(host + 11_000_000_000);
        h.update(&c, host + 11_000_000_000);
        assert_eq!(h.take_event(), None);
        let back = live_sample(&mut h, &mut c, host + 12_000_000_000);
        assert!(reporting(back.flags));
        assert_eq!(back.flags & FLAG_ASLEEP, 0);
        assert_eq!(
            h.take_event(),
            Some(GapEvent::Ended {
                last_sample_host_ns: host,
                silent_ns: 12_000_000_000,
                asleep: true,
                uncertain: false,
                recordable: false,
                capture_requested: false,
                cause: GapCause::Radio,
            })
        );
        // A later gap cannot inherit an earlier sleep's beacon.
        h.update(&c, host + 12_150_000_000);
        assert_eq!(
            h.take_event(),
            Some(GapEvent::Began {
                last_sample_host_ns: host + 12_000_000_000,
                moving: false,
                asleep: false,
                cause: GapCause::Radio,
            })
        );
    }

    #[test]
    fn a_beacon_after_expiry_can_still_identify_sleep() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        let host = 5_000_000_000;
        live_sample(&mut h, &mut c, host);
        h.update(&c, host + 150_000_000);
        h.take_event();
        h.update(&c, host + 10_000_000_000);
        assert_eq!(
            h.take_event(),
            Some(GapEvent::Expired {
                silent_ns: 10_000_000_000,
                asleep: false,
                cause: GapCause::Radio,
            })
        );
        h.note_sleep_beacon(host + 11_000_000_000);
        let asleep = h.update(&c, host + 11_000_000_000);
        assert_eq!(asleep.flags, FLAG_CALIBRATED | FLAG_ASLEEP);
        assert_eq!(h.take_event(), Some(GapEvent::Asleep));
        live_sample(&mut h, &mut c, host + 12_000_000_000);
        assert_eq!(
            h.take_event(),
            Some(GapEvent::Ended {
                last_sample_host_ns: host,
                silent_ns: 12_000_000_000,
                asleep: true,
                uncertain: false,
                recordable: false,
                capture_requested: false,
                cause: GapCause::Radio,
            })
        );
    }

    #[test]
    fn a_beacon_at_the_wake_still_classifies_the_gap_that_ended() {
        let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut c = input();
        let host = 5_000_000_000;
        live_sample(&mut h, &mut c, host);
        h.update(&c, host + 150_000_000);
        h.take_event();
        h.note_sleep_beacon(host + 750_000_000);
        live_sample(&mut h, &mut c, host + 800_000_000);
        assert_eq!(
            h.take_event(),
            Some(GapEvent::Ended {
                last_sample_host_ns: host,
                silent_ns: 800_000_000,
                asleep: true,
                uncertain: false,
                recordable: false,
                capture_requested: false,
                cause: GapCause::Radio,
            })
        );
    }

    #[test]
    fn uncalibrated() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let out = h.update(&ffi::ohmd_cv1touch_controller::default(), 1000);
        assert_eq!(out.flags, 0);
        assert_eq!(out.battery_percent, -1);
        assert_eq!(out.publish_host_ns, 1000);
    }

    /// Review Focus 1: standby must not be published as a live, positioned controller.
    #[test]
    fn standby_and_wake() {
        let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut c = input();
        let (mut host, mut dev) = (1_000_000_000_000u64, 50_000_000u64);
        let mut out = ControllerState::default();
        for _ in 0..50 {
            dev += 2_000_000;
            host += 2_000_000;
            c.device_time_ns = dev;
            c.received_host_ns = host;
            c.last_pose_fix_device_ns = dev;
            c.cap[CAP_THUMBREST] = 1.0;
            out = h.update(&c, host);
        }
        assert!(out.flags & FLAG_CONNECTED != 0 && out.flags & FLAG_POSITION != 0);
        assert!(out.touches & TOUCH_THUMBREST != 0);

        // Samples stop (standby); the engine keeps returning the frozen pose. A radio gap first,
        // then standby once the ride-through is over.
        let out = h.update(&c, host + 150_000_000);
        assert_eq!(out.flags & (FLAG_RADIO_GAP | FLAG_POSITION), FLAG_RADIO_GAP);
        let out = h.update(&c, host + 10_000_000_000);
        assert!(out.flags & FLAG_CALIBRATED != 0);
        assert_eq!(
            out.flags & (FLAG_CONNECTED | FLAG_POSITION | FLAG_RADIO_GAP),
            0
        );
        assert_eq!(out.touches, 0);

        // Wake: samples resume; position needs a new optical fix.
        dev += 30_000_000_000;
        host += 30_000_000_000;
        c.device_time_ns = dev;
        c.received_host_ns = host;
        let out = h.update(&c, host);
        assert!(out.flags & FLAG_CONNECTED != 0);
        assert_eq!(out.flags & FLAG_POSITION, 0);
    }

    /// A controller that drops out for good (standby, out of radio range) must not keep publishing
    /// held inputs: a trigger held at the moment of disconnect would otherwise stay held forever.
    #[test]
    fn inputs_released_when_disconnected() {
        let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 10_000_000;
        c.received_host_ns = 1_000_000_000;
        c.buttons = 0x1;
        c.trigger = 0.9;
        c.grip = 0.8;
        c.stick = [0.5, -0.5];
        c.cap = [1.0; 5];
        let live = h.update(&c, 1_000_000_000);
        assert!(live.flags & FLAG_CONNECTED != 0 && live.buttons != 0 && live.trigger > 0.5);
        let gone = h.update(&c, 1_000_000_000 + 10_000_000_000);
        assert_eq!(gone.flags & FLAG_CONNECTED, 0);
        assert_eq!((gone.buttons, gone.touches), (0, 0));
        assert_eq!(
            (gone.trigger, gone.grip, gone.stick, gone.cap),
            (0.0, 0.0, [0.0; 2], [0.0; 5])
        );
    }

    /// One live sample at `host` with these inputs, the device clock a step on.
    /// This reader decodes (device time, receive time, beacon) in order, through its own queue
    /// as the daemon does, and returns what that classified.
    fn decoded(h: &mut Hand, activity: &[(u64, u64, bool)]) -> Vec<GapEvent> {
        let queue = RadioActivityQueue::default();
        for &(device_time_ns, received_host_ns, asleep) in activity {
            queue.push(RadioActivity {
                device_time_ns,
                received_host_ns,
                asleep,
                moving: false,
            });
        }
        let mut events = Vec::new();
        queue.drain_events(h, |e| events.push(e));
        events
    }

    fn live_sample(
        h: &mut Hand,
        c: &mut ffi::ohmd_cv1touch_controller,
        host: u64,
    ) -> ControllerState {
        c.device_time_ns += 2_000_000;
        c.received_host_ns = host;
        h.update(c, host)
    }

    /// Through a radio gap the controller stays connected with every input as it last reported
    /// it, whatever the engine keeps or a sleeping controller's beacon says, and without a
    /// position; the gap is reported as it begins and as it ends.
    #[test]
    fn radio_gap_holds_the_inputs_as_they_were() {
        let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 10_000_000;
        c.buttons = 0x1 | 0x8;
        c.trigger = 0.9;
        c.grip = 0.8;
        c.stick = [0.5, -0.5];
        c.cap = [1.0; 5];
        c.imu_gyro = [0.0, 2.0, 0.0];
        let host = 1_000_000_000;
        let live = live_sample(&mut h, &mut c, host);
        assert!(reporting(live.flags));
        assert_eq!(h.take_event(), None);

        // the samples stop; the engine's copy of the inputs changes all the same
        c.buttons = 0;
        c.trigger = 0.0;
        c.grip = 0.0;
        c.stick = [0.0; 2];
        c.cap = [0.0; 5];
        let held = h.update(&c, host + 150_000_000);
        assert_eq!(
            held.flags,
            FLAG_CALIBRATED | FLAG_CONNECTED | FLAG_RADIO_GAP
        );
        assert!(!reporting(held.flags));
        assert_eq!(held.buttons, BTN_A | BTN_STICK);
        assert_eq!(
            (held.trigger, held.grip, held.stick, held.cap),
            (0.9, 0.8, [0.5, -0.5], [1.0; 5])
        );
        assert_eq!(held.touches, live.touches);
        assert_ne!(held.touches, 0);
        assert_eq!(
            h.take_event(),
            Some(GapEvent::Began {
                last_sample_host_ns: host,
                moving: true,
                asleep: false,
                cause: GapCause::Radio,
            })
        );
        // reported once
        let still = h.update(&c, host + 9_000_000_000);
        assert_eq!(still.flags & FLAG_RADIO_GAP, FLAG_RADIO_GAP);
        assert_eq!(still.buttons, BTN_A | BTN_STICK);
        assert_eq!(h.take_event(), None);

        // back: the inputs are the controller's again
        let back = live_sample(&mut h, &mut c, host + 9_500_000_000);
        assert!(reporting(back.flags));
        assert_eq!((back.buttons, back.trigger), (0, 0.0));
        assert_eq!(
            h.take_event(),
            Some(GapEvent::Ended {
                last_sample_host_ns: host,
                silent_ns: 9_500_000_000,
                asleep: false,
                uncertain: false,
                recordable: true,
                capture_requested: true,
                cause: GapCause::Radio,
            })
        );
    }

    /// A gap longer than the ride-through ends in standby, reported once, inputs released.
    #[test]
    fn radio_gap_expires_into_standby() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        c.trigger = 1.0;
        let host = 5_000_000_000;
        live_sample(&mut h, &mut c, host);
        h.update(&c, host + 200_000_000);
        assert!(matches!(
            h.take_event(),
            Some(GapEvent::Began { moving: false, .. })
        ));
        let held = h.update(&c, host + 9_999_000_000);
        assert_eq!(held.trigger, 1.0);
        let gone = h.update(&c, host + 10_001_000_000);
        assert_eq!(gone.flags, FLAG_CALIBRATED);
        assert_eq!(gone.trigger, 0.0);
        assert_eq!(
            h.take_event(),
            Some(GapEvent::Expired {
                silent_ns: 10_001_000_000,
                asleep: false,
                cause: GapCause::Radio,
            })
        );
        h.update(&c, host + 12_000_000_000);
        assert_eq!(h.take_event(), None);
        // a sample seen only once it is old is no gap of a controller published live long before
        c.device_time_ns += 2_000_000;
        c.received_host_ns = host + 20_000_000_000;
        assert_eq!(h.update(&c, host + 20_150_000_000).flags, FLAG_CALIBRATED);
        assert_eq!(h.take_event(), None);
        // waking long after is no gap's end
        live_sample(&mut h, &mut c, host + 30_000_000_000);
        assert_eq!(h.take_event(), None);
    }

    /// A gap holds what was published live: a controller never published live since its samples
    /// stopped (a publisher that first sees it late) is in standby, with nothing held.
    #[test]
    fn no_radio_gap_without_a_live_state_to_hold() {
        let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 10_000_000;
        c.received_host_ns = 1_000_000_000;
        c.buttons = 0x1;
        let late = h.update(&c, 1_150_000_000);
        assert_eq!(late.flags, FLAG_CALIBRATED);
        assert_eq!(late.buttons, 0);
        assert_eq!(h.take_event(), None);
    }

    /// `moving` says whether the controller turned within a few seconds of its last sample: one
    /// put down and left still has not.
    #[test]
    fn radio_gap_says_whether_the_controller_was_in_use() {
        let gap_after = |still_for_ns: u64| {
            let mut h = Hand::new(HAND_LEFT, HandConfig::default());
            let mut c = input();
            c.imu_gyro = [0.6, 0.0, 0.0];
            live_sample(&mut h, &mut c, 1_000_000_000);
            c.imu_gyro = [0.01, 0.0, 0.02];
            let mut t = 1_000_000_000;
            while t < 1_000_000_000 + still_for_ns {
                t += 2_000_000;
                live_sample(&mut h, &mut c, t);
            }
            h.update(&c, t + 150_000_000);
            match h.take_event() {
                Some(GapEvent::Began { moving, .. }) => moving,
                e => panic!("{e:?}"),
            }
        };
        assert!(gap_after(2_900_000_000));
        assert!(!gap_after(3_100_000_000));
    }

    #[test]
    fn publishes_the_imu_sample() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 5_000_000;
        c.received_host_ns = 1_000_000_000;
        c.imu_accel = [0.1, 9.8, -0.2];
        c.imu_gyro = [0.01, 0.0, -0.02];
        let out = h.update(&c, 1_000_000_000);
        assert_eq!(
            (out.imu_accel, out.imu_gyro),
            ([0.1, 9.8, -0.2], [0.01, 0.0, -0.02])
        );
    }

    #[test]
    fn position_hold_window() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 2_000_000_000;
        c.received_host_ns = 9_000_000_000;
        c.last_pose_fix_device_ns = c.device_time_ns - 400_000_000;
        assert!(h.update(&c, 9_000_000_000).flags & FLAG_POSITION != 0);
        c.device_time_ns += 2_000_000;
        c.received_host_ns += 2_000_000;
        c.last_pose_fix_device_ns = c.device_time_ns - 600_000_000;
        let out = h.update(&c, c.received_host_ns);
        assert_eq!(out.flags & FLAG_POSITION, 0);
        assert!(out.flags & FLAG_CONNECTED != 0);
    }

    #[test]
    fn buttons_per_hand() {
        let mut r = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut l = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 1;
        c.received_host_ns = 1;
        c.buttons = 0x1 | 0x4;
        assert_eq!(r.update(&c, 1).buttons, BTN_A | BTN_SYSTEM);
        c.buttons = 0x2 | 0x8;
        assert_eq!(l.update(&c, 1).buttons, BTN_Y | BTN_STICK);
    }

    #[test]
    fn touch_hysteresis() {
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 1;
        c.received_host_ns = 1;
        for (value, expect) in [(0.5f32, false), (0.65, true), (0.5, true), (0.39, false)] {
            c.device_time_ns += 2_000_000;
            c.received_host_ns += 2_000_000;
            c.cap[CAP_AX] = value;
            let out = h.update(&c, c.received_host_ns);
            assert_eq!(out.touches & TOUCH_X != 0, expect, "cap {value}");
        }
    }

    #[test]
    fn battery_comes_from_adc_channel_0x21() {
        let mut d = [0u8; 64];
        d[0] = 0x0c;
        // message 0: left Touch, channel 0x21 = 87
        d[3] = 0x1c;
        d[3 + 2] = 2;
        d[3 + 25] = 0x21;
        d[3 + 26] = 87;
        // message 1: right Touch, a capacitive channel
        d[31] = 0x1c;
        d[31 + 2] = 3;
        d[31 + 25] = 0x08;
        d[31 + 26] = 50;
        assert_eq!(battery_from_radio(&d), vec![(HAND_LEFT, 87)]);
        d[31 + 25] = 0x21;
        assert_eq!(
            battery_from_radio(&d),
            vec![(HAND_LEFT, 87), (HAND_RIGHT, 50)]
        );
        d[0] = 0x0b;
        assert!(battery_from_radio(&d).is_empty());
    }

    #[test]
    fn sample_time_mapping() {
        let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut c = input();
        c.device_time_ns = 5_000_000_000;
        c.received_host_ns = 6_000_000_000;
        c.last_pose_fix_device_ns = 4_990_000_000;
        let out = h.update(&c, 6_000_000_000);
        assert_eq!(out.sample_host_ns, 6_000_000_000);
        assert_eq!(out.last_position_fix_host_ns, 5_990_000_000);
        assert_eq!(
            (out.device_time_ns, out.received_host_ns),
            (5_000_000_000, 6_000_000_000)
        );
    }
    #[test]
    fn a_sample_stamped_before_the_newest_is_no_gap() {
        let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut c = input();
        let host = 5_000_000_000;
        let first = live_sample(&mut h, &mut c, host);
        // the other reader: its own device clock, and an earlier stamp
        let mut other = input();
        other.device_time_ns = 900_000_000;
        other.received_host_ns = host - 1_000_000;
        other.buttons = 0x1;
        let reread = h.update(&other, host + 500_000);
        assert_eq!(reread.flags, FLAG_CALIBRATED | FLAG_CONNECTED);
        assert_eq!(h.take_event(), None);
        assert_eq!(reread.buttons, BTN_A);
        assert_eq!(
            (reread.received_host_ns, reread.sample_host_ns),
            (first.received_host_ns, first.sample_host_ns)
        );
        other.device_time_ns += 2_000_000;
        other.received_host_ns = host + 1_000_000;
        let next = h.update(&other, host + 1_000_000);
        assert_eq!(next.flags, FLAG_CALIBRATED | FLAG_CONNECTED);
        assert_eq!(next.received_host_ns, host + 1_000_000);
        assert_eq!(h.take_event(), None);
        // the connect timeout still runs from the newest stamp
        let late = h.update(&other, host + 101_000_001);
        assert_eq!(late.flags & FLAG_RADIO_GAP, FLAG_RADIO_GAP);
    }

    #[test]
    fn a_gap_keeps_the_cause_it_began_with() {
        let host = 5_000_000_000;
        for cause in [GapCause::Reopen, GapCause::Stall] {
            let mut h = Hand::new(HAND_LEFT, HandConfig::default());
            let mut c = input();
            c.trigger = 1.0;
            live_sample(&mut h, &mut c, host);
            let held = h.unavailable(host + 20_000_000, cause);
            assert_eq!(held.flags & FLAG_RADIO_GAP, FLAG_RADIO_GAP);
            assert_eq!(held.trigger, 1.0);
            assert!(
                matches!(h.take_event(), Some(GapEvent::Began { cause: began, .. }) if began == cause)
            );
            // a radio-late sample during it is the same gap
            h.update(&c, host + 3_000_000_000);
            assert_eq!(h.take_event(), None);
            live_sample(&mut h, &mut c, host + 7_000_000_000);
            assert_eq!(
                h.take_event(),
                Some(GapEvent::Ended {
                    last_sample_host_ns: host,
                    silent_ns: 7_000_000_000,
                    cause,
                    asleep: false,
                    uncertain: false,
                    recordable: false,
                    capture_requested: false,
                })
            );
            h.unavailable(host + 7_200_000_000, GapCause::Radio);
            assert!(matches!(
                h.take_event(),
                Some(GapEvent::Began {
                    cause: GapCause::Radio,
                    ..
                })
            ));
            h.unavailable(host + 17_000_000_000, cause);
            assert_eq!(
                h.take_event(),
                Some(GapEvent::Expired {
                    silent_ns: 10_000_000_000,
                    cause: GapCause::Radio,
                    asleep: false,
                })
            );
        }
        let mut h = Hand::new(HAND_LEFT, HandConfig::default());
        let mut c = input();
        live_sample(&mut h, &mut c, host);
        h.update(&c, host + 200_000_000);
        assert!(matches!(
            h.take_event(),
            Some(GapEvent::Began {
                cause: GapCause::Radio,
                ..
            })
        ));
        h.unavailable(host + 300_000_000, GapCause::Reopen);
        assert_eq!(h.take_event(), None);
        live_sample(&mut h, &mut c, host + 7_000_000_000);
        assert!(matches!(
            h.take_event(),
            Some(GapEvent::Ended {
                cause: GapCause::Radio,
                ..
            })
        ));
    }

    #[test]
    fn a_controller_without_a_sample_yet_keeps_the_gap_as_it_was() {
        let mut h = Hand::new(HAND_RIGHT, HandConfig::default());
        let mut c = input();
        c.cap = [1.0; 5];
        let host = 5_000_000_000;
        live_sample(&mut h, &mut c, host);
        h.unavailable(host + 20_000_000, GapCause::Reopen);
        assert!(h.take_event().is_some());
        let empty = input();
        let held = h.update(&empty, host + 6_000_000_000);
        assert_eq!(held.flags & FLAG_RADIO_GAP, FLAG_RADIO_GAP);
        assert_eq!(h.take_event(), None);
        // in the hysteresis band: the touches stay as they were before the gap
        c.cap = [0.5; 5];
        let back = live_sample(&mut h, &mut c, host + 7_000_000_000);
        assert!(reporting(back.flags));
        assert_eq!(
            back.touches,
            TOUCH_A | TOUCH_B | TOUCH_TRIGGER | TOUCH_STICK | TOUCH_THUMBREST
        );
    }
}
