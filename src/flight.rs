// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The radio's recent traffic, kept in memory so that a controller dropping out while in use can
//! be looked at afterwards: what the radio delivered from `BEFORE_NS` before the controller's last
//! report to `AFTER_NS` after it, and the haptic commands sent meanwhile, go to a recording
//! (`dropout-<unix s>-<hand>.rec` in the state directory, the newest `KEEP_FILES` kept) in the
//! format `recorder` writes.
//!
//! Noting a record copies it into a slot of a fixed ring (no allocation); a thread that would wait
//! for the ring, while a recording is being copied out of it, leaves the record out instead, and
//! marking a dropout takes no lock: the radio thread never waits on a recording.

use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;

use crate::recorder;

pub const BEFORE_NS: u64 = 20_000_000_000;
/// Covers the ride-through: whether the controller came back, and how.
pub const AFTER_NS: u64 = 10_000_000_000;
pub const KEEP_FILES: usize = 10;
/// A gap shorter than this that ends is no dropout worth a recording of its own (a longer one's
/// recording shows it, from the 20 s before).
pub const SHORTEST_NS: u64 = 500_000_000;
/// Some 65.5 s of the radio at the round rate below.
const SLOTS: usize = 32_768;
/// A round figure just above the measured sustained rates: six saved dropouts held 441 to 492
/// a second over their 30 s. Their busiest second reached 501, one slot past this figure.
const RADIO_REPORTS_PER_S: u64 = 500;
/// How far back the ring reaches while the radio's reports fill it: 65.5 s.
const RING_SPAN_NS: u64 = SLOTS as u64 * 1_000_000_000 / RADIO_REPORTS_PER_S;
/// The latest after a controller's last report that its dropout can still be recorded: 35.5 s.
/// A window asked for later is due at once, so the ring must then still reach back `BEFORE_NS`
/// before that report. This keeps `AFTER_NS` of the ring's reach to spare, for the haptic
/// commands it also keeps (nearly 90 a second on top of the radio's 500) and the save tick.
pub const LATEST_NS: u64 = RING_SPAN_NS - BEFORE_NS - AFTER_NS;
/// The largest payload kept: a radio report (u32 source, 64 bytes).
const PAYLOAD: usize = 68;
const PREFIX: &str = "dropout-";

#[derive(Clone, Copy)]
struct Slot {
    host_ns: u64,
    rtype: u16,
    len: u8,
    data: [u8; PAYLOAD],
}

const EMPTY: Slot = Slot {
    host_ns: 0,
    rtype: 0,
    len: 0,
    data: [0; PAYLOAD],
};

struct Ring {
    slots: Vec<Slot>,
    next: usize,
}

#[derive(Default)]
struct Skip {
    count: AtomicU64,
    host_ns: AtomicU64,
}

impl Skip {
    fn write(&self, count: u64, host_ns: u64, unavailable_through: &AtomicU64) {
        if let Some(previous) = self.try_begin(count) {
            if previous != 0 {
                // The claim owns this timestamp until finish. Publish its loss before the new
                // tag, so a snapshot cannot mistake overwritten in-window history for zero.
                unavailable_through
                    .fetch_max(self.host_ns.load(Ordering::SeqCst), Ordering::SeqCst);
            }
            self.finish(count, host_ns);
        } else {
            unavailable_through.fetch_max(host_ns, Ordering::SeqCst);
        }
    }

    fn try_begin(&self, count: u64) -> Option<u64> {
        // A writer owns the entry until its timestamp is published (u64::MAX). Do not wait
        // for it, or let a delayed older producer replace a newer completed entry.
        let previous = self.count.load(Ordering::SeqCst);
        if previous >= count {
            return None;
        }
        self.count
            .compare_exchange(previous, u64::MAX, Ordering::SeqCst, Ordering::Relaxed)
            .ok()
    }

    fn finish(&self, count: u64, host_ns: u64) {
        for step in FINISH {
            step(self, count, host_ns);
        }
    }

    fn read(&self, count: u64) -> Option<u64> {
        if self.count.load(Ordering::SeqCst) != count {
            return None;
        }
        let host_ns = self.host_ns.load(Ordering::SeqCst);
        (self.count.load(Ordering::SeqCst) == count).then_some(host_ns)
    }
}

type FinishStep = fn(&Skip, u64, u64);

/// A claimed entry's publication, one atomic store each, in order: its timestamp, then the tag
/// that makes it readable. A `read` between the two still sees the claim, never an older time
/// under the new tag. The interleaving model runs these same steps one at a time.
const FINISH: [FinishStep; 2] = [
    |skip, _, host_ns| skip.host_ns.store(host_ns, Ordering::SeqCst),
    |skip, count, _| skip.count.store(count, Ordering::SeqCst),
];

pub struct FlightRecorder {
    ring: Mutex<Ring>,
    /// Records omitted because the ring was busy, cumulative since this recorder started.
    skipped_records: AtomicU64,
    /// Timestamped losses, with the same capacity as the traffic ring. Admission order need
    /// not be timestamp order: radio and haptic producers can reach this ring at different times.
    skips: Vec<Skip>,
    /// Latest timestamp whose detailed skip entry was discarded. Older windows may be uncertain;
    /// a window starting after this bound can still be known complete after history wraps.
    unavailable_through: AtomicU64,
    /// Producers from their registration, before they read the clock for a record's time, until
    /// the record is in the ring or its skip is published. Even a count outside the retained
    /// sequence range cannot be assumed old while its timestamp is pending.
    record_writers: AtomicU64,
    /// Per hand: the last report of a controller that dropped out, until it is recorded; 0: none.
    pending: [AtomicU64; 2],
    /// Published before `pending` by each hand's sole current publisher.
    began_in_use: [AtomicBool; 2],
}

/// One dropout's recording, taken out of the ring.
#[derive(Debug, Clone, PartialEq)]
pub struct Dropout {
    pub hand: usize,
    /// The controller's last report before it went silent.
    pub last_report_ns: u64,
    /// Gaps that began in use take priority when the writer falls behind.
    pub began_in_use: bool,
    /// Contention losses in the saved window, including its tail through the ring snapshot.
    /// UNKNOWN_FLIGHT_SKIPS means relevant timestamp history is unavailable.
    pub skipped_records: u64,
    /// (type, host ns, payload) in time order.
    pub records: Vec<(u16, u64, Vec<u8>)>,
}

/// One record's producer: registered before it read the clock for the record's time (`stamp`),
/// retired when dropped, after `note` put the record in the ring or published its skip.
pub struct Producer<'a> {
    flight: &'a FlightRecorder,
}

impl Producer<'_> {
    /// Keeps the record (`parts` concatenated: at most a radio report's 68 bytes).
    pub fn note(self, rtype: u16, host_ns: u64, parts: &[&[u8]]) {
        self.note_with(rtype, host_ns, parts, || self.flight.ring.try_lock().ok());
    }

    // Share the lock attempt with deterministic tests of admission across a failed try_lock.
    // The production closure inlines to the same nonblocking lock attempt.
    fn note_with<'r>(
        &self,
        rtype: u16,
        host_ns: u64,
        parts: &[&[u8]],
        try_ring: impl FnOnce() -> Option<std::sync::MutexGuard<'r, Ring>>,
    ) {
        let len: usize = parts.iter().map(|p| p.len()).sum();
        if len > PAYLOAD {
            return;
        }
        // a recording being taken out holds the ring for a moment: skip rather than wait
        let Some(mut ring) = try_ring() else {
            let count = self.flight.begin_skip();
            self.flight.finish_skip(count, host_ns);
            return;
        };
        let i = ring.next;
        ring.next = (i + 1) % SLOTS;
        let slot = &mut ring.slots[i];
        slot.host_ns = host_ns;
        slot.rtype = rtype;
        slot.len = len as u8;
        let mut at = 0;
        for p in parts {
            slot.data[at..at + p.len()].copy_from_slice(p);
            at += p.len();
        }
    }
}

impl Drop for Producer<'_> {
    fn drop(&mut self) {
        self.flight.record_writers.fetch_sub(1, Ordering::SeqCst);
    }
}

/// Counts a captured set of skip timestamps from `from` to `to`, the saved window's records.
/// Sequence numbers locate entries, not time order.
fn skips_in_window(
    from: u64,
    to: u64,
    total: u64,
    entries: &[(u64, u64)],
    pending: bool,
    unavailable_through: u64,
) -> u64 {
    let prefix = total.saturating_sub(SLOTS as u64);
    let missing_entries = entries.len() as u64 != total - prefix;
    if pending || missing_entries || (prefix != 0 && unavailable_through >= from) {
        recorder::UNKNOWN_FLIGHT_SKIPS
    } else {
        entries
            .iter()
            .filter(|(_, host)| (from..=to).contains(host))
            .count() as u64
    }
}

// Exact-or-unknown invariant. Every producer registers before it reads the clock, so one whose
// record falls in a window the save tick has reached registered before the ring copy. The writer
// count is read with the copy, under the ring lock and after the total: a producer registered
// then may have its record in neither, so the count is unknown. Every other one put its record in
// the ring before the copy, or took its skip sequence before the writer read. One that took it
// after the total read changed the total: revalidate it after the scan, so that even a fully
// retired producer makes this snapshot unknown. A count must never be a lower estimate.
fn skip_snapshot_changed(captured: u64, writers: u64, current: u64) -> bool {
    writers != 0 || captured != current
}

#[cfg(test)]
#[path = "flight_model.rs"]
mod interleavings;

impl Default for FlightRecorder {
    fn default() -> Self {
        Self::new()
    }
}

impl FlightRecorder {
    pub fn new() -> Self {
        Self {
            ring: Mutex::new(Ring {
                slots: vec![EMPTY; SLOTS],
                next: 0,
            }),
            skipped_records: AtomicU64::new(0),
            skips: (0..SLOTS).map(|_| Skip::default()).collect(),
            unavailable_through: AtomicU64::new(0),
            record_writers: AtomicU64::new(0),
            pending: [AtomicU64::new(0), AtomicU64::new(0)],
            began_in_use: [AtomicBool::new(false), AtomicBool::new(false)],
        }
    }

    /// Registers a producer, then reads `clock` for its record's time. Until the producer is
    /// dropped, after noting the record, a recording taken out cannot count its window's skips as
    /// complete: the record may be in neither its copy of the ring nor its skip total yet.
    pub fn stamp(&self, clock: impl FnOnce() -> u64) -> (Producer<'_>, u64) {
        self.record_writers.fetch_add(1, Ordering::SeqCst);
        (Producer { flight: self }, clock())
    }

    /// A record whose time no clock read gave, for tests.
    #[cfg(test)]
    fn note(&self, rtype: u16, host_ns: u64, parts: &[&[u8]]) {
        self.stamp(|| host_ns).0.note(rtype, host_ns, parts);
    }

    fn begin_skip(&self) -> u64 {
        self.skipped_records.fetch_add(1, Ordering::SeqCst) + 1
    }

    fn finish_skip(&self, count: u64, host_ns: u64) {
        self.skips[(count as usize - 1) % SLOTS].write(count, host_ns, &self.unavailable_through);
    }

    /// Request a window for this hand. True: accepted, or covered by the pending window.
    /// False: an older window stops before this gap; the caller must retain and retry it.
    /// Each hand has one publisher, serialized by the daemon's existing publication lock.
    pub fn dropout(&self, hand: usize, last_report_ns: u64, began_in_use: bool) -> bool {
        let pending = self.pending[hand].load(Ordering::Acquire);
        if pending != 0 {
            return (pending..=pending.saturating_add(AFTER_NS)).contains(&last_report_ns);
        }
        self.began_in_use[hand].store(began_in_use, Ordering::Relaxed);
        self.pending[hand]
            .compare_exchange(0, last_report_ns, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
    }

    /// The dropout whose last report was at `last_report_ns` was only a blip: not recorded, unless
    /// an earlier one of the same hand is already waiting.
    pub fn cancel(&self, hand: usize, last_report_ns: u64) {
        let _ = self.pending[hand].compare_exchange(
            last_report_ns,
            0,
            Ordering::AcqRel,
            Ordering::Acquire,
        );
    }

    /// The recordings due at `now`, taken out of the ring: the ring is copied under its lock and
    /// sorted through after.
    pub fn due(&self, now: u64) -> Vec<Dropout> {
        let due: Vec<(usize, u64)> = (0..2)
            .filter_map(|hand| {
                let last = self.pending[hand].load(Ordering::Acquire);
                (last != 0 && now >= last.saturating_add(AFTER_NS)).then_some((hand, last))
            })
            .collect();
        if due.is_empty() {
            return Vec::new();
        }
        self.take(due, self.snapshot())
    }

    /// The ring, then its skip total and its registered producers, all read under its lock
    /// (see skip_snapshot_changed).
    fn snapshot(&self) -> Snapshot {
        let mut copy = vec![EMPTY; SLOTS];
        let ring = self.ring.lock().unwrap();
        copy.copy_from_slice(&ring.slots);
        Snapshot {
            copy,
            total: self.skipped_records.load(Ordering::SeqCst),
            writers: self.record_writers.load(Ordering::SeqCst),
        }
    }

    /// The windows ending at the `due` (hand, last report) pairs, out of `snapshot`.
    fn take(&self, due: Vec<(usize, u64)>, snapshot: Snapshot) -> Vec<Dropout> {
        let Snapshot {
            copy,
            total: skipped_total,
            writers,
        } = snapshot;
        let skipped_prefix = skipped_total.saturating_sub(SLOTS as u64);
        let skips: Vec<_> = (skipped_prefix + 1..=skipped_total)
            .filter_map(|count| {
                self.skips[(count as usize - 1) % SLOTS]
                    .read(count)
                    .map(|host_ns| (count, host_ns))
            })
            .collect();
        // Read after the entries. A writer publishes the discarded timestamp bound before its
        // new tag, and one still registered at the copy has already made the count unknown;
        // overlapping writes can only make it unknown.
        let pending_timestamps = skip_snapshot_changed(
            skipped_total,
            writers,
            self.skipped_records.load(Ordering::SeqCst),
        );
        let unavailable_through = self.unavailable_through.load(Ordering::SeqCst);
        due.into_iter()
            .filter_map(|(hand, last)| {
                let began_in_use = self.began_in_use[hand].load(Ordering::Relaxed);
                // A blip may have been cancelled (and another gap begun) while the ring copied.
                if self.pending[hand]
                    .compare_exchange(last, 0, Ordering::AcqRel, Ordering::Acquire)
                    .is_err()
                {
                    return None;
                }
                let (from, to) = (
                    last.saturating_sub(BEFORE_NS),
                    last.saturating_add(AFTER_NS),
                );
                let mut records = Vec::new();
                for s in copy
                    .iter()
                    .filter(|s| s.len > 0 && s.host_ns >= from && s.host_ns <= to)
                {
                    records.push((s.rtype, s.host_ns, s.data[..s.len as usize].to_vec()));
                }
                // Count each timestamp in the window, including losses before its first retained
                // report and after its last. Sequence numbers only locate entries.
                let skipped_records = skips_in_window(
                    from,
                    to,
                    skipped_total,
                    &skips,
                    pending_timestamps,
                    unavailable_through,
                );
                records.sort_by_key(|r| r.1);
                Some(Dropout {
                    hand,
                    last_report_ns: last,
                    began_in_use,
                    skipped_records,
                    records,
                })
            })
            .collect()
    }
}

/// What one recording reads under the ring's lock.
struct Snapshot {
    copy: Vec<Slot>,
    total: u64,
    writers: u64,
}

/// The recording's bytes in `recorder`'s format, started at its first record.
pub fn encode(d: &Dropout) -> Vec<u8> {
    let start = d.records.first().map_or(d.last_report_ns, |r| r.1);
    let mut out = Vec::with_capacity(32 + d.records.len() * (16 + PAYLOAD));
    out.extend_from_slice(recorder::MAGIC);
    out.extend_from_slice(&recorder::VERSION.to_le_bytes());
    out.extend_from_slice(&32u32.to_le_bytes());
    out.extend_from_slice(&start.to_le_bytes());
    out.extend_from_slice(&d.skipped_records.to_le_bytes());
    for (rtype, host_ns, payload) in &d.records {
        out.extend_from_slice(&rtype.to_le_bytes());
        out.extend_from_slice(&0u16.to_le_bytes());
        out.extend_from_slice(&(payload.len() as u32).to_le_bytes());
        out.extend_from_slice(&host_ns.to_le_bytes());
        out.extend_from_slice(payload);
    }
    out
}

/// The unix time of a dropout recording's file name: exactly `dropout-<digits>-<left|right>.rec`.
fn recording_time(name: &str) -> Option<u64> {
    let rest = name.strip_prefix(PREFIX)?.strip_suffix(".rec")?;
    let (unix, hand) = rest.split_once('-')?;
    (matches!(hand, "left" | "right")
        && !unix.is_empty()
        && unix.bytes().all(|b| b.is_ascii_digit()))
    .then(|| unix.parse().ok())
    .flatten()
}

/// Writes `d` to `dir` as `dropout-<unix_s>-<left|right>.rec` and removes all but the newest
/// `KEEP_FILES` such recordings there (the newest by name: by time), never the one just written.
pub fn save(dir: &Path, d: &Dropout, unix_s: u64) -> io::Result<PathBuf> {
    std::fs::create_dir_all(dir)?;
    let name = format!(
        "{PREFIX}{unix_s}-{}.rec",
        if d.hand == 0 { "left" } else { "right" }
    );
    let path = dir.join(&name);
    let tmp = dir.join(format!(".{name}.tmp"));
    let written = std::fs::File::create(&tmp).and_then(|mut f| {
        f.write_all(&encode(d))?;
        f.sync_all()
    });
    if let Err(e) = written.and_then(|()| std::fs::rename(&tmp, &path)) {
        let _ = std::fs::remove_file(&tmp);
        return Err(e);
    }
    let older: Vec<PathBuf> = recordings(dir).into_iter().filter(|p| *p != path).collect();
    for old in older.iter().skip(KEEP_FILES - 1) {
        let _ = std::fs::remove_file(old);
    }
    Ok(path)
}

/// The dropout recordings in `dir`, newest first.
pub fn recordings(dir: &Path) -> Vec<PathBuf> {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut found: Vec<(u64, PathBuf)> = entries
        .filter_map(|e| e.ok())
        .filter(|e| e.file_type().is_ok_and(|t| t.is_file()))
        .filter_map(|e| Some((recording_time(&e.file_name().to_string_lossy())?, e.path())))
        .collect();
    found.sort_by(|a, b| b.cmp(a));
    found.into_iter().map(|(_, p)| p).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const S: u64 = 1_000_000_000;

    /// A registered producer that has taken its skip sequence and not yet published the skip.
    fn begin_skip(f: &FlightRecorder) -> (Producer<'_>, u64) {
        let (producer, _) = f.stamp(|| 100 * S);
        (producer, f.begin_skip())
    }

    fn report(n: u8) -> [u8; 64] {
        let mut r = [0u8; 64];
        r[0] = 0x0c;
        r[1] = n;
        r
    }

    #[test]
    fn a_failed_lock_attempt_is_admitted_before_a_skip_sequence_exists() {
        let f = FlightRecorder::new();
        f.dropout(0, 100 * S, false);
        let held = f.ring.lock().unwrap();
        let paused = std::sync::Barrier::new(2);
        let resume = std::sync::Barrier::new(2);
        std::thread::scope(|scope| {
            let writer = scope.spawn(|| {
                let (producer, at) = f.stamp(|| 90 * S);
                producer.note_with(recorder::RAW_REPORT, at, &[&[1]], || {
                    let result = f.ring.try_lock().ok();
                    assert!(result.is_none());
                    paused.wait();
                    resume.wait();
                    result
                })
            });
            paused.wait();
            drop(held);
            let d = f.due(110 * S);
            // Always release the paused producer, even when the assertion below fails.
            resume.wait();
            writer.join().unwrap();
            assert_eq!(d[0].skipped_records, recorder::UNKNOWN_FLIGHT_SKIPS);
        });
        f.dropout(0, 100 * S, false);
        assert_eq!(f.due(110 * S)[0].skipped_records, 1);
    }

    /// A report stamped in a saved window before the recording is taken, and put in the ring
    /// after it, is in neither the recording nor its skip count: the count is unknown.
    #[test]
    fn a_record_stamped_before_a_recording_and_noted_after_it_leaves_the_count_unknown() {
        let f = FlightRecorder::new();
        f.dropout(0, 100 * S, true);
        let (producer, at) = f.stamp(|| 110 * S);
        let d = f.due(110 * S);
        producer.note(recorder::RAW_REPORT, at, &[&1u32.to_le_bytes(), &report(1)]);
        assert!(d[0].records.is_empty());
        assert_eq!(d[0].skipped_records, recorder::UNKNOWN_FLIGHT_SKIPS);
        f.dropout(0, 100 * S, true);
        let d = f.due(110 * S);
        assert_eq!((d[0].records.len(), d[0].skipped_records), (1, 0));
    }

    /// A report stamped in the window before the recording's copy and put in the ring after it,
    /// its producer retired before the skip entries are scanned: the writer count read with the
    /// copy still sees it.
    #[test]
    fn a_record_put_in_the_ring_after_the_copy_leaves_the_count_unknown() {
        let f = FlightRecorder::new();
        f.dropout(0, 100 * S, true);
        let (producer, at) = f.stamp(|| 105 * S);
        let snapshot = f.snapshot();
        producer.note(recorder::RAW_REPORT, at, &[&1u32.to_le_bytes(), &report(1)]);
        assert_eq!(f.record_writers.load(Ordering::SeqCst), 0);
        let d = f.take(vec![(0, 100 * S)], snapshot);
        assert!(d[0].records.is_empty());
        assert_eq!(d[0].skipped_records, recorder::UNKNOWN_FLIGHT_SKIPS);
    }

    /// A producer is registered before its clock read: a recording taken in between already
    /// counts it as pending.
    #[test]
    fn stamp_registers_the_producer_before_it_reads_the_clock() {
        let f = FlightRecorder::new();
        f.dropout(0, 100 * S, true);
        let mut taken = Vec::new();
        let (producer, at) = f.stamp(|| {
            taken = f.due(110 * S);
            110 * S
        });
        producer.note(recorder::RAW_REPORT, at, &[&1u32.to_le_bytes(), &report(1)]);
        assert_eq!(taken[0].skipped_records, recorder::UNKNOWN_FLIGHT_SKIPS);
    }

    /// A dropout's recording holds what the radio delivered from 20 s before the controller's
    /// last report to 10 s after it, the haptic commands among it, in order; it is taken once,
    /// when the 10 s have passed.
    #[test]
    fn a_dropout_records_the_radio_around_it() {
        let f = FlightRecorder::new();
        for i in 0..400u64 {
            // one report every 100 ms from 0 to 40 s
            f.note(
                recorder::RAW_REPORT,
                i * S / 10,
                &[&1u32.to_le_bytes(), &report(i as u8)],
            );
        }
        f.note(recorder::HAPTIC, 25 * S, &[&[1, 0, 0, 0], &[7; 12]]);
        f.dropout(1, 25 * S, true);
        assert!(f.due(35 * S - 1).is_empty());
        let d = f.due(35 * S);
        assert_eq!(d.len(), 1);
        let d = &d[0];
        assert_eq!((d.hand, d.last_report_ns), (1, 25 * S));
        let times: Vec<u64> = d.records.iter().map(|r| r.1).collect();
        assert_eq!(times.first(), Some(&(5 * S)));
        assert_eq!(times.last(), Some(&(35 * S)));
        assert!(times.windows(2).all(|w| w[0] <= w[1]));
        assert_eq!(d.records.len(), 301 + 1);
        let haptic = d.records.iter().find(|r| r.0 == recorder::HAPTIC).unwrap();
        assert_eq!(haptic.2.len(), 16);
        let radio = d
            .records
            .iter()
            .find(|r| r.1 == 25 * S && r.0 == recorder::RAW_REPORT)
            .unwrap();
        assert_eq!(radio.2[4..], report(250u64 as u8));
        assert!(f.due(100 * S).is_empty(), "taken once");
    }

    #[test]
    fn a_pending_recording_keeps_whether_the_gap_began_in_use() {
        let f = FlightRecorder::new();
        f.dropout(0, S, true);
        f.dropout(0, S, false); // Its Ended event cannot lower its priority.
        f.dropout(0, 2 * S, false); // Nor can another gap folded into its recording.
        f.dropout(1, S, false);
        f.dropout(1, 2 * S, true);
        let d = f.due(S + AFTER_NS);
        assert_eq!(d.len(), 2);
        assert!(d[0].began_in_use);
        assert!(!d[1].began_in_use);
        f.dropout(0, 20 * S, false);
        f.dropout(1, 20 * S, true);
        let d = f.due(20 * S + AFTER_NS);
        assert!(!d[0].began_in_use);
        assert!(d[1].began_in_use);
    }

    #[test]
    fn admission_before_sequence_assignment_cannot_escape_a_snapshot() {
        let f = FlightRecorder::new();
        // A producer registered before the total read takes its sequence after it.
        let (producer, _) = f.stamp(|| 100 * S);
        let captured = f.skipped_records.load(Ordering::SeqCst);
        assert_eq!(captured, 0);
        let count = f.begin_skip();
        f.finish_skip(count, 100 * S);
        drop(producer);
        // The scan used captured=0, so no entries were visited; the writer has also retired.
        let unstable = skip_snapshot_changed(
            captured,
            f.record_writers.load(Ordering::SeqCst),
            f.skipped_records.load(Ordering::SeqCst),
        );
        assert_eq!(
            skips_in_window(80 * S, 110 * S, captured, &[], unstable, 0),
            recorder::UNKNOWN_FLIGHT_SKIPS
        );
    }

    #[test]
    fn skipped_counts_cover_each_saved_window_and_its_tail() {
        let f = FlightRecorder::new();
        f.note(recorder::RAW_REPORT, S, &[&1u32.to_le_bytes(), &report(1)]);
        let ring = f.ring.lock().unwrap();
        for t in [2 * S, 3 * S, 4 * S] {
            f.note(recorder::RAW_REPORT, t, &[&1u32.to_le_bytes(), &report(2)]);
        }
        drop(ring);
        // These three losses precede both saved windows, so neither should count them.
        f.note(
            recorder::RAW_REPORT,
            25 * S,
            &[&1u32.to_le_bytes(), &report(3)],
        );
        let ring = f.ring.lock().unwrap();
        f.note(
            recorder::RAW_REPORT,
            30 * S,
            &[&1u32.to_le_bytes(), &report(4)],
        );
        drop(ring);
        f.note(
            recorder::RAW_REPORT,
            45 * S,
            &[&1u32.to_le_bytes(), &report(5)],
        );
        let ring = f.ring.lock().unwrap();
        f.note(
            recorder::RAW_REPORT,
            46 * S,
            &[&1u32.to_le_bytes(), &report(6)],
        );
        f.note(
            recorder::HAPTIC,
            54 * S,
            &[&recorder::haptic_payload(0, 0.1, 100.0, 0.5)],
        );
        // An oversized payload is not a contention loss.
        f.note(recorder::RAW_REPORT, 54 * S, &[&[0; PAYLOAD + 1]]);
        drop(ring);
        f.dropout(0, 45 * S, true);
        let due = f.due(55 * S);
        assert_eq!(due.len(), 1);
        assert_eq!(due[0].skipped_records, 3); // one inside, two after the last slot
        assert_eq!(
            due[0].records.iter().map(|r| r.1).collect::<Vec<_>>(),
            [25 * S, 45 * S]
        );
        // An overlapping window starts at the later record and counts only the two tail losses.
        f.note(
            recorder::RAW_REPORT,
            65 * S,
            &[&1u32.to_le_bytes(), &report(7)],
        );
        f.dropout(1, 65 * S, true);
        assert_eq!(f.due(75 * S)[0].skipped_records, 2);
        // A later complete window must return to zero despite the lifetime total being six.
        f.note(
            recorder::RAW_REPORT,
            80 * S,
            &[&1u32.to_le_bytes(), &report(8)],
        );
        f.note(
            recorder::RAW_REPORT,
            100 * S,
            &[&1u32.to_le_bytes(), &report(9)],
        );
        f.dropout(0, 100 * S, true);
        assert_eq!(f.due(110 * S)[0].skipped_records, 0);
        assert_eq!(f.skipped_records.load(Ordering::Relaxed), 6);
    }

    #[test]
    fn an_empty_window_does_not_hide_known_losses() {
        let f = FlightRecorder::new();
        let ring = f.ring.lock().unwrap();
        f.note(recorder::RAW_REPORT, S, &[&1u32.to_le_bytes(), &report(1)]);
        f.note(
            recorder::RAW_REPORT,
            100 * S,
            &[&1u32.to_le_bytes(), &report(2)],
        );
        drop(ring);
        f.dropout(0, 100 * S, true);
        let due = f.due(110 * S);
        assert!(due[0].records.is_empty());
        assert_eq!(due[0].skipped_records, 1);
        f.dropout(0, 200 * S, true);
        assert_eq!(f.due(210 * S)[0].skipped_records, 0);
    }

    #[test]
    fn an_empty_window_counts_losses_at_both_window_boundaries() {
        let f = FlightRecorder::new();
        let ring = f.ring.lock().unwrap();
        for t in [79 * S, 80 * S, 110 * S] {
            f.note(recorder::RAW_REPORT, t, &[&1u32.to_le_bytes(), &report(1)]);
        }
        drop(ring);
        f.dropout(0, 100 * S, true);
        assert_eq!(f.due(110 * S)[0].skipped_records, 2);
    }

    /// A report stamped after the saved window, skipped while its recording is copied out, is not
    /// counted in it; one at the window's last instant is.
    #[test]
    fn a_skip_after_the_saved_window_is_not_counted_in_it() {
        let f = FlightRecorder::new();
        f.note(
            recorder::RAW_REPORT,
            100 * S,
            &[&1u32.to_le_bytes(), &report(1)],
        );
        f.dropout(0, 100 * S, true);
        let ring = f.ring.lock().unwrap();
        for t in [110 * S + 1, 110 * S] {
            f.note(recorder::RAW_REPORT, t, &[&1u32.to_le_bytes(), &report(2)]);
        }
        drop(ring);
        let due = f.due(110 * S + 1);
        assert_eq!(due[0].records.len(), 1);
        assert_eq!(due[0].skipped_records, 1);
    }

    #[test]
    fn an_empty_window_excludes_old_losses_after_the_skip_history_wraps() {
        let f = FlightRecorder::new();
        let ring = f.ring.lock().unwrap();
        for _ in 0..SLOTS + 2 {
            f.note(recorder::RAW_REPORT, S, &[&1u32.to_le_bytes(), &report(1)]);
        }
        f.note(
            recorder::RAW_REPORT,
            100 * S,
            &[&1u32.to_le_bytes(), &report(2)],
        );
        drop(ring);
        f.dropout(0, 100 * S, true);
        assert_eq!(f.due(110 * S)[0].skipped_records, 1);
    }

    #[test]
    fn losses_before_the_first_retained_record_are_counted() {
        for (lost, first) in [(80, 81), (90, 100)] {
            let f = FlightRecorder::new();
            let ring = f.ring.lock().unwrap();
            f.note(recorder::RAW_REPORT, S, &[&1u32.to_le_bytes(), &report(1)]);
            f.note(
                recorder::RAW_REPORT,
                lost * S,
                &[&1u32.to_le_bytes(), &report(2)],
            );
            drop(ring);
            f.note(
                recorder::RAW_REPORT,
                first * S,
                &[&1u32.to_le_bytes(), &report(3)],
            );
            f.dropout(0, 100 * S, true);
            let due = f.due(110 * S);
            assert_eq!(due[0].records.len(), 1);
            assert_eq!(
                due[0].skipped_records, 1,
                "lost at {lost}, first retained at {first}"
            );
        }
    }

    #[test]
    fn skip_sequence_numbers_do_not_order_producer_timestamps() {
        for retained in [false, true] {
            let f = FlightRecorder::new();
            let ring = f.ring.lock().unwrap();
            // A haptic caller stamped after the boundary gets count 1. A radio caller stamped
            // before it resumes later and gets count 2: admission order is not timestamp order.
            f.note(
                recorder::HAPTIC,
                81 * S,
                &[&recorder::haptic_payload(0, 0.1, 100.0, 0.5)],
            );
            f.note(
                recorder::RAW_REPORT,
                79 * S,
                &[&1u32.to_le_bytes(), &report(1)],
            );
            drop(ring);
            if retained {
                f.note(
                    recorder::RAW_REPORT,
                    100 * S,
                    &[&1u32.to_le_bytes(), &report(2)],
                );
            }
            f.dropout(0, 100 * S, true);
            assert_eq!(f.due(110 * S)[0].skipped_records, 1, "nonempty {retained}");
        }
    }

    #[test]
    fn overwritten_window_losses_are_unknown_until_the_window_moves_past_them() {
        let f = FlightRecorder::new();
        let ring = f.ring.lock().unwrap();
        for _ in 0..SLOTS + 2 {
            f.note(
                recorder::RAW_REPORT,
                100 * S,
                &[&1u32.to_le_bytes(), &report(1)],
            );
        }
        drop(ring);
        f.dropout(0, 100 * S, true);
        assert_eq!(f.due(110 * S)[0].skipped_records, u64::MAX);
        f.dropout(0, 200 * S, true);
        assert_eq!(f.due(210 * S)[0].skipped_records, 0);
    }

    #[test]
    fn a_pending_skip_timestamp_cannot_claim_a_complete_window() {
        let f = FlightRecorder::new();
        let _pending = begin_skip(&f);
        f.dropout(0, 100 * S, true);
        assert_eq!(f.due(110 * S)[0].skipped_records, u64::MAX);
    }

    #[test]
    fn a_pending_producer_stays_unknown_after_its_sequence_leaves_the_history() {
        let f = FlightRecorder::new();
        let (producer, delayed) = begin_skip(&f);
        let ring = f.ring.lock().unwrap();
        for _ in 0..SLOTS + 1 {
            f.note(recorder::RAW_REPORT, S, &[&1u32.to_le_bytes(), &report(1)]);
        }
        drop(ring);
        f.dropout(0, 100 * S, true);
        assert_eq!(
            f.due(110 * S)[0].skipped_records,
            recorder::UNKNOWN_FLIGHT_SKIPS
        );
        // Its rejected late write still records where unavailable history could matter.
        f.finish_skip(delayed, 100 * S);
        drop(producer);
        f.dropout(0, 100 * S, true);
        assert_eq!(
            f.due(110 * S)[0].skipped_records,
            recorder::UNKNOWN_FLIGHT_SKIPS
        );
        f.dropout(0, 200 * S, true);
        assert_eq!(f.due(210 * S)[0].skipped_records, 0);
    }

    #[test]
    fn a_skip_finishing_after_the_snapshot_scan_still_leaves_an_unknown_entry() {
        let f = FlightRecorder::new();
        let (producer, count) = begin_skip(&f);
        let total = f.skipped_records.load(Ordering::SeqCst);
        // The scan sees the producer's not-yet-published tag. It finishes before the later
        // in-flight check: zero current writers cannot fill the captured entry retroactively.
        let entries = Vec::new();
        assert_eq!(f.skips[0].read(count), None);
        f.finish_skip(count, 100 * S);
        drop(producer);
        let pending = f.record_writers.load(Ordering::SeqCst) != 0;
        assert!(!pending);
        let discarded = f.unavailable_through.load(Ordering::SeqCst);
        assert_eq!(
            skips_in_window(80 * S, 110 * S, total, &entries, pending, discarded),
            recorder::UNKNOWN_FLIGHT_SKIPS
        );
    }

    #[test]
    fn a_paused_skip_writer_cannot_mix_timestamps_across_a_wrap() {
        let skip = Skip::default();
        let second = SLOTS as u64 + 1;
        let third = 2 * SLOTS as u64 + 1;
        // The first writer pauses after claiming this slot. Another producer wraps the history.
        let unavailable = AtomicU64::new(0);
        assert_eq!(skip.try_begin(1), Some(0));
        skip.write(second, 100 * S, &unavailable);
        assert_eq!(
            skip.read(second),
            None,
            "the slot still has its first writer"
        );
        skip.finish(1, S);
        assert_eq!(skip.read(1), Some(S));
        skip.write(third, 100 * S, &unavailable);
        assert_eq!(skip.read(third), Some(100 * S));
        // A producer paused before its claim cannot overwrite a newer completed entry either.
        skip.write(second, 50 * S, &unavailable);
        assert_eq!(skip.read(third), Some(100 * S));
    }

    /// Each step of a skip's publication leaves the entry unreadable or readable with its own
    /// time: reordering them would let a reader pair the new tag with the overwritten time.
    #[test]
    fn a_skip_entry_publishes_its_time_before_its_tag() {
        let skip = Skip::default();
        skip.write(1, S, &AtomicU64::new(0));
        let next = SLOTS as u64 + 1;
        assert_eq!(skip.try_begin(next), Some(1));
        for step in FINISH {
            assert_eq!(skip.read(next), None);
            step(&skip, next, 2 * S);
        }
        assert_eq!(skip.read(next), Some(2 * S));
    }

    #[test]
    fn unknown_skip_counts_round_trip_in_the_existing_v2_header() {
        let dropout = Dropout {
            hand: 0,
            last_report_ns: S,
            began_in_use: true,
            skipped_records: recorder::UNKNOWN_FLIGHT_SKIPS,
            records: vec![(
                recorder::RAW_REPORT,
                S,
                [&1u32.to_le_bytes()[..], &report(1)].concat(),
            )],
        };
        let bytes = encode(&dropout);
        assert_eq!(u32::from_le_bytes(bytes[8..12].try_into().unwrap()), 2);
        assert_eq!(u32::from_le_bytes(bytes[12..16].try_into().unwrap()), 32);
        let path = std::env::temp_dir().join(format!(
            "cv1t-unknown-flight-skips-{}.rec",
            std::process::id()
        ));
        std::fs::write(&path, bytes).unwrap();
        let mut reader = recorder::Reader::open(&path).unwrap();
        assert_eq!(
            reader.flight_skipped_records,
            recorder::UNKNOWN_FLIGHT_SKIPS
        );
        assert_eq!(reader.next().unwrap().payload, dropout.records[0].2);
        assert!(reader.next().is_none() && reader.error().is_none());
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn skipped_counts_use_the_reserved_header_without_changing_records() {
        let d = Dropout {
            hand: 0,
            last_report_ns: S,
            began_in_use: true,
            skipped_records: 7,
            records: vec![(
                recorder::RAW_REPORT,
                S,
                [&1u32.to_le_bytes()[..], &report(1)].concat(),
            )],
        };
        let bytes = encode(&d);
        assert_eq!(u32::from_le_bytes(bytes[8..12].try_into().unwrap()), 2);
        assert_eq!(u32::from_le_bytes(bytes[12..16].try_into().unwrap()), 32);
        assert_eq!(u64::from_le_bytes(bytes[24..32].try_into().unwrap()), 7);
        let path =
            std::env::temp_dir().join(format!("cv1t-flight-header-{}.rec", std::process::id()));
        std::fs::write(&path, &bytes).unwrap();
        let mut reader = recorder::Reader::open(&path).unwrap();
        assert_eq!(reader.flight_skipped_records, 7);
        let records: Vec<_> = reader.by_ref().collect();
        assert!(reader.error().is_none());
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].payload, d.records[0].2);
        // Old files put zero in these bytes; old readers ignore them. All record offsets stay put.
        let old = encode(&Dropout {
            skipped_records: 0,
            ..d
        });
        assert_eq!(&old[..24], &bytes[..24]);
        assert_eq!(&old[32..], &bytes[32..]);
        std::fs::write(&path, old).unwrap();
        let mut reader = recorder::Reader::open(&path).unwrap();
        assert_eq!(reader.flight_skipped_records, 0);
        assert_eq!(reader.by_ref().collect::<Vec<_>>(), records);
        assert!(reader.error().is_none());
        std::fs::remove_file(path).unwrap();
    }

    /// A blip's cancel takes back its own dropout only, not an earlier one still waiting.
    #[test]
    fn a_cancel_takes_back_only_its_own_dropout() {
        let f = FlightRecorder::new();
        f.dropout(0, 5 * S, true);
        f.dropout(0, 6 * S, true);
        f.cancel(0, 6 * S);
        assert_eq!(f.due(20 * S)[0].last_report_ns, 5 * S);
        f.dropout(1, 7 * S, true);
        f.cancel(1, 7 * S);
        assert!(f.due(20 * S).is_empty());
    }

    /// The ring keeps the newest records: the oldest are overwritten, never the other way.
    #[test]
    fn the_ring_keeps_the_newest() {
        let f = FlightRecorder::new();
        let n = SLOTS as u64 + 100;
        for i in 0..n {
            f.note(
                recorder::RAW_REPORT,
                i * 1_000_000,
                &[&1u32.to_le_bytes(), &report(0)],
            );
        }
        let last = (n - 1) * 1_000_000;
        f.dropout(0, last, true);
        let d = f.due(last + AFTER_NS);
        let times: Vec<u64> = d[0].records.iter().map(|r| r.1).collect();
        assert_eq!(times.len(), (BEFORE_NS / 1_000_000 + 1) as usize);
        assert_eq!(times.last(), Some(&last));
        // too large to keep
        f.note(recorder::RAW_REPORT, last, &[&[0u8; 69]]);
        f.dropout(0, last, true);
        assert_eq!(f.due(last + AFTER_NS)[0].records.len(), times.len());
    }

    /// A dropout asked for as late as `LATEST_NS` after its last report is taken at the next save
    /// tick with its whole window still in the ring, even with 85 haptic commands a second
    /// sharing the ring with the radio's 500 reports.
    #[test]
    fn a_window_asked_for_at_the_latest_is_still_whole() {
        const MS: u64 = 1_000_000;
        let f = FlightRecorder::new();
        let last = 30 * S;
        let taken = last + LATEST_NS + 200 * MS;
        let mut haptic = 0;
        for t in (0..=taken).step_by(2 * MS as usize) {
            f.note(recorder::RAW_REPORT, t, &[&1u32.to_le_bytes(), &report(1)]);
            if t * 85 / S != haptic {
                haptic = t * 85 / S;
                f.note(
                    recorder::HAPTIC,
                    t,
                    &[&recorder::haptic_payload(0, 0.1, 100.0, 0.5)],
                );
            }
        }
        f.dropout(0, last, false);
        let d = f.due(taken);
        let times: Vec<u64> = d[0].records.iter().map(|r| r.1).collect();
        assert_eq!(times.first(), Some(&(last - BEFORE_NS)));
        assert_eq!(times.last(), Some(&(last + AFTER_NS)));
    }

    /// A recording is what `recorder::Reader` reads back, and the directory keeps the newest ten.
    #[test]
    fn saved_recordings_read_back_and_the_newest_are_kept() {
        let dir = std::env::temp_dir().join(format!("cv1t-flight-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let d = Dropout {
            hand: 1,
            last_report_ns: 5 * S,
            began_in_use: true,
            skipped_records: 0,
            records: vec![
                (
                    recorder::RAW_REPORT,
                    4 * S,
                    [&1u32.to_le_bytes()[..], &report(3)].concat(),
                ),
                (
                    recorder::HAPTIC,
                    5 * S,
                    vec![1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12],
                ),
            ],
        };
        let path = save(&dir, &d, 1_000).unwrap();
        assert_eq!(path.file_name().unwrap(), "dropout-1000-right.rec");
        let mut r = recorder::Reader::open(&path).unwrap();
        let first = r.next().unwrap();
        assert_eq!((first.rtype, first.host_ns), (recorder::RAW_REPORT, 4 * S));
        assert_eq!(
            recorder::decode_raw(&first.payload),
            Some((1, &report(3)[..]))
        );
        let second = r.next().unwrap();
        assert_eq!((second.rtype, second.payload.len()), (recorder::HAPTIC, 16));
        assert!(r.next().is_none() && r.error().is_none());

        // files that are not dropout recordings, whatever their names start with
        let others = [
            "dropout-notes.txt",
            "dropout-1500-right-frames.rec",
            "dropout-1x-left.rec",
            "dropout--left.rec",
            "dropout-+1500-left.rec",
        ];
        for o in others {
            std::fs::write(dir.join(o), b"kept").unwrap();
        }
        for t in 1_001..1_012 {
            save(
                &dir,
                &Dropout {
                    hand: 0,
                    ..d.clone()
                },
                t,
            )
            .unwrap();
        }
        let left = recordings(&dir);
        assert_eq!(left.len(), KEEP_FILES);
        assert_eq!(left[0].file_name().unwrap(), "dropout-1011-left.rec");
        assert_eq!(
            left[KEEP_FILES - 1].file_name().unwrap(),
            "dropout-1002-left.rec"
        );
        for o in others {
            assert!(dir.join(o).exists(), "{o}");
        }
        // the clock stepped back: the recording just made is kept, the oldest other goes
        let earlier = save(
            &dir,
            &Dropout {
                hand: 0,
                ..d.clone()
            },
            900,
        )
        .unwrap();
        assert!(earlier.exists());
        assert!(!dir.join("dropout-1002-left.rec").exists());
        assert_eq!(recordings(&dir).len(), KEEP_FILES);
        // a recording that cannot be put in place leaves nothing behind
        std::fs::create_dir(dir.join("dropout-2000-left.rec")).unwrap();
        std::fs::write(dir.join("dropout-2000-left.rec/x"), b"").unwrap();
        assert!(save(
            &dir,
            &Dropout {
                hand: 0,
                ..d.clone()
            },
            2_000
        )
        .is_err());
        assert!(!dir.join(".dropout-2000-left.rec.tmp").exists());
        // a directory of that name is no recording
        assert!(!recordings(&dir).contains(&dir.join("dropout-2000-left.rec")));
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
