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
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;

use crate::recorder;

pub const BEFORE_NS: u64 = 20_000_000_000;
/// Covers the ride-through: whether the controller came back, and how.
pub const AFTER_NS: u64 = 10_000_000_000;
pub const KEEP_FILES: usize = 10;
/// A gap shorter than this that ends is no dropout worth a recording of its own (a longer one's
/// recording shows it, from the 20 s before).
pub const SHORTEST_NS: u64 = 500_000_000;
/// Some 66 s of the radio's 490 reports a second.
const SLOTS: usize = 32_768;
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

pub struct FlightRecorder {
    ring: Mutex<Ring>,
    /// Per hand: the last report of a controller that dropped out, until it is recorded; 0: none.
    pending: [AtomicU64; 2],
}

/// One dropout's recording, taken out of the ring.
#[derive(Debug, Clone, PartialEq)]
pub struct Dropout {
    pub hand: usize,
    /// The controller's last report before it went silent.
    pub last_report_ns: u64,
    /// (type, host ns, payload) in time order.
    pub records: Vec<(u16, u64, Vec<u8>)>,
}

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
            pending: [AtomicU64::new(0), AtomicU64::new(0)],
        }
    }

    /// Keeps one record (`parts` concatenated: at most a radio report's 68 bytes).
    pub fn note(&self, rtype: u16, host_ns: u64, parts: &[&[u8]]) {
        let len: usize = parts.iter().map(|p| p.len()).sum();
        if len > PAYLOAD {
            return;
        }
        // a recording being taken out holds the ring for a moment: skip rather than wait
        let Ok(mut ring) = self.ring.try_lock() else {
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

    /// The controller of `hand` dropped out, its last report at `last_report_ns` (not 0): its
    /// recording is taken once `AFTER_NS` have passed. A second dropout of the same hand before
    /// then is in that recording.
    pub fn dropout(&self, hand: usize, last_report_ns: u64) {
        let _ = self.pending[hand].compare_exchange(
            0,
            last_report_ns,
            Ordering::AcqRel,
            Ordering::Acquire,
        );
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
        let mut copy = vec![EMPTY; SLOTS];
        copy.copy_from_slice(&self.ring.lock().unwrap().slots);
        due.into_iter()
            .map(|(hand, last)| {
                self.pending[hand].store(0, Ordering::Release);
                let (from, to) = (
                    last.saturating_sub(BEFORE_NS),
                    last.saturating_add(AFTER_NS),
                );
                let mut records: Vec<(u16, u64, Vec<u8>)> = copy
                    .iter()
                    .filter(|s| s.len > 0 && s.host_ns >= from && s.host_ns <= to)
                    .map(|s| (s.rtype, s.host_ns, s.data[..s.len as usize].to_vec()))
                    .collect();
                records.sort_by_key(|r| r.1);
                Dropout {
                    hand,
                    last_report_ns: last,
                    records,
                }
            })
            .collect()
    }
}

/// The recording's bytes in `recorder`'s format, started at its first record.
pub fn encode(d: &Dropout) -> Vec<u8> {
    let start = d.records.first().map_or(d.last_report_ns, |r| r.1);
    let mut out = Vec::with_capacity(32 + d.records.len() * (16 + PAYLOAD));
    out.extend_from_slice(recorder::MAGIC);
    out.extend_from_slice(&recorder::VERSION.to_le_bytes());
    out.extend_from_slice(&32u32.to_le_bytes());
    out.extend_from_slice(&start.to_le_bytes());
    out.extend_from_slice(&0u64.to_le_bytes());
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

    fn report(n: u8) -> [u8; 64] {
        let mut r = [0u8; 64];
        r[0] = 0x0c;
        r[1] = n;
        r
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
        f.dropout(1, 25 * S);
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

    /// A blip's cancel takes back its own dropout only, not an earlier one still waiting.
    #[test]
    fn a_cancel_takes_back_only_its_own_dropout() {
        let f = FlightRecorder::new();
        f.dropout(0, 5 * S);
        f.dropout(0, 6 * S);
        f.cancel(0, 6 * S);
        assert_eq!(f.due(20 * S)[0].last_report_ns, 5 * S);
        f.dropout(1, 7 * S);
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
        f.dropout(0, last);
        let d = f.due(last + AFTER_NS);
        let times: Vec<u64> = d[0].records.iter().map(|r| r.1).collect();
        assert_eq!(times.len(), (BEFORE_NS / 1_000_000 + 1) as usize);
        assert_eq!(times.last(), Some(&last));
        // too large to keep
        f.note(recorder::RAW_REPORT, last, &[&[0u8; 69]]);
        f.dropout(0, last);
        assert_eq!(f.due(last + AFTER_NS)[0].records.len(), times.len());
    }

    /// A recording is what `recorder::Reader` reads back, and the directory keeps the newest ten.
    #[test]
    fn saved_recordings_read_back_and_the_newest_are_kept() {
        let dir = std::env::temp_dir().join(format!("cv1t-flight-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let d = Dropout {
            hand: 1,
            last_report_ns: 5 * S,
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
