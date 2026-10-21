// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! A protocol model: sequentially consistent atomic steps of two record producers and one
//! recording. A producer registers, reads the clock for its record's time and tries the traffic
//! ring: free, it puts the record in; held by the recording's copy, it records a skip, with its
//! sequence, claim, publication and retirement as separate steps. The recording reads the save
//! tick's clock, copies the ring and reads the skip total and the writer count under the lock,
//! scans the skip entries and revalidates the total. Only an entry's publication runs the real
//! code, `FINISH`, one store per step; no producer is an atomic action. The oracle is every
//! record timestamped in the window that the copy does not hold: the saved count must be exactly
//! that or unknown. Two fresh slots bound this model: nine steps per producer, up to 13 for the
//! recording. Actual-atomic regressions in flight.rs separately cover slot reuse, delayed
//! producers, overwritten timestamp history and the publication order itself.
use super::{skip_snapshot_changed, skips_in_window, Skip, FINISH};
use crate::recorder::UNKNOWN_FLIGHT_SKIPS;
use std::collections::HashSet;
use std::sync::atomic::{AtomicU64, Ordering};

/// The saved window, and the times producers read: in it, before it, and after the save tick's
/// clock read, which comes after the window's end.
const WINDOW: std::ops::RangeInclusive<u64> = 80..=100;
const TIMES: [u64; 2] = [90, 70];
const AFTER_THE_TICK: u64 = 110;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Step {
    Register,
    Stamp,
    TryLock,
    Sequence,
    Previous,
    Claim,
    Finish(usize),
    Retire,
}

/// Where a producer registers: before its clock read (this protocol), after it but before the
/// lock attempt (the protocol that counted a record stamped before a recording but put in the
/// ring after its copy as no loss), or only after a failed lock attempt.
#[derive(Clone, Copy)]
enum Admission {
    BeforeStamp,
    BeforeTryLock,
    AfterTryLock,
}

impl Admission {
    fn program(self) -> [Step; 9] {
        use Step::*;
        let tail = [Sequence, Previous, Claim, Finish(0), Finish(1), Retire];
        let head = match self {
            Admission::BeforeStamp => [Register, Stamp, TryLock],
            Admission::BeforeTryLock => [Stamp, Register, TryLock],
            Admission::AfterTryLock => [Stamp, TryLock, Register],
        };
        std::array::from_fn(|i| if i < 3 { head[i] } else { tail[i - 3] })
    }
}

/// Where the recording reads the writer count: under the ring lock after the total (this
/// protocol), or after its scan of the skip entries.
#[derive(Clone, Copy)]
enum WriterRead {
    UnderLock,
    AfterScan,
}

#[derive(Clone, Copy, Debug, Default, Hash, PartialEq, Eq)]
struct Producer {
    pc: u8,
    registered: bool,
    time: u64,
    in_ring: bool,
    sequence: u64,
    previous: u64,
}

#[derive(Clone, Copy, Debug, Default, Hash, PartialEq, Eq)]
struct State {
    ticked: bool,
    copying: bool,
    copied: [bool; 2],
    producers: [Producer; 2],
    writers: u64,
    total: u64,
    tags: [u64; 2],
    times: [u64; 2],
    pc: u8,
    captured: u64,
    scan: u8,
    first_tag: u64,
    timestamp: u64,
    entries: [u64; 2],
    pending: u64,
    current: u64,
}

impl State {
    fn producer(mut self, i: usize, admission: Admission) -> Option<Self> {
        let program = admission.program();
        let ticked = self.ticked;
        let copying = self.copying;
        let p = &mut self.producers[i];
        let step = *program.get(p.pc as usize)?;
        match step {
            Step::Register => {
                self.writers += 1;
                p.registered = true;
            }
            Step::Stamp => p.time = if ticked { AFTER_THE_TICK } else { TIMES[i] },
            Step::TryLock if !copying => {
                // The record goes in before or after the recording's copy, never during it.
                p.in_ring = true;
                p.pc = program.iter().position(|&s| s == Step::Retire).unwrap() as u8;
                return Some(self);
            }
            Step::TryLock => {}
            Step::Sequence => {
                self.total += 1;
                p.sequence = self.total;
            }
            Step::Previous => p.previous = self.tags[p.sequence as usize - 1],
            Step::Claim => {
                assert_eq!(self.tags[p.sequence as usize - 1], p.previous);
                self.tags[p.sequence as usize - 1] = u64::MAX;
            }
            Step::Finish(n) => {
                let slot = p.sequence as usize - 1;
                let skip = Skip {
                    count: AtomicU64::new(self.tags[slot]),
                    host_ns: AtomicU64::new(self.times[slot]),
                };
                FINISH[n](&skip, p.sequence, p.time);
                self.tags[slot] = skip.count.load(Ordering::SeqCst);
                self.times[slot] = skip.host_ns.load(Ordering::SeqCst);
            }
            Step::Retire => {
                if p.registered {
                    self.writers -= 1;
                }
            }
        }
        p.pc += 1;
        Some(self)
    }

    fn snapshot(mut self, writer_read: WriterRead) -> Option<Self> {
        match self.pc {
            // The save tick reads the clock; every later clock read is past the window's end.
            0 => self.ticked = true,
            1 => {
                self.copying = true;
                self.copied = self.producers.map(|p| p.in_ring);
            }
            2 => self.captured = self.total,
            3 => {
                if matches!(writer_read, WriterRead::UnderLock) {
                    self.pending = self.writers;
                }
            }
            4 => self.copying = false,
            5 if u64::from(self.scan / 3) < self.captured => {
                let i = (self.scan / 3) as usize;
                match self.scan % 3 {
                    0 => self.first_tag = self.tags[i],
                    1 => self.timestamp = self.times[i],
                    2 => {
                        if self.first_tag == i as u64 + 1 && self.tags[i] == self.first_tag {
                            self.entries[i] = self.timestamp;
                        }
                    }
                    _ => unreachable!(),
                }
                self.scan += 1;
                return Some(self);
            }
            5 => {
                if matches!(writer_read, WriterRead::AfterScan) {
                    self.pending = self.writers;
                }
            }
            6 => self.current = self.total,
            _ => return None,
        }
        self.pc += 1;
        Some(self)
    }

    fn check(&self) {
        let entries: Vec<_> = self
            .entries
            .iter()
            .enumerate()
            .filter(|(_, t)| **t != 0)
            .map(|(i, t)| (i as u64 + 1, *t))
            .collect();
        let result = skips_in_window(
            *WINDOW.start(),
            *WINDOW.end(),
            self.captured,
            &entries,
            skip_snapshot_changed(self.captured, self.pending, self.current),
            0,
        );
        // Every record timestamped in the window that the copy does not hold: skipped, or still
        // to be put in the ring, which the saved recording will never hold either.
        let expected = self
            .producers
            .iter()
            .zip(self.copied)
            .filter(|(p, copied)| WINDOW.contains(&p.time) && !copied)
            .count() as u64;
        assert!(
            result == UNKNOWN_FLIGHT_SKIPS || result == expected,
            "snapshot undercount or inexact count: got {result}, expected {expected}; {self:?}"
        );
    }
}

fn explore(admission: Admission, writer_read: WriterRead) {
    let mut seen = HashSet::new();
    let mut stack = vec![State::default()];
    let mut terminals = 0;
    while let Some(s) = stack.pop() {
        if !seen.insert(s) {
            continue;
        }
        if s.pc == 7 {
            s.check();
            terminals += 1;
            continue;
        }
        for i in 0..2 {
            if let Some(next) = s.producer(i, admission) {
                stack.push(next);
            }
        }
        if let Some(next) = s.snapshot(writer_read) {
            stack.push(next);
        }
    }
    eprintln!(
        "skip interleaving model: {} distinct states, {terminals} terminal snapshots",
        seen.len()
    );
}

#[test]
fn two_producers_and_a_snapshot_are_exact_or_unknown() {
    explore(Admission::BeforeStamp, WriterRead::UnderLock);
}

#[test]
#[should_panic(expected = "snapshot undercount or inexact count")]
fn the_explorer_rejects_registration_after_the_clock_read() {
    explore(Admission::BeforeTryLock, WriterRead::UnderLock);
}

#[test]
#[should_panic(expected = "snapshot undercount or inexact count")]
fn the_explorer_rejects_admission_after_a_failed_lock() {
    explore(Admission::AfterTryLock, WriterRead::UnderLock);
}

#[test]
#[should_panic(expected = "snapshot undercount or inexact count")]
fn the_explorer_rejects_a_writer_count_read_after_the_scan() {
    explore(Admission::BeforeStamp, WriterRead::AfterScan);
}
