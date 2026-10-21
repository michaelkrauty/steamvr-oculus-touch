// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! A ground-truth stream is independent of publication, reader scheduling and queue capacity.
//! The implementation may record an uncertain nap; it must never hide an oracle dropout.
//! In-use gaps request capture from their start, even without a return; known blips under 0.5 s
//! cancel only their own request. A still single-reader gap without sleep proof records a known
//! return from 0.5 s up to, but not including, 10 s; later returns are standby. A still gap spanning
//! a handoff is never a nap: it records a known return at >=0.5 s or an unknown/lost first return.
//! A still gap's return, known or lost, is standby from `flight::LATEST_NS` after its last sample,
//! when the flight ring may no longer hold its window.
//! Both readers share a host-monotonic clock; absolute anchors survive publication delays.
//! Imported use is a plain bool for three seconds from actual takeover, unaffected by
//! still samples; only receiver-local evidence establishes use after that window.
//! A native session reset preserves Radio identity and a settled nap verdict, not reader-local
//! beacons. New-reader beacons cannot prove an old gap asleep; lost activity vetoes that verdict.
//! Reopen/Stall holds never request capture and end only at a fresh accepted publication.
//! Each reader stamps a report at its own read: the same report, up to tens of ms apart, in
//! either order. Another reader's copy of a gap's last sample never ends that gap.
use cv1touch::ffi;
use cv1touch::flight::LATEST_NS;
use cv1touch::hand::{GapEvent, Hand, HandConfig, RadioActivity, RadioActivityQueue};

const MS: u64 = 1_000_000;

/// Read latencies a reader stamps its reports with: up to tens of ms, below the connect timeout.
const READ_LATENCY_MS: [u64; 6] = [0, 1, 3, 20, 60, 90];

struct Reader {
    hand: Hand,
    queue: RadioActivityQueue,
    snapshot: ffi::ohmd_cv1touch_controller,
    /// This reader stamps the report sent at trace time t, and reads its clock at t, as offset + t.
    offset: u64,
}

/// Two readers on the shared clock, each with its own read latency.
fn reader_pair(rng: &mut u64) -> [Reader; 2] {
    std::array::from_fn(|_| {
        Reader::new(
            50_000 * MS + READ_LATENCY_MS[random(rng) as usize % READ_LATENCY_MS.len()] * MS,
        )
    })
}

/// A Radio gap ends only after at least the connect timeout without a sample: an earlier end of
/// the gap's interval came from another reader's copy of its last sample.
fn copy_ended(events: &[GapEvent], gap: impl Fn(u64) -> bool) -> bool {
    events.iter().any(|e| {
        matches!(e, GapEvent::Ended {
            last_sample_host_ns,
            silent_ns,
            cause: cv1touch::hand::GapCause::Radio,
            ..
        } if gap(*last_sample_host_ns)
            && *silent_ns < HandConfig::default().connect_timeout_ns)
    })
}

impl Reader {
    fn new(offset: u64) -> Self {
        Self {
            hand: Hand::new(0, HandConfig::default()),
            queue: RadioActivityQueue::default(),
            snapshot: ffi::ohmd_cv1touch_controller {
                calibrated: true,
                ..Default::default()
            },
            offset,
        }
    }

    /// A reader started now: it has read nothing, not even a newest sample.
    fn restart(&mut self) {
        self.hand = Hand::new(0, HandConfig::default());
        self.queue = RadioActivityQueue::default();
        self.snapshot.calibrated = false;
    }

    fn data(&mut self, t: u64, moving: bool) {
        self.snapshot.calibrated = true;
        self.snapshot.device_time_ns += 2 * MS;
        self.snapshot.received_host_ns = self.offset + t;
        self.snapshot.imu_gyro = [if moving { 1.0 } else { 0.0 }, 0.0, 0.0];
        self.queue.push(RadioActivity {
            device_time_ns: self.snapshot.device_time_ns,
            received_host_ns: self.offset + t,
            asleep: false,
            moving,
        });
    }

    fn beacon(&self, t: u64) {
        self.queue.push(RadioActivity {
            device_time_ns: self.snapshot.device_time_ns,
            received_host_ns: self.offset + t,
            asleep: true,
            moving: false,
        });
    }

    fn publish(&mut self, t: u64, events: &mut Vec<GapEvent>) {
        self.queue
            .drain_events(&mut self.hand, |event| events.push(event));
        self.hand
            .classify_events(&self.snapshot, self.offset + t, |event| events.push(event));
        if !self.snapshot.calibrated {
            self.hand
                .tick_events(self.offset + t, |event| events.push(event));
        }
        self.hand.update(&self.snapshot, self.offset + t);
        events.extend(self.hand.take_event());
    }
}

fn random(seed: &mut u64) -> u64 {
    *seed ^= *seed << 13;
    *seed ^= *seed >> 7;
    *seed ^= *seed << 17;
    *seed
}

// Each trace contains arbitrary drains, handoffs before and during a gap, motion only in an
// earlier drained sample, reader-local beacon ordering, and overflow. The oracle sees actual
// samples and beacons, never the newest published snapshot or the implementation's history.
fn trace(seed: u64) -> (bool, &'static str, String) {
    let mut rng = seed + 1;
    let mut readers = reader_pair(&mut rng);
    // A takeover reads the clock after both readers read the reports before it.
    let clock = readers[0].offset.max(readers[1].offset);
    // A reader that does not publish discards what it decodes, keeping only its timing.
    let discards = random(&mut rng) & 1 != 0;
    let mut owner = (random(&mut rng) & 1) as usize;
    let mut events = Vec::new();
    let mut t = 1_000 * MS;
    let mut last_motion = None;
    let mut last_data = t;
    let mut truth_anchor = readers[owner].offset + t;
    let mut truth_spans_handoff = false;
    let mut takeover_at = 0;
    for r in &mut readers {
        r.data(t, false);
    }
    readers[owner].publish(t, &mut events);
    let moving_drain = random(&mut rng) % 3 != 0;
    let warm_overflow = random(&mut rng) % 11 == 0;
    let count = if warm_overflow {
        300
    } else {
        2 + random(&mut rng) as usize % 8
    };
    let mid_drain_handoff = random(&mut rng) % 5 == 0;
    for i in 0..count {
        t += 2 * MS;
        let moving = moving_drain && i == if warm_overflow { count - 2 } else { 0 };
        if moving {
            last_motion = Some(t);
        }
        for r in &mut readers {
            r.data(t, moving);
        }
        last_data = t;
        // A reader cannot tell its stamp of a later sample, stamped before the newest anchor,
        // from its backlog: the interval keeps the newest stamp of its last samples.
        truth_anchor = truth_anchor.max(readers[owner].offset + t);
        // A sample the new owner read before (or at) its takeover still spans the handoff.
        truth_spans_handoff = truth_anchor <= takeover_at;
        if mid_drain_handoff && i == count / 2 {
            // Both readers read this sample before the takeover: the later stamp is the anchor.
            truth_spans_handoff = true;
            takeover_at = clock + t;
            truth_anchor = takeover_at;
            readers[owner].publish(t, &mut events);
            let history = readers[owner].hand.classification_history();
            owner ^= 1;
            if discards {
                readers[owner].queue.discard();
            }
            readers[owner]
                .hand
                .inherit_classification_at(history, takeover_at);
        } else if !warm_overflow && random(&mut rng) % 4 == 0 {
            readers[owner].publish(t, &mut events);
        }
    }
    readers[owner].publish(t, &mut events);
    events.clear();
    // The latest returns lie 100 ms either side of the flight ring's limit: more than the
    // spread of the readers' stamps.
    let latest_ms = LATEST_NS / MS;
    let duration = [
        50,
        200,
        800,
        1700,
        9998,
        10000,
        10002,
        11000,
        latest_ms - 100,
        latest_ms + 100,
        120_000,
    ][random(&mut rng) as usize % 11]
        * MS;
    let beacon = random(&mut rng) & 1 != 0;
    let handoff = duration >= 200 * MS && random(&mut rng) % 3 == 0;
    let early_handoff = handoff && random(&mut rng) & 1 != 0;
    if duration >= 200 * MS && !early_handoff {
        readers[owner].publish(t + 150 * MS, &mut events);
    }
    let transfers = if handoff { 1 + random(&mut rng) % 3 } else { 0 };
    let mut previous = if early_handoff { 0 } else { 150 * MS };
    for transfer in 0..transfers {
        let handoff_at = if transfer == 0 && early_handoff {
            20 * MS + random(&mut rng) % (80 * MS)
        } else {
            previous
                + 1
                + random(&mut rng) % (duration.min(1000 * MS) - previous - (transfers - transfer))
        };
        let history = readers[owner].hand.classification_history();
        owner ^= 1;
        if early_handoff && transfer == 0 {
            // A newly started reader never decoded the outgoing publisher's last sample.
            readers[owner].restart();
        } else if discards {
            readers[owner].queue.discard();
        }
        readers[owner]
            .hand
            .inherit_classification_at(history, clock + t + handoff_at);
        previous = handoff_at;
    }
    if beacon {
        for r in &readers {
            r.beacon(t + duration / 2);
        }
    }
    // A full queue removes evidence. It must not make the remaining beacon evidence stronger.
    let overflow = random(&mut rng) % 7 == 0;
    if overflow {
        for i in 0..300 {
            readers[owner].beacon(t + duration / 2 + i);
        }
    }
    let expiry_tick = duration >= 10000 * MS && random(&mut rng) & 1 != 0;
    if expiry_tick {
        readers[owner].publish(t + 10000 * MS, &mut events);
    }
    for r in &mut readers {
        r.data(t + duration, false);
    }
    if random(&mut rng) & 1 != 0 {
        // The wake and the following beacon/sample are drained in one publication.
        for r in &readers {
            r.beacon(t + duration + MS);
        }
        for r in &mut readers {
            r.data(t + duration + 2 * MS, false);
        }
    }
    readers[owner].publish(t + duration + 2 * MS, &mut events);
    let in_use = last_motion.is_some_and(|m| last_data - m <= 3_000 * MS);
    // A handoff at the last warm-up sample still spans this interval: only a subsequent
    // actual sample clears that fact in the ground-truth stream. Ordinary loss-free standby
    // remains unrecorded; handoff-spanning returns follow the conservative rule instead.
    let spanning = truth_spans_handoff || handoff;
    let in_ring = duration < LATEST_NS;
    let oracle_dropout = duration >= 500 * MS
        && (in_use || (spanning && in_ring) || (!beacon && duration < 10000 * MS));
    // The gap's anchor is some reader's stamp of one of its last samples: a stamp later than
    // another reader's anchor advances it, an earlier one is backlog. It is the only silence.
    let gap = |at: u64| {
        (readers[0].offset.min(readers[1].offset) + last_data - 100 * MS..=clock + last_data)
            .contains(&at)
    };
    let (recorded, nap) = outcome(&events, gap);
    let latency = readers.each_ref().map(|r| (r.offset - 50_000 * MS) / MS);
    let detail = format!("seed={seed} latency_ms={latency:?} discards={discards} owner={owner} moving_drain={moving_drain} handoff={handoff} transfers={transfers} early_handoff={early_handoff} mid_drain={mid_drain_handoff} overflow={overflow} expiry_tick={expiry_tick} duration={} beacon={beacon} events={events:?}", duration / MS);
    // A reader that started during the silence cannot exclude a return before it started, so
    // it may keep a standby recording too.
    (
        (oracle_dropout && (!recorded || nap))
            || copy_ended(&events, gap)
            || (spanning && nap)
            || (!in_use
                && duration >= 10000 * MS
                && recorded
                && !overflow
                && !warm_overflow
                && !spanning)
            || (!in_use && !in_ring && recorded),
        if !in_use && duration >= 10000 * MS {
            "standby"
        } else if handoff {
            "handoff"
        } else {
            "drain"
        },
        detail,
    )
}

/// Whether a recording of the gap is kept, and whether the gap was labelled a nap. `gap` names
/// its anchor: any reader's stamp of its last sample.
fn outcome(events: &[GapEvent], gap: impl Fn(u64) -> bool) -> (bool, bool) {
    let mut captured = std::collections::HashSet::new();
    let mut current = None;
    let mut nap = false;
    for e in events {
        match *e {
            GapEvent::Began {
                last_sample_host_ns,
                moving,
                asleep,
                cause: cv1touch::hand::GapCause::Radio,
            } => {
                current = Some(last_sample_host_ns);
                if moving {
                    captured.insert(last_sample_host_ns);
                }
                nap |= gap(last_sample_host_ns) && asleep;
            }
            GapEvent::InUse {
                last_sample_host_ns,
                cause: cv1touch::hand::GapCause::Radio,
            } => {
                captured.insert(last_sample_host_ns);
            }
            GapEvent::Ended {
                last_sample_host_ns,
                recordable,
                asleep,
                capture_requested,
                ..
            } => {
                if !recordable {
                    captured.remove(&last_sample_host_ns);
                } else if !capture_requested {
                    captured.insert(last_sample_host_ns);
                }
                nap |= gap(last_sample_host_ns) && asleep;
            }
            GapEvent::Asleep => nap |= current.is_none_or(&gap),
            _ => {}
        }
    }
    (captured.into_iter().any(gap), nap)
}

fn batch_trace(seed: u64) -> (usize, usize) {
    let mut rng = seed + 1;
    let mut reader = Reader::new(50_000 * MS);
    let mut events = Vec::new();
    let mut t = 1_000 * MS;
    reader.data(t, false);
    reader.publish(t, &mut events);
    let mut motion = None;
    let mut oracle = Vec::new();
    for _ in 0..1 + random(&mut rng) % 8 {
        for _ in 0..1 + random(&mut rng) % 8 {
            t += 2 * MS;
            let moving = random(&mut rng) % 5 == 0;
            if moving {
                motion = Some(t);
            }
            reader.data(t, moving);
        }
        let duration = [50, 200, 800, 1700][random(&mut rng) as usize % 4] * MS;
        let beacon = random(&mut rng) & 1 != 0;
        let in_use = motion.is_some_and(|m| t - m <= 3000 * MS);
        if duration >= 500 * MS && (in_use || !beacon) {
            oracle.push(reader.offset + t);
        }
        if beacon {
            reader.beacon(t + duration / 2);
        }
        t += duration;
        reader.data(t, false);
        if random(&mut rng) & 1 != 0 {
            reader.beacon(t + MS);
        }
    }
    reader.publish(t + 2 * MS, &mut events);
    let recorded: Vec<_> = events
        .iter()
        .filter_map(|event| match event {
            GapEvent::Ended {
                last_sample_host_ns,
                asleep: false,
                recordable: true,
                ..
            } => Some(*last_sample_host_ns),
            _ => None,
        })
        .collect();
    for anchor in &oracle {
        assert!(
            recorded.contains(anchor),
            "seed={seed}: missing batch dropout at {anchor}; events={events:?}"
        );
        assert!(!events.iter().any(|e| matches!(e, GapEvent::Ended { last_sample_host_ns, asleep: true, .. } if last_sample_host_ns == anchor)), "seed={seed}: batch dropout labelled a nap");
    }
    (oracle.len(), recorded.len())
}

// A handoff during a still gap, after the outgoing reader's last publication. The receiving
// reader may have read the gap begin, may have started reading only during the silence, and may
// have read a return the outgoing reader missed before the handoff, followed by another still
// gap. Beacons cannot prove the handoff-spanning interval asleep; only a later interval seen
// by one reader from its start can be a nap. Returns (unsafe, spanning gap labelled nap, detail).
fn nap_handoff_trace(seed: u64) -> (bool, bool, String) {
    let mut rng = seed + 1;
    let [mut a, mut b] = reader_pair(&mut rng);
    // The receiving reader's trace time when the shared clock reads a trace time on the later one.
    let lead = a.offset.max(b.offset) - b.offset;
    let b_discards = random(&mut rng) & 1 != 0;
    let mut events = Vec::new();
    let mut t = 1_000 * MS;
    let moving = random(&mut rng) % 3 == 0;
    for i in 0..4 {
        t += 2 * MS;
        let m = moving && i == 3;
        a.data(t, m);
        b.data(t, m);
    }
    a.publish(t, &mut events);
    let d1 = [600, 800, 1700, 5000, 9998, 11000][random(&mut rng) as usize % 6] * MS;
    let missed_return = random(&mut rng) & 1 != 0;
    let fresh_reader = random(&mut rng) % 3 == 0;
    let a_beacon = random(&mut rng) & 1 != 0;
    let b_beacon = random(&mut rng) % 4 != 0;
    // The outgoing reader publishes the gap's start, maybe its own beacon, then nothing more.
    let mut last_publish = t + 150 * MS;
    a.publish(last_publish, &mut events);
    if a_beacon && d1 > 760 * MS {
        a.beacon(t + 750 * MS);
        last_publish = t + 760 * MS;
        a.publish(last_publish, &mut events);
    }
    let handoff = if missed_return {
        // A return at the end of gap 1 after the outgoing reader's last read. It hands the hand
        // over later without reading it, so only the receiving reader has it, from before.
        for i in 0..1 + random(&mut rng) % 5 {
            b.data(t + d1 + i * 2 * MS, false);
        }
        let handoff = t + d1 + 10 * MS + random(&mut rng) % (40 * MS);
        a.hand.tick_events(a.offset + handoff, |e| events.push(e));
        handoff
    } else {
        last_publish + 1 + random(&mut rng) % (d1 - (last_publish - t) - 2 * MS)
    };
    if fresh_reader {
        // A newly started reader read nothing before the silence it started in.
        b.restart();
    }
    if b_discards {
        b.queue.discard();
    }
    let history = a.hand.classification_history();
    b.hand
        .inherit_classification_at(history, b.offset + lead + handoff);
    let end = if missed_return {
        handoff + [800, 1700, 5000][random(&mut rng) as usize % 3] * MS
    } else {
        t + d1
    };
    if b_beacon {
        b.beacon(handoff + (end - handoff) / 2);
    }
    b.publish(handoff + lead + MS, &mut events);
    if end - t >= 10_000 * MS && random(&mut rng) & 1 != 0 {
        b.publish(t + 10_000 * MS, &mut events);
    }
    b.data(end, false);
    b.publish((end + MS).max(handoff + lead + 2 * MS), &mut events);
    let oracle_dropout = d1 >= 500 * MS;
    let anchor = a.offset + t;
    let (recorded, _) = outcome(&events, |at| at == anchor);
    let copy = copy_ended(&events, |at| at == anchor);
    let gap1_nap = events.iter().any(|e| matches!(e,
        GapEvent::Ended { last_sample_host_ns, asleep: true, .. } if *last_sample_host_ns == anchor
    ));
    // Even when both readers read the start and beacons, the spanning gap must never be a nap.
    let latency = [&a, &b].map(|r| (r.offset - 50_000 * MS) / MS);
    let detail = format!("seed={seed} latency_ms={latency:?} b_discards={b_discards} moving={moving} d1={} missed_return={missed_return} fresh_reader={fresh_reader} a_beacon={a_beacon} b_beacon={b_beacon} events={events:?}", d1 / MS);
    (
        copy || (oracle_dropout && (!recorded || gap1_nap)),
        gap1_nap,
        detail,
    )
}

// Import only a bool. A full receiver-local 3 s starts at takeover, independent of source
// motion age. Still samples cannot clear it; after expiry only actual receiver motion counts.
fn imported_use_trace(seed: u64) {
    let mut rng = seed + 1;
    let mut a = Reader::new(50_000 * MS);
    let mut b = Reader::new(50_000 * MS);
    let t = 5_000 * MS;
    let source_age = [2, 1000, 2999][random(&mut rng) as usize % 3] * MS;
    let mut events = Vec::new();
    a.data(t - source_age, true);
    a.publish(t - source_age, &mut events);
    a.data(t, false);
    a.publish(t, &mut events);
    b.hand
        .inherit_classification_at(a.hand.classification_history(), b.offset + t);
    b.data(t + 2 * MS, false);
    b.publish(t + 2 * MS, &mut events);
    let elapsed = [2998 * MS, 3000 * MS, 3000 * MS + 1, 4500 * MS][random(&mut rng) as usize % 4];
    let own_motion = random(&mut rng) & 1 != 0;
    if own_motion {
        b.data(t + elapsed - 2 * MS, true);
    }
    b.data(t + elapsed, false);
    b.publish(t + elapsed, &mut events);
    events.clear();
    b.beacon(t + elapsed + 2 * MS);
    b.publish(t + elapsed + 150 * MS, &mut events);
    let (recorded, nap) = outcome(&events, |at| at == b.offset + t + elapsed);
    let want_use = elapsed <= 3000 * MS || own_motion;
    assert_eq!(
        recorded, want_use,
        "seed={seed}, elapsed={elapsed}, own_motion={own_motion}: {events:?}"
    );
    assert_eq!(nap, !want_use, "seed={seed}: {events:?}");
}

// Only the receiver saw the motion or return. It discards that activity before taking over;
// lost returns can predate takeover. The outgoing reader never receives any returning sample.
fn receiver_evidence_trace(seed: u64) {
    let mut rng = seed + 1;
    let [mut a, mut b] = reader_pair(&mut rng);
    let lead = a.offset.max(b.offset) - b.offset;
    let mut events = Vec::new();
    let t = 5_000 * MS;
    let motion = random(&mut rng) & 1 != 0;
    let returns = random(&mut rng) % 3;
    let moving_return = returns != 0 && random(&mut rng) & 1 != 0;
    a.data(t, false);
    a.publish(t, &mut events);
    a.publish(t + 150 * MS, &mut events);
    if motion {
        b.data(t - 2 * MS, true);
    }
    b.data(t, false);
    b.queue.discard();
    let handoff = if returns == 2 {
        10020 * MS
    } else {
        [1000, 4000][random(&mut rng) as usize % 2] * MS
    };
    if returns != 0 {
        if returns == 2 {
            for i in 0..256 {
                b.beacon(t + 100 * MS + i);
            }
        }
        b.data(
            t + if returns == 2 { 9990 * MS } else { 600 * MS },
            moving_return,
        );
        b.queue.discard();
    }
    b.hand.inherit_classification_at(
        a.hand.classification_history(),
        b.offset + lead + t + handoff,
    );
    b.publish(lead + t + handoff, &mut events);
    // No later sample can rescue a missed promotion or missed return.
    b.beacon(lead + t + handoff + 750 * MS);
    b.publish(lead + t + handoff + 11000 * MS, &mut events);
    let latency = [&a, &b].map(|r| (r.offset - 50_000 * MS) / MS);
    assert!(
        !copy_ended(&events, |at| at == a.offset + t),
        "seed={seed} latency_ms={latency:?}: copy ended the gap: {events:?}"
    );
    if motion {
        assert!(events.iter().any(|e| matches!(e,
            GapEvent::Began { last_sample_host_ns, moving: true, .. } | GapEvent::InUse { last_sample_host_ns , cause: cv1touch::hand::GapCause::Radio,}
            if *last_sample_host_ns == a.offset + t)), "seed={seed}: pre-gap use lost to moving return: {events:?}");
    }
    let (recorded, _) = outcome(&events, |at| at == a.offset + t);
    assert!(
        recorded || (!motion && returns == 0),
        "seed={seed}: {events:?}"
    );
    assert!(
        !events.iter().any(|e| matches!(e, GapEvent::Ended {
        last_sample_host_ns, asleep: true, ..
    } if *last_sample_host_ns == a.offset + t)),
        "seed={seed}: {events:?}"
    );
}

// Classification ticks and publication need not run together. Reports, takeover and the
// oracle all use one host clock; only the publication schedule changes. Pre-takeover returns
// establish their own spanning successor, including at exactly the ownership boundary.
fn delayed_publication_trace(seed: u64) {
    let mut rng = seed + 1;
    let mut readers = [Reader::new(50_000 * MS), Reader::new(50_000 * MS)];
    let owner = (random(&mut rng) & 1) as usize;
    let (source, receiver) = if owner == 0 {
        let (a, b) = readers.split_at_mut(1);
        (&mut a[0], &mut b[0])
    } else {
        let (a, b) = readers.split_at_mut(1);
        (&mut b[0], &mut a[0])
    };
    let t = 5_000 * MS;
    let anchor = source.offset + t;
    let mut events = Vec::new();
    let moving = random(&mut rng) & 1 != 0;
    source.data(t - 2 * MS, moving);
    source.data(t, false);
    source.publish(t, &mut events);
    let tick = [100, 150][random(&mut rng) as usize % 2] * MS;
    source.publish(t + tick, &mut events);
    let pause =
        [0, 400, 1000, 11000][random(&mut rng) as usize % 4] * MS + random(&mut rng) % (20 * MS);
    let publication = t + tick + pause;
    let pre_return = random(&mut rng) & 1 != 0;
    let kind = random(&mut rng) % 3; // retained, discarded, or overflowed
    let boundary_return = random(&mut rng) % 3 == 0;
    let first = if pre_return {
        publication + random(&mut rng) % (100 * MS)
    } else {
        publication + [200, 500, 800, 9998, 10000, 10002][random(&mut rng) as usize % 6] * MS
    };
    let takeover = if pre_return {
        first
            + if boundary_return {
                0
            } else {
                1 + random(&mut rng) % (1000 * MS)
            }
    } else {
        publication
    };
    if pre_return {
        if kind == 2 {
            for i in 0..256 {
                receiver.beacon(t + i);
            }
        }
        receiver.data(first, false);
        if kind != 0 {
            receiver.queue.discard();
        }
    }
    // Nothing updates the source's classification during this publication pause.
    receiver.hand.inherit_classification_at(
        source.hand.classification_history(),
        receiver.offset + takeover,
    );
    if pre_return {
        receiver.publish(takeover, &mut events);
        assert_eq!(
            receiver
                .hand
                .classification_history()
                .gap
                .unwrap()
                .recording_anchor_ns,
            receiver.offset + first,
            "seed={seed}: wrong successor anchor"
        );
        receiver.beacon(takeover + 750 * MS);
        receiver.publish(takeover + 800 * MS, &mut events);
        receiver.data(takeover + 1700 * MS, false);
        receiver.publish(takeover + 1700 * MS, &mut events);
        let (captured, nap) = outcome(&events, |at| at == receiver.offset + first);
        assert!(
            captured && !nap,
            "seed={seed}: pre-takeover successor lost: {events:?}"
        );
    } else {
        // A beacon from the receiver cannot prove sleep in the inherited interval.
        receiver.beacon(takeover + (first - takeover) / 2);
        receiver.publish(takeover, &mut events);
        receiver.data(first, false);
        receiver.publish(first, &mut events);
    }
    let (captured, nap) = outcome(&events, |at| at == anchor);
    let oracle_capture = first - t >= 500 * MS || (pre_return && kind != 0);
    assert!(!nap && (!oracle_capture || captured),
        "seed={seed} moving={moving} pause={pause} first={first} takeover={takeover} pre={pre_return} kind={kind}: {events:?}");
}

#[test]
fn seeded_publication_and_takeover_pauses_preserve_gap_time() {
    let seeds = std::env::var("CV1TOUCH_MODEL_SEEDS").map_or(256, |n| n.parse().unwrap());
    for seed in 0..seeds {
        delayed_publication_trace(seed);
    }
    eprintln!("classification model: {seeds} delayed-publication traces passed");
}

#[test]
fn seeded_imported_use_obeys_the_receiver_local_window() {
    let seeds = std::env::var("CV1TOUCH_MODEL_SEEDS").map_or(256, |n| n.parse().unwrap());
    for seed in 0..seeds {
        imported_use_trace(seed);
    }
    eprintln!("classification model: {seeds} imported-use window traces passed");
}

#[test]
fn seeded_receiver_only_evidence_survives_delayed_takeover() {
    let seeds = std::env::var("CV1TOUCH_MODEL_SEEDS").map_or(256, |n| n.parse().unwrap());
    for seed in 0..seeds {
        receiver_evidence_trace(seed);
    }
    eprintln!("classification model: {seeds} receiver-only evidence traces passed");
}

#[test]
fn seeded_nap_handoffs_never_hide_an_oracle_dropout() {
    let seeds = std::env::var("CV1TOUCH_MODEL_SEEDS").map_or(256, |n| n.parse().unwrap());
    let mut failures = Vec::new();
    let mut handoff_naps = Vec::new();
    for seed in 0..seeds {
        let (failed, labelled_nap, detail) = nap_handoff_trace(seed);
        if failed {
            failures.push(detail);
        } else if labelled_nap {
            handoff_naps.push(detail);
        }
    }
    assert!(
        failures.is_empty() && handoff_naps.is_empty(),
        "{} / {seeds} unsafe, {} handoff naps:\n{}",
        failures.len(),
        handoff_naps.len(),
        failures
            .iter()
            .chain(&handoff_naps)
            .take(24)
            .cloned()
            .collect::<Vec<_>>()
            .join("\n")
    );
    eprintln!("classification model: {seeds} seeded nap handoff traces passed");
}

#[test]
fn seeded_classification_never_hides_an_oracle_dropout() {
    let seeds = std::env::var("CV1TOUCH_MODEL_SEEDS").map_or(256, |n| n.parse().unwrap());
    let mut failures = Vec::new();
    for seed in 0..seeds {
        let (failed, kind, detail) = trace(seed);
        if failed {
            failures.push(format!("{}: {detail}", kind));
        }
    }
    assert!(
        failures.is_empty(),
        "{} / {seeds} unsafe traces:\n{}",
        failures.len(),
        failures
            .iter()
            .take(24)
            .cloned()
            .collect::<Vec<_>>()
            .join("\n")
    );
    eprintln!("classification model: {seeds} seeded handoff/overflow traces passed");
}

#[test]
fn seeded_drains_keep_all_oracle_dropouts() {
    let seeds = std::env::var("CV1TOUCH_MODEL_SEEDS").map_or(256, |n| n.parse().unwrap());
    let mut oracle_dropouts = 0;
    let mut recorded_dropouts = 0;
    for seed in 0..seeds {
        let (oracle, recorded) = batch_trace(seed);
        oracle_dropouts += oracle;
        recorded_dropouts += recorded;
    }
    eprintln!("classification model: {seeds} seeded multi-gap batch traces passed; batch oracle={oracle_dropouts}, recorded={recorded_dropouts}");
}

// A reopen drops reader-local evidence, not the Radio identity or its capture request.
// Ground truth names each interval's cause, use and beacon/return order before publication.
// The other reader decoded the same reports at its own reads and discarded them; teardown
// classifies its timing after the publisher's, as the daemon does once both readers stop.
fn reopen_trace(seed: u64) {
    use cv1touch::hand::GapCause;
    let mut rng = seed + 1;
    let kind = random(&mut rng) % 7;
    let [mut r, mut other] = reader_pair(&mut rng);
    let anchor = r.offset + MS;
    let mut events = Vec::new();
    let moving = kind == 2;
    r.data(MS, moving);
    r.publish(MS, &mut events);
    // (trace time, motion; None: a beacon) of each report the other reader may have read
    let mut copies = vec![(MS, Some(moving))];
    let nap = matches!(kind, 0 | 4 | 6);
    if nap {
        r.beacon(3 * MS);
        copies.push((3 * MS, None));
    }
    let boundary = if matches!(kind, 3) {
        20 * MS
    } else if kind == 4 {
        106 * MS
    } else {
        (151 + random(&mut rng) % 400) * MS
    };
    if kind == 4 {
        r.publish(91 * MS, &mut events);
    } else if kind != 3 {
        r.publish(151 * MS, &mut events);
    }
    let cause = if kind == 3 {
        if random(&mut rng) & 1 == 0 {
            GapCause::Reopen
        } else {
            GapCause::Stall
        }
    } else {
        GapCause::Radio
    };
    // The boundary after the 90 ms refresh is already Radio. A return queued just before
    // teardown ends that Radio interval, but its unpublished successor is a native hold.
    let queued_return = kind == 5;
    if queued_return {
        r.data(801 * MS, false);
        copies.push((801 * MS, Some(false)));
        r.queue.drain_events(&mut r.hand, |e| events.push(e));
    }
    let stop = if queued_return { 851 * MS } else { boundary };
    let stop_cause = if queued_return {
        GapCause::Reopen
    } else {
        cause
    };
    let now = r.offset + stop;
    r.hand.unavailable(now, stop_cause);
    events.extend(r.hand.take_event());
    r.hand.preserve_reopen_sleep();
    for (at, motion) in copies {
        // Only what it read before both readers stopped.
        if other.offset + at <= now {
            match motion {
                Some(moving) => other.data(at, moving),
                None => other.beacon(at),
            }
        }
    }
    other.queue.discard();
    other
        .hand
        .inherit_classification_at(r.hand.classification_history(), now);
    other
        .queue
        .drain_events(&mut other.hand, |e| events.push(e));
    other
        .hand
        .unavailable_events(now, stop_cause, |e| events.push(e));
    r.hand
        .inherit_classification_at(other.hand.classification_history(), now);
    r.hand.reset_reader_evidence();
    r.queue.reset();
    let history = r.hand.classification_history();
    let takeover = stop + (1 + random(&mut rng) % 300) * MS;
    r.hand
        .inherit_classification_at(history, r.offset + takeover);
    if kind == 3 {
        r.publish(takeover, &mut events);
        assert_eq!(
            r.hand.classification_history().gap.unwrap().cause,
            cause,
            "cached native seed={seed}: {events:?}"
        );
    }
    let latest_ms = LATEST_NS / MS;
    let silence = [
        499,
        500,
        501,
        9_999,
        10_000,
        10_001,
        12_000,
        latest_ms - 1,
        latest_ms,
        120_000,
    ][(random(&mut rng) % 10) as usize]
        * MS;
    let wake = (MS + silence).max(takeover + MS);
    if kind == 1 {
        r.beacon(wake - MS);
    } // New-reader beacons cannot prove the old gap asleep.
    if kind == 3 || kind == 6 {
        for i in 0..256 {
            r.beacon(takeover + i);
        }
        r.data(wake, false); // Lost first return: uncertain Radio captures, native never does.
    } else {
        r.data(wake, false);
    }
    r.publish(wake, &mut events);
    // Native and Radio holds can share the last published pose's numeric anchor. A native
    // return never cancels the Radio request; recording decisions are cause-specific.
    let radio_events: Vec<_> = events
        .iter()
        .copied()
        .filter(|e| {
            !matches!(
                e,
                GapEvent::Began {
                    cause: GapCause::Reopen | GapCause::Stall,
                    ..
                } | GapEvent::Ended {
                    cause: GapCause::Reopen | GapCause::Stall,
                    ..
                } | GapEvent::InUse {
                    cause: GapCause::Reopen | GapCause::Stall,
                    ..
                } | GapEvent::Expired {
                    cause: GapCause::Reopen | GapCause::Stall,
                    ..
                }
            )
        })
        .collect();
    let (captured, labelled_nap) = outcome(&radio_events, |at| at == anchor);
    let latency = [&r, &other].map(|r| (r.offset - 50_000 * MS) / MS);
    assert!(
        !copy_ended(&events, |at| at == anchor),
        "copy seed={seed} kind={kind} latency_ms={latency:?}: {events:?}"
    );
    assert!(
        !events.iter().any(|e| matches!(
            e,
            GapEvent::Ended {
                cause: GapCause::Reopen | GapCause::Stall,
                recordable: true,
                ..
            } | GapEvent::InUse {
                cause: GapCause::Reopen | GapCause::Stall,
                ..
            }
        )),
        "native capture seed={seed}: {events:?}"
    );
    // A still gap's return, known (kind 1) or lost (kind 6), is standby once it is too late
    // for the flight ring; a gap in use (kind 2) asked for its recording when it began.
    let in_ring = wake - MS < LATEST_NS;
    let oracle_capture = (kind == 2 && wake - MS >= 500 * MS)
        || (kind == 1 && wake - MS >= 500 * MS && in_ring)
        || kind == 5
        || (kind == 6 && in_ring);
    let nap_return = events.iter().any(|e| matches!(e, GapEvent::Ended { last_sample_host_ns, asleep: true, .. } if *last_sample_host_ns == anchor));
    assert!(
        !oracle_capture || (captured && !nap_return),
        "reopen seed={seed} kind={kind} latency_ms={latency:?} stop={stop} takeover={takeover} wake={wake}: {events:?}"
    );
    if kind == 3 {
        assert!(!captured && !labelled_nap, "native seed={seed}: {events:?}");
    }
    if matches!(kind, 1 | 6) && !in_ring {
        assert!(
            !captured,
            "too late for the flight ring seed={seed} kind={kind} wake={wake}: {events:?}"
        );
    }
    if kind == 0 || kind == 4 {
        assert!(
            !captured && labelled_nap,
            "nap seed={seed} latency_ms={latency:?}: {events:?}"
        );
    }
    if queued_return {
        assert!(!events.iter().any(|e| matches!(e, GapEvent::Ended { last_sample_host_ns, recordable: true, .. } if *last_sample_host_ns != anchor)), "successor seed={seed}: {events:?}");
    }
}

#[test]
fn seeded_reopens_preserve_radio_identity_and_exclude_native_holds() {
    let seeds = std::env::var("CV1TOUCH_MODEL_SEEDS").map_or(256, |n| n.parse().unwrap());
    for seed in 0..seeds {
        reopen_trace(seed);
    }
    eprintln!("classification model: {seeds} seeded reopen traces passed");
}

// Acquisition may drain reports while publication is delayed or handing over. The ground
// truth here is a native hold: only a later accepted fresh publication may end it, and no
// raw sample, unpublished snapshot, overflow or ownership transfer may arm a Radio capture.
fn unpublished_native_trace(seed: u64) {
    use cv1touch::hand::GapCause;
    let mut rng = seed + 1;
    let mut readers = [Reader::new(50_000 * MS), Reader::new(50_000 * MS)];
    let mut owner = 0;
    let mut events = Vec::new();
    readers[0].data(MS, random(&mut rng) & 1 != 0);
    readers[0].publish(MS, &mut events);
    events.clear();
    let cause = if random(&mut rng) & 1 == 0 {
        GapCause::Stall
    } else {
        GapCause::Reopen
    };
    readers[0]
        .hand
        .unavailable(readers[0].offset + 101 * MS, cause);
    events.extend(readers[0].hand.take_event());
    let mut t = 101 * MS;
    for _ in 0..(2 + random(&mut rng) % 12) {
        if random(&mut rng) % 5 == 0 {
            let history = readers[owner].hand.classification_history();
            owner ^= 1;
            let now = readers[owner].offset + t;
            readers[owner].hand.inherit_classification_at(history, now);
        }
        if random(&mut rng) % 5 == 0 {
            readers[owner].hand.reset_reader_evidence();
        }
        if random(&mut rng) % 7 == 0 {
            for i in 0..257 {
                readers[owner].beacon(t + i);
            }
        }
        t += (2 + random(&mut rng) % 900) * MS;
        readers[owner].data(t, random(&mut rng) & 1 != 0);
        let r = &mut readers[owner];
        r.queue.drain_events(&mut r.hand, |e| events.push(e));
        let now = r.offset + t + (100 + random(&mut rng) % 400) * MS;
        r.hand
            .classify_unpublished_events(&r.snapshot, now, |e| events.push(e));
        let gap = r.hand.classification_history().gap.unwrap();
        assert_eq!(
            (gap.recording_anchor_ns, gap.cause, gap.open),
            (50_001 * MS, cause, true),
            "native pre-publication seed={seed}: {events:?}"
        );
        assert!(
            !events.iter().any(|e| matches!(e, GapEvent::Ended { .. })),
            "unpublished native return seed={seed}: {events:?}"
        );
    }
    readers[owner].data(t + MS, false);
    readers[owner].publish(t + MS, &mut events);
    assert!(
        !events.iter().any(|e| matches!(
            e,
            GapEvent::Began {
                cause: GapCause::Radio,
                ..
            } | GapEvent::InUse { .. }
                | GapEvent::Ended {
                    recordable: true,
                    ..
                }
                | GapEvent::Asleep
        )),
        "native became a dropout or nap seed={seed}: {events:?}"
    );
}

#[test]
fn seeded_unpublished_native_drains_never_end_holds_or_arm_capture() {
    let seeds = std::env::var("CV1TOUCH_MODEL_SEEDS").map_or(256, |n| n.parse().unwrap());
    for seed in 0..seeds {
        unpublished_native_trace(seed);
    }
    eprintln!("classification model: {seeds} unpublished native traces passed");
}
