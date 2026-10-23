// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Battery parser equivalence, allocation regression and opt-in parse/consume timing.
use cv1touch::hand::battery_from_radio;
use std::hint::black_box;
use std::sync::atomic::{AtomicI32, Ordering};
use std::time::Instant;

// Release CPU probes use the ordinary system allocator, without counting hooks.
#[cfg(debug_assertions)]
mod allocation_probe {
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::cell::Cell;

    thread_local! {
        static ALLOCATIONS: Cell<Option<usize>> = const { Cell::new(None) };
    }
    struct CountingAllocator;
    // Count only the current probe thread, not the test harness or other test threads.
    unsafe impl GlobalAlloc for CountingAllocator {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            ALLOCATIONS.with(|n| n.set(n.get().map(|n| n + 1)));
            // SAFETY: forward the caller's allocation contract to the system allocator.
            unsafe { System.alloc(layout) }
        }
        unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
            // SAFETY: forward the unchanged pointer and layout to its allocator.
            unsafe { System.dealloc(ptr, layout) }
        }
        unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
            ALLOCATIONS.with(|n| n.set(n.get().map(|n| n + 1)));
            // SAFETY: forward the caller's reallocation contract unchanged.
            unsafe { System.realloc(ptr, layout, size) }
        }
    }
    #[global_allocator]
    static ALLOCATOR: CountingAllocator = CountingAllocator;

    pub fn count(run: impl FnOnce()) -> usize {
        ALLOCATIONS.with(|n| n.set(Some(0)));
        run();
        ALLOCATIONS.with(|n| n.replace(None).unwrap())
    }
}

fn ordered(updates: impl IntoIterator<Item = (usize, i32)>) -> Vec<(usize, i32)> {
    updates.into_iter().collect()
}

// Frozen pre-iterator implementation, kept only as the differential/timing oracle.
fn collected(d: &[u8]) -> Vec<(usize, i32)> {
    if d.len() != 64 || d[0] != 0x0c {
        return Vec::new();
    }
    (0..2)
        .filter_map(|i| {
            let m = &d[3 + 28 * i..3 + 28 * (i + 1)];
            let hand = match m[2] {
                2 => 0,
                3 => 1,
                _ => return None,
            };
            (u16::from_le_bytes([m[0], m[1]]) == 0x1c && m[25] == 0x21)
                .then(|| (hand, u16::from_le_bytes([m[26], m[27]]).min(100) as i32))
        })
        .collect()
}

fn report(messages: [(u16, u8, u8, u16); 2]) -> [u8; 64] {
    let mut d = [0; 64];
    d[0] = 0x0c;
    for (i, (id, device, channel, pct)) in messages.into_iter().enumerate() {
        let m = &mut d[3 + 28 * i..3 + 28 * (i + 1)];
        m[..2].copy_from_slice(&id.to_le_bytes());
        m[2] = device;
        m[25] = channel;
        m[26..28].copy_from_slice(&pct.to_le_bytes());
    }
    d
}

#[test]
fn battery_reports_preserve_guards_clamping_and_both_message_positions() {
    let good = (0x1c, 2, 0x21, 87);
    // Every device/channel byte, high report-id bytes and clamp boundaries. Put each
    // candidate before and after a valid message, including duplicate left hands.
    for device in 0..=255 {
        for channel in 0..=255 {
            for pct in [0, 100, 101, 65535] {
                for id in [0x1b, 0x1c, 0x11c, 0xffff] {
                    let candidate = (id, device, channel, pct);
                    for pair in [[candidate, good], [good, candidate]] {
                        let d = report(pair);
                        let actual: Vec<_> = ordered(battery_from_radio(&d));
                        assert_eq!(actual, collected(&d), "{pair:?}");
                    }
                }
            }
        }
    }
    let d = report([good, (0x1c, 3, 0x21, 65535)]);
    assert_eq!(ordered(battery_from_radio(&d)), [(0, 87), (1, 100)]);
    let duplicate_right = report([(0x1c, 3, 0x21, 101), (0x1c, 3, 0x21, 0)]);
    assert_eq!(
        ordered(battery_from_radio(&duplicate_right)),
        [(1, 100), (1, 0)]
    );
    let mut padded = [0; 256];
    padded[..64].copy_from_slice(&d);
    for len in (0..=65).chain([127, 128, 255, 256]) {
        assert_eq!(
            ordered(battery_from_radio(&padded[..len])),
            collected(&padded[..len]),
            "length {len}"
        );
    }
    for id in 0..=255 {
        padded[0] = id;
        assert_eq!(
            ordered(battery_from_radio(&padded[..64])),
            collected(&padded[..64]),
            "report id {id}"
        );
    }
}

// Same atomic swap and change condition as Daemon::report. Timing deliberately
// excludes diagnostic formatting/output; the daemon test covers those effects.
fn consume(updates: impl IntoIterator<Item = (usize, i32)>, battery: &[AtomicI32; 2]) -> i32 {
    let mut sum = 0;
    for (hand, pct) in updates {
        let was = battery[hand].swap(pct, Ordering::Relaxed);
        sum += was + pct + i32::from(was != pct);
    }
    sum
}

fn cases() -> Vec<(&'static str, Vec<u8>)> {
    let good = (0x1c, 2, 0x21, 87);
    let empty = (0, 0, 0, 0);
    let mut invalid = report([good; 2]);
    invalid[0] = 0x0b;
    vec![
        ("empty-input", vec![]),
        ("short", vec![0x0c; 63]),
        ("oversized", vec![0x0c; 65]),
        ("wrong-id", invalid.to_vec()),
        ("zero-updates", report([empty; 2]).to_vec()),
        ("one-update", report([good, empty]).to_vec()),
        ("two-updates", report([good, (0x1c, 3, 0x21, 50)]).to_vec()),
        (
            "duplicate-hand",
            report([good, (0x1c, 2, 0x21, 50)]).to_vec(),
        ),
    ]
}

#[cfg(debug_assertions)]
#[test]
fn consuming_battery_reports_allocates_nothing() {
    for (name, d) in cases() {
        let battery = [AtomicI32::new(-1), AtomicI32::new(-1)];
        let before = allocation_probe::count(|| {
            black_box(consume(collected(black_box(&d)), &battery));
        });
        let after = allocation_probe::count(|| {
            black_box(consume(battery_from_radio(black_box(&d)), &battery));
        });
        println!("{name}: allocations before={before} after={after}");
        assert_eq!(before, usize::from(!collected(&d).is_empty()), "{name}");
        assert_eq!(after, 0, "{name}");
    }
}

#[test]
#[ignore = "release CPU probe; run explicitly with --release --ignored --nocapture"]
fn battery_parse_and_consume_timings() {
    const N: usize = 1_000_000;
    for (name, d) in cases() {
        let run = |old: bool, n: usize| {
            let battery = [AtomicI32::new(-1), AtomicI32::new(-1)];
            let start = Instant::now();
            let mut sum = 0i64;
            for _ in 0..n {
                sum += i64::from(if old {
                    consume(collected(black_box(&d)), black_box(&battery))
                } else {
                    consume(battery_from_radio(black_box(&d)), black_box(&battery))
                });
            }
            black_box((sum, battery.map(|b| b.load(Ordering::Relaxed))));
            (start.elapsed().as_nanos() as f64 / n as f64, sum)
        };
        for round in 0..16 {
            let mut sums = [0; 2];
            for old in if round % 2 == 0 {
                [true, false]
            } else {
                [false, true]
            } {
                let (ns, sum) = run(old, N);
                sums[usize::from(old)] = sum;
                println!("time {name} round={round} old={old} ns={ns:.3} sum={sum}");
            }
            assert_eq!(sums[0], sums[1], "{name} round {round}");
        }
    }
}
