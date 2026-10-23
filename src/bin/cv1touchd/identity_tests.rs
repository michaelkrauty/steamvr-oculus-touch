// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Hardware-free native contract, complete input-state equivalence and allocation/CPU probes.
// Compile the production adapter source with a controlled native writer, without opening USB.
#[path = "../../engine/identity.rs"]
mod adapter;

use super::*;
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::ffi::c_char;
use std::hint::black_box;
use std::time::Instant;

thread_local! {
    static COUNTS: Cell<Option<(usize, usize)>> = const { Cell::new(None) };
}

struct CountingAllocator;
#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

fn count(reallocation: bool) {
    let _ = COUNTS.try_with(|counts| {
        if let Some((alloc, realloc)) = counts.get() {
            counts.set(Some((
                alloc + usize::from(!reallocation),
                realloc + usize::from(reallocation),
            )));
        }
    });
}

// SAFETY: forward every allocation, layout and pointer unchanged to the system allocator.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        count(false);
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        count(true);
        unsafe { System.realloc(ptr, layout, size) }
    }
}

fn allocations(f: impl FnOnce()) -> (usize, usize) {
    struct Reset;
    impl Drop for Reset {
        fn drop(&mut self) {
            COUNTS.with(|c| c.set(None));
        }
    }
    COUNTS.with(|c| assert!(c.replace(Some((0, 0))).is_none()));
    let _reset = Reset;
    f();
    COUNTS.with(|c| c.get().unwrap())
}

// Matches rift.c: one call writes 32 lowercase hex bytes plus NUL, or fails unavailable.
#[inline(never)]
fn native_contract(out: &mut [c_char; 33], value: Option<u8>) -> i32 {
    out[0] = 0;
    match black_box(value) {
        Some(byte) => {
            out[..32].fill(byte as c_char);
            out[32] = 0;
            0
        }
        None => -1,
    }
}

// Frozen pre-optimization adapter and setter, kept as the equivalence/performance oracle.
fn legacy_query(value: Option<u8>) -> Option<String> {
    let mut buf = [0; 33];
    (native_contract(&mut buf, value) == 0).then(|| engine::cstr_field(&buf))
}

fn legacy_set_identity(runtime: &mut InputRuntime, hand: usize, identity: String) {
    if runtime.latest[hand].identity != identity {
        runtime.latest[hand] = RawInputs {
            identity,
            ..Default::default()
        };
    }
}

fn legacy_update(runtime: &Mutex<InputRuntime>, hand: usize, value: Option<u8>) {
    if let Some(identity) = legacy_query(value) {
        let mut runtime = runtime.lock().unwrap();
        // Match the original method call: an unchanged String drops before the guard unlocks.
        legacy_set_identity(&mut runtime, hand, identity);
        black_box(&runtime.latest);
    }
}

fn current_update(runtime: &Mutex<InputRuntime>, hand: usize, value: Option<u8>) {
    if let Some(identity) = adapter::ControllerIdentity::query(|out| native_contract(out, value)) {
        let mut runtime = runtime.lock().unwrap();
        runtime.set_identity(hand, identity.as_str());
        black_box(&runtime.latest);
    }
}

fn config() -> input_calibration::CalibrationFile {
    use input_calibration::{CalibrationFile, Endpoint, HandCalibration, StickAxis};
    let hand = |byte: char| HandCalibration {
        identity: byte.to_string().repeat(32),
        trigger: Endpoint {
            released: -0.1,
            pressed: 0.9,
        },
        grip: Endpoint {
            released: -0.2,
            pressed: 0.8,
        },
        stick: [StickAxis {
            min: -1.0,
            center: 0.1,
            max: 1.0,
        }; 2],
    };
    CalibrationFile::new([hand('a'), hand('b')]).unwrap()
}

fn assert_equivalent(current: &Mutex<InputRuntime>, legacy: &Mutex<InputRuntime>) {
    let current = current.lock().unwrap();
    let legacy = legacy.lock().unwrap();
    // Serialize every RawInputs field, not just the identity or selected analog values.
    assert_eq!(
        serde_json::to_value(&current.latest).unwrap(),
        serde_json::to_value(&legacy.latest).unwrap()
    );
    assert_eq!(current.calibration, legacy.calibration);
    assert_eq!(current.error, legacy.error);
    for hand in 0..2 {
        assert_eq!(
            current.calibration_applies(hand),
            legacy.calibration_applies(hand)
        );
        // Probe calibrated output without disturbing the samples being compared.
        let mut a = InputRuntime {
            calibration: current.calibration.clone(),
            latest: current.latest.clone(),
            ..Default::default()
        };
        let mut b = InputRuntime {
            calibration: legacy.calibration.clone(),
            latest: legacy.latest.clone(),
            ..Default::default()
        };
        for flags in [ffi::CV1T_FLAG_CONNECTED, 0] {
            let mut left = ControllerState {
                flags,
                received_host_ns: 123,
                buttons: 0x8000_0042,
                trigger: 0.4,
                grip: 0.6,
                stick: [0.5, -0.3],
                ..Default::default()
            };
            let mut right = left;
            a.apply(hand, &mut left);
            b.apply(hand, &mut right);
            assert_eq!(format!("{left:?}"), format!("{right:?}"));
        }
    }
}

#[test]
fn identity_lifecycle_matches_owned_adapter() {
    let current = tests::status_daemon();
    let legacy = tests::status_daemon();
    for d in [&current, &legacy] {
        d.inputs.lock().unwrap().calibration = Some(config());
    }
    // First unavailable, first availability, repeats, replacement and calibration mismatch.
    for value in [
        None,
        Some(b'a'),
        Some(b'a'),
        None,
        Some(b'c'),
        Some(b'c'),
        Some(b'a'),
    ] {
        for hand in 0..2 {
            let value = value.map(|v| if hand == 1 && v == b'a' { b'b' } else { v });
            let before = serde_json::to_value(&current.inputs.lock().unwrap().latest).unwrap();
            let old_identity = current.inputs.lock().unwrap().latest[hand].identity.clone();
            current_update(&current.inputs, hand, value);
            legacy_update(&legacy.inputs, hand, value);
            assert_equivalent(&current.inputs, &legacy.inputs);
            let runtime = current.inputs.lock().unwrap();
            match value {
                Some(byte) if old_identity.as_bytes() != [byte; 32] => {
                    let expected = RawInputs {
                        identity: String::from_utf8(vec![byte; 32]).unwrap(),
                        ..Default::default()
                    };
                    assert_eq!(
                        serde_json::to_value(&runtime.latest[hand]).unwrap(),
                        serde_json::to_value(expected).unwrap()
                    );
                }
                _ => assert_eq!(serde_json::to_value(&runtime.latest).unwrap(), before),
            }
            drop(runtime);
            for d in [&current, &legacy] {
                let mut state = ControllerState {
                    flags: ffi::CV1T_FLAG_CONNECTED,
                    received_host_ns: 42,
                    buttons: 0x8000_0042,
                    trigger: 0.4,
                    grip: 0.6,
                    stick: [0.5, -0.3],
                    ..Default::default()
                };
                d.inputs.lock().unwrap().apply(hand, &mut state);
                if value == Some(b'a') || value == Some(b'b') {
                    assert!(d.inputs.lock().unwrap().calibration_applies(hand));
                    assert_ne!(state.grip, 0.6);
                }
                if value == Some(b'c') {
                    assert_eq!(
                        (state.trigger, state.grip, state.stick),
                        input_calibration::factory_values(0.4, 0.6, [0.5, -0.3])
                    );
                }
            }
            assert_equivalent(&current.inputs, &legacy.inputs);
        }
    }
    // Reopening preserves identity while native calibration is unavailable. The real teardown
    // publisher can change raw samples; run it on both sides rather than inventing reset rules.
    for cycle in 0..2 {
        for d in [&current, &legacy] {
            engine_unavailable(d, 1_000 + cycle);
        }
        assert_equivalent(&current.inputs, &legacy.inputs);
        let before_reopen = serde_json::to_value(&current.inputs.lock().unwrap().latest).unwrap();
        for d in [&current, &legacy] {
            start_engine_run(d, cycle + 1, &mut None, || {});
            assert!(!d.engine_offline.load(Ordering::Acquire));
        }
        assert_eq!(
            serde_json::to_value(&current.inputs.lock().unwrap().latest).unwrap(),
            before_reopen
        );
        assert_equivalent(&current.inputs, &legacy.inputs);
        for value in [None, Some(b'a'), Some(b'c')] {
            current_update(&current.inputs, 0, value);
            legacy_update(&legacy.inputs, 0, value);
            assert_equivalent(&current.inputs, &legacy.inputs);
        }
        for d in [&current, &legacy] {
            publish_disconnected(d);
        }
        assert_equivalent(&current.inputs, &legacy.inputs);
        assert!(current
            .inputs
            .lock()
            .unwrap()
            .latest
            .iter()
            .all(|raw| raw.identity.is_empty()));
        current_update(&current.inputs, 0, None);
        legacy_update(&legacy.inputs, 0, None);
        current_update(&current.inputs, 0, Some(b'a'));
        legacy_update(&legacy.inputs, 0, Some(b'a'));
        assert_equivalent(&current.inputs, &legacy.inputs);
    }
}

#[test]
fn identity_queries_allocate_only_on_change() {
    let runtime = Mutex::new(InputRuntime::default());
    current_update(&runtime, 0, Some(b'a')); // initial ownership is unavoidable
    assert_eq!(
        allocations(|| {
            for _ in 0..64 {
                current_update(&runtime, 0, Some(b'a'));
            }
        }),
        (0, 0)
    );
    assert_eq!(
        allocations(|| {
            for _ in 0..64 {
                current_update(&runtime, 0, None);
            }
        }),
        (0, 0)
    );
    assert_eq!(
        allocations(|| current_update(&runtime, 0, Some(b'b'))),
        (1, 0)
    );
    let legacy = Mutex::new(InputRuntime::default());
    legacy_update(&legacy, 0, Some(b'a'));
    assert_eq!(
        allocations(|| legacy_update(&legacy, 0, Some(b'a'))),
        (2, 2)
    );
}

#[test]
#[ignore = "offline CPU benchmark; run alone with --ignored --nocapture --test-threads=1"]
fn identity_adapter_benchmark() {
    const N: usize = 1_000_000;
    type Distribution = (&'static str, fn(usize) -> Option<u8>);
    let distributions: [Distribution; 4] = [
        ("unchanged", |_| Some(b'a')),
        ("changed", |i| Some(b'a' + (i & 1) as u8)),
        ("unavailable", |_| None),
        ("mixed", |i| match i % 1000 {
            0 => Some(b'b'),
            1 => None,
            _ => Some(b'a'),
        }),
    ];
    for (name, value) in distributions {
        let current = Mutex::new(InputRuntime::default());
        let legacy = Mutex::new(InputRuntime::default());
        current_update(&current, 0, Some(b'a'));
        legacy_update(&legacy, 0, Some(b'a'));
        let run = |optimized: bool| {
            for i in 0..N {
                let input = black_box(value(black_box(i)));
                if optimized {
                    current_update(&current, 0, input);
                } else {
                    legacy_update(&legacy, 0, input);
                }
            }
        };
        for round in 0..9 {
            // Identical consumed inputs, repeat and alternate order; allocator counting disabled.
            for optimized in if round % 2 == 0 {
                [false, true]
            } else {
                [true, false]
            } {
                let start = Instant::now();
                run(optimized);
                println!(
                    "{name} round={round} optimized={optimized} ns/query={:.3}",
                    start.elapsed().as_nanos() as f64 / N as f64
                );
            }
            assert_eq!(
                current.lock().unwrap().latest[0].identity,
                legacy.lock().unwrap().latest[0].identity
            );
        }
        for optimized in [false, true] {
            println!(
                "{name} optimized={optimized} alloc/realloc per 1000={:?}",
                allocations(|| {
                    for i in 0..1000 {
                        if optimized {
                            current_update(&current, 0, value(i));
                        } else {
                            legacy_update(&legacy, 0, value(i));
                        }
                    }
                })
            );
        }
    }
}
