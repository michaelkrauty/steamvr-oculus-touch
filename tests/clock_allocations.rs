// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

use cv1touch::track::clock::LinearClock;
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

// Count only allocations on this thread during its warmed-up observation loop.
struct Counter;
thread_local! {
    static ALLOCS: Cell<Option<usize>> = const { Cell::new(None) };
}
#[global_allocator]
static ALLOCATOR: Counter = Counter;
unsafe impl GlobalAlloc for Counter {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let _ = ALLOCS.try_with(|c| {
            if let Some(n) = c.get() {
                c.set(Some(n + 1));
            }
        });
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[test]
fn warmed_clock_observations_do_not_allocate() {
    let mut clock = LinearClock::default();
    for i in 0..100u64 {
        clock.observe(i * 1_000_000_000, i * 1_000_000_000 + 1_000_000);
    }
    ALLOCS.with(|c| c.set(Some(0)));
    for i in 100..1100u64 {
        let device = i * 1_000_000_000;
        clock.observe(device, device + 1_000_000);
        // Improve the current bin too, not just its rollover.
        clock.observe(device + 1, device + 999_999);
        std::hint::black_box(clock.to_host(device));
    }
    let count = ALLOCS.with(|c| c.replace(None).unwrap());
    assert_eq!(count, 0);
}
