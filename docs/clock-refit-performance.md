# Clock refit allocation removal

`LinearClock::observe` runs in the controller IMU and headset clock paths in
`src/track/pipeline.rs`. It refits on a bin rollover or an improved minimum,
not on every report. Once four closed bins exist, `refit` previously collected
those bins into a temporary `Vec` solely to iterate over them. Borrowing the
`VecDeque` removes that copy and allocation without changing the four reductions
or their forward order. No cache, dependency, or additional retained state is used.

## Verification

Run from the repository root:

```sh
cargo test --locked --no-default-features --test clock_allocations
cargo test --locked --no-default-features --lib track::clock::tests
scripts/check.sh
```

The allocation regression warms up the real public clock API, then counts
allocations on the calling thread during 1,000 bin rollovers and 1,000 improved
minima. Fixture construction and assertions are outside the counted interval.
Restoring the original `collect` line makes this regression fail with 2,000
allocation requests; borrowing the deque passes with zero. This tests only the
warmed clock observations, not the whole tracking pipeline.

The differential test compares both fitted coefficients by their exact floating
point bits against the original contiguous-copy algorithm. It covers lengths
4, 5, 19, 20, and 64, every deque rotation at each length, and both varying and
constant x (the zero-denominator branch). Existing drift, restart, and timestamp
wrap tests also pass. Startup fitting is untouched.

These checks were run on Linux x86-64, Intel Core i9-14900KS, Rust 1.86.0.
The verified benefit is removal of allocation requests and temporary copying.
No wall-clock speedup, frame-latency improvement, or live refit frequency is
claimed; those require a separate timing experiment with uncertainty estimates.
