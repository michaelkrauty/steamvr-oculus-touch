# Battery report parsing

`hand::battery_from_radio` borrows a radio report and yields accepted battery
messages in their original order. It checks for exactly 64 bytes and report ID
`0x0c` before accessing either 28-byte message. Device types 2 and 3 select the
left and right hands; message ID `0x1c` and ADC channel `0x21` select battery
readings, capped at 100 percent. Both messages are delivered even for the same
hand. Invalid reports and messages produce no updates.

The engine's source-1 report callback consumes these updates immediately. Its
relaxed atomic swaps and change-only diagnostics are unchanged. The separate
radio decoder does not invoke this callback. Previously the parser collected a
short-lived Vec: one allocation for one or two updates, zero for no updates.
The iterator allocates nothing on any of these paths.

## Offline measurement

On an Intel i9-14900KS's efficiency cores, Rust 1.99.0 release builds measured
these median elapsed nanoseconds per parse plus atomic consumption. The probe
uses `std::time::Instant`; scheduling delays and pauses are included, so these
are not measurements of thread CPU time:

| Input | Vec | Iterator |
| --- | ---: | ---: |
| Empty input | 2.09 | 1.17 |
| Short report | 2.10 | 1.17 |
| Oversized report | 2.10 | 1.17 |
| Wrong report ID | 2.10 | 1.68 |
| Valid report, no updates | 7.19 | 3.97 |
| One update | 15.28 | 8.47 |
| Two hands | 19.50 | 15.05 |
| Two updates to one hand | 19.44 | 14.85 |

These compare the actual public parser before and after the change, using saved
release test executables in eight alternating process pairs. Each process took
16 samples of one million consumed calls per input; the table gives the median
of its process medians. Every paired process median improved for every input.
Allocation probes counted 1,000 allocations per 1,000 nonempty reports before,
and zero after; empty and invalid paths were already zero. Timing builds
use the ordinary system allocator, without counting hooks; allocation counting
is confined to the separate, optimized test-profile executable.

The narrow reproducer is `tests/battery_radio.rs`. Its ignored
`battery_parse_and_consume_timings` test runs with `cargo test --locked
--no-default-features --release --test battery_radio -- --ignored --nocapture`.
It includes a frozen collected parser as an oracle; its local timing can differ
from the old public function's timing because of compiler visibility. Compare
saved before/after executables for measurements across the public API boundary.
Use a dedicated target directory and the development machine's build-slot/game
hold wrapper when running the probe.

This measures elapsed time for synthetic parsing and consumption, including
relaxed atomic swaps and change detection. Checksums are recorded and compared.
Final battery values are consumed through `black_box` to prevent optimization;
the timing logs do not retain or compare those arrays.
It excludes diagnostic formatting/output, USB delivery, other callback work,
and display latency. The daemon regression separately checks atomic state and
exact useful log order through the real callback. No current live packet rate,
end-to-end latency or judder improvement was measured.
