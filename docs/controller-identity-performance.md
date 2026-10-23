# Controller identity allocation check

The daemon still queries each available controller once per engine iteration, before updating the engine view or deciding who publishes the hand. The native call runs before taking the input mutex. An available identity still takes that mutex once, including when unchanged. The query result owns 32 bytes on the stack; the input runtime compares its borrowed string and allocates an owned string only when the identity changes. There is no identity cache in the engine.

The native contract in `openhmd_cv1touch.h` and `rift.c` is exactly 32 lowercase hexadecimal bytes and a NUL, or a nonzero error status. The Rust adapter validates that contract, including partial writes and termination. Unlike the old lossy conversion, a malformed successful native response is now unavailable. That is an intentional boundary check; valid native outputs and errors retain their behavior. The generic `cstr_field` is unchanged. Converting each C char back to a byte preserves its bits for both signed and unsigned C char; the supported Linux and Windows x86-64 targets both use signed C char. No pointer is reinterpreted as a string.

## Reproduce without hardware

Use a dedicated Cargo target directory. Run these through the repository's build-slot wrapper, at nice 19, with four build jobs; do not run the daemon or open devices. The benchmark is a test, not an application mode.

```sh
cargo +1.99.0 test --locked identity_
cargo +1.99.0 test --release --locked --bin cv1touchd identity_adapter_benchmark -- --ignored --nocapture --test-threads=1
```

`src/bin/cv1touchd/identity_tests.rs` compiles the production adapter source with a controlled writer implementing the native output contract. It uses the actual `InputRuntime::set_identity`, under the same mutex and `Some` condition as the production caller. The reference keeps the previous `cstr_field` conversion and owned setter, including dropping an unchanged String before the input mutex unlocks. This measures conversion, comparison, ownership, dropping replaced values and mutex acquisition/release; it does not measure native USB/engine work.

## Measurement

Measured on an Intel Core i9-14900KS, Linux x86-64, Rust 1.99.0, release profile opt-level 3, debug assertions off, debug info and incremental compilation disabled. The build-slot wrapper pins the process to four efficiency cores and respects game holds. Nine pairs of one million queries consumed identical inputs, alternating reference/candidate order. The complete latest inputs are consumed through `black_box` in each successful query. Allocation counting is disabled for timing, although the forwarding test allocator remains installed.

| Distribution | Reference median ns/query (range) | Candidate median ns/query (range) | Median change |
| --- | ---: | ---: | ---: |
| Unchanged | 110.309 (108.056–118.114) | 45.203 (43.975–47.695) | -59.0% |
| Changed every query | 109.535 (108.375–111.191) | 49.794 (49.319–50.647) | -54.5% |
| Unavailable | 3.297 (3.239–3.537) | 4.883 (4.832–4.915) | +48.1% |
| Mixed | 106.866 (105.616–108.127) | 44.092 (43.587–45.420) | -58.7% |

Mixed means replacement B at query 0 of every thousand, unavailable at query 1, and A otherwise. Changed alternates A and B; unchanged repeats A. Initial A ownership is outside the measured loop. The unavailable median regressed by 1.586 ns (+48.1%), with nonoverlapping observed ranges. The available-query savings and removal of repeated heap work justify the change; it is not a CPU win for the unavailable-only distribution.

Allocation calls and reallocation calls, counted separately over 1,000 complete queries after initial ownership:

| Distribution | Reference alloc/realloc | Candidate alloc/realloc |
| --- | ---: | ---: |
| Unchanged | 2,000 / 2,000 | 0 / 0 |
| Changed | 2,000 / 2,000 | 1,000 / 0 |
| Unavailable | 0 / 0 | 0 / 0 |
| Mixed | 1,998 / 1,998 | 2 / 0 |

This is a local CPU/allocation result with synthetic native-contract output. Live engine iteration rate, native FFI cost, contention, end-to-end latency and displayed judder were not measured. In particular these savings must not be multiplied by an assumed 500 Hz engine rate.

## Equivalence and sensitivity

The lifecycle test compares every serialized `RawInputs` field, calibration applicability, active calibration and error state after each operation. It also compares complete `ControllerState` debug outputs after calibrated connected and disconnected input application. Both hands cover first unavailability, first availability, repeated identity with populated samples, temporary unavailability, replacement, matching calibration, mismatched calibration with factory fallback, engine unavailability/reopening and explicit disconnect. Engine-unavailable and disconnect steps use the actual daemon lifecycle functions on both sides. No native engine is opened.

The boundary tests cover all 256 byte values at all 33 buffer positions, error statuses even with apparently valid output, missing writes including the terminator, one reader call per query and independent ownership of successive results. These tests also compile in the Windows checks. They cannot execute a Windows native engine on Linux.

Before the optimization, the allocation regression failed with 128 allocations and 128 reallocations for 64 unchanged queries, while lifecycle equivalence passed. Mutations removing the terminator check, accepting error statuses, and resetting raw inputs on unchanged identity each failed their relevant tests. Sources were restored afterward. The final gates run on the restored implementation.

Caller review confirms the only production getter call remains after successful `controller(h)` and before the engine-view lock, publication decision and publication. The query uses `FnOnce` and makes the same single native call with a 33-byte buffer. No lock is taken on native error; valid unchanged identities still lock once. `engine_unavailable`, `publish_disconnected`, engine construction/destruction and calibration application are unchanged. Native opening/teardown sequencing and hardware reads are covered by this source review, not by a simulated engine execution.
