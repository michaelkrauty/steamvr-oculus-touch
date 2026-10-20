# Unix socket test paths

The inventory below lists the tests that bind or connect a Unix socket under the
temporary base, by file and test name. Lengths are bytes, excluding the
terminating NUL. `T` is the temporary base length without a trailing slash; `P`
is the PID digit count (at most 7 on Linux); `R` is the random suffix (6 for
native `mkdtemp`, 8 for Python); `U` is the UID digit count (at most 10);
`N` is a host timestamp's decimal digit count (at most 20 for `u64`). The
before column evaluates `T=80`, `P=7`, `N=20`, and the largest applicable
suffix/UID. All `T`-dependent paths were unbounded by the tests. Linux's
`sun_path[108]` permits 107 pathname bytes.

## Fixture ownership and bounds

The Rust, native C++ and Python socket fixtures create a unique `0700` directory
under the selected temporary base. They keep an open directory descriptor and use
`/proc/<owner-pid>/fd/<fd>/r` as a short view of a real child directory. Every file
still lives inside that fixture's private tree under the original `TMPDIR`.
There is no external alias file, fixed shared socket name, or unconditional `/tmp`.
The descriptor stays open until the tests and their child processes finish, and
cleanup removes the owned tree before closing it. The owner PID is required:
`/proc/self` would point at a different process after exec. The real final `r`
preserves the existing final-component symlink and private-directory checks.

Linux procfs is required. The view root is at most `12 + P + F = 29` bytes, where
`F <= 10` is the directory descriptor's digit count. With the current relative
names, every normal socket candidate below is at most **75 bytes**, independent
of the temporary base's spelling or encoding. Windows tests retain ordinary
private temporary directories and their named-pipe transport.

The helpers are `tests/support/socket_dir.rs`, `driver/tests/socket_dir.h`, and
`tests/socket_dir.py`. Rust production modules include the helper only under
`cfg(test)`; the driver contract example also uses it. Production endpoints do
not use these views.

The helper's own regression,
`a_long_temp_base_keeps_a_private_short_socket_and_cleans_up_its_tree` in
`tests/support/socket_dir.rs`, creates a 100-byte nested temporary base, binds
through the short view, verifies real-directory ownership and `0700`
permissions, checks that the socket stays beneath that base, and confirms
cleanup removes only its own tree. It is a `cfg(test)` module, so it runs in the
library, `cv1touch` and `cv1touchd` test binaries and, under
`cargo test --all-targets` (what `scripts/check.sh` and CI run), in the driver
contract example's test build; a plain `cargo test` builds that example without `cfg(test)`.

`scripts/check.sh` also reruns the Rust and native suites, driver contract, and
Python setup and report suites with an explicitly long `TMPDIR`. It creates a
private tree under the build output, adds an 80-byte directory component, and
removes that tree on exit, including when a test fails. This catches a reverted helper or
caller even when the check script itself was started with a short temporary base.

## Inventory

For fixtures with multiple endpoints, the row gives the longest candidate unless
it lists both. A missing fallback candidate matters too: an overlong path would
exercise length rejection instead of the intended `ENOENT` retry.

| File | Test / endpoint | Before: bytes at T=80 | Fixed maximum |
| --- | --- | --- | --- |
| `src/control.rs` | `an_idle_client_is_closed_without_a_request` | T + P + 25 = 112 | 39 |
| `src/control.rs` | `partial_bytes_do_not_extend_a_request_deadline` | T + P + 28 = 115 | 42 |
| `src/control.rs` | `complete_requests_keep_a_persistent_connection_alive` | T + P + 31 = 118 | 45 |
| `src/control.rs` | `an_overlong_complete_request_is_not_handled` | T + P + 25 = 112 | 39 |
| `src/control.rs` | `request_roundtrip` | T + P + 20 = 107 | 42 |
| `src/control.rs` | `a_request_times_out_when_the_daemon_does_not_accept_connections` | T + P + 23 = 110 | 42 |
| `src/platform/mod.rs` | `an_expired_request_does_not_consume_its_reply` | T + P + 33 = 120 | 39 |
| `src/platform/mod.rs` | `an_expired_request_does_not_write_its_command` | T + P + 34 = 121 | 40 |
| `src/platform/mod.rs` | `a_connection_carries_a_line_each_way` | T + P + 28 = 115 | 39 |
| `src/platform/mod.rs` | `a_silent_listener_times_a_read_out_and_a_second_listener_is_refused` | T + P + 31 = 118 | 42 |
| `src/platform/mod.rs` | `the_listener_keeps_serving_after_clients_vanish` | T + P + 30 = 117 | 41 |
| `src/platform/unix.rs` | `container_control_creates_a_private_directory_and_socket` | T + P + 44 = 131 | 50 |
| `src/platform/unix.rs` | `container_control_new_directory_ignores_a_restrictive_umask` | T + P + 43 = 130 | 50 |
| `src/platform/unix.rs` | `container_control_refuses_existing_unsafe_directories_without_changing_them` | T + P + 44 = 131 | 50 |
| `src/platform/unix.rs` | `container_control_replaces_only_its_stale_socket_and_cleans_up_only_a_socket` | T + P + 43 = 130 | 50 |
| `src/bin/cv1touchd.rs` | `idle_control_clients_release_all_sixteen_slots` | T + P + 40 = 127 | 50 |
| `src/bin/cv1touchd.rs` | `the_control_server_answers_each_line_until_it_stops` | T + P + 35 = 122 | 50 |
| `src/bin/cv1touchd.rs` | `an_unsafe_container_directory_keeps_the_legacy_control_server` | T + P + 44 = 131 | 50 |
| `src/bin/cv1touchd.rs` | `a_full_container_control_backlog_keeps_the_legacy_server_available` | T + P + 40 = 127 | 50 |
| `src/bin/cv1touchd.rs` | `container_control_is_not_published_after_either_instance_override` | T + P + 61 = 148 | 75 |
| `src/bin/cv1touchd.rs` | `the_supervisor_starts_a_child_when_runtime_and_logs_overlap` / `supervisor_entry_fixture` | new fixture | 62 |
| `examples/driver_contract.rs` | `legacy control endpoint` | T + P + 20 = 107 | 42 |
| `examples/driver_contract.rs` | `private fallback endpoint` | T + P + 36 = 123 | 50 |
| `examples/driver_contract.rs` | `missing legacy candidate` | T + P + 23 = 110 | 45 |
| `driver/tests/test_control_disconnect.cpp` | `control / fallback` | T + P + 30 / 31 = 117 / 118 | 42 / 43 |
| `driver/tests/test_haptics.cpp` | `sends_to_the_socket` | T + P + 16 = 103 | 42 |
| `driver/tests/test_control_client.cpp` | `legacy / shared fixtures` | 39 + R / 47 + R = 45 / 53 | 41 / 49 |
| `driver/tests/test_control_client.cpp` | `default client / worker: fallback, missing runtime` | 58 + R + U / 65 + R = 74 / 71 | 70 / 67 |
| `tests/test_setup.py` | `SetupTests.test_check_is_read_only_and_explains_missing_stages` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_noninteractive_guide_fails_before_any_daemon_command_or_write` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_corrupt_checkpoint_is_not_overwritten_or_silently_ignored` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_legacy_adoption_and_unchanged_resume_only_write_the_journal` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_external_world_change_and_added_sensor_cannot_claim_ready` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_sleeping_hands_reuse_valid_files_without_capture_or_erasure` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_failed_camera_stage_persists_dependency_invalidation_before_mutation` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_invalid_camera_failure_restores_provisional_without_rewriting_artifacts` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_saved_stages_require_daemon_loaded_configuration_for_live_readiness` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_the_slip_check_can_be_turned_off_for_the_whole_guide` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_guide_does_not_claim_completion_while_the_daemon_ignores_the_saved_setup` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_pending_engine_reload_blocks_capture_despite_advancing_old_streams` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_legacy_daemon_checks_work_but_guided_reload_requires_ready_marker` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_human_actions_are_bold_when_forced_but_json_and_no_color_stay_plain` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_malformed_journal_is_read_only_and_never_contacts_daemon` | T + R + 46 = 134 | 55 |
| `tests/test_report.py` | `ReportTests.test_logs_settings_and_a_summary_in_one_zip` | T + R + 43 = 131 | 55 |
| `tests/test_report.py` | `ReportTests.test_an_interrupted_log_rotation_is_listed_by_name_and_size` | T + R + 43 = 131 | 55 |
| `tests/test_report.py` | `ReportTests.test_a_missing_or_non_executable_stub_cannot_reach_the_host_journal` | T + R + 43 = 131 | 55 |
| `tests/test_report.py` | `ReportTests.test_an_existing_file_is_left_alone` | T + R + 43 = 131 | 55 |
| `src/bin/cv1touch.rs` | `another_clients_early_end_acknowledgement_never_completes_our_capture` | T + N + 31 = 131 | 42 |
| `src/bin/cv1touchd.rs` | `blocked_recording_storage_does_not_block_control_commands` | T + N + 40 = 140 | 42 |
| `src/bin/cv1touchd.rs` | `scoped_stop_cancels_an_opening_recording_without_holding_up_status` | T + N + 35 = 135 | 42 |
| `src/bin/cv1touchd.rs` | `a_dispatched_recording_is_reserved_before_shutdown_can_pass_it` | T + N + 43 = 143 | 52 |
| `src/bin/cv1touchd.rs` | `a_control_record_start_opens_the_file_and_prepares_its_session` | T + N + 33 = 133 | 42 |
| `tests/test_setup.py` | `SetupTests.test_a_long_control_socket_is_explained_before_guide_state_is_created` | mock: T + R + 46 = 134 | 55 for mock; oversized endpoint rejected |
| `tests/test_setup.py` | `SetupTests.test_old_running_daemon_is_refused_before_capture_preparation` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_a_daemon_that_is_not_running_is_left_to_the_guided_checks` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_a_busy_daemon_is_left_to_the_guided_checks` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_pending_recording_blocks_idle_and_stages_with_other_checks_ready` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_pending_recording_offers_retry_without_stage_mutations` | T + R + 46 = 134 | 55 |
| `tests/test_setup.py` | `SetupTests.test_a_second_setup_names_the_running_guide_not_a_recording_to_stop` | T + R + 46 = 134 | 55 |
| `src/bin/cv1touchd.rs` | `a_blocked_startup_recording_open_or_header_leaves_control_available` | T + N + 45 = 145 | 52 |
| `src/bin/cv1touchd.rs` | `a_startup_recording_open_or_header_error_is_fatal_and_retires_ipc` | T + N + 50 = 150 | 52 |
| `src/bin/cv1touchd.rs` | `startup_recording_cleanup_reports_its_final_storage_outcome` | T + N + 62 = 162 | 52 |
| `src/bin/cv1touchd.rs` | `a_startup_error_queued_during_the_shutdown_wait_remains_fatal` | T + N + 42 = 142 | 52 |

`test_control_client.cpp` uses its legacy/shared fixture from
`runtime_is_tried_first_and_an_explicit_path_stays_exact`,
`a_missing_socket_falls_back_and_the_working_endpoint_is_remembered`,
`a_refused_socket_falls_back`,
`a_missing_cached_directory_falls_back_but_an_unsafe_one_does_not`,
`other_connection_errors_do_not_fall_back`,
`a_failed_acknowledgement_does_not_resend_the_request`,
`the_shared_socket_requires_a_private_real_directory`, and
`the_haptic_worker_uses_and_remembers_the_same_fallback`. The two default
constructor cases (`default_constructors_reach_the_shared_socket`) use the
additional paths listed above.
`test_daemon_contract.cpp` connects to the paths supplied by the Rust example;
`scripts/check.sh` runs that example and the native/Python suites.

## Deliberate invalid paths and non-socket fixtures

- `other_connection_errors_do_not_fall_back` in `test_control_client.cpp`
  deliberately appends 200 bytes to its root to test overlength rejection. It
  remains overlong (at most 230 bytes); it is never a listening endpoint. Its
  ENOTDIR and EACCES cases, and the final-directory symlink case in
  `the_shared_socket_requires_a_private_real_directory`, retain their original
  failure conditions.
- Fixed path assertions in `resolves_the_runtime_and_shared_paths_for_the_same_user`
  (`test_control_client.cpp`) and `container_control_is_scoped_to_the_effective_user`
  (`src/platform/unix.rs`) do not bind or connect. Their host runtime path is
  `32 + U <= 42` bytes; the shared path is `39 + U <= 49`.
  `/nonexistent/control.sock` in `newest_command_per_hand_wins`
  (`test_haptics.cpp`) is 25 bytes and its worker is never started.
- The Windows-only `temp_endpoint` helper in `test_haptics.cpp` derives pipe
  names from `GetTempPathA`, adding `P+15` or `P+16` beyond that returned string.
  Its overlong-pipe check (`names_the_pipe_like_the_daemon`) is intentional.
  `a_client_refuses_a_pipe_another_account_owns` in `src/platform/mod.rs` uses a
  named pipe, not AF_UNIX.
- `single_instance_lock` in `src/control.rs` and
  `the_daemon_lock_is_exclusive_until_released` in `src/platform/mod.rs` exercise
  ordinary lock files. They do not use `sun_path`; the latter shares the endpoint
  fixture.
- `ControlSocketPathTests.test_a_rejected_supervisor_path_does_not_wait_for_stderr`
  in `tests/test_runtime_dir.py` deliberately supplies an overlong runtime path.
  It must reject it before creating state or binding a socket; its full stderr
  pipe has no pathname. The fixture stays alive until the child exits.
- The overlapping runtime/logs supervisor entry fixture uses `SocketDir` and keeps it
  alive through the child test process's exit. Its control candidate is
  `root/state/cv1touch/logs/control.sock` (29 + 33 = 62 bytes). The test child
  rejects its arguments before any hardware or control listener starts.
- The `src/supervise.rs` fixtures use log and lock files, child-process pipes,
  and ordinary stop/ready files. `stand_in_daemon` and `unavailable_log_fixture`
  do not start a real daemon or create a control endpoint. Their parent tests
  wait for the children before removing the fixtures. These directories are
  not socket runtime directories and do not need a short view.
- The HID tests use an unnamed `socketpair` (`fake_hid` in
  `src/platform/unix.rs`), which has no pathname length. Ignored live OpenVR
  probes do not construct a test Unix socket address.
- `DriverPackageTests.setUp` in `test_driver_package.py` sets an isolated runtime
  directory but its install/check/remove commands do not request daemon control.
  A hypothetical runtime endpoint would be `T+R+48 = 136` bytes.
  `test_install.py` uses ordinary fixture files. Native C tests, remaining Python
  tests, scripts, and vendored third-party tests contain no additional pathname
  AF_UNIX fixtures. Ordinary config, recording, log, process and USB trees are
  unaffected.

## Production paths

These helpers apply to tests only. The daemon's and driver's real control
endpoints are validated separately.
