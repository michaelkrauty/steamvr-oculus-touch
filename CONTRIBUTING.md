# Contributing

Open an issue first with what you saw, how to reproduce it, your hardware, distribution and version. Search existing issues before opening one. Keep each pull request to one logical change, with conventional commit messages (`fix:`, `feat:`, `docs:` and so on).

## Build and check

Clone `main`, install the [build dependencies](docs/setup.md#build-dependencies), and run the checks before opening a pull request:

```sh
git clone https://github.com/michaelkrauty/steamvr-oculus-touch.git
cd steamvr-oculus-touch
scripts/check.sh
```

`scripts/check.sh` runs the Python release and installer tests, rustfmt, Clippy, the Rust tests, the native Meson tests and the driver contract tests, then repeats the socket tests with a long `TMPDIR`. On Linux it also audits the tests that abort on purpose. CI runs it with Rust 1.85.0 and stable. For Windows builds, see [building on Windows](docs/building-windows.md).

## Tests

- A bug fix needs a regression test that fails without the fix.
- Tests that need hardware are `#[ignore]`d, with the procedure in a comment. Never make a hardware test pass unconditionally.
- Socket fixtures (`tests/support/socket_dir.rs` and its native and Python counterparts) create a private `0700` directory under `TMPDIR` and reach it through a short `/proc/<pid>/fd` path, so socket paths stay within Linux's 107-byte limit whatever `TMPDIR` is. Use them instead of fixed socket names or `/tmp`.
- Tests that abort on purpose turn off core dumps, and on Linux dumpability, only in the child process just before it aborts. Do not apply that to a whole suite or to a process that owns a socket fixture. New intentional aborts must be added to `tests/check_expected_aborts.py`.
- The memory-locking and real-time scheduling tests are ignored by default because they need host limits that CI runners lack. Run one with, for example, `cargo test --locked --bin cv1touchd locked_memory_stays_resident_once_touched -- --ignored`.

## Changes

- Record user-visible changes in `CHANGELOG.md` under `Unreleased`.
- A change to the shared-memory ABI (`common/*.h`) or the recording format must update both readers and writers and their compatibility tests.
- Performance claims need a baseline, the method and the hardware.
- Keep upstream copyright and license notices, and update `third_party/*` provenance when changing pinned sources.
- Never commit SteamVR or Oculus assets, personal calibration, room recordings, device serial numbers or credentials. Examples use fictional identifiers and synthetic geometry.

## CI for forks

Pull requests from forks run on a self-hosted runner only after the maintainer approves each revision; a pending approval is not a passing check. You can also enable Actions on your fork to run the same checks on GitHub-hosted runners.

Workflows run only on `push`, `pull_request` and `workflow_dispatch`. Do not add triggers such as `pull_request_target`, `workflow_run` or `issue_comment` that would run contributor code without approval.

Maintainers: see the [release checklist](docs/releasing.md).

## License

Contributions are licensed under the repository's Apache License 2.0. Third-party code keeps its own license.
