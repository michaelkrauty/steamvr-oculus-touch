# Contributing

Start with an issue describing the observed behavior, reproduction, hardware, distribution, and version. Search existing issues first. Keep pull requests focused on one logical change and use conventional commit messages (`fix:`, `feat:`, `docs:`, and similar).

For development, clone the default `main` branch rather than a release tag:

```sh
git clone https://github.com/michaelkrauty/steamvr-oculus-touch.git
cd steamvr-oculus-touch
```

Install the dependencies in [setup](docs/setup.md), then run the checks in `scripts/check.sh`. Bug fixes should include a regression that fails before the fix. Hardware-dependent tests should be explicitly marked and describe the required procedure; do not replace missing hardware coverage with unconditional passes. See [Unix socket test paths](docs/testing-sockets.md) for the socket fixtures' ownership and byte budgets.

CI runs `scripts/check.sh` with Rust 1.85.0 and stable. In the upstream repository, the maintainer's pull requests and pushes to `main` run automatically on a dedicated Linux runner with preinstalled native dependencies. External fork pull requests require the maintainer to choose **Approve and run** before their checks execute on that runner; approval is for the current revision, so new commits require another approval. A pending approval is not a passing check. Fork owners can also enable Actions in their own repositories to run the same checks on GitHub-hosted Linux runners.

The maintainer can run checks on a repository branch through **Actions → Linux checks → Run workflow**. This manual trigger does not approve a fork pull request. While the upstream repository is private, workflows from private fork pull requests remain disabled.

## Maintainer CI policy

For the public upstream repository, set **Settings → Actions → General → Approval for running fork pull request workflows from contributors → Require approval for all external contributors**. The corresponding API policy is `all_external_contributors`. Keep repository write/admin access limited to the maintainer: GitHub's fork approval gate exempts repository members, and anyone with write access can approve runs or change workflows. The actor checks in the workflow select intended jobs; the GitHub-side approval setting controls whether external fork workflows may start.

Before approving a revision, review its workflow changes and every path that can execute during the build, including build scripts, tests, dependencies, and local actions. Approval authorizes that code to execute on a persistent runner. A mistaken approval can compromise the runner and affect later jobs; a dedicated service account and host hardening reduce exposure but do not provide a fresh environment for each run. See [GitHub's self-hosted runner security guidance](https://docs.github.com/en/actions/reference/security/secure-use#hardening-for-self-hosted-runners) and [fork workflow approval instructions](https://docs.github.com/en/actions/how-tos/manage-workflow-runs/approve-runs-from-forks).

Keep CI on `push`, `pull_request`, and `workflow_dispatch`. Do not add automatic handlers such as `pull_request_target`, `workflow_run`, or `issue_comment` that execute contributor code without a separate review of their trust model; the fork approval policy does not cover every event.

## Preparing a private repository for publication

GitHub's public fork approval policy cannot be configured while a repository is private. Before changing visibility, **disable GitHub Actions for the entire repository**, finish or cancel all remaining runs (including queued or waiting runs), and verify that the runner is idle. Keep Actions disabled until the public repository's approval policy has been set to `all_external_contributors` and read back successfully, and the collaborator list has been checked. Then re-enable Actions. The dedicated runner may remain registered during this sequence, but must not receive jobs during the transition.

Review all branches, tags, pull requests, issues, releases, artifacts, and Actions logs for private data before publication; changing visibility can expose earlier history. After publication, run **Linux checks** manually and verify that both Rust versions pass. Verify that an external fork revision waits for approval before any job starts. If that gate cannot be confirmed, disable Actions until it is resolved. Public visibility alone does not activate the intended approval policy.

## Local validation and contributions

Fatal-path tests disable core dumps in the child immediately before its expected abort. On Linux they also disable dumpability so piped crash handlers stay asleep. Keep this out of production and socket-fixture owners; see [the intentional-abort inventory and focused audit](docs/testing-aborts.md).

The memory-residency and real-time scheduling smoke tests are opt-in because they require host resource limits or services that ordinary CI runners do not provide. On a configured host, run the memory check separately with `cargo test --locked --bin cv1touchd locked_memory_stays_resident_once_touched -- --ignored`; it locks memory for that test process. Inspect the other ignored tests' requirements before running them. The daemon reports a fallback when memory locking or real-time scheduling is unavailable.

Preserve upstream copyright and license notices. Update third-party provenance when changing the pinned source. Do not submit copied SteamVR/Oculus assets, personal calibration, raw room captures, device serial numbers, credentials, or workstation-specific settings. Any public example configuration must use fictional identifiers and synthetic geometry.

Document user-visible changes in `CHANGELOG.md`. ABI/recording-format changes require coordinated reader/writer compatibility tests. Performance claims need a baseline, exact method, hardware context, and uncertainty. Keep the default setup portable; optional optimizations must fail visibly or fall back safely.

Maintainers: follow the [release checklist](docs/releasing.md) for version metadata, validation, source archives, and the Windows installer.

By submitting a contribution, you agree to license your original changes under the repository's Apache License 2.0. Third-party code must retain its applicable license.
