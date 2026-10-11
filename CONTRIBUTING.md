# Contributing

Start with an issue describing the observed behavior, reproduction, hardware, distribution, and version. Search existing issues first. Keep pull requests focused on one logical change and use conventional commit messages (`fix:`, `feat:`, `docs:`, and similar).

Install the dependencies in [setup](docs/setup.md), then run the checks in `scripts/check.sh`. Bug fixes should include a regression that fails before the fix. Hardware-dependent tests should be explicitly marked and describe the required procedure; do not replace missing hardware coverage with unconditional passes.

CI runs `scripts/check.sh` with Rust 1.85.0 and stable. Public repositories and fork pull requests use GitHub-hosted Linux runners. While the repository is private, trusted same-repository branches may use a dedicated Linux runner with preinstalled native dependencies. Keep GitHub's setting to run workflows from private fork pull requests disabled when using a persistent runner.

## Preparing a private repository for publication

Before changing visibility to public, finish active CI jobs and remove every persistent self-hosted runner registration from the repository's **Settings → Actions → Runners**, including offline runners. Verify the list is empty before changing visibility. Workflow conditions and fork-approval settings are not a security boundary: a pull request can change its own workflow. See [GitHub's self-hosted runner security guidance](https://docs.github.com/en/actions/reference/security/secure-use#hardening-for-self-hosted-runners).

Review all branches, tags, pull requests, issues, releases, artifacts, and Actions logs for private data before publication; changing visibility can expose earlier history. After publication, verify that both Rust versions pass on GitHub-hosted runners. Do not attach a persistent self-hosted runner to the public repository.

## Local validation and contributions

The memory-residency and real-time scheduling smoke tests are opt-in because they require host resource limits or services that ordinary CI runners do not provide. On a configured host, run the memory check separately with `cargo test --locked --bin cv1touchd locked_memory_stays_resident_once_touched -- --ignored`; it locks memory for that test process. Inspect the other ignored tests' requirements before running them. The daemon reports a fallback when memory locking or real-time scheduling is unavailable.

Preserve upstream copyright and license notices. Update third-party provenance when changing the pinned source. Do not submit copied SteamVR/Oculus assets, personal calibration, raw room captures, device serial numbers, credentials, or workstation-specific settings. Any public example configuration must use fictional identifiers and synthetic geometry.

Document user-visible changes in `CHANGELOG.md`. ABI/recording-format changes require coordinated reader/writer compatibility tests. Performance claims need a baseline, exact method, hardware context, and uncertainty. Keep the default setup portable; optional optimizations must fail visibly or fall back safely.

By submitting a contribution, you agree to license your original changes under the repository's Apache License 2.0. Third-party code must retain its applicable license.
