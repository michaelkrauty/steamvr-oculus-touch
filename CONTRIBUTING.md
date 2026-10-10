# Contributing

Start with an issue describing the observed behavior, reproduction, hardware, distribution, and version. Search existing issues first. Keep pull requests focused on one logical change and use conventional commit messages (`fix:`, `feat:`, `docs:`, and similar).

Install the dependencies in [setup](docs/setup.md), then run the checks in `scripts/check.sh`. Bug fixes should include a regression that fails before the fix. Hardware-dependent tests should be explicitly marked and describe the required procedure; do not replace missing hardware coverage with unconditional passes.

Preserve upstream copyright and license notices. Update third-party provenance when changing the pinned source. Do not submit copied SteamVR/Oculus assets, personal calibration, raw room captures, device serial numbers, credentials, or workstation-specific settings. Any public example configuration must use fictional identifiers and synthetic geometry.

Document user-visible changes in `CHANGELOG.md`. ABI/recording-format changes require coordinated reader/writer compatibility tests. Performance claims need a baseline, exact method, hardware context, and uncertainty. Keep the default setup portable; optional optimizations must fail visibly or fall back safely.

By submitting a contribution, you agree to license your original changes under the repository's Apache License 2.0. Third-party code must retain its applicable license.
