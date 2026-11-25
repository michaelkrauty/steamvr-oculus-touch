# Releasing

A release is an annotated `vMAJOR.MINOR.PATCH` tag and a GitHub release. Linux ships source; Windows ships an unsigned installer built by CI.

## Prepare

1. Set the version in `Cargo.toml`, the `cv1touch` entry of `Cargo.lock`, `meson.build`, `vcpkg.json` and `CITATION.cff` (with the release date and URL).
2. Move the `Unreleased` entries of `CHANGELOG.md` under a dated release heading, leaving `Unreleased` empty. Update the version in `README.md`, `docs/setup.md`, `docs/building-windows.md` and `windows/installer/cv1touch.iss`. `python3 -m unittest discover -s tests -p test_release.py` checks that they agree.
3. Run `scripts/check.sh`, push the release commit, and wait for both **Linux checks** and **Windows checks** to pass on that exact commit. Skipped, queued or awaiting-approval checks are not passes. Hardware checks are manual.

## Publish

Tag the validated commit and push the tag. Never move a published tag.

```sh
version=0.6.0
tag=v$version
git tag -a "$tag" -m "$tag"
git push origin "$tag"
```

The Windows workflow rebuilds the installer for the tag and, once its library, driver and installer jobs pass, uploads `cv1touch-setup-$version.exe` to a draft release (creating the draft if none exists). Wait for the whole workflow.

Build the source archive from the tag, not the working tree:

```sh
mkdir -p target/dist
git archive --format=tar --prefix="steamvr-oculus-touch-$version/" "$tag" \
  > "target/dist/steamvr-oculus-touch-$version.tar" &&
  gzip -n "target/dist/steamvr-oculus-touch-$version.tar"
```

Extract it into a fresh directory and run its release test and a build there. A smoke install uses a temporary prefix with `--no-service --no-steamvr`.

Download the draft's installer into `target/dist`, run `sha256sum` on both assets inside `target/dist` into `SHA256SUMS`, verify it, and upload the tarball and `SHA256SUMS` to the draft.

Write short release notes: what changed and what was verified, how to upgrade, whether the daemon and driver must be updated together, and that the installer is unsigned. Link the tagged changelog and setup guide. Publish the draft after checking the assets, checksums, CI and notes, then check the public release page and downloads. Never replace a published asset.
