# Releasing cv1touch

Releases use an annotated `vMAJOR.MINOR.PATCH` Git tag and a GitHub release. Linux
ships source to build against the user's native libraries; Windows ships an
unsigned installer built and smoke-tested by CI. Do not package personal settings,
captures, calibration, or an installed workstation directory.

## Prepare

1. Choose the version and update `Cargo.toml`, the `cv1touch` entry in `Cargo.lock`,
   `meson.build`, `vcpkg.json`, and `CITATION.cff` (including release date and URL).
2. Put the accumulated `CHANGELOG.md` entries under a dated release heading, leaving
   an empty `Unreleased` section. Update the release examples in `README.md`,
   `docs/setup.md`, `docs/building-windows.md`, and `windows/installer/cv1touch.iss`.
   `python3 -m unittest discover -s tests -p test_release.py` checks these agree,
   including when run from an extracted source archive.
3. Run `scripts/check.sh`. Check Rust 1.85 and current stable in Linux and Windows
   CI; the Windows workflow covers the engine, standalone driver, library, and
   installation/uninstallation on a clean hosted runner. A skipped, queued, or
   approval-pending check is not a pass. Review ignored tests separately; hardware
   checks remain manual and must not be described as covered by CI.
4. Commit and push the reviewed release revision. Wait for **both** Linux checks
   and Windows checks on that exact revision to pass before tagging it. Do not
   move a published tag to fix a release.

## Package and publish

For example, with `version=0.3.0` and `tag=v$version`, create an annotated tag on
the validated commit, then push that tag. The Windows tag workflow rebuilds and
tests the installer. Its release job waits for the library, standalone driver,
and installer checks and uploads `cv1touch-setup-$version.exe` to a **draft** release
if no release exists. Wait for the entire tag workflow, not just the upload.

Prepare the source asset from the tag, not from the working directory:

```sh
version=0.3.0
tag=v$version
mkdir -p target/dist
git archive --format=tar --prefix="steamvr-oculus-touch-$version/" "$tag" \
  > "target/dist/steamvr-oculus-touch-$version.tar" &&
  gzip -n "target/dist/steamvr-oculus-touch-$version.tar"
```

The `&&` chain stops packaging if archiving fails, rather than masking the error
behind a successful compressor at the end of a pipeline. The
repository tracks the modified OpenHMD sources and OpenVR SDK directly: no
submodule checkout is required. Inspect the archive and extract it into a fresh
directory. Run its metadata check and build/install checks there without relying
on `.git` or the original build directory. A Linux smoke install should use a
temporary prefix with `--no-service --no-steamvr`; never replace a running user's
driver just to test packaging.

Download the tag's Windows installer from the draft release into `target/dist`.
Generate `SHA256SUMS` for that installer and the source tarball, using bare asset
filenames (run `sha256sum` inside `target/dist`). Verify the sums and upload the
source tarball and checksum file to the draft. Keep checksums in sync if an asset
changes **before** publication; do not silently replace published assets.

Write concise release notes with highlights, upgrade instructions, known
limitations, and links to the tagged changelog and setup guide. Explain any need
to update the daemon and SteamVR driver together. State Windows signing and
hardware-validation limits. Publish the draft only after checking the assets,
checksums, exact tagged revision's CI, and release notes; then verify the public
release URL and downloadable files.
