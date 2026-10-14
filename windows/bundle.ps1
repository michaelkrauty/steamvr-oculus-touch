<#
.SYNOPSIS
Collects a Windows release build of cv1touch into the flat bundle the installer packs.

.DESCRIPTION
The same files scripts/install.py stages on Linux, laid out as the installed program is:
bin\ (cv1touchd.exe, cv1touch.exe, cv1touchd-quiet.exe and the engine's DLLs, which build.rs
puts beside the executables), lib\cv1touch\ (the SteamVR driver and its manifest, which
`cv1touch steamvr install` copies from there), and the licences: the repository's, and for what a
Windows build links in and Linux leaves to the distribution, the vcpkg ports' copyright files
with their versions (third_party\licenses\<port>-copyright.txt, windows-bundle.txt) and the Rust
crates with their licences (third_party\licenses\rust-crates.txt and rust\<crate>-<version>\).
Stops at the first missing file.

.PARAMETER Target
Cargo's target directory, holding release\ from `cargo build --release --bins`.

.PARAMETER Deps
The vcpkg ports the engine was built with ($env:CV1TOUCH_DEPS, an x-install-root triplet
directory).

.PARAMETER Out
The bundle directory; replaced if it exists. Must not contain the repository.
#>
param(
    [string]$Target = (Join-Path $PSScriptRoot '..\target'),
    [string]$Deps = $env:CV1TOUCH_DEPS,
    [string]$Out = (Join-Path $PSScriptRoot '..\target\bundle')
)
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$release = Join-Path $Target 'release'
$marker = Join-Path $release 'cv1touch-native-dir'
if (-not (Test-Path $marker)) { throw "no release build with the engine: $marker is missing" }
$native = (Get-Content $marker -Raw).Trim()
if (-not $Deps) { throw 'pass -Deps or set CV1TOUCH_DEPS: the vcpkg ports the engine was built with' }
$status = Join-Path (Split-Path $Deps) 'vcpkg\status'
if (-not (Test-Path $status)) { throw "missing: $status (the vcpkg installation's package list)" }

if (Test-Path $Out) {
    $resolved = (Resolve-Path $Out).Path.TrimEnd('\')
    if ($root -eq $resolved -or $root.StartsWith($resolved + '\')) { throw "refusing to replace $resolved, which holds the repository" }
    Remove-Item -Recurse -Force $Out
}
foreach ($dir in 'bin', 'lib\cv1touch', 'third_party\licenses') {
    New-Item -ItemType Directory -Path (Join-Path $Out $dir) -Force | Out-Null
}
$Out = (Resolve-Path $Out).Path

function Take([string]$From, [string]$To) {
    if (-not (Test-Path $From -PathType Leaf)) { throw "missing: $From" }
    New-Item -ItemType Directory -Path (Split-Path $To) -Force | Out-Null
    Copy-Item $From $To
}

foreach ($exe in 'cv1touchd.exe', 'cv1touch.exe', 'cv1touchd-quiet.exe') {
    Take (Join-Path $release $exe) (Join-Path $Out "bin\$exe")
}
foreach ($dll in 'openhmd-0.dll', 'libusb-1.0.dll') {
    Take (Join-Path $release $dll) (Join-Path $Out "bin\$dll")
}
Take (Join-Path $native 'driver\driver_cv1touch.dll') (Join-Path $Out 'lib\cv1touch\driver_cv1touch.dll')
Take (Join-Path $root 'driver\cv1touch\driver.vrdrivermanifest') (Join-Path $Out 'lib\cv1touch\driver.vrdrivermanifest')
foreach ($file in 'LICENSE', 'NOTICE', 'THIRD_PARTY_NOTICES.md') {
    Take (Join-Path $root $file) (Join-Path $Out $file)
}
foreach ($file in 'openhmd\LICENSE', 'licenses\nxjson-MIT.txt', 'licenses\miniz-Unlicense.txt', 'openvr\LICENSE', 'openvr\PROVENANCE.md') {
    Take (Join-Path $root "third_party\$file") (Join-Path $Out "third_party\$file")
}

# The libraries the engine's DLLs carry (libusb as its own DLL, the others inside openhmd-0.dll),
# with the version vcpkg built: the status file lists "Package: <port>" and "Version: <version>".
$ports = 'libusb', 'opencv4', 'libjpeg-turbo', 'hidapi', 'zlib'
$versions = @{}
$package = ''
foreach ($line in Get-Content $status) {
    if ($line -match '^Package: (.+)$') { $package = $Matches[1] }
    elseif ($line -match '^Version: (.+)$' -and $package) { $versions[$package] = $Matches[1] }
}
$listing = @('The libraries the Windows build links in, as vcpkg built them; each one''s terms are in <port>-copyright.txt beside this file.', '')
foreach ($port in $ports) {
    if (-not $versions.ContainsKey($port)) { throw "vcpkg did not install $port ($status)" }
    Take (Join-Path $Deps "share\$port\copyright") (Join-Path $Out "third_party\licenses\$port-copyright.txt")
    $listing += "$port $($versions[$port])"
}
Set-Content -Path (Join-Path $Out 'third_party\licenses\windows-bundle.txt') -Value $listing

# The Rust crates the build uses for this target: linked into the executables, or building them.
# `--filter-platform` narrows the dependency graph (resolve), not the package list, so the graph
# picks the packages. Each crate's licence files are copied beside its line, since MIT and BSD
# terms ask for the notice and the text, not only the name.
Push-Location $root
try {
    $metadata = cargo metadata --format-version 1 --locked --filter-platform x86_64-pc-windows-msvc | ConvertFrom-Json
    if ($LASTEXITCODE -ne 0) { throw 'cargo metadata failed' }
} finally {
    Pop-Location
}
$inGraph = @{}
foreach ($node in $metadata.resolve.nodes) { $inGraph[$node.id] = $true }
$crates = @('The Rust crates the Windows build uses (Cargo.lock pins them): linked into the executables, or building them. Each line names the licence the crate declares; its licence files, where the crate ships them, are in rust\<crate>-<version>\ beside this file.', '')
foreach ($crate in ($metadata.packages | Where-Object { $_.source -and $inGraph[$_.id] } | Sort-Object name, version)) {
    $terms = if ($crate.license) { $crate.license } else { "see $($crate.repository)" }
    $files = Get-ChildItem (Split-Path $crate.manifest_path) -File | Where-Object { $_.Name -match '^(LICEN[CS]E|COPYING|NOTICE)' }
    $where = if ($files) { " (rust\$($crate.name)-$($crate.version)\)" } else { ' (no licence file in the crate)' }
    $crates += "$($crate.name) $($crate.version): $terms$where"
    foreach ($file in $files) {
        Take $file.FullName (Join-Path $Out "third_party\licenses\rust\$($crate.name)-$($crate.version)\$($file.Name)")
    }
}
Set-Content -Path (Join-Path $Out 'third_party\licenses\rust-crates.txt') -Value $crates

Get-ChildItem -Recurse -File $Out | ForEach-Object { $_.FullName.Substring($Out.Length + 1) }
