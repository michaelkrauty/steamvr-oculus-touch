# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
# Disposable hosted CI only: install as an actual non-administrator with a fresh profile.
param([Parameter(Mandatory=$true)][string]$Installer)
$ErrorActionPreference = 'Stop'
if ($env:GITHUB_ACTIONS -ne 'true') { throw 'This changes local accounts; run only on a disposable hosted CI runner' }
$name = 'cv1touch_test'
if (Get-LocalUser -Name $name -ErrorAction SilentlyContinue) { throw 'Test account already exists; refusing to reuse it' }
$work = Join-Path $env:PUBLIC ('cv1touch-test-' + [guid]::NewGuid().ToString('N'))
$destination = Join-Path $PWD 'target\windows-install-test'
$user = $null
try {
    $password = ConvertTo-SecureString ('Cv1!' + [guid]::NewGuid().ToString('N')) -AsPlainText -Force
    $user = New-LocalUser -Name $name -Password $password -PasswordNeverExpires -UserMayNotChangePassword
    Add-LocalGroupMember -SID 'S-1-5-32-545' -Member $user
    New-Item -ItemType Directory -Path $work | Out-Null
    & icacls.exe $work /grant "${name}:(OI)(CI)M" | Out-Null
    if ($LASTEXITCODE -ne 0) { throw 'Could not grant the test user access to its test files' }
    Copy-Item $Installer (Join-Path $work 'setup.exe')
    Copy-Item (Join-Path $PSScriptRoot 'test_install.py') $work
    $python = (Get-Command python.exe).Source
    $credentials = [pscredential]::new("$env:COMPUTERNAME\$name", $password)
    Start-Service seclogon
    $process = Start-Process -FilePath $python -Credential $credentials -LoadUserProfile `
        -ArgumentList 'test_install.py setup.exe --disposable-runner' -WorkingDirectory $work `
        -RedirectStandardOutput (Join-Path $work 'stdout.log') -RedirectStandardError (Join-Path $work 'stderr.log') -PassThru
    if (-not $process.WaitForExit(900000)) {
        # Only the test tree under this newly created account, never a machine-wide image kill.
        & taskkill.exe /F /T /PID $process.Id | Out-Null
        throw 'Standard-user installer checks exceeded 15 minutes'
    }
    $process.Refresh()
    $code = $process.ExitCode
    Get-Content (Join-Path $work 'stdout.log')
    Get-Content (Join-Path $work 'stderr.log')
    if ($code -ne 0) { throw "Standard-user installer checks exited $code" }
} finally {
    New-Item -ItemType Directory -Force -Path $destination | Out-Null
    if (Test-Path "$work\target\windows-install-test") {
        Copy-Item "$work\target\windows-install-test\*" $destination -Recurse -Force
    }
    foreach ($log in 'stdout.log', 'stderr.log') {
        if (Test-Path "$work\$log") { Copy-Item "$work\$log" $destination -Force }
    }
    if ($user) {
        # The disposable account may have a supervisor left by a failed test. End only its
        # processes before deleting its profile and account; never the build runner's daemon.
        Get-Process -IncludeUserName | Where-Object { $_.UserName -eq "$env:COMPUTERNAME\$name" } | Stop-Process -Force -ErrorAction SilentlyContinue
        Start-Sleep 2
        Get-CimInstance Win32_UserProfile | Where-Object { $_.SID -eq $user.SID.Value } | Remove-CimInstance
        Remove-LocalUser -Name $name
    }
    if (Test-Path $work) { Remove-Item $work -Recurse -Force }
}
