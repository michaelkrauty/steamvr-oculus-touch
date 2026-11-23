// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! What the `cv1touch` binary prints and returns for help and unknown commands.
#![cfg(feature = "engine")]

use std::process::Command;

fn run(args: &[&str]) -> (Option<i32>, String, String) {
    let home = tempfile::tempdir().unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_cv1touch"))
        .args(args)
        .env("HOME", home.path())
        .env("LOCALAPPDATA", home.path())
        .env("XDG_CONFIG_HOME", home.path())
        .env("XDG_STATE_HOME", home.path())
        .env("XDG_RUNTIME_DIR", home.path())
        .output()
        .unwrap();
    let text = |b: Vec<u8>| String::from_utf8(b).unwrap();
    (out.status.code(), text(out.stdout), text(out.stderr))
}

#[test]
fn help_goes_to_stdout_and_succeeds() {
    let (code, stdout, stderr) = run(&["--help"]);
    assert_eq!((code, stderr.as_str()), (Some(0), ""));
    assert!(
        stdout.contains("cv1touch setup ") && stdout.contains("cv1touch help expert"),
        "{stdout}"
    );
}

#[test]
fn command_help_shows_that_command() {
    let (code, stdout, stderr) = run(&["status", "--help"]);
    assert_eq!((code, stderr.as_str()), (Some(0), ""));
    assert_eq!(
        stdout,
        "usage:\n  cv1touch status [--shm NAME] | status --json\n"
    );
}

#[test]
fn an_unknown_command_exits_2() {
    let (code, stdout, stderr) = run(&["bogus"]);
    assert_eq!((code, stdout.as_str()), (Some(2), ""));
    assert_eq!(
        stderr,
        "cv1touch: there is no command \"bogus\". See cv1touch --help.\n"
    );
}

#[test]
fn wrong_arguments_show_that_command_and_exit_2() {
    let (code, stdout, stderr) = run(&["tilt", "1", "2"]);
    assert_eq!((code, stdout.as_str()), (Some(2), ""));
    assert_eq!(stderr, "usage:\n  cv1touch tilt [DEGREES]\n");
}
