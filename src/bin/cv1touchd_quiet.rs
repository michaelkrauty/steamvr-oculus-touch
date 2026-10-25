// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! `cv1touchd-quiet`: starts `cv1touchd --supervise`, the program beside it, with no console
//! window, and returns. It is what Windows runs at logon for the installed cv1touch: a console
//! program started there opens a window, while this one has none (it is built for the windows
//! subsystem) and gives the supervisor none either. Its arguments go to the supervisor. When a
//! supervisor already runs for this user, the new one sees its lock and ends at once, quietly;
//! when the supervisor cannot be started, or ends at once with an error, that is said in a
//! message box, since there is no console to print to. On Linux systemd runs the daemon; here
//! the supervisor just inherits the terminal.
#![cfg_attr(all(windows, not(test)), windows_subsystem = "windows")]

use std::path::Path;
use std::process::{Command, ExitCode};
use std::time::Duration;

use cv1touch::platform::process;

#[cfg(windows)]
#[path = "cv1touchd_quiet/windows.rs"]
mod windows;

const DAEMON: &str = "cv1touchd";

/// How long an early failure of the supervisor is waited for: refusing its arguments, or its log
/// directory, takes it well under this.
const EARLY: Duration = Duration::from_millis(500);

/// The supervisor to start: `cv1touchd --supervise ARGS`, the daemon beside `launcher`, without a
/// console window of its own.
fn supervisor(launcher: &Path, args: &[String]) -> Command {
    let daemon = launcher.with_file_name(format!("{DAEMON}{}", std::env::consts::EXE_SUFFIX));
    let mut command = Command::new(daemon);
    command.arg("--supervise").args(args);
    process::without_window(&mut command);
    command
}

#[cfg(not(windows))]
fn launch(command: &mut Command) -> std::io::Result<Option<u32>> {
    let mut child = command.spawn()?;
    std::thread::sleep(EARLY);
    Ok(child
        .try_wait()?
        .map(|status| status.code().unwrap_or(1) as u32))
}

#[cfg(windows)]
fn launch(command: &mut Command) -> std::io::Result<Option<u32>> {
    windows::launch(command, EARLY)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let launcher = match std::env::current_exe() {
        Ok(path) => path,
        Err(e) => return fail(format!("cannot tell where it is installed: {e}")),
    };
    let mut command = supervisor(&launcher, &args);
    // a supervisor that ends at once had a reason: another one running (fine), or a failure
    match launch(&mut command) {
        Err(e) => fail(format!(
            "cannot start {}: {e}",
            command.get_program().to_string_lossy()
        )),
        Ok(Some(code)) if code != 0 => fail(format!(
            "the supervisor ended at once (exit {code}); Check cv1touch in the Start menu says what \
             is missing"
        )),
        _ => ExitCode::SUCCESS,
    }
}

fn fail(what: String) -> ExitCode {
    process::alert(
        "cv1touch",
        &format!("cv1touch did not start: {what}\nInstall cv1touch again, or send a report."),
    );
    ExitCode::FAILURE
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The daemon is the program beside the launcher, run as the supervisor with the arguments
    /// given.
    #[test]
    fn the_supervisor_runs_the_daemon_beside_the_launcher() {
        let launcher = Path::new("/opt/cv1touch/bin")
            .join(format!("cv1touchd-quiet{}", std::env::consts::EXE_SUFFIX));
        let command = supervisor(&launcher, &["--hmd-leds".to_string(), "off".to_string()]);
        assert_eq!(
            Path::new(command.get_program()),
            Path::new("/opt/cv1touch/bin")
                .join(format!("cv1touchd{}", std::env::consts::EXE_SUFFIX))
        );
        let args: Vec<_> = command
            .get_args()
            .map(|a| a.to_string_lossy().into_owned())
            .collect();
        assert_eq!(args, ["--supervise", "--hmd-leds", "off"]);
    }
}
