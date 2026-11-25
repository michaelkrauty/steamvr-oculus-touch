// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Launch the installed service controls and interactive CLI without blocking the GUI.
use std::io;
use std::process::Command;

use std::sync::atomic::{AtomicBool, Ordering};

static BUSY: AtomicBool = AtomicBool::new(false);

/// A service operation. Completion callbacks run on a worker, not the GUI thread.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ServiceAction {
    Start,
    Restart,
    Stop,
}

impl ServiceAction {
    pub fn label(self) -> &'static str {
        match self {
            Self::Start => "Start daemon",
            Self::Restart => "Restart daemon",
            Self::Stop => "Stop daemon",
        }
    }

    /// What is shown while it runs.
    pub fn progress(self) -> &'static str {
        match self {
            Self::Start => "Starting daemon…",
            Self::Restart => "Restarting daemon…",
            Self::Stop => "Stopping daemon…",
        }
    }

    pub fn tooltip(self) -> &'static str {
        match self {
            Self::Start => "Start cv1touchd, the background service that tracks the controllers. SteamVR shows them once it runs.",
            Self::Restart => "Stop and start cv1touchd again. The controllers drop out of SteamVR for a few seconds and come back.",
            Self::Stop => "Stop cv1touchd. The controllers stop tracking in SteamVR until it is started again.",
        }
    }
}

/// Whether a service action is in flight.
pub fn busy() -> bool {
    BUSY.load(Ordering::Acquire)
}

/// Start one control operation, or return an immediate error if one is already in flight
/// or its worker cannot be started. The callback receives the eventual command result.
/// Send it to the window/tray event loop rather than updating UI from the worker.
pub fn control(
    action: ServiceAction,
    done: impl FnOnce(io::Result<()>) + Send + 'static,
) -> io::Result<()> {
    run_control(move || service(action), done)
}

fn run_control(
    work: impl FnOnce() -> io::Result<()> + Send + 'static,
    done: impl FnOnce(io::Result<()>) + Send + 'static,
) -> io::Result<()> {
    if BUSY
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return Err(io::Error::new(
            io::ErrorKind::WouldBlock,
            "A service action is already in progress",
        ));
    }
    let worker = std::thread::Builder::new()
        .name("cv1touch-control".into())
        .spawn(move || {
            struct Release;
            impl Drop for Release {
                fn drop(&mut self) {
                    BUSY.store(false, Ordering::Release);
                }
            }
            let guard = Release;
            let result = work();
            drop(guard);
            done(result);
        });
    if let Err(error) = worker {
        BUSY.store(false, Ordering::Release);
        return Err(error);
    }
    Ok(())
}

fn bounded_status(command: &mut Command) -> io::Result<std::process::ExitStatus> {
    let mut child = command.spawn()?;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
    loop {
        match child.try_wait() {
            Ok(Some(status)) => return Ok(status),
            Ok(None) if std::time::Instant::now() < deadline => {
                std::thread::sleep(std::time::Duration::from_millis(50))
            }
            result => {
                let _ = child.kill();
                let _ = child.wait();
                return match result {
                    Err(error) => Err(error),
                    _ => Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        "service command timed out",
                    )),
                };
            }
        }
    }
}

fn checked(command: &mut Command, description: &str) -> io::Result<()> {
    let status = bounded_status(command)
        .map_err(|e| io::Error::new(e.kind(), format!("{description}: {e}")))?;
    if status.success() {
        Ok(())
    } else {
        Err(io::Error::other(format!("{description} failed ({status})")))
    }
}

#[cfg(target_os = "linux")]
fn service(action: ServiceAction) -> io::Result<()> {
    let verb = match action {
        ServiceAction::Start => "start",
        ServiceAction::Restart => "restart",
        ServiceAction::Stop => "stop",
    };
    checked(
        Command::new("systemctl").args(["--user", verb, "cv1touchd.service"]),
        &format!("Service {verb}"),
    )
}

#[cfg(windows)]
fn installed(name: &str) -> io::Result<std::path::PathBuf> {
    Ok(std::env::current_exe()?.with_file_name(name))
}

#[cfg(windows)]
fn hidden(command: &mut Command) -> &mut Command {
    use std::os::windows::process::CommandExt;
    command.creation_flags(0x0800_0000) // CREATE_NO_WINDOW
}

#[cfg(windows)]
fn service(action: ServiceAction) -> io::Result<()> {
    if matches!(action, ServiceAction::Stop | ServiceAction::Restart) {
        let status = bounded_status(hidden(Command::new(installed("cv1touch.exe")?).arg("stop")))?;
        // Exit 1 includes an already stopped daemon; 2 means it is still running.
        if !status.success()
            && !(matches!(action, ServiceAction::Restart) && status.code() == Some(1))
        {
            return Err(io::Error::other(format!("Service stop failed ({status})")));
        }
    }
    if matches!(action, ServiceAction::Start | ServiceAction::Restart) {
        checked(
            hidden(&mut Command::new(installed("cv1touchd-quiet.exe")?)),
            "Service start",
        )?;
    }
    Ok(())
}

/// Reap terminal processes without waiting on the GUI thread.
fn reap(mut child: std::process::Child) {
    std::thread::spawn(move || {
        let _ = child.wait();
    });
}

#[cfg(target_os = "linux")]
fn terminal_command(terminal: &str, cli: &std::path::Path, action: &str) -> Command {
    let mut command = Command::new(terminal);
    let separator = if matches!(terminal, "kgx" | "gnome-terminal") {
        "--"
    } else {
        "-e"
    };
    command
        .args([
            separator,
            "sh",
            "-c",
            "\"$1\" \"$2\"; printf \"\\nPress Enter to close this window...\"; read -r _",
            "_",
        ])
        .arg(cli)
        .arg(action);
    command
}

#[cfg(target_os = "linux")]
fn interactive(action: &str) -> io::Result<()> {
    let cli = crate::client::executable();
    let mut last = None;
    for terminal in [
        "x-terminal-emulator",
        "kgx",
        "gnome-terminal",
        "konsole",
        "xterm",
    ] {
        let mut command = terminal_command(terminal, &cli, action);
        match command.spawn() {
            Ok(child) => {
                reap(child);
                return Ok(());
            }
            Err(error) if error.kind() == io::ErrorKind::NotFound => last = Some(error),
            Err(error) => return Err(error),
        }
    }
    Err(last.unwrap_or_else(|| io::Error::new(io::ErrorKind::NotFound, "no terminal found")))
}

#[cfg(windows)]
fn interactive(action: &str) -> io::Result<()> {
    let cli = installed("cv1touch.exe")?;
    // Match the installer's cmd /k shortcuts, keeping output visible after the CLI exits.
    let child = Command::new("cmd.exe")
        .args(["/d", "/k"])
        .arg(format!("\"\"{}\" {action}\"", cli.display()))
        .spawn()?;
    reap(child);
    Ok(())
}

pub fn setup() -> io::Result<()> {
    interactive("setup")
}
pub fn report() -> io::Result<()> {
    interactive("report")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;

    #[test]
    fn control_reports_failure_and_rejects_overlap() {
        let (release, wait) = mpsc::channel();
        let (send, receive) = mpsc::channel();
        run_control(
            move || {
                wait.recv().unwrap();
                Err(io::Error::other("failed"))
            },
            move |r| {
                send.send(r).unwrap();
            },
        )
        .unwrap();
        assert_eq!(
            run_control(|| Ok(()), |_| {}).unwrap_err().kind(),
            io::ErrorKind::WouldBlock
        );
        release.send(()).unwrap();
        assert_eq!(receive.recv().unwrap().unwrap_err().to_string(), "failed");
        let (send, receive) = mpsc::channel();
        run_control(
            || Ok(()),
            move |r| {
                send.send(r).unwrap();
            },
        )
        .unwrap();
        assert!(receive.recv().unwrap().is_ok());
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn terminal_arguments_keep_cli_and_action_separate() {
        for terminal in [
            "kgx",
            "gnome-terminal",
            "x-terminal-emulator",
            "konsole",
            "xterm",
        ] {
            let command = terminal_command(
                terminal,
                std::path::Path::new("/path with spaces/cli"),
                "report",
            );
            let args: Vec<_> = command.get_args().collect();
            assert_eq!(
                args[0],
                if matches!(terminal, "kgx" | "gnome-terminal") {
                    "--"
                } else {
                    "-e"
                }
            );
            assert_eq!(args[1], "sh");
            assert_eq!(args[2], "-c");
            assert_eq!(
                args[3],
                "\"$1\" \"$2\"; printf \"\\nPress Enter to close this window...\"; read -r _"
            );
            assert_eq!(args[4], "_");
            assert_eq!(args[5], "/path with spaces/cli");
            assert_eq!(args[6], "report");
        }
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn checked_reports_exit_and_spawn_errors() {
        assert!(checked(Command::new("sh").args(["-c", "exit 7"]), "test")
            .unwrap_err()
            .to_string()
            .contains("7"));
        assert_eq!(
            checked(&mut Command::new("/nonexistent/cv1touch-test"), "test")
                .unwrap_err()
                .kind(),
            io::ErrorKind::NotFound
        );
    }
}
