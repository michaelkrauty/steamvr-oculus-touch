// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The installed service controls and the CLI commands the windows run, off the GUI thread.
use crate::{
    client::{self, Output},
    preview_client::PreviewError,
};
use serde_json::Value;
use std::io;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Duration;

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

/// Runs `work` on its own thread and hands its result to `done` there.
pub fn background<T: Send + 'static>(
    work: impl FnOnce() -> T + Send + 'static,
    done: impl FnOnce(T) + Send + 'static,
) {
    std::thread::spawn(move || done(work()));
}

/// A diagnostic report the CLI saved.
#[derive(Clone, Debug, PartialEq)]
pub struct Saved {
    pub path: PathBuf,
    pub bytes: u64,
    /// What the file holds, to read before sending it to anyone.
    pub notice: String,
}

impl Saved {
    /// "Saved NAME (SIZE)".
    pub fn summary(&self) -> String {
        let name = self.path.file_name().map_or_else(
            || self.path.display().to_string(),
            |n| n.to_string_lossy().into_owned(),
        );
        format!("Saved {name} ({})", size(self.bytes))
    }
}

fn size(bytes: u64) -> String {
    match bytes {
        0..1000 => format!("{bytes} B"),
        1000..1_000_000 => format!("{:.0} kB", bytes as f64 / 1e3),
        _ => format!("{:.1} MB", bytes as f64 / 1e6),
    }
}

const TOO_OLD: &str =
    "This version of cv1touch cannot do that from this window. Update or rebuild cv1touch.";

/// `report --json`'s one line, or why there is no report.
pub fn parse_report(out: &Output) -> Result<Saved, String> {
    let Ok(v) = serde_json::from_slice::<Value>(&out.stdout) else {
        // An older CLI rejects --json with its usage text and exit 2.
        return Err(if out.code == Some(2) || out.stderr.is_empty() {
            TOO_OLD.to_owned()
        } else {
            out.stderr.clone()
        });
    };
    match (v["ok"].as_bool(), v["path"].as_str(), v["bytes"].as_u64()) {
        (Some(true), Some(path), Some(bytes)) if out.code == Some(0) => Ok(Saved {
            path: path.into(),
            bytes,
            notice: v["notice"].as_str().unwrap_or_default().to_owned(),
        }),
        _ => Err(v["error"].as_str().map_or_else(
            || "cv1touch report returned invalid data".to_owned(),
            str::to_owned,
        )),
    }
}

/// Saves a diagnostic report without showing it; collecting the logs can take a while.
pub fn report() -> Result<Saved, String> {
    parse_report(&client::run(
        &["report", "--json", "--no-open"],
        Duration::from_secs(120),
    )?)
}

/// Shows the folder that holds `path` in the file manager.
pub fn open_folder(path: &Path) -> io::Result<()> {
    #[cfg(windows)]
    let child = {
        use std::os::windows::process::CommandExt;
        // Explorer reads its own command line: the path stays quoted after the comma.
        Command::new("explorer.exe")
            .raw_arg(format!("/select,\"{}\"", path.display()))
            .spawn()?
    };
    #[cfg(not(windows))]
    let child = Command::new("xdg-open")
        .arg(path.parent().unwrap_or(Path::new(".")))
        .stdin(std::process::Stdio::null())
        .spawn()?;
    background(move || drop(child.wait_with_output()), |_| {});
    Ok(())
}

/// `finish-setup --json` on Windows: the sensors' driver and the SteamVR registration.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Finish {
    pub ok: bool,
    pub reboot_needed: bool,
    /// Each step run, with its exit code.
    pub steps: Vec<(String, i64)>,
    pub remaining: Vec<String>,
    pub error: Option<String>,
}

pub fn parse_finish(out: &Output) -> Finish {
    let Ok(v) = serde_json::from_slice::<Value>(&out.stdout) else {
        return Finish {
            error: Some(if out.stderr.is_empty() || out.code == Some(2) {
                TOO_OLD.to_owned()
            } else {
                out.stderr.clone()
            }),
            ..Default::default()
        };
    };
    Finish {
        ok: v["ok"] == true,
        reboot_needed: v["reboot_needed"] == true,
        steps: v["steps"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|s| Some((s["step"].as_str()?.to_owned(), s["exit"].as_i64()?)))
            .collect(),
        remaining: v["remaining"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|s| s.as_str().map(str::to_owned))
            .collect(),
        error: v["error"].as_str().map(str::to_owned),
    }
}

/// Runs `finish-setup --json`; the driver install can take minutes.
pub fn finish_setup() -> Finish {
    match client::run(&["finish-setup", "--json"], Duration::from_secs(600)) {
        Ok(out) => parse_finish(&out),
        Err(error) => Finish {
            error: Some(error),
            ..Default::default()
        },
    }
}

/// The note the daemon adds when a setting applied but could not be saved.
fn unsaved(line: &str) -> Option<&str> {
    let start = line.find("(not saved: ")? + "(not saved: ".len();
    Some(line[start..].trim_end().trim_end_matches(')'))
}

/// `pin-base-station on|off`'s output: the setting now, or why it did not change.
pub fn parse_pin(out: &Output) -> Result<bool, String> {
    let stdout = String::from_utf8_lossy(&out.stdout);
    let line = stdout.trim();
    let on = if line.starts_with("Base-station pinning: on") {
        true
    } else if line.starts_with("Base-station pinning: off") {
        false
    } else if out.stderr.is_empty() || out.code == Some(2) {
        return Err(TOO_OLD.to_owned());
    } else {
        return Err(out.stderr.trim_start_matches("cv1touch: ").to_owned());
    };
    match unsaved(line) {
        Some(error) if out.code == Some(0) => Err(format!(
            "Pinning is {} for now, but it could not be saved: {error}",
            if on { "on" } else { "off" }
        )),
        None if out.code == Some(0) => Ok(on),
        _ => Err(out.stderr.clone()),
    }
}

/// Turns base-station pinning on or off, with the daemon running or not.
pub fn pin_base_station(on: bool) -> Result<bool, String> {
    parse_pin(&client::run(
        &["pin-base-station", if on { "on" } else { "off" }],
        Duration::from_secs(10),
    )?)
}

/// The daemon's reply to `tilt DEGREES`.
pub fn parse_tilt(reply: &str) -> Result<(), String> {
    let reply = reply.trim();
    match reply.strip_prefix("ok ") {
        Some(line) => match unsaved(line) {
            Some(error) => Err(format!(
                "The angle applies now, but it could not be saved: {error}"
            )),
            None => Ok(()),
        },
        None => Err(reply.strip_prefix("err ").unwrap_or(reply).to_owned()),
    }
}

/// Sets the controller angle through the running daemon, which saves it.
pub fn set_tilt(deg: f64) -> Result<(), String> {
    match crate::preview_client::request_once(&format!("tilt {deg:.1}\n")) {
        Ok(reply) => parse_tilt(&reply),
        Err(PreviewError::DaemonStopped) => {
            Err("cv1touchd isn't running: start it to change the angle.".into())
        }
        Err(PreviewError::UnsupportedDaemon) => Err(TOO_OLD.into()),
        Err(PreviewError::Other(error)) => Err(error),
    }
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

    fn output(code: i32, stdout: &str, stderr: &str) -> Output {
        Output {
            code: Some(code),
            stdout: stdout.as_bytes().to_vec(),
            stderr: stderr.into(),
        }
    }

    #[test]
    fn report_json_gives_the_file_or_the_reason() {
        let saved = parse_report(&output(
            0,
            r#"{"ok":true,"path":"/home/u/Desktop/cv1touch-report-2026-11-25-232800-UTC.zip","bytes":1430000,"opened":false,"notice":"It holds:\n- logs"}"#,
            "",
        ))
        .unwrap();
        assert_eq!(
            saved.summary(),
            "Saved cv1touch-report-2026-11-25-232800-UTC.zip (1.4 MB)"
        );
        assert_eq!(saved.notice, "It holds:\n- logs");
        assert_eq!(size(183_245), "183 kB");
        assert_eq!(size(999), "999 B");
        let error =
            "There is already a file at /home/u/Desktop/x.zip, so the report was not saved.";
        assert_eq!(
            parse_report(&output(
                1,
                &format!(r#"{{"ok":false,"path":"/home/u/Desktop/x.zip","error":"{error}"}}"#),
                ""
            )),
            Err(error.to_owned())
        );
        // An older CLI: usage on stderr, exit 2, nothing on stdout.
        assert_eq!(
            parse_report(&output(2, "", "usage: cv1touch report [--out PATH]")),
            Err(TOO_OLD.to_owned())
        );
        assert!(parse_report(&output(1, "{\"ok\":true}", "")).is_err());
    }

    #[test]
    fn pin_and_tilt_replies_report_what_applied() {
        assert_eq!(
            parse_pin(&output(0, "Base-station pinning: on (SteamVR starts tracking the headset from a base station headset alignment measured)\n", "")),
            Ok(true)
        );
        assert_eq!(
            parse_pin(&output(
                0,
                "Base-station pinning: off (SteamVR picks the base station itself)\n",
                ""
            )),
            Ok(false)
        );
        let unsaved = parse_pin(&output(
            0,
            "Base-station pinning: off (SteamVR picks the base station itself) (not saved: disk full)\n",
            "",
        ))
        .unwrap_err();
        assert!(
            unsaved.contains("off for now") && unsaved.ends_with("disk full"),
            "{unsaved}"
        );
        assert_eq!(
            parse_pin(&output(1, "", "cv1touch: cv1touchd: err unknown command")),
            Err("cv1touchd: err unknown command".to_owned())
        );
        assert_eq!(parse_pin(&output(2, "", "usage")), Err(TOO_OLD.to_owned()));
        assert_eq!(
            parse_tilt("ok tilt: the controllers are shown as tracked (no tilt)\n"),
            Ok(())
        );
        assert!(
            parse_tilt("ok tilt: pitched 5.0 deg up (not saved: read-only)\n")
                .unwrap_err()
                .ends_with("read-only")
        );
        assert_eq!(
            parse_tilt("err tilt must be a number of degrees within +-45\n"),
            Err("tilt must be a number of degrees within +-45".to_owned())
        );
    }

    #[test]
    fn finish_setup_json_lists_steps_and_what_remains() {
        let finish = parse_finish(&output(
            3,
            r#"{"ok":false,"reboot_needed":true,"steps":[{"step":"Checking that SteamVR is closed","exit":0},{"step":"Giving the sensors their driver","exit":3}],"remaining":["Restart Windows to finish giving the sensors their driver."]}"#,
            "",
        ));
        assert!(finish.reboot_needed && !finish.ok && finish.error.is_none());
        assert_eq!(
            finish.steps[1],
            ("Giving the sensors their driver".into(), 3)
        );
        assert_eq!(finish.remaining.len(), 1);
        let linux = parse_finish(&output(
            1,
            "",
            "cv1touch: finish-setup is for the Windows installer. On Linux, use scripts/install.py.",
        ));
        assert!(linux.error.unwrap().contains("Windows installer"));
    }

    /// Against a built `cv1touch` with no daemon, in a scratch home that cannot reach a live
    /// one: `CV1TOUCH_CLI=/path/to/cv1touch cargo test -- --ignored built_cli`.
    #[cfg(target_os = "linux")]
    #[test]
    #[ignore = "needs a built cv1touch in CV1TOUCH_CLI"]
    fn the_built_cli_answers_as_the_windows_expect() {
        use crate::setup_client::SetupClient;
        let cli = PathBuf::from(std::env::var_os("CV1TOUCH_CLI").expect("CV1TOUCH_CLI"));
        let home = std::env::temp_dir().join(format!("cv1touch-cli-{}", std::process::id()));
        for (var, dir) in [
            ("HOME", ""),
            ("XDG_CONFIG_HOME", ".config"),
            ("XDG_DATA_HOME", ".local/share"),
            ("XDG_STATE_HOME", ".local/state"),
            ("XDG_CACHE_HOME", ".cache"),
            ("XDG_RUNTIME_DIR", "run"),
        ] {
            std::fs::create_dir_all(home.join(dir)).unwrap();
            // SAFETY: this test runs alone (--ignored), before any thread reads them.
            unsafe { std::env::set_var(var, home.join(dir)) };
        }
        let run = |args: &[&str]| client::run_at(&cli, args, Duration::from_secs(60)).unwrap();
        // Pinning without a daemon writes settings.json, as the Settings tab reads it.
        assert_eq!(parse_pin(&run(&["pin-base-station", "off"])), Ok(false));
        let config = crate::config_stats::ConfigStats::load_from(&home.join(".config/cv1touch"));
        assert!(!config.pin_base_station());
        assert_eq!(parse_pin(&run(&["pin-base-station", "on"])), Ok(true));
        assert_eq!(
            set_tilt(5.0).unwrap_err(),
            "cv1touchd isn't running: start it to change the angle."
        );
        let saved = parse_report(&run(&["report", "--json", "--no-open"])).unwrap();
        assert_eq!(std::fs::metadata(&saved.path).unwrap().len(), saved.bytes);
        assert!(saved.path.starts_with(&home) && !saved.notice.is_empty());
        let out = saved.path.to_str().unwrap();
        let refused = parse_report(&run(&["report", "--out", out, "--json", "--no-open"]));
        assert!(refused.unwrap_err().contains("already a file"));
        assert!(parse_finish(&run(&["finish-setup", "--json"]))
            .error
            .unwrap()
            .contains("Windows installer"));
        let mut setup = SetupClient::spawn(&cli, &["setup", "--events"], || {}).unwrap();
        let wait = |setup: &SetupClient, done: &dyn Fn(&crate::setup_client::Guide) -> bool| {
            for _ in 0..1000 {
                let guide = setup.guide();
                if done(&guide) {
                    return guide;
                }
                std::thread::sleep(Duration::from_millis(20));
            }
            panic!("timed out: {:?}", setup.guide());
        };
        let guide = wait(&setup, &|g| {
            g.prompt.as_ref().is_some_and(|p| p.id == "menu")
        });
        let state = guide.state.unwrap();
        assert!(!state.daemon_running && !state.ready);
        assert!(state.checks.iter().any(|c| c.name == "daemon" && !c.ok));
        assert_eq!(state.stages.len(), 5);
        assert!(setup.send("run nothing"));
        wait(&setup, &|g| g.rejected.is_some());
        assert!(setup.send("quit"));
        let guide = wait(&setup, &|g| g.done.is_some());
        assert_eq!(guide.done, Some((false, None)));
        drop(setup);
        std::fs::remove_dir_all(home).unwrap();
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
