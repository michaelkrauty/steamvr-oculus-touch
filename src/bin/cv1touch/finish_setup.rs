// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! One repeatable Windows first-run/repair action, shared by the installer and Start menu.
//! It never calibrates or resets settings. Stop acquisition before changing USB drivers and
//! restart it afterward, including when only some prerequisites could be installed.

#[cfg(any(windows, test))]
#[derive(Clone, Copy, Debug, PartialEq)]
enum Step {
    SteamVrStopped,
    Stop,
    Sensors,
    SteamVr,
    Start,
}

#[cfg(any(windows, test))]
impl Step {
    /// The heading the window and its log show for this step.
    fn label(self) -> &'static str {
        match self {
            Step::SteamVrStopped => "Checking that SteamVR is closed",
            Step::Stop => "Stopping cv1touch",
            Step::Sensors => "Giving the sensors their driver",
            Step::SteamVr => "Adding cv1touch to SteamVR",
            Step::Start => "Starting cv1touch",
        }
    }
}

/// Printed before the reasons setup is incomplete; the installer shows the lines after it.
#[cfg(any(windows, test))]
const INCOMPLETE: &str = "Setup is incomplete:";

/// The exit code, and one line per step that did not finish saying what to do about it.
#[cfg(any(windows, test))]
fn prepare(mut run: impl FnMut(Step) -> i32) -> (i32, Vec<&'static str>) {
    // Refuse before stopping tracking if SteamVR is open or its state is unknown.
    if run(Step::SteamVrStopped) != 0 {
        return (1, vec!["Quit SteamVR, then try again."]);
    }
    if !matches!(run(Step::Stop), 0 | 1) {
        return (
            1,
            vec!["cv1touch could not stop. Restart Windows, then try again."],
        );
    }
    let sensors = run(Step::Sensors);
    let steamvr = run(Step::SteamVr);
    let start = run(Step::Start);
    let mut left = Vec::new();
    match sensors {
        0 => {}
        3 => left.push("Restart Windows to finish giving the sensors their driver."),
        _ => left.push(
            "The sensors did not get their driver. Plug in every sensor and allow Windows' permission prompt.",
        ),
    }
    if steamvr != 0 {
        left.push(
            "cv1touch could not add itself to SteamVR. Install SteamVR, start it once with your headset, then quit it.",
        );
    }
    if start != 0 {
        left.push("cv1touch could not start. Restart Windows: it starts when you log in.");
    }
    // Partial failure must not look successful just because the last command succeeded.
    let code = if steamvr != 0 || start != 0 || !matches!(sensors, 0 | 3) {
        1
    } else {
        sensors // 3: Windows asked for a restart, not ready for calibration yet.
    };
    (code, left)
}

/// `finish-setup --json`'s one object: whether setup finished, whether Windows asks for a
/// restart first, each step with its exit code, and what is left to do.
#[cfg(any(windows, test))]
fn json(code: i32, steps: &[(Step, i32)], left: &[&str]) -> serde_json::Value {
    let reboot_needed = steps
        .iter()
        .any(|&(step, exit)| step == Step::Sensors && exit == 3);
    serde_json::json!({
        "ok": code == 0,
        "reboot_needed": reboot_needed,
        "steps": steps
            .iter()
            .map(|(step, exit)| serde_json::json!({"step": step.label(), "exit": exit}))
            .collect::<Vec<_>>(),
        "remaining": left,
    })
}

/// `json`: one object on standard output instead of the steps' words (`finish-setup --json`);
/// the log has them either way.
#[cfg(windows)]
pub(super) fn run(json: bool) -> i32 {
    use super::*;
    use std::io::Write;

    let work = || -> Result<i32, String> {
        let dirs = platform::dirs::get();
        platform::fs::create_private_dir_all(&dirs.state).map_err(|e| e.to_string())?;
        let _lock = control::lock_single_instance(&dirs.state.join("setup.lock"))
            .map_err(|_| "Close the other cv1touch setup window, then try again.".to_string())?;
        let logs = cv1touch::supervise::log_dir();
        platform::fs::create_private_dir_all(&logs).map_err(|e| e.to_string())?;
        let path = logs.join("finish-setup.log");
        // Keep the previous attempt; bounded logs also survive a later failed repair.
        if path.exists() {
            platform::fs::rename_replacing(&path, &logs.join("finish-setup.previous.log"))
                .map_err(|e| format!("cannot preserve the previous setup log: {e}"))?;
        }
        let mut log = platform::fs::create_private(&path).map_err(|e| e.to_string())?;
        writeln!(
            log,
            "cv1touch {}: Windows driver setup",
            env!("CARGO_PKG_VERSION")
        )
        .map_err(|e| e.to_string())?;
        let tool = std::env::current_exe().map_err(|e| e.to_string())?;
        let mut log_error = false;
        let mut steps = Vec::new();
        let (code, left) = prepare(|step| {
            let (program, args): (_, &[&str]) = match step {
                Step::SteamVrStopped => (tool.clone(), &["steamvr", "check-stopped"]),
                Step::Stop => (tool.clone(), &["stop"]),
                Step::Sensors => (tool.clone(), &["sensors", "install"]),
                Step::SteamVr => (tool.clone(), &["steamvr", "install"]),
                Step::Start => (tool.with_file_name("cv1touchd-quiet.exe"), &[]),
            };
            if !json {
                println!("\n{}...", step.label());
            }
            let mut command = Command::new(&program);
            command.args(args);
            if step == Step::Start {
                // The detached supervisor must not inherit our capture pipes: output() would
                // otherwise wait for its entire lifetime rather than just the launcher.
                command
                    .stdout(process::Stdio::null())
                    .stderr(process::Stdio::null());
            }
            let (mut code, text) = match command.output() {
                Ok(output) => (
                    output.status.code().unwrap_or(1),
                    format!(
                        "{}{}",
                        String::from_utf8_lossy(&output.stdout),
                        String::from_utf8_lossy(&output.stderr)
                    ),
                ),
                Err(error) => (
                    1,
                    format!(
                        "{}: could not run {}: {error}\n",
                        step.label(),
                        program.display()
                    ),
                ),
            };
            if step == Step::Stop
                && ["daemon.lock", "supervisor.lock"]
                    .iter()
                    .any(|name| locked(&control::runtime_dir().join(name)))
            {
                code = STILL_RUNNING;
            }
            if !json {
                print!("{text}");
            }
            log_error |= writeln!(log, "{}: exit {code}\n{text}", step.label()).is_err();
            steps.push((step, code));
            code
        });
        log.sync_all().map_err(|e| e.to_string())?;
        if log_error {
            return Err("Could not save all setup diagnostics; check free disk space.".into());
        }
        if json {
            println!("{}", self::json(code, &steps, &left));
            return Ok(code);
        }
        if !left.is_empty() {
            println!("\n{INCOMPLETE}");
            for line in &left {
                println!("- {line}");
            }
        }
        println!("\n{}", match code {
            0 => "Driver setup finished. Choose Set up cv1touch in the Start menu to calibrate.",
            3 => "Then choose Set up cv1touch in the Start menu.",
            _ => "Then run Finish setting up cv1touch again. For help, choose Send a cv1touch report in the Start menu.",
        });
        Ok(code)
    };
    match work() {
        Ok(code) => code,
        Err(error) if json => {
            let mut value = self::json(1, &[], &[]);
            value["error"] = error.into();
            println!("{value}");
            1
        }
        Err(error) => {
            eprintln!("cv1touch: {error}");
            1
        }
    }
}

#[cfg(not(windows))]
pub(super) fn run(_json: bool) -> i32 {
    eprintln!(
        "cv1touch: finish-setup is for the Windows installer. On Linux, use scripts/install.py."
    );
    1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn active_steamvr_or_a_failed_stop_changes_no_drivers() {
        for failed in [Step::SteamVrStopped, Step::Stop] {
            let mut called = Vec::new();
            assert_eq!(
                prepare(|step| {
                    called.push(step);
                    if step == failed {
                        2
                    } else {
                        0
                    }
                })
                .0,
                1
            );
            assert_eq!(called.last(), Some(&failed));
            assert!(!called.contains(&Step::Sensors));
        }
    }

    #[test]
    fn json_says_what_finished_and_what_is_left() {
        let (code, left) = prepare(|step| if step == Step::Sensors { 3 } else { 0 });
        let steps = [(Step::Stop, 1), (Step::Sensors, 3)];
        assert_eq!(
            json(code, &steps, &left),
            serde_json::json!({"ok": false, "reboot_needed": true,
                "steps": [{"step": "Stopping cv1touch", "exit": 1},
                          {"step": "Giving the sensors their driver", "exit": 3}],
                "remaining": ["Restart Windows to finish giving the sensors their driver."]})
        );
        // Restart stays required even when another step fails and forces exit code 1.
        let (code, left) = prepare(|step| match step {
            Step::Sensors => 3,
            Step::SteamVr => 1,
            _ => 0,
        });
        assert_eq!(code, 1);
        let mixed_steps = [
            (Step::SteamVrStopped, 0),
            (Step::Stop, 0),
            (Step::Sensors, 3),
            (Step::SteamVr, 1),
            (Step::Start, 0),
        ];
        let mixed = json(code, &mixed_steps, &left);
        assert_eq!(mixed["ok"], false);
        assert_eq!(mixed["reboot_needed"], true);
        assert_eq!(json(0, &[], &[])["ok"], true);
        assert_eq!(json(0, &[], &[])["reboot_needed"], false);
    }

    #[test]
    fn steps_have_plain_labels() {
        for step in [
            Step::SteamVrStopped,
            Step::Stop,
            Step::Sensors,
            Step::SteamVr,
            Step::Start,
        ] {
            assert!(step.label().contains(' '), "{step:?}");
        }
    }

    #[test]
    fn the_installer_shows_the_reasons() {
        let installer = include_str!("../../../windows/installer/cv1touch.iss");
        assert!(installer.contains(&format!("if S = '{INCOMPLETE}' then")));
    }

    #[test]
    fn partial_failures_do_not_skip_other_repairs_or_daemon_start() {
        // UAC denied, no sensors, reboot requested, driver-registration failure, launch failure.
        for sensor in [0, 1, 3] {
            for driver in [0, 1] {
                for start in [0, 1] {
                    let mut called = Vec::new();
                    let (code, left) = prepare(|step| {
                        called.push(step);
                        match step {
                            Step::Sensors => sensor,
                            Step::SteamVr => driver,
                            Step::Start => start,
                            Step::Stop => 1, // no daemon yet is normal on a new install
                            _ => 0,
                        }
                    });
                    assert_eq!(
                        called,
                        [
                            Step::SteamVrStopped,
                            Step::Stop,
                            Step::Sensors,
                            Step::SteamVr,
                            Step::Start
                        ]
                    );
                    assert_eq!(
                        code,
                        if sensor == 1 || driver != 0 || start != 0 {
                            1
                        } else {
                            sensor
                        }
                    );
                    // One reason per step that did not finish, a restart included.
                    let failed = [sensor != 0, driver != 0, start != 0];
                    assert_eq!(left.len(), failed.iter().filter(|f| **f).count());
                }
            }
        }
    }
}
