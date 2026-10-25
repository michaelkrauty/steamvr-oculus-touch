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
fn prepare(mut run: impl FnMut(Step) -> i32) -> i32 {
    // Refuse before stopping tracking if SteamVR is open or its state is unknown.
    if run(Step::SteamVrStopped) != 0 || !matches!(run(Step::Stop), 0 | 1) {
        return 1;
    }
    let sensors = run(Step::Sensors);
    let steamvr = run(Step::SteamVr);
    let start = run(Step::Start);
    // Partial failure must not look successful just because the last command succeeded.
    if steamvr != 0 || start != 0 || !matches!(sensors, 0 | 3) {
        1
    } else {
        sensors // 3: Windows asked for a restart, not ready for calibration yet.
    }
}

#[cfg(windows)]
pub(super) fn run() -> i32 {
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
        let code = prepare(|step| {
            let (program, args): (_, &[&str]) = match step {
                Step::SteamVrStopped => (tool.clone(), &["steamvr", "check-stopped"]),
                Step::Stop => (tool.clone(), &["stop"]),
                Step::Sensors => (tool.clone(), &["sensors", "install"]),
                Step::SteamVr => (tool.clone(), &["steamvr", "install"]),
                Step::Start => (tool.with_file_name("cv1touchd-quiet.exe"), &[]),
            };
            println!("\n{step:?}...");
            let mut command = Command::new(program);
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
                Err(error) => (1, format!("Could not run {step:?}: {error}\n")),
            };
            if step == Step::Stop
                && ["daemon.lock", "supervisor.lock"]
                    .iter()
                    .any(|name| locked(&control::runtime_dir().join(name)))
            {
                code = STILL_RUNNING;
            }
            print!("{text}");
            log_error |= writeln!(log, "{step:?}: exit {code}\n{text}").is_err();
            code
        });
        log.sync_all().map_err(|e| e.to_string())?;
        if log_error {
            return Err("Could not save all setup diagnostics; check free disk space.".into());
        }
        println!("\n{}", match code {
            0 => "Driver setup finished. Choose Set up cv1touch in the Start menu to calibrate.",
            3 => "Restart Windows, then choose Set up cv1touch in the Start menu.",
            _ => "Setup is incomplete. Fix the messages above, then run Finish setting up cv1touch again. For help, choose Send a cv1touch report in the Start menu.",
        });
        Ok(code)
    };
    match work() {
        Ok(code) => code,
        Err(error) => {
            eprintln!("cv1touch: {error}");
            1
        }
    }
}

#[cfg(not(windows))]
pub(super) fn run() -> i32 {
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
                }),
                1
            );
            assert_eq!(called.last(), Some(&failed));
            assert!(!called.contains(&Step::Sensors));
        }
    }

    #[test]
    fn partial_failures_do_not_skip_other_repairs_or_daemon_start() {
        // UAC denied, no sensors, reboot requested, driver-registration failure, launch failure.
        for sensor in [0, 1, 3] {
            for driver in [0, 1] {
                for start in [0, 1] {
                    let mut called = Vec::new();
                    let code = prepare(|step| {
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
                }
            }
        }
    }
}
