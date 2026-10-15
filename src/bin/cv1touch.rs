// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! cv1touch: CLI for cv1touchd. Subcommands: status, haptic, record, rec-summary, pace, calibration,
//! calibrate, consistency.

use std::collections::{BTreeMap, HashMap};
use std::io::{self, IsTerminal, Write};
use std::path::{Path, PathBuf};
use std::process::{self, Command};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use cv1touch::calib::{self, capture, consistency, file, solve};
use cv1touch::control;
use cv1touch::platform::{
    self,
    console::{self, Line},
};
use cv1touch::recorder::{self, Reader};
use cv1touch::shm::{self, ShmReader, HAND_LEFT, HAND_RIGHT};
use cv1touch::status;
use cv1touch::summary::{self, Window};
use cv1touch::{align, eval, register, rendermodel, steamvr, track, vr};

#[path = "cv1touch/setup.rs"]
mod guided_setup;
#[path = "cv1touch/report.rs"]
mod report;
#[path = "cv1touch/sensors.rs"]
mod sensors;

fn human_color_enabled(no_color: bool, term_dumb: bool, terminal: bool, forced: bool) -> bool {
    !no_color && !term_dumb && (terminal || forced)
}

fn human_text(message: &str) -> String {
    let color = human_color_enabled(
        std::env::var_os("NO_COLOR").is_some(),
        std::env::var("TERM").is_ok_and(|value| value == "dumb"),
        io::stdout().is_terminal(),
        std::env::var("CLICOLOR_FORCE").is_ok_and(|value| value == "1"),
    );
    if color {
        format!("\x1b[1m{message}\x1b[0m")
    } else {
        message.to_owned()
    }
}

fn human_action(message: &str) {
    println!("{}", human_text(message));
}

fn usage_code(code: i32) -> ! {
    eprintln!(
        "usage:\n  cv1touch setup [--no-slip-check] | setup --check [--json]\n  cv1touch status [--shm NAME] | status --json\n  cv1touch stop   (stops the daemon, and its supervisor, and waits until they have ended)\n  cv1touch report [--out PATH]   (one file with the logs and settings, for someone helping you)\n  cv1touch sensors | sensors install | sensors remove   (Windows: the sensors' USB driver)\n  cv1touch calibration capture|provisional|load PATH\n  cv1touch alignment reload\n  cv1touch haptic left|right DURATION_S FREQUENCY_HZ AMPLITUDE\n  \
         cv1touch test-inject [on|off]   (input injection for automated tests: off by default, on for 10 min)\n  \
         cv1touch inject left|right x|y|a|b|system|stick|trigger|grip[,...] MS   (at most 2000 ms; needs test-inject on)\n  \
         cv1touch record start PATH [--frames] | record stop\n  cv1touch rec-summary PATH [--window NAME:START_NS:END_NS]...\n  \
         cv1touch pace NAME:SECONDS...\n  cv1touch calibrate [--out PATH]\n  cv1touch calibrate inputs [--out PATH | --reset]\n  cv1touch inputs --json | inputs reload\n  \
         cv1touch calibrate solve REC [--out PATH] [--force]\n  cv1touch consistency REC [--calibration PATH]\n  \
         cv1touch calibrate refine REC [--calibration PATH] [--write]\n  cv1touch calibrate gravity REC [--calibration PATH] [--write]\n  cv1touch calibrate place SERIAL REC [--calibration PATH] [--write]\n  cv1touch universe | universe-watch [on|off] | shaping [on|off]\n  cv1touch latency [MS] | prediction [PERCENT] | prediction position|rotation PERCENT\n  cv1touch calibrate rig BASE [--write | --out-dir DIR]\n  \
         cv1touch fit-render [REC]\n  cv1touch steamvr install|check [--legacy-assets SO MANIFEST] | steamvr uninstall\n  cv1touch vr-devices | vr-pulse INDEX [MS]\n  \
         cv1touch register --headset\n  cv1touch doctor [--json]\n  cv1touch register [--force|--check] | register --solve FILE [--force]\n  \
         cv1touch register --jig | register --holds BASE [--reference SERIAL|hmd] [--write]\n  \
         cv1touch register --follow BASE --reference SERIAL|hmd [--write] | register --stations REC [--write | --out PATH]\n  \
         cv1touch eval-capture [--free [SECONDS]]\n  cv1touch eval BASE | eval-rig BASE [--replay] | eval-holds BASE\n  \
         cv1touch track BASE [--calibration PATH] [--registration PATH] [--offset-ms MS] [--set KEY=VALUE]...\n  \
         cv1touch landmarks learn REC [--write] | landmarks check REC\n  cv1touch input-test"
    );
    process::exit(code);
}

fn usage() -> ! {
    usage_code(2)
}

fn send_path(line: Result<String, String>) -> i32 {
    match line {
        Ok(line) => send(&line),
        Err(error) => {
            eprintln!("cv1touch: {error}");
            1
        }
    }
}

fn send(line: &str) -> i32 {
    match control::request(&control::socket_path(), line, Duration::from_secs(5)) {
        Ok(reply) => {
            println!("{reply}");
            if reply.starts_with("err") {
                1
            } else {
                0
            }
        }
        Err(e) => {
            eprintln!(
                "cv1touch: cannot reach cv1touchd at {}: {e}",
                control::socket_path().display()
            );
            1
        }
    }
}

/// `cv1touch stop`'s exit code when the daemon, or its supervisor, still runs 10 s after it was
/// asked to stop, or ran but never answered; 1 means it could not be reached at all, or refused.
const STILL_RUNNING: i32 = 2;

/// `cv1touch stop`: asks the daemon to stop, then waits until it, and a supervisor running it,
/// have ended (10 s at most), so that what comes next, an uninstaller say, finds their files
/// free. Each holds a lock in the runtime directory while it runs: this user's daemon, not
/// another's. A daemon that does not answer yet, because it is starting or its supervisor is
/// about to start it again, is asked again while either lock is held.
fn stop() -> i32 {
    const PATIENCE: Duration = Duration::from_secs(10);
    const STEP: Duration = Duration::from_millis(200);
    let runtime = control::runtime_dir();
    let locks = [runtime.join("daemon.lock"), runtime.join("supervisor.lock")];
    let running = || locks.iter().any(|lock| locked(lock));
    let start = Instant::now();
    let mut asked = ask_to_stop();
    while asked.is_err() && running() && start.elapsed() < PATIENCE {
        std::thread::sleep(STEP);
        asked = ask_to_stop();
    }
    match asked {
        Err(e) if running() => {
            eprintln!("cv1touch: cv1touchd runs but did not answer within 10 s: {e}");
            STILL_RUNNING
        }
        Err(e) => {
            eprintln!(
                "cv1touch: cannot reach cv1touchd at {}: {e}",
                control::socket_path().display()
            );
            1
        }
        Ok(reply) if reply.starts_with("err") => {
            println!("{reply}");
            1
        }
        Ok(_) => {
            let left = PATIENCE.saturating_sub(start.elapsed());
            if ended(running, left, STEP) {
                println!("stopped");
                0
            } else {
                eprintln!("cv1touch: cv1touchd still runs 10 s after it was asked to stop");
                STILL_RUNNING
            }
        }
    }
}

fn ask_to_stop() -> io::Result<String> {
    control::request(&control::socket_path(), "stop", Duration::from_secs(5))
}

/// Whether another process holds the lock file `path`.
fn locked(path: &Path) -> bool {
    matches!(control::lock_single_instance(path), Err(e) if e.kind() == io::ErrorKind::WouldBlock)
}

/// Whether `running` turns false within `timeout`, looked at every `step`.
fn ended(mut running: impl FnMut() -> bool, timeout: Duration, step: Duration) -> bool {
    let start = Instant::now();
    loop {
        if !running() {
            return true;
        }
        if start.elapsed() >= timeout {
            return false;
        }
        std::thread::sleep(step);
    }
}

fn json_status() -> i32 {
    match control::request(&control::socket_path(), "status", Duration::from_secs(5)) {
        Ok(reply) => match reply
            .strip_prefix("ok ")
            .and_then(|s| serde_json::from_str::<serde_json::Value>(s).ok())
        {
            Some(v) => {
                println!("{}", serde_json::to_string_pretty(&v).unwrap());
                0
            }
            None => {
                eprintln!("cv1touch: invalid status response: {reply}");
                1
            }
        },
        Err(e) => {
            eprintln!("cv1touch: cannot reach daemon: {e}");
            1
        }
    }
}

/// A check `doctor` makes: its name, whether it passed, and what it looked at or found.
type DoctorCheck = (&'static str, bool, String);

/// The Oculus USB vendor: the headset, its sensors and the Touch controllers' dongle.
const OCULUS: u16 = 0x2833;

/// What `doctor` checks, and the daemon's status if it answered.
fn doctor_checks() -> (Vec<DoctorCheck>, Option<serde_json::Value>) {
    let runtime = steamvr::runtime_root();
    let calibration = file::load(&file::default_path());
    let registration = align::load_registration(&align::registration_path());
    let render = align::load_render(&align::render_path());
    let mut checks = vec![
        (
            "steamvr_runtime",
            steamvr::vrpathreg().is_file(),
            runtime.display().to_string(),
        ),
        (
            "driver_assets",
            steamvr::driver_assets().is_ok(),
            "installed payload or development build".into(),
        ),
        (
            "openvr_library",
            vr::runtime_library().is_file(),
            "SteamVR native OpenVR client library".into(),
        ),
        (
            "input_profile",
            runtime
                .join("drivers/oculus/resources/input/touch_profile.json")
                .is_file(),
            "SteamVR Oculus Touch input profile".into(),
        ),
        (
            "render_models",
            rendermodel::steamvr_body_obj(true).is_file()
                && rendermodel::steamvr_body_obj(false).is_file(),
            "SteamVR CV1 models".into(),
        ),
        (
            "calibration",
            calibration.is_ok(),
            calibration
                .err()
                .unwrap_or_else(|| file::default_path().display().to_string()),
        ),
        (
            "registration",
            registration.is_ok(),
            registration
                .err()
                .unwrap_or_else(|| align::registration_path().display().to_string()),
        ),
        (
            "render_offsets",
            render.is_ok(),
            render
                .err()
                .unwrap_or_else(|| align::render_path().display().to_string()),
        ),
    ];
    #[cfg(windows)]
    checks.push(sensors::doctor_check());
    let daemon = status_json();
    checks.push((
        "daemon",
        daemon.is_some(),
        control::socket_path().display().to_string(),
    ));
    if let Some(d) = &daemon {
        let (ok, detail) = sensor_poses_check(d);
        checks.push(("sensor_poses", ok, detail));
    }
    (checks, daemon)
}

/// Whether the sensors stand where the calibration says (the daemon's pose watch, `status`):
/// one that moved is being placed again from the others, one that could not be is waiting for
/// `setup`, and with fewer than two sensors agreeing nothing can be told.
fn sensor_poses_check(status: &serde_json::Value) -> (bool, String) {
    let sensors = status["sensors"].as_array().cloned().unwrap_or_default();
    let mut moved = Vec::new();
    let mut given_up = Vec::new();
    let mut placed = Vec::new();
    let mut released = Vec::new();
    for s in &sensors {
        let serial = s["serial"].as_str().unwrap_or("?");
        match s["pose"]["state"].as_str() {
            Some("moved") => moved.push(serial),
            Some("given_up") => given_up.push(serial),
            Some("placed") => placed.push(serial),
            Some("released") => released.push(serial),
            _ => {}
        }
    }
    if status["sensors_unsure"].as_bool() == Some(true) {
        return (
            false,
            "fewer than two sensors agree with each other: run cv1touch setup".to_string(),
        );
    }
    if !given_up.is_empty() {
        return (
            false,
            format!(
                "{} could not be placed from the other sensors: run cv1touch setup",
                given_up.join(", ")
            ),
        );
    }
    if !moved.is_empty() {
        return (
            false,
            format!(
                "{} moved and is being placed again: move the controllers where it sees them",
                moved.join(", ")
            ),
        );
    }
    let mut detail = format!("{} sensor(s) stand where calibrated", sensors.len());
    if !placed.is_empty() {
        detail = format!(
            "{} sensor(s); {} placed again since the daemon started",
            sensors.len(),
            placed.join(", ")
        );
    }
    if !released.is_empty() {
        detail.push_str(&format!(
            "; {} judged moved by mistake and let go",
            released.join(", ")
        ));
    }
    (true, detail)
}

fn doctor(json: bool) -> i32 {
    let (checks, daemon) = doctor_checks();
    if json {
        println!("{}", serde_json::to_string_pretty(&serde_json::json!({ "checks": checks.iter().map(|(name, ok, detail)| serde_json::json!({"name": name, "ok": ok, "detail": detail})).collect::<Vec<_>>(), "daemon": daemon, "note": "Wake sleeping controllers before checking tracking; calibration and registration require physical capture." })).unwrap());
    } else {
        for (name, ok, detail) in &checks {
            println!("{} {name}: {detail}", if *ok { "ok" } else { "FAIL" });
        }
        #[cfg(unix)]
        println!("Wake sleeping controllers before checking tracking. USB access needs active-session udev permissions; logs: journalctl --user -u cv1touchd.");
        #[cfg(windows)]
        println!(
            "Wake sleeping controllers before checking tracking. The daemon's log: {}",
            cv1touch::supervise::log_file().display()
        );
        println!("To send someone everything they need to see what went wrong: cv1touch report");
    }
    if checks.iter().all(|(_, ok, _)| *ok) {
        0
    } else {
        1
    }
}

fn status_board(shm_name: &str) -> i32 {
    let mut reader: Option<ShmReader> = None;
    let mut trackers: HashMap<String, status::RateTracker> = HashMap::new();
    loop {
        let now = shm::now_ns();
        if reader.as_ref().is_none_or(|r| r.stale(now, 500_000_000)) {
            reader = match ShmReader::open(shm_name) {
                Ok(r) => Some(r),
                Err(e) => {
                    println!("\x1b[H\x1b[2Jwaiting for cv1touchd: {e}");
                    let _ = io::stdout().flush();
                    thread::sleep(Duration::from_millis(500));
                    continue;
                }
            };
        }
        let r = reader.as_ref().unwrap();
        let complete = control::request(
            &control::socket_path(),
            "status",
            Duration::from_millis(200),
        )
        .ok()
        .and_then(|reply| {
            serde_json::from_str::<serde_json::Value>(reply.strip_prefix("ok ")?).ok()
        })
        .filter(|v| v["generation"].as_u64() == Some(r.generation()));
        let (sensors, sensor_note) = match complete.as_ref().map(status::sensors_from_status) {
            Some(Ok(sensors)) => {
                let note = if complete.as_ref().unwrap()["sensor_total"].as_u64().is_none() {
                    "Legacy daemon diagnostics may show only the first eight sensors."
                } else { "" };
                (sensors, note)
            }
            _ => (r.sensors().unwrap_or_default(),
                "Control diagnostics unavailable; showing the SHM compatibility prefix (at most eight sensors)."),
        };
        let mut rates = HashMap::new();
        for s in &sensors {
            let key = shm::serial_str(s);
            let tracker = trackers
                .entry(key.clone())
                .or_insert_with(|| status::RateTracker::new(2_000_000_000));
            let [frames, left, right] = tracker.update(
                now,
                [s.frames, s.pose_fixes[HAND_LEFT], s.pose_fixes[HAND_RIGHT]],
            );
            rates.insert(key, (frames, left, right));
        }
        let hands = [
            r.hand(HAND_LEFT).unwrap_or_default(),
            r.hand(HAND_RIGHT).unwrap_or_default(),
        ];
        let age = now.saturating_sub(r.heartbeat_ns()) as f64 / 1e9;
        println!(
            "\x1b[H\x1b[2J{}",
            status::render(&hands, &sensors, &rates, age, r.generation())
        );
        if !sensor_note.is_empty() {
            println!("{sensor_note}");
        }
        let _ = io::stdout().flush();
        thread::sleep(Duration::from_millis(200));
    }
}

fn rec_summary(path: &Path, windows: &[Window]) -> i32 {
    let mut reader = match Reader::open(path) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return 1;
        }
    };
    let mut by_type: BTreeMap<u16, u64> = BTreeMap::new();
    let mut frames: BTreeMap<String, (u64, u64, u64, u64)> = BTreeMap::new(); // count, first, last, device poses
    let mut states = Vec::new();
    for rec in reader.by_ref() {
        *by_type.entry(rec.rtype).or_default() += 1;
        match rec.rtype {
            recorder::CONTROLLER_STATE => {
                if let Some((hand, s)) = recorder::decode_state(&rec.payload) {
                    states.push((rec.host_ns, hand, s));
                }
            }
            recorder::FRAME_OBS => {
                if let Some((info, _)) = recorder::decode_frame_obs(&rec.payload) {
                    let serial = cv1touch::engine::cstr_field(&info.serial);
                    let poses = info.devices[..(info.n_devices as usize).min(3)]
                        .iter()
                        .filter(|d| d.pose_valid != 0)
                        .count() as u64;
                    let e = frames
                        .entry(serial)
                        .or_insert((0, rec.host_ns, rec.host_ns, 0));
                    e.0 += 1;
                    e.2 = rec.host_ns;
                    e.3 += poses;
                }
            }
            _ => {}
        }
    }
    if let Some(error) = reader.error() {
        eprintln!("cv1touch: {error}");
        return 1;
    }
    println!("records by type (1 state, 2 raw report, 4 frame obs, 5 sensor info, 6 LED model): {by_type:?}");
    for (serial, (n, first, last, poses)) in &frames {
        let span = (*last - *first) as f64 / 1e9;
        println!(
            "sensor {serial}: {n} frames over {span:.1} s = {:.1} frames/s, {poses} device poses",
            *n as f64 / span.max(1e-9)
        );
    }
    let mut all = windows.to_vec();
    if all.is_empty() {
        all.push(Window {
            name: "all".into(),
            start_ns: 0,
            end_ns: u64::MAX,
        });
    }
    for w in &all {
        for (hand, name) in [(HAND_LEFT, "left"), (HAND_RIGHT, "right")] {
            let s = summary::window_stats(&states, hand, w);
            let ms = |v: Option<u64>| {
                v.map_or("n/a".to_string(), |ns| format!("{:.3} ms", ns as f64 / 1e6))
            };
            println!(
                "{:>10} {name:>5}: n={} connected={:.2} position={:.2} receive->publish p50={} p99={}",
                w.name, s.n, s.connected, s.position, ms(s.latency_p50_ns), ms(s.latency_p99_ns)
            );
        }
    }
    for (hand, name) in [(HAND_LEFT, "left"), (HAND_RIGHT, "right")] {
        for (t, delay) in summary::disconnect_delays(&states, hand) {
            println!(
                "{name}: disconnected at {t} ns, {:.1} ms after its last radio report",
                delay as f64 / 1e6
            );
        }
    }
    0
}

/// Plays `count` short 880 Hz beeps.
fn beeps(count: usize) {
    const RATE: u32 = 48_000;
    let mut samples: Vec<i16> = Vec::new();
    for _ in 0..count {
        for i in 0..(RATE as usize * 12 / 100) {
            let v = (2.0 * std::f64::consts::PI * 880.0 * i as f64 / RATE as f64).sin() * 0.35;
            samples.push((v * i16::MAX as f64) as i16);
        }
        samples.extend(std::iter::repeat_n(0, RATE as usize / 10));
    }
    let data_len = (samples.len() * 2) as u32;
    let mut wav = Vec::with_capacity(44 + data_len as usize);
    wav.extend_from_slice(b"RIFF");
    wav.extend_from_slice(&(36 + data_len).to_le_bytes());
    wav.extend_from_slice(b"WAVEfmt ");
    wav.extend_from_slice(&16u32.to_le_bytes());
    wav.extend_from_slice(&1u16.to_le_bytes()); // PCM
    wav.extend_from_slice(&1u16.to_le_bytes()); // mono
    wav.extend_from_slice(&RATE.to_le_bytes());
    wav.extend_from_slice(&(RATE * 2).to_le_bytes());
    wav.extend_from_slice(&2u16.to_le_bytes());
    wav.extend_from_slice(&16u16.to_le_bytes());
    wav.extend_from_slice(b"data");
    wav.extend_from_slice(&data_len.to_le_bytes());
    for s in samples {
        wav.extend_from_slice(&s.to_le_bytes());
    }
    platform::sound::play_wav(&wav);
}

/// From now on an interrupt (Ctrl+C, closing the terminal) sets a flag instead of ending the
/// process, so `calibrate` can stop the recording and restore the daemon's mode however it is left.
fn catch_interrupts() {
    console::catch_interrupts();
}

fn interrupted() -> bool {
    console::interrupted()
}

fn commit_fit(interrupted: bool, save: impl FnOnce() -> Result<(), String>) -> Result<(), String> {
    if interrupted {
        return Err("interrupted; fitted configuration was not installed".into());
    }
    save()
}

/// Waits for Enter (or end of input). False when interrupted.
fn wait_enter() -> bool {
    loop {
        if interrupted() {
            return false;
        }
        if console::next_line(Duration::from_millis(200)) != Line::NotYet {
            return !interrupted();
        }
    }
}

/// Enter-paced phases: one beep starts a phase, two end it, three end the whole run.
/// Prints `--window NAME:START_NS:END_NS` arguments for rec-summary.
/// Exit code 130: interrupted.
fn pace(phases: &[String]) -> i32 {
    match pace_windows(phases) {
        Ok(windows) => {
            let flags: Vec<String> = windows
                .iter()
                .map(|(name, start, end)| format!("--window {name}:{start}:{end}"))
                .collect();
            println!("Finished.\n{}", flags.join(" "));
            0
        }
        Err(code) => code,
    }
}

/// Runs the beep-paced phases; the (name, start_ns, end_ns) of each, or the exit code when interrupted.
fn pace_windows(phases: &[String]) -> Result<Vec<(String, u64, u64)>, i32> {
    let mut parsed = Vec::new();
    for p in phases {
        let Some((name, secs)) = p.split_once(':') else {
            usage()
        };
        let Ok(secs) = secs.parse::<f64>() else {
            usage()
        };
        parsed.push((name.to_string(), secs));
    }
    let mut windows = Vec::new();
    for (i, (name, secs)) in parsed.iter().enumerate() {
        print!(
            "{}",
            human_text(&format!(
                "Phase {}/{}: {name} ({secs} s). Press Enter to start...",
                i + 1,
                parsed.len()
            ))
        );
        let _ = io::stdout().flush();
        if !wait_enter() {
            println!();
            return Err(130);
        }
        beeps(1);
        let start = shm::now_ns();
        let end_target = start + (*secs * 1e9) as u64;
        while shm::now_ns() < end_target {
            if interrupted() {
                println!();
                return Err(130);
            }
            print!(
                "\r  recording... {:5.1} s left ",
                (end_target - shm::now_ns()) as f64 / 1e9
            );
            let _ = io::stdout().flush();
            thread::sleep(Duration::from_millis(100));
        }
        let end = shm::now_ns();
        println!("\r  {name} done.                         ");
        beeps(2);
        windows.push((name.clone(), start, end));
    }
    beeps(3);
    Ok(windows)
}

/// The samples taken inside a paced phase: not while the user read the briefing or after the beeps.
fn phase_samples(
    samples: Vec<register::Sample>,
    start_ns: u64,
    end_ns: u64,
) -> Vec<register::Sample> {
    samples
        .into_iter()
        .filter(|s| s.t_ns >= start_ns && s.t_ns <= end_ns)
        .collect()
}

const CALIBRATION_BRIEFING: &str = "\
Calibration capture: 60 s of motion, paced by beeps (press Enter to start).
  Pick up both controllers and wave them slowly through the whole play space, high and low,
  turning your wrists through every orientation as you go. Make sure every sensor sees them, and
  spend time where two sensors see them at once. Keep them moving: no need to hold still.
Wake the controllers (press any button) right before starting.";

fn unix_now() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}

fn absolute(p: &Path) -> PathBuf {
    std::path::absolute(p).unwrap_or_else(|_| p.to_path_buf())
}

/// cv1touch's state directory (platform::dirs): calibration recordings.
fn state_dir() -> PathBuf {
    cv1touch::platform::dirs::get().state
}

fn status_json() -> Option<serde_json::Value> {
    let reply = control::request(&control::socket_path(), "status", Duration::from_secs(5)).ok()?;
    serde_json::from_str(reply.strip_prefix("ok ")?).ok()
}

/// The daemon's latency adjustment from its status, ms (none from a daemon that does not say).
fn latency_adjustment(status: Option<&serde_json::Value>) -> Option<f64> {
    status?["latency_adjustment_ms"].as_f64()
}

/// Waits until cv1touchd reports `mode` with every sensor's frame counter advancing.
fn wait_for_mode(mode: &str, timeout: Duration) -> Result<usize, String> {
    let start = Instant::now();
    let mut last: HashMap<String, u64> = HashMap::new();
    while start.elapsed() < timeout {
        if interrupted() {
            return Err("interrupted".into());
        }
        thread::sleep(Duration::from_secs(1));
        let Some(v) = status_json() else { continue };
        if v["mode"] != mode {
            continue;
        }
        let frames: HashMap<String, u64> = v["sensors"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|s| Some((s["serial"].as_str()?.to_string(), s["frames"].as_u64()?)))
            .collect();
        let advancing = !frames.is_empty()
            && frames
                .iter()
                .all(|(k, f)| last.get(k).is_some_and(|p| f > p));
        last = frames;
        if advancing {
            return Ok(last.len());
        }
    }
    Err(format!(
        "cv1touchd did not reach {mode} mode with all sensors streaming within {} s",
        timeout.as_secs()
    ))
}

fn calibrate_solve(rec: &Path, out: &Path, force: bool) -> i32 {
    calibrate_solve_saved(rec, out, force).map_or_else(|code| code, |_| 0)
}

/// Return the accepted saved output hash for guided completion; preserve expert
/// diagnostics and exit codes through the ordinary wrapper.
fn calibrate_solve_saved(rec: &Path, out: &Path, force: bool) -> Result<String, i32> {
    let cap = match capture::load(rec) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return Err(1);
        }
    };
    let sol = match solve::solve_capture(&cap, &solve::SolveOptions::default()) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("cv1touch: calibration failed: {e}");
            return Err(1);
        }
    };
    print!("{}", solve::format_report(&sol));
    if let Ok(previous) = file::load(out) {
        let serials: Vec<String> = sol.sensors.iter().map(|s| s.serial.clone()).collect();
        let missing = file::missing_from(&previous, &serials);
        if !missing.is_empty() && !force {
            eprintln!(
                "cv1touch: not installing: sensor(s) {} are in the current calibration but not in this capture (not open?). \
                 Re-run with every sensor streaming, or add --force to drop them",
                missing.join(", ")
            );
            return Err(1);
        }
    }
    if let Err(e) = solve::check(&sol, &solve::Acceptance::default()) {
        if !force {
            eprintln!(
                "cv1touch: not installing: {e} (re-run, or `calibrate solve {} --force`)",
                rec.display()
            );
            return Err(1);
        }
        eprintln!("cv1touch: warning: {e}");
    }
    if interrupted() {
        eprintln!("cv1touch: interrupted: not installing");
        return Err(130);
    }
    let f = file::CalibrationFile::from_solution(
        &sol,
        &absolute(rec).display().to_string(),
        unix_now(),
    );
    match file::save_with_fingerprint(out, &f) {
        Ok((backup, fingerprint)) => {
            if let Some(b) = backup {
                println!("previous calibration kept as {}", b.display());
            }
            println!("installed {}", out.display());
            Ok(fingerprint)
        }
        Err(e) => {
            eprintln!("cv1touch: {e}");
            Err(1)
        }
    }
}

/// Back to tracking: the calibration at `out` if there is one, else provisional placement.
fn restore_mode(out: &Path) -> i32 {
    if file::load(out).is_ok() {
        if send_path(control::calibration_load_line(&absolute(out))) == 0 {
            return 0;
        }
        eprintln!("cv1touch: calibration reload failed; attempting provisional tracking recovery");
        send("calibration provisional");
        1
    } else {
        send("calibration provisional")
    }
}

fn calibrate(out: &Path) -> i32 {
    catch_interrupts();
    println!("{CALIBRATION_BRIEFING}\n");
    if send("calibration capture") != 0 {
        return 1;
    }
    match wait_for_mode("capture", Duration::from_secs(30)) {
        Ok(n) => println!("{n} sensors streaming in capture mode."),
        Err(e) => {
            eprintln!("cv1touch: {e}");
            restore_mode(out);
            return 1;
        }
    }
    let dir = state_dir();
    if let Err(e) = std::fs::create_dir_all(&dir) {
        eprintln!("cv1touch: {}: {e}", dir.display());
        restore_mode(out);
        return 1;
    }
    let rec = dir.join(format!("calib-{}.rec", unix_now()));
    if send_path(control::record_start_line(&rec, false)) != 0 {
        restore_mode(out);
        return 1;
    }
    let paced = pace(&["wave:60".to_string()]);
    send("record stop");
    if paced != 0 {
        eprintln!(
            "cv1touch: calibration interrupted (recording kept at {})",
            rec.display()
        );
        restore_mode(out);
        return paced;
    }
    println!("Solving {} ...", rec.display());
    let code = calibrate_solve(&rec, out, false);
    restore_mode(out);
    code
}

/// Fits each controller's LED model (from a recording) onto the SteamVR CV1 render model.
fn fit_render(rec: Option<&Path>) -> i32 {
    let rec = match rec.map(Path::to_path_buf).or_else(newest_recording) {
        Some(r) => r,
        None => {
            eprintln!(
                "cv1touch: no recording given and none in {}",
                state_dir().display()
            );
            return 1;
        }
    };
    let cap = match capture::load(&rec) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return 1;
        }
    };
    let mut meshes = Vec::new();
    for left in [true, false] {
        let obj = rendermodel::steamvr_body_obj(left);
        match std::fs::read_to_string(&obj)
            .map_err(|e| format!("{}: {e}", obj.display()))
            .and_then(|t| rendermodel::parse_obj(&t))
        {
            Ok(m) => meshes.push(m),
            Err(e) => {
                eprintln!("cv1touch: {e}");
                return 1;
            }
        }
    }
    let (ll, lr) = (
        cap.leds.get(&calib::DEVICE_LEFT),
        cap.leds.get(&calib::DEVICE_RIGHT),
    );
    let fits: Vec<(&str, usize, rendermodel::Fit, Option<rendermodel::Fit>)> = match (ll, lr) {
        // both hands: fitted together, mirror-consistent (see `rendermodel::fit_mirrored`)
        (Some(l), Some(r)) => {
            let (fl, fr) = rendermodel::fit_mirrored(l, r, &meshes[0], &meshes[1]);
            vec![
                ("left", l.len(), fl, Some(rendermodel::fit(l, &meshes[0]))),
                ("right", r.len(), fr, Some(rendermodel::fit(r, &meshes[1]))),
            ]
        }
        _ => [(ll, "left", 0usize), (lr, "right", 1)]
            .into_iter()
            .filter_map(|(leds, hand, k)| {
                leds.map(|l| (hand, l.len(), rendermodel::fit(l, &meshes[k]), None))
            })
            .collect(),
    };
    let mut hands = Vec::new();
    for (hand, n, f, alone) in fits {
        let moved = alone.map_or(String::new(), |a| {
            let d = a.render_from_model.inverse() * f.render_from_model;
            format!(" (fitted alone: {:.2} mm rms; this placement differs from that by {:.1} mm, {:.2} deg)", a.rms_m * 1e3, d.translation.vector.norm() * 1e3, d.rotation.angle().to_degrees())
        });
        println!(
            "{hand}: {n} LEDs, surface distance rms {:.2} mm, max {:.2} mm{moved}",
            f.rms_m * 1e3,
            f.max_m * 1e3
        );
        hands.push(align::RenderOffset {
            hand: hand.into(),
            model_from_render: align::PoseEntry::from_iso(&f.render_from_model.inverse()),
            rms_mm: f.rms_m * 1e3,
            leds: n,
        });
    }
    for (leds, hand) in [(ll, "left"), (lr, "right")] {
        if leds.is_none() {
            eprintln!(
                "cv1touch: {} has no LED model for the {hand} controller",
                rec.display()
            );
        }
    }
    if hands.is_empty() {
        return 1;
    }
    let file = align::RenderOffsets {
        format: align::RENDER_FORMAT.into(),
        version: align::VERSION,
        created_unix_s: unix_now(),
        source: absolute(&rec).display().to_string(),
        hands,
    };
    match commit_fit(interrupted(), || {
        align::save(&align::render_path(), &file).map(|_| ())
    }) {
        Ok(()) => println!("installed {}", align::render_path().display()),
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return 1;
        }
    }
    send("alignment reload")
}

fn newest_recording() -> Option<PathBuf> {
    std::fs::read_dir(state_dir())
        .ok()?
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "rec"))
        .max_by_key(|p| p.metadata().and_then(|m| m.modified()).ok())
}

fn vr_devices() -> i32 {
    let c = match vr::Client::connect() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return 1;
        }
    };
    let poses = c.raw_poses();
    for d in c.devices() {
        let p = poses[d.index as usize].map_or("no pose".to_string(), |t| {
            format!(
                "({:+.3}, {:+.3}, {:+.3})",
                t.translation.vector.x, t.translation.vector.y, t.translation.vector.z
            )
        });
        let universe = c
            .uint64_property(d.index, vr::PROP_CURRENT_UNIVERSE_ID)
            .unwrap_or(0);
        let battery = c
            .battery(d.index)
            .map_or(String::new(), |b| format!(" battery {:.0}%", b * 100.0));
        println!(
            "{:2} class {} {:10} {:14} {:28} {:24} universe {universe} raw {p}{battery}",
            d.index, d.class, d.tracking_system, d.controller_type, d.serial, d.model
        );
    }
    0
}

const REGISTER_BRIEFING: &str = "\
Quick Lighthouse registration: about 60 s, paced by beeps (press Enter to start).
  Hold one Touch controller pressed firmly against the left Index controller, in one hand or
  both, so they move as one. Move the pair slowly through the play space, and keep turning it:
  roll, pitch and yaw your wrist through as many orientations as you can. Keep both controllers
  where the sensors and the base stations can see them. Slow and steady beats fast.";

/// cv1touchd's shared memory, by the name its status gives (older daemons: the default).
fn touch_reader() -> Result<ShmReader, String> {
    let status = status_json().ok_or("cannot read cv1touchd status")?;
    let name = status["shm"].as_str().unwrap_or(shm::DEFAULT_NAME);
    let reader = ShmReader::open(name).map_err(|e| format!("cannot read controller poses: {e}"))?;
    if status["generation"].as_u64() != Some(reader.generation()) {
        return Err("cv1touchd restarted; no recording started".into());
    }
    Ok(reader)
}

/// A Touch pose worth pairing: awake, with an optical fix in the last 100 ms, carried to `now`.
fn touch_sample(st: &shm::ControllerState, now: u64) -> Option<nalgebra::Isometry3<f64>> {
    let want = cv1touch::ffi::CV1T_FLAG_CONNECTED | cv1touch::ffi::CV1T_FLAG_POSITION;
    (st.flags & want == want && now.saturating_sub(st.last_position_fix_host_ns) <= 100_000_000)
        .then(|| register::extrapolate(st, now))
}

fn register_cmd(check_only: bool, force: bool) -> i32 {
    catch_interrupts();
    if status_json()
        .and_then(|v| v["mode"].as_str().map(str::to_string))
        .as_deref()
        != Some("calibrated")
    {
        eprintln!(
            "cv1touch: cv1touchd must be running in calibrated mode (cv1touch status --json)"
        );
        return 1;
    }
    let saved = if check_only {
        match align::load_registration(&align::registration_path()) {
            Ok(r) => Some(r),
            Err(e) => {
                eprintln!("cv1touch: {e}");
                return 1;
            }
        }
    } else {
        None
    };
    let client = match vr::Client::connect() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return 1;
        }
    };
    let devices = client.devices();
    let Some(reference) = vr::find_reference(&devices).cloned() else {
        eprintln!(
            "cv1touch: no Lighthouse-tracked Index controller: turn on the left Index controller"
        );
        return 1;
    };
    let universe = client
        .uint64_property(reference.index, vr::PROP_CURRENT_UNIVERSE_ID)
        .unwrap_or(0);
    let reader = match shm::ShmReader::open(shm::DEFAULT_NAME) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return 1;
        }
    };
    println!(
        "reference: Index controller {} (device {}, universe {universe})\n",
        reference.serial, reference.index
    );
    println!("{REGISTER_BRIEFING}\n");
    let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let sampler = {
        let stop = stop.clone();
        std::thread::spawn(move || {
            let mut out: Vec<register::Sample> = Vec::new();
            while !stop.load(std::sync::atomic::Ordering::Relaxed) {
                let now = shm::now_ns();
                if let Some(Some(lh)) = client.raw_poses().get(reference.index as usize).copied() {
                    let touch =
                        [0, 1].map(|h| reader.hand(h).and_then(|st| touch_sample(&st, now)));
                    out.push(register::Sample {
                        t_ns: now,
                        reference: lh,
                        touch,
                    });
                }
                std::thread::sleep(Duration::from_millis(10));
            }
            out
        })
    };
    let paced = pace_windows(&[if check_only { "check:20" } else { "move:60" }.to_string()]);
    stop.store(true, std::sync::atomic::Ordering::Relaxed);
    let samples = sampler.join().unwrap_or_default();
    let (start, end) = match paced {
        Ok(w) => (w[0].1, w[0].2),
        Err(code) => return code,
    };
    let samples = phase_samples(samples, start, end);
    let session = state_dir().join(format!("register-{}.json", unix_now()));
    match register::save_session(&session, &reference.serial, universe, &samples) {
        Ok(()) => println!(
            "samples kept in {} (cv1touch register --solve FILE re-solves them)",
            session.display()
        ),
        Err(e) => eprintln!("cv1touch: {e}"),
    }
    register_solve(&samples, &reference.serial, universe, saved.as_ref(), force)
}

fn register_solve_file(path: &Path, force: bool) -> i32 {
    match register::load_session(path) {
        Ok((reference, universe, samples)) => {
            register_solve(&samples, &reference, universe, None, force)
        }
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

/// Picks the held Touch, lines its stream up with the reference's, then checks against `saved` or
/// solves and installs.
fn register_solve(
    samples: &[register::Sample],
    reference: &str,
    universe: u64,
    saved: Option<&align::Registration>,
    force: bool,
) -> i32 {
    let Some((hand, corr)) = register::pick_touch(samples) else {
        eprintln!("cv1touch: neither Touch moved with the Index controller (were they held together and tracked?)");
        return 1;
    };
    let name = ["left", "right"][hand];
    let mut steps: Vec<u64> = samples
        .windows(2)
        .map(|w| w[1].t_ns.saturating_sub(w[0].t_ns))
        .collect();
    steps.sort_unstable();
    let step_ms = steps.get(steps.len() / 2).copied().unwrap_or(0) as f64 * 1e-6;
    let (shift, lag_corr) = register::estimate_lag(samples, hand, 20);
    let pairs = register::shifted_pairs(samples, hand, shift);
    println!(
        "{} samples; the {name} Touch moved with the reference (correlation {corr:.2}); its stream lags the reference by {:.0} ms (correlation {lag_corr:.2}); {} paired",
        samples.len(),
        shift as f64 * step_ms,
        pairs.len()
    );
    if let Some(saved) = saved {
        let (_, pos, rot) = register::check(&pairs, &saved.lighthouse_from_world.iso());
        println!("registration check: held-offset spread {:.1} mm, {:.2} deg (the hold's slip plus registration error)", pos * 1e3, rot);
        return 0;
    }
    let sol = match register::solve(&pairs, &register::Options::default()) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("cv1touch: registration failed: {e}");
            return 1;
        }
    };
    println!(
        "{} pairs, {} inliers; residual {:.1} mm, {:.2} deg rms; axis ratio {:.2}",
        sol.pairs,
        sol.inliers,
        sol.pos_rms_m * 1e3,
        sol.rot_rms_deg,
        sol.axis_ratio
    );
    let y = sol.reference_from_touch.translation.vector;
    println!(
        "held offset: Touch origin at ({:+.1}, {:+.1}, {:+.1}) mm in the Index frame",
        y.x * 1e3,
        y.y * 1e3,
        y.z * 1e3
    );
    if (sol.pos_rms_m > 0.010 || sol.inliers * 10 < sol.pairs * 8) && !force {
        eprintln!("cv1touch: not installing: residual over 10 mm rms or under 80% inliers (hold firmer and move slower, or --force)");
        return 1;
    }
    let file = align::Registration {
        format: align::REGISTRATION_FORMAT.into(),
        version: align::VERSION,
        created_unix_s: unix_now(),
        lighthouse_from_world: align::PoseEntry::from_iso(&sol.lighthouse_from_world),
        universe_id: universe,
        touch: name.into(),
        reference: reference.to_string(),
        pairs: sol.pairs,
        pos_rms_mm: sol.pos_rms_m * 1e3,
        rot_rms_deg: sol.rot_rms_deg,
        stations: vr::Client::connect()
            .map(|c| c.stations())
            .unwrap_or_default()
            .iter()
            .map(|(serial, p)| align::StationPose {
                serial: serial.clone(),
                pose: align::PoseEntry::from_iso(p),
            })
            .collect(),
        play_area: mean_position(pairs.iter().map(|p| &p.0)),
    };
    match align::save(&align::registration_path(), &file) {
        Ok(backup) => {
            if let Some(b) = backup {
                println!("previous registration kept as {}", b.display());
            }
            println!("installed {}", align::registration_path().display());
        }
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return 1;
        }
    }
    send("alignment reload")
}

fn vr_pulse(index: &str, ms: &str) -> i32 {
    let (Ok(index), Ok(ms)) = (index.parse::<u32>(), ms.parse::<u32>()) else {
        usage()
    };
    match vr::Client::connect() {
        Ok(c) => {
            // legacy pulses are at most ~4 ms each: repeat them for the requested length
            let until = Instant::now() + Duration::from_millis(ms as u64);
            while Instant::now() < until {
                c.haptic_pulse(index, 3999);
                thread::sleep(Duration::from_millis(5));
            }
            0
        }
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

fn vrserver_running() -> Result<bool, String> {
    platform::process::running("vrserver")
        .map_err(|e| format!("cannot check SteamVR processes: {e}"))
}

fn steamvr_check(legacy: Option<(&Path, &Path)>) -> i32 {
    let assets = steamvr::driver_assets().ok();
    let legacy = legacy.or_else(|| {
        assets
            .as_ref()
            .map(|(so, manifest)| (so.as_path(), manifest.as_path()))
    });
    match steamvr::check_driver(&steamvr::driver_dir(), legacy) {
        Ok(()) => 0,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

fn steamvr_install(legacy: Option<(&Path, &Path)>) -> i32 {
    match vrserver_running() {
        Ok(false) => {}
        Ok(true) => {
            eprintln!(
                "cv1touch: quit SteamVR first (it rewrites steamvr.vrsettings when it exits)"
            );
            return 1;
        }
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return 1;
        }
    }
    if let Err(e) = steamvr::validate_settings(&steamvr::vrsettings_path()) {
        eprintln!("cv1touch: {e}");
        return 1;
    }
    if !steamvr::vrpathreg().is_file() {
        eprintln!(
            "cv1touch: vrpathreg missing at {}; set CV1TOUCH_STEAMVR",
            steamvr::vrpathreg().display()
        );
        return 1;
    }
    let (so, manifest) = match steamvr::driver_assets() {
        Ok(paths) => paths,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return 1;
        }
    };
    let dir = steamvr::driver_dir();
    let was_registered = match steamvr::driver_registered(&dir) {
        Ok(value) => value,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return 1;
        }
    };
    if let Err(e) =
        steamvr::assemble_with_legacy(&so, &manifest, &dir, legacy.or(Some((&so, &manifest))))
    {
        eprintln!("cv1touch: {e}");
        return 1;
    }
    println!("driver files in {}", dir.display());
    match Command::new(steamvr::vrpathreg())
        .arg("adddriver")
        .arg(&dir)
        .status()
    {
        Ok(s) if s.success() => println!("registered with vrpathreg"),
        Ok(s) => {
            eprintln!("cv1touch: vrpathreg adddriver failed: {s}");
            return 1;
        }
        Err(e) => {
            eprintln!("cv1touch: {}: {e}", steamvr::vrpathreg().display());
            return 1;
        }
    }
    match steamvr::enable_multiple_drivers(&steamvr::vrsettings_path()) {
        Ok(Some(b)) => println!(
            "activateMultipleDrivers enabled (previous settings kept as {})",
            b.display()
        ),
        Ok(None) => println!("activateMultipleDrivers already enabled"),
        Err(e) => {
            eprintln!("cv1touch: {e}");
            if !was_registered {
                match Command::new(steamvr::vrpathreg()).arg("removedriver").arg(&dir).status() {
                    Ok(s) if s.success() => { eprintln!("cv1touch: rolled back new driver registration"); },
                    result => eprintln!("cv1touch: registration rollback failed ({result:?}); run `cv1touch steamvr uninstall` after repairing SteamVR"),
                }
            }
            return 1;
        }
    }
    println!("start SteamVR: the CV1 Touch controllers appear as soon as cv1touchd runs");
    0
}

fn steamvr_uninstall() -> i32 {
    if steamvr_check(None) != 0 {
        return 1;
    }
    match Command::new(steamvr::vrpathreg())
        .arg("removedriver")
        .arg(steamvr::driver_dir())
        .status()
    {
        Ok(s) if s.success() => {
            if let Err(e) = steamvr::remove_owned_driver(&steamvr::driver_dir()) {
                eprintln!("cv1touch: unregistered, but {e}");
                return 1;
            }
            println!(
                "unregistered and removed {} (activateMultipleDrivers left as is)",
                steamvr::driver_dir().display()
            );
            0
        }
        Ok(s) => {
            eprintln!("cv1touch: vrpathreg removedriver failed: {s}");
            1
        }
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

const EVAL_BRIEFING: &str = "\
Accuracy and latency capture: 80 s against Lighthouse truth, paced by beeps (press Enter to start each phase).
  Keep the Touch fixed to the left Index controller (the jig) the whole time.
  still (10 s): set the pair down where the sensors and base stations see it; hands off.
  move  (60 s): move the pair through the play space like you would in a game: reach, swing,
                turn your wrist, fast and slow, high and low.
  still (10 s): set it down again, hands off.";

/// Stops the daemon's recording when dropped unless disarmed: a terminal closed mid-capture makes
/// printing fail, and the panic must not leave the daemon recording.
struct StopRecordingGuard(bool);

impl Drop for StopRecordingGuard {
    fn drop(&mut self) {
        if self.0 {
            let _ = control::request(
                &control::socket_path(),
                "record stop",
                Duration::from_secs(5),
            );
        }
    }
}

fn eval_capture() -> i32 {
    catch_interrupts();
    let status = status_json();
    if status.as_ref().and_then(|v| v["mode"].as_str()) != Some("calibrated") {
        eprintln!("cv1touch: cv1touchd must be running in calibrated mode");
        return 1;
    }
    let latency_adjustment_ms = latency_adjustment(status.as_ref());
    let client = match vr::Client::connect() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return 1;
        }
    };
    let Some(reference) = vr::find_reference(&client.devices()).cloned() else {
        eprintln!(
            "cv1touch: no Lighthouse-tracked Index controller: turn on the left Index controller"
        );
        return 1;
    };
    let universe = client
        .uint64_property(reference.index, vr::PROP_CURRENT_UNIVERSE_ID)
        .unwrap_or(0);
    let base = state_dir().join(format!("eval-{}", unix_now()));
    let rec = base.with_extension("rec");
    if let Err(e) = std::fs::create_dir_all(state_dir()) {
        eprintln!("cv1touch: {e}");
        return 1;
    }
    println!(
        "reference: Index controller {} (device {})\n\n{EVAL_BRIEFING}\n",
        reference.serial, reference.index
    );
    if send_path(control::record_start_line(&rec, false)) != 0 {
        return 1;
    }
    let mut guard = StopRecordingGuard(true);
    let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let sampler = {
        let stop = stop.clone();
        let index = reference.index as usize;
        std::thread::spawn(move || {
            let mut out: Vec<[f64; 8]> = Vec::with_capacity(60_000);
            while !stop.load(std::sync::atomic::Ordering::Relaxed) {
                let now = shm::now_ns();
                if let Some(Some(p)) = client.raw_poses().get(index).copied() {
                    out.push(eval::truth_row(now, &p));
                }
                std::thread::sleep(Duration::from_millis(2));
            }
            out
        })
    };
    let paced = pace_windows(&[
        "still:10".to_string(),
        "move:60".to_string(),
        "still:10".to_string(),
    ]);
    stop.store(true, std::sync::atomic::Ordering::Relaxed);
    let samples = sampler.join().unwrap_or_default();
    guard.0 = false;
    send("record stop");
    let windows = match paced {
        Ok(w) => w,
        Err(code) => {
            eprintln!(
                "cv1touch: interrupted (recording kept at {})",
                rec.display()
            );
            return code;
        }
    };
    let file = eval::TruthFile {
        format: eval::TRUTH_FORMAT.into(),
        version: 1,
        reference: reference.serial.clone(),
        universe_id: universe,
        windows,
        samples,
        others: Vec::new(),
        latency_adjustment_ms,
    };
    let truth_path = base.with_extension("truth.json");
    if let Err(e) = std::fs::write(
        &truth_path,
        serde_json::to_string(&file).unwrap_or_default(),
    ) {
        eprintln!("cv1touch: {}: {e}", truth_path.display());
        return 1;
    }
    println!(
        "recording {}\ntruth {} ({} samples)\n",
        rec.display(),
        truth_path.display(),
        file.samples.len()
    );
    eval_cmd(&base)
}

#[derive(Clone, serde::Deserialize, serde::Serialize)]
struct RawInput {
    #[serde(default)]
    buttons: Option<u32>,
    identity: String,
    connected: bool,
    received_host_ns: u64,
    trigger: f32,
    grip: f32,
    stick: [f32; 2],
}

#[derive(serde::Deserialize)]
struct RawInputs {
    hands: [RawInput; 2],
}

fn parse_raw_inputs(reply: &str) -> Result<RawInputs, String> {
    let json = reply
        .strip_prefix("ok ")
        .ok_or_else(|| format!("cannot read raw inputs: {reply}"))?;
    let inputs: RawInputs = serde_json::from_str(json)
        .map_err(|e| format!("invalid raw input reply (axes must be finite numbers): {e}"))?;
    if inputs.hands.iter().any(|hand| {
        [hand.trigger, hand.grip, hand.stick[0], hand.stick[1]]
            .iter()
            .any(|v| !v.is_finite())
    }) {
        return Err("raw inputs contain nonfinite axes".into());
    }
    Ok(inputs)
}

fn raw_inputs() -> Result<RawInputs, String> {
    control::request(&control::socket_path(), "inputs", Duration::from_secs(1))
        .map_err(|e| format!("cannot read raw inputs from cv1touchd: {e}"))
        .and_then(|reply| parse_raw_inputs(&reply))
}

fn raw_inputs_json() -> i32 {
    let result = (|| -> Result<String, String> {
        let reply = control::request(&control::socket_path(), "inputs", Duration::from_secs(5))
            .map_err(|e| e.to_string())?;
        parse_raw_inputs(&reply)?;
        let value: serde_json::Value =
            serde_json::from_str(reply.strip_prefix("ok ").ok_or("invalid raw input reply")?)
                .map_err(|e| e.to_string())?;
        serde_json::to_string_pretty(&value).map_err(|e| e.to_string())
    })();
    match result {
        Ok(json) => {
            println!("{json}");
            0
        }
        Err(error) => {
            eprintln!("cv1touch: {error}");
            1
        }
    }
}

fn input_identities_match(inputs: &RawInputs, identities: &[String; 2]) -> Result<(), String> {
    for (hand, expected) in inputs.hands.iter().zip(identities) {
        if hand.identity.len() != 32
            || !hand
                .identity
                .bytes()
                .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
        {
            return Err(
                "controller identity is unavailable; wake both controllers and retry".into(),
            );
        }
        if &hand.identity != expected {
            return Err(
                "a physical controller changed during calibration; no calibration was saved".into(),
            );
        }
    }
    Ok(())
}

#[derive(Default)]
struct InputPhase {
    last_ns: u64,
    samples: Vec<RawInput>,
}

impl InputPhase {
    fn observe(
        &mut self,
        inputs: &RawInputs,
        identities: &[String; 2],
        hand: usize,
        now_ns: u64,
    ) -> Result<bool, String> {
        input_identities_match(inputs, identities)?;
        let sample = &inputs.hands[hand];
        if !sample.connected {
            return Err(format!(
                "{} controller disconnected; wake it and retry",
                ["left", "right"][hand]
            ));
        }
        if sample.received_host_ns == 0
            || now_ns
                .checked_sub(sample.received_host_ns)
                .is_none_or(|age| age > 500_000_000)
        {
            return Err(format!(
                "{} input sample is stale or has an invalid timestamp",
                ["left", "right"][hand]
            ));
        }
        if sample.received_host_ns < self.last_ns {
            return Err("input timestamps moved backward; retry calibration".into());
        }
        if sample.received_host_ns == self.last_ns {
            return Ok(false);
        }
        if [
            sample.trigger,
            sample.grip,
            sample.stick[0],
            sample.stick[1],
        ]
        .iter()
        .any(|v| !v.is_finite() || v.abs() > 4.0)
        {
            return Err(
                "raw input sample is outside the supported factory-coordinate range".into(),
            );
        }
        self.last_ns = sample.received_host_ns;
        self.samples.push(sample.clone());
        Ok(true)
    }

    fn finish(self, minimum: usize) -> Result<Vec<RawInput>, String> {
        if self.samples.len() < minimum {
            return Err(format!("only {} fresh input samples; need at least {minimum}. Wake the controller and retry", self.samples.len()));
        }
        Ok(self.samples)
    }
}

struct PairedInputPhase {
    hands: [InputPhase; 2],
}

impl PairedInputPhase {
    fn new(last_ns: [u64; 2]) -> Self {
        Self {
            hands: last_ns.map(|last_ns| InputPhase {
                last_ns,
                ..Default::default()
            }),
        }
    }

    fn observe(
        &mut self,
        inputs: &RawInputs,
        identities: &[String; 2],
        now_ns: u64,
    ) -> Result<(), String> {
        for (hand, phase) in self.hands.iter_mut().enumerate() {
            phase.observe(inputs, identities, hand, now_ns)?;
        }
        Ok(())
    }

    fn last_ns(&self) -> [u64; 2] {
        std::array::from_fn(|hand| self.hands[hand].last_ns)
    }

    fn finish(self, minimum: usize) -> Result<[Vec<RawInput>; 2], String> {
        let [left, right] = self.hands;
        Ok([
            left.finish(minimum).map_err(|e| format!("left: {e}"))?,
            right.finish(minimum).map_err(|e| format!("right: {e}"))?,
        ])
    }
}

fn input_quantile(
    samples: &[RawInput],
    axis: impl Fn(&RawInput) -> f32,
    q: f32,
) -> Result<f32, String> {
    let mut values: Vec<f32> = samples.iter().map(axis).collect();
    if values.is_empty() || values.iter().any(|v| !v.is_finite() || v.abs() > 4.0) {
        return Err("missing or invalid input samples".into());
    }
    values.sort_by(f32::total_cmp);
    let at = q * (values.len() - 1) as f32;
    let lo = at.floor() as usize;
    let hi = at.ceil() as usize;
    Ok(values[lo] + (values[hi] - values[lo]) * (at - lo as f32))
}

fn measured_inputs(
    identity: &str,
    phases: &[Vec<RawInput>; 4],
) -> Result<cv1touch::input_calibration::HandCalibration, String> {
    use cv1touch::input_calibration::{Endpoint, HandCalibration, StickAxis};
    Ok(HandCalibration {
        identity: identity.into(),
        trigger: Endpoint {
            released: input_quantile(&phases[0], |s| s.trigger, 0.5)?,
            pressed: input_quantile(&phases[1], |s| s.trigger, 0.5)?,
        },
        grip: Endpoint {
            released: input_quantile(&phases[0], |s| s.grip, 0.5)?,
            pressed: input_quantile(&phases[2], |s| s.grip, 0.5)?,
        },
        stick: [
            StickAxis {
                min: input_quantile(&phases[3], |s| s.stick[0], 0.02)?,
                center: input_quantile(&phases[0], |s| s.stick[0], 0.5)?,
                max: input_quantile(&phases[3], |s| s.stick[0], 0.98)?,
            },
            StickAxis {
                min: input_quantile(&phases[3], |s| s.stick[1], 0.02)?,
                center: input_quantile(&phases[0], |s| s.stick[1], 0.5)?,
                max: input_quantile(&phases[3], |s| s.stick[1], 0.98)?,
            },
        ],
    })
}

/// Drops whatever was typed before a prompt, so an Enter pressed during a capture cannot answer
/// the next question or start the next step.
fn discard_typed_ahead() {
    console::discard_typed_ahead();
}

/// A line typed at a start prompt: Some(true) Enter starts, Some(false) quit, None ask again.
fn start_line(line: &str) -> Option<bool> {
    match line.trim().to_ascii_lowercase().as_str() {
        "" => Some(true),
        "q" | "quit" | "exit" => Some(false),
        _ => None,
    }
}

/// The setup guide's introduction to input calibration.
const INPUTS_GUIDE: &str = "This measures how far each trigger, grip and thumbstick moves, so in SteamVR a full pull reads\nas a full pull.\nEach step starts recording (one beep) as soon as both hands are in position (Enter also starts\nit) and ends at the double beep.";

/// The four input calibration steps, in order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CalStep {
    Rest,
    Triggers,
    Grips,
    Sticks,
}

impl CalStep {
    const ALL: [CalStep; 4] = [
        CalStep::Rest,
        CalStep::Triggers,
        CalStep::Grips,
        CalStep::Sticks,
    ];

    /// Seconds recorded and the fewest fresh samples each hand must give.
    fn length(self) -> (u64, usize) {
        match self {
            CalStep::Sticks => (10, 150),
            _ => (2, 30),
        }
    }

    /// Whether one awake hand is in this step's position, on raw factory values. Loose on
    /// purpose: it only decides when to start recording; the recording sets the endpoints.
    fn in_position(self, h: &RawInput) -> bool {
        let stick = (h.stick[0] * h.stick[0] + h.stick[1] * h.stick[1]).sqrt();
        h.connected
            && match self {
                CalStep::Rest => h.trigger < 0.15 && h.grip < 0.15 && stick < 0.25,
                CalStep::Triggers => h.trigger > 0.75 && h.grip < 0.5,
                CalStep::Grips => h.grip > 0.75 && h.trigger < 0.5,
                CalStep::Sticks => stick > 0.6 && h.trigger < 0.5 && h.grip < 0.5,
            }
    }

    /// The standalone command's prompt (Enter starts the recording).
    fn prompt(self) -> &'static str {
        match self {
            CalStep::Rest => "Input calibration 1/4: neutral controls.\nRelease both triggers and grips; let both thumbsticks center.\nPress Enter, then leave all controls untouched until 'Done' appears (2 seconds).",
            CalStep::Triggers => "Input calibration 2/4: triggers.\nKeep both grips released and both thumbsticks centered.\nPress Enter first. Immediately pull both triggers fully and keep them held until 'Done' appears (2 seconds).",
            CalStep::Grips => "Input calibration 3/4: grips.\nKeep both triggers released and both thumbsticks centered.\nPress Enter first. Immediately squeeze both grips fully and keep them held until 'Done' appears (2 seconds).",
            CalStep::Sticks => "Input calibration 4/4: thumbsticks.\nRelease both triggers and grips.\nPress Enter first. Immediately move BOTH thumbsticks in full circles along their outer edges until 'Done' appears (10 seconds).\nCover forward, back, left and right; keep the sticks against their outer edges.",
        }
    }

    /// What the guide asks for (numbered by the caller).
    fn guided(self) -> &'static str {
        match self {
            CalStep::Rest => "Let go of both triggers, grips and sticks, until the double beep.",
            CalStep::Triggers => {
                "Pull both triggers all the way and hold them there until the double beep."
            }
            CalStep::Grips => {
                "Let go of the triggers and squeeze both grips fully until the double beep."
            }
            CalStep::Sticks => "Let go of the grips and circle both sticks around their outer edges until the double beep.",
        }
    }
}

/// Both hands held in a step's position for `HOLD_NS` without a break.
#[derive(Default)]
struct InPosition {
    since: Option<u64>,
}

impl InPosition {
    const HOLD_NS: u64 = 300_000_000;

    fn update(&mut self, step: CalStep, raw: &RawInputs, now_ns: u64) -> bool {
        if !raw.hands.iter().all(|h| step.in_position(h)) {
            self.since = None;
            return false;
        }
        let since = *self.since.get_or_insert(now_ns);
        now_ns.saturating_sub(since) >= Self::HOLD_NS
    }
}

/// Waits until both hands hold `step`'s position (or Enter, offered after a while for controls
/// that never reach the expected travel).
fn input_gate(step: CalStep) -> Result<(), String> {
    discard_typed_ahead();
    let start = Instant::now();
    let mut gate = InPosition::default();
    let mut hinted = false;
    loop {
        if interrupted() {
            return Err("input calibration cancelled; no calibration saved".into());
        }
        match console::next_line(Duration::from_millis(30)) {
            Line::Closed => {
                return Err("input calibration needs an interactive terminal; input closed".into())
            }
            Line::Typed(line) => match start_line(&line) {
                Some(true) => return Ok(()),
                Some(false) => return Err("input calibration stopped; no calibration saved".into()),
                None => println!("      (Enter records now, q quits)"),
            },
            Line::NotYet => {}
        }
        if gate.update(step, &raw_inputs()?, shm::now_ns()) {
            return Ok(());
        }
        if !hinted && start.elapsed() > Duration::from_secs(8) {
            hinted = true;
            println!("      (not detected yet: press Enter to record anyway)");
        }
    }
}

/// `pulses` short buzzes from both controllers and as many beeps from the speakers: one starts a timed step, two
/// end it. Best effort: a missed cue does not stop anything.
fn cue(pulses: usize) {
    for pulse in 0..pulses {
        for hand in ["left", "right"] {
            let _ = control::request(
                &control::socket_path(),
                &format!("haptic {hand} 0.12 160 0.6"),
                Duration::from_millis(300),
            );
        }
        if pulse + 1 < pulses {
            thread::sleep(Duration::from_millis(180));
        }
    }
    beeps(pulses);
}

fn input_enter(prompt: &str) -> Result<(), String> {
    human_action(&format!("{prompt}\nPress Enter when ready."));
    io::stdout().flush().map_err(|e| e.to_string())?;
    loop {
        if interrupted() {
            return Err("input calibration cancelled; no calibration saved".into());
        }
        match console::next_line(Duration::from_millis(100)) {
            Line::Closed => {
                return Err(
                    "input calibration requires Enter confirmation; standard input is closed"
                        .into(),
                )
            }
            Line::Typed(_) => return Ok(()),
            Line::NotYet => {}
        }
    }
}

fn collect_inputs(
    identities: &[String; 2],
    last_ns: &mut [u64; 2],
    seconds: u64,
    minimum: usize,
    quiet: bool,
) -> Result<[Vec<RawInput>; 2], String> {
    if !quiet {
        println!("Sampling both hands for {seconds} seconds...");
    }
    let deadline = Instant::now() + Duration::from_secs(seconds);
    let mut phase = PairedInputPhase::new(*last_ns);
    while Instant::now() < deadline {
        if interrupted() {
            return Err("input calibration cancelled; no calibration saved".into());
        }
        let tick = Instant::now();
        phase.observe(&raw_inputs()?, identities, shm::now_ns())?;
        thread::sleep(Duration::from_millis(50).saturating_sub(tick.elapsed()));
    }
    *last_ns = phase.last_ns();
    let samples = phase.finish(minimum)?;
    if !quiet {
        println!(
            "Done: left {} / right {} fresh samples.",
            samples[0].len(),
            samples[1].len()
        );
    }
    Ok(samples)
}

fn private_input_bytes(path: &Path, bytes: &[u8]) -> Result<(), String> {
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    let staged = parent.join(format!(
        ".input-calibration-{}-{}",
        process::id(),
        shm::now_ns()
    ));
    let mut created = false;
    let result = (|| {
        let mut file = platform::fs::create_private(&staged).map_err(|e| e.to_string())?;
        created = true;
        file.write_all(bytes)
            .and_then(|_| file.sync_all())
            .map_err(|e| e.to_string())?;
        platform::fs::rename_replacing(&staged, path).map_err(|e| e.to_string())?;
        platform::fs::sync_dir(parent).map_err(|e| e.to_string())
    })();
    if created {
        let _ = std::fs::remove_file(staged);
    }
    result
}

fn remove_input_file(path: &Path) -> Result<(), String> {
    match std::fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(e) => Err(format!("{}: {e}", path.display())),
    }
}

fn previous_input_bytes(path: &Path) -> Result<Option<Vec<u8>>, String> {
    use std::io::Read;
    let file = match platform::fs::open_read_nofollow(path) {
        Ok(file) => file,
        Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(format!("{}: {e}", path.display())),
    };
    let metadata = file.metadata().map_err(|e| e.to_string())?;
    const MAX_BACKUP: u64 = 1_048_576;
    if !metadata.is_file() || metadata.len() > MAX_BACKUP {
        return Err(format!(
            "refusing nonregular or oversized input calibration: {}",
            path.display()
        ));
    }
    let mut bytes = Vec::new();
    file.take(MAX_BACKUP + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() as u64 > MAX_BACKUP {
        return Err("input calibration grew beyond the backup limit".into());
    }
    Ok(Some(bytes))
}

fn write_input_calibration(
    path: &Path,
    candidate: Option<&cv1touch::input_calibration::CalibrationFile>,
    reload: Option<&mut dyn FnMut() -> Result<(), String>>,
) -> Result<(), String> {
    write_input_calibration_with(path, candidate, reload, cv1touch::input_calibration::save)
}

fn write_input_calibration_with(
    path: &Path,
    candidate: Option<&cv1touch::input_calibration::CalibrationFile>,
    mut reload: Option<&mut dyn FnMut() -> Result<(), String>>,
    save: impl FnOnce(&Path, &cv1touch::input_calibration::CalibrationFile) -> Result<(), String>,
) -> Result<(), String> {
    if let Some(file) = candidate {
        file.validate()?;
    }
    let previous = previous_input_bytes(path)?;
    if let Some(bytes) = &previous {
        let mut name = path
            .file_name()
            .ok_or("input calibration path needs a filename")?
            .to_os_string();
        name.push(format!(".bak-{}-{}", unix_now(), shm::now_ns()));
        let backup = path.with_file_name(name);
        private_input_bytes(&backup, bytes)?;
        println!("Backup: {}", backup.display());
    }
    let saved = if let Some(file) = candidate {
        save(path, file)
    } else {
        remove_input_file(path)
    };
    let outcome = saved.and_then(|_| match &mut reload {
        Some(reload) => reload(),
        None => Ok(()),
    });
    if let Err(error) = outcome {
        let restored = if let Some(bytes) = &previous {
            private_input_bytes(path, bytes)
        } else {
            remove_input_file(path)
        };
        restored.map_err(|rollback| {
            format!("input calibration update failed: {error}; file rollback failed: {rollback}")
        })?;
        let recovery = reload
            .as_mut()
            .and_then(|reload| reload().err())
            .map(|e| format!("; restored file could not reload: {e}"))
            .unwrap_or_default();
        return Err(format!(
            "input calibration update failed: {error}; previous input file restored{recovery}"
        ));
    }
    Ok(())
}

fn reload_inputs() -> Result<(), String> {
    let reply = control::request(
        &control::socket_path(),
        "inputs reload",
        Duration::from_secs(5),
    )
    .map_err(|e| e.to_string())?;
    if reply == "ok" || reply.starts_with("ok ") {
        Ok(())
    } else {
        Err(reply)
    }
}

fn calibrate_inputs(out: Option<&Path>, reset: bool) -> i32 {
    calibrate_inputs_with(out, reset, false)
}

/// `guided`: the setup guide's version: each step records as soon as both hands are in position,
/// with buzzes and beeps instead of Enter, and nothing printed but the steps.
fn calibrate_inputs_with(out: Option<&Path>, reset: bool, guided: bool) -> i32 {
    match calibrate_inputs_result(out, reset, guided) {
        Ok(()) => 0,
        Err(error) => {
            eprintln!("cv1touch: {error}");
            if interrupted() {
                130
            } else {
                1
            }
        }
    }
}

/// `calibrate_inputs_with`, the error as the guide shows it.
fn calibrate_inputs_result(out: Option<&Path>, reset: bool, guided: bool) -> Result<(), String> {
    catch_interrupts();
    let path = out
        .map(Path::to_path_buf)
        .unwrap_or_else(cv1touch::input_calibration::default_path);
    (|| -> Result<(), String> {
        if reset {
            write_input_calibration(&path, None, Some(&mut reload_inputs))?;
            println!("Input calibration reset; factory input mapping reloaded.");
            return Ok(());
        }
        if guided {
            human_action(INPUTS_GUIDE);
        } else {
            input_enter("Input calibration uses raw factory inputs. Wake both controllers.\nIt changes triggers, grips and sticks only; tracking and SteamVR can keep running.")?;
        }
        let first = raw_inputs()?;
        let identities = std::array::from_fn(|hand| first.hands[hand].identity.clone());
        for hand in 0..2 {
            InputPhase::default().observe(&first, &identities, hand, shm::now_ns())?;
        }
        let mut last_ns = std::array::from_fn(|hand| first.hands[hand].received_host_ns);
        let mut phases: [[Vec<RawInput>; 4]; 2] =
            std::array::from_fn(|_| std::array::from_fn(|_| Vec::new()));
        for (phase, step) in CalStep::ALL.into_iter().enumerate() {
            let (seconds, minimum) = step.length();
            if guided {
                human_action(&format!("  {}/4 {}", phase + 1, step.guided()));
                input_gate(step)?;
                cue(1);
            } else {
                input_enter(step.prompt())?;
            }
            let samples = collect_inputs(&identities, &mut last_ns, seconds, minimum, guided)?;
            if guided {
                cue(2);
            }
            for (hand, samples) in samples.into_iter().enumerate() {
                phases[hand][phase] = samples;
            }
        }
        let mut measured = Vec::new();
        for (hand, identity) in identities.iter().enumerate() {
            let name = ["left", "right"][hand];
            let calibration = measured_inputs(identity, &phases[hand])?;
            calibration.validate().map_err(|e| format!("{name} input measurements rejected: {e}; retry with released controls and full travel"))?;
            if !guided {
                println!("{name}: trigger {:.3}..{:.3}, grip {:.3}..{:.3}; stick X {:.3}/{:.3}/{:.3}, Y {:.3}/{:.3}/{:.3}", calibration.trigger.released, calibration.trigger.pressed, calibration.grip.released, calibration.grip.pressed, calibration.stick[0].min, calibration.stick[0].center, calibration.stick[0].max, calibration.stick[1].min, calibration.stick[1].center, calibration.stick[1].max);
            }
            measured.push(calibration);
        }
        let candidate = cv1touch::input_calibration::CalibrationFile::new(
            measured
                .try_into()
                .map_err(|_| "both hands must be calibrated")?,
        )?;
        if !guided {
            input_enter("Measurements accepted. Wake both controllers again if needed; press Enter to verify their identities and save.")?;
        }
        let final_inputs = raw_inputs()?;
        for hand in 0..2 {
            InputPhase::default().observe(&final_inputs, &identities, hand, shm::now_ns())?;
        }
        if interrupted() {
            return Err("input calibration cancelled; no calibration saved".into());
        }
        if out.is_some() {
            write_input_calibration(&path, Some(&candidate), None)?;
            println!(
                "Saved {}. The daemon was not reloaded (--out).",
                path.display()
            );
        } else {
            write_input_calibration(&path, Some(&candidate), Some(&mut reload_inputs))?;
            if !guided {
                println!(
                    "Saved {} and reloaded inputs. Run `cv1touch input-test` to verify full travel.",
                    path.display()
                );
            }
        }
        Ok(())
    })()
}

/// The Oculus Touch profile's legacy input mapping (axis 0 joystick, 1 trigger, 2 grip;
/// button bits 7 A/X, 1 B/Y, 32 joystick, 33 trigger). Unsupported inputs need a manual check.
#[derive(Clone, Copy)]
enum LegacyInput {
    Axis {
        axis: usize,
        component: usize,
        threshold: f32,
    },
    Press(u64),
    Touch {
        mask: u64,
        released: u64,
        trigger_released: bool,
    },
}

struct InputStep {
    prompt: String,
    done: Box<dyn Fn(&shm::ControllerState) -> bool>,
    steam: Option<LegacyInput>,
}

#[derive(Default)]
struct InputProgress {
    daemon_seen: bool,
    steam_seen: bool,
    daemon_trigger: Option<(f32, f32)>,
    steam_trigger: Option<(f32, f32)>,
    daemon_grip: Option<(f32, f32)>,
    steam_grip: Option<(f32, f32)>,
}

impl InputProgress {
    fn observe(
        &mut self,
        step: &InputStep,
        daemon: Option<&shm::ControllerState>,
        steam: Option<&vr::ControllerState>,
    ) -> bool {
        let range = |range: &mut Option<(f32, f32)>, value: f32| {
            if value.is_finite() {
                *range = Some(range.map_or((value, value), |(min, max)| {
                    (min.min(value), max.max(value))
                }));
            }
        };
        if let Some(state) = daemon {
            range(&mut self.daemon_trigger, state.trigger);
            range(&mut self.daemon_grip, state.grip);
        }
        if let Some(state) = steam {
            range(&mut self.steam_trigger, state.2[1].0);
            range(&mut self.steam_grip, state.2[2].0);
        }
        let daemon_now = daemon.is_some_and(|state| (step.done)(state));
        self.daemon_seen |= daemon_now;
        let steam_now = step.steam.is_some_and(|expected| {
            steam.is_some_and(|state| match expected {
                LegacyInput::Axis {
                    axis,
                    component,
                    threshold,
                } => {
                    let value = if component == 0 {
                        state.2[axis].0
                    } else {
                        state.2[axis].1
                    };
                    value.is_finite()
                        && if threshold > 0.0 {
                            value > threshold
                        } else {
                            value < threshold
                        }
                }
                LegacyInput::Press(mask) => state.0 & mask != 0,
                LegacyInput::Touch {
                    mask,
                    released,
                    trigger_released,
                } => {
                    state.1 & mask != 0
                        && state.0 & released == 0
                        && (!trigger_released || state.2[1].0 < 0.2)
                }
            })
        });
        self.steam_seen |= daemon_now && steam_now;
        input_delivery_ok(daemon_now, step.steam.is_none() || steam_now)
    }

    fn analog_report(&self, axis: usize) -> String {
        let range = |r: Option<(f32, f32)>| {
            r.map_or_else(
                || "unavailable".into(),
                |(min, max)| format!("{min:.3}..{max:.3}"),
            )
        };
        let (name, daemon, steam) = if axis == 1 {
            ("trigger", self.daemon_trigger, self.steam_trigger)
        } else {
            ("grip", self.daemon_grip, self.steam_grip)
        };
        format!(
            "{name} range: daemon {}, SteamVR {}; full travel requires > 0.9",
            range(daemon),
            range(steam)
        )
    }
}

fn input_steps(hand: usize) -> Vec<InputStep> {
    use cv1touch::hand as h;
    let right = hand == HAND_RIGHT;
    let (a, b, ta, tb) = if right {
        ("A", "B", h::TOUCH_A, h::TOUCH_B)
    } else {
        ("X", "Y", h::TOUCH_X, h::TOUCH_Y)
    };
    let (ba, bb) = if right {
        (h::BTN_A, h::BTN_B)
    } else {
        (h::BTN_X, h::BTN_Y)
    };
    let step =
        |prompt: String, done: Box<dyn Fn(&shm::ControllerState) -> bool>, steam| InputStep {
            prompt,
            done,
            steam,
        };
    let axis = |axis, component, threshold| {
        Some(LegacyInput::Axis {
            axis,
            component,
            threshold,
        })
    };
    let press = |bit| Some(LegacyInput::Press(1_u64 << bit));
    let touch = |bit, released, trigger_released| {
        Some(LegacyInput::Touch {
            mask: 1_u64 << bit,
            released,
            trigger_released,
        })
    };
    let mut v = vec![
        step(
            "pull the trigger all the way".into(),
            Box::new(|s| s.trigger > 0.9),
            axis(1, 0, 0.9),
        ),
        step(
            "squeeze the grip all the way".into(),
            Box::new(|s| s.grip > 0.9),
            axis(2, 0, 0.9),
        ),
        step(
            "push the stick forward".into(),
            Box::new(|s| s.stick[1] > 0.8),
            axis(0, 1, 0.8),
        ),
        step(
            "pull the stick back".into(),
            Box::new(|s| s.stick[1] < -0.8),
            axis(0, 1, -0.8),
        ),
        step(
            "push the stick left".into(),
            Box::new(|s| s.stick[0] < -0.8),
            axis(0, 0, -0.8),
        ),
        step(
            "push the stick right".into(),
            Box::new(|s| s.stick[0] > 0.8),
            axis(0, 0, 0.8),
        ),
        step(
            "click the stick down".into(),
            Box::new(|s| s.buttons & h::BTN_STICK != 0),
            press(32),
        ),
        step(
            format!("press {a}"),
            Box::new(move |s| s.buttons & ba != 0),
            press(7),
        ),
        step(
            format!("press {b}"),
            Box::new(move |s| s.buttons & bb != 0),
            press(1),
        ),
        step(
            format!("rest a finger on {a} without pressing"),
            Box::new(move |s| s.touches & ta != 0 && s.buttons & ba == 0),
            touch(7, 1 << 7, false),
        ),
        step(
            format!("rest a finger on {b} without pressing"),
            Box::new(move |s| s.touches & tb != 0 && s.buttons & bb == 0),
            touch(1, 1 << 1, false),
        ),
        step(
            "rest a finger on the trigger without pulling".into(),
            Box::new(|s| s.touches & h::TOUCH_TRIGGER != 0 && s.trigger < 0.2),
            touch(33, 1 << 33, true),
        ),
        step(
            "rest the thumb on the stick without moving it".into(),
            Box::new(|s| s.touches & h::TOUCH_STICK != 0),
            touch(32, 0, false),
        ),
        step(
            "rest the thumb on the thumb rest".into(),
            Box::new(|s| s.touches & h::TOUCH_THUMBREST != 0),
            None,
        ),
    ];
    if !right {
        v.push(step(
            "press the menu button (the one below Y)".into(),
            Box::new(|s| s.buttons & h::BTN_SYSTEM != 0),
            None,
        ));
    }
    v
}

/// Walks through every input of both Touches and checks each in the daemon's published state and
/// in SteamVR's exact supported legacy input. Menu/thumb-rest need a manual check, then a
/// haptic pulse per hand.
fn input_test() -> i32 {
    catch_interrupts();
    let reader = match ShmReader::open(shm::DEFAULT_NAME) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("cv1touch: cv1touchd is not running ({e})");
            return 1;
        }
    };
    let client = vr::Client::connect().ok();
    let devices = client.as_ref().map(|c| c.devices()).unwrap_or_default();
    let index_of = |serial: &str| devices.iter().find(|d| d.serial == serial).map(|d| d.index);
    if client.is_none() || index_of("CV1T-LEFT").is_none() || index_of("CV1T-RIGHT").is_none() {
        eprintln!("cv1touch: input-test requires SteamVR with both cv1touch controllers; start SteamVR and check `cv1touch doctor`");
        return 1;
    }
    let mut results: Vec<(String, bool, String)> = Vec::new();
    let (mut daemon_seen, mut daemon_total, mut steam_seen, mut steam_total) = (0, 0, 0, 0);
    let mut manual = Vec::new();
    println!("Keep SteamVR input focus available; unsupported menu/thumb-rest delivery needs a manual check.");
    for hand in [HAND_LEFT, HAND_RIGHT] {
        let name = ["left", "right"][hand];
        let vr_index = index_of(if hand == HAND_LEFT {
            "CV1T-LEFT"
        } else {
            "CV1T-RIGHT"
        });
        println!("\n== {name} Touch (Enter skips a step)");
        for st in input_steps(hand) {
            print!("  {name}: {} ... ", st.prompt);
            let _ = io::stdout().flush();
            let mut progress = InputProgress::default();
            let deadline = Instant::now() + Duration::from_secs(20);
            let mut ok = false;
            let mut seen = String::from("-");
            while Instant::now() < deadline && !interrupted() {
                let daemon = reader.hand(hand);
                let steam = vr_index.and_then(|i| client.as_ref()?.controller_state(i));
                if progress.observe(&st, daemon.as_ref(), steam.as_ref()) {
                    ok = true;
                    seen = if st.steam.is_some() {
                        "required input observed".into()
                    } else {
                        "unverified by legacy interface; check manually in SteamVR".into()
                    };
                    break;
                }
                if console::next_line(Duration::from_millis(10)) != Line::NotYet {
                    break;
                }
            }
            daemon_total += 1;
            daemon_seen += usize::from(progress.daemon_seen);
            if st.steam.is_some() {
                steam_total += 1;
                steam_seen += usize::from(progress.steam_seen);
                if !ok {
                    seen = "required input not observed; check SteamVR input focus".into();
                }
            } else {
                seen = "unverified by legacy interface; check manually in SteamVR".into();
                manual.push(format!("{name}: {}", st.prompt));
            }
            if !ok {
                if let Some(LegacyInput::Axis {
                    axis: axis @ (1 | 2),
                    ..
                }) = st.steam
                {
                    seen.push_str(&format!("; {}", progress.analog_report(axis)));
                }
            }
            println!(
                "{}  [daemon: {}; SteamVR: {seen}]",
                if ok { "ok" } else { "NOT SEEN" },
                if progress.daemon_seen {
                    "seen"
                } else {
                    "not seen"
                }
            );
            results.push((format!("{name}: {}", st.prompt), ok, seen));
            // wait for the release before the next step
            let release = Instant::now() + Duration::from_secs(5);
            while ok && Instant::now() < release && reader.hand(hand).is_some_and(|s| (st.done)(&s))
            {
                std::thread::sleep(Duration::from_millis(10));
            }
            if interrupted() {
                return 130;
            }
        }
        if let (Some(c), Some(i)) = (client.as_ref(), vr_index) {
            print!("  {name}: haptic pulse sent through SteamVR: did it buzz? [y/n] ");
            let _ = io::stdout().flush();
            for _ in 0..5 {
                c.haptic_pulse(i, 3999);
                std::thread::sleep(Duration::from_millis(10));
            }
            let line = console::wait_line();
            if interrupted() {
                return 130;
            }
            let line = line.unwrap_or_default();
            results.push((
                format!("{name}: haptic"),
                line.trim().eq_ignore_ascii_case("y"),
                "-".into(),
            ));
        }
    }
    let failed: Vec<&(String, bool, String)> = results.iter().filter(|r| !r.1).collect();
    println!("\nDaemon inputs: {daemon_seen} of {daemon_total} seen; supported SteamVR inputs: {steam_seen} of {steam_total} seen");
    let haptics: Vec<_> = results
        .iter()
        .filter(|r| r.0.ends_with(": haptic"))
        .collect();
    println!(
        "Haptics: {} of {} confirmed",
        haptics.iter().filter(|r| r.1).count(),
        haptics.len()
    );
    println!(
        "Manual SteamVR checks (legacy interface cannot verify): {}",
        manual.join("; ")
    );
    for f in &failed {
        println!("  NOT SEEN: {}", f.0);
    }
    if failed.is_empty() {
        0
    } else {
        1
    }
}

fn input_delivery_ok(daemon_seen: bool, steamvr_seen: bool) -> bool {
    daemon_seen && steamvr_seen
}

/// Free-run capture: records the daemon and every Lighthouse-tracked device (controllers, also
/// ones switched on mid-capture, the headset, the base stations) until Enter, an interrupt or
/// `max_s` seconds.
fn eval_capture_free(max_s: u64) -> i32 {
    match capture_free(max_s, "Switch the controllers on, then move the jig through the play space: reach, swing, turn, fast and slow.") {
        Ok(_) => 0,
        Err(code) => code,
    }
}

/// The re-registration of a Lighthouse session: a free capture of the jig held still in many
/// orientations, the registration refitted from its holds, installed and loaded.
fn register_jig() -> i32 {
    let base = match capture_free(
        300,
        "With the Touches fixed to the Index controller (the jig): hold the jig completely still in 12 or more\n\
         clearly different orientations, about 3 s each (point it at each base station and Rift sensor, roll it,\n\
         tilt it, upside down, near and far), moving gently between holds. Press Enter when done.",
    ) {
        Ok(b) => b,
        Err(code) => return code,
    };
    let code = register_holds(&base, true, None);
    if code != 0 {
        return code;
    }
    send("alignment reload")
}

/// Records the daemon and every Lighthouse device until Enter, Ctrl+C or `max_s`; the capture's
/// base path, or the exit code.
fn capture_free(max_s: u64, instructions: &str) -> Result<PathBuf, i32> {
    capture_free_with(max_s, instructions, false)
}

/// As `capture_free`; with `frames` the recording keeps the sensors' pixels too.
fn capture_free_with(max_s: u64, instructions: &str, frames: bool) -> Result<PathBuf, i32> {
    capture_free_named(max_s, instructions, frames, None)
}

fn guided_record_start_with(
    cancelled: impl FnOnce() -> bool,
    request: impl FnOnce() -> Result<String, String>,
    reconcile: impl FnOnce() -> Result<(), String>,
) -> Result<(), String> {
    if cancelled() {
        return Err(
            "capture interrupted before record-start dispatch; no recording started".into(),
        );
    }
    let response = request();
    if response.as_ref().is_ok_and(|reply| reply.trim() == "ok") {
        return Ok(());
    }
    if let Ok(reply) = &response {
        if reply.starts_with("err ") {
            return Err(reply.clone()); // An explicit rejection did not start our recorder.
        }
    }
    let error = response
        .err()
        .unwrap_or_else(|| "missing or malformed record-start acknowledgement".into());
    reconcile().map_err(|cleanup| format!("{error}; scoped recorder cleanup failed: {cleanup}"))?;
    Err(format!(
        "{error}; own recording safely stopped if it started"
    ))
}

struct ScopedRecordingGuard {
    command: String,
    armed: bool,
}
impl Drop for ScopedRecordingGuard {
    fn drop(&mut self) {
        if self.armed {
            let _ = control::request(
                &control::socket_path(),
                &self.command,
                Duration::from_millis(750),
            );
        }
    }
}

fn scoped_cleanup_reply(reply: &str) -> bool {
    reply.trim() == "ok not-matching" || reply.starts_with("ok stopped ")
}

fn guided_start_recording(rec: &Path, frames: bool) -> Result<ScopedRecordingGuard, String> {
    let capability = control::request(
        &control::socket_path(),
        "status",
        Duration::from_millis(750),
    )
    .ok()
    .and_then(|reply| {
        reply
            .strip_prefix("ok ")
            .and_then(|json| serde_json::from_str::<serde_json::Value>(json).ok())
    });
    if capability.as_ref().is_none_or(|status| {
        status["recording_scoped_stop"] != true || status["recording"] != false
    }) {
        return Err("guided capture requires an idle matching daemon with scoped recording cleanup; no recording started".into());
    }
    let command = control::record_stop_if_line(rec)?;
    let start_command = control::record_start_line(rec, frames)?;
    let mut scoped = ScopedRecordingGuard {
        command,
        armed: true,
    };
    let command = start_command;
    let rejected = std::cell::Cell::new(false);
    let cleaned = std::cell::Cell::new(false);
    let result = guided_record_start_with(
        || {
            let cancelled = interrupted();
            if cancelled {
                rejected.set(true);
            }
            cancelled
        },
        || {
            let reply = control::request(
                &control::socket_path(),
                &command,
                Duration::from_millis(750),
            )
            .map_err(|error| error.to_string());
            rejected.set(reply.as_ref().is_ok_and(|reply| reply.starts_with("err ")));
            reply
        },
        || {
            let reply = control::request(
                &control::socket_path(),
                &scoped.command,
                Duration::from_millis(750),
            )
            .map_err(|error| error.to_string())?;
            if !scoped_cleanup_reply(&reply) {
                return Err(reply);
            }
            cleaned.set(true);
            Ok(())
        },
    );
    if let Err(error) = result {
        // Explicit rejections must never stop a pre-existing recording. Other failures
        // already attempted path-scoped cleanup; Drop retries only our path if necessary.
        if rejected.get() || cleaned.get() {
            scoped.armed = false;
        }
        return Err(error);
    }
    Ok(scoped)
}

/// The headset's orientation (SteamVR) and each Touch's (cv1touchd) at one instant.
struct PoseFrame {
    t_ns: u64,
    hmd: nalgebra::UnitQuaternion<f64>,
    touch: [Option<nalgebra::UnitQuaternion<f64>>; 2],
}

/// A guided capture's hooks. An error from one ends the capture with Err(1); the hook reports
/// it in its own words.
trait CaptureFeedback {
    fn before_start(&mut self) -> Result<(), String> {
        Ok(())
    }
    /// Every pose frame, in order, before the `poll` that follows it.
    fn pose(&mut self, _frame: &PoseFrame) {}
    fn started(&mut self) -> Result<(), String>;
    fn poll(&mut self) -> Result<bool, String>;
    fn stopped(&mut self) -> Result<(), String>;
}

fn capture_free_named(
    max_s: u64,
    instructions: &str,
    frames: bool,
    base: Option<&Path>,
) -> Result<PathBuf, i32> {
    capture_free_named_with_feedback(max_s, instructions, frames, base, None)
}

fn capture_free_named_with_feedback(
    max_s: u64,
    instructions: &str,
    frames: bool,
    base: Option<&Path>,
    mut feedback: Option<&mut dyn CaptureFeedback>,
) -> Result<PathBuf, i32> {
    catch_interrupts();
    let status = status_json();
    if status.as_ref().and_then(|v| v["mode"].as_str()) != Some("calibrated") {
        eprintln!("cv1touch: cv1touchd must be running in calibrated mode");
        return Err(1);
    }
    let latency_adjustment_ms = latency_adjustment(status.as_ref());
    let client = match vr::Client::connect() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return Err(1);
        }
    };
    let universe = client
        .devices()
        .iter()
        .find_map(|d| {
            client
                .uint64_property(d.index, vr::PROP_CURRENT_UNIVERSE_ID)
                .filter(|&u| u != 0)
        })
        .unwrap_or(0);
    let base = base
        .map(Path::to_path_buf)
        .unwrap_or_else(|| state_dir().join(format!("eval-{}", unix_now())));
    let rec = base.with_extension("rec");
    if let Err(e) = std::fs::create_dir_all(state_dir()) {
        eprintln!("cv1touch: {e}");
        return Err(1);
    }
    // the Touches as cv1touchd tracks them, for the feedback: SteamVR's copy may be paused
    let (reader, frames_out, frames_in) = if feedback.is_some() {
        let reader = touch_reader().map_err(|error| {
            eprintln!("cv1touch: {error}; no recording started");
            1
        })?;
        let (tx, rx) = std::sync::mpsc::channel::<PoseFrame>();
        (Some(reader), Some(tx), Some(rx))
    } else {
        (None, None, None)
    };
    if let Some(hook) = feedback.as_mut() {
        if hook.before_start().is_err() {
            return Err(1);
        }
        if interrupted() {
            return Err(130);
        }
    }
    let mut scoped_guard = if feedback.is_some() {
        Some(guided_start_recording(&rec, frames).map_err(|error| {
            eprintln!("cv1touch: {error}; no registration fit installed");
            1
        })?)
    } else {
        if send_path(control::record_start_line(&rec, frames)) != 0 {
            return Err(1);
        }
        None
    };
    let mut guard = StopRecordingGuard(scoped_guard.is_none());
    let start = shm::now_ns();
    let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let sampler = {
        let stop = stop.clone();
        std::thread::spawn(move || {
            let mut tracks: BTreeMap<String, Vec<[f64; 8]>> = BTreeMap::new();
            let mut kinds: BTreeMap<String, String> = BTreeMap::new();
            let mut serials: Vec<Option<String>> = Vec::new();
            let mut hmd_index = None;
            let mut next_scan = 0u64;
            while !stop.load(std::sync::atomic::Ordering::Relaxed) {
                let now = shm::now_ns();
                if now >= next_scan {
                    let devs = client.devices();
                    serials = vec![None; 64];
                    hmd_index = devs
                        .iter()
                        .find(|d| d.class == vr::CLASS_HMD)
                        .map(|d| d.index as usize);
                    // every Lighthouse device (controllers, the headset, the base stations), and our
                    // Touches as SteamVR shows them
                    for d in devs.iter().filter(|d| {
                        d.tracking_system == "lighthouse"
                            || d.class == vr::CLASS_HMD
                            || d.serial.starts_with("CV1T-")
                    }) {
                        if let Some(slot) = serials.get_mut(d.index as usize) {
                            *slot = Some(d.serial.clone());
                        }
                        let kind = match d.class {
                            _ if d.serial.starts_with("CV1T-") => "touch",
                            vr::CLASS_CONTROLLER => "controller",
                            vr::CLASS_HMD => "hmd",
                            vr::CLASS_TRACKING_REFERENCE => "base",
                            _ => "other",
                        };
                        kinds.insert(d.serial.clone(), kind.to_string());
                    }
                    next_scan = now + 500_000_000;
                }
                let raw = client.raw_poses();
                for (i, p) in raw.iter().enumerate() {
                    if let (Some(p), Some(Some(serial))) = (p, serials.get(i)) {
                        tracks
                            .entry(serial.clone())
                            .or_default()
                            .push(eval::truth_row(now, p));
                    }
                }
                if let (Some(out), Some(reader), Some(Some(hmd))) =
                    (&frames_out, &reader, hmd_index.and_then(|i| raw.get(i)))
                {
                    let touch = [HAND_LEFT, HAND_RIGHT].map(|hand| {
                        reader
                            .hand(hand)
                            .filter(|st| st.flags & cv1touch::hand::FLAG_CONNECTED != 0)
                            .map(|st| register::extrapolate(&st, now).rotation)
                    });
                    let _ = out.send(PoseFrame {
                        t_ns: now,
                        hmd: hmd.rotation,
                        touch,
                    });
                }
                std::thread::sleep(Duration::from_millis(2));
            }
            (tracks, kinds)
        })
    };
    // the guided flow (with feedback) shows its own short instructions and keeps paths out of view
    let chatty = feedback.is_none();
    // an Enter typed before the recording started must not end it
    discard_typed_ahead();
    if chatty {
        println!(
            "recording {} (stops on Enter or Ctrl+C, keeping the data, or after {max_s} s)",
            rec.display()
        );
    }
    human_action(instructions);
    let mut feedback_error = feedback.as_mut().and_then(|hook| hook.started().err());
    let deadline = start + max_s * 1_000_000_000;
    let mut last_print = 0u64;
    loop {
        if interrupted() || feedback_error.is_some() {
            break;
        }
        if let Some(hook) = feedback.as_mut() {
            for frame in frames_in.iter().flat_map(|rx| rx.try_iter()) {
                hook.pose(&frame);
            }
            match hook.poll() {
                Ok(true) => break,
                Ok(false) => {}
                Err(error) => {
                    feedback_error = Some(error);
                    break;
                }
            }
        }
        let wait = Duration::from_millis(if feedback.is_some() { 20 } else { 200 });
        if console::next_line(wait) != Line::NotYet {
            break;
        }
        let now = shm::now_ns();
        if now >= deadline {
            break;
        }
        if now - last_print >= 1_000_000_000 {
            last_print = now;
            let _ = write!(io::stdout(), "\r  {:5.1} s   ", (now - start) as f64 * 1e-9);
            let _ = io::stdout().flush();
        }
    }
    let end = shm::now_ns();
    stop.store(true, std::sync::atomic::Ordering::Relaxed);
    let (tracks, kinds) = sampler.join().unwrap_or_default();
    // Ctrl+C reaches the whole pipeline, `tee` included: stop the recording and keep the data
    // before printing anything, and print without panicking on a closed pipe
    let stop_command = scoped_guard
        .as_ref()
        .map_or("record stop", |guard| guard.command.as_str());
    let stopped = control::request(
        &control::socket_path(),
        stop_command,
        Duration::from_secs(5),
    );
    let stopped_ok = stopped.as_ref().is_ok_and(|reply| {
        if scoped_guard.is_some() {
            reply.starts_with("ok stopped ")
        } else {
            reply.starts_with("ok")
        }
    });
    if stopped_ok {
        guard.0 = false;
        if let Some(scoped) = scoped_guard.as_mut() {
            scoped.armed = false;
        }
        if let Some(hook) = feedback.as_mut() {
            if let Err(error) = hook.stopped() {
                feedback_error.get_or_insert(error);
            }
        }
    }
    let kind = |serial: &str| kinds.get(serial).cloned().unwrap_or_default();
    let mut out = io::stdout();
    match &stopped {
        Ok(reply) if chatty => {
            let _ = writeln!(out, "\n{reply}");
        }
        Ok(_) => {
            let _ = writeln!(out);
        }
        Err(e) => {
            let _ = writeln!(io::stderr(), "\ncv1touch: record stop failed: {e}");
        }
    }
    let Some(((reference, samples), rest)) = reference_first(tracks.into_iter().collect(), kind)
    else {
        let _ = writeln!(
            io::stderr(),
            "cv1touch: no reference controller or headset was tracked (recording kept at {})",
            rec.display()
        );
        return Err(1);
    };
    if chatty {
        let _ = writeln!(
            out,
            "  {reference} ({}, the reference): {} poses",
            kind(&reference),
            samples.len()
        );
        for (serial, rows) in &rest {
            let _ = writeln!(out, "  {serial} ({}): {} poses", kind(serial), rows.len());
        }
    }
    let others = rest
        .iter()
        .map(|(serial, samples)| eval::RefTrack {
            serial: serial.clone(),
            samples: samples.clone(),
            kind: kind(serial),
        })
        .collect();
    let file = eval::TruthFile {
        format: eval::TRUTH_FORMAT.into(),
        version: 1,
        reference,
        universe_id: universe,
        windows: vec![("free".into(), start, end)],
        samples,
        others,
        latency_adjustment_ms,
    };
    let truth_path = base.with_extension("truth.json");
    if let Err(e) = std::fs::write(
        &truth_path,
        serde_json::to_string(&file).unwrap_or_default(),
    ) {
        let _ = writeln!(io::stderr(), "cv1touch: {}: {e}", truth_path.display());
        return Err(1);
    }
    if chatty {
        let _ = writeln!(
            out,
            "recording {}\ntruth {}",
            rec.display(),
            truth_path.display()
        );
    }
    if feedback_error.is_some() {
        return Err(1);
    }
    if !stopped_ok {
        return Err(1);
    }
    Ok(base)
}

type PoseTrack = (String, Vec<[f64; 8]>);

/// A capture's reference and the other tracks: a Lighthouse controller when one was tracked (the
/// one with the most poses), else the headset; never our own Touches, which SteamVR shows too.
/// The others in order of poses. None with neither.
fn reference_first(
    mut tracks: Vec<PoseTrack>,
    kind: impl Fn(&str) -> String,
) -> Option<(PoseTrack, Vec<PoseTrack>)> {
    tracks.sort_by_key(|t| std::cmp::Reverse(t.1.len()));
    let i = tracks
        .iter()
        .position(|t| kind(&t.0) == "controller")
        .or_else(|| tracks.iter().position(|t| kind(&t.0) == "hmd"))?;
    let reference = tracks.remove(i);
    Some((reference, tracks))
}

/// One device of a rig capture: its poses in the world frame.
struct RigDevice {
    name: String,
    /// The raw Lighthouse poses, for Index controllers (to fit a registration against a Touch).
    lighthouse: Option<Vec<eval::TruthSample>>,
    poses: Vec<eval::Published>,
}

/// `b` against `a` held rigidly to it: lag (b behind a), errors with slips segmented out.
fn rig_pair(a: &RigDevice, b: &RigDevice) {
    let truth: Vec<eval::TruthSample> = a
        .poses
        .iter()
        .map(|p| eval::TruthSample {
            t_ns: p.t_ns,
            pose: p.pose,
        })
        .collect();
    let (first, last) = (
        truth.first().map_or(0, |t| t.t_ns),
        truth.last().map_or(0, |t| t.t_ns),
    );
    let pubs: Vec<eval::Published> = b
        .poses
        .iter()
        .filter(|p| p.t_ns > first && p.t_ns < last)
        .copied()
        .collect();
    if pubs.len() < 1000 {
        println!(
            "  {} vs {}: overlap too short ({} poses)",
            b.name,
            a.name,
            pubs.len()
        );
        return;
    }
    let corr = eval::pick_hand([&[], &pubs], &truth).map_or(f64::NAN, |x| x.1);
    let params = eval::SegmentParams::default();
    let id = nalgebra::Isometry3::identity();
    let tr0 = eval::Truth {
        samples: &truth,
        world_from_lh: id,
        segments: eval::offset_segments(&pubs, &truth, &id, 0, &params),
    };
    let (lag, _) = eval::lag_ms(&pubs, &tr0, (-50, 50));
    let lag_ns = (lag * 1e6) as i64;
    let segs = eval::offset_segments(&pubs, &truth, &id, lag_ns, &params);
    let covered: u64 = segs.iter().map(|s| s.end_ns - s.start_ns).sum();
    let nseg = segs.len();
    let tr = eval::Truth {
        samples: &truth,
        world_from_lh: id,
        segments: segs,
    };
    let (e0, el) = (
        eval::errors(&pubs, &tr, 0),
        eval::errors(&pubs, &tr, lag_ns),
    );
    println!(
        "  {:>20} vs {:<20} corr {:.3} | lag {:+5.1} ms | as is: {:4.1}/{:4.1}/{:5.1} mm {:.2}/{:.2} deg | lag removed: {:4.1}/{:4.1}/{:5.1} mm {:.2} deg | {} poses, {} segment(s) over {:.0} s",
        b.name,
        a.name,
        corr,
        lag,
        e0.pos_median_mm,
        e0.pos_p90_mm,
        e0.pos_p99_mm,
        e0.rot_median_deg,
        e0.rot_p90_deg,
        el.pos_median_mm,
        el.pos_p90_mm,
        el.pos_p99_mm,
        el.rot_median_deg,
        e0.n,
        nseg,
        covered as f64 * 1e-9
    );
}

/// The pose of a series averaged over [a, b]: median translation, chordal-mean rotation.
fn mean_pose(poses: &[eval::Published], a: u64, b: u64) -> Option<nalgebra::Isometry3<f64>> {
    let set: Vec<&eval::Published> = poses
        .iter()
        .filter(|p| p.t_ns >= a && p.t_ns <= b)
        .collect();
    if set.len() < 20 {
        return None;
    }
    let med = |k: usize| {
        let mut v: Vec<f64> = set.iter().map(|p| p.pose.translation.vector[k]).collect();
        v.sort_by(f64::total_cmp);
        v[v.len() / 2]
    };
    let r = calib::motion::chordal_mean(&set.iter().map(|p| p.pose.rotation).collect::<Vec<_>>());
    Some(nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::new(med(0), med(1), med(2)),
        r,
    ))
}

/// Holds: spans of at least `min_ns` in which `poses` moves less than 2 mm and 0.3 degrees from
/// the span's start; each is trimmed by a quarter second at both ends.
fn find_holds(poses: &[eval::Published], min_ns: u64) -> Vec<(u64, u64)> {
    let mut holds = Vec::new();
    let mut i = 0;
    while i < poses.len() {
        let start = poses[i];
        let mut j = i;
        while j + 1 < poses.len()
            && (poses[j + 1].pose.translation.vector - start.pose.translation.vector).norm() < 0.002
            && poses[j + 1].pose.rotation.angle_to(&start.pose.rotation) < 0.3f64.to_radians()
        {
            j += 1;
        }
        if poses[j].t_ns - start.t_ns >= min_ns {
            holds.push((start.t_ns + 250_000_000, poses[j].t_ns - 250_000_000));
            i = j + 1;
        } else {
            i += 1;
        }
    }
    holds
}

/// Static holds of a rig (`eval-capture --free`, the jig held still in many orientations): each
/// pair's relative pose per hold against one rigid offset fitted to all holds, and each device's
/// share of the scatter (three-cornered hat: pairwise variances add).
fn eval_holds(base: &Path) -> i32 {
    let run = || -> Result<(), String> {
        let x = align::load_registration(&align::registration_path())?
            .lighthouse_from_world
            .iso();
        println!("== with the installed registration");
        let fits = holds_report(base, &x, true)?;
        for (name, fit) in &fits {
            let d = fit.inverse() * x;
            println!("registration refit on the holds from {name}: differs from the installed one by {:.1} mm, {:.2} deg", d.translation.vector.norm() * 1e3, d.rotation.angle().to_degrees());
        }
        if let [(_, a), (_, b)] = fits.as_slice() {
            let d = a.inverse() * b;
            println!(
                "the two refits differ by {:.1} mm, {:.2} deg",
                d.translation.vector.norm() * 1e3,
                d.rotation.angle().to_degrees()
            );
            let mid = nalgebra::Isometry3::from_parts(
                nalgebra::Translation3::from((a.translation.vector + b.translation.vector) * 0.5),
                a.rotation.slerp(&b.rotation, 0.5),
            );
            println!("== with the mean of the two refits");
            holds_report(base, &mid, false)?;
        }
        Ok(())
    };
    match run() {
        Ok(()) => 0,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

/// The holds analysis under registration `x`; returns, per Touch, the registration fitted to the
/// hold means of the first Index and that Touch.
fn holds_report(
    base: &Path,
    x: &nalgebra::Isometry3<f64>,
    list: bool,
) -> Result<Vec<(String, nalgebra::Isometry3<f64>)>, String> {
    {
        let x = *x;
        let truth_path = base.with_extension("truth.json");
        let tf: eval::TruthFile = serde_json::from_str(
            &std::fs::read_to_string(&truth_path)
                .map_err(|e| format!("{}: {e}", truth_path.display()))?,
        )
        .map_err(|e| e.to_string())?;
        let mut devices: Vec<RigDevice> = Vec::new();
        for (serial, rows) in std::iter::once((tf.reference.clone(), tf.samples.clone())).chain(
            tf.others
                .iter()
                .filter(|o| o.kind != "base" && o.kind != "touch")
                .map(|o| (o.serial.clone(), o.samples.clone())),
        ) {
            let ts = eval::truth_samples(&rows);
            let poses = ts
                .iter()
                .map(|s| eval::Published {
                    t_ns: s.t_ns,
                    pose: x.inverse() * s.pose,
                    vel: nalgebra::Vector3::zeros(),
                    ang_vel_world: nalgebra::Vector3::zeros(),
                })
                .collect();
            let kind = if tf
                .others
                .iter()
                .any(|o| o.serial == serial && o.kind == "hmd")
            {
                "HMD"
            } else {
                "Index"
            };
            devices.push(RigDevice {
                name: format!("{kind} {serial}"),
                lighthouse: Some(ts),
                poses,
            });
        }
        for (h, poses) in engine_published(&base.with_extension("rec"))?
            .into_iter()
            .enumerate()
        {
            if poses.len() > 1000 {
                devices.push(RigDevice {
                    name: format!("Touch {}", ["left", "right"][h]),
                    lighthouse: None,
                    poses,
                });
            }
        }
        let holds = find_holds(&devices[0].poses, 1_500_000_000);
        let t0 = devices[0].poses.first().map_or(0, |p| p.t_ns);
        println!(
            "{} holds of the jig (from {})",
            holds.len(),
            devices[0].name
        );
        // per hold, each device's mean pose
        let means: Vec<Vec<Option<nalgebra::Isometry3<f64>>>> = holds
            .iter()
            .map(|&(a, b)| devices.iter().map(|d| mean_pose(&d.poses, a, b)).collect())
            .collect();
        if list {
            for (k, &(a, b)) in holds.iter().enumerate() {
                let have: Vec<&str> = devices
                    .iter()
                    .zip(&means[k])
                    .filter(|(_, m)| m.is_some())
                    .map(|(d, _)| d.name.as_str())
                    .collect();
                println!(
                    "  hold {k:2}: {:6.1}-{:6.1} s, {}",
                    (a - t0) as f64 * 1e-9,
                    (b - t0) as f64 * 1e-9,
                    have.join(", ")
                );
            }
        }
        // registrations fitted to the holds: the first Index against each Touch
        let mut fits = Vec::new();
        if let Some(lh) = devices[0].lighthouse.as_ref() {
            for (j, d) in devices
                .iter()
                .enumerate()
                .filter(|(_, d)| d.lighthouse.is_none())
            {
                let pairs: Vec<_> = holds
                    .iter()
                    .zip(&means)
                    .filter_map(|(&(a, b), m)| {
                        let raw: Vec<eval::Published> = lh
                            .iter()
                            .map(|s| eval::Published {
                                t_ns: s.t_ns,
                                pose: s.pose,
                                vel: nalgebra::Vector3::zeros(),
                                ang_vel_world: nalgebra::Vector3::zeros(),
                            })
                            .collect();
                        Some((mean_pose(&raw, a, b)?, m[j]?))
                    })
                    .collect();
                if let Ok(sol) = register::solve(
                    &pairs,
                    &register::Options {
                        min_pairs: 12,
                        ..register::Options::default()
                    },
                ) {
                    fits.push((d.name.clone(), sol.lighthouse_from_world));
                }
            }
        }
        let n = devices.len();
        // pairwise scatter about the fitted offset
        let mut var_t = vec![vec![f64::NAN; n]; n];
        let mut var_r = vec![vec![f64::NAN; n]; n];
        println!("pair scatter over holds about one fitted offset (rms, max):");
        for i in 0..n {
            for j in i + 1..n {
                let ys: Vec<nalgebra::Isometry3<f64>> = means
                    .iter()
                    .filter_map(|m| Some(m[i]?.inverse() * m[j]?))
                    .collect();
                if ys.len() < 4 {
                    continue;
                }
                let r =
                    calib::motion::chordal_mean(&ys.iter().map(|y| y.rotation).collect::<Vec<_>>());
                let tmean = ys
                    .iter()
                    .map(|y| y.translation.vector)
                    .sum::<nalgebra::Vector3<f64>>()
                    / ys.len() as f64;
                let dt: Vec<f64> = ys
                    .iter()
                    .map(|y| (y.translation.vector - tmean).norm() * 1e3)
                    .collect();
                let dr: Vec<f64> = ys
                    .iter()
                    .map(|y| y.rotation.angle_to(&r).to_degrees())
                    .collect();
                let rms =
                    |v: &[f64]| (v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt();
                var_t[i][j] = rms(&dt).powi(2);
                var_r[i][j] = rms(&dr).powi(2);
                println!(
                    "  {:>20} - {:<20} {:2} holds: {:5.2} mm rms (max {:5.2}), {:5.3} deg rms (max {:5.3})",
                    devices[i].name,
                    devices[j].name,
                    ys.len(),
                    rms(&dt),
                    dt.iter().copied().fold(0.0, f64::max),
                    rms(&dr),
                    dr.iter().copied().fold(0.0, f64::max)
                );
            }
        }
        // three-cornered hat: v_ij = s_i + s_j, least squares over the pairs
        for (label, var, unit) in [("position", &var_t, "mm"), ("rotation", &var_r, "deg")] {
            let pairs: Vec<(usize, usize, f64)> = (0..n)
                .flat_map(|i| (i + 1..n).map(move |j| (i, j)))
                .filter(|&(i, j)| var[i][j].is_finite())
                .map(|(i, j)| (i, j, var[i][j]))
                .collect();
            if pairs.len() < n {
                continue;
            }
            let mut a = nalgebra::DMatrix::<f64>::zeros(pairs.len(), n);
            let mut b = nalgebra::DVector::<f64>::zeros(pairs.len());
            for (k, &(i, j, v)) in pairs.iter().enumerate() {
                a[(k, i)] = 1.0;
                a[(k, j)] = 1.0;
                b[k] = v;
            }
            let Some(sol) = (a.transpose() * &a)
                .try_inverse()
                .map(|m| m * a.transpose() * b)
            else {
                continue;
            };
            let parts: Vec<String> = (0..n)
                .map(|i| format!("{} {:.2}", devices[i].name, sol[i].max(0.0).sqrt()))
                .collect();
            println!(
                "each device's own {label} scatter ({unit} rms): {}",
                parts.join(", ")
            );
            // with both systems twice over, errors one system shares across its devices (and the
            // registration's) cancel within same-system pairs: one more unknown for cross pairs
            let lighthouse = |i: usize| devices[i].lighthouse.is_some();
            let kinds = (0..n).filter(|&i| lighthouse(i)).count();
            if kinds >= 2 && n - kinds >= 2 && pairs.len() > n {
                let mut a = nalgebra::DMatrix::<f64>::zeros(pairs.len(), n + 1);
                let mut b = nalgebra::DVector::<f64>::zeros(pairs.len());
                for (k, &(i, j, v)) in pairs.iter().enumerate() {
                    a[(k, i)] = 1.0;
                    a[(k, j)] = 1.0;
                    if lighthouse(i) != lighthouse(j) {
                        a[(k, n)] = 1.0;
                    }
                    b[k] = v;
                }
                if let Some(sol) = (a.transpose() * &a)
                    .try_inverse()
                    .map(|m| m * a.transpose() * b)
                {
                    let parts: Vec<String> = (0..n)
                        .map(|i| format!("{} {:.2}", devices[i].name, sol[i].max(0.0).sqrt()))
                        .collect();
                    println!(
                        "  with a shared Lighthouse-vs-CV1 term ({:.2} {unit}): {}",
                        sol[n].max(0.0).sqrt(),
                        parts.join(", ")
                    );
                }
            }
        }
        Ok(fits)
    }
}

/// A rig capture (`eval-capture --free` with several controllers fixed together): every device
/// against every other. Errors are position median/p90/p99 and rotation median/p90.
fn eval_rig(base: &Path, replay: bool) -> i32 {
    let run = || -> Result<(), String> {
        let truth_path = base.with_extension("truth.json");
        let tf: eval::TruthFile = serde_json::from_str(
            &std::fs::read_to_string(&truth_path)
                .map_err(|e| format!("{}: {e}", truth_path.display()))?,
        )
        .map_err(|e| e.to_string())?;
        let reg = align::load_registration(&align::registration_path())?;
        let x = reg.lighthouse_from_world.iso();
        let mut devices = Vec::new();
        let refs: Vec<(String, Vec<[f64; 8]>)> =
            std::iter::once((tf.reference.clone(), tf.samples.clone()))
                .chain(
                    tf.others
                        .iter()
                        .filter(|o| o.kind != "base" && o.kind != "touch")
                        .map(|o| (o.serial.clone(), o.samples.clone())),
                )
                .collect();
        for (serial, rows) in refs {
            let ts = eval::truth_samples(&rows);
            let poses = ts
                .iter()
                .map(|s| eval::Published {
                    t_ns: s.t_ns,
                    pose: x.inverse() * s.pose,
                    vel: nalgebra::Vector3::zeros(),
                    ang_vel_world: nalgebra::Vector3::zeros(),
                })
                .collect();
            let kind = if tf
                .others
                .iter()
                .any(|o| o.serial == serial && o.kind == "hmd")
            {
                "HMD"
            } else {
                "Index"
            };
            devices.push(RigDevice {
                name: format!("{kind} {serial}"),
                lighthouse: Some(ts),
                poses,
            });
        }
        let hands = if replay {
            let cal = file::load(&file::default_path())?;
            let dir = track::factory::openhmd_config_dir();
            track::replay::replay(
                &base.with_extension("rec"),
                &cal,
                &|l| track::factory::imu_position(&dir, l),
                &track::replay::Options::default(),
            )?
            .hands
        } else {
            engine_published(&base.with_extension("rec"))?
        };
        for (h, poses) in hands.into_iter().enumerate() {
            if poses.len() > 1000 {
                devices.push(RigDevice {
                    name: format!("Touch {}", ["left", "right"][h]),
                    lighthouse: None,
                    poses,
                });
            }
        }
        for d in &devices {
            let span = d.poses.last().map_or(0, |p| p.t_ns) - d.poses.first().map_or(0, |p| p.t_ns);
            println!(
                "{}: {} poses over {:.1} s",
                d.name,
                d.poses.len(),
                span as f64 * 1e-9
            );
        }
        println!("with the installed registration (b vs a: b's pose against a's, the rig offset fitted):");
        for i in 0..devices.len() {
            for j in i + 1..devices.len() {
                rig_pair(&devices[i], &devices[j]);
            }
        }
        println!("Index vs Touch with a registration fitted to that pair (tracking alone):");
        for a in devices.iter().filter(|d| d.lighthouse.is_some()) {
            for b in devices.iter().filter(|d| d.lighthouse.is_none()) {
                let lh = a.lighthouse.as_ref().unwrap();
                let pairs: Vec<_> = b
                    .poses
                    .iter()
                    .filter_map(|p| Some((eval::interpolate(lh, p.t_ns)?, p.pose)))
                    .collect();
                match register::solve(
                    &pairs,
                    &register::Options {
                        min_pairs: 500,
                        ..register::Options::default()
                    },
                ) {
                    Ok(sol) => {
                        let d = sol.lighthouse_from_world.inverse() * x;
                        let fitted = RigDevice {
                            name: a.name.clone(),
                            lighthouse: None,
                            poses: lh
                                .iter()
                                .map(|s| eval::Published {
                                    t_ns: s.t_ns,
                                    pose: sol.lighthouse_from_world.inverse() * s.pose,
                                    vel: nalgebra::Vector3::zeros(),
                                    ang_vel_world: nalgebra::Vector3::zeros(),
                                })
                                .collect(),
                        };
                        print!(
                            "  (registration differs by {:.1} mm, {:.2} deg)",
                            d.translation.vector.norm() * 1e3,
                            d.rotation.angle().to_degrees()
                        );
                        println!();
                        rig_pair(&fitted, b);
                    }
                    Err(e) => println!("  {} vs {}: no fit ({e})", b.name, a.name),
                }
            }
        }
        Ok(())
    };
    match run() {
        Ok(()) => 0,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

/// Accuracy and latency of a capture (BASE.rec + BASE.truth.json) against its Lighthouse truth.
/// The engine's published poses per hand, from a recording's CONTROLLER_STATE records.
fn engine_published(rec: &Path) -> Result<[Vec<eval::Published>; 2], String> {
    let mut hands: [Vec<eval::Published>; 2] = [Vec::new(), Vec::new()];
    for rec in Reader::open(rec)? {
        if rec.rtype != recorder::CONTROLLER_STATE {
            continue;
        }
        let Some((h, st)) = recorder::decode_state(&rec.payload) else {
            continue;
        };
        // what SteamVR takes as a tracked pose: connected, with a position
        let tracked = cv1touch::ffi::CV1T_FLAG_CONNECTED | cv1touch::ffi::CV1T_FLAG_POSITION;
        if h > 1
            || st.flags & tracked != tracked
            || hands[h].last().is_some_and(|p| p.t_ns >= st.sample_host_ns)
        {
            continue;
        }
        hands[h].push(eval::published_from_state(&st));
    }
    Ok(hands)
}

/// Both Touches as the tracker follows them now: the recording replayed under the installed
/// calibration. A registration has to come from these, not from what the daemon published at the
/// time, which may be in a world frame the calibration has since turned away from.
fn replayed_now(base: &Path) -> Result<[Vec<eval::Published>; 2], String> {
    let cal = file::load(&file::default_path())?;
    let dir = track::factory::openhmd_config_dir();
    Ok(track::replay::replay(
        &base.with_extension("rec"),
        &cal,
        &|l| track::factory::imu_position(&dir, l),
        &track::replay::Options::default(),
    )?
    .hands)
}

fn eval_cmd(base: &Path) -> i32 {
    match engine_published(&base.with_extension("rec")) {
        Ok(hands) => eval_report(base, &hands),
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

/// Parses `cv1touch track` options into replay options: --offset-ms X and --set key=value for
/// tracker parameters.
fn track_options(args: &[&str]) -> Result<track::replay::Options, String> {
    let mut o = track::replay::Options::default();
    let mut it = args.iter();
    while let Some(flag) = it.next() {
        let val = it.next().ok_or_else(|| format!("{flag} needs a value"))?;
        match *flag {
            "--offset-ms" => {
                o.exposure_offset_ns = (val
                    .parse::<f64>()
                    .map_err(|e| format!("--offset-ms: {e}"))?
                    * 1e6) as i64
            }
            "--set" => {
                let (k, v) = val.split_once('=').ok_or("--set takes key=value")?;
                let v: f64 = v.parse().map_err(|e| format!("--set {k}: {e}"))?;
                let p = &mut o.params;
                match k {
                    "gyro" => p.noise.gyro = v,
                    "accel" => p.noise.accel = v,
                    "gyro_bias" => p.noise.gyro_bias = v,
                    "accel_bias" => p.noise.accel_bias = v,
                    "accel_per_accel" => p.noise.accel_per_accel = v,
                    "accel_per_rate" => p.noise.accel_per_rate = v,
                    "sigma_px" => p.fit.update.sigma_px = v,
                    "huber_k" => p.fit.update.huber_k = v,
                    "gate" => p.fit.assoc.gate = v,
                    "max_px" => p.fit.assoc.max_px = v,
                    "merge_px" => p.fit.assoc.merge_px = v,
                    "margin" => p.fit.assoc.margin = v,
                    "min_facing" => p.fit.assoc.min_facing = v,
                    "min_leds" => p.fit.min_leds = v as usize,
                    "inlier_px" => p.fit.inlier_px = v,
                    "max_correction" => p.fit.max_correction = v,
                    "unexplained_ratio" => p.fit.unexplained_ratio = v,
                    "lost_ms" => p.lost_after_ns = (v * 1e6) as u64,
                    _ => return Err(format!("unknown tracker parameter {k}")),
                }
            }
            _ => return Err(format!("unknown option {flag}")),
        }
    }
    Ok(o)
}

type UnlabelledSlices = BTreeMap<String, Vec<(usize, Vec<nalgebra::Vector2<f64>>)>>;

/// Each sensor's unlabelled blobs in ten consecutive time slices of a recording, with frame counts.
fn unlabelled_slices(rec: &Path) -> Result<UnlabelledSlices, String> {
    let mut frames: Vec<(String, u64, Vec<nalgebra::Vector2<f64>>)> = Vec::new();
    for r in Reader::open(rec)? {
        if r.rtype != recorder::FRAME_OBS {
            continue;
        }
        let Some((info, bl)) = recorder::decode_frame_obs(&r.payload) else {
            continue;
        };
        frames.push((
            cv1touch::engine::cstr_field(&info.serial),
            info.frame_start_host_ns,
            bl.iter()
                .filter(|b| b.led_id < 0)
                .map(|b| nalgebra::Vector2::new(b.x as f64, b.y as f64))
                .collect(),
        ));
    }
    let (t0, t1) = (
        frames.iter().map(|f| f.1).min().unwrap_or(0),
        frames.iter().map(|f| f.1).max().unwrap_or(0),
    );
    let span = (t1 - t0).max(1);
    let mut out = UnlabelledSlices::new();
    for (serial, t, blobs) in frames {
        let slots = out
            .entry(serial)
            .or_insert_with(|| vec![(0, Vec::new()); 10]);
        let k = (((t - t0) as u128 * 10 / span as u128) as usize).min(9);
        slots[k].0 += 1;
        slots[k].1.extend(blobs);
    }
    Ok(out)
}

/// Landmarks are kept only where a Lighthouse base station projects (other emitters in a room
/// get moved); the stations' poses come from SteamVR.
fn landmarks_learn(rec: &Path, write: bool) -> i32 {
    let run = || -> Result<(), String> {
        let client = vr::Client::connect()?;
        let poses = client.raw_poses();
        let stations: Vec<nalgebra::Isometry3<f64>> = client
            .devices()
            .into_iter()
            .filter(|d| {
                d.class == vr::CLASS_TRACKING_REFERENCE && d.tracking_system == "lighthouse"
            })
            .filter_map(|d| poses.get(d.index as usize).copied().flatten())
            .collect();
        drop(client);
        let cal = file::load(&file::default_path())?;
        let x = align::load_registration(&align::registration_path())?
            .lighthouse_from_world
            .iso();
        let (models, _) = track::refine::recording_models(rec)?;
        let near_station = |serial: &str, l: &track::landmarks::Landmark| -> bool {
            let (Some(e), Some(m)) = (
                cal.sensors.iter().find(|e| e.serial == serial),
                models.get(serial),
            ) else {
                return false;
            };
            let w = calib::iso_from_xyzw(e.orient_xyzw, e.pos);
            stations.iter().any(|st| {
                let pc = w.inverse() * (x.inverse() * calib::stations::station_point(st, 0.0));
                pc.z > 0.0
                    && m.project(&pc.coords)
                        .is_some_and(|uv| (uv - nalgebra::Vector2::new(l.x, l.y)).norm() < 15.0)
            })
        };
        let mut all = Vec::new();
        for (serial, slices) in unlabelled_slices(rec)? {
            let found: Vec<_> = track::landmarks::learn(&serial, &slices, 0.03)
                .into_iter()
                .filter(|l| near_station(&serial, l))
                .collect();
            for l in &found {
                println!(
                    "{serial}: landmark at ({:.2}, {:.2}), in {:.1}% of frames",
                    l.x,
                    l.y,
                    l.seen * 100.0
                );
            }
            all.extend(found);
        }
        if !write {
            println!("(not written: run with --write)");
            return Ok(());
        }
        let f = track::landmarks::LandmarkFile {
            format: track::landmarks::FORMAT.into(),
            version: 1,
            created_unix_s: unix_now(),
            source: rec.display().to_string(),
            landmarks: all,
        };
        let path = track::landmarks::default_path();
        let b = align::save(&path, &f)?;
        println!("wrote {} (previous: {:?})", path.display(), b);
        Ok(())
    };
    run().map_or_else(
        |e| {
            eprintln!("cv1touch: {e}");
            1
        },
        |()| 0,
    )
}

/// Replays a recording's frames through the landmark monitor: each sensor's shift per window.
fn landmarks_check(rec: &Path) -> i32 {
    let run = || -> Result<(), String> {
        let f = track::landmarks::load(&track::landmarks::default_path())?;
        let mut m = track::landmarks::Monitor::new(f.landmarks, 30_000_000_000);
        for r in Reader::open(rec)? {
            if r.rtype != recorder::FRAME_OBS {
                continue;
            }
            let Some((info, bl)) = recorder::decode_frame_obs(&r.payload) else {
                continue;
            };
            let blobs: Vec<nalgebra::Vector2<f64>> = bl
                .iter()
                .filter(|b| b.led_id < 0)
                .map(|b| nalgebra::Vector2::new(b.x as f64, b.y as f64))
                .collect();
            if let Some(shifts) = m.frame(
                &cv1touch::engine::cstr_field(&info.serial),
                info.frame_start_host_ns,
                &blobs,
                20,
            ) {
                let parts: Vec<String> = shifts
                    .iter()
                    .map(|s| {
                        format!(
                            "{} ({:+.2}, {:+.2}) px over {}",
                            s.serial, s.shift.x, s.shift.y, s.landmarks
                        )
                    })
                    .collect();
                println!("{}", parts.join("; "));
            }
        }
        Ok(())
    };
    run().map_or_else(
        |e| {
            eprintln!("cv1touch: {e}");
            1
        },
        |()| 0,
    )
}

/// Registration from a rig capture's static holds: each Touch's tracked pose against the
/// reference controller's, per hold, fitted (one fit per Touch, their mean taken). The sensors
/// are left as they are.
/// A capture's Lighthouse track by serial, or the headset's with "hmd" (the capture's own
/// reference when None): (serial, samples).
fn truth_track(
    tf: &eval::TruthFile,
    which: Option<&str>,
) -> Result<(String, Vec<eval::TruthSample>), String> {
    match which {
        None => Ok((tf.reference.clone(), tf.truth())),
        Some(w) if w == tf.reference => Ok((tf.reference.clone(), tf.truth())),
        // the headset is always recorded, so when it is not among the others it is the reference
        // (a capture made with no controller on)
        Some("hmd") if !tf.others.iter().any(|o| o.kind == "hmd") => {
            Ok((tf.reference.clone(), tf.truth()))
        }
        Some(w) => tf
            .others
            .iter()
            .find(|o| o.serial == w || (w == "hmd" && o.kind == "hmd"))
            .map(|o| (o.serial.clone(), eval::truth_samples(&o.samples)))
            .ok_or_else(|| format!("the capture has no Lighthouse track {w}")),
    }
}

fn register_holds(base: &Path, write: bool, reference: Option<&str>) -> i32 {
    register_from_capture(base, write, reference, false)
}

/// The mean of some poses' positions (raw, m): a registration's play area.
fn mean_position<'a>(
    poses: impl Iterator<Item = &'a nalgebra::Isometry3<f64>>,
) -> Option<[f64; 3]> {
    let (sum, n) = poses.fold((nalgebra::Vector3::zeros(), 0usize), |(s, n), p| {
        (s + p.translation.vector, n + 1)
    });
    (n > 0).then(|| (sum / n as f64).into())
}

/// Each pair's Touch in the reference's frame under `x`, by the reference's heading in the world
/// (`world_x` turns the reference into the world), for `register::stir_radius`.
fn held_by_heading(
    pairs: &[(nalgebra::Isometry3<f64>, nalgebra::Isometry3<f64>)],
    world_x: &nalgebra::Isometry3<f64>,
    x: &nalgebra::Isometry3<f64>,
) -> Vec<(f64, nalgebra::Vector3<f64>)> {
    pairs
        .iter()
        .map(|(l, c)| {
            let fwd = (world_x.inverse() * l).rotation * -nalgebra::Vector3::z();
            (
                fwd.x.atan2(-fwd.z),
                (l.inverse() * x * c).translation.vector,
            )
        })
        .collect()
}

/// Registration from a capture in which the Touches moved rigidly with the reference (pressed to
/// the headset while the wearer turns, looks up and down and tilts): every 20 ms a pair.
fn initial_registration(universe_id: u64) -> align::Registration {
    align::Registration {
        format: align::REGISTRATION_FORMAT.into(),
        version: align::VERSION,
        created_unix_s: unix_now(),
        lighthouse_from_world: align::PoseEntry::from_iso(&nalgebra::Isometry3::identity()),
        universe_id,
        touch: String::new(),
        reference: String::new(),
        pairs: 0,
        pos_rms_mm: 0.0,
        rot_rms_deg: 0.0,
        stations: Vec::new(),
        play_area: None,
    }
}

fn register_headset() -> i32 {
    let base = match capture_free(150,
        "Headset registration: press each Touch firmly to a side of the headset, away from its tracking surface.\n\
         Turn two slow circles each way, then nod and tilt your head repeatedly. Keep the Touches attached\n\
         for the entire motion; press Enter after releasing them. No reference controller is needed.") {
        Ok(base) => base,
        Err(code) => return code,
    };
    let code = register_follow(&base, true, Some("hmd"));
    if code == 0 {
        send("alignment reload")
    } else {
        code
    }
}

fn register_follow(base: &Path, write: bool, reference: Option<&str>) -> i32 {
    register_from_capture(base, write, reference, true)
}

fn register_from_capture(base: &Path, write: bool, reference: Option<&str>, follow: bool) -> i32 {
    register_from_capture_with(base, write, reference, follow, true)
}

fn registration_seed(
    path: &Path,
    universe_id: u64,
    reuse: bool,
) -> Result<(bool, align::Registration), String> {
    let had_registration = reuse && path.exists();
    let reg = if had_registration {
        align::load_registration(path)?
    } else {
        initial_registration(universe_id)
    };
    Ok((had_registration, reg))
}

fn register_from_capture_with(
    base: &Path,
    write: bool,
    reference: Option<&str>,
    follow: bool,
    reuse: bool,
) -> i32 {
    let run = || -> Result<(), String> {
        let truth_path = base.with_extension("truth.json");
        let tf: eval::TruthFile = serde_json::from_str(
            &std::fs::read_to_string(&truth_path)
                .map_err(|e| format!("{}: {e}", truth_path.display()))?,
        )
        .map_err(|e| e.to_string())?;
        let reg_path = align::registration_path();
        let (had_registration, mut reg) = registration_seed(&reg_path, tf.universe_id, reuse)?;
        let (ref_serial, ref_samples) = truth_track(&tf, reference)?;
        println!("reference {ref_serial}");
        let holds = calib::rig::holds(&ref_samples, 1_500_000_000);
        let published = replayed_now(base)?;
        // the registration as the daemon applied it during the capture: followed through any
        // re-solve of the universe since it was made
        let stations: Vec<(String, nalgebra::Isometry3<f64>)> = tf
            .others
            .iter()
            .filter(|o| o.kind == "base")
            .filter_map(|o| {
                let s = eval::truth_samples(&o.samples);
                Some((o.serial.clone(), s.get(s.len() / 2)?.pose))
            })
            .collect();
        let x0 = match reg.follow(&stations) {
            Some(shift) => {
                let s = shift.now_from_then;
                let play = reg.play_area_point();
                println!("the universe had moved {:.1} mm, {:.2} deg in the play area since the installed registration (stations disagreeing there by {:.1} mm rms)", (s * play - play).norm() * 1e3, s.rotation.angle().to_degrees(), shift.residual_m * 1e3);
                s * reg.lighthouse_from_world.iso()
            }
            None => reg.lighthouse_from_world.iso(),
        };
        let mut fits = Vec::new();
        let mut held_pairs = Vec::new();
        let min_pairs = if follow { 500 } else { 12 };
        // turning in place and nodding: mostly about one axis, but over a far wider range than a
        // hand-held registration needs
        let min_axis_ratio = if follow {
            0.1
        } else {
            register::Options::default().min_axis_ratio
        };
        for (h, pubs) in published.iter().enumerate() {
            let pairs: Vec<_> = if follow {
                // the IMU's 500 Hz outputs, one in ten, over the stretch the Touch stayed on the
                // reference: found under the installed registration, then again under a first fit
                let timed: Vec<(u64, nalgebra::Isometry3<f64>, nalgebra::Isometry3<f64>)> = pubs
                    .iter()
                    .step_by(10)
                    .filter_map(|p| {
                        Some((p.t_ns, eval::interpolate(&ref_samples, p.t_ns)?, p.pose))
                    })
                    .collect();
                let t0 = ref_samples.first().map_or(0, |s| s.t_ns);
                let span = |x: &nalgebra::Isometry3<f64>, o: &register::Rigid| {
                    register::rigid_span(&timed, x, o)
                };
                let first_span = if had_registration {
                    span(&x0, &register::Rigid::default())
                } else {
                    register::bootstrap_span(&timed)
                };
                let Some(first) = first_span else {
                    println!(
                        "Touch {}: never moved with the reference",
                        ["left", "right"][h]
                    );
                    continue;
                };
                let rough = register::solve(
                    &timed[first.clone()]
                        .iter()
                        .map(|p| (p.1, p.2))
                        .collect::<Vec<_>>(),
                    &register::Options {
                        min_pairs,
                        min_axis_ratio,
                        ..register::Options::default()
                    },
                )?;
                // the two streams' stamps lined up: the shift that holds the offset steadiest
                let at = |lag_ns: i64| -> Vec<(u64, nalgebra::Isometry3<f64>, nalgebra::Isometry3<f64>)> {
                    timed[first.clone()].iter().filter_map(|p| Some((p.0, eval::interpolate(&ref_samples, (p.0 as i64 + lag_ns) as u64)?, p.2))).collect()
                };
                let spread = |lag_ns: i64| {
                    let (_, pos, rot) = register::check(
                        &at(lag_ns).iter().map(|p| (p.1, p.2)).collect::<Vec<_>>(),
                        &rough.lighthouse_from_world,
                    );
                    pos.powi(2)
                        + (rot.to_radians() * register::Options::default().rot_weight_m).powi(2)
                };
                let lag_ns = (-15..=15)
                    .map(|k| k as i64 * 2_000_000)
                    .min_by(|a, b| spread(*a).total_cmp(&spread(*b)))
                    .unwrap_or(0);
                let timed: Vec<_> = timed
                    .iter()
                    .filter_map(|p| {
                        Some((
                            p.0,
                            eval::interpolate(&ref_samples, (p.0 as i64 + lag_ns) as u64)?,
                            p.2,
                        ))
                    })
                    .collect();
                let tight = register::Rigid {
                    tol_m: 0.015,
                    tol_rad: 3f64.to_radians(),
                    ..register::Rigid::default()
                };
                let r = register::rigid_span(&timed, &rough.lighthouse_from_world, &tight)
                    .unwrap_or(first);
                let secs = |i: usize| (timed[i].0.saturating_sub(t0)) as f64 * 1e-9;
                println!("Touch {}: on the reference from {:.1} s to {:.1} s; the reference's stamps {} ours by {:.0} ms", ["left", "right"][h], secs(r.start), secs(r.end - 1), if lag_ns >= 0 { "trail" } else { "lead" }, lag_ns.abs() as f64 * 1e-6);
                timed[r].iter().map(|p| (p.1, p.2)).collect()
            } else {
                holds
                    .iter()
                    .filter_map(|&(a, b, l)| Some((l, mean_pose(pubs, a, b)?)))
                    .collect()
            };
            if pairs.len() < min_pairs {
                continue;
            }
            let sol = register::solve(
                &pairs,
                &register::Options {
                    min_pairs,
                    min_axis_ratio,
                    ..register::Options::default()
                },
            )?;
            if sol.inliers * 5 < sol.pairs * 4 || sol.pos_rms_m > 0.010 || sol.rot_rms_deg > 3.0 {
                return Err(format!("Touch {}: registration rejected ({:.1} mm, {:.2} deg, {} of {} inliers); hold firmly and repeat", ["left", "right"][h], sol.pos_rms_m * 1e3, sol.rot_rms_deg, sol.inliers, sol.pairs));
            }
            let d = sol.lighthouse_from_world.inverse() * x0;
            println!(
                "Touch {} ({} {}): {:.1} mm, {:.2} deg rms",
                ["left", "right"][h],
                pairs.len(),
                if follow { "pairs" } else { "holds" },
                sol.pos_rms_m * 1e3,
                sol.rot_rms_deg
            );
            if had_registration {
                println!(
                    "  differs from the installed registration by {:.1} mm, {:.2} deg",
                    d.translation.vector.norm() * 1e3,
                    d.rotation.angle().to_degrees()
                );
            } else {
                println!("  initial registration from this capture");
            }
            if follow && had_registration {
                println!(
                    "  stir (radius of the circle it seems to trace as you turn): {:.1} mm as it was, {:.1} mm with this fit",
                    register::stir_radius(&held_by_heading(&pairs, &x0, &x0)) * 1e3,
                    register::stir_radius(&held_by_heading(&pairs, &x0, &sol.lighthouse_from_world)) * 1e3
                );
            }
            fits.push((
                sol.lighthouse_from_world,
                sol.pos_rms_m,
                sol.rot_rms_deg,
                pairs.len(),
            ));
            held_pairs.push(pairs);
        }
        let Some(first) = fits.first() else {
            return Err(format!(
                "no Touch tracked through enough {}",
                if follow { "of the capture" } else { "holds" }
            ));
        };
        let x = if let [a, b] = fits.as_slice() {
            let d = a.0.inverse() * b.0;
            println!(
                "the two fits differ by {:.1} mm, {:.2} deg",
                d.translation.vector.norm() * 1e3,
                d.rotation.angle().to_degrees()
            );
            if d.translation.vector.norm() > 0.020 || d.rotation.angle().to_degrees() > 2.0 {
                return Err("the two Touch registrations disagree; hold both firmly and repeat (not written)".into());
            }
            if follow {
                // out of sample: each Touch under the other's fit
                println!(
                    "  stir under the other Touch's fit: left {:.1} mm, right {:.1} mm",
                    register::stir_radius(&held_by_heading(&held_pairs[0], &x0, &b.0)) * 1e3,
                    register::stir_radius(&held_by_heading(&held_pairs[1], &x0, &a.0)) * 1e3
                );
            }
            nalgebra::Isometry3::from_parts(
                nalgebra::Translation3::from(
                    (a.0.translation.vector + b.0.translation.vector) * 0.5,
                ),
                a.0.rotation.slerp(&b.0.rotation, 0.5),
            )
        } else {
            first.0
        };
        if !write {
            println!("(not written: run with --write)");
            return Ok(());
        }
        reg.lighthouse_from_world = align::PoseEntry::from_iso(&x);
        reg.created_unix_s = unix_now();
        reg.universe_id = tf.universe_id;
        reg.reference = ref_serial.clone();
        reg.touch = format!(
            "{} of {}",
            if follow {
                "moving with the reference"
            } else {
                "holds"
            },
            base.display()
        );
        // the universe the holds were measured in: the stations as the capture saw them
        reg.play_area = mean_position(held_pairs.iter().flatten().map(|p| &p.0));
        reg.stations = tf
            .others
            .iter()
            .filter(|o| o.kind == "base")
            .filter_map(|o| {
                let s = eval::truth_samples(&o.samples);
                Some(align::StationPose {
                    serial: o.serial.clone(),
                    pose: align::PoseEntry::from_iso(&s.get(s.len() / 2)?.pose),
                })
            })
            .collect();
        reg.pairs = fits.iter().map(|f| f.3).sum();
        reg.pos_rms_mm = fits.iter().map(|f| f.1).sum::<f64>() / fits.len() as f64 * 1e3;
        reg.rot_rms_deg = fits.iter().map(|f| f.2).sum::<f64>() / fits.len() as f64;
        let mut b = None;
        commit_fit(interrupted(), || {
            b = align::save(&reg_path, &reg)?;
            Ok(())
        })?;
        println!(
            "wrote {} (previous: {:?}); run `cv1touch alignment reload`",
            reg_path.display(),
            b
        );
        Ok(())
    };
    match run() {
        Ok(()) => 0,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

/// Registration from the Lighthouse base stations the Rift sensors see (see `calib::stations`):
/// the stations' poses from SteamVR now, their spots from a recording's unlabelled blobs.
fn register_stations(rec: &Path, write: bool, out: Option<&Path>) -> i32 {
    use cv1touch::calib::stations;
    let run = || -> Result<(), String> {
        let client = vr::Client::connect()?;
        let poses = client.raw_poses();
        let stations: Vec<(String, nalgebra::Isometry3<f64>)> = client
            .devices()
            .into_iter()
            .filter(|d| {
                d.class == vr::CLASS_TRACKING_REFERENCE && d.tracking_system == "lighthouse"
            })
            .filter_map(|d| {
                Some((
                    d.serial.clone(),
                    poses.get(d.index as usize).copied().flatten()?,
                ))
            })
            .collect();
        drop(client);
        if stations.is_empty() {
            return Err("SteamVR reports no base station poses (is it running?)".into());
        }
        let cal = file::load(&file::default_path())?;
        let reg_path = align::registration_path();
        let mut reg = align::load_registration(&reg_path)?;
        let x0 = reg.lighthouse_from_world.iso();
        let (models, _) = track::refine::recording_models(rec)?;
        let serials: Vec<String> = cal.sensors.iter().map(|e| e.serial.clone()).collect();
        let cams = serials
            .iter()
            .map(|s| {
                models
                    .get(s)
                    .copied()
                    .ok_or(format!("no intrinsics for {s} in the recording"))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let world_from_cam: Vec<nalgebra::Isometry3<f64>> = cal
            .sensors
            .iter()
            .map(|e| calib::iso_from_xyzw(e.orient_xyzw, e.pos))
            .collect();
        let mut blobs: Vec<Vec<nalgebra::Vector2<f64>>> = vec![Vec::new(); serials.len()];
        let mut frames = vec![0usize; serials.len()];
        for r in Reader::open(rec)? {
            if r.rtype != recorder::FRAME_OBS {
                continue;
            }
            let Some((info, bl)) = recorder::decode_frame_obs(&r.payload) else {
                continue;
            };
            let Some(c) = serials
                .iter()
                .position(|s| *s == cv1touch::engine::cstr_field(&info.serial))
            else {
                continue;
            };
            frames[c] += 1;
            blobs[c].extend(
                bl.iter()
                    .filter(|b| b.led_id < 0)
                    .map(|b| nalgebra::Vector2::new(b.x as f64, b.y as f64)),
            );
        }
        let station_poses: Vec<nalgebra::Isometry3<f64>> = stations.iter().map(|s| s.1).collect();
        let mut spots = Vec::new();
        for (c, cam) in cams.iter().enumerate() {
            for (i, (serial, st)) in stations.iter().enumerate() {
                let pc =
                    world_from_cam[c].inverse() * (x0.inverse() * stations::station_point(st, 0.0));
                let Some(uv) = cam.project(&pc.coords).filter(|uv| {
                    pc.z > 0.0
                        && uv.x >= 0.0
                        && uv.y >= 0.0
                        && uv.x < cam.width as f64
                        && uv.y < cam.height as f64
                }) else {
                    continue;
                };
                match stations::spot(&blobs[c], uv, 40.0, 3.0, 100) {
                    Some((xy, n)) => {
                        println!("station {serial} in sensor {}: predicted ({:.1}, {:.1}), seen at ({:.1}, {:.1}) in {n} of {} frames", serials[c], uv.x, uv.y, xy.x, xy.y, frames[c]);
                        spots.push(stations::Spot {
                            camera: c,
                            station: i,
                            xy,
                            frames: n,
                        });
                    }
                    None => println!(
                        "station {serial} in sensor {}: predicted ({:.1}, {:.1}), not seen",
                        serials[c], uv.x, uv.y
                    ),
                }
            }
        }
        let report = |name: &str, f: &stations::Fit| {
            let d = f.lighthouse_from_world.inverse() * x0;
            let r: Vec<String> = f.residuals.iter().map(|r| format!("{r:.2}")).collect();
            println!("{name}: residuals [{}] px; differs from the installed registration by {:.1} mm, {:.2} deg (aperture {:.1} mm)", r.join(", "), d.translation.vector.norm() * 1e3, d.rotation.angle().to_degrees(), f.aperture * 1e3);
        };
        let plain = stations::solve(
            &cams,
            &world_from_cam,
            &station_poses,
            &spots,
            &x0,
            0.0,
            false,
        )
        .ok_or("too few station spots to solve (need 3)")?;
        report("stations", &plain);
        let best = match stations::solve(
            &cams,
            &world_from_cam,
            &station_poses,
            &spots,
            &x0,
            0.0,
            true,
        ) {
            Some(f) => {
                report("stations, aperture fitted", &f);
                f
            }
            None => plain,
        };
        if !write {
            println!("(not written: run with --write)");
            return Ok(());
        }
        reg.lighthouse_from_world = align::PoseEntry::from_iso(&best.lighthouse_from_world);
        reg.created_unix_s = unix_now();
        reg.touch = "base stations".into();
        reg.pairs = spots.len();
        reg.pos_rms_mm = 0.0;
        reg.rot_rms_deg = 0.0;
        let reg_path = out.map_or(reg_path, Path::to_path_buf);
        let b = align::save(&reg_path, &reg)?;
        println!(
            "wrote {} (previous: {:?}); run `cv1touch alignment reload`",
            reg_path.display(),
            b
        );
        Ok(())
    };
    match run() {
        Ok(()) => 0,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

/// Joint calibration from a rig capture's static holds (see `calib::rig`): the sensors (the
/// calibration's reference sensor held), the registration and the Touches' rig offsets.
fn calibrate_rig(base: &Path, write: bool, out_dir: Option<&Path>) -> i32 {
    use cv1touch::calib::rig;
    let run = || -> Result<(), String> {
        let truth_path = base.with_extension("truth.json");
        let tf: eval::TruthFile = serde_json::from_str(
            &std::fs::read_to_string(&truth_path)
                .map_err(|e| format!("{}: {e}", truth_path.display()))?,
        )
        .map_err(|e| e.to_string())?;
        let cal_path = file::default_path();
        let mut cal = file::load(&cal_path)?;
        let reg_path = align::registration_path();
        let mut reg = align::load_registration(&reg_path)?;
        if reg.reference != tf.reference {
            return Err(format!(
                "the capture's reference controller is {}, the registration's {}",
                tf.reference, reg.reference
            ));
        }
        let holds = rig::holds(&eval::truth_samples(&tf.samples), 1_500_000_000);
        let serials: Vec<String> = cal.sensors.iter().map(|e| e.serial.clone()).collect();
        let fixed = serials
            .iter()
            .position(|s| *s == cal.stats.reference)
            .ok_or("the calibration's reference sensor is not among its sensors")?;
        let (models, leds) = track::refine::recording_models(&base.with_extension("rec"))?;
        let cameras = serials
            .iter()
            .map(|s| {
                models
                    .get(s)
                    .copied()
                    .ok_or(format!("no intrinsics for sensor {s} in the recording"))
            })
            .collect::<Result<Vec<_>, _>>()?;
        // frames per hold and sensor; Touch hold means from the published poses
        let mut frames: Vec<Vec<Vec<Vec<nalgebra::Vector2<f64>>>>> =
            vec![vec![Vec::new(); serials.len()]; holds.len()];
        for rec in Reader::open(&base.with_extension("rec"))? {
            if rec.rtype != recorder::FRAME_OBS {
                continue;
            }
            let Some((info, blobs)) = recorder::decode_frame_obs(&rec.payload) else {
                continue;
            };
            let Some(c) = serials
                .iter()
                .position(|s| *s == cv1touch::engine::cstr_field(&info.serial))
            else {
                continue;
            };
            let t = info.frame_start_host_ns;
            if let Some(k) = holds.iter().position(|h| t >= h.0 && t <= h.1) {
                frames[k][c].push(
                    blobs
                        .iter()
                        .map(|b| nalgebra::Vector2::new(b.x as f64, b.y as f64))
                        .collect(),
                );
            }
        }
        let published = replayed_now(base)?;
        let x0 = reg.lighthouse_from_world.iso();
        let mut touches = Vec::new();
        let mut offsets = Vec::new();
        let mut names = Vec::new();
        for (h, pubs) in published.iter().enumerate() {
            let Some(model) = leds[h].clone() else {
                continue;
            };
            let ys: Vec<nalgebra::Isometry3<f64>> = holds
                .iter()
                .filter_map(|&(a, b, l)| Some(l.inverse() * x0 * mean_pose(pubs, a, b)?))
                .collect();
            if ys.len() < 8 {
                continue;
            }
            let t = ys
                .iter()
                .map(|y| y.translation.vector)
                .sum::<nalgebra::Vector3<f64>>()
                / ys.len() as f64;
            let r = calib::motion::chordal_mean(&ys.iter().map(|y| y.rotation).collect::<Vec<_>>());
            touches.push(model);
            offsets.push(nalgebra::Isometry3::from_parts(
                nalgebra::Translation3::from(t),
                r,
            ));
            names.push(["left", "right"][h]);
        }
        if touches.is_empty() {
            return Err("no Touch was tracked through the holds".into());
        }
        let problem = rig::Problem {
            cameras,
            fixed,
            touches,
            holds: holds
                .iter()
                .zip(frames)
                .map(|(&(_, _, l), f)| rig::Hold {
                    lighthouse_from_ref: l,
                    frames: f,
                })
                .collect(),
        };
        let initial = rig::State {
            cam_from_world: cal
                .sensors
                .iter()
                .map(|e| calib::iso_from_xyzw(e.orient_xyzw, e.pos).inverse())
                .collect(),
            lighthouse_from_world: x0,
            ref_from_touch: offsets,
        };
        println!(
            "{} holds, Touches {}; fixed sensor {}",
            holds.len(),
            names.join(" and "),
            serials[fixed]
        );
        let stats = |state: &rig::State, obs: &[rig::Sighting]| -> String {
            (0..serials.len())
                .map(|c| {
                    let mut r: Vec<f64> = rig::residuals(
                        &problem,
                        state,
                        &obs.iter()
                            .filter(|o| o.camera == c)
                            .copied()
                            .collect::<Vec<_>>(),
                    );
                    r.sort_by(f64::total_cmp);
                    if r.is_empty() {
                        return format!("{}: -", serials[c]);
                    }
                    format!(
                        "{}: {} LEDs, median {:.3} p90 {:.3} px",
                        serials[c],
                        r.len(),
                        r[r.len() / 2],
                        r[r.len() * 9 / 10]
                    )
                })
                .collect::<Vec<_>>()
                .join("; ")
        };
        // validation: fit on even holds, compare on odd holds at a 1 px gate
        let half = rig::calibrate(&problem, &initial, &|k| k % 2 == 0);
        let odd = |state: &rig::State| -> Vec<rig::Sighting> {
            rig::associate(&problem, state, 1.0, 0.8)
                .into_iter()
                .filter(|o| o.hold % 2 == 1)
                .collect()
        };
        println!(
            "held-out holds, current calibration: {}",
            stats(&initial, &odd(&initial))
        );
        println!(
            "held-out holds, fitted on the others: {}",
            stats(&half.state, &odd(&half.state))
        );
        let full = rig::calibrate(&problem, &initial, &|_| true);
        println!(
            "all holds, current: {}",
            stats(&initial, &rig::associate(&problem, &initial, 1.0, 0.8))
        );
        println!(
            "all holds, fitted:  {}",
            stats(&full.state, &full.sightings)
        );
        for (c, serial) in serials.iter().enumerate() {
            let d = full.state.cam_from_world[c].inverse().inverse()
                * initial.cam_from_world[c].inverse();
            println!(
                "  sensor {} moves {:.2} mm, {:.3} deg",
                serial,
                d.translation.vector.norm() * 1e3,
                d.rotation.angle().to_degrees()
            );
        }
        let d = full.state.lighthouse_from_world.inverse() * x0;
        println!(
            "  registration moves {:.2} mm, {:.3} deg",
            d.translation.vector.norm() * 1e3,
            d.rotation.angle().to_degrees()
        );
        if full.state.ref_from_touch.len() == 2 {
            let lr = full.state.ref_from_touch[0].inverse() * full.state.ref_from_touch[1];
            println!(
                "  Touch {} <- Touch {} on the rig: {:.1} mm, {:.2} deg",
                names[0],
                names[1],
                lr.translation.vector.norm() * 1e3,
                lr.rotation.angle().to_degrees()
            );
        }
        if !write {
            println!("(not written: run with --write)");
            return Ok(());
        }
        for (c, e) in cal.sensors.iter_mut().enumerate() {
            let (o, p) = calib::iso_to_xyzw(&full.state.cam_from_world[c].inverse());
            e.orient_xyzw = o;
            e.pos = p;
        }
        cal.source = format!("{}; rig-calibrated against {}", cal.source, base.display());
        let (cal_path, reg_path) = match out_dir {
            Some(d) => (d.join("calibration.json"), d.join("registration.json")),
            None => (cal_path, reg_path),
        };
        let cb = file::save(&cal_path, &cal)?;
        reg.lighthouse_from_world = align::PoseEntry::from_iso(&full.state.lighthouse_from_world);
        reg.created_unix_s = unix_now();
        reg.touch = format!("rig ({})", names.join("+"));
        reg.pairs = full.sightings.len();
        // how far the tracked Touches sit from where the fitted rig puts them, over the holds
        let (mut se_t, mut se_r, mut n) = (0.0, 0.0, 0usize);
        for (ti, name) in names.iter().enumerate() {
            let h = if *name == "left" { 0 } else { 1 };
            for (k, &(a, b, _)) in holds.iter().enumerate() {
                if let Some(p) = mean_pose(&published[h], a, b) {
                    let d = full.state.world_from_touch(&problem.holds[k], ti).inverse() * p;
                    se_t += d.translation.vector.norm_squared();
                    se_r += d.rotation.angle().powi(2);
                    n += 1;
                }
            }
        }
        reg.pos_rms_mm = (se_t / n.max(1) as f64).sqrt() * 1e3;
        reg.rot_rms_deg = (se_r / n.max(1) as f64).sqrt().to_degrees();
        let rb = align::save(&reg_path, &reg)?;
        println!(
            "wrote {} (previous: {:?}) and {} (previous: {:?})",
            cal_path.display(),
            cb,
            reg_path.display(),
            rb
        );
        println!(
            "load them: cv1touch calibration load {} && cv1touch alignment reload",
            cal_path.display()
        );
        Ok(())
    };
    match run() {
        Ok(()) => 0,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

/// Re-poses sensors that disagree with the others, against the controller poses the tracker gets
/// from the others: the worst one first, then again with it fixed, until none improves.
fn calibrate_refine(rec: &Path, path: &Path, write: bool) -> i32 {
    let run = || -> Result<i32, String> {
        let mut cal = file::load(path)?;
        let (models, leds) = track::refine::recording_models(rec)?;
        let dir = track::factory::openhmd_config_dir();
        let imu = |l: &[cv1touch::calib::capture::Led]| track::factory::imu_position(&dir, l);
        let mut changed = Vec::new();
        for round in 1..=3 {
            let mut results = Vec::new();
            for entry in &cal.sensors {
                let Some(model) = models.get(&entry.serial) else {
                    continue;
                };
                let o = track::replay::Options {
                    probe_sensors: vec![entry.serial.clone()],
                    ..Default::default()
                };
                let r = track::replay::replay(rec, &cal, &imu, &o)?;
                let initial = calib::iso_from_xyzw(entry.orient_xyzw, entry.pos);
                match track::refine::refine_sensor(
                    model,
                    &initial,
                    [leds[0].as_deref(), leds[1].as_deref()],
                    &r.probes,
                    0.002,
                ) {
                    Ok(rf) => {
                        let d = rf.world_from_cam.inverse() * initial;
                        println!(
                            "round {round}: {}: {} matches in {} frames; median residual {:.2} -> {:.2} px; moves {:.1} mm, {:.3} deg",
                            entry.serial,
                            rf.pairs,
                            rf.frames,
                            rf.before_px,
                            rf.after_px,
                            d.translation.vector.norm() * 1e3,
                            d.rotation.angle().to_degrees()
                        );
                        results.push((entry.serial.clone(), rf));
                    }
                    Err(e) => println!("round {round}: {}: {e}", entry.serial),
                }
            }
            let Some((serial, rf)) = results
                .into_iter()
                .filter(|(_, rf)| rf.before_px - rf.after_px > 0.15)
                .max_by(|a, b| a.1.before_px.total_cmp(&b.1.before_px))
            else {
                break;
            };
            println!("  -> re-posing {serial}");
            let entry = cal.sensors.iter_mut().find(|e| e.serial == serial).unwrap();
            let (orient, pos) = calib::iso_to_xyzw(&rf.world_from_cam);
            (entry.orient_xyzw, entry.pos, entry.rms_px) = (orient, pos, rf.after_px);
            changed.push(serial);
        }
        if changed.is_empty() {
            println!("no sensor improves by re-posing");
            return Ok(0);
        }
        if !write {
            println!(
                "(not written: run with --write to save {})",
                changed.join(", ")
            );
            return Ok(0);
        }
        cal.source = format!(
            "{}; re-posed {} against {}",
            cal.source,
            changed.join(", "),
            rec.display()
        );
        let mut backup = None;
        commit_fit(interrupted(), || {
            backup = file::save(path, &cal)?;
            Ok(())
        })?;
        println!(
            "wrote {}{}",
            path.display(),
            backup.map_or(String::new(), |b| format!(" (previous: {})", b.display()))
        );
        println!(
            "restart cv1touchd or run `cv1touch calibration load {}` to use it",
            path.display()
        );
        Ok(0)
    };
    run().unwrap_or_else(|e| {
        eprintln!("cv1touch: {e}");
        1
    })
}

/// How the Lighthouse universe moved since the installed registration was fitted, station by
/// station (what cv1touchd's universe watch follows).
fn universe_cmd() -> i32 {
    let run = || -> Result<(), String> {
        let reg = align::load_registration(&align::registration_path())?;
        if reg.stations.is_empty() {
            return Err("the installed registration records no base stations (refit it with `cv1touch register --holds BASE --write`)".into());
        }
        let client = vr::Client::connect()?;
        let now = client.stations();
        drop(client);
        let then: Vec<(String, nalgebra::Isometry3<f64>)> = reg
            .stations
            .iter()
            .map(|s| (s.serial.clone(), s.pose.iso()))
            .collect();
        let play = reg.play_area_point();
        println!(
            "play area (raw, when fitted): ({:+.2}, {:+.2}, {:+.2}){}",
            play.x,
            play.y,
            play.z,
            if reg.play_area.is_none() {
                " (guessed: the registration records none)"
            } else {
                ""
            }
        );
        for (serial, t) in &then {
            match now.iter().find(|(s, _)| s == serial) {
                Some((_, n)) => {
                    let d = n * t.inverse();
                    println!("{serial}: believed pose moved {:.2} mm, {:.3} deg; moves what it tracks in the play area by {:.1} mm", d.translation.vector.norm() * 1e3, d.rotation.angle().to_degrees(), (d * play - play).norm() * 1e3);
                }
                None => println!("{serial}: not tracked now"),
            }
        }
        match reg.follow(&now) {
            Some(s) => println!(
                "universe moved {:.1} mm, {:.2} deg in the play area since the registration ({} stations, disagreeing there by {:.1} mm rms)",
                (s.now_from_then * play - play).norm() * 1e3,
                s.now_from_then.rotation.angle().to_degrees(),
                s.stations,
                s.residual_m * 1e3
            ),
            None => println!("the stations do not agree on a move (fewer than two in common, or one was bumped)"),
        }
        Ok(())
    };
    match run() {
        Ok(()) => 0,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

/// Places a sensor that has no pose yet (or re-places one) from a recording of the controllers
/// moving in its view while the other sensors track them: the engine's LED labels on its blobs
/// give a first pose, `refine_sensor` the final one.
fn calibrate_place(serial: &str, rec: &Path, path: &Path, write: bool) -> i32 {
    let run = || -> Result<(), String> {
        let mut cal = file::load(path)?;
        let (models, leds) = track::refine::recording_models(rec)?;
        let model = models
            .get(serial)
            .ok_or_else(|| format!("{serial} is not in {}", rec.display()))?;
        let dir = track::factory::openhmd_config_dir();
        let imu = |l: &[capture::Led]| track::factory::imu_position(&dir, l);
        let mut without = cal.clone();
        without.sensors.retain(|e| e.serial != serial);
        let o = track::replay::Options {
            probe_sensors: vec![serial.to_string()],
            ..Default::default()
        };
        let r = track::replay::replay(rec, &without, &imu, &o)?;
        let bodies = [leds[0].as_deref(), leds[1].as_deref()];
        let placed = track::refine::place_sensor(model, bodies, &r.probes, 0.002)?;
        println!(
            "{serial}: placed from {} of {} labelled LED sightings in {} frames, median {:.2} px",
            placed.inliers,
            placed.offered,
            r.probes.len(),
            placed.median_px
        );
        let rf =
            track::refine::refine_sensor(model, &placed.world_from_cam, bodies, &r.probes, 0.002)?;
        let d = rf.world_from_cam.inverse() * placed.world_from_cam;
        let p = rf.world_from_cam.translation.vector;
        let fwd = rf.world_from_cam.rotation * nalgebra::Vector3::z();
        println!(
            "refined on {} matches in {} frames: median {:.2} px (moved {:.1} mm, {:.2} deg); at ({:.2}, {:.2}, {:.2}) m looking ({:+.2}, {:+.2}, {:+.2})",
            rf.pairs, rf.frames, rf.after_px, d.translation.vector.norm() * 1e3, d.rotation.angle().to_degrees(), p.x, p.y, p.z, fwd.x, fwd.y, fwd.z
        );
        if let Some(old) = cal.sensors.iter().find(|e| e.serial == serial) {
            let d = calib::iso_from_xyzw(old.orient_xyzw, old.pos).inverse() * rf.world_from_cam;
            println!(
                "the calibration had it {:.1} mm, {:.2} deg away",
                d.translation.vector.norm() * 1e3,
                d.rotation.angle().to_degrees()
            );
        }
        if !write {
            println!("(not written: run with --write)");
            return Ok(());
        }
        let (orient_xyzw, pos) = calib::iso_to_xyzw(&rf.world_from_cam);
        cal.sensors.retain(|e| e.serial != serial);
        cal.sensors.push(file::SensorEntry {
            serial: serial.to_string(),
            orient_xyzw,
            pos,
            rms_px: rf.after_px,
            observations: rf.pairs,
        });
        cal.source = format!("{}; placed {serial} against {}", cal.source, rec.display());
        let mut backup = None;
        commit_fit(interrupted(), || {
            backup = file::save(path, &cal)?;
            Ok(())
        })?;
        println!(
            "wrote {}{}; restart cv1touchd to use it",
            path.display(),
            backup.map_or(String::new(), |b| format!(" (previous: {})", b.display()))
        );
        Ok(())
    };
    match run() {
        Ok(()) => 0,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

/// Stage the related calibration and registration before changing either live file.
/// Backups are copied, so a failed commit never removes the previous live configuration.
fn save_gravity_files(
    cal: &Path,
    cal_bytes: &[u8],
    reg: Option<(&Path, &[u8])>,
) -> Result<(), String> {
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|e| e.to_string())?
        .as_nanos();
    let mut staged: Vec<(PathBuf, PathBuf, Option<PathBuf>)> = Vec::new();
    let mut payloads = vec![(cal, cal_bytes)];
    if let Some(r) = reg {
        payloads.push(r);
    }
    let stage = (|| -> Result<(), String> {
        for (path, bytes) in payloads {
            if let Some(parent) = path.parent() {
                std::fs::create_dir_all(parent)
                    .map_err(|e| format!("{}: {e}", parent.display()))?;
            }
            let tmp = path.with_file_name(format!(
                "{}.stage-{nonce}",
                path.file_name().unwrap_or_default().to_string_lossy()
            ));
            let backup = if path.exists() {
                Some(path.with_file_name(format!(
                    "{}.bak-{nonce}",
                    path.file_name().unwrap_or_default().to_string_lossy()
                )))
            } else {
                None
            };
            staged.push((path.to_path_buf(), tmp.clone(), backup.clone()));
            std::fs::write(&tmp, bytes).map_err(|e| format!("{}: {e}", tmp.display()))?;
            if let Some(b) = backup {
                std::fs::copy(path, &b).map_err(|e| format!("{}: {e}", path.display()))?;
            }
        }
        Ok(())
    })();
    if let Err(e) = stage {
        for (_, tmp, backup) in &staged {
            let _ = std::fs::remove_file(tmp);
            if let Some(b) = backup {
                let _ = std::fs::remove_file(b);
            }
        }
        return Err(e);
    }
    for (i, (path, tmp, _)) in staged.iter().enumerate() {
        if let Err(e) = std::fs::rename(tmp, path) {
            let mut rollback = Vec::new();
            for (old, restore, backup) in staged[..i].iter().rev() {
                let restored = if let Some(b) = backup {
                    std::fs::copy(b, restore).and_then(|_| std::fs::rename(restore, old))
                } else {
                    std::fs::remove_file(old)
                };
                if let Err(err) = restored {
                    rollback.push(format!("{}: {err}", old.display()));
                }
            }
            for (_, remaining, _) in &staged {
                let _ = std::fs::remove_file(remaining);
            }
            return Err(format!(
                "{}: {e}; rollback {}",
                path.display(),
                if rollback.is_empty() {
                    "completed".into()
                } else {
                    rollback.join(", ")
                }
            ));
        }
    }
    for (path, _, backup) in staged {
        println!("wrote {} (previous: {:?})", path.display(), backup);
    }
    Ok(())
}

/// Levels the calibration's world with gravity from the static holds in a recording (see
/// `calib::gravity`): each Touch's accelerometer is calibrated from its holds, and the tracked
/// orientations carry the measured up into the world. Turning the world turns the registration
/// with it, so SteamVR sees the same poses.
fn calibrate_gravity(rec: &Path, path: &Path, write: bool) -> i32 {
    calibrate_gravity_with(rec, path, write, true)
}

fn calibrate_gravity_with(
    rec: &Path,
    path: &Path,
    write: bool,
    compensate_registration: bool,
) -> i32 {
    use cv1touch::calib::gravity;
    use nalgebra::{UnitQuaternion, Vector3};
    let run = || -> Result<(), String> {
        let cal0 = file::load(path)?;
        let dir = track::factory::openhmd_config_dir();
        let imu = |l: &[capture::Led]| track::factory::imu_position(&dir, l);
        let mut accel: [Vec<(u64, Vector3<f64>)>; 2] = [vec![], vec![]];
        for r in Reader::open(rec)? {
            if r.rtype != recorder::CONTROLLER_STATE {
                continue;
            }
            let Some((h, st)) = recorder::decode_state(&r.payload) else {
                continue;
            };
            if h < 2 && accel[h].last().is_none_or(|p| p.0 < st.sample_host_ns) {
                accel[h].push((st.sample_host_ns, calib::vec3(st.imu_accel)));
            }
        }
        let turned = |c: &UnitQuaternion<f64>| {
            let mut cal = cal0.clone();
            let turn = nalgebra::Isometry3::from_parts(nalgebra::Translation3::identity(), *c);
            for s in &mut cal.sensors {
                (s.orient_xyzw, s.pos) =
                    calib::iso_to_xyzw(&(turn * calib::iso_from_xyzw(s.orient_xyzw, s.pos)));
            }
            cal
        };
        // The world's remaining tilt as the tracker sees it, the holds weighted alike. The filter
        // leans part of the way toward the gravity it is told, so this reads short of the truth
        // by a fraction that the secant steps below measure; the loose accelerometer noise keeps
        // that fraction small.
        let mut first = true;
        let mut measure = |cal: &file::CalibrationFile,
                           round: usize|
         -> Result<Vector3<f64>, String> {
            let mut o = track::replay::Options::default();
            (
                o.params.noise.accel,
                o.params.noise.accel_bias,
                o.params.noise.accel_per_accel,
            ) = (0.05, 2e-3, 0.0);
            let r = track::replay::replay(rec, cal, &imu, &o)?;
            let (mut sum, mut n) = (Vector3::zeros(), 0usize);
            for (h, hand_accel) in accel.iter().enumerate() {
                let poses: Vec<eval::TruthSample> = r.hands[h]
                    .iter()
                    .map(|p| eval::TruthSample {
                        t_ns: p.t_ns,
                        pose: p.pose,
                    })
                    .collect();
                let mut rots = vec![];
                let mut raw = vec![];
                for (a, b, pose) in calib::rig::holds(&poses, 1_500_000_000) {
                    let s: Vec<Vector3<f64>> = hand_accel
                        .iter()
                        .filter(|x| x.0 >= a && x.0 <= b)
                        .map(|x| x.1)
                        .collect();
                    if s.len() >= 100 {
                        rots.push(pose.rotation);
                        raw.push(s.iter().sum::<Vector3<f64>>() / s.len() as f64);
                    }
                }
                let name = ["left", "right"][h];
                let Some(ac) = gravity::accel_cal(&raw, gravity::STANDARD_GRAVITY) else {
                    if first {
                        println!("Touch {name}: {} holds, too few or too alike to calibrate its accelerometer", raw.len());
                    }
                    continue;
                };
                if first {
                    println!(
                        "Touch {name}: {} holds; accelerometer bias ({:+.3}, {:+.3}, {:+.3}) m/s^2, scale ({:.4}, {:.4}, {:.4}), fits |g| to {:.4} m/s^2 rms",
                        raw.len(), ac.b.x, ac.b.y, ac.b.z, ac.d[(0, 0)], ac.d[(1, 1)], ac.d[(2, 2)], ac.rms
                    );
                }
                let ups: Vec<Vector3<f64>> = raw.iter().map(|a| ac.apply(a)).collect();
                let (corr, res) = gravity::world_tilt(&rots, &ups);
                let rms = (res.iter().map(|x| x * x).sum::<f64>() / res.len() as f64).sqrt();
                println!("round {round}: Touch {name}: reads {:.3} deg of tilt; tracked tilt vs accelerometer {:.3} deg rms per hold", corr.norm().to_degrees(), rms.to_degrees());
                sum += corr * raw.len() as f64;
                n += raw.len();
            }
            first = false;
            if n == 0 {
                return Err("no Touch has enough holds in different orientations".into());
            }
            Ok(sum / n as f64)
        };
        let m0 = measure(&cal0, 1)?;
        if m0.norm() < 0.02f64.to_radians() {
            println!("level already");
            return Ok(());
        }
        // secant: the reading falls by `gain` per unit of correction applied
        let mut c = m0;
        let mut m_prev = m0;
        let mut c_prev = Vector3::zeros();
        let mut gain = 1.0;
        for round in 2..=4 {
            let m = measure(&turned(&UnitQuaternion::from_scaled_axis(c)), round)?;
            let dc = c - c_prev;
            gain = ((m_prev - m).dot(&dc) / dc.norm_squared()).clamp(0.3, 1.5);
            if m.norm() < 0.02f64.to_radians() {
                break;
            }
            (c_prev, m_prev) = (c, m);
            c += m / gain;
        }
        println!("the tracker read {:.0}% of the tilt", gain * 100.0);
        let total = UnitQuaternion::from_scaled_axis(c);
        let cal = turned(&total);
        let v = total.scaled_axis().map(|x| x.to_degrees());
        println!(
            "world turned ({:+.3}, {:+.3}, {:+.3}) deg ({:.3} deg) to gravity",
            v.x,
            v.y,
            v.z,
            total.angle().to_degrees()
        );
        if !write {
            println!("(not written: run with --write)");
            return Ok(());
        }
        let mut cal = cal;
        cal.source = format!(
            "{}; levelled ({:+.3}, {:+.3}, {:+.3}) deg from the holds of {}",
            cal.source,
            v.x,
            v.y,
            v.z,
            rec.display()
        );
        cal.validate()?;
        let reg_path = align::registration_path();
        let turned_reg = if compensate_registration && reg_path.exists() {
            let mut reg = align::load_registration(&reg_path)?;
            let turn = nalgebra::Isometry3::from_parts(nalgebra::Translation3::identity(), total);
            reg.lighthouse_from_world =
                align::PoseEntry::from_iso(&(reg.lighthouse_from_world.iso() * turn.inverse()));
            Some(serde_json::to_vec_pretty(&reg).map_err(|e| e.to_string())?)
        } else {
            None
        };
        let cal_bytes = serde_json::to_vec_pretty(&cal).map_err(|e| e.to_string())?;
        commit_fit(interrupted(), || {
            save_gravity_files(
                path,
                &cal_bytes,
                turned_reg.as_deref().map(|b| (reg_path.as_path(), b)),
            )
        })?;
        println!("restart cv1touchd to use them");
        Ok(())
    };
    match run() {
        Ok(()) => 0,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

/// Replays a recording through the Rust tracker and evaluates its output against the jig truth.
fn track_cmd(base: &Path, args: &[&str]) -> i32 {
    let run = || -> Result<track::replay::Replay, String> {
        let (mut cal_path, mut rest) = (file::default_path(), Vec::new());
        let mut it = args.iter();
        while let Some(a) = it.next() {
            match *a {
                "--calibration" => {
                    cal_path = PathBuf::from(*it.next().ok_or("--calibration needs a path")?)
                }
                "--registration" => {
                    it.next().ok_or("--registration needs a path")?;
                }
                "--fit-registration" => {}
                _ => rest.push(*a),
            }
        }
        let o = track_options(&rest)?;
        let cal = file::load(&cal_path)?;
        let dir = track::factory::openhmd_config_dir();
        track::replay::replay(
            &base.with_extension("rec"),
            &cal,
            &|leds| track::factory::imu_position(&dir, leds),
            &o,
        )
    };
    match run() {
        Ok(r) => {
            for (hand, st) in r.stats.iter().enumerate() {
                if st.frames > 0 {
                    println!(
                        "tracker {}: {} frames: {} updates ({:.1} LEDs each, {} out of order, {} dropped on rebuild), {} not in view, {} too few matches, {} rejected; {} acquisitions ({} failed, {} refused); {} too old; {} outputs while tracking",
                        ["left", "right"][hand],
                        st.frames,
                        st.updates,
                        st.leds_used as f64 / st.updates.max(1) as f64,
                        st.out_of_order,
                        st.dropped_on_rebuild,
                        st.not_visible,
                        st.too_few,
                        st.rejected,
                        st.acquisitions,
                        st.failed_acquisitions,
                        st.refused_acquisitions,
                        st.too_old,
                        r.hands[hand].len()
                    );
                }
            }
            for (serial, a) in &r.agreement {
                println!(
                    "sensor {serial}: agrees with the others to {:.2} px median, offset {:.2} px ({} cross-checks); missed {} of {} checks",
                    a.median_px,
                    a.bias_px.norm(),
                    a.samples,
                    a.misses,
                    a.checks
                );
            }
            let mut registration = args
                .iter()
                .position(|a| *a == "--registration")
                .and_then(|i| args.get(i + 1))
                .map_or_else(align::registration_path, PathBuf::from);
            if args.contains(&"--fit-registration") {
                // a registration fitted to this capture (for a calibration in another world frame)
                match fit_capture_registration(base, &r.hands, &registration) {
                    Ok(path) => registration = path,
                    Err(e) => eprintln!("cv1touch: no registration fitted: {e}"),
                }
            }
            eval_report_with(base, &r.hands, &registration, false)
        }
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

/// Fits a registration to a capture (the tracked Touch against its truth) and writes it beside the
/// capture for the evaluation.
fn fit_capture_registration(
    base: &Path,
    hands: &[Vec<eval::Published>; 2],
    template: &Path,
) -> Result<PathBuf, String> {
    let truth_path = base.with_extension("truth.json");
    let tf: eval::TruthFile =
        serde_json::from_str(&std::fs::read_to_string(&truth_path).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
    let truth = tf.truth();
    let (hand, _) = eval::pick_hand([&hands[0], &hands[1]], &truth)
        .ok_or("no Touch moved with the reference")?;
    let pairs: Vec<_> = hands[hand]
        .iter()
        .filter_map(|p| Some((eval::interpolate(&truth, p.t_ns)?, p.pose)))
        .collect();
    let sol = register::solve(
        &pairs,
        &register::Options {
            min_pairs: 500,
            ..register::Options::default()
        },
    )?;
    let mut reg = align::load_registration(template)?;
    reg.lighthouse_from_world = align::PoseEntry::from_iso(&sol.lighthouse_from_world);
    let out = std::env::temp_dir().join(format!(
        "cv1touch-fitted-registration-{}.json",
        std::process::id()
    ));
    align::save(&out, &reg)?;
    Ok(out)
}

/// Accuracy and latency of published poses against the jig truth recorded beside them.
fn eval_report(base: &Path, hands: &[Vec<eval::Published>; 2]) -> i32 {
    eval_report_with(base, hands, &align::registration_path(), true)
}

/// `published_by_daemon`: the poses are the daemon's own records, whose stamps carry the latency
/// adjustment the capture recorded; a replay's stamps carry none, so its lag is the tracker's and
/// gets no advice on the adjustment.
fn eval_report_with(
    base: &Path,
    hands: &[Vec<eval::Published>; 2],
    registration: &Path,
    published_by_daemon: bool,
) -> i32 {
    let truth_path = base.with_extension("truth.json");
    let tf: eval::TruthFile = match std::fs::read_to_string(&truth_path)
        .map_err(|e| e.to_string())
        .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
    {
        Ok(t) => t,
        Err(e) => {
            eprintln!("cv1touch: {}: {e}", truth_path.display());
            return 1;
        }
    };
    let truth = tf.truth();
    let reg = match align::load_registration(registration) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return 1;
        }
    };
    let Some((hand, corr)) = eval::pick_hand([&hands[0], &hands[1]], &truth) else {
        eprintln!("cv1touch: no Touch moved with the reference");
        return 1;
    };
    let pubs = &hands[hand];
    let window = |name: &str, k: usize| {
        tf.windows
            .iter()
            .filter(|w| w.0 == name)
            .nth(k)
            .map(|w| (w.1, w.2))
    };
    let in_window = |w: Option<(u64, u64)>| -> Vec<eval::Published> {
        w.map_or(Vec::new(), |(a, b)| {
            pubs.iter()
                .filter(|p| p.t_ns >= a && p.t_ns <= b)
                .copied()
                .collect()
        })
    };
    let (still1, moving, still2) = (
        in_window(window("still", 0)),
        in_window(window("move", 0)),
        in_window(window("still", 1)),
    );
    let name = ["left", "right"][hand];
    println!(
        "{name} Touch on the jig (correlation {corr:.2}); {} published poses, {} truth samples",
        pubs.len(),
        truth.len()
    );
    // a still phase counts only while the truth itself is still (the pair may have been handled)
    let truly_still = |set: &[eval::Published]| -> Vec<eval::Published> {
        let Some(mid) = set
            .get(set.len() / 2)
            .and_then(|p| eval::interpolate(&truth, p.t_ns))
        else {
            return Vec::new();
        };
        set.iter()
            .filter(|p| {
                eval::interpolate(&truth, p.t_ns).is_some_and(|l| {
                    (l.translation.vector - mid.translation.vector).norm() < 0.002
                        && l.rotation.angle_to(&mid.rotation) < 0.3f64.to_radians()
                })
            })
            .copied()
            .collect()
    };
    let (still1, still2) = (truly_still(&still1), truly_still(&still2));
    let report = |label: &str, x: &nalgebra::Isometry3<f64>, advise: bool| -> f64 {
        let params = eval::SegmentParams::default();
        let first = eval::Truth {
            samples: &truth,
            world_from_lh: x.inverse(),
            segments: eval::offset_segments(pubs, &truth, x, 0, &params),
        };
        let (lag, _) = eval::lag_ms(&moving, &first, (-50, 150));
        let segments = eval::offset_segments(pubs, &truth, x, (lag * 1e6) as i64, &params);
        let tr = eval::Truth {
            samples: &truth,
            world_from_lh: x.inverse(),
            segments,
        };
        let e0 = eval::errors(&moving, &tr, 0);
        let el = eval::errors(&moving, &tr, (lag * 1e6) as i64);
        let (hops, n) = eval::hops(&moving, &tr, (lag * 1e6) as i64, 0.005);
        println!("== {label}");
        let t0 = pubs.first().map_or(0, |p| p.t_ns);
        let segs: Vec<String> = tr
            .segments
            .iter()
            .map(|s| {
                format!(
                    "{:.1}-{:.1} s",
                    (s.start_ns - t0) as f64 * 1e-9,
                    (s.end_ns - t0) as f64 * 1e-9
                )
            })
            .collect();
        println!(
            "  jig segments (offset constant within each): {}",
            segs.join(", ")
        );
        println!(
            "  lag: {lag:.1} ms (published pose vs truth, moving; negative: the Touch runs ahead \
             of the reference)"
        );
        if let Some(advice) = eval::latency_advice(lag, tf.latency_adjustment_ms)
            .filter(|_| advise && published_by_daemon)
        {
            println!("  {advice}");
        }
        println!("  moving, as published: position median {:.1} p90 {:.1} p99 {:.1} max {:.1} mm (rms {:.1}); rotation median {:.2} p90 {:.2} deg ({} poses)", e0.pos_median_mm, e0.pos_p90_mm, e0.pos_p99_mm, e0.pos_max_mm, e0.pos_rms_mm, e0.rot_median_deg, e0.rot_p90_deg, e0.n);
        println!("  moving, lag removed:  position median {:.1} p90 {:.1} p99 {:.1} max {:.1} mm (rms {:.1}); rotation median {:.2} p90 {:.2} deg", el.pos_median_mm, el.pos_p90_mm, el.pos_p99_mm, el.pos_max_mm, el.pos_rms_mm, el.rot_median_deg, el.rot_p90_deg);
        println!(
            "  hops (error jumps > 5 mm between consecutive poses): {hops} of {n} ({:.1}%)",
            100.0 * hops as f64 / n.max(1) as f64
        );
        for h in [0.0, 20.0, 40.0] {
            let p = eval::prediction(&moving, &tr, h);
            println!("  predicted {h:>2.0} ms ahead with published velocities: position median {:.1} p90 {:.1} mm; rotation median {:.2} deg", p.pos_median_mm, p.pos_p90_mm, p.rot_median_deg);
        }
        for (label, set) in [("still 1", &still1), ("still 2", &still2)] {
            let (rms, max) = eval::jitter(set);
            let hf = eval::jitter_hf(set, 100_000_000);
            let e = eval::errors(set, &tr, 0);
            println!("  {label} (truth still): shake {hf:.2} mm rms (vs 100 ms mean); spread rms {rms:.2} mm max {max:.2} mm; error vs truth median {:.1} mm, {:.2} deg ({} poses)", e.pos_median_mm, e.rot_median_deg, set.len());
        }
        lag
    };
    let installed = reg.lighthouse_from_world.iso();
    let lag = report(
        "with the installed registration (what SteamVR shows)",
        &installed,
        true,
    );
    // best fit to this capture on its longest jig segment, the lag removed: tracking error alone
    let segs = eval::offset_segments(
        pubs,
        &truth,
        &installed,
        (lag * 1e6) as i64,
        &eval::SegmentParams::default(),
    );
    if let Some(longest) = segs.iter().max_by_key(|s| s.end_ns - s.start_ns) {
        let pairs: Vec<_> = pubs
            .iter()
            .filter(|p| p.t_ns >= longest.start_ns && p.t_ns <= longest.end_ns)
            .filter_map(|p| {
                Some((
                    eval::interpolate(&truth, (p.t_ns as i64 - (lag * 1e6) as i64).max(0) as u64)?,
                    p.pose,
                ))
            })
            .collect();
        match register::solve(
            &pairs,
            &register::Options {
                min_pairs: 500,
                ..register::Options::default()
            },
        ) {
            Ok(sol) => {
                let d = sol.lighthouse_from_world.inverse() * installed;
                println!(
                    "(best-fit registration differs from the installed one by {:.1} mm, {:.2} deg)",
                    d.translation.vector.norm() * 1e3,
                    d.rotation.angle().to_degrees()
                );
                report(
                    "with a best-fit registration (tracking error alone)",
                    &sol.lighthouse_from_world,
                    false,
                );
            }
            Err(e) => println!("(no best-fit registration: {e})"),
        }
    }
    0
}

fn consistency_cmd(rec: &Path, cal: &Path) -> i32 {
    let result = capture::load(rec).and_then(|c| consistency::from_capture(&c, &file::load(cal)?));
    match result {
        Ok(c) => {
            println!("{c}");
            0
        }
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let code = match args.iter().map(String::as_str).collect::<Vec<_>>().as_slice() {
        ["setup"] => guided_setup::run(false, false),
        ["setup", "--no-slip-check"] => {
            guided_setup::SLIP_CHECK.store(false, Ordering::Relaxed);
            guided_setup::run(false, false)
        }
        ["setup", "--check"] => guided_setup::run(true, false),
        ["setup", "--check", "--json"] | ["setup", "--json", "--check"] => guided_setup::run(true, true),
        ["status"] => status_board(shm::DEFAULT_NAME),
        ["status", "--shm", name] => status_board(name),
        ["status", "--json"] => json_status(),
        ["inputs", "--json"] => raw_inputs_json(),
        ["inputs", "reload"] => send("inputs reload"),
        ["--help"] | ["-h"] | ["help"] => usage_code(0),
        ["--version"] | ["-V"] => { println!("cv1touch {} (shm ABI {})", env!("CARGO_PKG_VERSION"), cv1touch::ffi::CV1T_SHM_VERSION); 0 },
        ["doctor"] => doctor(false),
        ["doctor", "--json"] => doctor(true),
        ["alignment", "reload"] => send("alignment reload"),
        ["stop"] => stop(),
        ["report"] => report::run(None),
        ["sensors"] => sensors::status(),
        ["sensors", "install"] => sensors::install(),
        ["sensors", "remove"] => sensors::remove(),
        ["report", "--out", path] => report::run(Some(PathBuf::from(path))),
        ["calibration", "capture"] => send("calibration capture"),
        ["calibration", "provisional"] => send("calibration provisional"),
        ["calibration", "load", path] => {
            let abs = std::path::absolute(PathBuf::from(path)).unwrap_or_else(|_| PathBuf::from(path));
            send_path(control::calibration_load_line(&abs))
        }
        ["haptic", hand, d, f, a] => send(&format!("haptic {hand} {d} {f} {a}")),
        ["test-inject"] => send("test-inject"),
        ["test-inject", on] => send(&format!("test-inject {on}")),
        ["inject", hand, inputs, ms] => send(&format!("inject {hand} {inputs} {ms}")),
        ["record", "start", path] => {
            let abs = std::path::absolute(PathBuf::from(path)).unwrap_or_else(|_| PathBuf::from(path));
            send_path(control::record_start_line(&abs, false))
        }
        ["record", "start", path, "--frames"] => {
            let abs = std::path::absolute(PathBuf::from(path)).unwrap_or_else(|_| PathBuf::from(path));
            send_path(control::record_start_line(&abs, true))
        }
        ["record", "stop"] => send("record stop"),
        ["rec-summary", path, rest @ ..] => {
            let mut windows = Vec::new();
            let mut it = rest.iter();
            while let Some(flag) = it.next() {
                if *flag != "--window" {
                    usage();
                }
                match it.next().map(|w| summary::parse_window(w)) {
                    Some(Ok(w)) => windows.push(w),
                    Some(Err(e)) => {
                        eprintln!("cv1touch: {e}");
                        process::exit(2);
                    }
                    None => usage(),
                }
            }
            rec_summary(Path::new(path), &windows)
        }
        ["calibrate", "inputs"] => calibrate_inputs(None, false),
        ["calibrate", "inputs", "--out", out] => calibrate_inputs(Some(Path::new(out)), false),
        ["calibrate", "inputs", "--reset"] => calibrate_inputs(None, true),
        ["calibrate"] => calibrate(&file::default_path()),
        ["calibrate", "--out", out] => calibrate(Path::new(out)),
        ["calibrate", "solve", rec, rest @ ..] => {
            let (mut out, mut force) = (file::default_path(), false);
            let mut it = rest.iter();
            while let Some(flag) = it.next() {
                match *flag {
                    "--out" => out = PathBuf::from(*it.next().unwrap_or_else(|| usage())),
                    "--force" => force = true,
                    _ => usage(),
                }
            }
            calibrate_solve(Path::new(rec), &out, force)
        }
        ["calibrate", "rig", base] => calibrate_rig(Path::new(base), false, None),
        ["calibrate", "rig", base, "--write"] => calibrate_rig(Path::new(base), true, None),
        ["calibrate", "rig", base, "--out-dir", dir] => calibrate_rig(Path::new(base), true, Some(Path::new(dir))),
        ["calibrate", "place", serial, rec, rest @ ..] => {
            let (mut cal, mut write) = (file::default_path(), false);
            let mut it = rest.iter();
            while let Some(flag) = it.next() {
                match *flag {
                    "--calibration" => cal = PathBuf::from(*it.next().unwrap_or_else(|| usage())),
                    "--write" => write = true,
                    _ => usage(),
                }
            }
            calibrate_place(serial, &Path::new(rec).with_extension("rec"), &cal, write)
        }
        ["calibrate", "gravity", rec, rest @ ..] => {
            let (mut cal, mut write) = (file::default_path(), false);
            let mut it = rest.iter();
            while let Some(flag) = it.next() {
                match *flag {
                    "--calibration" => cal = PathBuf::from(*it.next().unwrap_or_else(|| usage())),
                    "--write" => write = true,
                    _ => usage(),
                }
            }
            calibrate_gravity(&Path::new(rec).with_extension("rec"), &cal, write)
        }
        ["calibrate", "refine", rec, rest @ ..] => {
            let (mut cal, mut write) = (file::default_path(), false);
            let mut it = rest.iter();
            while let Some(flag) = it.next() {
                match *flag {
                    "--calibration" => cal = PathBuf::from(*it.next().unwrap_or_else(|| usage())),
                    "--write" => write = true,
                    _ => usage(),
                }
            }
            calibrate_refine(Path::new(rec), &cal, write)
        }
        ["register", "--headset"] => register_headset(),
        ["register"] => register_cmd(false, false),
        ["register", "--force"] => register_cmd(false, true),
        ["register", "--check"] => register_cmd(true, false),
        ["landmarks", "learn", rec] => landmarks_learn(Path::new(rec), false),
        ["landmarks", "learn", rec, "--write"] => landmarks_learn(Path::new(rec), true),
        ["landmarks", "check", rec] => landmarks_check(Path::new(rec)),
        ["register", "--jig"] => register_jig(),
        ["register", "--holds", base] => register_holds(Path::new(base), false, None),
        ["register", "--holds", base, "--write"] => register_holds(Path::new(base), true, None),
        ["register", "--holds", base, "--reference", r] => register_holds(Path::new(base), false, Some(r)),
        ["register", "--holds", base, "--reference", r, "--write"] => register_holds(Path::new(base), true, Some(r)),
        ["register", "--follow", base, "--reference", r] => register_follow(Path::new(base), false, Some(r)),
        ["register", "--follow", base, "--reference", r, "--write"] => register_follow(Path::new(base), true, Some(r)),
        ["register", "--stations", rec] => register_stations(Path::new(rec), false, None),
        ["register", "--stations", rec, "--write"] => register_stations(Path::new(rec), true, None),
        ["register", "--stations", rec, "--out", out] => register_stations(Path::new(rec), true, Some(Path::new(out))),
        ["register", "--solve", file] => register_solve_file(Path::new(file), false),
        ["register", "--solve", file, "--force"] => register_solve_file(Path::new(file), true),
        ["steamvr", "install"] => steamvr_install(None),
        ["steamvr", "install", "--legacy-assets", so, manifest] => steamvr_install(Some((Path::new(so), Path::new(manifest)))),
        ["steamvr", "check"] => steamvr_check(None),
        ["steamvr", "check", "--legacy-assets", so, manifest] => steamvr_check(Some((Path::new(so), Path::new(manifest)))),
        ["steamvr", "uninstall"] => steamvr_uninstall(),
        ["eval-capture"] => eval_capture(),
        ["input-test"] => input_test(),
        ["eval-capture", "--free"] => eval_capture_free(180),
        ["eval-capture", "--free", secs] => eval_capture_free(secs.parse().unwrap_or_else(|_| usage())),
        ["eval-capture", "--free", secs, "--frames"] => match capture_free_with(secs.parse().unwrap_or_else(|_| usage()), "Switch the controllers on, then move the jig through the play space: reach, swing, turn, fast and slow.", true) {
            Ok(_) => 0,
            Err(code) => code,
        },
        ["eval", base] => eval_cmd(Path::new(base)),
        ["track", base, rest @ ..] => track_cmd(Path::new(base), rest),
        ["eval-rig", base] => eval_rig(Path::new(base), false),
        ["eval-holds", base] => eval_holds(Path::new(base)),
        ["eval-rig", base, "--replay"] => eval_rig(Path::new(base), true),
        ["vr-devices"] => vr_devices(),
        ["universe"] => universe_cmd(),
        ["shaping"] => send("shaping"),
        ["shaping", on @ ("on" | "off")] => send(&format!("shaping {on}")),
        ["universe-watch"] => send("universe-watch"),
        ["universe-watch", on @ ("on" | "off")] => send(&format!("universe-watch {on}")),
        ["latency"] => send("latency"),
        ["latency", ms] => send(&format!("latency {ms}")),
        ["prediction"] => send("prediction"),
        ["prediction", percent] => send(&format!("prediction {percent}")),
        ["prediction", which @ ("position" | "rotation"), percent] => {
            send(&format!("prediction {which} {percent}"))
        }
        ["vr-pulse", index] => vr_pulse(index, "300"),
        ["vr-pulse", index, ms] => vr_pulse(index, ms),
        ["fit-render"] => fit_render(None),
        ["fit-render", rec] => fit_render(Some(Path::new(rec))),
        ["consistency", rec] => consistency_cmd(Path::new(rec), &file::default_path()),
        ["consistency", rec, "--calibration", cal] => consistency_cmd(Path::new(rec), Path::new(cal)),
        ["pace", phases @ ..] if !phases.is_empty() => pace(&phases.iter().map(|s| s.to_string()).collect::<Vec<_>>()),
        _ => usage(),
    };
    process::exit(code);
}
#[cfg(test)]
mod tests {
    use super::*;

    /// `doctor`'s sensor_poses check reads the daemon's pose watch: moved beats placed, given up
    /// beats moved, and unsure beats all.
    #[test]
    fn doctor_tells_what_the_pose_watch_found() {
        let status = |unsure: bool, states: &[Option<&str>]| {
            serde_json::json!({
                "sensors_unsure": unsure,
                "sensors": states.iter().enumerate().map(|(i, st)| {
                    serde_json::json!({"serial": format!("S{i}"), "pose": st.map(|s| serde_json::json!({"state": s}))})
                }).collect::<Vec<_>>()
            })
        };
        assert_eq!(
            sensor_poses_check(&status(false, &[None, None])),
            (true, "2 sensor(s) stand where calibrated".to_string())
        );
        let (ok, detail) = sensor_poses_check(&status(false, &[None, Some("placed")]));
        assert!(ok && detail.contains("S1 placed again"), "{detail}");
        let (ok, detail) = sensor_poses_check(&status(false, &[Some("released"), None]));
        assert!(
            ok && detail.contains("S0 judged moved by mistake"),
            "{detail}"
        );
        let (ok, detail) = sensor_poses_check(&status(false, &[Some("placed"), Some("moved")]));
        assert!(!ok && detail.starts_with("S1 moved"), "{detail}");
        let (ok, detail) = sensor_poses_check(&status(false, &[Some("moved"), Some("given_up")]));
        assert!(!ok && detail.starts_with("S1 could not"), "{detail}");
        let (ok, detail) = sensor_poses_check(&status(true, &[Some("given_up")]));
        assert!(!ok && detail.starts_with("fewer than two"), "{detail}");
    }

    /// `stop` waits for the daemon to end: true once it has, false when the time is up while it
    /// still runs. The first look comes before any sleep, so a zero timeout still looks once.
    #[test]
    fn stop_waits_until_the_daemon_has_ended_or_the_time_is_up() {
        let mut looks = 0;
        let gone_at_third = || {
            looks += 1;
            looks < 3
        };
        assert!(ended(
            gone_at_third,
            Duration::from_secs(5),
            Duration::from_millis(1)
        ));
        assert_eq!(looks, 3);
        let mut looks = 0;
        let forever = || {
            looks += 1;
            true
        };
        assert!(!ended(
            forever,
            Duration::from_millis(20),
            Duration::from_millis(5)
        ));
        assert!(looks >= 2, "{looks}");
        assert!(ended(|| false, Duration::ZERO, Duration::from_secs(1)));
    }

    /// A lock file is `locked` while a process holds it, and a missing one is not.
    #[test]
    fn a_lock_held_by_the_daemon_is_seen() {
        let dir = std::env::temp_dir().join(format!("cv1t-stop-lock-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("daemon.lock");
        assert!(!locked(&path));
        let held = control::lock_single_instance(&path).unwrap();
        assert!(locked(&path));
        drop(held);
        assert!(!locked(&path));
        assert!(!locked(&dir.join("no-such-dir").join("daemon.lock")));
        std::fs::remove_dir_all(dir).unwrap();
    }

    fn raw_hand(trigger: f32, grip: f32, stick: [f32; 2]) -> RawInput {
        RawInput {
            buttons: Some(0),
            identity: "1".repeat(32),
            connected: true,
            received_host_ns: 1,
            trigger,
            grip,
            stick,
        }
    }

    #[test]
    fn only_enter_starts_a_step_and_quit_works_in_any_case() {
        assert_eq!(start_line("\n"), Some(true));
        assert_eq!(start_line("  "), Some(true));
        for quit in ["q", "Q", "quit", "EXIT\n"] {
            assert_eq!(start_line(quit), Some(false), "{quit}");
        }
        for again in ["n", "no", "yes", "qq", "x"] {
            assert_eq!(start_line(again), None, "{again}");
        }
    }

    #[test]
    fn each_input_step_knows_when_a_hand_is_in_position() {
        let rest = raw_hand(0.02, 0.05, [0.03, -0.04]);
        let triggers = raw_hand(0.93, 0.1, [0.0, 0.0]);
        let grips = raw_hand(0.1, 0.9, [0.0, 0.0]);
        let edge = raw_hand(0.0, 0.0, [0.55, 0.6]);
        assert!(CalStep::Rest.in_position(&rest) && !CalStep::Rest.in_position(&triggers));
        assert!(CalStep::Triggers.in_position(&triggers) && !CalStep::Triggers.in_position(&grips));
        assert!(CalStep::Grips.in_position(&grips) && !CalStep::Grips.in_position(&triggers));
        assert!(CalStep::Sticks.in_position(&edge) && !CalStep::Sticks.in_position(&rest));
        let mut asleep = triggers.clone();
        asleep.connected = false;
        assert!(
            !CalStep::Triggers.in_position(&asleep),
            "a sleeping controller is not in position"
        );
    }

    #[test]
    fn a_step_starts_once_both_hands_hold_position_long_enough() {
        let mut gate = InPosition::default();
        let both = |t, g| RawInputs {
            hands: [raw_hand(t, g, [0.; 2]), raw_hand(t, g, [0.; 2])],
        };
        let one = RawInputs {
            hands: [raw_hand(0.95, 0., [0.; 2]), raw_hand(0.1, 0., [0.; 2])],
        };
        assert!(!gate.update(CalStep::Triggers, &one, 0));
        assert!(!gate.update(CalStep::Triggers, &both(0.95, 0.), 100_000_000));
        assert!(
            !gate.update(CalStep::Triggers, &both(0.95, 0.), 300_000_000),
            "held only 200 ms"
        );
        assert!(
            !gate.update(CalStep::Triggers, &one, 350_000_000),
            "a dropout restarts the hold"
        );
        assert!(!gate.update(CalStep::Triggers, &both(0.9, 0.), 400_000_000));
        assert!(gate.update(CalStep::Triggers, &both(0.9, 0.), 700_000_000));
    }
    use cv1touch::shm::ControllerState;

    #[test]
    fn guided_start_requires_ack_and_reconciles_ambiguous_failures() {
        for response in [
            Err("timeout".to_owned()),
            Ok(String::new()),
            Ok("unexpected".into()),
        ] {
            let stops = std::cell::Cell::new(0);
            assert!(guided_record_start_with(
                || false,
                || response,
                || {
                    stops.set(stops.get() + 1);
                    Ok(())
                }
            )
            .is_err());
            assert_eq!(stops.get(), 1);
        }
        let stops = std::cell::Cell::new(0);
        assert!(guided_record_start_with(
            || false,
            || Ok("err already recording".into()),
            || {
                stops.set(1);
                Ok(())
            }
        )
        .is_err());
        assert_eq!(stops.get(), 0);
        assert!(guided_record_start_with(
            || false,
            || Ok("ok".into()),
            || panic!("acknowledged start needs no reconciliation")
        )
        .is_ok());
        assert!(guided_record_start_with(
            || false,
            || Ok(String::new()),
            || Err("stop failed".into())
        )
        .unwrap_err()
        .contains("stop failed"));
    }
    #[test]
    fn cancelled_guided_dispatch_never_starts_or_stops_recording() {
        let requests = std::cell::Cell::new(0);
        let cleanup = std::cell::Cell::new(0);
        let result = guided_record_start_with(
            || true,
            || {
                requests.set(1);
                Ok("ok".into())
            },
            || {
                cleanup.set(1);
                Ok(())
            },
        );
        assert!(result.is_err());
        assert_eq!(requests.get(), 0);
        assert_eq!(cleanup.get(), 0);
    }
    #[test]
    fn scoped_cleanup_accepts_stopped_or_absent_but_not_empty_or_rejection() {
        assert!(scoped_cleanup_reply(
            "ok stopped records=1 dropped=0 write_errors=0 bytes=48"
        ));
        assert!(scoped_cleanup_reply("ok not-matching"));
        for reply in ["", "ok", "err failed", "ok stopping", "unexpected"] {
            assert!(!scoped_cleanup_reply(reply));
        }
    }
    /// Review fix 1: samples from before Enter (reading the briefing, picking things up) or after the
    /// final beep do not enter the solve.
    #[test]
    fn wizard_recording_path_is_a_single_control_argument() {
        let root = if cfg!(windows) { "C:/tmp" } else { "/tmp" };
        let state = PathBuf::from(format!(
            "{root}/custom state directory/quoted \"name\"\nstate"
        ));
        let recording = state.join("cv1touch/calib-123.rec");
        let line = control::record_start_line(&recording, false).unwrap();
        assert!(!line.contains('\n'));
        assert_eq!(
            control::parse(&line).unwrap(),
            control::Command::RecordStart(recording, false)
        );
        let calibration = state.join("cv1touch/calibration.json");
        let line = control::calibration_load_line(&calibration).unwrap();
        assert_eq!(
            control::parse(&line).unwrap(),
            control::Command::CalibrationLoad(calibration)
        );
    }

    #[test]
    fn gravity_files_are_unchanged_when_second_file_cannot_stage() {
        let dir = std::env::temp_dir().join(format!("cv1t-gravity-stage-{}", process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let cal = dir.join("calibration.json");
        let blocker = dir.join("not-a-directory");
        std::fs::write(&cal, b"old calibration").unwrap();
        std::fs::write(&blocker, b"block").unwrap();
        assert!(save_gravity_files(
            &cal,
            b"new calibration",
            Some((&blocker.join("registration.json"), b"new registration"))
        )
        .is_err());
        assert_eq!(std::fs::read(&cal).unwrap(), b"old calibration");
        assert_eq!(std::fs::read_dir(&dir).unwrap().count(), 2);
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn gravity_files_save_both_and_keep_previous_versions() {
        let dir = std::env::temp_dir().join(format!("cv1t-gravity-pair-{}", process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let cal = dir.join("calibration.json");
        let reg = dir.join("registration.json");
        std::fs::write(&cal, b"old calibration").unwrap();
        std::fs::write(&reg, b"old registration").unwrap();
        save_gravity_files(&cal, b"new calibration", Some((&reg, b"new registration"))).unwrap();
        assert_eq!(std::fs::read(&cal).unwrap(), b"new calibration");
        assert_eq!(std::fs::read(&reg).unwrap(), b"new registration");
        let backups: Vec<_> = std::fs::read_dir(&dir)
            .unwrap()
            .filter_map(|e| e.ok())
            .filter(|e| e.file_name().to_string_lossy().contains(".bak-"))
            .map(|e| std::fs::read(e.path()).unwrap())
            .collect();
        assert!(
            backups.contains(&b"old calibration".to_vec())
                && backups.contains(&b"old registration".to_vec())
        );
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn human_color_policy_respects_no_color_dumb_and_pipes() {
        assert!(human_color_enabled(false, false, true, false));
        assert!(human_color_enabled(false, false, false, true));
        assert!(!human_color_enabled(false, false, false, false));
        assert!(!human_color_enabled(true, false, true, true));
        assert!(!human_color_enabled(false, true, true, true));
    }
    fn raw_input_snapshot(timestamp: u64) -> RawInputs {
        RawInputs {
            hands: std::array::from_fn(|hand| RawInput {
                buttons: None,
                identity: format!("{:032x}", hand + 1),
                connected: true,
                received_host_ns: timestamp,
                trigger: 0.0,
                grip: 0.0,
                stick: [0.0; 2],
            }),
        }
    }

    #[test]
    fn paired_input_capture_tracks_each_hand_and_phase_independently() {
        let mut readings = raw_input_snapshot(1_000_000_000);
        let identities = std::array::from_fn(|hand| readings.hands[hand].identity.clone());
        let mut phase = PairedInputPhase::new([0; 2]);
        phase
            .observe(&readings, &identities, 1_000_000_010)
            .unwrap();
        readings.hands[0].received_host_ns += 1;
        phase
            .observe(&readings, &identities, 1_000_000_010)
            .unwrap();
        readings.hands[1].received_host_ns += 2;
        phase
            .observe(&readings, &identities, 1_000_000_010)
            .unwrap();
        // A duplicate response contributes neither hand; independent advances contribute one.
        phase
            .observe(&readings, &identities, 1_000_000_010)
            .unwrap();
        let last = phase.last_ns();
        let samples = phase.finish(2).unwrap();
        assert_eq!(
            samples[0]
                .iter()
                .map(|s| s.received_host_ns)
                .collect::<Vec<_>>(),
            [1_000_000_000, 1_000_000_001]
        );
        assert_eq!(
            samples[1]
                .iter()
                .map(|s| s.received_host_ns)
                .collect::<Vec<_>>(),
            [1_000_000_000, 1_000_000_002]
        );
        let mut next = PairedInputPhase::new(last);
        next.observe(&readings, &identities, 1_000_000_010).unwrap();
        assert!(
            next.finish(1).is_err(),
            "prior phase timestamps must not count again"
        );
    }

    #[test]
    fn paired_input_capture_rejects_invalid_peer_despite_fresh_other_hand() {
        for bad_hand in 0..2 {
            for fault in 0..6 {
                let mut readings = raw_input_snapshot(1_000_000_000);
                let identities = std::array::from_fn(|hand| readings.hands[hand].identity.clone());
                let mut phase = PairedInputPhase::new([0; 2]);
                phase
                    .observe(&readings, &identities, 1_000_000_001)
                    .unwrap();
                readings.hands[1 - bad_hand].received_host_ns += 100;
                match fault {
                    0 => readings.hands[bad_hand].connected = false,
                    1 => readings.hands[bad_hand].received_host_ns = 400_000_000,
                    2 => readings.hands[bad_hand].identity = "f".repeat(32),
                    3 => readings.hands[bad_hand].received_host_ns -= 1,
                    4 => readings.hands[bad_hand].received_host_ns = 1_000_000_200,
                    _ => {
                        readings.hands[bad_hand].received_host_ns += 1;
                        readings.hands[bad_hand].trigger = f32::NAN;
                    }
                }
                assert!(
                    phase
                        .observe(&readings, &identities, 1_000_000_100)
                        .is_err(),
                    "hand {bad_hand}, fault {fault}"
                );
            }
        }
    }

    #[test]
    fn paired_input_capture_requires_minimum_samples_from_both_hands() {
        for slower in 0..2 {
            let mut readings = raw_input_snapshot(1_000_000_000);
            let identities = std::array::from_fn(|hand| readings.hands[hand].identity.clone());
            let mut phase = PairedInputPhase::new([0; 2]);
            phase
                .observe(&readings, &identities, 1_000_000_001)
                .unwrap();
            readings.hands[1 - slower].received_host_ns += 1;
            phase
                .observe(&readings, &identities, 1_000_000_002)
                .unwrap();
            assert!(phase.finish(2).is_err());
        }
    }

    #[test]
    fn input_wizard_rejects_stale_duplicate_and_swapped_controller_samples() {
        let mut readings = raw_input_snapshot(1_000_000_000);
        let identities = std::array::from_fn(|hand| readings.hands[hand].identity.clone());
        let mut phase = InputPhase::default();
        assert!(phase
            .observe(&readings, &identities, 0, 1_000_000_001)
            .unwrap());
        assert!(!phase
            .observe(&readings, &identities, 0, 1_000_000_002)
            .unwrap());
        assert_eq!(phase.samples.len(), 1);
        assert!(phase
            .observe(&readings, &identities, 0, 1_600_000_000)
            .is_err());
        readings.hands[1].identity = "f".repeat(32);
        assert!(phase
            .observe(&readings, &identities, 0, 1_000_000_003)
            .is_err());
        readings.hands[1].identity = identities[1].clone();
        readings.hands[0].received_host_ns -= 1;
        assert!(phase
            .observe(&readings, &identities, 0, 1_000_000_003)
            .is_err());
        readings.hands[0].connected = false;
        assert!(phase
            .observe(&readings, &identities, 0, 1_000_000_003)
            .is_err());
        assert!(phase.finish(30).is_err());
    }

    #[test]
    fn input_wizard_rejects_null_axes_and_future_or_empty_identity() {
        let readings = raw_input_snapshot(100);
        let identities = std::array::from_fn(|hand| readings.hands[hand].identity.clone());
        let mut json = serde_json::json!({"hands": readings.hands});
        json["hands"][0]["grip"] = serde_json::Value::Null;
        assert!(parse_raw_inputs(&format!("ok {json}")).is_err());
        assert!(InputPhase::default()
            .observe(&readings, &identities, 0, 99)
            .is_err());
        let mut bad = readings;
        bad.hands[0].identity.clear();
        assert!(InputPhase::default()
            .observe(&bad, &identities, 0, 101)
            .is_err());
    }

    fn input_wizard_phases(hand: usize) -> [Vec<RawInput>; 4] {
        std::array::from_fn(|phase| {
            (0..if phase == 3 { 200 } else { 40 })
                .map(|i| {
                    let mut sample = raw_input_snapshot(100 + i as u64).hands[hand].clone();
                    match phase {
                        1 => sample.trigger = 1.0,
                        2 => sample.grip = 1.0,
                        3 => {
                            let angle = i as f32 * std::f32::consts::TAU / 200.0;
                            sample.stick = [angle.cos(), angle.sin()];
                        }
                        _ => {}
                    }
                    sample
                })
                .collect()
        })
    }

    fn input_wizard_candidate() -> cv1touch::input_calibration::CalibrationFile {
        cv1touch::input_calibration::CalibrationFile::new(std::array::from_fn(|hand| {
            measured_inputs(&format!("{:032x}", hand + 1), &input_wizard_phases(hand)).unwrap()
        }))
        .unwrap()
    }

    #[test]
    fn paired_input_capture_preserves_both_measurements_through_four_phases() {
        let fixtures = [input_wizard_phases(0), input_wizard_phases(1)];
        let identities = [format!("{:032x}", 1), format!("{:032x}", 2)];
        let mut last = [0; 2];
        let mut captured: [[Vec<RawInput>; 4]; 2] =
            std::array::from_fn(|_| std::array::from_fn(|_| Vec::new()));
        let mut timestamp = 1_000_000_000;
        for phase_index in 0..4 {
            let mut phase = PairedInputPhase::new(last);
            for (left, right) in fixtures[0][phase_index]
                .iter()
                .zip(&fixtures[1][phase_index])
            {
                timestamp += 10;
                let mut readings = RawInputs {
                    hands: [left.clone(), right.clone()],
                };
                for (hand, sample) in readings.hands.iter_mut().enumerate() {
                    sample.received_host_ns = timestamp + hand as u64;
                }
                // Distinct right-hand endpoints must not be replaced with left-hand data.
                readings.hands[1].trigger *= 0.8;
                readings.hands[1].grip *= 0.9;
                phase
                    .observe(&readings, &identities, timestamp + 2)
                    .unwrap();
            }
            last = phase.last_ns();
            for (hand, samples) in phase
                .finish(if phase_index == 3 { 150 } else { 30 })
                .unwrap()
                .into_iter()
                .enumerate()
            {
                captured[hand][phase_index] = samples;
            }
        }
        let candidate =
            cv1touch::input_calibration::CalibrationFile::new(std::array::from_fn(|hand| {
                measured_inputs(&identities[hand], &captured[hand]).unwrap()
            }))
            .unwrap();
        assert_eq!(candidate.hands[0].trigger.pressed, 1.0);
        assert_eq!(candidate.hands[1].trigger.pressed, 0.8);
        assert_eq!(candidate.hands[0].grip.pressed, 1.0);
        assert_eq!(candidate.hands[1].grip.pressed, 0.9);
        for bad_hand in 0..2 {
            for bad_phase in 1..4 {
                let mut incomplete = captured.clone();
                for sample in &mut incomplete[bad_hand][bad_phase] {
                    sample.trigger = 0.0;
                    sample.grip = 0.0;
                    sample.stick = [0.0; 2];
                }
                assert!(
                    cv1touch::input_calibration::CalibrationFile::new(std::array::from_fn(
                        |hand| measured_inputs(&identities[hand], &incomplete[hand]).unwrap()
                    ))
                    .is_err(),
                    "hand {bad_hand}, phase {bad_phase} lacks required travel"
                );
            }
        }
    }

    #[test]
    fn input_wizard_uses_hold_medians_and_circle_quantiles_not_single_spikes() {
        let mut phases = input_wizard_phases(0);
        phases[1][0].trigger = -4.0;
        phases[2][0].grip = 4.0;
        phases[3][0].stick = [-4.0, -4.0];
        phases[3][1].stick = [4.0, 4.0];
        let hand = measured_inputs(&"1".repeat(32), &phases).unwrap();
        assert_eq!(hand.trigger.pressed, 1.0);
        assert_eq!(hand.grip.pressed, 1.0);
        for axis in hand.stick {
            assert!(axis.min >= -1.0 && axis.min < -0.9);
            assert!(axis.max <= 1.0 && axis.max > 0.9);
            assert_eq!(axis.center, 0.0);
        }
        let mut incomplete = phases;
        for sample in &mut incomplete[3] {
            sample.stick = [0.3, 0.3];
        }
        let hand = measured_inputs(&"1".repeat(32), &incomplete).unwrap();
        assert!(cv1touch::input_calibration::CalibrationFile::new([
            hand,
            input_wizard_candidate().hands[1].clone()
        ])
        .is_err());
    }

    #[test]
    fn input_wizard_save_and_reset_restore_only_input_file_on_reload_failure() {
        let dir = std::env::temp_dir().join(format!("cv1t-input-transaction-{}", process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("input-calibration.json");
        let unrelated = dir.join("calibration.json");
        std::fs::write(&unrelated, b"camera configuration").unwrap();
        let original = b"original input bytes";
        std::fs::write(&path, original).unwrap();
        let candidate = input_wizard_candidate();
        let mut reject_reload = || Err("mock reload rejected".into());
        assert!(
            write_input_calibration(&path, Some(&candidate), Some(&mut reject_reload)).is_err()
        );
        assert_eq!(std::fs::read(&path).unwrap(), original);
        assert_eq!(std::fs::read(&unrelated).unwrap(), b"camera configuration");
        assert!(write_input_calibration(&path, None, Some(&mut reject_reload)).is_err());
        assert_eq!(std::fs::read(&path).unwrap(), original);
        assert!(std::fs::read_dir(&dir)
            .unwrap()
            .filter_map(Result::ok)
            .any(|entry| {
                entry.file_name().to_string_lossy().contains(".bak-")
                    && std::fs::read(entry.path()).unwrap() == original
            }));
        write_input_calibration(&path, Some(&candidate), None).unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            assert_eq!(
                std::fs::metadata(&path).unwrap().permissions().mode() & 0o777,
                0o600
            );
        }
        assert_eq!(
            cv1touch::input_calibration::load(&path).unwrap(),
            Some(candidate)
        );
        write_input_calibration(&path, None, None).unwrap();
        assert!(!path.exists());
        assert_eq!(std::fs::read(&unrelated).unwrap(), b"camera configuration");
        assert!(write_input_calibration(
            &path,
            Some(&input_wizard_candidate()),
            Some(&mut reject_reload)
        )
        .is_err());
        assert!(!path.exists());
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn input_wizard_restores_malformed_previous_bytes_after_post_write_save_error() {
        let dir = std::env::temp_dir().join(format!("cv1t-input-post-write-{}", process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("input-calibration.json");
        let old = b"{ malformed old configuration";
        std::fs::write(&path, old).unwrap();
        let error = write_input_calibration_with(
            &path,
            Some(&input_wizard_candidate()),
            None,
            |path, _| {
                std::fs::write(path, b"new bytes after rename").unwrap();
                Err("injected directory fsync failure after rename".into())
            },
        )
        .unwrap_err();
        assert!(error.contains("directory fsync failure"), "{error}");
        assert_eq!(std::fs::read(&path).unwrap(), old);
        std::fs::remove_file(&path).unwrap();
        assert!(write_input_calibration_with(
            &path,
            Some(&input_wizard_candidate()),
            None,
            |path, _| {
                std::fs::write(path, b"new file after rename").unwrap();
                Err("injected post-write failure".into())
            }
        )
        .is_err());
        assert!(!path.exists());
        std::fs::remove_dir_all(dir).unwrap();
    }

    /// Unix only: a symlink takes privileges to make on Windows.
    #[cfg(unix)]
    #[test]
    fn input_wizard_never_follows_a_config_symlink_for_backup_or_reset() {
        let dir = std::env::temp_dir().join(format!("cv1t-input-symlink-{}", process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("input-calibration.json");
        let target = dir.join("unrelated.json");
        std::fs::write(&target, b"unrelated configuration").unwrap();
        std::os::unix::fs::symlink(&target, &path).unwrap();
        assert!(write_input_calibration(&path, None, None).is_err());
        assert!(write_input_calibration(&path, Some(&input_wizard_candidate()), None).is_err());
        assert_eq!(std::fs::read(&target).unwrap(), b"unrelated configuration");
        assert!(std::fs::symlink_metadata(&path)
            .unwrap()
            .file_type()
            .is_symlink());
        assert_eq!(std::fs::read_dir(&dir).unwrap().count(), 2);
        std::fs::remove_dir_all(dir).unwrap();
    }

    fn steam_input(axis: usize, x: f32, y: f32) -> vr::ControllerState {
        let mut state = (0, 0, [(0.0, 0.0); 5]);
        state.2[axis] = (x, y);
        state
    }

    #[test]
    fn input_delivery_rejects_unrelated_axis_and_wrong_direction() {
        let steps = input_steps(HAND_LEFT);
        let trigger = ControllerState {
            trigger: 1.0,
            ..Default::default()
        };
        assert!(!InputProgress::default().observe(
            &steps[0],
            Some(&trigger),
            Some(&steam_input(2, 1.0, 0.0))
        ));
        assert!(InputProgress::default().observe(
            &steps[0],
            Some(&trigger),
            Some(&steam_input(1, 1.0, 0.0))
        ));
        let grip = ControllerState {
            grip: 1.0,
            ..Default::default()
        };
        assert!(!InputProgress::default().observe(
            &steps[1],
            Some(&grip),
            Some(&steam_input(1, 1.0, 0.0))
        ));
        let forward = ControllerState {
            stick: [0.0, 1.0],
            ..Default::default()
        };
        assert!(!InputProgress::default().observe(
            &steps[2],
            Some(&forward),
            Some(&steam_input(0, 0.0, -1.0))
        ));
        assert!(!InputProgress::default().observe(
            &steps[2],
            Some(&forward),
            Some(&steam_input(0, 1.0, 0.0))
        ));
        assert!(InputProgress::default().observe(
            &steps[2],
            Some(&forward),
            Some(&steam_input(0, 0.0, 1.0))
        ));
    }

    #[test]
    fn input_delivery_accepts_held_full_grip_and_retries_driver_lag() {
        let steps = input_steps(HAND_LEFT);
        let grip = ControllerState {
            grip: 1.0,
            ..Default::default()
        };
        let mut progress = InputProgress::default();
        assert!(!progress.observe(&steps[1], Some(&grip), None));
        assert!(!progress.observe(&steps[1], Some(&grip), Some(&steam_input(2, 0.2, 0.0))));
        assert!(!progress.observe(&steps[1], Some(&grip), Some(&steam_input(2, 0.8, 0.0))));
        assert!(progress.observe(&steps[1], Some(&grip), Some(&steam_input(2, 1.0, 0.0))));
        assert!(InputProgress::default().observe(
            &steps[1],
            Some(&grip),
            Some(&steam_input(2, 1.0, 0.0))
        ));
        assert!(!InputProgress::default().observe(
            &steps[1],
            None,
            Some(&steam_input(2, 1.0, 0.0))
        ));
    }

    #[test]
    fn input_delivery_reports_partial_grip_without_relaxing_full_squeeze() {
        let steps = input_steps(HAND_LEFT);
        let mut progress = InputProgress::default();
        for value in [0.0, 0.25, 0.5, 0.9] {
            let grip = ControllerState {
                grip: value,
                ..Default::default()
            };
            assert!(!progress.observe(&steps[1], Some(&grip), Some(&steam_input(2, value, 0.0))));
        }
        let diagnostic = progress.analog_report(2);
        assert!(diagnostic.contains("daemon 0.000..0.900"), "{diagnostic}");
        assert!(diagnostic.contains("SteamVR 0.000..0.900"), "{diagnostic}");
        assert!(diagnostic.contains("> 0.9"), "{diagnostic}");
        assert!(!progress.daemon_seen);
    }

    #[test]
    fn input_delivery_reports_trigger_delivery_failure_separately_from_grip() {
        let steps = input_steps(HAND_LEFT);
        let mut progress = InputProgress::default();
        for value in [0.0, 0.5, 1.0] {
            let state = ControllerState {
                trigger: value,
                grip: 0.25,
                ..Default::default()
            };
            let mut steam = steam_input(1, 0.0, 0.0);
            steam.2[2].0 = 0.75;
            assert!(!progress.observe(&steps[0], Some(&state), Some(&steam)));
        }
        let diagnostic = progress.analog_report(1);
        assert!(
            diagnostic.contains("trigger range: daemon 0.000..1.000"),
            "{diagnostic}"
        );
        assert!(diagnostic.contains("SteamVR 0.000..0.000"), "{diagnostic}");
        assert!(progress.daemon_seen);
        assert!(!progress.steam_seen);
    }

    #[test]
    fn input_delivery_requires_exact_button_and_unpressed_touch() {
        let steps = input_steps(HAND_RIGHT);
        let a = ControllerState {
            buttons: cv1touch::hand::BTN_A,
            ..Default::default()
        };
        assert!(!InputProgress::default().observe(
            &steps[7],
            Some(&a),
            Some(&(1 << 1, 0, [(0.0, 0.0); 5]))
        ));
        assert!(InputProgress::default().observe(
            &steps[7],
            Some(&a),
            Some(&(1 << 7, 0, [(0.0, 0.0); 5]))
        ));
        let touch = ControllerState {
            touches: cv1touch::hand::TOUCH_A,
            ..Default::default()
        };
        assert!(!InputProgress::default().observe(
            &steps[9],
            Some(&touch),
            Some(&(1 << 7, 1 << 7, [(0.0, 0.0); 5]))
        ));
        assert!(InputProgress::default().observe(
            &steps[9],
            Some(&touch),
            Some(&(0, 1 << 7, [(0.0, 0.0); 5]))
        ));
    }

    #[test]
    fn input_delivery_marks_legacy_unavailable_steps_as_manual() {
        let steps = input_steps(HAND_LEFT);
        let thumb = ControllerState {
            touches: cv1touch::hand::TOUCH_THUMBREST,
            ..Default::default()
        };
        assert!(steps[13].steam.is_none());
        assert!(InputProgress::default().observe(&steps[13], Some(&thumb), None));
        assert!(!InputProgress::default().observe(&steps[13], None, None));
        assert!(steps[14].steam.is_none());
    }

    #[test]
    fn input_delivery_requires_both_layers() {
        assert!(!input_delivery_ok(true, false));
        assert!(!input_delivery_ok(false, true));
        assert!(input_delivery_ok(true, true));
    }

    #[test]
    fn fresh_registration_records_capture_universe() {
        let r = initial_registration(42);
        assert_eq!(r.universe_id, 42);
        assert_eq!(r.format, align::REGISTRATION_FORMAT);
        assert!(r.stations.is_empty());
    }

    #[test]
    fn cancelled_fits_never_call_the_writer_or_replace_previous_files() {
        let path = std::env::temp_dir().join(format!("cv1t-cancel-fit-{}", process::id()));
        std::fs::write(&path, b"previous configuration").unwrap();
        let result = commit_fit(true, || {
            std::fs::write(&path, b"new fit").map_err(|e| e.to_string())
        });
        assert!(result.unwrap_err().contains("not installed"));
        assert_eq!(std::fs::read(&path).unwrap(), b"previous configuration");
        commit_fit(false, || {
            std::fs::write(&path, b"accepted fit").map_err(|e| e.to_string())
        })
        .unwrap();
        assert_eq!(std::fs::read(&path).unwrap(), b"accepted fit");
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn stale_registration_does_not_seed_a_new_world_and_is_retained_until_fit() {
        use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
        let path = std::env::temp_dir().join(format!("cv1t-new-world-seed-{}", process::id()));
        std::fs::write(&path, b"malformed previous registration").unwrap();
        let (reuse, fresh) = registration_seed(&path, 42, false).unwrap();
        assert!(!reuse);
        assert_eq!(fresh.universe_id, 42);
        assert!(registration_seed(&path, 42, true).is_err());
        let world = Isometry3::from_parts(
            Translation3::new(2.0, -1.0, 3.0),
            UnitQuaternion::from_euler_angles(0.4, -0.7, 0.2),
        );
        let offset = Isometry3::translation(0.1, 0.05, 0.0);
        let timed: Vec<_> = (0..1000)
            .map(|i| {
                let t = i as f64 * 0.02;
                let reference = Isometry3::from_parts(
                    Translation3::new(0.1 * t.sin(), 1.6, 0.1 * t.cos()),
                    UnitQuaternion::from_scaled_axis(Vector3::new(
                        0.6 * t.sin(),
                        0.7 * (0.8 * t).cos(),
                        0.5 * (1.3 * t).sin(),
                    )),
                );
                (
                    i * 20_000_000,
                    reference,
                    world.inverse() * reference * offset,
                )
            })
            .collect();
        let range = register::bootstrap_span(&timed).unwrap();
        let pairs: Vec<_> = timed[range].iter().map(|p| (p.1, p.2)).collect();
        let fit = register::solve(
            &pairs,
            &register::Options {
                min_pairs: 500,
                min_axis_ratio: 0.1,
                ..Default::default()
            },
        )
        .unwrap();
        let error = fit.lighthouse_from_world.inverse() * world;
        assert!(error.translation.vector.norm() < 0.001);
        assert!(error.rotation.angle() < 0.001);
        assert_eq!(
            std::fs::read(&path).unwrap(),
            b"malformed previous registration"
        );
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn only_samples_inside_the_paced_phase_count() {
        let mk = |t| register::Sample {
            t_ns: t,
            reference: nalgebra::Isometry3::identity(),
            touch: [None, None],
        };
        let kept = phase_samples(vec![mk(5), mk(10), mk(15), mk(20), mk(25)], 10, 20);
        assert_eq!(
            kept.iter().map(|s| s.t_ns).collect::<Vec<_>>(),
            vec![10, 15, 20]
        );
    }

    /// With no Index controller on, our Touches (shown by SteamVR as controllers, and with the
    /// most poses) must not become the reference, and the headset must not be lost from the file.
    #[test]
    fn a_capture_without_a_controller_is_measured_against_the_headset() {
        let kinds: std::collections::HashMap<String, String> = [
            ("CV1T-LEFT", "touch"),
            ("CV1T-RIGHT", "touch"),
            ("LHB-1", "base"),
            ("LHR-HMD", "hmd"),
        ]
        .iter()
        .map(|(a, b)| (a.to_string(), b.to_string()))
        .collect();
        let track = |s: &str, n: usize| (s.to_string(), vec![[0.0; 8]; n]);
        let tracks = vec![
            track("CV1T-LEFT", 300),
            track("LHR-HMD", 250),
            track("LHB-1", 300),
            track("CV1T-RIGHT", 290),
        ];
        let (reference, others) = reference_first(tracks.clone(), |s| {
            kinds.get(s).cloned().unwrap_or_default()
        })
        .unwrap();
        assert_eq!(reference.0, "LHR-HMD");
        assert_eq!(reference.1.len(), 250);
        let mut rest: Vec<&str> = others.iter().map(|t| t.0.as_str()).collect();
        rest.sort();
        assert_eq!(rest, vec!["CV1T-LEFT", "CV1T-RIGHT", "LHB-1"]);
        // an Index controller, when on, is the reference, and the headset stays among the others
        let mut with_index = tracks;
        with_index.push(track("LHR-INDEX", 200));
        let mut kinds = kinds;
        kinds.insert("LHR-INDEX".into(), "controller".into());
        let (reference, others) =
            reference_first(with_index, |s| kinds.get(s).cloned().unwrap_or_default()).unwrap();
        assert_eq!(reference.0, "LHR-INDEX");
        assert!(others.iter().any(|t| t.0 == "LHR-HMD"));
        assert!(reference_first(vec![track("LHB-1", 10)], |_| "base".into()).is_none());
    }

    #[test]
    fn only_fresh_positioned_touch_states_are_sampled() {
        let ok = ControllerState {
            flags: cv1touch::ffi::CV1T_FLAG_CONNECTED | cv1touch::ffi::CV1T_FLAG_POSITION,
            last_position_fix_host_ns: 1_000_000_000,
            orient: [0.0, 0.0, 0.0, 1.0],
            ..Default::default()
        };
        assert!(touch_sample(&ok, 1_050_000_000).is_some());
        assert!(touch_sample(&ok, 1_200_000_000).is_none()); // optical fix 200 ms old
        let asleep = ControllerState {
            flags: cv1touch::ffi::CV1T_FLAG_POSITION,
            ..ok
        };
        assert!(touch_sample(&asleep, 1_050_000_000).is_none());
    }
}
