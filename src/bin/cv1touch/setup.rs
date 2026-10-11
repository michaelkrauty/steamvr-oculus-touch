// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Interactive orchestration; the library planner has no hardware side effects.
use super::*;
use cv1touch::input_calibration;
use cv1touch::setup::{self, Artifact, Checkpoint, Snapshot, Stage, State};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::io::IsTerminal;

#[path = "setup_ui.rs"]
mod ui;

#[derive(Serialize)]
struct Check {
    name: &'static str,
    ok: bool,
    detail: String,
}
struct Inspection {
    snapshot: Snapshot,
    checks: Vec<Check>,
    unplaced: Vec<String>,
    identities: Option<[String; 2]>,
    generation: Option<u64>,
    sensors: BTreeSet<String>,
    engine_ready: Option<bool>,
}

fn journal_path() -> PathBuf {
    state_dir().join("setup.json")
}

fn artifact(path: &Path, validate: impl FnOnce() -> Result<(), String>) -> Artifact {
    let first = match setup::fingerprint(path) {
        Ok(hash) => hash,
        Err(error) if error.kind() == io::ErrorKind::NotFound => return Artifact::Missing,
        Err(error) => return Artifact::Invalid(error.to_string()),
    };
    if let Err(error) = validate() {
        return Artifact::Invalid(error);
    }
    match setup::fingerprint(path) {
        Ok(hash) if hash == first => Artifact::Valid(hash),
        _ => Artifact::Invalid("file changed while inspecting it; retry".into()),
    }
}

fn read_status() -> Option<serde_json::Value> {
    control::request(
        &control::socket_path(),
        "status",
        Duration::from_millis(750),
    )
    .ok()
    .and_then(|reply| serde_json::from_str(reply.strip_prefix("ok ")?).ok())
}

fn camera_artifact(path: &Path) -> (Artifact, Option<file::CalibrationFile>) {
    let mut calibration = None;
    let camera = artifact(path, || {
        calibration = Some(file::load(path)?);
        Ok(())
    });
    if !matches!(camera, Artifact::Valid(_)) {
        calibration = None;
    }
    (camera, calibration)
}

fn inspect(checkpoint: &Checkpoint) -> Inspection {
    let (camera, calibration) = camera_artifact(&file::default_path());
    let registration = artifact(&align::registration_path(), || {
        align::load_registration(&align::registration_path()).map(|_| ())
    });
    let render = artifact(&align::render_path(), || {
        let value = align::load_render(&align::render_path())?;
        if ["left", "right"]
            .iter()
            .all(|hand| value.hands.iter().any(|h| h.hand == *hand))
        {
            Ok(())
        } else {
            Err("render offsets must contain both hands".into())
        }
    });
    let raw = raw_inputs();
    let identities = raw
        .as_ref()
        .ok()
        .map(|r| std::array::from_fn(|h| r.hands[h].identity.clone()));
    let known_identities = raw
        .as_ref()
        .ok()
        .zip(identities.as_ref())
        .is_some_and(|(r, ids)| ids[0] != ids[1] && input_identities_match(r, ids).is_ok());
    let fresh = raw
        .as_ref()
        .ok()
        .zip(identities.as_ref())
        .is_some_and(|(r, ids)| {
            PairedInputPhase::new([0, 0])
                .observe(r, ids, shm::now_ns())
                .is_ok()
        });
    let inputs = artifact(&input_calibration::default_path(), || {
        let value = input_calibration::load(&input_calibration::default_path())?
            .ok_or("input calibration is missing")?;
        if !known_identities {
            return Err("wake both controllers to verify input identities".into());
        }
        let ids = identities.as_ref().unwrap();
        if (0..2).all(|h| value.hands[h].identity == ids[h]) {
            Ok(())
        } else {
            Err("input calibration belongs to a different controller pair".into())
        }
    });
    let mut source_hash = Sha256::new();
    let source_result = (|| -> Result<(), String> {
        if !known_identities {
            return Err("wake both controllers to verify render geometry".into());
        }
        for identity in identities.as_ref().unwrap() {
            source_hash.update(identity.as_bytes());
        }
        for left in [true, false] {
            let mesh = rendermodel::steamvr_body_obj(left);
            let validated = artifact(&mesh, || {
                let text = std::fs::read_to_string(&mesh).map_err(|e| e.to_string())?;
                rendermodel::parse_obj(&text).map(|_| ())
            });
            let Artifact::Valid(hash) = validated else {
                return Err("SteamVR controller mesh is missing, invalid or changed".into());
            };
            source_hash.update(hash.as_bytes());
        }
        Ok(())
    })();
    let render_sources = match source_result {
        Ok(()) => Artifact::Valid(format!("{:x}", source_hash.finalize())),
        Err(error) => Artifact::Invalid(error),
    };
    let first = read_status();
    thread::sleep(Duration::from_millis(300));
    let second = read_status();
    let streaming = first.as_ref().zip(second.as_ref()).is_some_and(|(a, b)| {
        if a["generation"].as_u64().is_none()
            || a["generation"] != b["generation"]
            || a["mode"] != b["mode"]
        {
            return false;
        }
        let (Ok(old), Ok(new)) = (
            status::sensors_from_status(a),
            status::sensors_from_status(b),
        ) else {
            return false;
        };
        !new.is_empty()
            && old.len() == new.len()
            && new.iter().all(|sensor| {
                old.iter().any(|previous| {
                    shm::serial_str(previous) == shm::serial_str(sensor)
                        && sensor.frames > previous.frames
                })
            })
    });
    let sensors = second
        .as_ref()
        .and_then(|v| status::sensors_from_status(v).ok())
        .unwrap_or_default();
    let unplaced: Vec<_> = sensors
        .iter()
        .filter_map(|sensor| {
            let serial = shm::serial_str(sensor);
            (!calibration
                .as_ref()
                .is_some_and(|cal| cal.sensors.iter().any(|entry| entry.serial == serial)))
            .then_some(serial)
        })
        .collect();
    let recording = second.as_ref().is_some_and(|v| v["recording"] == true);
    let normal_mode = second
        .as_ref()
        .is_some_and(|v| matches!(v["mode"].as_str(), Some("calibrated" | "provisional")));
    let runtime = steamvr::runtime_root();
    let runtime_ok = runtime.join("bin/linux64/libopenvr_api.so").is_file()
        && runtime
            .join("drivers/oculus/resources/input/touch_profile.json")
            .is_file();
    let loaded = second.as_ref().is_some_and(|v| {
        v["mode"] == "calibrated"
            && !sensors.is_empty()
            && sensors.iter().all(|s| s.placed != 0)
            && v["alignment"]["registration"] == true
            && v["alignment"]["render"] == serde_json::json!([true, true])
            && v["input_calibration"]["calibrated"] == serde_json::json!([true, true])
    });
    let checks = vec![
        Check { name: "daemon", ok: second.is_some(), detail: "Start cv1touchd with access to the CV1 USB devices.".into() },
        Check { name: "sensor_streams", ok: streaming, detail: "Every connected sensor must deliver advancing frames; check USB access/bandwidth if stopped.".into() },
        Check { name: "controllers_awake", ok: fresh, detail: "Wake both Touch controllers with a face button before a capture.".into() },
        Check { name: "idle", ok: !recording && normal_mode && second.as_ref().is_none_or(|v| v["engine_ready"] != false), detail: "Wait for engine initialization/reload and finish any active recording or capture before starting the guide.".into() },
        Check { name: "steamvr_assets", ok: runtime_ok && steamvr::driver_assets().is_ok(), detail: "Install the driver and native SteamVR runtime before setup; start SteamVR before headset registration.".into() },
        Check { name: "sensor_coverage", ok: unplaced.is_empty() && !sensors.is_empty(), detail: "New connected sensors need placement; disconnected saved sensors can remain in the calibration.".into() },
        Check { name: "render_geometry", ok: matches!(render_sources, Artifact::Valid(_)), detail: "Both controller identities and installed SteamVR meshes must be valid.".into() },
        Check { name: "tracking_configuration", ok: loaded, detail: "The daemon must report camera calibration, both render offsets, registration and both input calibrations loaded. Reload configuration explicitly if saved files are ready but this check fails.".into() },
    ];
    let mut blocked = BTreeMap::new();
    for stage in [
        Stage::Camera,
        Stage::Gravity,
        Stage::Render,
        Stage::Registration,
        Stage::Inputs,
    ] {
        let reason = if second.is_none() {
            Some("daemon unavailable")
        } else if second.as_ref().is_some_and(|v| v["engine_ready"] == false) {
            Some("tracking engine is initializing or reloading; wait for it to resume")
        } else if recording || !normal_mode {
            Some("daemon is busy; finish its active operation first")
        } else if !fresh {
            Some("wake both controllers with a face button")
        } else if stage != Stage::Inputs && !streaming {
            Some("sensor frames are stopped; check USB access and bandwidth")
        } else if matches!(stage, Stage::Gravity | Stage::Registration)
            && second.as_ref().is_none_or(|v| v["mode"] != "calibrated")
        {
            Some("load the saved camera calibration before this stage")
        } else if matches!(stage, Stage::Render | Stage::Registration) && !runtime_ok {
            Some("install native SteamVR assets first")
        } else {
            None
        };
        if let Some(reason) = reason {
            blocked.insert(stage, reason.into());
        }
    }
    let recordings = checkpoint
        .recordings
        .values()
        .flatten()
        .chain(checkpoint.completed.values().flat_map(|b| &b.recordings))
        .filter(|p| p.is_file())
        .cloned()
        .collect::<BTreeSet<_>>();
    Inspection {
        snapshot: Snapshot {
            camera,
            registration,
            render,
            render_sources,
            inputs,
            recordings,
            blocked,
        },
        checks,
        unplaced,
        identities: identities.filter(|_| known_identities),
        generation: second.as_ref().and_then(|v| v["generation"].as_u64()),
        sensors: sensors.iter().map(shm::serial_str).collect(),
        engine_ready: second.as_ref().and_then(|v| v["engine_ready"].as_bool()),
    }
}

fn reports(inspection: &Inspection, checkpoint: &Checkpoint) -> Vec<setup::StageReport> {
    let mut reports = setup::plan(&inspection.snapshot, checkpoint);
    if !inspection.unplaced.is_empty() && matches!(inspection.snapshot.camera, Artifact::Valid(_)) {
        reports[0].state = if inspection.snapshot.blocked.contains_key(&Stage::Camera) {
            State::Blocked
        } else {
            State::Stale
        };
        reports[0].reason = format!(
            "{} connected sensor(s) need placement",
            inspection.unplaced.len()
        );
        for report in reports
            .iter_mut()
            .filter(|r| matches!(r.stage, Stage::Gravity | Stage::Registration))
        {
            report.state = State::Blocked;
            report.reason = "place connected sensors before completing setup".into();
        }
    }
    reports
}

fn print_report(inspection: &Inspection, checkpoint: &Checkpoint, json: bool) -> bool {
    let stages = reports(inspection, checkpoint);
    let ready =
        stages.iter().all(|r| r.state == State::Ready) && inspection.checks.iter().all(|c| c.ok);
    if json {
        println!("{}", serde_json::to_string_pretty(&serde_json::json!({"ready": ready, "checks": inspection.checks, "stages": stages, "note": "Saved dependency reuse does not verify physical alignment. Setup never pairs or resets controllers."})).unwrap());
    } else {
        let g = glyphs();
        println!(
            "{}\n\n{}",
            ui::checks(&check_views(inspection), inspection.sensors.len(), &g),
            ui::checklist(&stages, &g)
        );
    }
    ready
}

fn choice(prompt: &str, allowed: &[&str]) -> Result<String, String> {
    human_action(prompt);
    io::stdout().flush().map_err(|e| e.to_string())?;
    loop {
        if interrupted() {
            return Err("setup interrupted; earlier files and recordings are retained".into());
        }
        let mut fd = libc::pollfd {
            fd: 0,
            events: libc::POLLIN | libc::POLLHUP,
            revents: 0,
        };
        // SAFETY: one initialized poll descriptor; no device access.
        let result = unsafe { libc::poll(&mut fd, 1, 100) };
        if result < 0 && io::Error::last_os_error().kind() != io::ErrorKind::Interrupted {
            return Err(io::Error::last_os_error().to_string());
        }
        if result > 0 {
            let mut line = String::new();
            if io::stdin()
                .lock()
                .read_line(&mut line)
                .map_err(|e| e.to_string())?
                == 0
            {
                return Err("setup requires an interactive terminal; input closed".into());
            }
            let answer = line.trim().to_ascii_lowercase();
            if allowed.contains(&answer.as_str()) {
                return Ok(answer);
            }
            human_action(&if allowed.contains(&"") {
                format!(
                    "Press Enter{}, or q to quit.",
                    if allowed.contains(&"k") { ", k" } else { "" }
                )
            } else {
                format!("Enter {}.", allowed.join(" / "))
            });
        }
    }
}

/// A stage's log: solver and fit output goes here instead of the terminal, and a failure points
/// at it.
struct StageLog {
    path: PathBuf,
}

impl StageLog {
    fn new(stage: Stage) -> StageLog {
        StageLog {
            path: absolute(&state_dir().join(format!("setup-{stage:?}-{}.log", shm::now_ns()))),
        }
    }

    /// Runs `f` (which must not prompt) with stdout and stderr appended to the log; both are
    /// restored afterwards, also if the log cannot be opened (then `f` prints as usual).
    fn quiet<T>(&self, f: impl FnOnce() -> T) -> T {
        use std::os::unix::fs::OpenOptionsExt;
        use std::os::unix::io::AsRawFd;
        let _ = io::stdout().flush();
        let Ok(log) = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .mode(0o600)
            .open(&self.path)
        else {
            return f();
        };
        // SAFETY: plain descriptor duplication; the saved copies are restored and closed below.
        let saved = unsafe { [libc::dup(1), libc::dup(2)] };
        if saved.contains(&-1) {
            return f();
        }
        unsafe {
            libc::dup2(log.as_raw_fd(), 1);
            libc::dup2(log.as_raw_fd(), 2);
        }
        let out = f();
        let _ = io::stdout().flush();
        unsafe {
            libc::dup2(saved[0], 1);
            libc::dup2(saved[1], 2);
            libc::close(saved[0]);
            libc::close(saved[1]);
        }
        out
    }

    /// The last error the stage reported, for the one-line failure.
    fn last_error(&self) -> Option<String> {
        let text = std::fs::read_to_string(&self.path).ok()?;
        text.lines()
            .rev()
            .find_map(|l| l.strip_prefix("cv1touch: "))
            .map(str::to_owned)
    }

    fn failure(&self, fallback: &str) -> String {
        format!(
            "{} (details: {})",
            self.last_error().unwrap_or_else(|| fallback.to_owned()),
            self.path.display()
        )
    }
}

/// `send` without echoing the daemon's reply; 0 on "ok".
fn send_quiet(line: &str) -> i32 {
    match control::request(&control::socket_path(), line, Duration::from_secs(5)) {
        Ok(reply) if !reply.starts_with("err") => 0,
        Ok(reply) => {
            eprintln!("cv1touch: {line}: {reply}");
            1
        }
        Err(e) => {
            eprintln!("cv1touch: cannot reach cv1touchd: {e}");
            1
        }
    }
}

fn glyphs() -> ui::Glyphs {
    ui::Glyphs::for_terminal(std::env::var("TERM").is_ok_and(|t| t == "dumb"))
}

fn check_views(inspection: &Inspection) -> Vec<ui::CheckView<'_>> {
    inspection
        .checks
        .iter()
        .map(|c| ui::CheckView {
            name: c.name,
            ok: c.ok,
            detail: &c.detail,
        })
        .collect()
}

/// The file a stage installs, parsed, for its result line.
fn installed(stage: Stage) -> Option<serde_json::Value> {
    let path = match stage {
        Stage::Camera | Stage::Gravity => file::default_path(),
        Stage::Render => cv1touch::align::render_path(),
        Stage::Registration => cv1touch::align::registration_path(),
        Stage::Inputs => input_calibration::default_path(),
    };
    serde_json::from_str(&std::fs::read_to_string(path).ok()?).ok()
}

/// Asks at a stage prompt: Enter starts, q quits, k keeps (where offered).
fn ask(prompt: &str, can_keep: bool) -> Result<ui::Answer, String> {
    let keys: &[&str] = if can_keep {
        &[
            "", "new", "start", "s", "y", "yes", "k", "keep", "q", "quit", "exit",
        ]
    } else {
        &["", "new", "start", "s", "y", "yes", "q", "quit", "exit"]
    };
    let line = choice(prompt, keys)?;
    ui::answer(&line, can_keep).ok_or_else(|| "unrecognised answer".to_string())
}

fn persist(checkpoint: &Checkpoint) -> Result<(), String> {
    setup::save(&journal_path(), checkpoint).map_err(|e| format!("setup journal: {e}"))
}

fn capture_path(stage: Stage, checkpoint: &mut Checkpoint) -> Result<PathBuf, String> {
    let rec = absolute(&state_dir().join(format!("setup-{stage:?}-{}.rec", shm::now_ns())));
    // Remember the planned path BEFORE opening the recorder so interruptions keep it discoverable.
    checkpoint.recordings.insert(stage, vec![rec.clone()]);
    persist(checkpoint)?;
    Ok(rec)
}

/// A fresh press of A on the right controller: released in one fresh sample, pressed in a newer
/// one. While the controller is asleep or stale it is not known to be released, so the press
/// that wakes it, or one held from before, starts nothing.
struct PressA {
    last_ns: u64,
    released: bool,
}

impl PressA {
    fn new() -> PressA {
        PressA {
            last_ns: 0,
            released: false,
        }
    }

    fn pressed(&mut self, raw: &RawInputs, now_ns: u64) -> bool {
        let s = &raw.hands[HAND_RIGHT];
        let fresh = s.connected
            && s.received_host_ns != 0
            && now_ns
                .checked_sub(s.received_host_ns)
                .is_some_and(|age| age <= FRESH_INPUT_NS);
        if !fresh {
            self.released = false;
            return false;
        }
        if s.received_host_ns <= self.last_ns {
            return false;
        }
        self.last_ns = s.received_host_ns;
        let down = s.buttons.is_some_and(|b| b & cv1touch::hand::BTN_A != 0);
        let edge = down && self.released;
        self.released = !down;
        edge
    }
}

/// Shows `prompt` and waits for A on the right controller or Enter; q at the terminal stops.
fn go_signal(prompt: &str) -> Result<(), String> {
    human_action(prompt);
    let mut a = PressA::new();
    loop {
        if interrupted() {
            return Err("interrupted; nothing was recorded".into());
        }
        let mut fd = libc::pollfd {
            fd: 0,
            events: libc::POLLIN | libc::POLLHUP,
            revents: 0,
        };
        // SAFETY: one initialized poll descriptor.
        if unsafe { libc::poll(&mut fd, 1, 20) } > 0 {
            let mut line = String::new();
            if io::stdin()
                .lock()
                .read_line(&mut line)
                .map_err(|e| e.to_string())?
                == 0
            {
                return Err("setup requires an interactive terminal; input closed".into());
            }
            if matches!(line.trim(), "q" | "quit") {
                return Err("stopped; nothing was recorded".into());
            }
            return Ok(());
        }
        if raw_inputs().is_ok_and(|raw| a.pressed(&raw, shm::now_ns())) {
            return Ok(());
        }
    }
}

/// A timed step: a cue, a countdown on one line, two cues at the end.
fn timed_window(seconds: u64) -> Result<(), String> {
    cue(1);
    let end = Instant::now() + Duration::from_secs(seconds);
    loop {
        if interrupted() {
            println!();
            return Err("interrupted; the capture is kept".into());
        }
        let left = end.saturating_duration_since(Instant::now());
        if left.is_zero() {
            break;
        }
        print!("\r  {:>3} s ", left.as_secs() + 1);
        let _ = io::stdout().flush();
        thread::sleep(Duration::from_millis(200));
    }
    print!("\r        \r");
    let _ = io::stdout().flush();
    cue(2);
    Ok(())
}

const START: &str = "Press A on the right controller (or Enter) to start.";

const CAMERA_MOTION: &str = "For 60 seconds, wave both controllers slowly through the whole play space:\nhigh and low, near and far, twisting your wrists. Every sensor should see them, often two at once.";

fn timed_capture(
    stage: Stage,
    checkpoint: &mut Checkpoint,
    gravity: bool,
) -> Result<PathBuf, String> {
    let rec = capture_path(stage, checkpoint)?;
    let mut guard = guided_start_recording(&rec, false)?;
    let result = if gravity {
        human_action("Hold both controllers still in 12 different orientations: tilt AND roll them between holds, rings visible.\nFor each one: hold still, press A (or Enter), and keep still until the double buzz (3 s).");
        (1..=12).try_for_each(|hold| -> Result<(), String> {
            go_signal(&format!(
                "  Hold {hold}/12: still, in a new orientation, then A."
            ))?;
            timed_window(3)
        })
    } else if stage == Stage::Render {
        go_signal(&format!(
            "Wake both controllers and keep them awake for 5 seconds.\n{START}"
        ))
        .and_then(|()| timed_window(5))
    } else {
        go_signal(&format!("{CAMERA_MOTION}\n{START}")).and_then(|()| timed_window(60))
    };
    let stopped = control::request(
        &control::socket_path(),
        &guard.command,
        Duration::from_millis(750),
    )
    .map_err(|error| error.to_string());
    let stopped_ok = stopped
        .as_ref()
        .is_ok_and(|reply| reply.starts_with("ok stopped "));
    if stopped_ok {
        guard.armed = false;
    }
    result?;
    if !stopped_ok {
        return Err(format!(
            "recording did not stop successfully; own-path cleanup will retry: {}",
            stopped.unwrap_or_else(|error| error)
        ));
    }
    Ok(rec)
}

struct RestoreCamera(bool);
impl Drop for RestoreCamera {
    fn drop(&mut self) {
        if self.0 {
            restore_mode(&file::default_path());
        }
    }
}

fn verify_capture(inspection: &Inspection) -> Result<(), String> {
    if interrupted() {
        return Err("setup interrupted; capture retained and no fit installed".into());
    }
    let ids = inspection
        .identities
        .as_ref()
        .ok_or("controller identities unavailable")?;
    let raw = raw_inputs()?;
    input_identities_match(&raw, ids)?;
    PairedInputPhase::new([0, 0]).observe(&raw, ids, shm::now_ns())?;
    let status = read_status().ok_or("daemon unavailable after capture")?;
    if inspection.generation.is_none() || status["generation"].as_u64() != inspection.generation {
        return Err("daemon restarted during capture; recording retained, no fit installed".into());
    }
    if status["recording"] != false {
        return Err("recording is still active; finish it before fitting".into());
    }
    if let Artifact::Valid(before) = &inspection.snapshot.camera {
        let now = setup::fingerprint(&file::default_path()).map_err(|e| e.to_string())?;
        if &now != before {
            return Err(
                "camera calibration changed during capture; recording retained, no fit installed"
                    .into(),
            );
        }
    }
    Ok(())
}

fn verify_fitted_dependencies(
    stage: Stage,
    before: &Inspection,
    after: &Inspection,
) -> Result<(), String> {
    if after.generation != before.generation || after.identities != before.identities {
        return Err("daemon or controller identities changed during fitting; stage remains incomplete, previous backups retained".into());
    }
    if stage == Stage::Render && after.snapshot.render_sources != before.snapshot.render_sources {
        return Err(
            "render source meshes changed during fitting; stage remains incomplete, repeat the fit"
                .into(),
        );
    }
    if stage == Stage::Registration && after.snapshot.camera != before.snapshot.camera {
        return Err("camera world changed during registration fitting; stage remains incomplete, repeat the fit".into());
    }
    Ok(())
}

// Calibration load acknowledges the requested mode before the engine reopens.
// Require two samples in the applied mode: counters reset during reopen, so compare the
// new stream with itself rather than requiring it to exceed pre-reload counters.
fn wait_reload_with(
    before: &Inspection,
    expected_mode: &str,
    timeout: Duration,
    mut sample: impl FnMut() -> Result<(Option<serde_json::Value>, Option<RawInputs>, u64), String>,
    mut elapsed: impl FnMut() -> Duration,
    mut pause: impl FnMut(),
) -> Result<(), String> {
    let generation = before
        .generation
        .ok_or("daemon generation unavailable before reload")?;
    let ids = before
        .identities
        .as_ref()
        .ok_or("controller identities unavailable before reload")?;
    if before.sensors.is_empty() {
        return Err("connected sensor set unavailable before reload".into());
    }
    let mut previous: Option<BTreeMap<String, u32>> = None;
    while elapsed() < timeout {
        let (status, raw, now_ns) = sample()?;
        if elapsed() >= timeout {
            break;
        }
        if status
            .as_ref()
            .and_then(|v| v["generation"].as_u64())
            .is_some_and(|g| g != generation)
        {
            return Err("daemon restarted during reload; stage remains incomplete".into());
        }
        if status
            .as_ref()
            .is_some_and(|v| v["engine_ready"].as_bool().is_none())
        {
            return Err("guided reload requires a matching cv1touchd with engine readiness diagnostics; stage remains incomplete".into());
        }
        if let Some(raw) = &raw {
            for (hand, expected) in raw.hands.iter().zip(ids) {
                let known = hand.identity.len() == 32
                    && hand
                        .identity
                        .bytes()
                        .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b));
                if known && &hand.identity != expected {
                    return Err(
                        "physical controller changed during reload; stage remains incomplete"
                            .into(),
                    );
                }
            }
        }
        let fresh = raw.as_ref().is_some_and(|raw| {
            input_identities_match(raw, ids).is_ok()
                && PairedInputPhase::new([0, 0])
                    .observe(raw, ids, now_ns)
                    .is_ok()
        });
        let sensors = status
            .as_ref()
            .filter(|v| {
                v["generation"].as_u64() == Some(generation)
                    && v["mode"] == expected_mode
                    && v["engine_ready"] == true
                    && v["recording"] == false
            })
            .and_then(|v| status::sensors_from_status(v).ok());
        let current = sensors
            .filter(|sensors| {
                !sensors.is_empty()
                    && (expected_mode != "calibrated"
                        || sensors.iter().all(|sensor| sensor.placed != 0))
                    && sensors.iter().map(shm::serial_str).collect::<BTreeSet<_>>()
                        == before.sensors
                    && sensors.len() == before.sensors.len()
            })
            .map(|sensors| {
                sensors
                    .into_iter()
                    .map(|sensor| (shm::serial_str(&sensor), sensor.frames))
                    .collect::<BTreeMap<_, _>>()
            });
        if let Some(current) = current {
            if fresh
                && previous.as_ref().is_some_and(|old| {
                    current
                        .iter()
                        .all(|(serial, frames)| old.get(serial).is_some_and(|prior| frames > prior))
                })
            {
                return Ok(());
            }
            previous = Some(current);
        } else {
            previous = None;
        }
        pause();
    }
    Err(format!("timed out waiting for {expected_mode} sensor streams and awake matching controllers after reload; stage remains incomplete, rerun setup when ready"))
}

fn wait_reload(before: &Inspection, expected_mode: &str) -> Result<(), String> {
    println!("Tracking restarts; keep the controllers awake.");
    let start = std::time::Instant::now();
    wait_reload_with(
        before,
        expected_mode,
        Duration::from_secs(30),
        || {
            if interrupted() {
                return Err("setup interrupted during reload; stage remains incomplete".into());
            }
            let status = read_status();
            let raw = raw_inputs().ok();
            if interrupted() {
                return Err("setup interrupted during reload; stage remains incomplete".into());
            }
            Ok((status, raw, shm::now_ns()))
        },
        || start.elapsed(),
        || thread::sleep(Duration::from_millis(100)),
    )
}

/// Input older than this is stale: the controller stopped sending (asleep, or out of range).
const FRESH_INPUT_NS: u64 = 500_000_000;

struct ButtonEdges {
    last_ns: [u64; 2],
    released: [u32; 2],
    armed_after_ns: u64,
    /// Preparation: per hand, whether its newest sample was asleep or stale.
    waiting: [bool; 2],
    /// Preparation: a controller dropped out since the last press check; both hands re-arm.
    rearm: bool,
}
impl ButtonEdges {
    fn new(armed_after_ns: u64) -> Self {
        Self {
            last_ns: [0; 2],
            released: [0; 2],
            armed_after_ns,
            waiting: [false; 2],
            rearm: false,
        }
    }
    /// Before the countdown, a controller that falls asleep (or goes stale) is waited for: None
    /// until both are fresh again, and then both hands need a fresh release and press, so a wake,
    /// held or queued press starts nothing. A changed controller, missing button support and a
    /// zero, future or backward timestamp still fail at once, and a fresh B on the right
    /// controller cancels even while the left one sleeps.
    fn observe_preparation(
        &mut self,
        raw: &RawInputs,
        ids: &[String; 2],
        now_ns: u64,
    ) -> Result<Option<[u32; 2]>, String> {
        input_identities_match(raw, ids)?;
        for (hand, sample) in raw.hands.iter().enumerate() {
            let buttons = sample.buttons.ok_or("controller-guided registration requires a matching cv1touchd with raw button support; update the daemon")?;
            if sample.received_host_ns == 0 || sample.received_host_ns > now_ns {
                return Err(
                    "controller input has an invalid timestamp; registration not started or fitted"
                        .into(),
                );
            }
            if sample.received_host_ns < self.last_ns[hand] {
                return Err(
                    "controller input timestamp moved backward; registration cancelled".into(),
                );
            }
            self.waiting[hand] =
                !sample.connected || now_ns - sample.received_host_ns > FRESH_INPUT_NS;
            if hand == HAND_RIGHT && !self.waiting[hand] && buttons & cv1touch::hand::BTN_B != 0 {
                return Err(
                    "registration cancelled with B on the right controller; no recording started"
                        .into(),
                );
            }
        }
        if self.waiting.contains(&true) {
            for (last, sample) in self.last_ns.iter_mut().zip(&raw.hands) {
                *last = (*last).max(sample.received_host_ns);
            }
            self.rearm = true;
            return Ok(None);
        }
        if std::mem::take(&mut self.rearm) {
            self.released = [0; 2];
        }
        self.observe(raw, ids, now_ns).map(Some)
    }
    fn observe(
        &mut self,
        raw: &RawInputs,
        ids: &[String; 2],
        now_ns: u64,
    ) -> Result<[u32; 2], String> {
        input_identities_match(raw, ids)?;
        let mut edges = [0; 2];
        for (hand, sample) in raw.hands.iter().enumerate() {
            let buttons = sample.buttons.ok_or("controller-guided registration requires a matching cv1touchd with raw button support; update the daemon")?;
            if !sample.connected
                || sample.received_host_ns == 0
                || now_ns
                    .checked_sub(sample.received_host_ns)
                    .is_none_or(|age| age > FRESH_INPUT_NS)
            {
                return Err("controller input is disconnected, stale or has an invalid timestamp; registration not started or fitted".into());
            }
            if sample.received_host_ns < self.last_ns[hand] {
                return Err(
                    "controller input timestamp moved backward; registration cancelled".into(),
                );
            }
            if sample.received_host_ns == self.last_ns[hand] {
                continue;
            }
            self.last_ns[hand] = sample.received_host_ns;
            if sample.received_host_ns <= self.armed_after_ns {
                continue;
            }
            edges[hand] = buttons & self.released[hand];
            self.released[hand] = !buttons;
        }
        Ok(edges)
    }
}

const REGISTRATION_PREPARATION: &str = "Put on the headset (SteamVR running, headset tracking).
Press both controllers firmly against it, rings and the headset's sensors uncovered.
The fit measures where they sit on the headset, so don't let them slip.
Press A to start: 10 pulses to get in position, then a double buzz starts the recording.";

/// The controllers' raw inputs, with the daemon, its camera world and sensor placement checked
/// unchanged since `before`; the samples themselves are not judged here.
fn registration_context(before: &Inspection) -> Result<RawInputs, String> {
    if interrupted() {
        return Err("registration interrupted; no fit installed".into());
    }
    let reply = control::request(
        &control::socket_path(),
        "inputs",
        Duration::from_millis(500),
    )
    .map_err(|e| e.to_string())?;
    let raw = parse_raw_inputs(&reply)?;
    let reply = control::request(
        &control::socket_path(),
        "status",
        Duration::from_millis(500),
    )
    .map_err(|e| e.to_string())?;
    let status: serde_json::Value = serde_json::from_str(
        reply
            .strip_prefix("ok ")
            .ok_or("cannot read daemon status during registration")?,
    )
    .map_err(|e| e.to_string())?;
    if before.generation.is_none() || status["generation"].as_u64() != before.generation {
        return Err(
            "daemon restarted during registration preparation or capture; no fit installed".into(),
        );
    }
    let sensors = status::sensors_from_status(&status)?;
    if status["mode"] != "calibrated"
        || status["engine_ready"] == false
        || sensors.len() != before.sensors.len()
        || sensors.iter().any(|sensor| sensor.placed == 0)
        || sensors.iter().map(shm::serial_str).collect::<BTreeSet<_>>() != before.sensors
    {
        return Err(
            "camera engine or sensor placement changed during registration; no fit installed"
                .into(),
        );
    }
    if before.snapshot.camera
        != Artifact::Valid(setup::fingerprint(&file::default_path()).map_err(|e| e.to_string())?)
    {
        return Err("camera world changed during registration; no fit installed".into());
    }
    Ok(raw)
}

/// `registration_context` with both controllers required awake and fresh, as the countdown and
/// the recording need them.
fn registration_inputs(before: &Inspection) -> Result<RawInputs, String> {
    let raw = registration_context(before)?;
    let identities = before
        .identities
        .as_ref()
        .ok_or("controller identities unavailable")?;
    PairedInputPhase::new([0, 0]).observe(&raw, identities, shm::now_ns())?;
    if raw.hands.iter().any(|hand| hand.buttons.is_none()) {
        return Err("guided controller controls require a matching cv1touchd with raw button support; update the daemon and retry".into());
    }
    if raw.hands[HAND_RIGHT]
        .buttons
        .is_some_and(|buttons| buttons & cv1touch::hand::BTN_B != 0)
    {
        return Err(
            "registration cancelled with B on the right controller; capture retained if started, no fit installed"
                .into(),
        );
    }
    Ok(raw)
}

fn registration_haptic(
    hand: usize,
    duration: f32,
    frequency: f32,
    amplitude: f32,
) -> Result<(), String> {
    let command = format!(
        "haptic {} {duration} {frequency} {amplitude}",
        ["left", "right"][hand]
    );
    let reply = control::request(
        &control::socket_path(),
        &command,
        Duration::from_millis(500),
    )
    .map_err(|e| format!("haptic command failed: {e}; cancel and verify controller feedback"))?;
    if reply.trim() != "ok" {
        return Err(format!(
            "haptic command rejected: {reply}; cancel and verify controller feedback"
        ));
    }
    Ok(())
}

/// Waits up to 300 s, before the countdown, for a fresh release and press of `button` on `hand`
/// (`name` in prompts). A controller that falls asleep meanwhile is waited for, with one line when
/// it drops out and one when it is back; anything else wrong fails at once.
fn wait_preparation_press(
    before: &Inspection,
    hand: usize,
    button: u32,
    name: &str,
) -> Result<(), String> {
    let start = Instant::now();
    let mut edges = ButtonEdges::new(shm::now_ns());
    let ids = before
        .identities
        .as_ref()
        .ok_or("controller identities unavailable")?;
    let mut shown = [false; 2];
    while start.elapsed() < Duration::from_secs(300) {
        let raw = registration_context(before)?;
        let edge = edges.observe_preparation(&raw, ids, shm::now_ns())?;
        if let Some(line) = readiness_change(shown, edges.waiting, name) {
            human_action(&line);
        }
        shown = edges.waiting;
        if edge.is_some_and(|edges| edges[hand] & button != 0) {
            return Ok(());
        }
        thread::sleep(Duration::from_millis(20));
    }
    Err("timed out waiting for a fresh controller release and press; rerun setup when ready".into())
}

/// The line to show when the controllers' readiness changes: which one fell asleep, or, once
/// both are back, the press still awaited. None while nothing changed.
fn readiness_change(shown: [bool; 2], waiting: [bool; 2], name: &str) -> Option<String> {
    let names = ["left", "right"];
    if let Some(hand) = (0..2).find(|&h| waiting[h] && !shown[h]) {
        return Some(format!(
            "  The {} controller is asleep: press any button on it.",
            names[hand]
        ));
    }
    (shown.contains(&true) && !waiting.contains(&true))
        .then(|| format!("  Both awake. Press {name}."))
}

fn confirm_registration_controls_with(
    mut press: impl FnMut(usize, u32, &str) -> Result<(), String>,
    mut buzz: impl FnMut(usize) -> Result<(), String>,
) -> Result<(), String> {
    for (hand, button, name, key) in [
        (
            HAND_LEFT,
            cv1touch::hand::BTN_X,
            "X on the left controller",
            "X",
        ),
        (
            HAND_RIGHT,
            cv1touch::hand::BTN_A,
            "A on the right controller",
            "A",
        ),
    ] {
        human_action(&format!("  Press {name}."));
        press(hand, button, name)?;
        buzz(hand)?;
        human_action(&format!("  Felt a buzz? Press {key} again."));
        press(hand, button, name)?;
    }
    Ok(())
}

fn registration_countdown_with(
    mut tick: impl FnMut(usize) -> Result<(), String>,
    mut second: impl FnMut() -> Result<(), String>,
) -> Result<(), String> {
    for remaining in (1..=10).rev() {
        print!("\r  getting in position: {remaining:>2} s ");
        let _ = io::stdout().flush();
        tick(HAND_LEFT)?;
        tick(HAND_RIGHT)?;
        second()?;
    }
    Ok(())
}

fn registration_countdown(before: &Inspection) -> Result<(), String> {
    let ids = before
        .identities
        .as_ref()
        .ok_or("controller identities unavailable")?;
    let mut edges = ButtonEdges::new(shm::now_ns());
    registration_countdown_with(
        |hand| registration_haptic(hand, 0.08, 80., 0.35),
        || {
            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(1) {
                let raw = registration_inputs(before)?;
                edges.observe(&raw, ids, shm::now_ns())?;
                thread::sleep(Duration::from_millis(20));
            }
            Ok(())
        },
    )
}

struct RegistrationFeedback<'a> {
    before: &'a Inspection,
    edges: ButtonEdges,
}
impl RegistrationFeedback<'_> {
    fn rearm_finish(&mut self, now_ns: u64) {
        self.edges = ButtonEdges::new(now_ns);
    }
}

impl CaptureFeedback for RegistrationFeedback<'_> {
    fn before_start(&mut self) -> Result<(), String> {
        let raw = registration_inputs(self.before)?;
        let ids = self
            .before
            .identities
            .as_ref()
            .ok_or("controller identities unavailable")?;
        self.edges.observe(&raw, ids, shm::now_ns())?;
        Ok(())
    }
    fn started(&mut self) -> Result<(), String> {
        for pulse in 0..2 {
            for hand in [HAND_LEFT, HAND_RIGHT] {
                registration_haptic(hand, 0.15, 160., 0.65)?;
            }
            if pulse == 0 {
                thread::sleep(Duration::from_millis(200));
            }
        }
        // Start ACK and both start-cue pulses must precede any finish-button edge.
        self.rearm_finish(shm::now_ns());
        Ok(())
    }
    fn poll(&mut self) -> Result<bool, String> {
        let raw = registration_inputs(self.before)?;
        let ids = self
            .before
            .identities
            .as_ref()
            .ok_or("controller identities unavailable")?;
        Ok(self.edges.observe(&raw, ids, shm::now_ns())?[HAND_RIGHT] & cv1touch::hand::BTN_A != 0)
    }
    fn stopped(&mut self) -> Result<(), String> {
        for hand in [HAND_LEFT, HAND_RIGHT] {
            registration_haptic(hand, 0.3, 80., 0.45)?;
        }
        Ok(())
    }
}

fn verify_registration_preparation(before: &Inspection, after: &Inspection) -> Result<(), String> {
    if before.generation.is_none() || before.identities.is_none() {
        return Err(
            "registration preparation requires known daemon and controller identities".into(),
        );
    }
    verify_fitted_dependencies(Stage::Registration, before, after)?;
    if let Some(reason) = after.snapshot.blocked.get(&Stage::Registration) {
        return Err(format!(
            "registration preparation is not ready: {reason}; no recording started"
        ));
    }
    if !matches!(after.snapshot.camera, Artifact::Valid(_))
        || !after.unplaced.is_empty()
        || after.sensors != before.sensors
    {
        return Err("camera placement or connected sensors changed during preparation; no recording started".into());
    }
    Ok(())
}

fn registration_capture_after_preparation<T>(
    checkpoint: &mut Checkpoint,
    prepare: impl FnOnce() -> Result<(), String>,
    countdown: impl FnOnce() -> Result<(), String>,
    verify: impl FnOnce(&Checkpoint) -> Result<(), String>,
    capture: impl FnOnce(&mut Checkpoint) -> Result<T, String>,
) -> Result<T, String> {
    prepare()?;
    countdown()?;
    verify(checkpoint)?;
    capture(checkpoint)
}

fn run_stage(
    stage: Stage,
    inspection: &Inspection,
    checkpoint: &mut Checkpoint,
) -> Result<(), String> {
    if matches!(stage, Stage::Camera | Stage::Gravity) && inspection.engine_ready.is_none() {
        return Err("guided reload requires a matching cv1touchd with engine readiness diagnostics; no capture was started".into());
    }
    let old = checkpoint.clone();
    let preserve = stage == Stage::Camera
        && !inspection.unplaced.is_empty()
        && matches!(inspection.snapshot.camera, Artifact::Valid(_));
    setup::begin_stage(stage, checkpoint);
    persist(checkpoint)?;
    let mut recordings = Vec::new();
    let mut accepted_camera_sha256 = None;
    let log = StageLog::new(stage);
    let code = match stage {
        Stage::Camera if preserve => {
            println!("New sensors join the existing ones; keep those where they are.");
            let rec = timed_capture(stage, checkpoint, false)?;
            recordings.push(rec.clone());
            verify_capture(inspection)?;
            for serial in &inspection.unplaced {
                if log.quiet(|| calibrate_place(serial, &rec, &file::default_path(), true)) != 0 {
                    return Err(log.failure("sensor placement rejected; earlier backups retained"));
                }
            }
            log.quiet(|| restore_mode(&file::default_path()))
        }
        Stage::Camera => {
            let mut restore = RestoreCamera(true);
            if log.quiet(|| send_quiet("calibration capture")) != 0 {
                return Err("cannot enter camera capture mode".into());
            }
            wait_reload(inspection, "capture")?;
            let rec = timed_capture(stage, checkpoint, false)?;
            recordings.push(rec.clone());
            verify_capture(inspection)?;
            let solved =
                match log.quiet(|| calibrate_solve_saved(&rec, &file::default_path(), false)) {
                    Ok(fingerprint) => {
                        accepted_camera_sha256 = Some(fingerprint);
                        0
                    }
                    Err(code) => code,
                };
            let restored = log.quiet(|| restore_mode(&file::default_path()));
            if restored == 0 {
                restore.0 = false;
            }
            if solved == 0 {
                restored
            } else {
                solved
            }
        }
        Stage::Gravity => {
            let rec = timed_capture(stage, checkpoint, true)?;
            recordings.push(rec.clone());
            verify_capture(inspection)?;
            let compensate = setup::plan(&inspection.snapshot, &old)
                .iter()
                .any(|r| r.stage == Stage::Registration && r.state == State::Ready);
            log.quiet(|| {
                let code = calibrate_gravity_with(&rec, &file::default_path(), true, compensate);
                if code == 0 && restore_mode(&file::default_path()) == 0 {
                    send_quiet("alignment reload")
                } else {
                    1
                }
            })
        }
        Stage::Render => {
            let rec = timed_capture(stage, checkpoint, false)?;
            recordings.push(rec.clone());
            verify_capture(inspection)?;
            log.quiet(|| fit_render(Some(&rec)))
        }
        Stage::Registration => {
            let (rec, base) = registration_capture_after_preparation(
                checkpoint,
                || {
                    human_action("Button check (B on the right controller cancels at any point):");
                    confirm_registration_controls_with(
                        |hand, button, name| wait_preparation_press(inspection, hand, button, name),
                        |hand| registration_haptic(hand, 0.12, 80., 0.35),
                    )?;
                    human_action(REGISTRATION_PREPARATION);
                    wait_preparation_press(
                        inspection,
                        HAND_RIGHT,
                        cv1touch::hand::BTN_A,
                        "A on the right controller",
                    )
                },
                || registration_countdown(inspection),
                |checkpoint| {
                    let current = inspect(checkpoint);
                    if interrupted() {
                        return Err(
                            "setup interrupted during preparation; no recording started".into()
                        );
                    }
                    verify_registration_preparation(inspection, &current)
                },
                |checkpoint| {
                    if interrupted() {
                        return Err(
                            "setup interrupted during preparation; no recording started".into()
                        );
                    }
                    let rec = capture_path(stage, checkpoint)?;
                    let base = rec.with_extension("");
                    checkpoint
                        .recordings
                        .entry(stage)
                        .or_default()
                        .push(base.with_extension("truth.json"));
                    persist(checkpoint)?;
                    if interrupted() {
                        return Err(
                            "setup interrupted during preparation; no recording started".into()
                        );
                    }
                    print!("\r                            \r");
                    let mut feedback = RegistrationFeedback {
                        before: inspection,
                        edges: ButtonEdges::new(shm::now_ns()),
                    };
                    capture_free_named_with_feedback(150, "Recording: turn slowly twice each way, then nod and tilt. Keep the controllers from slipping.\nA to finish (or Enter), B to cancel.", false, Some(&base), Some(&mut feedback)).map_err(|code| format!("headset capture failed ({code}); capture retained, no fit installed"))?;
                    Ok((rec, base))
                },
            )?;
            recordings.push(rec);
            recordings.push(base.with_extension("truth.json"));
            verify_capture(inspection)?;
            log.quiet(|| {
                let code = register_from_capture_with(&base, true, Some("hmd"), true, false);
                if code == 0 {
                    send_quiet("alignment reload")
                } else {
                    code
                }
            })
        }
        Stage::Inputs => calibrate_inputs_with(None, false, true),
    };
    if interrupted() {
        return Err("interrupted; earlier settings and the capture are kept".into());
    }
    if code != 0 {
        return Err(log.failure("the fit was rejected; earlier settings and the capture are kept"));
    }
    if matches!(stage, Stage::Camera | Stage::Gravity) {
        wait_reload(inspection, "calibrated")?;
    }
    let mut next = inspect(checkpoint);
    verify_fitted_dependencies(stage, inspection, &next)?;
    for path in &recordings {
        if path.is_file() {
            next.snapshot.recordings.insert(path.clone());
        }
    }
    if preserve || stage == Stage::Gravity {
        let mut carried = old;
        setup::preserve_camera_update(
            &inspection.snapshot,
            &next.snapshot,
            &mut carried,
            stage == Stage::Gravity,
        )?;
        carried.recordings = checkpoint.recordings.clone();
        *checkpoint = carried;
    }
    if interrupted() {
        return Err("setup interrupted; stage remains incomplete and captures retained".into());
    }
    if stage == Stage::Camera && !preserve {
        let accepted = accepted_camera_sha256
            .as_deref()
            .ok_or("accepted camera output fingerprint unavailable; stage remains incomplete")?;
        setup::record_full_camera_success(&next.snapshot, checkpoint, accepted, recordings)?;
    } else {
        setup::record_success(stage, &next.snapshot, checkpoint, recordings)?;
    }
    persist(checkpoint)
}

pub(super) fn run(check_only: bool, json: bool) -> i32 {
    if !check_only && !io::stdin().is_terminal() {
        eprintln!("cv1touch: setup needs an interactive terminal. Use setup --check [--json] for a read-only report.");
        return 2;
    }
    let mut checkpoint = match setup::load(&journal_path()) {
        Ok(value) => value.unwrap_or_default(),
        Err(error) => {
            eprintln!("cv1touch: setup journal: {error}; inspect or move the journal explicitly before retrying.");
            return 1;
        }
    };
    if check_only {
        return if print_report(&inspect(&checkpoint), &checkpoint, json) {
            0
        } else {
            1
        };
    }
    catch_interrupts();
    let mut work = || -> Result<(), String> {
        std::fs::create_dir_all(state_dir()).map_err(|e| e.to_string())?;
        let _lock = control::lock_single_instance(&state_dir().join("setup.lock"))
            .map_err(|_| "another setup guide is running; finish it first".to_string())?;
        checkpoint = setup::load(&journal_path())
            .map_err(|e| format!("setup journal: {e}"))?
            .unwrap_or_default();
        let g = glyphs();
        let mut shown_checks = String::new();
        // the stage list shows at the start and again after a failure; a finished stage is one
        // result line, then the next prompt
        let mut show_list = true;
        loop {
            let inspection = inspect(&checkpoint);
            let views = check_views(&inspection);
            let check_text = ui::checks(&views, inspection.sensors.len(), &g);
            if check_text != shown_checks {
                println!("{check_text}");
                shown_checks = check_text;
            }
            let stage_reports = reports(&inspection, &checkpoint);
            if std::mem::take(&mut show_list) {
                println!("\n{}\n", ui::checklist(&stage_reports, &g));
            }
            let Some(report) = stage_reports.iter().find(|r| r.state != State::Ready) else {
                human_action("Saved setup is complete. Check both hands in SteamVR: position, buttons and haptics.");
                return Ok(());
            };
            if !ui::checks_pass(&views) || report.state == State::Blocked {
                if report.state == State::Blocked {
                    println!(
                        "{} {}: {}",
                        g.fail,
                        ui::stage_name(report.stage),
                        report.reason
                    );
                }
                show_list = true;
                if ask(
                    "Fix that, then press Enter to check again (q: quit).",
                    false,
                )? == ui::Answer::Quit
                {
                    return Err("setup stopped before finishing; completed stages are kept".into());
                }
                continue;
            }
            let can_adopt = matches!(report.state, State::Unknown | State::Stale)
                && inspection.unplaced.is_empty()
                && matches!(
                    report.stage,
                    Stage::Camera | Stage::Gravity | Stage::Render | Stage::Registration
                );
            match ask(&ui::stage_prompt(report.stage, can_adopt), can_adopt)? {
                ui::Answer::Quit => return Ok(()),
                ui::Answer::Keep => {
                    setup::adopt(report.stage, &inspection.snapshot, &mut checkpoint)?;
                    persist(&checkpoint)?;
                    println!("{} {}: kept", g.done, ui::stage_name(report.stage));
                }
                ui::Answer::Start => {
                    // Recheck live prerequisites after the user has prepared the hardware.
                    let current = inspect(&checkpoint);
                    if let Some(reason) = current.snapshot.blocked.get(&report.stage) {
                        println!("{} {}: {reason}", g.fail, ui::stage_name(report.stage));
                        show_list = true;
                        continue;
                    }
                    let name = ui::stage_name(report.stage);
                    match run_stage(report.stage, &current, &mut checkpoint) {
                        Ok(()) => println!(
                            "{} {name}: {}",
                            g.done,
                            ui::result_summary(report.stage, installed(report.stage).as_ref())
                        ),
                        Err(error) => {
                            println!("{} {name}: {error}", g.fail);
                            show_list = true;
                            if interrupted() {
                                return Err("setup interrupted; completed stages are kept".into());
                            }
                            if ask("Enter: try again   q: quit", false)? == ui::Answer::Quit {
                                return Err(format!(
                                    "{name} not finished; completed stages are kept"
                                ));
                            }
                        }
                    }
                }
            }
        }
    };
    match work() {
        Ok(()) => 0,
        Err(error) => {
            eprintln!("cv1touch: {error}");
            1
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn saved() -> Inspection {
        let valid = || Artifact::Valid("a".repeat(64));
        Inspection {
            snapshot: Snapshot {
                camera: valid(),
                registration: valid(),
                render: valid(),
                render_sources: valid(),
                inputs: valid(),
                recordings: BTreeSet::new(),
                blocked: BTreeMap::new(),
            },
            checks: vec![],
            unplaced: vec![],
            identities: Some(["1".repeat(32), "2".repeat(32)]),
            generation: Some(1),
            sensors: BTreeSet::from(["CAMTEST".into()]),
            engine_ready: Some(true),
        }
    }
    #[test]
    fn controls_need_a_press_buzz_and_another_confirmation_press_for_each_hand() {
        let events = std::cell::RefCell::new(Vec::new());
        confirm_registration_controls_with(
            |hand, _, _| {
                events.borrow_mut().push(("press", hand));
                Ok(())
            },
            |hand| {
                events.borrow_mut().push(("buzz", hand));
                Ok(())
            },
        )
        .unwrap();
        assert_eq!(
            *events.borrow(),
            [
                ("press", HAND_LEFT),
                ("buzz", HAND_LEFT),
                ("press", HAND_LEFT),
                ("press", HAND_RIGHT),
                ("buzz", HAND_RIGHT),
                ("press", HAND_RIGHT)
            ]
        );
    }
    #[test]
    fn countdown_ticks_both_hands_ten_times_before_capture() {
        let events = std::cell::RefCell::new(Vec::new());
        registration_capture_after_preparation(
            &mut Checkpoint::default(),
            || Ok(()),
            || {
                registration_countdown_with(
                    |hand| {
                        events.borrow_mut().push(hand);
                        Ok(())
                    },
                    || {
                        events.borrow_mut().push(2);
                        Ok(())
                    },
                )
            },
            |_| Ok(()),
            |_| {
                events.borrow_mut().push(3);
                Ok(())
            },
        )
        .unwrap();
        assert_eq!(events.borrow().len(), 31);
        for second in events.borrow()[..30].chunks(3) {
            assert_eq!(second, [HAND_LEFT, HAND_RIGHT, 2]);
        }
        assert_eq!(events.borrow()[30], 3);
    }
    #[test]
    fn haptic_rejection_or_countdown_cancel_prevents_recording() {
        for haptic_failure in [true, false] {
            let starts = std::cell::Cell::new(0);
            let result = registration_capture_after_preparation(
                &mut Checkpoint::default(),
                || Ok(()),
                || {
                    registration_countdown_with(
                        |_| {
                            if haptic_failure {
                                Err("haptic rejected".into())
                            } else {
                                Ok(())
                            }
                        },
                        || Err("B on the right controller or interrupt cancelled countdown".into()),
                    )
                },
                |_| Ok(()),
                |_| {
                    starts.set(starts.get() + 1);
                    Ok(())
                },
            );
            assert!(result.is_err());
            assert_eq!(starts.get(), 0);
        }
        let presses = std::cell::Cell::new(0);
        assert!(confirm_registration_controls_with(
            |_, _, _| {
                presses.set(presses.get() + 1);
                Ok(())
            },
            |_| Err("haptic rejected".into())
        )
        .is_err());
        assert_eq!(
            presses.get(),
            1,
            "queue failure must not solicit or claim receipt confirmation"
        );
    }
    #[test]
    fn controller_start_requires_a_new_release_press_and_ignores_duplicate_reports() {
        let mut raw = reload_inputs();
        let ids = saved().identities.unwrap();
        let mut edges = ButtonEdges::new(999_999_999);
        raw.hands[1].buttons = Some(cv1touch::hand::BTN_A);
        assert_eq!(
            edges.observe(&raw, &ids, 1_000_000_010).unwrap()[1],
            0,
            "already held must not start"
        );
        raw.hands[1].buttons = Some(0);
        raw.hands[1].received_host_ns += 1;
        edges.observe(&raw, &ids, 1_000_000_010).unwrap();
        raw.hands[1].buttons = Some(cv1touch::hand::BTN_A);
        assert_eq!(
            edges.observe(&raw, &ids, 1_000_000_010).unwrap()[1],
            0,
            "same timestamp is not a new press"
        );
        raw.hands[1].received_host_ns += 1;
        assert_eq!(
            edges.observe(&raw, &ids, 1_000_000_010).unwrap()[1],
            cv1touch::hand::BTN_A
        );
        assert_eq!(edges.observe(&raw, &ids, 1_000_000_010).unwrap()[1], 0);
    }
    #[test]
    fn controller_start_rejects_missing_stale_disconnected_or_replaced_samples() {
        let ids = saved().identities.unwrap();
        let mut raw = reload_inputs();
        raw.hands[1].buttons = None;
        assert!(ButtonEdges::new(0)
            .observe(&raw, &ids, 1_000_000_010)
            .is_err());
        raw = reload_inputs();
        raw.hands[0].connected = false;
        assert!(ButtonEdges::new(0)
            .observe(&raw, &ids, 1_000_000_010)
            .is_err());
        raw = reload_inputs();
        raw.hands[0].identity = "3".repeat(32);
        assert!(ButtonEdges::new(0)
            .observe(&raw, &ids, 1_000_000_010)
            .is_err());
        raw = reload_inputs();
        assert!(ButtonEdges::new(0)
            .observe(&raw, &ids, 2_000_000_000)
            .is_err());
        raw = reload_inputs();
        let mut edges = ButtonEdges::new(0);
        edges.observe(&raw, &ids, 1_000_000_010).unwrap();
        raw.hands[0].received_host_ns -= 1;
        assert!(edges.observe(&raw, &ids, 1_000_000_010).is_err());
    }
    #[test]
    fn readiness_lines_only_on_a_dropout_and_the_recovery() {
        let name = "A on the right controller";
        assert_eq!(readiness_change([false; 2], [false; 2], name), None);
        assert!(readiness_change([false; 2], [true, false], name)
            .unwrap()
            .contains("left controller is asleep"));
        assert_eq!(
            readiness_change([true, false], [true, false], name),
            None,
            "no repeats while it sleeps"
        );
        assert!(readiness_change([true, false], [true, true], name)
            .unwrap()
            .contains("right controller is asleep"));
        assert_eq!(
            readiness_change([true, true], [false, true], name),
            None,
            "the right one is still asleep"
        );
        assert_eq!(
            readiness_change([false, true], [false, false], name).as_deref(),
            Some("  Both awake. Press A on the right controller.")
        );
    }
    #[test]
    fn preparation_resumes_after_sleep_or_stale_only_with_a_fresh_release_press() {
        let ids = saved().identities.unwrap();
        for asleep in [true, false] {
            let mut edges = ButtonEdges::new(999_999_999);
            let mut raw = reload_inputs();
            assert_eq!(
                edges
                    .observe_preparation(&raw, &ids, 1_000_000_010)
                    .unwrap(),
                Some([0, 0])
            );
            if asleep {
                raw.hands[0].connected = false;
            }
            raw.hands[1].received_host_ns = 1_600_000_000;
            raw.hands[1].buttons = Some(cv1touch::hand::BTN_A);
            assert_eq!(
                edges
                    .observe_preparation(&raw, &ids, 1_600_000_010)
                    .unwrap(),
                None
            );
            raw.hands[0].connected = true;
            raw.hands[0].received_host_ns = 1_600_000_020;
            raw.hands[1].received_host_ns = 1_600_000_020;
            assert_eq!(
                edges
                    .observe_preparation(&raw, &ids, 1_600_000_030)
                    .unwrap(),
                Some([0, 0]),
                "wake/held press must not start"
            );
            raw.hands[1].buttons = Some(0);
            assert_eq!(
                edges
                    .observe_preparation(&raw, &ids, 1_600_000_040)
                    .unwrap(),
                Some([0, 0]),
                "duplicate/queued release must not arm"
            );
            raw.hands[1].buttons = Some(cv1touch::hand::BTN_A);
            raw.hands[1].received_host_ns = 1_600_000_050;
            assert_eq!(
                edges
                    .observe_preparation(&raw, &ids, 1_600_000_060)
                    .unwrap(),
                Some([0, 0])
            );
            raw.hands[1].buttons = Some(0);
            raw.hands[1].received_host_ns = 1_600_000_070;
            edges
                .observe_preparation(&raw, &ids, 1_600_000_080)
                .unwrap();
            raw.hands[1].buttons = Some(cv1touch::hand::BTN_A);
            raw.hands[1].received_host_ns = 1_600_000_090;
            assert_eq!(
                edges
                    .observe_preparation(&raw, &ids, 1_600_000_100)
                    .unwrap(),
                Some([0, cv1touch::hand::BTN_A])
            );
        }
    }
    #[test]
    fn preparation_cancels_on_fresh_right_b_even_when_left_is_sleeping() {
        let ids = saved().identities.unwrap();
        let mut raw = reload_inputs();
        raw.hands[0].connected = false;
        raw.hands[1].buttons = Some(cv1touch::hand::BTN_B);
        let error = ButtonEdges::new(0)
            .observe_preparation(&raw, &ids, 1_000_000_010)
            .unwrap_err();
        assert!(error.contains("cancelled with B"), "{error}");
    }
    #[test]
    fn preparation_keeps_identity_support_and_timestamp_errors_strict_while_waiting() {
        let ids = saved().identities.unwrap();
        for fault in 0..5 {
            let mut edges = ButtonEdges::new(0);
            let mut raw = reload_inputs();
            edges
                .observe_preparation(&raw, &ids, 1_000_000_010)
                .unwrap();
            raw.hands[0].connected = false;
            assert_eq!(
                edges
                    .observe_preparation(&raw, &ids, 1_000_000_020)
                    .unwrap(),
                None
            );
            match fault {
                0 => raw.hands[0].identity = "3".repeat(32),
                1 => raw.hands[0].buttons = None,
                2 => raw.hands[0].received_host_ns = 0,
                3 => raw.hands[0].received_host_ns = 2_000_000_000,
                _ => raw.hands[0].received_host_ns -= 1,
            }
            assert!(
                edges
                    .observe_preparation(&raw, &ids, 1_000_000_030)
                    .is_err(),
                "fault {fault}"
            );
        }
    }
    #[test]
    fn registration_starts_only_after_ready_countdown_and_verification() {
        let events = std::cell::RefCell::new(Vec::new());
        let result = registration_capture_after_preparation(
            &mut Checkpoint::default(),
            || {
                events.borrow_mut().push("ready");
                Ok(())
            },
            || {
                events.borrow_mut().push("ten-second countdown complete");
                Ok(())
            },
            |_| {
                events.borrow_mut().push("prerequisites verified");
                Ok(())
            },
            |_| {
                events.borrow_mut().push("record-start");
                Ok("capture")
            },
        )
        .unwrap();
        assert_eq!(result, "capture");
        assert_eq!(
            *events.borrow(),
            [
                "ready",
                "ten-second countdown complete",
                "prerequisites verified",
                "record-start"
            ]
        );
    }
    #[test]
    fn registration_quit_interrupt_or_failed_readiness_never_starts_recording() {
        for rejected_phase in 0..3 {
            let starts = std::cell::Cell::new(0);
            let gate = |phase| {
                if phase == rejected_phase {
                    Err("preparation cancelled or unavailable".into())
                } else {
                    Ok(())
                }
            };
            let result = registration_capture_after_preparation(
                &mut Checkpoint::default(),
                || gate(0),
                || gate(1),
                |_| gate(2),
                |_| {
                    starts.set(starts.get() + 1);
                    Ok(())
                },
            );
            assert!(
                result.is_err(),
                "rejected preparation phase {rejected_phase}"
            );
            assert_eq!(
                starts.get(),
                0,
                "rejected preparation phase {rejected_phase}"
            );
        }
    }
    #[test]
    fn registration_preparation_rejects_changed_or_unready_prerequisites() {
        let before = saved();
        assert!(verify_registration_preparation(&before, &before).is_ok());
        let mut after = saved();
        after.generation = Some(2);
        assert!(verify_registration_preparation(&before, &after).is_err());
        after = saved();
        after.identities.as_mut().unwrap()[0] = "3".repeat(32);
        assert!(verify_registration_preparation(&before, &after).is_err());
        after = saved();
        after.snapshot.camera = Artifact::Valid("b".repeat(64));
        assert!(verify_registration_preparation(&before, &after).is_err());
        after = saved();
        after
            .snapshot
            .blocked
            .insert(Stage::Registration, "controllers asleep".into());
        assert!(verify_registration_preparation(&before, &after).is_err());
        after = saved();
        after.sensors.insert("NEWCAM".into());
        assert!(verify_registration_preparation(&before, &after).is_err());
    }
    #[test]
    fn recording_finish_requires_release_press_after_start_cue() {
        let before = saved();
        let ids = before.identities.as_ref().unwrap();
        let mut feedback = RegistrationFeedback {
            before: &before,
            edges: ButtonEdges::new(0),
        };
        let mut raw = reload_inputs();
        feedback.edges.observe(&raw, ids, 1_000_000_010).unwrap();
        feedback.rearm_finish(1_000_000_100);
        // A pressed before the recorder acknowledgement/start cue is queued, not a finish.
        raw.hands[HAND_RIGHT].buttons = Some(cv1touch::hand::BTN_A);
        raw.hands[HAND_RIGHT].received_host_ns = 1_000_000_050;
        assert_eq!(
            feedback.edges.observe(&raw, ids, 1_000_000_110).unwrap()[HAND_RIGHT],
            0
        );
        raw.hands[HAND_RIGHT].received_host_ns = 1_000_000_120;
        assert_eq!(
            feedback.edges.observe(&raw, ids, 1_000_000_130).unwrap()[HAND_RIGHT],
            0
        );
        raw.hands[HAND_RIGHT].buttons = Some(0);
        raw.hands[HAND_RIGHT].received_host_ns = 1_000_000_140;
        feedback.edges.observe(&raw, ids, 1_000_000_150).unwrap();
        raw.hands[HAND_RIGHT].buttons = Some(cv1touch::hand::BTN_A);
        raw.hands[HAND_RIGHT].received_host_ns = 1_000_000_160;
        assert_eq!(
            feedback.edges.observe(&raw, ids, 1_000_000_170).unwrap()[HAND_RIGHT],
            cv1touch::hand::BTN_A
        );
    }
    fn reload_sample(generation: u64, frames: u32) -> serde_json::Value {
        serde_json::json!({"generation": generation, "mode": "calibrated", "recording": false, "engine_ready": true,
            "sensors": [{"serial": "CAMTEST", "placed": 1, "frames": frames, "fixes": [0,0]}]})
    }
    fn reload_inputs() -> RawInputs {
        RawInputs {
            hands: std::array::from_fn(|hand| RawInput {
                buttons: Some(0),
                identity: (hand + 1).to_string().repeat(32),
                connected: true,
                received_host_ns: 1_000_000_000,
                trigger: 0.,
                grip: 0.,
                stick: [0.; 2],
            }),
        }
    }
    fn simulated_engine(
        expected_mode: &str,
        samples: Vec<(Option<serde_json::Value>, Option<RawInputs>)>,
    ) -> Result<(), String> {
        let mut samples = samples.into_iter();
        let tick = std::cell::Cell::new(0);
        wait_reload_with(
            &saved(),
            expected_mode,
            Duration::from_secs(1),
            || {
                let (status, raw) = samples.next().unwrap_or((None, None));
                Ok((status, raw, 1_000_000_010))
            },
            || Duration::from_millis(tick.get()),
            || tick.set(tick.get() + 100),
        )
    }
    fn simulated_reload(
        samples: Vec<(Option<serde_json::Value>, Option<RawInputs>)>,
    ) -> Result<(), String> {
        simulated_engine("calibrated", samples)
    }
    #[test]
    fn reload_acknowledgement_must_survive_transient_disconnected_publication() {
        let disconnected = serde_json::json!({"generation": 1, "mode": "calibrated", "recording": false, "engine_ready": false, "sensors": []});
        let mut cleared = reload_inputs();
        for hand in &mut cleared.hands {
            hand.identity.clear();
            hand.connected = false;
        }
        assert!(simulated_reload(vec![
            (Some(disconnected), Some(cleared)),
            (Some(reload_sample(1, 1)), Some(reload_inputs())),
            (Some(reload_sample(1, 2)), Some(reload_inputs())),
        ])
        .is_ok());
    }
    #[test]
    fn capture_mode_waits_for_applied_engine_without_requiring_placed_sensors() {
        let samples = (1..3)
            .map(|frames| {
                let mut status = reload_sample(1, frames);
                status["mode"] = "capture".into();
                status["sensors"][0]["placed"] = 0.into();
                (Some(status), Some(reload_inputs()))
            })
            .collect();
        assert!(simulated_engine("capture", samples).is_ok());
    }
    #[test]
    fn reload_outage_or_stopped_streams_time_out() {
        assert!(simulated_reload(vec![]).unwrap_err().contains("timed out"));
        let stopped = (0..10)
            .map(|_| (Some(reload_sample(1, 1)), Some(reload_inputs())))
            .collect();
        assert!(simulated_reload(stopped).unwrap_err().contains("timed out"));
    }
    #[test]
    fn reload_requires_matching_daemon_readiness_capability() {
        let mut legacy = reload_sample(1, 1);
        legacy.as_object_mut().unwrap().remove("engine_ready");
        assert!(
            simulated_reload(vec![(Some(legacy), Some(reload_inputs()))])
                .unwrap_err()
                .contains("matching cv1touchd")
        );
    }
    #[test]
    fn reload_cannot_accept_old_engine_frames_before_reopen_applied() {
        let samples = (1..11)
            .map(|frames| {
                let mut status = reload_sample(1, frames);
                status["engine_ready"] = false.into();
                (Some(status), Some(reload_inputs()))
            })
            .collect();
        assert!(simulated_reload(samples).unwrap_err().contains("timed out"));
    }
    #[test]
    fn reload_restart_or_replaced_controller_is_fatal() {
        assert!(
            simulated_reload(vec![(Some(reload_sample(2, 1)), Some(reload_inputs()))])
                .unwrap_err()
                .contains("restarted")
        );
        let mut replaced = reload_inputs();
        replaced.hands[1].identity = "3".repeat(32);
        assert!(
            simulated_reload(vec![(Some(reload_sample(1, 1)), Some(replaced))])
                .unwrap_err()
                .contains("controller changed")
        );
    }
    #[test]
    fn reload_requires_original_sensor_set_and_fresh_inputs() {
        let mut changed = reload_sample(1, 1);
        changed["sensors"][0]["serial"] = "NEWCAM".into();
        let samples = (0..10)
            .map(|_| (Some(changed.clone()), Some(reload_inputs())))
            .collect();
        assert!(simulated_reload(samples).unwrap_err().contains("timed out"));
        let samples = (1..11)
            .map(|frames| {
                let mut asleep = reload_inputs();
                asleep.hands[0].connected = false;
                (Some(reload_sample(1, frames)), Some(asleep))
            })
            .collect();
        assert!(simulated_reload(samples).unwrap_err().contains("timed out"));
    }
    #[test]
    fn registration_cannot_complete_against_a_world_changed_during_fit() {
        let before = saved();
        let mut after = saved();
        after.snapshot.camera = Artifact::Valid("b".repeat(64));
        assert!(verify_fitted_dependencies(Stage::Registration, &before, &after).is_err());
        assert!(verify_fitted_dependencies(Stage::Render, &before, &after).is_ok());
        assert!(verify_fitted_dependencies(Stage::Registration, &before, &before).is_ok());
    }
    #[test]
    fn render_cannot_complete_against_replaced_meshes_or_controllers() {
        let before = saved();
        let mut after = saved();
        after.snapshot.render_sources = Artifact::Valid("b".repeat(64));
        assert!(verify_fitted_dependencies(Stage::Render, &before, &after).is_err());
        after = saved();
        after.identities.as_mut().unwrap()[1] = "3".repeat(32);
        assert!(verify_fitted_dependencies(Stage::Render, &before, &after).is_err());
        after = saved();
        after.generation = Some(2);
        assert!(verify_fitted_dependencies(Stage::Registration, &before, &after).is_err());
    }
}
