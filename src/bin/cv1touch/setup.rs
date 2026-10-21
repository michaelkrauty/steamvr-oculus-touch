// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Interactive orchestration; the library planner has no hardware side effects.
use super::*;
use cv1touch::headset_watch::{Cue, HeadsetCapture};
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

pub(super) fn journal_path() -> PathBuf {
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

fn read_status() -> io::Result<Option<serde_json::Value>> {
    let reply = daemon_request("status", Duration::from_millis(750))?;
    Ok(reply
        .strip_prefix("ok ")
        .and_then(|json| serde_json::from_str(json).ok()))
}

/// Connected sensors the calibration has no pose for. It may spell a serial in another letter
/// case than the device does, as the engine allows.
fn unplaced_sensors(
    calibration: Option<&file::CalibrationFile>,
    sensors: &[shm::SensorStatus],
) -> Vec<String> {
    sensors
        .iter()
        .map(shm::serial_str)
        .filter(|serial| calibration.is_none_or(|cal| cal.sensor(serial).is_none()))
        .collect()
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
    let first = read_status().ok().flatten();
    thread::sleep(Duration::from_millis(300));
    let second = read_status();
    let daemon_detail = second.as_ref().err().map_or_else(
        || "Start cv1touchd with access to the CV1 USB devices.".to_string(),
        |error| {
            if error.kind() == io::ErrorKind::InvalidInput {
                error.to_string()
            } else {
                format!("Start cv1touchd with access to the CV1 USB devices. ({error})")
            }
        },
    );
    let second = second.ok().flatten();
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
                    status::same_sensor(&shm::serial_str(previous), &shm::serial_str(sensor))
                        && sensor.frames > previous.frames
                })
            })
    });
    let sensors = second
        .as_ref()
        .and_then(|v| status::sensors_from_status(v).ok())
        .unwrap_or_default();
    let unplaced = unplaced_sensors(calibration.as_ref(), &sensors);
    let recording = second
        .as_ref()
        .is_some_and(|v| v["recording"] == true || v["recording_status"]["pending"] == true);
    let normal_mode = second
        .as_ref()
        .is_some_and(|v| matches!(v["mode"].as_str(), Some("calibrated" | "provisional")));
    let runtime = steamvr::runtime_root();
    let runtime_ok = vr::runtime_library().is_file()
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
        Check { name: "daemon", ok: second.is_some(), detail: daemon_detail },
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
        sensors: sensors
            .iter()
            .map(|s| status::sensor_identity(&shm::serial_str(s)))
            .collect(),
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

/// Whether setup is complete, the stages, and `setup --check --json`'s report.
fn report_json(
    inspection: &Inspection,
    checkpoint: &Checkpoint,
) -> (bool, Vec<setup::StageReport>, serde_json::Value) {
    let stages = reports(inspection, checkpoint);
    let ready =
        stages.iter().all(|r| r.state == State::Ready) && inspection.checks.iter().all(|c| c.ok);
    let json = serde_json::json!({"ready": ready, "checks": inspection.checks, "stages": stages, "note": "Saved dependency reuse does not verify physical alignment. Setup never pairs or resets controllers."});
    (ready, stages, json)
}

/// `setup --check --json`'s report, or why there is none.
pub(super) fn check_json() -> Result<serde_json::Value, String> {
    let checkpoint = setup::load(&journal_path())
        .map_err(|e| format!("setup journal: {e}"))?
        .unwrap_or_default();
    Ok(report_json(&inspect(&checkpoint), &checkpoint).2)
}

fn print_report(inspection: &Inspection, checkpoint: &Checkpoint, json: bool) -> bool {
    let (ready, stages, report) = report_json(inspection, checkpoint);
    if json {
        println!("{}", serde_json::to_string_pretty(&report).unwrap());
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
    // not flushed: a stray Enter here only advances to a step that itself waits for a deliberate
    // start (go_signal, input_gate and the recordings discard typed-ahead input)
    human_action(prompt);
    io::stdout().flush().map_err(|e| e.to_string())?;
    loop {
        if interrupted() {
            return Err("setup interrupted; completed steps are kept".into());
        }
        match console::next_line(Duration::from_millis(100)) {
            Line::Closed => {
                return Err("setup requires an interactive terminal; input closed".into())
            }
            Line::Typed(line) => {
                let answer = line.trim().to_ascii_lowercase();
                if allowed.contains(&answer.as_str()) {
                    return Ok(answer);
                }
                human_action(&ui::hint(allowed));
            }
            Line::NotYet => {}
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
        let Ok(log) = platform::fs::append_private(&self.path) else {
            return f();
        };
        let Ok(_to_log) = console::redirect_output(&log) else {
            return f();
        };
        f()
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
        let error = self.last_error().unwrap_or_else(|| fallback.to_owned());
        if self.path.is_file() {
            format!("{error} (details: {})", self.path.display())
        } else {
            error
        }
    }
}

/// `send` without echoing the daemon's reply; 0 on "ok".
fn send_quiet(line: &str) -> i32 {
    match daemon_request(line, Duration::from_secs(5)) {
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

/// Asks at a prompt for one of the `offered` answers.
fn ask(prompt: &str, offered: &[ui::Answer]) -> Result<ui::Answer, String> {
    let keys: Vec<&str> = offered.iter().flat_map(|a| a.words()).copied().collect();
    let line = choice(prompt, &keys)?;
    ui::answer(&line, offered).ok_or_else(|| "unrecognised answer".to_string())
}

const START_OR_QUIT: &[ui::Answer] = &[ui::Answer::Start, ui::Answer::Quit];

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
    discard_typed_ahead();
    human_action(prompt);
    let mut a = PressA::new();
    loop {
        if interrupted() {
            return Err("interrupted; nothing was recorded".into());
        }
        match console::next_line(Duration::from_millis(20)) {
            Line::Closed => {
                return Err("setup requires an interactive terminal; input closed".into())
            }
            Line::Typed(line) => match start_line(&line) {
                Some(true) => return Ok(()),
                Some(false) => return Err("stopped; nothing was recorded".into()),
                None => println!("  (Enter or A starts, q quits)"),
            },
            Line::NotYet => {}
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
            return Err("interrupted; your previous calibration is unchanged".into());
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

const CAMERA_MOTION: &str = "This works out where each of your Rift sensors is in the room. The sensors find their places by\nwatching the lights in the controllers' rings, and moments when two sensors see a controller at\nonce matter most. For 60 seconds, until the double beep, slowly wave both controllers around\nthe whole play area: high and low, close to each sensor and far from it, turning your wrists so\nthe rings face every way.";

const GRAVITY_HOLDS: &str = "This finds which way is down, so the floor in VR is level. Each controller can feel gravity, but\nonly while it is held perfectly still, and only accurately when that is done in many different\npositions. Keep the rings where the sensors can see them.\nDo this 12 times: turn and tilt both controllers into a new position, hold them still, press A\n(or Enter), and keep them still until the double beep (3 seconds).";

const RENDER_WAKE: &str = "This lines up the controller you see in VR with the real one: each controller reports where its\nlights are, and the guide moves SteamVR's 3D model to match. Nothing to aim or hold: just keep\nboth controllers awake (move them a little now and then) for 5 seconds, until the double beep.";

fn timed_capture(
    stage: Stage,
    checkpoint: &mut Checkpoint,
    gravity: bool,
) -> Result<PathBuf, String> {
    let rec = capture_path(stage, checkpoint)?;
    let mut guard = start_recording(&rec, false)?;
    let result = if gravity {
        human_action(GRAVITY_HOLDS);
        (1..=12).try_for_each(|hold| -> Result<(), String> {
            go_signal(&format!(
                "  Hold {hold}/12: still, in a new orientation, then A."
            ))?;
            timed_window(3)
        })
    } else if stage == Stage::Render {
        go_signal(&format!("{RENDER_WAKE}\n{START}")).and_then(|()| timed_window(5))
    } else {
        go_signal(&format!("{CAMERA_MOTION}\n{START}")).and_then(|()| timed_window(60))
    };
    let stopped =
        daemon_request(&guard.command, RECORD_STOP_TIMEOUT).map_err(|error| error.to_string());
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
        return Err("setup interrupted; your previous calibration is unchanged".into());
    }
    let ids = inspection
        .identities
        .as_ref()
        .ok_or("controller identities unavailable")?;
    let raw = raw_inputs()?;
    input_identities_match(&raw, ids)?;
    PairedInputPhase::new([0, 0]).observe(&raw, ids, shm::now_ns())?;
    let status = read_status().map_err(|error| error.to_string())?.ok_or(
        "cv1touchd stopped responding after the recording; check it is running and try again",
    )?;
    if inspection.generation.is_none() || status["generation"].as_u64() != inspection.generation {
        return Err(
            "cv1touchd restarted during the recording; your previous calibration is unchanged"
                .into(),
        );
    }
    if status["recording"] != false {
        return Err(
            "another recording is still running; stop it (cv1touch record stop) and try again"
                .into(),
        );
    }
    if let Artifact::Valid(before) = &inspection.snapshot.camera {
        let now = setup::fingerprint(&file::default_path()).map_err(|e| e.to_string())?;
        if &now != before {
            return Err(
                "the sensor calibration changed during the recording; your previous calibration is unchanged"
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
        return Err("cv1touchd restarted or a different controller connected meanwhile; your previous calibration is unchanged".into());
    }
    if stage == Stage::Render && after.snapshot.render_sources != before.snapshot.render_sources {
        return Err("SteamVR's controller model files changed meanwhile; try again".into());
    }
    if stage == Stage::Registration && after.snapshot.camera != before.snapshot.camera {
        return Err("the sensor calibration changed meanwhile; try again".into());
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
            return Err("cv1touchd restarted unexpectedly; try again".into());
        }
        if status
            .as_ref()
            .is_some_and(|v| v["engine_ready"].as_bool().is_none())
        {
            return Err(
                "this cv1touchd is older than cv1touch; install both from the same build".into(),
            );
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
                        "a different controller connected meanwhile; use the same two controllers and try again"
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
                    && sensors
                        .iter()
                        .map(|s| status::sensor_identity(&shm::serial_str(s)))
                        .collect::<BTreeSet<_>>()
                        == before.sensors
                    && sensors.len() == before.sensors.len()
            })
            .map(|sensors| {
                sensors
                    .into_iter()
                    .map(|sensor| {
                        (
                            status::sensor_identity(&shm::serial_str(&sensor)),
                            sensor.frames,
                        )
                    })
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
    Err("tracking did not come back after restarting (a sensor not sending images, or a controller asleep); wake both controllers, check the sensors' USB cables and try again".into())
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
                return Err("setup interrupted; this step was not saved".into());
            }
            let status = match read_status() {
                Err(error) if error.kind() == io::ErrorKind::InvalidInput => {
                    return Err(error.to_string())
                }
                result => result.ok().flatten(),
            };
            let raw = raw_inputs().ok();
            if interrupted() {
                return Err("setup interrupted; this step was not saved".into());
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
            let buttons = sample
                .buttons
                .ok_or("this cv1touchd is older than cv1touch; install both from the same build")?;
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
            let buttons = sample
                .buttons
                .ok_or("this cv1touchd is older than cv1touch; install both from the same build")?;
            if !sample.connected
                || sample.received_host_ns == 0
                || now_ns
                    .checked_sub(sample.received_host_ns)
                    .is_none_or(|age| age > FRESH_INPUT_NS)
            {
                return Err("a controller stopped responding (asleep or out of range); your previous calibration is unchanged".into());
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

const REGISTRATION_PREPARATION: &str =
    "Headset alignment: this lines up the controllers' tracking with the headset's, so in VR the
controllers appear exactly where your hands are. You'll wear the headset and hold both
controllers against it while you turn around and move your head, for about a minute.
Read all of this now: you won't see this screen with the headset on.
  1. Put on the headset (SteamVR running).
  2. Press the real controllers firmly against the outside of the headset: the left controller
     against the left side, the right controller against the right side. Keep your hands off
     the controllers' rings and the headset's sensors. From now until the end, the controllers
     must not slide or twist on the headset: if one does, the calibration fails and you start
     this step again.
  3. Press A on the right controller with your thumb. You'll feel a 10-second countdown via
     controller haptic pulses, one a second. Nothing is recorded yet: use the 10 seconds to
     press both controllers firmly back into place, in case pressing A moved them.
  4. Two quick pulses: recording starts. Slowly turn your whole body around on the spot:
     two full circles to your left, then two to your right, 5 to 10 seconds per circle.
     Keep turning until you feel a single pulse.
  5. Then look up and down, and tilt your head toward each shoulder, as far as is comfortable,
     a few times each, until you feel one long pulse: done. Take off the headset.
If you feel three quick pulses instead, the recording stopped early (a controller slid on the
headset, you pressed B, or something else went wrong): take off the headset and read this
screen. B on the right controller cancels at any time. What you see in the headset during this
step doesn't matter.";

/// The controllers' raw inputs, with the daemon, its camera world and sensor placement checked
/// unchanged since `before`; the samples themselves are not judged here.
fn registration_context(before: &Inspection) -> Result<RawInputs, String> {
    if interrupted() {
        return Err("interrupted; your previous calibration is unchanged".into());
    }
    let reply = daemon_request("inputs", Duration::from_millis(500)).map_err(|e| e.to_string())?;
    let raw = parse_raw_inputs(&reply)?;
    let reply = daemon_request("status", Duration::from_millis(500)).map_err(|e| e.to_string())?;
    let status: serde_json::Value = serde_json::from_str(
        reply
            .strip_prefix("ok ")
            .ok_or("cannot reach cv1touchd; check it is running")?,
    )
    .map_err(|e| e.to_string())?;
    if before.generation.is_none() || status["generation"].as_u64() != before.generation {
        return Err("cv1touchd restarted; your previous calibration is unchanged".into());
    }
    let sensors = status::sensors_from_status(&status)?;
    if status["mode"] != "calibrated"
        || status["engine_ready"] == false
        || sensors.len() != before.sensors.len()
        || sensors.iter().any(|sensor| sensor.placed == 0)
        || sensors
            .iter()
            .map(|s| status::sensor_identity(&shm::serial_str(s)))
            .collect::<BTreeSet<_>>()
            != before.sensors
    {
        return Err(
            "the sensors or their calibration changed; your previous calibration is unchanged"
                .into(),
        );
    }
    if before.snapshot.camera
        != Artifact::Valid(setup::fingerprint(&file::default_path()).map_err(|e| e.to_string())?)
    {
        return Err(
            "the sensor calibration changed; your previous calibration is unchanged".into(),
        );
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
        return Err(
            "this cv1touchd is older than cv1touch; install both from the same build".into(),
        );
    }
    if raw.hands[HAND_RIGHT]
        .buttons
        .is_some_and(|buttons| buttons & cv1touch::hand::BTN_B != 0)
    {
        return Err("cancelled with B; your previous calibration is unchanged".into());
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
    let reply = daemon_request(&command, Duration::from_millis(500)).map_err(|e| {
        format!("a haptic pulse could not be sent ({e}); check both controllers are awake")
    })?;
    if reply.trim() != "ok" {
        return Err(format!(
            "a haptic pulse was refused ({reply}); check both controllers are awake"
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
    Err("no button press within 5 minutes; nothing was recorded".into())
}

/// The line to show when the controllers' readiness changes: which one fell asleep, or, once
/// both are back, the press still awaited. None while nothing changed.
fn readiness_change(shown: [bool; 2], waiting: [bool; 2], name: &str) -> Option<String> {
    let names = ["left", "right"];
    if let Some(hand) = (0..2).find(|&h| waiting[h] && !shown[h]) {
        // never "any button": B on the right controller cancels
        return Some(format!(
            "  The {} controller is asleep: press {} or its trigger to wake it.",
            names[hand],
            ["X", "A"][hand]
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
        human_action(&format!("  Felt the pulse? Press {key} again."));
        press(hand, button, name)?;
    }
    Ok(())
}

/// The countdown's line on screen, `remaining` seconds before the recording starts.
fn countdown_line(remaining: usize) -> String {
    format!("\r  recording starts in {remaining:>2} s ")
}

fn registration_countdown_with(
    mut tick: impl FnMut(usize) -> Result<(), String>,
    mut second: impl FnMut() -> Result<(), String>,
) -> Result<(), String> {
    let result = (|| {
        for remaining in (1..=10).rev() {
            print!("{}", countdown_line(remaining));
            let _ = io::stdout().flush();
            tick(HAND_LEFT)?;
            tick(HAND_RIGHT)?;
            second()?;
        }
        Ok(())
    })();
    if result.is_err() {
        // end the countdown line before the error
        println!();
    }
    result
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

/// Whether the headset alignment watches for a controller slipping: `setup --no-slip-check`, or
/// the answer to a slip, turns it off for the rest of the guide.
pub(super) static SLIP_CHECK: AtomicBool = AtomicBool::new(true);
/// Set when the last headset alignment stopped because a controller slipped.
static SLIPPED: AtomicBool = AtomicBool::new(false);

/// `pulses` buzzes of `seconds` in both controllers, 0.1 s apart.
fn registration_buzz(
    pulses: u32,
    seconds: f32,
    frequency: f32,
    amplitude: f32,
) -> Result<(), String> {
    for pulse in 0..pulses {
        if pulse > 0 {
            thread::sleep(Duration::from_secs_f32(seconds + 0.1));
        }
        for hand in [HAND_LEFT, HAND_RIGHT] {
            registration_haptic(hand, seconds, frequency, amplitude)?;
        }
    }
    Ok(())
}

/// The headset recording: stops by itself once the person has turned, nodded and tilted enough
/// (a long buzz), or with three quick buzzes when a controller moves on the headset (with the slip
/// check), B cancels, or anything else goes wrong.
struct RegistrationFeedback<'a> {
    before: &'a Inspection,
    edges: ButtonEdges,
    capture: HeadsetCapture,
    /// What ended the recording early, in the person's words.
    error: Option<String>,
}

impl<'a> RegistrationFeedback<'a> {
    fn new(before: &'a Inspection, slip_check: bool) -> Self {
        Self {
            before,
            edges: ButtonEdges::new(shm::now_ns()),
            capture: HeadsetCapture::new(slip_check),
            error: None,
        }
    }

    /// Keeps the first error, and tells the person wearing the headset at once: three quick
    /// buzzes, whatever went wrong.
    fn noted<T>(&mut self, result: Result<T, String>) -> Result<T, String> {
        if let Err(error) = &result {
            if self.error.is_none() {
                self.error = Some(error.clone());
                // the error says what matters; a buzz that fails too adds nothing
                let _ = registration_buzz(3, 0.1, 160., 0.8);
            }
        }
        result
    }

    fn check(&mut self) -> Result<bool, String> {
        let raw = registration_inputs(self.before)?;
        let ids = self
            .before
            .identities
            .as_ref()
            .ok_or("controller identities unavailable")?;
        // the controllers stay awake and their samples keep moving forward
        self.edges.observe(&raw, ids, shm::now_ns())?;
        match self.capture.cue() {
            Cue::Slipped(hand, deg) => Err(format!(
                "the {} controller slid on the headset (it turned {deg:.0}°); your previous calibration is unchanged",
                ["left", "right"][hand]
            )),
            Cue::Turned => registration_buzz(1, 0.25, 160., 0.65).map(|()| false),
            Cue::Done => Ok(true),
            Cue::Recording => Ok(false),
        }
    }
}

impl CaptureFeedback for RegistrationFeedback<'_> {
    fn before_start(&mut self) -> Result<(), String> {
        let result = registration_inputs(self.before).and_then(|raw| {
            let ids = self
                .before
                .identities
                .as_ref()
                .ok_or("controller identities unavailable")?;
            self.edges.observe(&raw, ids, shm::now_ns()).map(|_| ())
        });
        self.noted(result)
    }
    fn started(&mut self) -> Result<(), String> {
        let result = registration_buzz(2, 0.15, 160., 0.65);
        self.noted(result)
    }
    fn pose(&mut self, frame: &PoseFrame) {
        self.capture.push(frame.t_ns, frame.hmd, frame.touch);
    }
    fn poll(&mut self) -> Result<bool, String> {
        let result = self.check();
        self.noted(result)
    }
    fn stopped(&mut self) -> Result<(), String> {
        // a recording stopped by an error has had its three buzzes
        if self.error.is_some() {
            return Ok(());
        }
        let result = registration_buzz(1, 0.7, 80., 0.45);
        self.noted(result)
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
        return Err("the sensors changed meanwhile (one connected, disconnected or recalibrated); nothing was recorded".into());
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
        return Err(
            "this cv1touchd is older than cv1touch; install both from the same build (nothing was recorded)".into(),
        );
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
                    return Err(log.failure("the new sensors could not be placed; your previous calibration is unchanged"));
                }
            }
            log.quiet(|| restore_mode(&file::default_path()))
        }
        Stage::Camera => {
            let mut restore = RestoreCamera(true);
            if log.quiet(|| send_quiet("calibration capture")) != 0 {
                return Err("cv1touchd did not switch to sensor calibration; try again".into());
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
                    // the recording reads the controllers' poses: fail now, not after the countdown
                    touch_reader()?;
                    human_action("Button and pulse check: with the headset on you'll press A and go by haptic pulses, so first\nmake sure both work. B on the right controller cancels at any point.");
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
                        return Err("setup interrupted; nothing was recorded".into());
                    }
                    verify_registration_preparation(inspection, &current)
                },
                |checkpoint| {
                    if interrupted() {
                        return Err("setup interrupted; nothing was recorded".into());
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
                        return Err("setup interrupted; nothing was recorded".into());
                    }
                    print!("\r                            \r");
                    let mut feedback =
                        RegistrationFeedback::new(inspection, SLIP_CHECK.load(Ordering::Relaxed));
                    let captured = capture_free_named_with_feedback(
                        150,
                        "Recording: two circles left, two right; after the single pulse, look up and down and tilt your head until the long pulse. B cancels.",
                        false,
                        Some(&base),
                        Some(&mut feedback),
                    );
                    SLIPPED.store(feedback.capture.slipped(), Ordering::Relaxed);
                    captured.map_err(|code| {
                        feedback.error.take().unwrap_or_else(|| {
                            format!("the recording failed ({code}); your previous calibration is unchanged")
                        })
                    })?;
                    Ok((rec, base))
                },
            )?;
            recordings.push(rec);
            recordings.push(base.with_extension("truth.json"));
            verify_capture(inspection)?;
            log.quiet(|| {
                let code = register_from_capture_with(&base, true, Some("hmd"), true, false, None);
                if code == 0 {
                    send_quiet("alignment reload")
                } else {
                    code
                }
            })
        }
        // its own errors say what went wrong (a controller asleep, travel too short)
        Stage::Inputs => calibrate_inputs_result(None, false, true).map(|()| 0)?,
    };
    if interrupted() {
        return Err("interrupted; your previous calibration is unchanged".into());
    }
    if code != 0 {
        return Err(log.failure("the calibration failed; your previous calibration is unchanged"));
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
        return Err("setup interrupted; this step was not saved".into());
    }
    if stage == Stage::Camera && !preserve {
        let accepted = accepted_camera_sha256
            .as_deref()
            .ok_or("the new sensor calibration could not be read back; try again")?;
        setup::record_full_camera_success(&next.snapshot, checkpoint, accepted, recordings)?;
    } else {
        setup::record_success(stage, &next.snapshot, checkpoint, recordings)?;
    }
    persist(checkpoint)
}

pub(super) fn run(check_only: bool, json: bool, redo: Option<Stage>) -> i32 {
    if !check_only && !io::stdin().is_terminal() {
        eprintln!("cv1touch: setup needs an interactive terminal. Use setup --check [--json] for a read-only report.");
        return 2;
    }
    // Refuse an unaddressable endpoint or an older running daemon before preparation. A daemon
    // that is not running or is busy is for the guide's own checks to explain and retry, and
    // every capture looks again before it starts.
    if !check_only {
        let status = match status_json() {
            Err(error) if error.kind() == io::ErrorKind::InvalidInput => {
                eprintln!("{}", cannot_reach(&control::socket_path(), &error));
                return 1;
            }
            result => result.ok().flatten(),
        };
        if let Some(Err(error)) = status.map(|s| recording_capability(&s, cfg!(windows))) {
            eprintln!("cv1touch: {error}");
            return 1;
        }
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
        if let Some(stage) = redo {
            // offered again (redo, or keep): pending outranks a result that still validates, as
            // for the buttons stage. In memory only: the journal is written when the stage starts
            // or is kept, so quitting at the prompt changes nothing on disk.
            checkpoint.completed.remove(&stage);
            checkpoint.pending.insert(stage);
        }
        let g = glyphs();
        let mut shown_checks = String::new();
        // the stage list shows at the start and again after a failure; a finished stage is one
        // result line, then the next prompt
        let mut show_list = true;
        // a stage the person chose to try again starts without asking twice
        let mut retry = None;
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
                if let Some(c) = inspection
                    .checks
                    .iter()
                    .find(|c| c.name == "tracking_configuration" && !c.ok)
                {
                    println!(
                        "{} The daemon is not using the saved setup yet: {}",
                        g.fail, c.detail
                    );
                    if ask(
                        "Fix that, then press Enter to check again (q: quit).",
                        START_OR_QUIT,
                    )? == ui::Answer::Quit
                    {
                        return Err("setup is saved but the daemon has not loaded it".into());
                    }
                    continue;
                }
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
                    START_OR_QUIT,
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
            let offered: &[ui::Answer] = if can_adopt {
                &[ui::Answer::Start, ui::Answer::Keep, ui::Answer::Quit]
            } else {
                START_OR_QUIT
            };
            let answer = if retry.take() == Some(report.stage) {
                ui::Answer::Start
            } else {
                ask(&ui::stage_prompt(report.stage, can_adopt), offered)?
            };
            match answer {
                ui::Answer::Quit => {
                    return Err("setup stopped before finishing; completed stages are kept".into())
                }
                ui::Answer::Keep => {
                    setup::adopt(report.stage, &inspection.snapshot, &mut checkpoint)?;
                    persist(&checkpoint)?;
                    println!("{} {}: kept", g.done, ui::stage_name(report.stage));
                }
                // (Unwatched is offered only after a slip, and turns the check off there)
                ui::Answer::Start | ui::Answer::Unwatched => {
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
                            let slipped = SLIPPED.swap(false, Ordering::Relaxed);
                            let offered: &[ui::Answer] = if slipped {
                                &[ui::Answer::Start, ui::Answer::Unwatched, ui::Answer::Quit]
                            } else {
                                START_OR_QUIT
                            };
                            match ask(ui::retry_prompt(slipped), offered)? {
                                ui::Answer::Quit => {
                                    return Err(format!(
                                        "{name} not finished; completed stages are kept"
                                    ))
                                }
                                ui::Answer::Unwatched => SLIP_CHECK.store(false, Ordering::Relaxed),
                                ui::Answer::Start | ui::Answer::Keep => {}
                            }
                            retry = Some(report.stage);
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
    fn a_sensor_calibrated_in_another_letter_case_is_placed() {
        let sensor = |serial: &str| {
            let mut s = shm::SensorStatus::default();
            shm::set_serial(&mut s, serial);
            s
        };
        let entry = |serial: &str| file::SensorEntry {
            serial: serial.into(),
            orient_xyzw: [0.0, 0.0, 0.0, 1.0],
            pos: [0.0, 2.0, 1.0],
            rms_px: 0.3,
            observations: 100,
        };
        let calibration = file::CalibrationFile {
            format: file::FORMAT.into(),
            version: file::VERSION,
            created_unix_s: 1,
            world: "gravity".into(),
            source: "a test".into(),
            sensors: vec![entry("camtest")],
            stats: file::Stats {
                reference: "test".into(),
                gravity_mps2: 9.81,
                gravity_spread_deg: None,
                motion_segments: 1,
                samples: 100,
                rejected_views: 0,
            },
        };
        let connected = [sensor("CAMTEST"), sensor("NEWCAM")];
        assert_eq!(unplaced_sensors(Some(&calibration), &connected), ["NEWCAM"]);
        assert_eq!(unplaced_sensors(None, &connected), ["CAMTEST", "NEWCAM"]);
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
                        || Err("cancelled during the countdown; nothing was recorded".into()),
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
    fn the_guides_messages_avoid_engineer_speak() {
        // everything after a stage's ✗ is read by someone who just wants it to work
        let source = include_str!("setup.rs");
        let code = &source[..source.find("#[cfg(test)]").unwrap()];
        for (n, line) in code.lines().enumerate() {
            let Some(quote) = line.find('"') else {
                continue;
            };
            if line.trim_start().starts_with("//") {
                continue;
            }
            for jargon in [
                "remains incomplete",
                "rerun setup",
                "retained",
                "no fit installed",
                "matching cv1touchd",
                "fitting",
                "timed out waiting",
                "stale or has",
            ] {
                assert!(!line[quote..].contains(jargon), "line {}: {jargon}", n + 1);
            }
        }
    }

    /// `text` with its line breaks and indents as single spaces.
    fn one_line(text: &str) -> String {
        text.split_whitespace().collect::<Vec<_>>().join(" ")
    }

    #[test]
    fn every_timed_step_says_what_it_measures_and_when_it_ends() {
        for (text, why) in [
            (CAMERA_MOTION, "where each of your Rift sensors is"),
            (GRAVITY_HOLDS, "so the floor in VR is level"),
            (
                RENDER_WAKE,
                "lines up the controller you see in VR with the real one",
            ),
            (INPUTS_GUIDE, "a full pull reads as a full pull"),
        ] {
            let text = one_line(text);
            assert!(text.contains(why), "{text}");
            assert!(text.contains("double beep"), "{text}");
        }
        for step in CalStep::ALL {
            assert!(step.guided().contains("until the double beep"), "{step:?}");
        }
    }

    #[test]
    fn headset_instructions_say_everything_before_the_headset_goes_on() {
        // once the headset is on the screen is out of sight: what to hold, when, how to move,
        // what each pulse means and why, and how it ends must all be read beforehand, in plain
        // words for someone who has never calibrated anything
        for needed in [
            "won't see this screen",
            "in VR the controllers appear exactly where your hands are",
            "real controllers",
            "the calibration fails",
            "10-second countdown via controller haptic pulses",
            "Nothing is recorded yet",
            "Two quick pulses: recording starts",
            "two full circles to your left, then two to your right",
            "seconds per circle",
            "single pulse",
            "look up and down",
            "tilt your head toward each shoulder",
            "one long pulse: done",
            "three quick pulses",
            "cancels at any time",
            "What you see in the headset during this step doesn't matter",
        ] {
            assert!(
                one_line(REGISTRATION_PREPARATION).contains(needed),
                "{needed}"
            );
        }
        // jargon, and asking for stillness nothing here measures
        for vague in ["fit", "settle", "grip", "still", "buzz"] {
            assert!(!REGISTRATION_PREPARATION.contains(vague), "{vague}");
        }
        assert!(!countdown_line(3).contains("still"));
        assert_eq!(countdown_line(3), "\r  recording starts in  3 s ");
        // it finishes by itself: pressing anything while holding the controllers to the
        // headset risks moving them
        assert!(!REGISTRATION_PREPARATION.contains("to finish"));
        // its cues are the controllers' haptics: buzzes; beeps are the speakers
        assert!(!REGISTRATION_PREPARATION.contains("beep"));
    }

    #[test]
    fn readiness_lines_only_on_a_dropout_and_the_recovery() {
        let name = "A on the right controller";
        assert_eq!(readiness_change([false; 2], [false; 2], name), None);
        let left = readiness_change([false; 2], [true, false], name).unwrap();
        let right = readiness_change([false; 2], [false, true], name).unwrap();
        assert!(
            left.contains("press X or its trigger") && right.contains("press A or its trigger")
        );
        assert!(
            !left.contains("any button") && !right.contains("any button"),
            "B would cancel"
        );
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
    fn mixed_case_reload_samples_keep_the_same_sensor() {
        let mut first = reload_sample(1, 10);
        first["sensors"][0]["serial"] = "camtest".into();
        let mut second = reload_sample(1, 11);
        second["sensors"][0]["serial"] = "CamTest".into();
        assert!(simulated_reload(vec![
            (Some(first), Some(reload_inputs())),
            (Some(second), Some(reload_inputs()))
        ])
        .is_ok());
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
        assert!(simulated_reload(vec![])
            .unwrap_err()
            .contains("did not come back"));
        let stopped = (0..10)
            .map(|_| (Some(reload_sample(1, 1)), Some(reload_inputs())))
            .collect();
        assert!(simulated_reload(stopped)
            .unwrap_err()
            .contains("did not come back"));
    }
    #[test]
    fn reload_requires_matching_daemon_readiness_capability() {
        let mut legacy = reload_sample(1, 1);
        legacy.as_object_mut().unwrap().remove("engine_ready");
        assert!(
            simulated_reload(vec![(Some(legacy), Some(reload_inputs()))])
                .unwrap_err()
                .contains("older than cv1touch")
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
        assert!(simulated_reload(samples)
            .unwrap_err()
            .contains("did not come back"));
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
                .contains("different controller connected")
        );
    }
    #[test]
    fn reload_requires_original_sensor_set_and_fresh_inputs() {
        let mut changed = reload_sample(1, 1);
        changed["sensors"][0]["serial"] = "NEWCAM".into();
        let samples = (0..10)
            .map(|_| (Some(changed.clone()), Some(reload_inputs())))
            .collect();
        assert!(simulated_reload(samples)
            .unwrap_err()
            .contains("did not come back"));
        let samples = (1..11)
            .map(|frames| {
                let mut asleep = reload_inputs();
                asleep.hands[0].connected = false;
                (Some(reload_sample(1, frames)), Some(asleep))
            })
            .collect();
        assert!(simulated_reload(samples)
            .unwrap_err()
            .contains("did not come back"));
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
