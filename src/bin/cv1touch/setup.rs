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
        Check { name: "idle", ok: !recording && normal_mode, detail: "Finish any active recording or calibration before starting the guide.".into() },
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
        for check in &inspection.checks {
            println!(
                "{} {}: {}",
                if check.ok { "ok" } else { "ATTENTION" },
                check.name,
                check.detail
            );
        }
        for report in stages {
            println!("{:?}: {:?} — {}", report.stage, report.state, report.reason);
        }
    }
    ready
}

fn choice(prompt: &str, allowed: &[&str]) -> Result<String, String> {
    println!("{prompt}");
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
            println!("Enter {}.", allowed.join(" / "));
        }
    }
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

fn static_hold() -> Result<(), String> {
    beeps(1);
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(3) {
        if interrupted() {
            return Err("gravity capture interrupted".into());
        }
        thread::sleep(Duration::from_millis(100));
    }
    beeps(2);
    Ok(())
}

fn timed_capture(
    stage: Stage,
    checkpoint: &mut Checkpoint,
    gravity: bool,
) -> Result<PathBuf, String> {
    let rec = capture_path(stage, checkpoint)?;
    if send_path(control::record_start_line(&rec, false)) != 0 {
        return Err("cannot start recording".into());
    }
    let mut guard = StopRecordingGuard(true);
    let result = if gravity {
        println!("Gravity: rest both controllers completely still in 12 varied orientations. Change pitch AND roll between holds, covering faces and edges; keep the rings visible. Each hold lasts 3 seconds.");
        (1..=12).try_for_each(|hold| -> Result<(), String> {
            choice(&format!("Hold {hold}/12: place both controllers in a NEW orientation. Enter 'ready' when stationary, or 'quit'."), &["ready", "quit"])
                .and_then(|answer| if answer == "quit" { Err("setup stopped; recording retained".into()) } else { Ok(()) })?;
            static_hold()
        })
    } else {
        let seconds = if stage == Stage::Render { 5 } else { 60 };
        if stage == Stage::Render {
            println!("Keep both controllers awake while recording their factory geometry.");
        } else {
            println!("{CALIBRATION_BRIEFING}");
        }
        let name = if stage == Stage::Render {
            "geometry"
        } else {
            "wave"
        };
        if pace(&[format!("{name}:{seconds}")]) == 0 {
            Ok(())
        } else {
            Err("capture interrupted".into())
        }
    };
    let stopped = send("record stop");
    if stopped == 0 {
        guard.0 = false;
    }
    result?;
    if stopped != 0 {
        return Err("recording did not stop successfully".into());
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

fn run_stage(
    stage: Stage,
    inspection: &Inspection,
    checkpoint: &mut Checkpoint,
) -> Result<(), String> {
    let old = checkpoint.clone();
    let preserve = stage == Stage::Camera
        && !inspection.unplaced.is_empty()
        && matches!(inspection.snapshot.camera, Artifact::Valid(_));
    setup::begin_stage(stage, checkpoint);
    persist(checkpoint)?;
    let mut recordings = Vec::new();
    let code = match stage {
        Stage::Camera if preserve => {
            println!("Place new sensors in the existing world. Keep all previously calibrated sensors fixed.");
            let rec = timed_capture(stage, checkpoint, false)?;
            recordings.push(rec.clone());
            verify_capture(inspection)?;
            for serial in &inspection.unplaced {
                if calibrate_place(serial, &rec, &file::default_path(), true) != 0 {
                    return Err("sensor placement rejected; earlier backups retained".into());
                }
            }
            restore_mode(&file::default_path())
        }
        Stage::Camera => {
            let mut restore = RestoreCamera(true);
            if send("calibration capture") != 0 {
                return Err("cannot enter camera capture mode".into());
            }
            wait_for_mode("capture", Duration::from_secs(30))?;
            let rec = timed_capture(stage, checkpoint, false)?;
            recordings.push(rec.clone());
            verify_capture(inspection)?;
            let solved = calibrate_solve(&rec, &file::default_path(), false);
            let restored = restore_mode(&file::default_path());
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
            let code = calibrate_gravity_with(&rec, &file::default_path(), true, compensate);
            if code == 0 && restore_mode(&file::default_path()) == 0 {
                send("alignment reload")
            } else {
                1
            }
        }
        Stage::Render => {
            let rec = timed_capture(stage, checkpoint, false)?;
            recordings.push(rec.clone());
            verify_capture(inspection)?;
            fit_render(Some(&rec))
        }
        Stage::Registration => {
            let rec = capture_path(stage, checkpoint)?;
            let base = rec.with_extension("");
            checkpoint
                .recordings
                .entry(stage)
                .or_default()
                .push(base.with_extension("truth.json"));
            persist(checkpoint)?;
            capture_free_named(150, "Press both Touch controllers firmly to the headset, away from its tracking surface. Keep the attachment rigid while making two slow turns each way, then nodding and tilting repeatedly. Press Enter after releasing the controllers.", false, Some(&base)).map_err(|code| format!("headset capture failed ({code})"))?;
            recordings.push(rec);
            recordings.push(base.with_extension("truth.json"));
            verify_capture(inspection)?;
            let code = register_from_capture_with(&base, true, Some("hmd"), true, false);
            if code == 0 {
                send("alignment reload")
            } else {
                code
            }
        }
        Stage::Inputs => calibrate_inputs(None, false),
    };
    if code != 0 || interrupted() {
        return Err("stage did not complete; previous backups and capture paths retained. Rerun setup when ready.".into());
    }
    let mut next = inspect(checkpoint);
    if next.generation != inspection.generation || next.identities != inspection.identities {
        return Err("daemon or controller identities changed during fitting; stage remains incomplete, previous backups retained".into());
    }
    if stage == Stage::Render && next.snapshot.render_sources != inspection.snapshot.render_sources
    {
        return Err(
            "render source meshes changed during fitting; stage remains incomplete, repeat the fit"
                .into(),
        );
    }
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
    setup::record_success(stage, &next.snapshot, checkpoint, recordings)?;
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
        println!("Setup reuses verified saved work. It never pairs/resets controllers, restarts services or changes USB permissions. Type quit at a prompt to stop.");
        loop {
            let inspection = inspect(&checkpoint);
            print_report(&inspection, &checkpoint, false);
            let stage_reports = reports(&inspection, &checkpoint);
            let Some(report) = stage_reports.iter().find(|r| r.state != State::Ready) else {
                println!("Saved setup is complete. Check physical alignment, movement and input-test in a focused VR app. Wake controllers if asleep.");
                return Ok(());
            };
            if report.state == State::Blocked {
                return Err(format!("{:?} is blocked: {}. Resolve this prerequisite, then rerun setup; no recapture was started.", report.stage, report.reason));
            }
            let can_adopt = matches!(report.state, State::Unknown | State::Stale)
                && inspection.unplaced.is_empty()
                && matches!(
                    report.stage,
                    Stage::Camera | Stage::Gravity | Stage::Render | Stage::Registration
                );
            let answer = if can_adopt {
                choice(&format!("{:?}: type keep ONLY if you have verified the current camera placement, level, render alignment and SteamVR registration are correct together. This adopts saved files without changing them. Type new to capture this stage, or quit.", report.stage), &["keep", "new", "quit"])?
            } else {
                choice(&format!("{:?}: type new to run this stage, or quit. Existing settings have backups; captures must pass the existing solver checks.", report.stage), &["new", "quit"])?
            };
            if answer == "quit" {
                return Ok(());
            }
            if answer == "keep" {
                setup::adopt(report.stage, &inspection.snapshot, &mut checkpoint)?;
                persist(&checkpoint)?;
            } else {
                // Recheck live prerequisites after the user has prepared the hardware.
                let current = inspect(&checkpoint);
                if let Some(reason) = current.snapshot.blocked.get(&report.stage) {
                    return Err(reason.clone());
                }
                run_stage(report.stage, &current, &mut checkpoint)?;
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
