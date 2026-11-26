// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The guided setup's backend: `cv1touch setup --events` run without a terminal, its events
//! gathered into what the setup window shows, and a live tap of the controllers' raw inputs.

use crate::preview_client::{Connect, Control};
use serde_json::Value;
use std::{
    io::{self, BufRead, BufReader, Read, Write},
    path::Path,
    process::{Child, ChildStdin, Command, Stdio},
    sync::{mpsc, Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

/// The setup stages, in the planner's order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    Sensors,
    Floor,
    Model,
    Buttons,
    Headset,
}

impl Stage {
    pub const ALL: [Stage; 5] = [
        Stage::Sensors,
        Stage::Floor,
        Stage::Model,
        Stage::Buttons,
        Stage::Headset,
    ];

    /// The `setup --redo` name.
    pub fn id(self) -> &'static str {
        match self {
            Self::Sensors => "sensors",
            Self::Floor => "floor",
            Self::Model => "model",
            Self::Buttons => "buttons",
            Self::Headset => "headset",
        }
    }

    pub fn parse(id: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|s| s.id() == id)
    }

    /// The progress bar's word for it.
    pub fn short(self) -> &'static str {
        match self {
            Self::Sensors => "Sensors",
            Self::Floor => "Floor",
            Self::Model => "Model",
            Self::Buttons => "Buttons",
            Self::Headset => "Headset",
        }
    }
}

/// What the setup window was opened for: the stage list, or one stage first (`--redo`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Request {
    pub stage: Option<Stage>,
    /// The short headset alignment.
    pub quick: bool,
}

impl Request {
    pub fn stage(stage: Stage) -> Self {
        Self {
            stage: Some(stage),
            quick: false,
        }
    }

    pub const QUICK_HEADSET: Self = Self {
        stage: Some(Stage::Headset),
        quick: true,
    };

    /// The options after `--setup` that ask for it.
    pub fn gui_args(self) -> Vec<&'static str> {
        let mut args = vec!["--setup"];
        if let Some(stage) = self.stage {
            args.extend(["--stage", stage.id()]);
        }
        if self.quick {
            args.push("--quick");
        }
        args
    }

    /// The CLI's arguments for it.
    pub fn cli_args(self) -> Vec<&'static str> {
        let mut args = vec!["setup", "--events"];
        if let Some(stage) = self.stage {
            args.extend(["--redo", stage.id()]);
        }
        if self.quick {
            args.push("--quick");
        }
        args
    }

    /// The options that follow `--setup`.
    pub fn parse(args: &[String]) -> Result<Self, String> {
        let mut request = Self::default();
        let mut args = args.iter().map(String::as_str);
        while let Some(arg) = args.next() {
            match arg {
                "--stage" if request.stage.is_none() => {
                    request.stage = Some(
                        args.next()
                            .and_then(Stage::parse)
                            .ok_or("--stage takes sensors, floor, model, buttons or headset")?,
                    )
                }
                "--quick" if !request.quick => request.quick = true,
                other => return Err(format!("unknown or repeated setup option {other}")),
            }
        }
        if request.quick && request.stage != Some(Stage::Headset) {
            return Err("--quick goes with --stage headset".into());
        }
        Ok(request)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Check {
    pub name: String,
    pub ok: bool,
    pub detail: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StageInfo {
    pub id: String,
    pub title: String,
    /// `missing`, `invalid`, `unknown`, `stale`, `blocked` or `ready`.
    pub state: String,
    pub summary: String,
    pub minutes: String,
    pub can_run: bool,
    pub can_keep: bool,
    pub quick_available: bool,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct State {
    pub ready: bool,
    pub checks: Vec<Check>,
    pub stages: Vec<StageInfo>,
    pub next: Option<String>,
    pub sensors: u64,
    pub daemon_running: bool,
    pub left_out: Vec<String>,
    pub settling: Vec<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Prompt {
    pub id: String,
    pub stage: Option<String>,
    pub message: String,
    pub choices: Vec<String>,
    pub accepts_controller_button: bool,
}

impl Prompt {
    pub fn offers(&self, command: &str) -> bool {
        self.choices.iter().any(|c| c == command)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Progress {
    pub stage: Option<String>,
    /// `capture`, `countdown`, `solve`, `reload` or `inputs`.
    pub phase: String,
    pub elapsed_s: Option<f64>,
    pub total_s: Option<f64>,
    pub detail: String,
    pub metrics: Value,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StageStart {
    pub stage: String,
    pub title: String,
    pub quick: bool,
    pub instructions: String,
}

/// One line of `setup --events`.
#[derive(Clone, Debug, PartialEq)]
pub enum Event {
    State(State),
    Prompt(Prompt),
    StageStart(StageStart),
    Status {
        stage: Option<String>,
        message: String,
    },
    Progress(Progress),
    StageDone {
        stage: String,
        summary: String,
    },
    StageFailed {
        stage: String,
        error: String,
        slipped: bool,
    },
    Rejected {
        command: String,
        error: String,
    },
    Done {
        all_ready: bool,
        error: Option<String>,
    },
}

fn text(v: &Value, key: &str) -> Result<String, String> {
    v[key]
        .as_str()
        .map(str::to_owned)
        .ok_or_else(|| format!("{key} is not text"))
}

fn flag(v: &Value, key: &str) -> Result<bool, String> {
    v[key]
        .as_bool()
        .ok_or_else(|| format!("{key} is not a boolean"))
}

fn texts(v: &Value) -> Vec<String> {
    v.as_array()
        .into_iter()
        .flatten()
        .filter_map(|s| s.as_str().map(str::to_owned))
        .collect()
}

fn list<T>(
    v: &Value,
    key: &str,
    item: impl Fn(&Value) -> Result<T, String>,
) -> Result<Vec<T>, String> {
    v[key]
        .as_array()
        .ok_or_else(|| format!("{key} is not a list"))?
        .iter()
        .map(item)
        .collect()
}

/// One event line; None for an event this version does not know.
pub fn parse_event(line: &str) -> Result<Option<Event>, String> {
    let v: Value = serde_json::from_str(line).map_err(|e| e.to_string())?;
    let stage = v["stage"].as_str().map(str::to_owned);
    Ok(Some(match v["event"].as_str().ok_or("no event name")? {
        "state" => Event::State(State {
            ready: flag(&v, "ready")?,
            checks: list(&v, "checks", |c| {
                Ok(Check {
                    name: text(c, "name")?,
                    ok: flag(c, "ok")?,
                    detail: text(c, "detail")?,
                })
            })?,
            stages: list(&v, "stages", |s| {
                Ok(StageInfo {
                    id: text(s, "id")?,
                    title: text(s, "title")?,
                    state: text(s, "state")?,
                    summary: text(s, "summary")?,
                    minutes: text(s, "minutes").unwrap_or_default(),
                    can_run: flag(s, "can_run")?,
                    can_keep: flag(s, "can_keep")?,
                    quick_available: flag(s, "quick_available")?,
                })
            })?,
            next: v["next"].as_str().map(str::to_owned),
            sensors: v["sensors"].as_u64().unwrap_or(0),
            daemon_running: v["daemon_running"].as_bool().unwrap_or(false),
            left_out: texts(&v["moved"]["left_out"]),
            settling: texts(&v["moved"]["settling"]),
        }),
        "prompt" => Event::Prompt(Prompt {
            id: text(&v, "id")?,
            stage,
            message: text(&v, "message")?,
            choices: texts(&v["choices"]),
            accepts_controller_button: v["accepts_controller_button"] == true,
        }),
        "stage_start" => Event::StageStart(StageStart {
            stage: text(&v, "stage")?,
            title: text(&v, "title")?,
            quick: v["quick"] == true,
            instructions: text(&v, "instructions")?,
        }),
        "status" => Event::Status {
            stage,
            message: text(&v, "message")?,
        },
        "progress" => Event::Progress(Progress {
            stage,
            phase: text(&v, "phase")?,
            elapsed_s: v["elapsed_s"].as_f64(),
            total_s: v["total_s"].as_f64(),
            detail: text(&v, "detail").unwrap_or_default(),
            metrics: v["metrics"].clone(),
        }),
        "stage_done" => Event::StageDone {
            stage: text(&v, "stage")?,
            summary: text(&v, "summary")?,
        },
        "stage_failed" => Event::StageFailed {
            stage: text(&v, "stage")?,
            error: text(&v, "error")?,
            slipped: v["slipped"] == true,
        },
        "rejected" => Event::Rejected {
            command: text(&v, "command")?,
            error: text(&v, "error")?,
        },
        "done" => Event::Done {
            all_ready: flag(&v, "all_ready")?,
            error: v["error"].as_str().map(str::to_owned),
        },
        _ => return Ok(None),
    }))
}

/// Everything the guide has said that the window shows.
#[derive(Clone, Debug, Default)]
pub struct Guide {
    pub state: Option<State>,
    /// What the guide waits for, if it waits.
    pub prompt: Option<Prompt>,
    pub stage: Option<StageStart>,
    pub status: Option<String>,
    pub progress: Option<Progress>,
    /// The stages finished in this run, with their result lines.
    pub finished: Vec<(String, String)>,
    pub failed: Option<(String, String)>,
    pub rejected: Option<(String, String)>,
    pub done: Option<(bool, Option<String>)>,
    /// Why the guide ended without its `done` event.
    pub ended: Option<String>,
    pub events: usize,
}

impl Guide {
    pub fn apply(&mut self, event: Event) {
        self.events += 1;
        match event {
            Event::State(state) => {
                self.state = Some(state);
                self.stage = None;
                self.progress = None;
                self.prompt = None;
            }
            Event::Prompt(prompt) => {
                if prompt.id == "menu" {
                    self.stage = None;
                    self.status = None;
                }
                if matches!(prompt.id.as_str(), "menu" | "retry") {
                    self.progress = None;
                }
                self.prompt = Some(prompt);
            }
            Event::StageStart(start) => {
                self.stage = Some(start);
                self.progress = None;
                self.prompt = None;
                self.failed = None;
                self.status = None;
            }
            Event::Status { message, .. } => self.status = Some(message),
            Event::Progress(progress) => {
                // Buttons and sticks reports the hands while it waits at a step's prompt.
                if progress.phase != "inputs" || progress.elapsed_s.is_some() {
                    self.prompt = None;
                }
                self.progress = Some(progress);
            }
            Event::StageDone { stage, summary } => {
                self.finished.push((stage, summary));
                self.stage = None;
                self.progress = None;
                self.prompt = None;
            }
            Event::StageFailed { stage, error, .. } => {
                self.failed = Some((stage, error));
                self.progress = None;
                self.prompt = None;
            }
            Event::Rejected { command, error } => self.rejected = Some((command, error)),
            Event::Done { all_ready, error } => {
                self.done = Some((all_ready, error));
                self.stage = None;
                self.progress = None;
                self.prompt = None;
            }
        }
    }

    /// The stage running now, by id.
    pub fn active_stage(&self) -> Option<&str> {
        self.stage
            .as_ref()
            .map(|s| s.stage.as_str())
            .or_else(|| self.prompt.as_ref()?.stage.as_deref())
    }
}

/// A running `cv1touch setup --events`. Dropping it stops the guide the way Ctrl+C does at a
/// terminal, so that the daemon goes back to tracking.
pub struct SetupClient {
    child: Child,
    stdin: Option<ChildStdin>,
    guide: Arc<Mutex<Guide>>,
}

const TOO_OLD: &str = "This version of cv1touch cannot run setup from this window. Update or rebuild cv1touch, then open setup again.";

impl SetupClient {
    pub fn start(request: Request, ctx: eframe::egui::Context) -> io::Result<Self> {
        Self::spawn(
            &crate::client::executable(),
            &request.cli_args(),
            move || ctx.request_repaint(),
        )
    }

    pub fn spawn(
        program: &Path,
        args: &[&str],
        repaint: impl Fn() + Send + 'static,
    ) -> io::Result<Self> {
        let mut command = Command::new(program);
        command
            .args(args)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        #[cfg(windows)]
        {
            use std::os::windows::process::CommandExt;
            command.creation_flags(0x0800_0000); // CREATE_NO_WINDOW
        }
        let mut child = command.spawn()?;
        let stdin = child.stdin.take();
        let stdout = child.stdout.take().expect("piped stdout");
        let mut stderr = child.stderr.take().expect("piped stderr");
        let guide = Arc::new(Mutex::new(Guide::default()));
        // Standard error is a free-text log; its last line explains an early exit.
        let log = thread::spawn(move || {
            let mut text = Vec::new();
            let _ = stderr.read_to_end(&mut text);
            String::from_utf8_lossy(&text)
                .lines()
                .rev()
                .find(|l| !l.trim().is_empty())
                .unwrap_or_default()
                .trim()
                .to_owned()
        });
        let shared = Arc::clone(&guide);
        thread::spawn(move || {
            for line in BufReader::new(stdout).lines() {
                let Ok(line) = line else { break };
                if let Ok(Some(event)) = parse_event(&line) {
                    shared.lock().unwrap().apply(event);
                    repaint();
                }
            }
            let last = log.join().unwrap_or_default();
            let mut guide = shared.lock().unwrap();
            if guide.done.is_none() {
                let why = if guide.events == 0 {
                    TOO_OLD
                } else {
                    "Setup stopped unexpectedly."
                };
                guide.ended = Some(if last.is_empty() {
                    why.to_owned()
                } else {
                    format!("{why}\n{last}")
                });
            }
            drop(guide);
            repaint();
        });
        Ok(Self {
            child,
            stdin,
            guide,
        })
    }

    /// Sends one command line; false once the guide has stopped reading.
    pub fn send(&mut self, line: &str) -> bool {
        self.guide.lock().unwrap().rejected = None;
        let Some(stdin) = self.stdin.as_mut() else {
            return false;
        };
        stdin
            .write_all(format!("{line}\n").as_bytes())
            .and_then(|()| stdin.flush())
            .is_ok()
    }

    pub fn guide(&self) -> Guide {
        self.guide.lock().unwrap().clone()
    }
}

impl Drop for SetupClient {
    fn drop(&mut self) {
        if let Some(mut stdin) = self.stdin.take() {
            let _ = stdin.write_all(b"quit\n");
        }
        // The guide restores the daemon's tracking mode as it stops; kill it only if stuck.
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            if !matches!(self.child.try_wait(), Ok(None)) {
                return;
            }
            thread::sleep(Duration::from_millis(20));
        }
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// One controller's analog inputs, raw factory values.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Hand {
    pub connected: bool,
    pub trigger: f32,
    pub grip: f32,
    pub stick: [f32; 2],
}

/// Left, then right.
pub type Hands = [Hand; 2];

fn hand(v: &Value) -> Option<Hand> {
    let axis = |v: &Value| v.as_f64().map(|x| x as f32).filter(|x| x.is_finite());
    Some(Hand {
        connected: v["connected"].as_bool()?,
        trigger: axis(&v["trigger"])?,
        grip: axis(&v["grip"])?,
        stick: [axis(&v["stick"][0])?, axis(&v["stick"][1])?],
    })
}

/// The daemon's `inputs` reply.
pub fn parse_inputs(reply: &str) -> Option<Hands> {
    let v: Value = serde_json::from_str(reply.trim_end().strip_prefix("ok ")?).ok()?;
    Some([hand(&v["hands"][0])?, hand(&v["hands"][1])?])
}

/// The hands in a Buttons and sticks progress event's metrics.
pub fn metric_hands(metrics: &Value) -> Option<Hands> {
    Some([hand(&metrics["left"])?, hand(&metrics["right"])?])
}

/// Polls the daemon's raw inputs about 15 times a second while it lives, so that the
/// Buttons and sticks stage shows the triggers, grips and sticks move smoothly.
pub struct InputsTap {
    latest: Arc<Mutex<Option<(Hands, Instant)>>>,
    _stop: mpsc::Sender<()>,
}

const TAP_INTERVAL: Duration = Duration::from_millis(66);

impl InputsTap {
    pub fn start(connect: Connect, repaint: impl Fn() + Send + 'static) -> Self {
        let latest = Arc::new(Mutex::new(None));
        let shared = Arc::clone(&latest);
        let (stop, stopped) = mpsc::channel::<()>();
        thread::spawn(move || {
            let mut control = Control::new(connect);
            loop {
                let started = Instant::now();
                let hands = control
                    .request("inputs\n")
                    .ok()
                    .and_then(|r| parse_inputs(&r));
                let wait = match hands {
                    Some(hands) => {
                        *shared.lock().unwrap() = Some((hands, Instant::now()));
                        repaint();
                        TAP_INTERVAL.saturating_sub(started.elapsed())
                    }
                    None => Duration::from_millis(500),
                };
                if !matches!(
                    stopped.recv_timeout(wait),
                    Err(mpsc::RecvTimeoutError::Timeout)
                ) {
                    break;
                }
            }
        });
        Self {
            latest,
            _stop: stop,
        }
    }

    /// The latest inputs, if they are recent.
    pub fn latest(&self) -> Option<Hands> {
        let latest = *self.latest.lock().unwrap();
        latest
            .filter(|(_, at)| at.elapsed() < Duration::from_millis(500))
            .map(|(hands, _)| hands)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// A look at a setup with one stage done, two to do, one to check and one waiting.
    pub const STATE: &str = r#"{"event":"state","ready":false,"checks":[{"name":"daemon","ok":true,"detail":"cv1touchd is running"},{"name":"sensor_streams","ok":true,"detail":""},{"name":"controllers_awake","ok":false,"detail":"a controller is asleep or out of range: press a button on each controller"},{"name":"idle","ok":true,"detail":"no other guide or recording is running"}],"stages":[{"id":"sensors","title":"Sensor positions","state":"ready","summary":"done","reason":"saved configuration matches its recorded dependencies","minutes":"~2 min","can_run":true,"can_keep":false,"quick_available":false},{"id":"floor","title":"Floor level","state":"stale","summary":"needs redoing (the sensors moved)","reason":"dependency changed","minutes":"~2 min","can_run":true,"can_keep":true,"quick_available":false},{"id":"model","title":"Controller model","state":"missing","summary":"not done","reason":"artifact is missing","minutes":"~1 min","can_run":true,"can_keep":false,"quick_available":false},{"id":"buttons","title":"Buttons and sticks","state":"unknown","summary":"saved earlier, not verified","reason":"no recorded dependencies","minutes":"~1 min","can_run":true,"can_keep":false,"quick_available":false},{"id":"headset","title":"Headset alignment","state":"blocked","summary":"waiting: press a button on each controller","reason":"press a button on each controller; artifact is missing","minutes":"~3 min","can_run":false,"can_keep":false,"quick_available":true}],"next":"floor","sensors":3,"daemon_running":true,"moved":{"left_out":[],"settling":["Sensor 2 moved and is being placed again by itself."]}}"#;
    pub const MENU: &str = r#"{"event":"prompt","id":"menu","stage":null,"message":"Choose a stage.","choices":["go","run","keep","recheck","quit"],"accepts_controller_button":false}"#;
    pub const SENSORS_START: &str = r#"{"event":"stage_start","stage":"sensors","title":"Sensor positions","quick":false,"instructions":"This works out where each of your Rift sensors is. Hold both controllers and walk\nslowly around the play area for a minute, so that every sensor sees them.\n\nKeep the controllers awake: press a button now and then."}"#;
    pub const START: &str = r#"{"event":"prompt","id":"start","stage":"sensors","message":"Stand where the sensors can see you.\nPress A on the right controller (or Enter) to start.","choices":["go","quit"],"accepts_controller_button":true}"#;
    pub const CAPTURE: &str = r#"{"event":"progress","stage":"sensors","phase":"capture","elapsed_s":24.4,"total_s":60.0,"detail":"","metrics":{}}"#;
    pub const STATUS: &str = r#"{"event":"status","stage":"sensors","message":"Walk slowly around the play area with both controllers."}"#;
    pub const HEADSET_START: &str = r#"{"event":"stage_start","stage":"headset","title":"Headset alignment","quick":false,"instructions":"This lines up the controllers with SteamVR's headset tracking. Hold both\ncontrollers against the headset and wear it."}"#;
    pub const HEADSET_PROMPT: &str = r#"{"event":"prompt","id":"headset_start","stage":"headset","message":"Put on the headset with both controllers held to it.\nPress A on the right controller to start.","choices":["quit"],"accepts_controller_button":true}"#;
    pub const COUNTDOWN: &str = r#"{"event":"progress","stage":"headset","phase":"countdown","elapsed_s":3.0,"total_s":10.0,"detail":"","metrics":{"remaining_s":7}}"#;
    pub const HEADSET_CAPTURE: &str = r#"{"event":"progress","stage":"headset","phase":"capture","elapsed_s":18.7,"total_s":150.0,"detail":"","metrics":{"pairs":812,"seconds":18.7,"turn_left_deg":512.3,"turn_right_deg":95.0,"needed_deg":660.0,"pitch_deg":[-22.1,18.4],"roll_deg":[-9.0,12.5],"nod_needed_deg":15.0}}"#;
    pub const BUTTONS_START: &str = r#"{"event":"stage_start","stage":"buttons","title":"Buttons and sticks","quick":false,"instructions":"This measures your triggers, grips and sticks.\nFollow each step; it records by itself once both hands are in position."}"#;
    pub const BUTTONS_STEP: &str = r#"{"event":"status","stage":"buttons","message":"  2/4 Pull both triggers all the way and hold them there until the double beep."}"#;
    pub const TRIGGERS: &str = r#"{"event":"prompt","id":"triggers","stage":"buttons","message":"Pull both triggers all the way and hold them there.","choices":["go","quit"],"accepts_controller_button":false}"#;
    pub const INPUTS: &str = r#"{"event":"progress","stage":"buttons","phase":"inputs","elapsed_s":null,"total_s":null,"detail":"triggers","metrics":{"left":{"connected":true,"trigger":0.62,"grip":0.0,"stick":[0.01,-0.03]},"right":{"connected":true,"trigger":0.91,"grip":0.05,"stick":[0.4,0.55]},"in_position":false}}"#;
    pub const SOLVE: &str = r#"{"event":"progress","stage":"sensors","phase":"solve","elapsed_s":null,"total_s":null,"detail":"Working out where the sensors are","metrics":{}}"#;
    pub const STAGE_DONE: &str =
        r#"{"event":"stage_done","stage":"floor","summary":"floor levelled: 0.4° correction"}"#;
    pub const STAGE_FAILED: &str = r#"{"event":"stage_failed","stage":"headset","error":"the left controller slid on the headset (it turned 4°); your previous calibration is unchanged","slipped":true}"#;
    pub const RETRY: &str = r#"{"event":"prompt","id":"retry","stage":"headset","message":"Headset alignment did not finish.","choices":["retry","unwatched","skip","quit"],"accepts_controller_button":false}"#;
    pub const REJECTED: &str = r#"{"event":"rejected","command":"run nothing","error":"no stage nothing: sensors, floor, model, buttons or headset"}"#;
    pub const DONE: &str = r#"{"event":"done","all_ready":true}"#;
    pub const DONE_ERROR: &str = r#"{"event":"done","all_ready":false,"error":"another setup guide is running; finish it first"}"#;

    pub fn guide(lines: &[&str]) -> Guide {
        let mut guide = Guide::default();
        for line in lines {
            guide.apply(parse_event(line).unwrap().unwrap());
        }
        guide
    }

    #[test]
    fn every_event_of_the_contract_parses() {
        let Some(Event::State(state)) = parse_event(STATE).unwrap() else {
            panic!("state")
        };
        assert!(!state.ready && state.sensors == 3 && state.daemon_running);
        assert_eq!(state.stages.len(), 5);
        assert_eq!(state.next.as_deref(), Some("floor"));
        assert_eq!(
            state
                .stages
                .iter()
                .map(|s| s.state.as_str())
                .collect::<Vec<_>>(),
            ["ready", "stale", "missing", "unknown", "blocked"]
        );
        assert!(state.stages[4].quick_available && !state.stages[4].can_run);
        assert!(state.stages[1].can_keep);
        assert_eq!(state.checks[2].name, "controllers_awake");
        assert!(!state.checks[2].ok);
        assert_eq!(state.settling.len(), 1);
        let Some(Event::Prompt(menu)) = parse_event(MENU).unwrap() else {
            panic!("prompt")
        };
        assert_eq!(menu.id, "menu");
        assert!(menu.stage.is_none() && menu.offers("recheck") && !menu.offers("retry"));
        assert!(matches!(
            parse_event(HEADSET_PROMPT).unwrap(),
            Some(Event::Prompt(Prompt { ref id, ref choices, accepts_controller_button: true, .. }))
                if id == "headset_start" && choices == &["quit"]
        ));
        assert!(matches!(
            parse_event(HEADSET_START).unwrap(),
            Some(Event::StageStart(StageStart { quick: false, .. }))
        ));
        assert_eq!(
            parse_event(STATUS).unwrap(),
            Some(Event::Status {
                stage: Some("sensors".into()),
                message: "Walk slowly around the play area with both controllers.".into()
            })
        );
        let Some(Event::Progress(capture)) = parse_event(HEADSET_CAPTURE).unwrap() else {
            panic!("progress")
        };
        assert_eq!(
            (capture.elapsed_s, capture.total_s),
            (Some(18.7), Some(150.0))
        );
        assert_eq!(capture.metrics["pitch_deg"][0], -22.1);
        let Some(Event::Progress(solve)) = parse_event(SOLVE).unwrap() else {
            panic!("progress")
        };
        assert_eq!(
            (solve.elapsed_s, solve.detail.as_str()),
            (None, "Working out where the sensors are")
        );
        assert_eq!(
            parse_event(STAGE_DONE).unwrap(),
            Some(Event::StageDone {
                stage: "floor".into(),
                summary: "floor levelled: 0.4° correction".into()
            })
        );
        assert!(matches!(
            parse_event(STAGE_FAILED).unwrap(),
            Some(Event::StageFailed { slipped: true, .. })
        ));
        assert!(matches!(
            parse_event(REJECTED).unwrap(),
            Some(Event::Rejected { .. })
        ));
        assert_eq!(
            parse_event(DONE).unwrap(),
            Some(Event::Done {
                all_ready: true,
                error: None
            })
        );
        assert_eq!(
            parse_event(DONE_ERROR).unwrap(),
            Some(Event::Done {
                all_ready: false,
                error: Some("another setup guide is running; finish it first".into())
            })
        );
        // A later version's event is ignored; a broken line is an error.
        assert_eq!(parse_event(r#"{"event":"hint","text":"x"}"#).unwrap(), None);
        assert!(parse_event("{").is_err());
        assert!(parse_event(r#"{"event":"done"}"#).is_err());
    }

    #[test]
    fn the_guide_keeps_what_the_window_shows() {
        let g = guide(&[STATE, MENU]);
        assert_eq!(g.prompt.as_ref().unwrap().id, "menu");
        assert_eq!(g.active_stage(), None);
        let g = guide(&[STATE, MENU, SENSORS_START, STATUS, START]);
        assert_eq!(g.active_stage(), Some("sensors"));
        assert_eq!(g.prompt.as_ref().unwrap().id, "start");
        // The capture starts: the prompt is answered.
        let g = guide(&[STATE, MENU, SENSORS_START, START, CAPTURE]);
        assert!(g.prompt.is_none() && g.progress.is_some());
        // Buttons and sticks reports the hands while its prompt still waits.
        let g = guide(&[BUTTONS_START, TRIGGERS, INPUTS]);
        assert_eq!(g.prompt.as_ref().unwrap().id, "triggers");
        assert_eq!(
            metric_hands(&g.progress.unwrap().metrics).unwrap()[1].trigger,
            0.91
        );
        let g = guide(&[HEADSET_START, STAGE_FAILED, RETRY]);
        assert_eq!(g.failed.as_ref().unwrap().0, "headset");
        assert_eq!(g.active_stage(), Some("headset"));
        let g = guide(&[STATE, MENU, STAGE_DONE, STATE, MENU, DONE]);
        assert_eq!(g.finished.len(), 1);
        assert_eq!(g.done, Some((true, None)));
        assert!(g.prompt.is_none());
    }

    #[test]
    fn requests_map_to_window_and_cli_arguments() {
        let args = |a: &[&str]| a.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        assert_eq!(Request::parse(&[]).unwrap(), Request::default());
        let quick = Request::parse(&args(&["--stage", "headset", "--quick"])).unwrap();
        assert_eq!(quick, Request::QUICK_HEADSET);
        assert_eq!(
            quick.cli_args(),
            ["setup", "--events", "--redo", "headset", "--quick"]
        );
        assert_eq!(
            quick.gui_args(),
            ["--setup", "--stage", "headset", "--quick"]
        );
        assert_eq!(
            Request::stage(Stage::Buttons).cli_args(),
            ["setup", "--events", "--redo", "buttons"]
        );
        assert_eq!(Request::default().cli_args(), ["setup", "--events"]);
        for bad in [
            &["--quick"][..],
            &["--stage", "floor", "--quick"],
            &["--stage", "nothing"],
            &["--stage"],
            &["--stage", "floor", "--stage", "model"],
            &["--bogus"],
        ] {
            assert!(Request::parse(&args(bad)).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn inputs_parse_from_the_daemon_reply() {
        let reply = r#"ok {"hands":[{"identity":"L","connected":true,"received_host_ns":1,"trigger":0.25,"grip":0.5,"stick":[-1.0,0.5]},{"identity":"R","connected":false,"received_host_ns":0,"trigger":0.0,"grip":0.0,"stick":[0.0,0.0]}]}"#;
        let hands = parse_inputs(&format!("{reply}\n")).unwrap();
        assert_eq!(
            hands[0],
            Hand {
                connected: true,
                trigger: 0.25,
                grip: 0.5,
                stick: [-1.0, 0.5]
            }
        );
        assert!(!hands[1].connected);
        assert!(parse_inputs("err unknown command 'inputs'").is_none());
        assert!(parse_inputs(r#"ok {"hands":[]}"#).is_none());
    }

    #[cfg(unix)]
    fn script(name: &str, body: &str) -> std::path::PathBuf {
        let dir =
            std::env::temp_dir().join(format!("cv1touch-setup-{name}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("cv1touch");
        std::fs::write(&path, format!("#!/bin/sh\n{body}\n")).unwrap();
        path
    }

    #[cfg(unix)]
    fn wait_for(client: &SetupClient, done: impl Fn(&Guide) -> bool) -> Guide {
        for _ in 0..500 {
            let guide = client.guide();
            if done(&guide) {
                return guide;
            }
            thread::sleep(Duration::from_millis(10));
        }
        panic!("timed out: {:?}", client.guide());
    }

    /// A stand-in for `cv1touch setup --events` that logs every command it reads. `printf`
    /// rather than `echo`, which some shells give backslash escapes.
    #[cfg(unix)]
    #[test]
    fn the_client_runs_a_stage_and_quits_the_guide_when_dropped() {
        let body = format!(
            r#"log="$1"
state='{STATE}'
menu='{MENU}'
printf '%s\n' "$state" "$menu"
while IFS= read -r line; do
  printf '%s\n' "$line" >> "$log"
  case "$line" in
    "run sensors")
      printf '%s\n' '{SENSORS_START}'
      printf '%s\n' '{CAPTURE}'
      printf '%s\n' '{{"event":"stage_done","stage":"sensors","summary":"3 sensors placed"}}'
      printf '%s\n' "$state" "$menu";;
    quit) printf '%s\n' '{{"event":"done","all_ready":false}}'; exit 1;;
    *) printf '%s\n' '{REJECTED}';;
  esac
done
printf '%s\n' '{{"event":"done","all_ready":false}}'"#
        );
        let path = script("session", &body);
        let log = path.with_file_name("log");
        let mut client = SetupClient::spawn(
            Path::new("/bin/sh"),
            &[path.to_str().unwrap(), log.to_str().unwrap()],
            || {},
        )
        .unwrap();
        wait_for(&client, |g| {
            g.prompt.as_ref().is_some_and(|p| p.id == "menu")
        });
        assert!(client.send("run sensors"));
        let g = wait_for(&client, |g| {
            g.finished.len() == 1 && g.prompt.as_ref().is_some_and(|p| p.id == "menu")
        });
        assert_eq!(g.events, 7);
        assert_eq!(g.finished[0], ("sensors".into(), "3 sensors placed".into()));
        assert!(client.send("run nothing"));
        let g = wait_for(&client, |g| g.rejected.is_some());
        assert_eq!(g.rejected.unwrap().0, "run nothing");
        drop(client);
        // The guide was told to quit, not only left without input.
        assert_eq!(
            std::fs::read_to_string(&log).unwrap(),
            "run sensors\nrun nothing\nquit\n"
        );
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }

    #[cfg(unix)]
    #[test]
    fn an_older_cli_and_a_stuck_guide_are_handled() {
        let path = script(
            "old",
            "echo 'usage: cv1touch setup [--no-slip-check] [--redo STAGE]' >&2; exit 2",
        );
        let client =
            SetupClient::spawn(Path::new("/bin/sh"), &[path.to_str().unwrap()], || {}).unwrap();
        let g = wait_for(&client, |g| g.ended.is_some());
        let ended = g.ended.unwrap();
        assert!(ended.starts_with(TOO_OLD), "{ended}");
        assert!(ended.ends_with("usage: cv1touch setup [--no-slip-check] [--redo STAGE]"));
        drop(client);
        // A guide that ignores quit and its closed input is killed after 2 s.
        let path = script("stuck", "trap '' TERM; exec sleep 30");
        let client =
            SetupClient::spawn(Path::new("/bin/sh"), &[path.to_str().unwrap()], || {}).unwrap();
        let started = Instant::now();
        drop(client);
        let took = started.elapsed();
        assert!(
            took >= Duration::from_secs(2) && took < Duration::from_secs(4),
            "{took:?}"
        );
        for name in ["old", "stuck"] {
            let _ = std::fs::remove_dir_all(
                std::env::temp_dir().join(format!("cv1touch-setup-{name}-{}", std::process::id())),
            );
        }
    }
}
