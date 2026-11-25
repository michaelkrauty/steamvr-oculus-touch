// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! `cv1touch setup --events`: the guide for a program that shows it (cv1touch's window) rather
//! than for a person at a terminal. It runs the same stages with the same checks and words; what
//! the terminal would show goes out as one JSON object per line on standard output, and what a
//! person would type comes in as one command per line on standard input. Everything else the
//! process prints goes to standard error. `quit` or `abort`, or the end of standard input, stop
//! the guide as Ctrl+C stops it at a terminal: what is running stops, the daemon gets its
//! tracking mode back, and finished stages are kept.
//!
//! The state is the thread's: only the guide's thread emits, and tests run side by side.
use std::cell::RefCell;
use std::collections::VecDeque;
use std::io::Write;
use std::time::{Duration, Instant};

use cv1touch::platform::console::Line;
use cv1touch::setup::Stage;

struct Events {
    out: Box<dyn Write>,
    lines: Box<dyn FnMut(Duration) -> Line>,
    /// Lines read while checking for a stop, for the next wait for a line.
    queued: VecDeque<String>,
    stop: bool,
    stage: Option<Stage>,
}

thread_local! {
    static EVENTS: RefCell<Option<Events>> = const { RefCell::new(None) };
}

/// From now on this thread's guide talks in events: to `out`, reading commands with `lines`.
pub(super) fn start(out: Box<dyn Write>, lines: Box<dyn FnMut(Duration) -> Line>) {
    EVENTS.with(|e| {
        *e.borrow_mut() = Some(Events {
            out,
            lines,
            queued: VecDeque::new(),
            stop: false,
            stage: None,
        })
    });
}

/// Back to the terminal (tests).
#[cfg(test)]
pub(super) fn end() {
    EVENTS.with(|e| *e.borrow_mut() = None);
}

pub(super) fn on() -> bool {
    EVENTS.with(|e| e.borrow().is_some())
}

/// The id a stage has in events and commands: the name `setup --redo` takes.
pub(super) fn stage_id(stage: Stage) -> &'static str {
    match stage {
        Stage::Camera => "sensors",
        Stage::Gravity => "floor",
        Stage::Render => "model",
        Stage::Registration => "headset",
        Stage::Inputs => "buttons",
    }
}

/// Writes `event` as one line and flushes it; nothing without events.
pub(super) fn emit(event: serde_json::Value) {
    EVENTS.with(|e| {
        if let Some(e) = e.borrow_mut().as_mut() {
            // a reader that went away ends the guide through standard input
            let _ = writeln!(e.out, "{event}").and_then(|()| e.out.flush());
        }
    });
}

/// The stage events from now on belong to.
pub(super) fn set_stage(stage: Option<Stage>) {
    EVENTS.with(|e| {
        if let Some(e) = e.borrow_mut().as_mut() {
            e.stage = stage;
        }
    });
}

fn current_stage() -> serde_json::Value {
    EVENTS.with(|e| {
        e.borrow()
            .as_ref()
            .and_then(|e| e.stage)
            .map_or(serde_json::Value::Null, |s| stage_id(s).into())
    })
}

/// A line the terminal would show.
pub(super) fn status(message: &str) {
    emit(serde_json::json!({"event": "status", "stage": current_stage(), "message": message}));
}

/// Where a step has got: seconds of `total_s` (either unknown), and what the window may show
/// live.
pub(super) fn progress(
    phase: &str,
    elapsed_s: Option<f64>,
    total_s: Option<f64>,
    detail: &str,
    metrics: serde_json::Value,
) {
    let metrics = if metrics.is_null() {
        serde_json::json!({})
    } else {
        metrics
    };
    emit(
        serde_json::json!({"event": "progress", "stage": current_stage(), "phase": phase,
        "elapsed_s": elapsed_s, "total_s": total_s, "detail": detail, "metrics": metrics}),
    );
}

/// A wait for one of `choices`; `button`: pressing the button the message names on a
/// controller answers it too.
pub(super) fn prompt(id: &str, message: &str, choices: &[&str], button: bool) {
    emit(
        serde_json::json!({"event": "prompt", "id": id, "stage": current_stage(),
        "message": message, "choices": choices, "accepts_controller_button": button}),
    );
}

/// A command that does not fit where the guide is.
pub(super) fn rejected(command: &str, error: &str) {
    emit(serde_json::json!({"event": "rejected", "command": command.trim(), "error": error}));
}

/// Whether the line asks the guide to stop.
fn stops(line: &str) -> bool {
    matches!(line.trim(), "quit" | "abort")
}

/// Takes the lines that came, keeping all but a stop for later; true once a stop came or
/// standard input ended.
pub(super) fn stopped() -> bool {
    EVENTS.with(|e| {
        let mut e = e.borrow_mut();
        let Some(e) = e.as_mut() else {
            return false;
        };
        while !e.stop {
            match (e.lines)(Duration::ZERO) {
                Line::Typed(line) if stops(&line) => e.stop = true,
                Line::Typed(line) => e.queued.push_back(line),
                Line::Closed => e.stop = true,
                Line::NotYet => break,
            }
        }
        e.stop
    })
}

/// The next command, waiting up to `timeout`. A stop is never one: it reads as nothing yet, and
/// `stopped` is true from then on.
pub(super) fn next_line(timeout: Duration) -> Line {
    if stopped() {
        return Line::NotYet;
    }
    let queued = EVENTS.with(|e| e.borrow_mut().as_mut().and_then(|e| e.queued.pop_front()));
    if let Some(line) = queued {
        return Line::Typed(line);
    }
    let line = EVENTS.with(|e| {
        e.borrow_mut()
            .as_mut()
            .map_or(Line::Closed, |e| (e.lines)(timeout))
    });
    match line {
        Line::Typed(line) if !stops(&line) => Line::Typed(line),
        Line::NotYet => Line::NotYet,
        _ => {
            EVENTS.with(|e| {
                if let Some(e) = e.borrow_mut().as_mut() {
                    e.stop = true;
                }
            });
            Line::NotYet
        }
    }
}

/// Drops the commands that came before a prompt, as the terminal drops what was typed ahead; a
/// stop still counts.
pub(super) fn discard() {
    stopped();
    EVENTS.with(|e| {
        if let Some(e) = e.borrow_mut().as_mut() {
            e.queued.clear();
        }
    });
}

/// What the guide was told at a prompt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Command {
    /// Start what the prompt offers; at the stage list, the next stage to do.
    Go,
    /// Run this stage (`quick`: the short headset alignment).
    Run(Stage, bool),
    /// Keep a stage's result saved earlier, as it is.
    Keep(Stage),
    /// Look at the checks and stages again.
    Recheck,
    /// After a failure: the same stage again; `Unwatched`: the headset alignment again without
    /// watching for a slip; `Skip`: back to the stage list.
    Retry,
    Unwatched,
    Skip,
}

pub(super) fn parse(line: &str) -> Result<Command, String> {
    let words: Vec<&str> = line.split_whitespace().collect();
    let stage = |name: &str| {
        Stage::from_redo_name(name)
            .ok_or_else(|| format!("no stage {name}: sensors, floor, model, buttons or headset"))
    };
    match words.as_slice() {
        ["go"] => Ok(Command::Go),
        ["run", name] => stage(name).map(|s| Command::Run(s, false)),
        ["run", "headset", "quick"] => Ok(Command::Run(Stage::Registration, true)),
        ["run", _, "quick"] => Err("quick goes with headset".into()),
        ["keep", name] => stage(name).map(Command::Keep),
        ["recheck"] => Ok(Command::Recheck),
        ["retry"] => Ok(Command::Retry),
        ["unwatched"] => Ok(Command::Unwatched),
        ["skip"] => Ok(Command::Skip),
        _ => Err("unknown command: go, run STAGE [quick], keep STAGE, recheck, retry, unwatched, skip, quit".into()),
    }
}

/// True at most every `every`, the first time at once: for progress that changes faster than it
/// is worth sending.
pub(super) struct Ticker {
    last: Option<Instant>,
    every: Duration,
}

impl Ticker {
    pub(super) fn new(every: Duration) -> Ticker {
        Ticker { last: None, every }
    }

    pub(super) fn due(&mut self) -> bool {
        let now = Instant::now();
        if self.last.is_some_and(|last| now < last + self.every) {
            return false;
        }
        self.last = Some(now);
        true
    }
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    /// What a test's guide wrote, line by line.
    #[derive(Clone, Default)]
    pub(crate) struct Sink(pub Arc<Mutex<Vec<u8>>>);

    impl Write for Sink {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    impl Sink {
        /// Every line as JSON: a line that is not fails the test.
        pub(crate) fn events(&self) -> Vec<serde_json::Value> {
            String::from_utf8(self.0.lock().unwrap().clone())
                .unwrap()
                .lines()
                .map(|l| serde_json::from_str(l).unwrap_or_else(|e| panic!("{e}: {l}")))
                .collect()
        }
    }

    /// Starts events with `script` as standard input, which then ends, as a window sends it:
    /// each line in answer to a wait for one, but a line marked `!` (a stop) and the end at
    /// once, also to a check for a stop while a stage runs. A guide that goes on reading long
    /// after the end fails the test instead of hanging it.
    pub(crate) fn scripted(script: &[&str]) -> Sink {
        let sink = Sink::default();
        let mut lines: VecDeque<String> = script.iter().map(|l| l.to_string()).collect();
        let mut after_end = 0;
        start(
            Box::new(sink.clone()),
            Box::new(move |timeout| match lines.front() {
                None => {
                    after_end += 1;
                    assert!(
                        after_end < 10_000,
                        "the guide kept reading after its input ended"
                    );
                    Line::Closed
                }
                Some(line) if timeout.is_zero() && !line.starts_with('!') => Line::NotYet,
                Some(_) => {
                    let line = lines.pop_front().unwrap();
                    Line::Typed(format!("{}\n", line.trim_start_matches('!')))
                }
            }),
        );
        sink
    }

    #[test]
    fn events_are_one_json_object_per_line() {
        let sink = scripted(&[]);
        assert!(on());
        status("before a stage");
        set_stage(Some(Stage::Registration));
        progress(
            "countdown",
            Some(3.0),
            Some(10.0),
            "",
            serde_json::json!({"remaining_s": 7}),
        );
        progress("solve", None, None, "fitting", serde_json::Value::Null);
        prompt("start", "Press A\nto start.", &["go", "quit"], true);
        rejected("dance\n", "unknown command");
        end();
        assert!(!on());
        status("not sent");
        assert_eq!(
            sink.events(),
            [
                serde_json::json!({"event": "status", "stage": null, "message": "before a stage"}),
                serde_json::json!({"event": "progress", "stage": "headset", "phase": "countdown",
                    "elapsed_s": 3.0, "total_s": 10.0, "detail": "", "metrics": {"remaining_s": 7}}),
                serde_json::json!({"event": "progress", "stage": "headset", "phase": "solve",
                    "elapsed_s": null, "total_s": null, "detail": "fitting", "metrics": {}}),
                serde_json::json!({"event": "prompt", "id": "start", "stage": "headset",
                    "message": "Press A\nto start.", "choices": ["go", "quit"],
                    "accepts_controller_button": true}),
                serde_json::json!({"event": "rejected", "command": "dance", "error": "unknown command"}),
            ]
        );
    }

    #[test]
    fn commands_parse() {
        assert_eq!(parse("go\n"), Ok(Command::Go));
        assert_eq!(parse("run sensors"), Ok(Command::Run(Stage::Camera, false)));
        assert_eq!(
            parse(" run  floor "),
            Ok(Command::Run(Stage::Gravity, false))
        );
        assert_eq!(parse("run model"), Ok(Command::Run(Stage::Render, false)));
        assert_eq!(parse("run buttons"), Ok(Command::Run(Stage::Inputs, false)));
        assert_eq!(
            parse("run headset"),
            Ok(Command::Run(Stage::Registration, false))
        );
        assert_eq!(
            parse("run headset quick"),
            Ok(Command::Run(Stage::Registration, true))
        );
        assert_eq!(parse("keep model"), Ok(Command::Keep(Stage::Render)));
        assert_eq!(parse("recheck"), Ok(Command::Recheck));
        assert_eq!(parse("retry"), Ok(Command::Retry));
        assert_eq!(parse("unwatched"), Ok(Command::Unwatched));
        assert_eq!(parse("skip"), Ok(Command::Skip));
        for bad in ["run floor quick", "run camera", "", "GO", "run"] {
            assert!(parse(bad).is_err(), "{bad}");
        }
        for stage in [
            Stage::Camera,
            Stage::Gravity,
            Stage::Render,
            Stage::Registration,
            Stage::Inputs,
        ] {
            assert_eq!(Stage::from_redo_name(stage_id(stage)), Some(stage));
        }
    }

    /// quit, abort and the end of standard input all stop; other lines wait their turn, and a
    /// prompt drops those that came before it.
    #[test]
    fn a_stop_is_seen_while_waiting_and_lines_keep_their_order() {
        for stop in [Some("!quit"), Some("!abort"), None] {
            let mut script = vec!["!recheck", "!go"];
            script.extend(stop);
            let sink = scripted(&script);
            assert!(stopped(), "{stop:?}");
            assert_eq!(next_line(Duration::ZERO), Line::NotYet);
            end();
            assert!(sink.events().is_empty());
        }
        let mut fed = VecDeque::from([
            Line::Typed("recheck\n".into()),
            Line::NotYet,
            Line::Typed("go\n".into()),
            Line::NotYet,
            Line::Typed("go\n".into()),
        ]);
        start(
            Box::new(Sink::default()),
            Box::new(move |_| fed.pop_front().unwrap_or(Line::NotYet)),
        );
        assert!(!stopped());
        assert_eq!(next_line(Duration::ZERO), Line::Typed("recheck\n".into()));
        assert_eq!(next_line(Duration::ZERO), Line::Typed("go\n".into()));
        assert!(!stopped());
        discard();
        assert_eq!(next_line(Duration::ZERO), Line::NotYet);
        assert!(!stopped());
        end();
    }

    #[test]
    fn a_ticker_lets_one_through_per_period() {
        let mut ticker = Ticker::new(Duration::from_secs(3600));
        assert!(ticker.due());
        assert!(!ticker.due());
        let mut always = Ticker::new(Duration::ZERO);
        assert!(always.due() && always.due());
    }
}
