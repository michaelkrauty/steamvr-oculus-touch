// SPDX-License-Identifier: Apache-2.0

use crate::{actions, client};
use actions::ServiceAction;
use eframe::egui;
use serde_json::Value;
use std::{
    io::{self, BufRead, BufReader, Read, Write},
    process::{Command, Stdio},
    sync::mpsc::{self, Receiver, Sender, TryRecvError},
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Controller {
    pub hand: &'static str,
    pub address: String,
}

impl Controller {
    fn parse(v: &Value) -> Option<Self> {
        let hand = match v["device"].as_str()? {
            "left" => "left",
            "right" => "right",
            _ => return None,
        };
        let address = v["address"].as_str()?;
        (address.len() == 8 && address.bytes().all(|b| b.is_ascii_hexdigit())).then(|| Self {
            hand,
            address: address.to_ascii_lowercase(),
        })
    }
}

#[derive(Debug, PartialEq)]
pub enum Event {
    Status(String),
    Snapshot {
        after: bool,
        headset: String,
        controllers: Vec<Controller>,
    },
    Listening,
    Broadcast(Controller, String, bool, bool),
    Paired(Controller, Result<(), String>),
    Error(String, bool),
    Done,
    Finished,
}

fn parse_event(line: &str) -> Option<Event> {
    let v: Value = serde_json::from_str(line).ok()?;
    let text = |key: &str| v[key].as_str().map(str::to_owned);
    Some(match v["event"].as_str()? {
        "status" => Event::Status(text("message")?),
        name @ ("before" | "after") => Event::Snapshot {
            after: name == "after",
            headset: text("headset")?,
            controllers: v["devices"]
                .as_array()?
                .iter()
                .filter_map(Controller::parse)
                .collect(),
        },
        "listening" => Event::Listening,
        "broadcast" => Event::Broadcast(
            Controller::parse(&v)?,
            text("firmware")?,
            v["already_paired"].as_bool()?,
            v["also_paired"].as_bool()?,
        ),
        "paired" => Event::Paired(
            Controller::parse(&v)?,
            if v["ok"].as_bool()? {
                Ok(())
            } else {
                Err(text("error")?)
            },
        ),
        "error" => Event::Error(text("message")?, v["recovery"].as_bool().unwrap_or(false)),
        "done" => Event::Done,
        _ => return None,
    })
}

pub enum Request {
    Pair(Controller),
    Done,
    Cancel,
}

impl Request {
    fn command(&self) -> String {
        match self {
            Self::Pair(c) => format!("pair {} {}", c.hand, c.address),
            Self::Done => "done".into(),
            Self::Cancel => "cancel".into(),
        }
    }
}

pub struct Client {
    send: Sender<Request>,
    pub events: Receiver<Event>,
    worker: Option<JoinHandle<()>>,
}

impl Client {
    pub fn start(recovery: bool, ctx: egui::Context) -> Self {
        let (send, commands) = mpsc::channel();
        let (events, receive) = mpsc::channel();
        let worker = thread::spawn(move || {
            let emit = |event| {
                let _ = events.send(event);
                ctx.request_repaint();
            };
            let result = with_daemon_stopped(
                &emit,
                || match client::status() {
                    Ok(_) => Ok(true),
                    Err(error) if error.down => Ok(false),
                    Err(error) => Err(error.message),
                },
                actions::service,
                || run(recovery, &commands, &emit),
            );
            if let Err(error) = result {
                emit(Event::Error(error, false));
            }
            emit(Event::Finished);
        });
        Self {
            send,
            events: receive,
            worker: Some(worker),
        }
    }

    pub fn request(&self, request: Request) {
        let _ = self.send.send(request);
    }
}

impl Drop for Client {
    fn drop(&mut self) {
        self.request(Request::Cancel);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

fn with_daemon_stopped(
    emit: &impl Fn(Event),
    probe: impl Fn() -> Result<bool, String>,
    service: impl Fn(ServiceAction) -> std::io::Result<()>,
    work: impl FnOnce() -> Result<(), String>,
) -> Result<(), String> {
    emit(Event::Status(
        "Checking whether the daemon is running…".into(),
    ));
    let was_running = probe()?;
    let result = (|| {
        if was_running {
            emit(Event::Status("Stopping the daemon for pairing…".into()));
            service(ServiceAction::Stop).map_err(|e| format!("Could not stop the daemon: {e}"))?;
            let deadline = Instant::now() + Duration::from_secs(5);
            loop {
                if probe() == Ok(false) {
                    break;
                }
                if Instant::now() >= deadline {
                    return Err("The daemon did not stop. Pairing did not start.".into());
                }
                thread::sleep(Duration::from_millis(100));
            }
        }
        work()
    })();
    if was_running {
        emit(Event::Status("Starting the daemon again…".into()));
        if let Err(error) = service(ServiceAction::Start) {
            emit(Event::Error(
                format!("Could not start the daemon again: {error}"),
                false,
            ));
        }
    }
    result
}

fn run(recovery: bool, commands: &Receiver<Request>, emit: &impl Fn(Event)) -> Result<(), String> {
    if matches!(
        commands.try_recv(),
        Ok(Request::Cancel | Request::Done) | Err(TryRecvError::Disconnected)
    ) {
        return Ok(());
    }
    emit(Event::Status(
        if recovery {
            "Returning the headset to normal mode…"
        } else {
            "Reading the Touch controllers…"
        }
        .into(),
    ));
    let mut command = Command::new(client::executable());
    command
        .args(["pair", if recovery { "--normal" } else { "--events" }])
        .stdin(if recovery {
            Stdio::null()
        } else {
            Stdio::piped()
        })
        .stdout(if recovery {
            Stdio::null()
        } else {
            Stdio::piped()
        })
        .stderr(Stdio::piped());
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        command.creation_flags(0x0800_0000); // CREATE_NO_WINDOW
    }
    let mut child = command
        .spawn()
        .map_err(|e| format!("Could not run cv1touch pair: {e}"))?;
    let stderr = child.stderr.take().expect("piped stderr");
    let stderr = thread::spawn(move || read_stderr(stderr));
    let mut stdin = child.stdin.take();
    let (send, output) = mpsc::channel();
    if let Some(stdout) = child.stdout.take() {
        thread::spawn(move || {
            for line in BufReader::new(stdout).lines().map_while(Result::ok) {
                if let Some(event) = parse_event(&line) {
                    if send.send(event).is_err() {
                        break;
                    }
                }
            }
        });
    } else {
        drop(send);
    }
    let mut reported = false;
    let mut done = false;
    let mut forward = |event| {
        reported |= matches!(event, Event::Error(..));
        done |= matches!(event, Event::Done);
        emit(event);
    };
    // Allow for headset reboots, radio timeouts, and restoration after a failed reboot.
    let cleanup = Duration::from_secs(120);
    let mut deadline = Instant::now()
        + if recovery {
            cleanup
        } else {
            Duration::from_secs(180) + cleanup
        };
    let mut stopping = recovery;
    let mut warned_slow_cleanup = false;
    loop {
        match output.recv_timeout(Duration::from_millis(50)) {
            Ok(event) => forward(event),
            Err(mpsc::RecvTimeoutError::Disconnected) => thread::sleep(Duration::from_millis(50)),
            Err(mpsc::RecvTimeoutError::Timeout) => {}
        }
        if let Some(status) = child.try_wait().map_err(|e| e.to_string())? {
            output.into_iter().for_each(forward);
            if status.success() {
                if recovery {
                    emit(Event::Done);
                } else if !done {
                    return Err("cv1touch did not confirm that pairing ended. Check that the command-line tool is up to date.".into());
                }
                return Ok(());
            }
            if !reported {
                let message = stderr.join().unwrap_or_default();
                let message = if message.is_empty() {
                    "cv1touch could not complete pairing or restore normal mode.".into()
                } else {
                    message
                };
                emit(Event::Error(message, true));
            }
            return Ok(());
        }
        if !stopping {
            let request = match commands.try_recv() {
                Ok(request) => Some(request),
                Err(TryRecvError::Disconnected) => Some(Request::Cancel),
                Err(TryRecvError::Empty) if Instant::now() >= deadline => Some(Request::Cancel),
                Err(TryRecvError::Empty) => None,
            };
            if let Some(request) = request {
                let sent = stdin.as_mut().is_some_and(|input| {
                    writeln!(input, "{}", request.command())
                        .and_then(|()| input.flush())
                        .is_ok()
                });
                if !sent || matches!(request, Request::Done | Request::Cancel) {
                    stopping = true;
                    stdin.take();
                    deadline = Instant::now() + cleanup;
                }
            }
        } else if !warned_slow_cleanup && Instant::now() >= deadline {
            // Never kill or detach the CLI while it may still be returning the headset to normal
            // mode, and keep the daemon stopped until it exits so nothing touches the headset
            // concurrently.
            warned_slow_cleanup = true;
            emit(Event::Status(
                "Still waiting for the headset to return to normal mode…".into(),
            ));
        }
    }
}

// Retain the diagnostic prefix but keep draining so a noisy child cannot block on the pipe.
fn read_stderr(mut pipe: impl Read) -> String {
    let mut bytes = Vec::new();
    let _ = (&mut pipe).take(4096).read_to_end(&mut bytes);
    let _ = io::copy(&mut pipe, &mut io::sink());
    String::from_utf8_lossy(&bytes).trim().to_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stderr_is_trimmed_and_bounded() {
        assert_eq!(
            read_stderr(&b"  headset is not connected\n"[..]),
            "headset is not connected"
        );
        assert_eq!(read_stderr(&[b'x'; 8192][..]).len(), 4096);
    }

    #[test]
    fn daemon_is_restored_after_pairing_fails_but_not_started_if_it_was_down() {
        use std::cell::{Cell, RefCell};

        for was_running in [false, true] {
            let running = Cell::new(was_running);
            let calls = RefCell::new(Vec::new());
            let result = with_daemon_stopped(
                &|_| {},
                || Ok(running.get()),
                |action| {
                    calls.borrow_mut().push(action);
                    running.set(action == ServiceAction::Start);
                    Ok(())
                },
                || {
                    assert!(!running.get());
                    Err("pairing failed".into())
                },
            );
            assert_eq!(result, Err("pairing failed".into()));
            assert_eq!(running.get(), was_running);
            assert_eq!(
                *calls.borrow(),
                if was_running {
                    vec![ServiceAction::Stop, ServiceAction::Start]
                } else {
                    vec![]
                }
            );
        }
    }

    #[test]
    fn parses_touch_broadcast_and_builds_the_matching_command() {
        let line = r#"{"event":"broadcast","device":"right","address":"000000AB","firmware":"1.0","already_paired":false,"also_paired":true}"#;
        let Some(Event::Broadcast(controller, firmware, false, true)) = parse_event(line) else {
            panic!("invalid event")
        };
        assert_eq!(firmware, "1.0");
        assert_eq!(Request::Pair(controller).command(), "pair right 000000ab");
        assert_eq!(parse_event(&line.replace("right", "other")), None);
        assert_eq!(parse_event(&line.replace("000000AB", "00\nquit")), None);
    }
}
