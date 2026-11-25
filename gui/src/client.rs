// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

use serde_json::Value;
use std::{
    io::Read,
    path::{Path, PathBuf},
    process::{Command, Stdio},
    sync::mpsc::{self, Receiver, Sender},
    thread,
    time::{Duration, Instant},
};

/// Why a status probe failed: `down` only when the daemon is known not to run.
#[derive(Clone, Debug, PartialEq)]
pub struct StatusError {
    pub down: bool,
    pub message: String,
}

impl From<String> for StatusError {
    fn from(message: String) -> Self {
        Self {
            down: false,
            message,
        }
    }
}

impl From<&str> for StatusError {
    fn from(message: &str) -> Self {
        message.to_owned().into()
    }
}

pub type StatusResult = Result<Value, StatusError>;

pub enum Update {
    Status(StatusResult, Instant),
    Doctor(Result<Value, String>),
    Config(crate::config_stats::ConfigStats),
}

pub struct Client {
    pub updates: Receiver<Update>,
    refresh: Sender<()>,
    poll: Sender<()>,
}

/// The second status sample follows the first quickly so that rates appear at once.
const FIRST_INTERVAL: Duration = Duration::from_millis(400);
#[cfg(target_os = "linux")]
pub const STATUS_INTERVAL: Duration = Duration::from_millis(1500);
// Windows starts a process per probe.
#[cfg(not(target_os = "linux"))]
pub const STATUS_INTERVAL: Duration = Duration::from_secs(2);

impl Client {
    pub fn start(ctx: eframe::egui::Context) -> Self {
        let (updates_tx, updates) = mpsc::channel();
        let (refresh, requests) = mpsc::channel();
        let (poll, polls) = mpsc::channel();
        // Separate workers keep a slow doctor from delaying the live status board.
        let tx = updates_tx.clone();
        let repaint = ctx.clone();
        thread::spawn(move || {
            let mut interval = FIRST_INTERVAL;
            loop {
                if tx.send(Update::Status(status(), Instant::now())).is_err() {
                    break;
                }
                repaint.request_repaint();
                match polls.recv_timeout(interval) {
                    Ok(()) => while polls.try_recv().is_ok() {},
                    Err(mpsc::RecvTimeoutError::Timeout) => {}
                    Err(mpsc::RecvTimeoutError::Disconnected) => break,
                }
                interval = STATUS_INTERVAL;
            }
        });
        thread::spawn(move || loop {
            let config = crate::config_stats::ConfigStats::load();
            let result = invoke("doctor");
            if updates_tx.send(Update::Config(config)).is_err()
                || updates_tx.send(Update::Doctor(result)).is_err()
            {
                break;
            }
            ctx.request_repaint();
            match requests.recv_timeout(Duration::from_secs(30)) {
                Ok(()) => while requests.try_recv().is_ok() {},
                Err(mpsc::RecvTimeoutError::Timeout) => {}
                Err(mpsc::RecvTimeoutError::Disconnected) => break,
            }
        });
        Self {
            updates,
            refresh,
            poll,
        }
    }

    /// Rerun the checks and reread the configuration now.
    pub fn refresh(&self) {
        let _ = self.refresh.send(());
    }

    /// Take the next status sample now.
    pub fn poll(&self) {
        let _ = self.poll.send(());
    }
}

/// One status sample: straight from the control socket on Linux, else through the CLI.
pub fn status() -> StatusResult {
    #[cfg(target_os = "linux")]
    if let Some(dir) = std::env::var_os("XDG_RUNTIME_DIR") {
        match socket_status(&Path::new(&dir).join("cv1touch/control.sock")) {
            Ok(value) => return Ok(value),
            Err(error) if error.down => return Err(error),
            // Fall back to the CLI, which explains other failures in full.
            Err(_) => {}
        }
    }
    match invoke_at(&executable(), "status", Duration::from_secs(10)) {
        Ok(value) => Ok(value),
        Err(message) => Err(StatusError {
            down: message.starts_with("cv1touch: cv1touchd isn't running"),
            message,
        }),
    }
}

#[cfg(target_os = "linux")]
fn socket_status(path: &Path) -> StatusResult {
    use std::io::{BufRead, BufReader, ErrorKind, Write};
    let mut stream = std::os::unix::net::UnixStream::connect(path).map_err(|e| StatusError {
        down: matches!(e.kind(), ErrorKind::NotFound | ErrorKind::ConnectionRefused),
        message: if matches!(e.kind(), ErrorKind::NotFound | ErrorKind::ConnectionRefused) {
            "cv1touchd isn't running".to_owned()
        } else {
            format!("Could not reach cv1touchd: {e}")
        },
    })?;
    let timeout = Some(Duration::from_secs(2));
    let mut reply = String::new();
    stream
        .set_read_timeout(timeout)
        .and_then(|()| stream.set_write_timeout(timeout))
        .and_then(|()| stream.write_all(b"status\n"))
        .and_then(|()| BufReader::new(stream).read_line(&mut reply))
        .map_err(|e| format!("Could not read cv1touchd status: {e}"))?;
    reply
        .trim_end()
        .strip_prefix("ok ")
        .and_then(|json| serde_json::from_str(json).ok())
        .filter(valid_status)
        .ok_or_else(|| "cv1touchd returned invalid status".into())
}

fn valid_status(value: &Value) -> bool {
    value["generation"].as_u64().is_some()
        && value["engine_ready"].is_boolean()
        && value["hands"].is_array()
        && value["sensors"].is_array()
}

pub(crate) fn executable() -> PathBuf {
    let name = if cfg!(windows) {
        "cv1touch.exe"
    } else {
        "cv1touch"
    };
    std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|p| p.join(name)))
        .filter(|p| p.is_file())
        .unwrap_or_else(|| name.into())
}

pub(crate) fn invoke(subcommand: &str) -> Result<Value, String> {
    invoke_at(&executable(), subcommand, Duration::from_secs(10))
}

fn invoke_at(path: &Path, subcommand: &str, timeout: Duration) -> Result<Value, String> {
    let mut command = Command::new(path);
    command
        .args([subcommand, "--json"])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        command.creation_flags(0x08000000);
    }
    let mut child = command
        .spawn()
        .map_err(|e| format!("Could not run cv1touch {subcommand}: {e}"))?;
    // Drain both pipes concurrently so a verbose child cannot block before exit.
    let (tx, rx) = mpsc::channel();
    for (which, mut pipe) in [
        (
            0,
            Box::new(child.stdout.take().unwrap()) as Box<dyn Read + Send>,
        ),
        (
            1,
            Box::new(child.stderr.take().unwrap()) as Box<dyn Read + Send>,
        ),
    ] {
        let tx = tx.clone();
        thread::spawn(move || {
            let mut data = Vec::new();
            let result = pipe.read_to_end(&mut data).map(|_| data);
            let _ = tx.send((which, result));
        });
    }
    drop(tx);
    let deadline = Instant::now() + timeout;
    let status = loop {
        match child.try_wait() {
            Ok(Some(status)) => break status,
            Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(10)),
            Ok(None) => {
                let _ = child.kill();
                let _ = child.wait();
                return Err(format!("cv1touch {subcommand} timed out"));
            }
            Err(e) => {
                let _ = child.kill();
                let _ = child.wait();
                return Err(format!("Could not wait for cv1touch {subcommand}: {e}"));
            }
        }
    };
    let mut streams = [Vec::new(), Vec::new()];
    for _ in 0..2 {
        let (which, result) = rx
            .recv_timeout(deadline.saturating_duration_since(Instant::now()))
            .map_err(|_| format!("cv1touch {subcommand} timed out reading output"))?;
        streams[which] =
            result.map_err(|e| format!("Could not read cv1touch {subcommand}: {e}"))?;
    }
    // doctor exits 1 when a check fails, but still returns a useful JSON report.
    let value: Value = serde_json::from_slice(&streams[0]).map_err(|_| {
        let detail = String::from_utf8_lossy(&streams[1]);
        if detail.trim().is_empty() {
            format!("cv1touch {subcommand} returned invalid data")
        } else {
            detail.trim().to_owned()
        }
    })?;
    let valid = match subcommand {
        "doctor" => {
            value["checks"].as_array().is_some_and(|checks| {
                !checks.is_empty()
                    && checks
                        .iter()
                        .all(|check| check["ok"].is_boolean() && check["detail"].is_string())
            }) && (status.success() || status.code() == Some(1))
        }
        "status" => status.success() && valid_status(&value),
        _ => false,
    };
    if !valid {
        return Err(format!("cv1touch {subcommand} returned invalid data"));
    }
    Ok(value)
}

#[cfg(all(test, target_os = "linux"))]
mod tests {
    use super::*;
    use std::os::unix::fs::PermissionsExt;

    #[test]
    fn subprocess_deadline_schema_and_recovery() {
        let dir = std::env::temp_dir().join(format!("cv1touch-client-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let script = dir.join("fake");
        let write = |body: &str| {
            std::fs::write(&script, format!("#!/bin/sh\n{body}\n")).unwrap();
            std::fs::set_permissions(&script, std::fs::Permissions::from_mode(0o700)).unwrap();
        };
        write("exec sleep 5");
        assert!(invoke_at(&script, "status", Duration::from_millis(100))
            .unwrap_err()
            .contains("timed out"));
        write("echo '{'");
        assert!(invoke_at(&script, "status", Duration::from_secs(1)).is_err());
        write("echo '{}'");
        assert!(invoke_at(&script, "status", Duration::from_secs(1)).is_err());
        write("echo '{\"generation\":1,\"engine_ready\":true,\"hands\":[],\"sensors\":[]}'");
        assert!(invoke_at(&script, "status", Duration::from_secs(1)).is_ok());
        write("echo '{\"checks\":[{\"ok\":false,\"detail\":\"failed\"}]}'; exit 1");
        assert!(invoke_at(&script, "doctor", Duration::from_secs(1)).is_ok());
        write("echo '{\"checks\":[]}'");
        assert!(invoke_at(&script, "doctor", Duration::from_secs(1)).is_err());
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn socket_status_reads_one_reply_and_reports_a_stopped_daemon() {
        use std::io::{BufRead, BufReader, Write};
        use std::os::unix::net::UnixListener;
        let dir = std::env::temp_dir().join(format!("cv1touch-ipc-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("control.sock");
        assert!(socket_status(&path).unwrap_err().down);
        let serve = |reply: &'static str| {
            let _ = std::fs::remove_file(&path);
            let listener = UnixListener::bind(&path).unwrap();
            thread::spawn(move || {
                let (stream, _) = listener.accept().unwrap();
                let mut line = String::new();
                BufReader::new(&stream).read_line(&mut line).unwrap();
                assert_eq!(line, "status\n");
                (&stream).write_all(reply.as_bytes()).unwrap();
            })
        };
        let server =
            serve("ok {\"generation\":3,\"engine_ready\":true,\"hands\":[],\"sensors\":[]}\n");
        assert_eq!(socket_status(&path).unwrap()["generation"], 3);
        server.join().unwrap();
        let server = serve("err busy\n");
        let error = socket_status(&path).unwrap_err();
        assert!(!error.down, "{error:?}");
        server.join().unwrap();
        // The socket file outlives its listener: connecting is refused.
        let _ = std::fs::remove_file(&path);
        drop(UnixListener::bind(&path).unwrap());
        assert!(socket_status(&path).unwrap_err().down);
        std::fs::remove_dir_all(dir).unwrap();
    }
}
