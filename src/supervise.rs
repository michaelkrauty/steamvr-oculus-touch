// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Supervising the daemon where no service manager does (Windows): the daemon runs as a child
//! process whose output goes to a log file, each line stamped with the time it arrived. After a
//! failure it is started again 2 s later, as systemd's Restart=on-failure and RestartSec=2 do on
//! Linux, unless it refused its arguments or has failed five times in a row within 30 s of
//! starting.

use std::fs::File;
use std::io::{self, BufRead, BufReader, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, ExitStatus, Stdio};
use std::sync::mpsc::{self, RecvTimeoutError};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::platform::{self, fs};

/// The log's file name; the earlier ones are cv1touchd.1.log (the newest) to cv1touchd.4.log.
const LOG_NAME: &str = "cv1touchd";
const EARLIER_LOGS: usize = 4;
/// Past this size a log is set aside like an earlier one, and a new one begun.
const LOG_MAX_BYTES: u64 = 16 << 20;

/// Where the supervisor keeps the daemon's logs.
pub fn log_dir() -> PathBuf {
    platform::dirs::get().state.join("logs")
}

/// The daemon's current log.
pub fn log_file() -> PathBuf {
    log_path(&log_dir(), 0)
}

fn log_path(dir: &Path, age: usize) -> PathBuf {
    if age == 0 {
        dir.join(format!("{LOG_NAME}.log"))
    } else {
        dir.join(format!("{LOG_NAME}.{age}.log"))
    }
}

/// Moves each log one place older, dropping the oldest. The current log moves first, out of the
/// way: if another program holds it open, nothing changes. If the newest earlier log cannot make
/// room for it (another program holds that one), the current log goes back where it was.
fn set_aside(dir: &Path) -> io::Result<()> {
    let moving = dir.join(format!("{LOG_NAME}.log.moving"));
    match std::fs::rename(log_path(dir, 0), &moving) {
        Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(()),
        result => result?,
    }
    let _ = std::fs::remove_file(log_path(dir, EARLIER_LOGS));
    for age in (1..EARLIER_LOGS).rev() {
        let _ = std::fs::rename(log_path(dir, age), log_path(dir, age + 1));
    }
    std::fs::rename(&moving, log_path(dir, 1)).inspect_err(|_| {
        let _ = std::fs::rename(&moving, log_path(dir, 0));
    })
}

/// The daemon's log: one file per start of the supervisor, and a new one each time it fills up.
pub struct Log {
    dir: PathBuf,
    file: File,
    /// Bytes written since the log began or a full one could not be set aside.
    written: u64,
}

impl Log {
    /// Begins a new log in `dir`, keeping four earlier ones. If the current one cannot be moved
    /// (another program holds it open), it goes on.
    pub fn open(dir: &Path) -> io::Result<Log> {
        fs::create_private_dir_all(dir)?;
        let _ = set_aside(dir);
        let file = fs::append_private(&log_path(dir, 0))?;
        Ok(Log {
            dir: dir.into(),
            file,
            written: 0,
        })
    }

    pub fn path(&self) -> PathBuf {
        log_path(&self.dir, 0)
    }

    /// Writes `text` as one line, stamped with `at`.
    pub fn line(&mut self, at: SystemTime, text: &str) -> io::Result<()> {
        if self.written >= LOG_MAX_BYTES {
            // tried once per filling: a log that cannot be set aside grows another 16 MB first
            self.written = 0;
            if set_aside(&self.dir).is_ok() {
                self.file = fs::append_private(&self.path())?;
            }
        }
        let line = format!("{} {}\n", utc(at), text.trim_end_matches(['\r', '\n']));
        self.file.write_all(line.as_bytes())?;
        self.written += line.len() as u64;
        Ok(())
    }
}

/// `at` in UTC, to the millisecond: 2026-10-14T17:05:03.123Z.
pub fn utc(at: SystemTime) -> String {
    let since = at.duration_since(UNIX_EPOCH).unwrap_or_default();
    let (days, second) = ((since.as_secs() / 86_400) as i64, since.as_secs() % 86_400);
    // the civil date of a day count (Howard Hinnant's days_from_civil, inverted)
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let day_of_era = z.rem_euclid(146_097);
    let year_of_era =
        (day_of_era - day_of_era / 1_460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    let shifted_month = (5 * day_of_year + 2) / 153;
    let day = day_of_year - (153 * shifted_month + 2) / 5 + 1;
    let month = if shifted_month < 10 {
        shifted_month + 3
    } else {
        shifted_month - 9
    };
    let year = year_of_era + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}.{:03}Z",
        second / 3600,
        second / 60 % 60,
        second % 60,
        since.subsec_millis()
    )
}

/// When the daemon is started again after it fails.
#[derive(Debug, Clone, Copy)]
pub struct Policy {
    /// The wait before starting it again.
    pub delay: Duration,
    /// A run that fails sooner than this after its start is a quick failure.
    pub quick: Duration,
    /// After this many quick failures in a row the supervisor gives up.
    pub quick_failures: usize,
}

pub const POLICY: Policy = Policy {
    delay: Duration::from_secs(2),
    quick: Duration::from_secs(30),
    quick_failures: 5,
};

/// The exit status the daemon gives for arguments it does not take: starting it again would not
/// help.
const USAGE_ERROR: i32 = 2;

/// The quick failures in a row.
pub struct Failures {
    policy: Policy,
    in_a_row: usize,
}

impl Failures {
    pub fn new(policy: Policy) -> Failures {
        Failures {
            policy,
            in_a_row: 0,
        }
    }

    /// Counts a failed run that lasted `ran`; whether the daemon may start again.
    pub fn failed(&mut self, ran: Duration) -> bool {
        self.in_a_row = if ran < self.policy.quick {
            self.in_a_row + 1
        } else {
            0
        };
        self.in_a_row < self.policy.quick_failures
    }
}

/// Sends each line `from` gives (lossily decoded, without its end) down `to`.
fn forward(from: impl Read + Send + 'static, to: mpsc::Sender<String>) {
    std::thread::spawn(move || {
        let mut from = BufReader::new(from);
        let mut line = Vec::new();
        loop {
            line.clear();
            match from.read_until(b'\n', &mut line) {
                Ok(0) | Err(_) => break,
                Ok(_) => {
                    let text = String::from_utf8_lossy(&line).into_owned();
                    if to.send(text).is_err() {
                        break;
                    }
                }
            }
        }
    });
}

fn note(log: &mut Log, text: &str) {
    let _ = log.line(SystemTime::now(), &format!("cv1touchd supervisor: {text}"));
}

/// How long the lines a daemon wrote just before it ended may take to arrive; after that its
/// output is left to whatever it started that still holds it.
const LAST_LINES: Duration = Duration::from_secs(1);

/// Runs one daemon to its end, its output to `log`.
fn run_once(command: &mut Command, log: &mut Log) -> io::Result<ExitStatus> {
    if let Err(e) = platform::process::end_with_this_process(command) {
        note(
            log,
            &format!("the daemon could outlive the supervisor ({e})"),
        );
    }
    platform::process::without_window(command);
    let mut child = command
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()?;
    note(log, &format!("started the daemon (process {})", child.id()));
    // its standard output and error are read apart, so their lines are logged as they arrive
    let (tx, rx) = mpsc::channel();
    if let Some(out) = child.stdout.take() {
        forward(out, tx.clone());
    }
    if let Some(err) = child.stderr.take() {
        forward(err, tx);
    } else {
        drop(tx);
    }
    let mut ended: Option<(ExitStatus, Instant)> = None;
    loop {
        let wait = match ended {
            Some((status, at)) if at.elapsed() >= LAST_LINES => return Ok(status),
            Some((_, at)) => LAST_LINES - at.elapsed(),
            None => Duration::from_millis(100),
        };
        match rx.recv_timeout(wait) {
            Ok(line) => {
                let _ = log.line(SystemTime::now(), &line);
            }
            Err(RecvTimeoutError::Disconnected) => match ended {
                Some((status, _)) => return Ok(status),
                None => std::thread::sleep(Duration::from_millis(100)),
            },
            Err(RecvTimeoutError::Timeout) => {}
        }
        if ended.is_none() {
            ended = child.try_wait()?.map(|status| (status, Instant::now()));
        }
    }
}

/// Runs the daemons `command` makes, one at a time, until one stops cleanly (exit status 0), each
/// line it writes to its standard output or error going to `log`; each ends with the thread that
/// calls this, so call it from the main thread. After a failure the next starts once
/// `policy.delay` has passed, unless the daemon refused its arguments (exit status 2) or the
/// policy allows no more quick failures. What the supervisor has to say goes to the log as
/// "cv1touchd supervisor: ...". The exit code for the supervisor: 0 once the daemon stopped
/// cleanly, 1 when it gave up.
pub fn run(mut command: impl FnMut() -> Command, log: &mut Log, policy: Policy) -> i32 {
    let mut failures = Failures::new(policy);
    let delay = policy.delay.as_secs_f64();
    loop {
        let started = Instant::now();
        match run_once(&mut command(), log) {
            Ok(status) if status.success() => {
                note(log, "the daemon stopped");
                return 0;
            }
            Ok(status) if status.code() == Some(USAGE_ERROR) => {
                note(
                    log,
                    &format!("the daemon refused its arguments ({status}); not starting it again"),
                );
                return 1;
            }
            Ok(status) => note(log, &format!("the daemon ended ({status})")),
            Err(e) => note(log, &format!("cannot start the daemon ({e})")),
        }
        if !failures.failed(started.elapsed()) {
            note(
                log,
                &format!(
                    "the daemon failed {} times in a row within {} s of starting; not starting \
                     it again",
                    policy.quick_failures,
                    policy.quick.as_secs_f64()
                ),
            );
            return 1;
        }
        note(log, &format!("starting it again in {delay} s"));
        std::thread::sleep(policy.delay);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dir(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("cv1t-supervise-{}-{tag}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    fn read(path: &Path) -> String {
        std::fs::read_to_string(path).unwrap()
    }

    /// The text of each line, without its stamp.
    fn texts(path: &Path) -> Vec<String> {
        read(path)
            .lines()
            .map(|l| l.split_once(' ').unwrap().1.to_string())
            .collect()
    }

    #[test]
    fn times_are_utc_to_the_millisecond() {
        let at = |secs: u64, ms: u64| UNIX_EPOCH + Duration::from_millis(secs * 1000 + ms);
        assert_eq!(utc(at(0, 0)), "1970-01-01T00:00:00.000Z");
        assert_eq!(utc(at(951_782_400, 5)), "2000-02-29T00:00:00.005Z");
        assert_eq!(utc(at(1_791_997_503, 123)), "2026-10-14T17:05:03.123Z");
        assert_eq!(utc(at(4_102_444_799, 999)), "2099-12-31T23:59:59.999Z");
    }

    #[test]
    fn each_start_begins_a_new_log_and_keeps_four_earlier_ones() {
        let d = dir("rotate");
        for n in 0..6 {
            let mut log = Log::open(&d).unwrap();
            log.line(SystemTime::now(), &format!("run {n}\n")).unwrap();
        }
        assert_eq!(texts(&log_path(&d, 0)), ["run 5"]);
        for age in 1..=4 {
            assert_eq!(texts(&log_path(&d, age)), [format!("run {}", 5 - age)]);
        }
        assert!(!log_path(&d, 5).exists());
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_full_log_is_set_aside() {
        let d = dir("full");
        let mut log = Log::open(&d).unwrap();
        log.line(SystemTime::now(), "first").unwrap();
        log.written = LOG_MAX_BYTES;
        log.line(SystemTime::now(), "second").unwrap();
        assert_eq!(texts(&log_path(&d, 1)), ["first"]);
        assert_eq!(texts(&log_path(&d, 0)), ["second"]);
        std::fs::remove_dir_all(d).unwrap();
    }

    /// A current log that cannot be moved (here a directory stands in the way of the move) leaves
    /// the earlier ones as they were.
    #[test]
    fn earlier_logs_survive_a_current_log_that_cannot_be_moved() {
        let d = dir("stuck");
        std::fs::create_dir_all(&d).unwrap();
        for age in 1..=4 {
            std::fs::write(log_path(&d, age), format!("x run {age}\n")).unwrap();
        }
        std::fs::write(log_path(&d, 0), "x current\n").unwrap();
        let blocker = d.join(format!("{LOG_NAME}.log.moving"));
        std::fs::create_dir_all(blocker.join("occupied")).unwrap();
        let mut log = Log::open(&d).unwrap();
        log.line(SystemTime::now(), "goes on").unwrap();
        assert_eq!(texts(&log_path(&d, 0)), ["current", "goes on"]);
        for age in 1..=4 {
            assert_eq!(texts(&log_path(&d, age)), [format!("run {age}")]);
        }
        std::fs::remove_dir_all(d).unwrap();
    }

    /// When the newest earlier log cannot make room (here directories stand in the earlier logs'
    /// places, so none moves), the current log stays where `doctor` and the next start look for
    /// it.
    #[test]
    fn a_log_that_cannot_be_set_aside_stays_the_current_one() {
        let d = dir("no-room");
        for age in 1..=4 {
            std::fs::create_dir_all(log_path(&d, age).join("occupied")).unwrap();
        }
        std::fs::write(log_path(&d, 0), "x current\n").unwrap();
        let mut log = Log::open(&d).unwrap();
        log.line(SystemTime::now(), "goes on").unwrap();
        assert_eq!(texts(&log_path(&d, 0)), ["current", "goes on"]);
        assert!(!d.join(format!("{LOG_NAME}.log.moving")).exists());
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn quick_failures_in_a_row_end_the_restarts() {
        let mut failures = Failures::new(POLICY);
        let s = Duration::from_secs;
        for _ in 0..4 {
            assert!(failures.failed(s(1)));
        }
        // a run of a while starts the count again
        assert!(failures.failed(s(60)));
        for _ in 0..4 {
            assert!(failures.failed(s(29)));
        }
        assert!(!failures.failed(s(29)));
    }

    /// The shell running `script` (sh on Linux, cmd on Windows).
    fn shell(script: &str) -> Command {
        #[cfg(unix)]
        let mut c = {
            let mut c = Command::new("sh");
            c.arg("-c");
            c
        };
        #[cfg(windows)]
        let mut c = {
            let mut c = Command::new("cmd");
            c.arg("/C");
            c
        };
        c.arg(script);
        c
    }

    fn quick(delay_ms: u64, quick_failures: usize) -> Policy {
        Policy {
            delay: Duration::from_millis(delay_ms),
            quick: Duration::from_secs(60),
            quick_failures,
        }
    }

    #[test]
    fn a_failing_daemon_runs_again_after_the_delay_until_the_limit() {
        let d = dir("fails");
        let mut log = Log::open(&d).unwrap();
        let mut runs = 0;
        let started = Instant::now();
        let script = if cfg!(windows) {
            "echo out && echo err 1>&2 && <nul set /p =last && exit 3"
        } else {
            "echo out && echo err 1>&2 && printf last && exit 3"
        };
        let code = run(
            || {
                runs += 1;
                shell(script)
            },
            &mut log,
            quick(150, 3),
        );
        assert_eq!((code, runs), (1, 3));
        // two waits between three runs
        assert!(started.elapsed() >= Duration::from_millis(300));
        let lines = texts(&log.path());
        let ended = lines
            .iter()
            .filter(|l| l.starts_with("cv1touchd supervisor: the daemon ended"))
            .count();
        assert_eq!(ended, 3, "{lines:?}");
        for text in ["out", "err", "last"] {
            let n = lines.iter().filter(|l| l.trim_end() == text).count();
            assert_eq!(n, 3, "{text}: {lines:?}");
        }
        assert_eq!(
            lines.last().unwrap(),
            "cv1touchd supervisor: the daemon failed 3 times in a row within 60 s of starting; \
             not starting it again"
        );
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_clean_stop_ends_the_supervisor() {
        let d = dir("stops");
        let mut log = Log::open(&d).unwrap();
        let mut runs = 0;
        let code = run(
            || {
                runs += 1;
                shell(if runs == 1 { "exit 1" } else { "exit 0" })
            },
            &mut log,
            quick(10, 5),
        );
        assert_eq!((code, runs), (0, 2));
        assert_eq!(
            texts(&log.path()).last().unwrap(),
            "cv1touchd supervisor: the daemon stopped"
        );
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn arguments_the_daemon_refuses_end_the_supervisor() {
        let d = dir("usage");
        let mut log = Log::open(&d).unwrap();
        let mut runs = 0;
        let code = run(
            || {
                runs += 1;
                shell("exit 2")
            },
            &mut log,
            quick(10, 5),
        );
        assert_eq!((code, runs), (1, 1));
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_daemon_that_cannot_start_is_tried_again() {
        let d = dir("missing");
        let mut log = Log::open(&d).unwrap();
        let code = run(
            || Command::new(d.join("no-such-daemon")),
            &mut log,
            quick(10, 2),
        );
        assert_eq!(code, 1);
        let lines = texts(&log.path());
        let tries = lines
            .iter()
            .filter(|l| l.starts_with("cv1touchd supervisor: cannot start the daemon"))
            .count();
        assert_eq!(tries, 2, "{lines:?}");
        std::fs::remove_dir_all(d).unwrap();
    }

    /// Something the daemon started that still holds its output does not keep the supervisor
    /// from seeing the daemon end.
    #[test]
    fn the_daemon_ending_is_seen_while_its_output_is_still_held() {
        let d = dir("held");
        let mut log = Log::open(&d).unwrap();
        let script = if cfg!(windows) {
            "start /b ping -n 30 127.0.0.1 && exit 0"
        } else {
            "sleep 30 & exit 0"
        };
        let started = Instant::now();
        assert_eq!(run(|| shell(script), &mut log, quick(10, 5)), 0);
        assert!(started.elapsed() < Duration::from_secs(10));
        std::fs::remove_dir_all(d).unwrap();
    }
}
