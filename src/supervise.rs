// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Supervising the daemon where no service manager does (Windows): the daemon runs as a child
//! process whose output goes to a log file, each line stamped with the time it arrived. After a
//! failure it is started again 2 s later, as systemd's Restart=on-failure and RestartSec=2 do on
//! Linux, unless it refused its arguments or has failed five times in a row within 30 s of
//! starting. Asked to stop, the supervisor asks the daemon to stop and waits for it.

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

/// How the supervisor learns that it is to stop, and how it asks the daemon to.
pub struct Stopping<'a> {
    /// A request to stop, if one came: what it was (polled often, so quick).
    pub requested: Box<dyn FnMut() -> Option<String> + 'a>,
    /// Asks the running daemon to stop.
    pub ask: Box<dyn FnMut() -> Result<(), String> + 'a>,
    /// How long the daemon has to stop once asked; then it is ended. (After its console window
    /// is closed, Windows ends the supervisor 5 s later whatever this says.)
    pub within: Duration,
}

/// How often a daemon not yet taking requests (it is just starting) is asked again.
const ASK_AGAIN: Duration = Duration::from_millis(250);

/// A daemon's end: its exit status, and whether the supervisor had asked it to stop.
struct Ended {
    status: ExitStatus,
    asked: bool,
}

/// Runs one daemon to its end, its output to `log`.
fn run_once(command: &mut Command, log: &mut Log, stopping: &mut Stopping) -> io::Result<Ended> {
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
    let mut asked: Option<Instant> = None;
    let mut last_ask: Option<Instant> = None;
    let mut accepted = false;
    let mut ending = false;
    loop {
        let wait = match ended {
            Some((status, at)) if at.elapsed() >= LAST_LINES => {
                let asked = asked.is_some();
                return Ok(Ended { status, asked });
            }
            Some((_, at)) => LAST_LINES - at.elapsed(),
            None => Duration::from_millis(100),
        };
        match rx.recv_timeout(wait) {
            Ok(line) => {
                let _ = log.line(SystemTime::now(), &line);
            }
            Err(RecvTimeoutError::Disconnected) => match ended {
                Some((status, _)) => {
                    let asked = asked.is_some();
                    return Ok(Ended { status, asked });
                }
                None => std::thread::sleep(Duration::from_millis(100)),
            },
            Err(RecvTimeoutError::Timeout) => {}
        }
        if ended.is_some() {
            continue;
        }
        if asked.is_none() {
            if let Some(why) = (stopping.requested)() {
                note(log, &format!("{why}: asking the daemon to stop"));
                asked = Some(Instant::now());
            }
        }
        if asked.is_some() && !accepted && last_ask.is_none_or(|t| t.elapsed() >= ASK_AGAIN) {
            match (stopping.ask)() {
                Ok(()) => accepted = true,
                Err(e) if last_ask.is_none() => {
                    note(log, &format!("cannot ask it yet ({e}); asking again"))
                }
                Err(_) => {}
            }
            last_ask = Some(Instant::now());
        }
        if asked.is_some_and(|at| at.elapsed() >= stopping.within) && !ending {
            note(
                log,
                &format!(
                    "the daemon did not stop within {} s; ending it",
                    stopping.within.as_secs_f64()
                ),
            );
            let _ = child.kill();
            ending = true;
        }
        ended = child.try_wait()?.map(|status| (status, Instant::now()));
    }
}

/// Runs the daemons `command` makes, one at a time, until one stops cleanly (exit status 0) or
/// the supervisor is asked to stop, each line the daemon writes to its standard output or error
/// going to `log`; each ends with the thread that calls this, so call it from the main thread.
/// After a failure the next starts once `policy.delay` has passed, unless the daemon refused its
/// arguments (exit status 2) or the policy allows no more quick failures. What the supervisor has
/// to say goes to the log as "cv1touchd supervisor: ...". The exit code for the supervisor: 0
/// once the daemon stopped, 1 when it gave up.
pub fn run(
    mut command: impl FnMut() -> Command,
    log: &mut Log,
    policy: Policy,
    stopping: &mut Stopping,
) -> i32 {
    let mut failures = Failures::new(policy);
    let delay = policy.delay.as_secs_f64();
    loop {
        let started = Instant::now();
        match run_once(&mut command(), log, stopping) {
            Ok(Ended { status, asked }) if asked || status.success() => {
                note(log, &format!("the daemon stopped ({status})"));
                return 0;
            }
            Ok(Ended { status, .. }) if status.code() == Some(USAGE_ERROR) => {
                note(
                    log,
                    &format!("the daemon refused its arguments ({status}); not starting it again"),
                );
                return 1;
            }
            Ok(Ended { status, .. }) => note(log, &format!("the daemon ended ({status})")),
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
        let until = Instant::now() + policy.delay;
        while let Some(left) = until.checked_duration_since(Instant::now()) {
            if let Some(why) = (stopping.requested)() {
                note(log, &format!("{why}: not starting the daemon again"));
                return 0;
            }
            std::thread::sleep(left.min(Duration::from_millis(100)));
        }
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

    /// Never asked to stop.
    fn never() -> Stopping<'static> {
        Stopping {
            requested: Box::new(|| None),
            ask: Box::new(|| Ok(())),
            within: Duration::from_secs(10),
        }
    }

    /// The file whose appearance the stand-in daemon waits for.
    const STAND_IN: &str = "CV1TOUCH_TEST_STAND_IN";

    /// Stands in for a daemon in other tests, which run this test binary for it: it ends cleanly
    /// once the file named by CV1TOUCH_TEST_STAND_IN exists, and runs at most 30 s.
    #[test]
    #[ignore = "run by other tests"]
    fn stand_in_daemon() {
        let Some(file) = std::env::var_os(STAND_IN) else {
            return;
        };
        let until = Instant::now() + Duration::from_secs(30);
        while !Path::new(&file).exists() && Instant::now() < until {
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    fn stand_in(file: &Path) -> Command {
        let mut c = Command::new(std::env::current_exe().unwrap());
        c.args(["--exact", "supervise::tests::stand_in_daemon", "--ignored"])
            .env(STAND_IN, file);
        c
    }

    /// A stop request after `after`, and how many times the daemon was asked to stop.
    fn stop_after(
        after: Duration,
        ask: impl FnMut() + 'static,
        within: Duration,
    ) -> Stopping<'static> {
        let at = Instant::now() + after;
        let mut ask = ask;
        let mut sent = false;
        Stopping {
            requested: Box::new(move || {
                (!sent && Instant::now() >= at).then(|| {
                    sent = true;
                    "Ctrl+C".to_string()
                })
            }),
            ask: Box::new(move || {
                ask();
                Ok(())
            }),
            within,
        }
    }

    #[test]
    fn asked_to_stop_the_supervisor_asks_the_daemon_and_waits_for_it() {
        let d = dir("asked");
        let mut log = Log::open(&d).unwrap();
        let file = d.join("stop");
        let asked = file.clone();
        let mut stopping = stop_after(
            Duration::from_millis(200),
            move || std::fs::write(&asked, b"").unwrap(),
            Duration::from_secs(20),
        );
        let code = run(|| stand_in(&file), &mut log, quick(10, 5), &mut stopping);
        assert_eq!(code, 0);
        let lines = texts(&log.path());
        assert!(
            lines.contains(&"cv1touchd supervisor: Ctrl+C: asking the daemon to stop".into()),
            "{lines:?}"
        );
        assert!(lines
            .last()
            .unwrap()
            .starts_with("cv1touchd supervisor: the daemon stopped"));
        assert!(!lines.iter().any(|l| l.contains("ending it")), "{lines:?}");
        std::fs::remove_dir_all(d).unwrap();
    }

    /// A daemon that does not take requests yet (it is just starting) is asked again.
    #[test]
    fn a_daemon_not_yet_taking_requests_is_asked_again() {
        let d = dir("not-yet");
        let mut log = Log::open(&d).unwrap();
        let file = d.join("stop");
        let asked = file.clone();
        let mut tries = 0;
        let at = Instant::now() + Duration::from_millis(100);
        let mut stopping = Stopping {
            requested: Box::new(move || (Instant::now() >= at).then(|| "Ctrl+C".to_string())),
            ask: Box::new(move || {
                tries += 1;
                if tries < 3 {
                    return Err("no daemon there yet".into());
                }
                std::fs::write(&asked, b"").map_err(|e| e.to_string())
            }),
            within: Duration::from_secs(20),
        };
        let code = run(|| stand_in(&file), &mut log, quick(10, 5), &mut stopping);
        assert_eq!(code, 0);
        let lines = texts(&log.path());
        let not_yet = "cv1touchd supervisor: cannot ask it yet (no daemon there yet); asking again";
        assert_eq!(
            lines.iter().filter(|l| *l == not_yet).count(),
            1,
            "{lines:?}"
        );
        assert!(!lines.iter().any(|l| l.contains("ending it")), "{lines:?}");
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_daemon_that_does_not_stop_when_asked_is_ended() {
        let d = dir("deaf");
        let mut log = Log::open(&d).unwrap();
        let file = d.join("never");
        let mut stopping = stop_after(
            Duration::from_millis(100),
            || {},
            Duration::from_millis(300),
        );
        let started = Instant::now();
        let code = run(|| stand_in(&file), &mut log, quick(10, 5), &mut stopping);
        assert_eq!(code, 0);
        assert!(started.elapsed() < Duration::from_secs(10));
        let lines = texts(&log.path());
        assert!(
            lines.contains(
                &"cv1touchd supervisor: the daemon did not stop within 0.3 s; ending it".into()
            ),
            "{lines:?}"
        );
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_stop_while_waiting_to_start_the_daemon_again_ends_the_supervisor() {
        let d = dir("waiting");
        let mut log = Log::open(&d).unwrap();
        let mut stopping = stop_after(Duration::from_millis(300), || {}, Duration::from_secs(10));
        let started = Instant::now();
        let code = run(
            || shell("exit 1"),
            &mut log,
            quick(20_000, 5),
            &mut stopping,
        );
        assert_eq!(code, 0);
        assert!(started.elapsed() < Duration::from_secs(10));
        assert_eq!(
            texts(&log.path()).last().unwrap(),
            "cv1touchd supervisor: Ctrl+C: not starting the daemon again"
        );
        std::fs::remove_dir_all(d).unwrap();
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
            &mut never(),
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
            &mut never(),
        );
        assert_eq!((code, runs), (0, 2));
        assert!(texts(&log.path())
            .last()
            .unwrap()
            .starts_with("cv1touchd supervisor: the daemon stopped"));
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
            &mut never(),
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
            &mut never(),
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
        assert_eq!(
            run(|| shell(script), &mut log, quick(10, 5), &mut never()),
            0
        );
        assert!(started.elapsed() < Duration::from_secs(10));
        std::fs::remove_dir_all(d).unwrap();
    }
}
