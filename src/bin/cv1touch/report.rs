// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! `cv1touch report`: one zip file with what it takes to see what went wrong on someone else's
//! computer. A summary (version, system, `doctor`'s checks, the Oculus USB devices and the
//! drivers they run with, which programs run, the daemon's status, setup's state), the daemon's
//! and SteamVR's logs (their ends), and cv1touch's settings and calibration. Nothing else: no
//! recordings, nothing outside cv1touch's and SteamVR's files (SteamVR's log names the other VR
//! devices, the user's folder and the network addresses: `CONTENTS` says so).

use super::*;
use cv1touch::calendar::Utc;
use cv1touch::supervise;
use cv1touch::zip::ZipWriter;
use std::io::{Read, Seek, SeekFrom};

/// The Oculus USB vendor: the headset, its sensors and the Touch controllers' dongle.
const OCULUS: u16 = 0x2833;

/// The most of one log a report keeps: its end, where the problem usually is.
const LOG_TAIL: u64 = 4 << 20;

/// At most the last `max` bytes of `data`, from the start of a line, with a note saying how much
/// was left out; `before` bytes came before `data` (left out already).
fn line_tail(data: &[u8], max: usize, before: u64) -> Vec<u8> {
    if before == 0 && data.len() <= max {
        return data.to_vec();
    }
    let cut = data.len().saturating_sub(max);
    // a cut right after a line's end is a line's start; with no line start after it (one line
    // longer than `max`), the end of that line is kept
    let start = if cut > 0 && data[cut - 1] == b'\n' {
        cut
    } else {
        data[cut..]
            .iter()
            .position(|&b| b == b'\n')
            .map(|i| cut + i + 1)
            .filter(|&start| start < data.len())
            .unwrap_or(cut)
    };
    let mut kept =
        format!("[the first {} bytes are left out]\n", before + start as u64).into_bytes();
    kept.extend_from_slice(&data[start..]);
    kept
}

/// The end of the regular file at `path`, at most `max` bytes, from the start of a line (see
/// `line_tail`). Anything else (a FIFO, a device) is refused rather than waited on, and a file
/// still growing is read as long as it was.
fn tail(path: &Path, max: u64) -> io::Result<Vec<u8>> {
    let mut file = platform::fs::open_read_nonblocking(path)?;
    let meta = file.metadata()?;
    if !meta.is_file() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "not a regular file",
        ));
    }
    let len = meta.len();
    // one byte more than kept, to see whether the cut falls at a line's start
    let from = len.saturating_sub(max + 1);
    file.seek(SeekFrom::Start(from))?;
    let mut end = Vec::new();
    file.take(len - from).read_to_end(&mut end)?;
    Ok(line_tail(&end, max as usize, from))
}

fn modified(path: &Path) -> SystemTime {
    std::fs::metadata(path)
        .and_then(|m| m.modified())
        .unwrap_or_else(|_| SystemTime::now())
}

/// A report being put together: its files, and what could not be collected.
struct Report {
    files: Vec<(String, Vec<u8>, SystemTime)>,
    missing: Vec<String>,
    /// What cv1touch's directories hold.
    listed: Vec<String>,
}

impl Report {
    /// The end of the file at `from` as `name`; a missing file is noted only if `expected`.
    fn log(&mut self, name: &str, from: &Path, expected: bool) {
        match tail(from, LOG_TAIL) {
            Ok(data) => self.files.push((name.into(), data, modified(from))),
            Err(e) if e.kind() == io::ErrorKind::NotFound && !expected => {}
            Err(e) => self
                .missing
                .push(format!("{name}: {} ({e})", from.display())),
        }
    }

    /// The names and sizes of the files directly in `dir`, for the summary: recordings and
    /// backups are not sent, but whether they are there can matter.
    fn listing(&mut self, dir: &Path) {
        let Ok(entries) = std::fs::read_dir(dir) else {
            return;
        };
        let mut lines: Vec<String> = entries
            .filter_map(|e| e.ok())
            .filter_map(|e| {
                let name = e.file_name().to_string_lossy().into_owned();
                let meta = e.metadata().ok()?;
                Some(if meta.is_dir() {
                    format!("  {name}/")
                } else if meta.is_symlink() {
                    let to = std::fs::read_link(e.path()).unwrap_or_default();
                    format!("  {name} -> {}", to.display())
                } else {
                    format!("  {name}  {} bytes", meta.len())
                })
            })
            .collect();
        lines.sort();
        self.listed.push(format!("{}:", dir.display()));
        self.listed.extend(lines);
    }
}

/// One line per device or interface.
fn usb_lines(nodes: &[cv1touch::platform::usb::Node]) -> Vec<String> {
    nodes
        .iter()
        .map(|n| {
            let driver = n.driver.as_deref().unwrap_or("none");
            let problem = n
                .problem
                .map_or(String::new(), |p| format!(", Windows problem code {p}"));
            let name = if n.name.is_empty() { "-" } else { &n.name };
            format!(
                "  {:04x}:{:04x}  {name}  (driver: {driver}{problem})  {}",
                OCULUS, n.product, n.id
            )
        })
        .collect()
}

fn pretty(value: &serde_json::Value) -> String {
    serde_json::to_string_pretty(value).unwrap_or_default()
}

/// The summary, report.txt.
fn summary(created: SystemTime, report: &mut Report) -> String {
    let mut text = vec![
        format!("cv1touch report, {}", Utc::of(created).iso()),
        format!(
            "cv1touch {} (shm ABI {})",
            env!("CARGO_PKG_VERSION"),
            cv1touch::ffi::CV1T_SHM_VERSION
        ),
        format!("System: {}", platform::system::description()),
        String::new(),
        "Checks (cv1touch doctor)".into(),
    ];
    let (checks, daemon) = doctor_checks();
    for (name, ok, detail) in &checks {
        text.push(format!(
            "  {} {name}: {detail}",
            if *ok { "ok" } else { "FAIL" }
        ));
    }
    text.push(String::new());
    text.push(format!("Oculus USB devices (vendor {OCULUS:04x})"));
    match platform::usb::devices(OCULUS) {
        Ok(nodes) if nodes.is_empty() => text.push("  none plugged in".into()),
        Ok(nodes) => text.extend(usb_lines(&nodes)),
        Err(e) => text.push(format!("  cannot list them: {e}")),
    }
    text.push(String::new());
    text.push("Programs".into());
    text.push(match platform::process::running("vrserver") {
        Ok(true) => "  SteamVR (vrserver): running".into(),
        Ok(false) => "  SteamVR (vrserver): not running".into(),
        Err(e) => format!("  SteamVR (vrserver): cannot tell ({e})"),
    });
    text.push(match &daemon {
        Some(_) => format!(
            "  cv1touchd: answering at {}",
            control::socket_path().display()
        ),
        None => format!(
            "  cv1touchd: not answering at {}",
            control::socket_path().display()
        ),
    });
    text.push(String::new());
    text.push("Daemon status (cv1touch status --json)".into());
    text.push(
        daemon
            .as_ref()
            .map_or("  none: the daemon is not answering".into(), pretty),
    );
    text.push(String::new());
    text.push("Setup (cv1touch setup --check --json)".into());
    text.push(match guided_setup::check_json() {
        Ok(value) => pretty(&value),
        Err(e) => format!("  {e}"),
    });
    text.push(String::new());
    text.push(
        "Files in cv1touch's directories (only its settings and calibration are sent)".into(),
    );
    if report.listed.is_empty() {
        text.push("  none: the directories do not exist yet".into());
    }
    text.append(&mut report.listed);
    if !report.missing.is_empty() {
        text.push(String::new());
        text.push("Could not be collected".into());
        text.extend(report.missing.drain(..).map(|m| format!("  {m}")));
    }
    text.join("\n") + "\n"
}

/// The daemon's journal for this boot (Linux, where systemd runs it).
#[cfg(unix)]
fn journal(report: &mut Report) {
    let output = Command::new("journalctl")
        .args([
            "--user",
            "-u",
            "cv1touchd",
            "-b",
            "--no-pager",
            "-o",
            "short-iso-precise",
        ])
        .stdin(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .output();
    match output {
        Ok(out) if out.status.success() => {
            let data = line_tail(&out.stdout, LOG_TAIL as usize, 0);
            report
                .files
                .push(("logs/journal.txt".into(), data, SystemTime::now()));
        }
        Ok(out) => report
            .missing
            .push(format!("the daemon's journal: journalctl {}", out.status)),
        Err(e) => report
            .missing
            .push(format!("the daemon's journal: journalctl: {e}")),
    }
}

#[cfg(windows)]
fn journal(_report: &mut Report) {}

/// Everything but the summary.
fn collect() -> Report {
    let mut report = Report {
        files: Vec::new(),
        missing: Vec::new(),
        listed: Vec::new(),
    };
    // the supervisor's logs: on Windows, where it runs the daemon, the current one is expected
    let logs = supervise::log_dir();
    report.log(
        "logs/cv1touchd.log",
        &logs.join("cv1touchd.log"),
        cfg!(windows),
    );
    for age in 1..=4 {
        let name = format!("cv1touchd.{age}.log");
        report.log(&format!("logs/{name}"), &logs.join(&name), false);
    }
    journal(&mut report);
    let steamvr = steamvr::log_dir();
    report.log("steamvr/vrserver.txt", &steamvr.join("vrserver.txt"), false);
    report.log(
        "steamvr/vrserver.previous.txt",
        &steamvr.join("vrserver.previous.txt"),
        false,
    );
    // the settings and calibration cv1touch keeps, which are small; not its recordings
    let kept = [
        ("config", file::default_path()),
        ("config", align::registration_path()),
        ("config", align::render_path()),
        ("config", cv1touch::input_calibration::default_path()),
        ("config", cv1touch::settings::default_path()),
        ("config", cv1touch::track::landmarks::default_path()),
        ("state", cv1touch::track::biases::default_path()),
        ("state", guided_setup::journal_path()),
    ];
    for (prefix, path) in kept {
        let name = format!("{prefix}/{}", path.file_name().unwrap().to_string_lossy());
        report.log(&name, &path, false);
    }
    let dirs = platform::dirs::get();
    report.listing(&dirs.config);
    report.listing(&dirs.state);
    report
}

/// Writes a new file at `path` with `write` into what `create` makes there: if writing fails,
/// the half-written file is removed; a file `create` refuses (one already there) is never
/// touched.
fn save<W>(
    path: &Path,
    create: impl FnOnce(&Path) -> io::Result<W>,
    write: impl FnOnce(W) -> io::Result<()>,
) -> io::Result<()> {
    let out = create(path)?;
    let written = write(out);
    if written.is_err() {
        let _ = std::fs::remove_file(path);
    }
    written
}

/// The report as a zip file at `path`, which must not exist yet: created readable by this user
/// only (on Windows, as the folder's files are), as the files it copies are.
fn write(path: &Path, created: SystemTime, report: &mut Report) -> io::Result<()> {
    let summary = summary(created, report);
    save(path, platform::fs::create_private, |file| {
        let mut zip = ZipWriter::new(io::BufWriter::new(file));
        zip.add("report.txt", summary.as_bytes(), created)?;
        for (name, data, at) in &report.files {
            zip.add(name, data, *at)?;
        }
        zip.finish()?.into_inner()?.sync_all()
    })
}

/// Where a report goes unless asked: the desktop on Windows, where people look for a file they
/// were asked to send; the current directory on Linux. Named for when it was made, in UTC.
fn default_path(created: SystemTime) -> PathBuf {
    let t = Utc::of(created);
    let name = format!(
        "cv1touch-report-{:04}-{:02}-{:02}-{:02}{:02}{:02}-UTC.zip",
        t.year, t.month, t.day, t.hour, t.minute, t.second
    );
    platform::dirs::desktop()
        .filter(|d| d.is_dir())
        .unwrap_or_default()
        .join(name)
}

/// What the report holds, for the person sending it.
const CONTENTS: &str = concat!(
    "It holds:\n",
    "- cv1touch's logs, settings and calibration (the calibration says where your sensors ",
    "stand in your room);\n",
    "- SteamVR's log, which also names your other VR devices, your user folder and your ",
    "network addresses;\n",
    "- the Oculus devices plugged in, and your system's version.\n",
    "Nothing else: no recordings, and no other files."
);

pub(super) fn run(out: Option<PathBuf>) -> i32 {
    let created = SystemTime::now();
    let named = out.is_some();
    let path = out.unwrap_or_else(|| default_path(created));
    let shown = std::path::absolute(&path).unwrap_or_else(|_| path.clone());
    println!("Collecting cv1touch's logs and settings into one file...");
    let mut report = collect();
    if let Err(e) = write(&path, created, &mut report) {
        if e.kind() == io::ErrorKind::AlreadyExists {
            let instead = if named {
                "Name another file"
            } else {
                "Run it again in a second, or name another file"
            };
            eprintln!(
                "There is already a file at {}, so the report was not saved. {instead}: cv1touch \
                 report --out PATH",
                shown.display()
            );
        } else {
            eprintln!(
                "The report could not be saved at {} ({e}). Name another place: cv1touch \
                 report --out PATH",
                shown.display()
            );
        }
        return 1;
    }
    println!("Saved the report: {}", shown.display());
    if platform::process::show_file(&shown) {
        println!("A window shows it now.");
    }
    println!("Send that file to whoever is helping you. {CONTENTS}");
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_long_log_keeps_its_end_from_a_line_start() {
        let dir = std::env::temp_dir().join(format!("cv1t-report-tail-{}", process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("log");
        std::fs::write(&path, "first line\nsecond line\nthird line\n").unwrap();
        assert_eq!(
            tail(&path, 1000).unwrap(),
            b"first line\nsecond line\nthird line\n"
        );
        // 15 bytes from the end fall inside "second line"
        assert_eq!(
            String::from_utf8(tail(&path, 15).unwrap()).unwrap(),
            "[the first 23 bytes are left out]\nthird line\n"
        );
        // 11 bytes from the end are exactly the last line
        assert_eq!(
            String::from_utf8(tail(&path, 11).unwrap()).unwrap(),
            "[the first 23 bytes are left out]\nthird line\n"
        );
        std::fs::remove_dir_all(dir).unwrap();
    }

    /// A FIFO (or a link to a device) would never end: it is refused, not read.
    #[cfg(unix)]
    #[test]
    fn only_regular_files_are_read() {
        let dir = std::env::temp_dir().join(format!("cv1t-report-fifo-{}", process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let fifo = dir.join("settings.json");
        let name = std::ffi::CString::new(fifo.to_str().unwrap()).unwrap();
        // SAFETY: a NUL-terminated path.
        assert_eq!(unsafe { libc::mkfifo(name.as_ptr(), 0o600) }, 0);
        assert!(tail(&fifo, 1000).is_err());
        std::os::unix::fs::symlink("/dev/zero", dir.join("zero")).unwrap();
        assert!(tail(&dir.join("zero"), 1000).is_err());
        std::fs::remove_dir_all(dir).unwrap();
    }

    /// A line longer than what is kept keeps its end, not nothing.
    #[test]
    fn a_long_line_keeps_its_end() {
        let data = b"short\na very long line without an end";
        assert_eq!(
            line_tail(data, 10, 0),
            b"[the first 27 bytes are left out]\nout an end"
        );
    }

    /// Writing that fails partway removes the file it was writing; a file already there stays.
    #[test]
    fn a_failed_save_leaves_no_half_file_and_never_touches_another() {
        struct Failing;
        impl Write for Failing {
            fn write(&mut self, _: &[u8]) -> io::Result<usize> {
                Err(io::Error::other("disk full"))
            }
            fn flush(&mut self) -> io::Result<()> {
                Ok(())
            }
        }
        let dir = std::env::temp_dir().join(format!("cv1t-report-save-{}", process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("report.zip");
        // left by an earlier run that stopped half-way
        let _ = std::fs::remove_file(&path);
        let created = |p: &Path| platform::fs::create_private(p).map(|_| Failing);
        let failed = save(&path, created, |mut out| out.write_all(b"zip"));
        assert!(failed.is_err() && !path.exists());
        std::fs::write(&path, b"someone else's").unwrap();
        assert!(save(&path, created, |mut out| out.write_all(b"zip")).is_err());
        assert_eq!(std::fs::read(&path).unwrap(), b"someone else's");
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn a_journal_is_cut_at_a_line_start() {
        let journal = b"one\ntwo\nthree\n";
        assert_eq!(line_tail(journal, 100, 0), journal);
        assert_eq!(
            line_tail(journal, 8, 0),
            b"[the first 8 bytes are left out]\nthree\n"
        );
    }

    #[test]
    fn devices_are_listed_one_per_line() {
        let node = cv1touch::platform::usb::Node {
            id: "1-2:1.0".into(),
            product: 0x0211,
            name: "interface 00 (video)".into(),
            driver: None,
            problem: Some(28),
            parent: Some("1-2".into()),
        };
        assert_eq!(
            usb_lines(&[node]),
            ["  2833:0211  interface 00 (video)  (driver: none, Windows problem code 28)  1-2:1.0"]
        );
    }

    #[test]
    fn the_default_name_says_when() {
        let at = std::time::UNIX_EPOCH + Duration::from_secs(1_791_997_503);
        let name = default_path(at);
        assert_eq!(
            name.file_name().unwrap(),
            "cv1touch-report-2026-10-14-170503-UTC.zip"
        );
    }
}
