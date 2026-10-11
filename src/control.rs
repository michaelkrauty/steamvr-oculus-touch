// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Control protocol over $XDG_RUNTIME_DIR/cv1touch/control.sock (AF_UNIX stream): one request line
//! gets one reply line starting with "ok" or "err <reason>".

use std::fs::{File, OpenOptions};
use std::io::{self, BufRead, BufReader, Write};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::time::Duration;

use crate::hand::{
    BTN_A, BTN_B, BTN_STICK, BTN_SYSTEM, BTN_X, BTN_Y, TOUCH_A, TOUCH_B, TOUCH_STICK,
    TOUCH_TRIGGER, TOUCH_X, TOUCH_Y,
};
use crate::shm::{HAND_LEFT, HAND_RIGHT};

#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    AlignmentReload,
    /// Extra milliseconds the published poses are stamped earlier (SteamVR predicts that much
    /// further), on top of the measured radio and IMU delay; None shows the current value.
    Latency(Option<f64>),
    Haptic {
        hand: usize,
        duration_s: f32,
        frequency_hz: f32,
        amplitude: f32,
    },
    /// Path, and whether to record the sensors' pixels too.
    RecordStart(PathBuf, bool),
    RecordStop,
    RecordStopIf(PathBuf),
    Status,
    /// Factory-normalized inputs, before optional user calibration and output clamping.
    Inputs,
    InputsReload,
    CalibrationCapture,
    CalibrationProvisional,
    CalibrationLoad(PathBuf),
    /// The base stations' raw poses, as the SteamVR driver sees them from inside vrserver:
    /// (serial, [x, y, z, qx, qy, qz, qw]).
    Stations(Vec<(String, [f64; 7])>),
    /// Turn the universe watch on or off (None shows it).
    UniverseWatch(Option<bool>),
    /// Turn the velocity shaping for SteamVR's prediction on or off (None shows it).
    Shaping(Option<bool>),
    /// Arm or disarm input injection for automated tests (None shows it). Armed, it disarms
    /// itself after a while.
    TestInject(Option<bool>),
    /// While armed: press `inputs` on `hand` for `ms` milliseconds (at most `INJECT_MAX_MS`),
    /// replacing what an earlier inject for that hand still pressed.
    Inject {
        hand: usize,
        inputs: Injected,
        ms: u32,
    },
}

/// The longest press one `inject` makes, ms.
pub const INJECT_MAX_MS: u32 = 2000;

/// What an `inject` presses: buttons (each with its touch, as a finger on it gives), the trigger
/// and grip held at 1.0.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Injected {
    pub buttons: u32,
    pub touches: u32,
    pub trigger: bool,
    pub grip: bool,
}

fn injected(hand: usize, list: &str) -> Result<Injected, String> {
    let left = hand == HAND_LEFT;
    let mut v = Injected::default();
    for name in list.split(',') {
        match (name, left) {
            ("x", true) => (v.buttons, v.touches) = (v.buttons | BTN_X, v.touches | TOUCH_X),
            ("y", true) => (v.buttons, v.touches) = (v.buttons | BTN_Y, v.touches | TOUCH_Y),
            ("system", true) => v.buttons |= BTN_SYSTEM,
            ("a", false) => (v.buttons, v.touches) = (v.buttons | BTN_A, v.touches | TOUCH_A),
            ("b", false) => (v.buttons, v.touches) = (v.buttons | BTN_B, v.touches | TOUCH_B),
            ("stick", _) => {
                (v.buttons, v.touches) = (v.buttons | BTN_STICK, v.touches | TOUCH_STICK)
            }
            ("trigger", _) => (v.trigger, v.touches) = (true, v.touches | TOUCH_TRIGGER),
            ("grip", _) => v.grip = true,
            _ => {
                return Err(format!(
                    "{name}: not an input of the {} Touch ({})",
                    if left { "left" } else { "right" },
                    if left {
                        "x y system stick trigger grip"
                    } else {
                        "a b stick trigger grip"
                    }
                ))
            }
        }
    }
    Ok(v)
}

fn number(s: &str) -> Result<f32, String> {
    s.parse::<f32>()
        .ok()
        .filter(|v| v.is_finite())
        .ok_or_else(|| "duration, frequency and amplitude must be finite numbers".to_string())
}

fn word(input: &str) -> Option<(&str, &str)> {
    let input = input.trim_start();
    if input.is_empty() {
        return None;
    }
    let end = input.find(char::is_whitespace).unwrap_or(input.len());
    Some((&input[..end], &input[end..]))
}

fn path_arguments(input: &str, frames_allowed: bool) -> Result<(PathBuf, bool), String> {
    let input = input.trim_start();
    let (path, rest) = if input.starts_with('"') {
        let mut decoder = serde_json::Deserializer::from_str(input).into_iter::<String>();
        let path = decoder
            .next()
            .ok_or("missing path")?
            .map_err(|e| format!("invalid JSON-quoted path: {e}"))?;
        let rest = &input[decoder.byte_offset()..];
        if rest.chars().next().is_some_and(|c| !c.is_whitespace()) {
            return Err("expected whitespace after JSON-quoted path".into());
        }
        (path, rest)
    } else {
        let (path, rest) = word(input).ok_or("missing absolute path")?;
        (path.to_owned(), rest)
    };
    let path = PathBuf::from(path);
    if !path.is_absolute() {
        return Err("needs an absolute path".into());
    }
    if path.as_os_str().as_encoded_bytes().contains(&0) {
        return Err("path must not contain NUL".into());
    }
    let frames = match rest.trim() {
        "" => false,
        "frames" if frames_allowed => true,
        _ => {
            return Err(if frames_allowed {
                "unexpected trailing arguments (only frames is allowed)"
            } else {
                "unexpected trailing arguments after path"
            }
            .into())
        }
    };
    Ok((path, frames))
}

fn quoted_path(path: &Path) -> Result<String, String> {
    let path = path.to_str().ok_or("control paths must be UTF-8")?;
    if !Path::new(path).is_absolute() {
        return Err("control paths must be absolute".into());
    }
    if path.contains('\0') {
        return Err("path must not contain NUL".into());
    }
    serde_json::to_string(path).map_err(|e| e.to_string())
}

/// JSON quoting keeps spaces, quotes and newlines inside a single protocol argument/line.
pub fn record_start_line(path: &Path, frames: bool) -> Result<String, String> {
    Ok(format!(
        "record start {}{}",
        quoted_path(path)?,
        if frames { " frames" } else { "" }
    ))
}

pub fn calibration_load_line(path: &Path) -> Result<String, String> {
    Ok(format!("calibration load {}", quoted_path(path)?))
}

/// Stop only the recording started with this exact path, even after a lost start reply.
pub fn record_stop_if_line(path: &Path) -> Result<String, String> {
    Ok(format!("record stop-if {}", quoted_path(path)?))
}

pub fn parse(line: &str) -> Result<Command, String> {
    if let Some((verb, rest)) = word(line) {
        if let Some((subcommand, args)) = word(rest) {
            match (verb, subcommand) {
                ("record", "stop-if") => {
                    return path_arguments(args, false)
                        .map(|(path, _)| Command::RecordStopIf(path))
                        .map_err(|e| format!("record stop-if: {e}"))
                }
                ("record", "start") => {
                    return path_arguments(args, true)
                        .map(|(path, frames)| Command::RecordStart(path, frames))
                        .map_err(|e| format!("record start: {e}"))
                }
                ("calibration", "load") => {
                    return path_arguments(args, false)
                        .map(|(path, _)| Command::CalibrationLoad(path))
                        .map_err(|e| format!("calibration load: {e}"))
                }
                _ => {}
            }
        }
    }
    let words: Vec<&str> = line.split_whitespace().collect();
    match words.as_slice() {
        [] => Err("empty command".into()),
        ["status"] => Ok(Command::Status),
        ["inputs"] => Ok(Command::Inputs),
        ["inputs", "reload"] => Ok(Command::InputsReload),
        ["inputs", ..] => Err("usage: inputs [reload]".into()),
        ["stations", rest @ ..] => {
            if rest.len() % 8 != 0 {
                return Err("stations: a serial and 7 numbers (x y z qx qy qz qw) per station".into());
            }
            rest.chunks(8)
                .map(|c| {
                    let mut v = [0.0; 7];
                    for (k, w) in c[1..].iter().enumerate() {
                        v[k] = w.parse::<f64>().ok().filter(|x| x.is_finite()).ok_or_else(|| format!("stations: {w} is not a finite number"))?;
                    }
                    if v[3..].iter().map(|x| x * x).sum::<f64>() < 0.25 {
                        return Err(format!("stations: {} has no orientation", c[0]));
                    }
                    Ok((c[0].to_string(), v))
                })
                .collect::<Result<Vec<_>, String>>()
                .map(Command::Stations)
        }
        ["shaping"] => Ok(Command::Shaping(None)),
        ["shaping", "on"] => Ok(Command::Shaping(Some(true))),
        ["shaping", "off"] => Ok(Command::Shaping(Some(false))),
        ["shaping", ..] => Err("usage: shaping [on|off]".into()),
        ["universe-watch"] => Ok(Command::UniverseWatch(None)),
        ["universe-watch", "on"] => Ok(Command::UniverseWatch(Some(true))),
        ["universe-watch", "off"] => Ok(Command::UniverseWatch(Some(false))),
        ["universe-watch", ..] => Err("usage: universe-watch [on|off]".into()),
        ["test-inject"] => Ok(Command::TestInject(None)),
        ["test-inject", "on"] => Ok(Command::TestInject(Some(true))),
        ["test-inject", "off"] => Ok(Command::TestInject(Some(false))),
        ["test-inject", ..] => Err("usage: test-inject [on|off]".into()),
        ["inject", hand, list, ms] => {
            let hand = match *hand {
                "left" => HAND_LEFT,
                "right" => HAND_RIGHT,
                _ => return Err("hand must be left or right".into()),
            };
            let inputs = injected(hand, list)?;
            let ms = ms.parse::<u32>().ok().filter(|m| (1..=INJECT_MAX_MS).contains(m)).ok_or_else(|| format!("duration must be 1 to {INJECT_MAX_MS} ms"))?;
            Ok(Command::Inject { hand, inputs, ms })
        }
        ["inject", ..] => Err("usage: inject <left|right> <x|y|a|b|system|stick|trigger|grip>[,...] <ms>".into()),
        ["status", ..] => Err("unexpected extra arguments".into()),
        ["record", "stop"] => Ok(Command::RecordStop),
        ["record", ..] => Err("usage: record start <absolute-path> | record stop".into()),
        ["haptic", hand, d, f, a] => {
            let hand = match *hand {
                "left" => HAND_LEFT,
                "right" => HAND_RIGHT,
                _ => return Err("hand must be left or right".into()),
            };
            let (duration_s, frequency_hz, amplitude) = (number(d)?, number(f)?, number(a)?);
            if !(duration_s > 0.0 && duration_s <= 10.0) {
                return Err("duration must be in (0, 10] seconds".into());
            }
            if !(frequency_hz > 0.0 && frequency_hz <= 1000.0) {
                return Err("frequency must be in (0, 1000] Hz".into());
            }
            if !(0.0..=1.0).contains(&amplitude) {
                return Err("amplitude must be in [0, 1]".into());
            }
            Ok(Command::Haptic { hand, duration_s, frequency_hz, amplitude })
        }
        ["haptic", ..] => Err("usage: haptic left|right <duration_s> <frequency_hz> <amplitude>".into()),
        ["calibration", "capture"] => Ok(Command::CalibrationCapture),
        ["calibration", "provisional"] => Ok(Command::CalibrationProvisional),
        ["calibration", ..] => Err("usage: calibration capture | calibration provisional | calibration load <absolute-path>".into()),
        ["alignment", "reload"] => Ok(Command::AlignmentReload),
        ["latency"] => Ok(Command::Latency(None)),
        ["latency", ms] => {
            let v = number(ms)?;
            if v.abs() > 30.0 {
                return Err("latency adjustment must be within +-30 ms".into());
            }
            Ok(Command::Latency(Some(v as f64)))
        }
        ["alignment", ..] => Err("usage: alignment reload".into()),
        [verb, ..] => Err(format!("unknown command '{verb}'")),
    }
}

/// Exclusive, non-blocking flock on `path`; held until the returned File is dropped.
pub fn lock_single_instance(path: &Path) -> io::Result<File> {
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .mode(0o600)
        .open(path)?;
    // SAFETY: valid fd owned by `file`.
    if unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) } < 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(file)
}

pub fn runtime_dir() -> PathBuf {
    std::env::var_os("XDG_RUNTIME_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(format!("/run/user/{}", unsafe { libc::getuid() })))
        .join("cv1touch")
}

pub fn socket_path() -> PathBuf {
    runtime_dir().join("control.sock")
}

pub fn request(socket: &Path, line: &str, timeout: Duration) -> io::Result<String> {
    let mut stream = UnixStream::connect(socket)?;
    stream.set_read_timeout(Some(timeout))?;
    stream.set_write_timeout(Some(timeout))?;
    stream.write_all(line.trim_end().as_bytes())?;
    stream.write_all(b"\n")?;
    let mut reply = String::new();
    BufReader::new(stream).read_line(&mut reply)?;
    Ok(reply.trim_end().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{BufRead, BufReader, Write};
    use std::os::unix::net::UnixListener;
    use std::thread;

    /// Review Focus 5: values the engine would silently mangle must be rejected.
    #[test]
    fn scoped_stop_quotes_paths_and_rejects_malformed_arguments() {
        let path = Path::new("/tmp/a \"quoted\" \\name\nframes.rec");
        let line = record_stop_if_line(path).unwrap();
        assert!(!line.contains('\n'));
        assert_eq!(parse(&line), Ok(Command::RecordStopIf(path.to_path_buf())));
        for line in [
            "record stop-if",
            "record stop-if relative.rec",
            "record stop-if \"/tmp/unterminated",
            "record stop-if \"/tmp/a\"frames",
            "record stop-if \"/tmp/a\" frames",
            "record stop-if \"/tmp/a\\u0000b\"",
        ] {
            assert!(parse(line).is_err(), "{line}");
        }
    }

    #[test]
    fn quoted_paths_round_trip_spaces_quotes_newlines_and_frames() {
        let path = Path::new("/tmp/state with spaces/a \"quoted\" \\name\nframes.rec");
        for frames in [false, true] {
            let line = record_start_line(path, frames).unwrap();
            assert!(!line.contains('\n'), "request must stay one wire line");
            assert_eq!(
                parse(&line).unwrap(),
                Command::RecordStart(path.to_path_buf(), frames)
            );
        }
        let line = calibration_load_line(path).unwrap();
        assert_eq!(
            parse(&line).unwrap(),
            Command::CalibrationLoad(path.to_path_buf())
        );
        assert_eq!(
            parse("record start \"/tmp/file frames\"").unwrap(),
            Command::RecordStart(PathBuf::from("/tmp/file frames"), false)
        );
    }

    #[test]
    fn quoted_path_syntax_rejects_invalid_or_trailing_input() {
        for line in [
            "record start \"/tmp/unterminated",
            "record start \"/tmp/a\"frames",
            "record start \"/tmp/a\" trailing",
            "record start \"/tmp/a\" frames trailing",
            "calibration load \"/tmp/a\" frames",
            "calibration load \"relative\"",
            "record start \"/tmp/a\\u0000b\"",
            "record start",
            "calibration load",
        ] {
            assert!(parse(line).is_err(), "{line}");
        }
        assert_eq!(
            parse("  record\tstart\t/tmp/legacy.rec frames  ").unwrap(),
            Command::RecordStart(PathBuf::from("/tmp/legacy.rec"), true)
        );
    }

    #[test]
    fn generated_control_paths_reject_non_utf8_and_nul() {
        use std::os::unix::ffi::OsStringExt;
        let bad = PathBuf::from(std::ffi::OsString::from_vec(b"/tmp/non-utf8-\xff".to_vec()));
        assert!(record_start_line(&bad, false)
            .unwrap_err()
            .contains("UTF-8"));
        assert!(calibration_load_line(&bad).unwrap_err().contains("UTF-8"));
        assert!(record_start_line(Path::new("/tmp/nul\0path"), false).is_err());
    }

    #[test]
    fn parses_test_injection() {
        assert_eq!(
            parse("test-inject on").unwrap(),
            Command::TestInject(Some(true))
        );
        assert_eq!(
            parse("test-inject off").unwrap(),
            Command::TestInject(Some(false))
        );
        assert_eq!(parse("test-inject").unwrap(), Command::TestInject(None));
        assert_eq!(
            parse("inject left x,y 200").unwrap(),
            Command::Inject {
                hand: HAND_LEFT,
                inputs: Injected {
                    buttons: BTN_X | BTN_Y,
                    touches: TOUCH_X | TOUCH_Y,
                    trigger: false,
                    grip: false
                },
                ms: 200
            }
        );
        assert_eq!(
            parse("inject right a,trigger,grip,stick 2000").unwrap(),
            Command::Inject {
                hand: HAND_RIGHT,
                inputs: Injected {
                    buttons: BTN_A | BTN_STICK,
                    touches: TOUCH_A | TOUCH_TRIGGER | TOUCH_STICK,
                    trigger: true,
                    grip: true
                },
                ms: 2000
            }
        );
        assert_eq!(
            parse("inject left system 5").unwrap(),
            Command::Inject {
                hand: HAND_LEFT,
                inputs: Injected {
                    buttons: BTN_SYSTEM,
                    ..Default::default()
                },
                ms: 5
            }
        );
        // the right Touch has no x, y or system; a press is at most 2 s; a hand and a known input
        for bad in [
            "inject right x 100",
            "inject right system 100",
            "inject left a 100",
            "inject left x 2001",
            "inject left x 0",
            "inject left x -5",
            "inject left boost 100",
            "inject up x 100",
            "inject left x",
            "test-inject maybe",
        ] {
            assert!(parse(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn parse_alignment_reload() {
        assert_eq!(parse("alignment reload"), Ok(Command::AlignmentReload));
        assert_eq!(parse("latency"), Ok(Command::Latency(None)));
        assert_eq!(parse("latency -2.5"), Ok(Command::Latency(Some(-2.5))));
        assert!(parse("latency 40").is_err());
        assert!(parse("alignment").is_err());
    }

    #[test]
    fn parse_haptic() {
        assert_eq!(
            parse("haptic right 0.25 160 1"),
            Ok(Command::Haptic {
                hand: HAND_RIGHT,
                duration_s: 0.25,
                frequency_hz: 160.0,
                amplitude: 1.0
            })
        );
        assert!(matches!(
            parse("haptic left 0.1 320 0\n"),
            Ok(Command::Haptic {
                hand: HAND_LEFT,
                ..
            })
        ));
        assert!(parse("haptic right 0.25 160 1.5")
            .unwrap_err()
            .contains("amplitude"));
        assert!(parse("haptic right 0.25 160 -0.1").is_err());
        assert!(parse("haptic right 0.25 160 nan").is_err());
        assert!(parse("haptic right 0 160 1")
            .unwrap_err()
            .contains("duration"));
        assert!(parse("haptic right 11 160 1").is_err());
        assert!(parse("haptic right 0.2 -5 1")
            .unwrap_err()
            .contains("frequency"));
        assert!(parse("haptic middle 0.2 160 1")
            .unwrap_err()
            .contains("left or right"));
        assert!(parse("haptic right 0.2 160").is_err());
        assert!(parse("haptic right 0.2 160 1 extra").is_err());
        assert!(parse("haptic right 0.2x 160 1").is_err());
    }

    /// The driver reports the base stations' raw poses from inside SteamVR; serial, position (m),
    /// orientation (x y z w) per station.
    #[test]
    fn parse_stations_and_the_universe_watch_switch() {
        let c = parse("stations LHB-TEST-A 1.85 0.36 -2.61 0.01 0.38 -0.02 0.92 LHB-TEST-B -1.5 0.35 -2.56 0 0 0 1").unwrap();
        let Command::Stations(v) = c else {
            panic!("{c:?}")
        };
        assert_eq!(v.len(), 2);
        assert_eq!(v[0].0, "LHB-TEST-A");
        assert_eq!(v[1].1, [-1.5, 0.35, -2.56, 0.0, 0.0, 0.0, 1.0]);
        assert_eq!(parse("stations"), Ok(Command::Stations(vec![])));
        assert!(
            parse("stations LHB-1 1 2 3 0 0 0").is_err(),
            "a station needs 7 numbers"
        );
        assert!(parse("stations LHB-1 1 2 3 0 0 0 nan").is_err());
        assert!(
            parse("stations LHB-1 1 2 3 0 0 0 0").is_err(),
            "a zero quaternion is no orientation"
        );
        assert_eq!(parse("universe-watch"), Ok(Command::UniverseWatch(None)));
        assert_eq!(
            parse("universe-watch off"),
            Ok(Command::UniverseWatch(Some(false)))
        );
        assert_eq!(
            parse("universe-watch on"),
            Ok(Command::UniverseWatch(Some(true)))
        );
        assert!(parse("universe-watch maybe").is_err());
        assert_eq!(parse("shaping"), Ok(Command::Shaping(None)));
        assert_eq!(parse("shaping off"), Ok(Command::Shaping(Some(false))));
        assert_eq!(parse("shaping on"), Ok(Command::Shaping(Some(true))));
        assert!(parse("shaping 1").is_err());
    }

    #[test]
    fn parse_other() {
        assert_eq!(parse("status"), Ok(Command::Status));
        assert!(parse("status now").is_err());
        assert_eq!(parse("record stop"), Ok(Command::RecordStop));
        assert_eq!(
            parse("record start /tmp/a.rec"),
            Ok(Command::RecordStart(PathBuf::from("/tmp/a.rec"), false))
        );
        assert_eq!(
            parse("record start /tmp/a.rec frames"),
            Ok(Command::RecordStart(PathBuf::from("/tmp/a.rec"), true))
        );
        assert!(parse("record start relative.rec")
            .unwrap_err()
            .contains("absolute"));
        assert!(parse("record").is_err());
        assert!(parse("").is_err());
        assert!(parse("frobnicate").unwrap_err().contains("unknown"));
    }

    /// Review Focus 3: a second daemon must not get the lock.
    #[test]
    fn single_instance_lock() {
        let path = std::env::temp_dir().join(format!("cv1t-lock-test-{}", std::process::id()));
        let first = lock_single_instance(&path).unwrap();
        let err = lock_single_instance(&path).unwrap_err();
        assert_eq!(err.raw_os_error(), Some(libc::EWOULDBLOCK));
        drop(first);
        let again = lock_single_instance(&path).unwrap();
        drop(again);
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn request_roundtrip() {
        let path = std::env::temp_dir().join(format!("cv1t-ctl-test-{}.sock", std::process::id()));
        let _ = std::fs::remove_file(&path);
        let listener = UnixListener::bind(&path).unwrap();
        let server = thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            let mut line = String::new();
            BufReader::new(&stream).read_line(&mut line).unwrap();
            (&stream).write_all(b"ok pong\n").unwrap();
            line
        });
        let reply = request(&path, "status", Duration::from_secs(2)).unwrap();
        assert_eq!(reply, "ok pong");
        assert_eq!(server.join().unwrap(), "status\n");
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn parse_calibration() {
        assert_eq!(
            parse("calibration capture"),
            Ok(Command::CalibrationCapture)
        );
        assert_eq!(
            parse("calibration provisional"),
            Ok(Command::CalibrationProvisional)
        );
        assert_eq!(
            parse("calibration load /tmp/cv1touch-test/.config/cv1touch/calibration.json"),
            Ok(Command::CalibrationLoad(PathBuf::from(
                "/tmp/cv1touch-test/.config/cv1touch/calibration.json"
            )))
        );
        assert!(parse("calibration load relative.json")
            .unwrap_err()
            .contains("absolute"));
        assert!(parse("calibration").unwrap_err().contains("usage"));
    }

    #[test]
    fn input_sampling_and_reload_have_no_implicit_write_command() {
        assert_eq!(parse("inputs"), Ok(Command::Inputs));
        assert_eq!(parse("inputs reload"), Ok(Command::InputsReload));
        for command in ["inputs reset", "inputs reload extra", "inputs write"] {
            assert!(parse(command).is_err());
        }
    }
}
