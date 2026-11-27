//! Notification-area process; the status, sensor views and setup windows each run in their
//! own process.

use crate::{actions::ServiceAction, client::StatusResult, icon::Tint, model::Daemon};
use eframe::egui::{Context, ViewportCommand};
use serde_json::Value;
use std::{
    process::{Child, Command, Stdio},
    sync::{
        mpsc::{self, Receiver, Sender},
        Arc, Mutex,
    },
    thread,
    time::Instant,
};

#[cfg(target_os = "linux")]
use linux as platform;
#[cfg(windows)]
use windows as platform;

/// What the tray asks of a window process.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Message {
    Open,
    Close,
}

/// The windows, each its own process with its own single instance.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Status,
    Sensors,
    Setup,
}

impl Kind {
    fn flag(self) -> &'static str {
        match self {
            Self::Status => "--window",
            Self::Sensors => "--sensors",
            Self::Setup => "--setup",
        }
    }

    /// The menu entry that shows it, and the one that hides it.
    fn labels(self) -> [&'static str; 2] {
        match self {
            Self::Status => ["Show status window", "Hide status window"],
            Self::Sensors => ["Show sensor views", "Hide sensor views"],
            Self::Setup => ["Set up…", "Set up…"],
        }
    }

    fn run(self) {
        match self {
            Self::Status => crate::window::run(),
            Self::Sensors => crate::cameras::run(),
            Self::Setup => crate::setup_window::run(),
        }
    }

    fn spawn(self) -> std::io::Result<Child> {
        self.spawn_with(&[self.flag()])
    }

    fn spawn_with(self, args: &[&str]) -> std::io::Result<Child> {
        let mut command = Command::new(std::env::current_exe()?);
        command.args(args).stdin(Stdio::null());
        #[cfg(target_os = "linux")]
        if let Some(token) = ksni::current_xdg_activation_token() {
            command.env("XDG_ACTIVATION_TOKEN", token);
        }
        command.spawn()
    }
}

/// Raise a window, or start it; for a window opening another.
pub fn show(kind: Kind) -> std::io::Result<()> {
    show_with(kind, &[kind.flag()])
}

/// Raise the setup window, or start it for `request`. A setup window already open keeps
/// what it is doing.
pub fn show_setup(request: crate::setup_client::Request) -> std::io::Result<()> {
    show_with(Kind::Setup, &request.gui_args())
}

fn show_with(kind: Kind, args: &[&str]) -> std::io::Result<()> {
    if !platform::signal_window(kind, Message::Open) {
        let mut child = kind.spawn_with(args)?;
        thread::spawn(move || child.wait());
    }
    Ok(())
}

/// Saves a diagnostic report in the background and shows its folder; `done` gets the
/// tray's line about it.
fn save_report(done: impl FnOnce(String) + Send + 'static) {
    crate::actions::background(crate::actions::report, |result| {
        done(match result {
            Ok(saved) => match crate::actions::open_folder(&saved.path) {
                Ok(()) => saved.summary(),
                Err(error) => format!("{}; could not show its folder: {error}", saved.summary()),
            },
            Err(error) => format!("Report failed: {error}"),
        })
    });
}

struct Window {
    kind: Kind,
    child: Option<Child>,
}

impl Window {
    fn new(kind: Kind) -> Self {
        Self { kind, child: None }
    }

    /// Forget a window that has exited; true if that changed anything.
    fn reap(&mut self) -> bool {
        let exited = self
            .child
            .as_mut()
            .is_some_and(|child| !matches!(child.try_wait(), Ok(None)));
        if exited {
            self.child = None;
        }
        exited
    }

    fn is_open(&mut self) -> bool {
        self.reap();
        self.child.is_some()
    }

    fn open(&mut self) {
        self.reap();
        if platform::signal_window(self.kind, Message::Open) || self.child.is_some() {
            return;
        }
        match self.kind.spawn() {
            Ok(child) => self.child = Some(child),
            Err(error) => eprintln!("cv1touch-gui: Could not open the window: {error}"),
        }
    }

    /// Open the window, or close the one this tray opened.
    fn toggle(&mut self) {
        if self.is_open() {
            platform::signal_window(self.kind, Message::Close);
        } else {
            self.open();
        }
    }

    fn toggle_label(&mut self) -> &'static str {
        self.kind.labels()[usize::from(self.is_open())]
    }
}

pub fn run_window(kind: Kind) -> Result<(), Box<dyn std::error::Error>> {
    platform::run_window(kind)
}

/// Closes the windows and ends the tray, which leaves cv1touchd running.
pub fn quit() -> Result<(), Box<dyn std::error::Error>> {
    close_windows();
    platform::quit()
}

pub(super) fn close_windows() {
    for kind in [Kind::Status, Kind::Sensors, Kind::Setup] {
        platform::signal_window(kind, Message::Close);
    }
}

pub fn listen_for_activation(ctx: &Context) {
    platform::listen_for_activation(ctx);
}

fn handle(ctx: &Context, message: Message) {
    match message {
        Message::Open => raise_window(ctx),
        Message::Close => ctx.send_viewport_cmd(ViewportCommand::Close),
    }
    ctx.request_repaint();
}

fn raise_window(ctx: &Context) {
    ctx.send_viewport_cmd(ViewportCommand::Minimized(false));
    ctx.send_viewport_cmd(ViewportCommand::Focus);
    #[cfg(target_os = "linux")]
    if std::env::var_os("WAYLAND_DISPLAY").is_some() {
        ctx.send_viewport_cmd(ViewportCommand::RequestUserAttention(
            eframe::egui::UserAttentionType::Informational,
        ));
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Health {
    Checking,
    Ready,
    Warning,
    Stopped,
    Unknown,
}

impl Health {
    fn from_doctor(value: &Value) -> Self {
        if value.get("daemon").is_some_and(Value::is_null) {
            return Self::Stopped;
        }
        if value["daemon"].is_object()
            && value["checks"].as_array().is_some_and(|checks| {
                !checks.is_empty()
                    && checks.iter().all(|check| {
                        check["ok"].as_bool() == Some(true)
                            && check["detail"].as_str().is_some_and(|detail| {
                                !detail.starts_with("todo: ") && !detail.starts_with("warning: ")
                            })
                    })
            })
        {
            Self::Ready
        } else {
            Self::Warning
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::Checking => "Checking",
            Self::Ready => "Ready",
            Self::Warning => "Needs attention",
            Self::Stopped => "Daemon stopped",
            Self::Unknown => "Status unknown",
        }
    }

    fn tint(self) -> Tint {
        match self {
            Self::Ready => Tint::Ready,
            Self::Warning => Tint::Warning,
            Self::Checking | Self::Stopped | Self::Unknown => Tint::Stopped,
        }
    }
}

/// What the tray shows: the icon's health, the summary line, and the daemon's state.
#[derive(Clone, Debug, PartialEq)]
struct Report {
    health: Health,
    summary: String,
    daemon: Daemon,
}

impl Default for Report {
    fn default() -> Self {
        Self {
            health: Health::Checking,
            summary: "cv1touch: checking…".into(),
            daemon: Daemon::Checking,
        }
    }
}

impl Report {
    /// Whether the menu offers `action` now.
    fn allows(&self, action: ServiceAction) -> bool {
        !crate::actions::busy() && self.daemon.allows(action)
    }
}

// A doctor result cannot turn a failed status probe green, and an offline
// diagnosis must have started after the current status outage began.
#[derive(Default)]
struct HealthState {
    status: Option<bool>,
    outage: Option<Instant>,
    /// The current outage is a daemon known not to run.
    down: bool,
    /// Any status probe has finished.
    seen: bool,
    last_failure: Option<Instant>,
    live_warning: bool,
    detail: String,
    doctor: Option<(Health, Instant)>,
}

/// A short live summary: sensors and battery levels, as in "5 sensors · L 85% R 90%".
fn summary(value: &Value) -> String {
    let sensors = value["sensors"].as_array().map_or(0, Vec::len);
    let hands: Vec<String> = ["L", "R"]
        .iter()
        .enumerate()
        .filter_map(|(i, side)| {
            let hand = &value["hands"][i];
            let asleep = hand["flags"]
                .as_u64()
                .is_some_and(|f| f & 1 == 0 || f & 16 != 0);
            match hand["battery_percent"].as_i64() {
                _ if asleep => Some(format!("{side} asleep")),
                Some(b) => Some(format!("{side} {b}%")),
                None => None,
            }
        })
        .collect();
    let sensors = format!("{sensors} sensor{}", if sensors == 1 { "" } else { "s" });
    if hands.is_empty() {
        sensors
    } else {
        format!("{sensors} · {}", hands.join(" "))
    }
}

impl HealthState {
    fn status(&mut self, result: &StatusResult, now: Instant) -> bool {
        self.seen = true;
        match result {
            Ok(value) => {
                self.status = value["engine_ready"].as_bool();
                self.live_warning = matches!(
                    value["alignment"]["headset"].as_str(),
                    Some("other" | "universe_changed")
                ) || value["alignment"]["unmeasured_base"].as_str().is_some()
                    || value["sensors"].as_array().is_some_and(|sensors| {
                        sensors.iter().any(|sensor| {
                            matches!(
                                sensor["pose"]["state"].as_str(),
                                Some("moved" | "given_up" | "released")
                            ) || sensor["placed"].as_u64() == Some(0)
                        })
                    });
                self.detail = summary(value);
                self.outage = None;
                self.down = false;
                false
            }
            Err(error) => {
                self.status = None;
                self.down = error.down;
                self.detail.clear();
                if self.outage.is_none() {
                    self.outage = Some(now);
                    self.last_failure = Some(now);
                    self.doctor = None;
                    true
                } else {
                    false
                }
            }
        }
    }

    fn health(&self) -> Health {
        if let Some(outage) = self.outage {
            return match self.doctor {
                _ if self.down => Health::Stopped,
                Some((Health::Stopped, started)) if started >= outage => Health::Stopped,
                _ => Health::Unknown,
            };
        }
        if self.live_warning && self.status.is_some() {
            return Health::Warning;
        }
        match (self.status, self.doctor.map(|d| d.0)) {
            (Some(false), _) => Health::Warning,
            (Some(true), Some(Health::Ready)) => Health::Ready,
            (Some(true), Some(Health::Warning)) => Health::Warning,
            (Some(true), _) => Health::Unknown,
            (None, _) => Health::Checking,
        }
    }

    fn report(&self) -> Report {
        let health = self.health();
        let daemon = match health {
            Health::Stopped => Daemon::Stopped,
            // A daemon that answers, or is there but not answering, can be restarted.
            _ if self.seen => Daemon::Running,
            _ => Daemon::Checking,
        };
        Report {
            health,
            summary: if self.detail.is_empty() {
                health.label().to_owned()
            } else {
                format!("{} · {}", health.label(), self.detail)
            },
            daemon,
        }
    }
}

fn poll_health() -> (Receiver<Report>, Sender<()>) {
    let (updates, results) = mpsc::channel();
    let (stop, stopped) = mpsc::channel();
    let (doctor_request, requests) = mpsc::channel();
    let state = Arc::new(Mutex::new(HealthState::default()));
    let doctor_state = Arc::clone(&state);
    let doctor_updates = updates.clone();
    thread::spawn(move || loop {
        let started = Instant::now();
        let result = crate::client::invoke("doctor");
        let mut state = doctor_state.lock().unwrap();
        if state.last_failure.is_none_or(|failure| started >= failure) {
            state.doctor = Some((
                result.as_ref().map_or(Health::Unknown, Health::from_doctor),
                started,
            ));
        }
        if doctor_updates.send(state.report()).is_err() {
            break;
        }
        drop(state);
        match requests.recv_timeout(std::time::Duration::from_secs(30)) {
            Ok(()) => while requests.try_recv().is_ok() {},
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }
    });
    thread::spawn(move || loop {
        let result = crate::client::status();
        let mut state = state.lock().unwrap();
        if state.status(&result, Instant::now()) {
            let _ = doctor_request.send(());
        }
        if updates.send(state.report()).is_err() {
            break;
        }
        drop(state);
        if !matches!(
            stopped.recv_timeout(crate::client::STATUS_INTERVAL),
            Err(mpsc::RecvTimeoutError::Timeout)
        ) {
            break;
        }
    });
    (results, stop)
}

#[cfg(target_os = "linux")]
mod linux {
    use super::*;
    use ksni::blocking::TrayMethods;
    use std::{
        error::Error,
        fs::OpenOptions,
        os::{fd::AsRawFd, unix::net::UnixDatagram},
        sync::atomic::{AtomicBool, Ordering},
        time::Duration,
    };

    unsafe extern "C" {
        fn flock(fd: i32, operation: i32) -> i32;
    }

    struct Tray {
        report: Report,
        action_message: Option<String>,
        action_updates: Sender<Option<String>>,
        window: Arc<Mutex<Window>>,
        sensors: Arc<Mutex<Window>>,
        quit: Arc<AtomicBool>,
    }

    fn item(
        label: &str,
        enabled: bool,
        f: impl Fn(&mut Tray) + Send + Sync + 'static,
    ) -> ksni::MenuItem<Tray> {
        ksni::menu::StandardItem {
            label: label.into(),
            enabled,
            activate: Box::new(f),
            ..Default::default()
        }
        .into()
    }

    fn note(label: String, visible: bool) -> ksni::MenuItem<Tray> {
        ksni::menu::StandardItem {
            label,
            enabled: false,
            visible,
            ..Default::default()
        }
        .into()
    }

    impl ksni::Tray for Tray {
        fn id(&self) -> String {
            "cv1touch".into()
        }
        fn title(&self) -> String {
            format!("cv1touch: {}", self.report.summary)
        }
        fn activate(&mut self, _: i32, _: i32) {
            self.window.lock().unwrap().toggle();
        }
        fn icon_pixmap(&self) -> Vec<ksni::Icon> {
            // Every size, so that the panel picks the one that fits without scaling.
            crate::icon::SIZES
                .iter()
                .map(|&size| {
                    let mut data = crate::icon::rgba(self.report.health.tint(), size);
                    for pixel in data.as_chunks_mut::<4>().0 {
                        pixel.rotate_right(1); // StatusNotifierItem uses ARGB in network byte order.
                    }
                    ksni::Icon {
                        width: size as i32,
                        height: size as i32,
                        data,
                    }
                })
                .collect()
        }
        fn menu(&self) -> Vec<ksni::MenuItem<Self>> {
            let toggle = self.window.lock().unwrap().toggle_label();
            let sensors = self.sensors.lock().unwrap().toggle_label();
            let mut menu = vec![
                item(toggle, true, |t| t.window.lock().unwrap().toggle()),
                item(sensors, true, |t| t.sensors.lock().unwrap().toggle()),
                note(self.report.summary.clone(), true),
                note(
                    self.action_message.clone().unwrap_or_default(),
                    self.action_message.is_some(),
                ),
                ksni::MenuItem::Separator,
            ];
            for action in [
                ServiceAction::Start,
                ServiceAction::Restart,
                ServiceAction::Stop,
            ] {
                menu.push(item(action.label(), self.report.allows(action), move |t| {
                    t.control(action)
                }));
            }
            menu.extend([
                ksni::MenuItem::Separator,
                item("Set up…", true, |t| {
                    if let Err(error) = show_setup(Default::default()) {
                        t.action_message = Some(format!("Could not open setup: {error}"));
                    }
                }),
                item("Save diagnostic report…", true, |t| {
                    t.action_message = Some("Saving diagnostic report…".into());
                    let updates = t.action_updates.clone();
                    save_report(move |line| {
                        let _ = updates.send(Some(line));
                    });
                }),
                ksni::MenuItem::Separator,
                item("Quit (cv1touchd keeps running)", true, |t| {
                    t.quit.store(true, Ordering::Relaxed)
                }),
            ]);
            menu
        }
    }

    impl Tray {
        fn control(&mut self, action: ServiceAction) {
            self.action_message = Some(action.progress().into());
            let updates = self.action_updates.clone();
            if let Err(error) = crate::actions::control(action, move |result| {
                let _ = updates.send(
                    result
                        .err()
                        .map(|error| format!("{} failed: {error}", action.label())),
                );
            }) {
                self.action_message = Some(format!("{} failed: {error}", action.label()));
            }
        }
    }

    thread_local! {
        static WINDOW_SOCKET: std::cell::RefCell<Option<UnixDatagram>> = const { std::cell::RefCell::new(None) };
    }

    fn runtime() -> std::io::Result<std::path::PathBuf> {
        std::env::var_os("XDG_RUNTIME_DIR")
            .map(Into::into)
            .ok_or_else(|| {
                std::io::Error::new(std::io::ErrorKind::NotFound, "XDG_RUNTIME_DIR is not set")
            })
    }

    fn send(path: &std::path::Path, message: &[u8]) -> std::io::Result<()> {
        let socket = UnixDatagram::unbound()?;
        socket.set_nonblocking(true)?;
        match socket.send_to(message, path) {
            Ok(_) => Ok(()),
            // A full receive queue already contains open requests.
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(()),
            Err(e) => Err(e),
        }
    }

    fn name(kind: Kind) -> &'static str {
        match kind {
            Kind::Status => "cv1touch-window",
            Kind::Sensors => "cv1touch-sensors",
            Kind::Setup => "cv1touch-setup",
        }
    }

    pub(super) fn signal_window(kind: Kind, message: Message) -> bool {
        runtime()
            .and_then(|dir| {
                let message: &[u8] = match message {
                    Message::Open => b"open",
                    Message::Close => b"close",
                };
                send(&dir.join(format!("{}.sock", name(kind))), message)
            })
            .is_ok()
    }

    /// Whether a running instance holds `name`'s lock.
    fn held(dir: &std::path::Path, name: &str) -> bool {
        OpenOptions::new()
            .write(true)
            .open(dir.join(format!("{name}.lock")))
            .is_ok_and(|lock| unsafe { flock(lock.as_raw_fd(), 2 | 4) } != 0)
    }

    pub(super) fn quit() -> Result<(), Box<dyn Error>> {
        stop(&runtime()?)
    }

    /// Asks the tray and any open windows in `dir` to quit, and waits for them to exit.
    fn stop(dir: &std::path::Path) -> Result<(), Box<dyn Error>> {
        let _ = send(&dir.join("cv1touch-gui.sock"), b"quit");
        let names = [Kind::Status, Kind::Sensors, Kind::Setup].map(name);
        for name in names {
            let _ = send(&dir.join(format!("{name}.sock")), b"close");
        }
        for _ in 0..60 {
            if !names.iter().chain(&["cv1touch-gui"]).any(|n| held(dir, n)) {
                return Ok(());
            }
            thread::sleep(Duration::from_millis(50));
        }
        Err("cv1touch-gui is still running".into())
    }

    fn singleton(
        dir: &std::path::Path,
        name: &str,
        serve: impl FnOnce(UnixDatagram) -> Result<(), Box<dyn Error>>,
    ) -> Result<(), Box<dyn Error>> {
        let lock = OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(false)
            .open(dir.join(format!("{name}.lock")))?;
        let socket_path = dir.join(format!("{name}.sock"));
        for _ in 0..100 {
            if unsafe { flock(lock.as_raw_fd(), 2 | 4) } == 0 {
                // Only the lock owner may remove a stale socket or create a new one.
                match std::fs::remove_file(&socket_path) {
                    Ok(()) => {}
                    Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                    Err(e) => return Err(e.into()),
                }
                let result = serve(UnixDatagram::bind(&socket_path)?);
                let _ = std::fs::remove_file(socket_path);
                return result;
            }
            let error = std::io::Error::last_os_error();
            if error.kind() != std::io::ErrorKind::WouldBlock {
                return Err(error.into());
            }
            if send(&socket_path, b"open").is_ok() {
                return Ok(());
            }
            thread::sleep(Duration::from_millis(50));
        }
        Err("Existing instance did not respond".into())
    }

    pub fn run(open_window: bool) -> Result<(), Box<dyn Error>> {
        singleton(&runtime()?, "cv1touch-gui", |socket| {
            serve(socket, open_window)
        })
    }

    pub(super) fn run_window(kind: Kind) -> Result<(), Box<dyn Error>> {
        singleton(&runtime()?, name(kind), |socket| {
            WINDOW_SOCKET.with(|slot| slot.replace(Some(socket)));
            kind.run();
            Ok(())
        })
    }

    pub(super) fn listen_for_activation(ctx: &Context) {
        // Screenshot tests may construct the app without a singleton socket.
        if let Some(socket) = WINDOW_SOCKET.with(|slot| slot.take()) {
            let ctx = ctx.clone();
            thread::spawn(move || {
                let mut message = [0; 16];
                while let Ok(n) = socket.recv(&mut message) {
                    match &message[..n] {
                        b"open" => handle(&ctx, Message::Open),
                        b"close" => handle(&ctx, Message::Close),
                        _ => {}
                    }
                }
            });
        }
    }

    #[cfg(test)]
    #[test]
    fn singleton_forwards_open_and_can_be_reacquired_after_exit() {
        // Keep the Unix socket path short enough even with a long TMPDIR.
        let dir = std::env::temp_dir();
        let name = format!("tray-test-{}", std::process::id());
        let result = singleton(&dir, &name, |socket| {
            singleton(&dir, &name, |_| panic!("existing window must receive open"))?;
            socket.set_read_timeout(Some(Duration::from_secs(1)))?;
            let mut message = [0; 16];
            let n = socket.recv(&mut message)?;
            assert_eq!(&message[..n], b"open");
            Ok(())
        });
        let mut reacquired = false;
        let next = singleton(&dir, &name, |_| {
            reacquired = true;
            Ok(())
        });
        let _ = std::fs::remove_file(dir.join(format!("{name}.lock")));
        result.unwrap();
        next.unwrap();
        assert!(reacquired);
    }

    #[cfg(test)]
    #[test]
    fn closing_all_windows_signals_every_window_kind() {
        let dir = std::env::temp_dir().join(format!("tray-close-all-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let sockets: Vec<_> = [Kind::Status, Kind::Sensors, Kind::Setup]
            .into_iter()
            .map(|kind| {
                let path = dir.join(format!("{}.sock", name(kind)));
                let _ = std::fs::remove_file(&path);
                let socket = UnixDatagram::bind(&path).unwrap();
                socket
                    .set_read_timeout(Some(Duration::from_secs(1)))
                    .unwrap();
                (path, socket)
            })
            .collect();
        for (path, _) in &sockets {
            send(path, b"close").unwrap();
        }
        for (_, socket) in &sockets {
            let mut message = [0; 16];
            let n = socket.recv(&mut message).unwrap();
            assert_eq!(&message[..n], b"close");
        }
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[cfg(test)]
    #[test]
    fn close_reaches_the_window_socket() {
        let path = std::env::temp_dir().join(format!("tray-close-{}.sock", std::process::id()));
        let _ = std::fs::remove_file(&path);
        let socket = UnixDatagram::bind(&path).unwrap();
        send(&path, b"close").unwrap();
        send(&path, b"open").unwrap();
        let mut message = [0; 16];
        let n = socket.recv(&mut message).unwrap();
        assert_eq!(&message[..n], b"close");
        let n = socket.recv(&mut message).unwrap();
        assert_eq!(&message[..n], b"open");
        std::fs::remove_file(path).unwrap();
    }

    #[cfg(test)]
    #[test]
    fn quit_ends_the_tray_and_waits_for_it() {
        let dir = std::env::temp_dir().join(format!("tray-quit-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        stop(&dir).unwrap(); // none running
        let tray = thread::spawn({
            let dir = dir.clone();
            move || {
                singleton(&dir, "cv1touch-gui", |socket| {
                    let mut message = [0; 16];
                    let n = socket.recv(&mut message)?;
                    assert_eq!(&message[..n], b"quit");
                    Ok(())
                })
                .unwrap()
            }
        });
        while !dir.join("cv1touch-gui.sock").exists() {
            thread::sleep(Duration::from_millis(10));
        }
        let window = thread::spawn({
            let dir = dir.clone();
            move || {
                singleton(&dir, name(Kind::Status), |socket| {
                    let mut message = [0; 16];
                    let n = socket.recv(&mut message)?;
                    assert_eq!(&message[..n], b"close");
                    Ok(())
                })
                .unwrap()
            }
        });
        while !dir.join(format!("{}.sock", name(Kind::Status))).exists() {
            thread::sleep(Duration::from_millis(10));
        }
        stop(&dir).unwrap();
        tray.join().unwrap();
        window.join().unwrap();
        std::fs::remove_dir_all(dir).unwrap();
    }

    fn serve(socket: UnixDatagram, open_window: bool) -> Result<(), Box<dyn Error>> {
        socket.set_read_timeout(Some(Duration::from_millis(50)))?;
        let quit = Arc::new(AtomicBool::new(false));
        let window = Arc::new(Mutex::new(Window::new(Kind::Status)));
        let sensors = Arc::new(Mutex::new(Window::new(Kind::Sensors)));
        let (action_updates, action_results) = mpsc::channel();
        let handle = Tray {
            report: Report::default(),
            action_message: None,
            action_updates,
            window: window.clone(),
            sensors: sensors.clone(),
            quit: quit.clone(),
        }
        .spawn()?;
        if open_window {
            window.lock().unwrap().open();
        }
        let (updates, _stop) = poll_health();
        let mut previous = Report::default();
        let mut shown = (false, false, false);
        while !quit.load(Ordering::Relaxed) && !handle.is_closed() {
            if let Some(message) = action_results.try_iter().last() {
                handle.update(|t| t.action_message = message);
            }
            if let Some(report) = updates.try_iter().last().filter(|r| *r != previous) {
                previous = report.clone();
                handle.update(|t| t.report = report);
            }
            let mut message = [0; 16];
            match socket.recv(&mut message).map(|n| &message[..n]) {
                Ok(b"open") => window.lock().unwrap().open(),
                Ok(b"quit") => quit.store(true, Ordering::Relaxed),
                _ => {}
            }
            // Refresh the menu when a window opens or closes or an action starts or ends.
            let now = (
                window.lock().unwrap().is_open(),
                sensors.lock().unwrap().is_open(),
                crate::actions::busy(),
            );
            if now != shown {
                shown = now;
                handle.update(|_| {});
            }
        }
        if quit.load(Ordering::Relaxed) {
            close_windows();
        }
        handle.shutdown().wait();
        Ok(())
    }
}

#[cfg(target_os = "linux")]
pub fn run(open_window: bool) -> Result<(), Box<dyn std::error::Error>> {
    linux::run(open_window)
}
#[cfg(windows)]
#[path = "tray_windows.rs"]
mod windows;
#[cfg(windows)]
pub fn run(open_window: bool) -> Result<(), Box<dyn std::error::Error>> {
    windows::run(open_window)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn doctor_health_distinguishes_offline_and_problems() {
        let living = |ok, detail| json!({"daemon": {}, "checks": [{"ok": ok, "detail": detail}]});
        assert_eq!(Health::from_doctor(&living(true, "OK")), Health::Ready);
        for report in [
            living(false, "failed"),
            living(true, "todo: set up"),
            living(true, "warning: USB"),
            json!({}),
            json!({"daemon":{}, "checks": []}),
        ] {
            assert_eq!(Health::from_doctor(&report), Health::Warning);
        }
        assert_eq!(
            Health::from_doctor(&json!({"daemon": null, "checks": [{"ok": false}]})),
            Health::Stopped
        );
    }

    #[test]
    fn status_and_doctor_failures_never_leave_stale_green() {
        let mut state = HealthState::default();
        let now = Instant::now();
        state.doctor = Some((Health::Ready, now));
        state.status(&Ok(json!({"engine_ready": true})), now);
        assert_eq!(state.health(), Health::Ready);
        state.status(&Ok(json!({"engine_ready": false})), now);
        assert_eq!(state.health(), Health::Warning);
        assert!(state.status(&Err("failed".into()), now));
        assert_eq!(state.health(), Health::Unknown);
        state.doctor = Some((Health::Stopped, now));
        assert_eq!(state.health(), Health::Stopped);
        state.status(&Ok(json!({"engine_ready": true})), now);
        assert_eq!(state.health(), Health::Unknown);
        state.doctor = Some((Health::Unknown, now));
        assert_eq!(state.health(), Health::Unknown);
        state.doctor = Some((Health::Warning, now));
        assert_eq!(state.health(), Health::Warning);
    }

    #[test]
    fn live_warnings_override_healthy_doctor() {
        let now = Instant::now();
        let mut state = HealthState {
            doctor: Some((Health::Ready, now)),
            ..Default::default()
        };
        for value in [
            json!({"engine_ready": true, "sensors": [{"pose":{"state":"moved"}}]}),
            json!({"engine_ready": true, "sensors": [{"pose":{"state":"given_up"}}]}),
            json!({"engine_ready": true, "alignment": {"headset":"other"}}),
            json!({"engine_ready": true, "alignment": {"headset":"universe_changed"}}),
            json!({"engine_ready": true, "alignment": {"unmeasured_base":"base"}}),
        ] {
            state.status(&Ok(value), now);
            assert_eq!(state.health(), Health::Warning);
        }
        state.status(
            &Ok(json!({"engine_ready":true, "hands":[{"flags":0}]})),
            now,
        );
        assert_eq!(state.health(), Health::Ready);
    }

    #[test]
    fn report_gates_service_actions_by_daemon_state() {
        let _busy_guard = crate::actions::busy_test_lock();
        let now = Instant::now();
        let mut state = HealthState::default();
        let offered = |report: &Report| {
            [
                ServiceAction::Start,
                ServiceAction::Restart,
                ServiceAction::Stop,
            ]
            .into_iter()
            .filter(|a| report.allows(*a))
            .collect::<Vec<_>>()
        };
        assert_eq!(state.report().daemon, Daemon::Checking);
        assert!(offered(&state.report()).is_empty());
        let live = json!({"engine_ready": true, "sensors": [{}, {}],
            "hands": [{"flags": 7, "battery_percent": 85}, {"flags": 17, "battery_percent": 90}]});
        state.status(&Ok(live), now);
        let report = state.report();
        assert_eq!(report.daemon, Daemon::Running);
        assert_eq!(
            offered(&report),
            [ServiceAction::Restart, ServiceAction::Stop]
        );
        assert_eq!(
            report.summary,
            "Status unknown · 2 sensors · L 85% R asleep"
        );
        state.status(
            &Err(crate::client::StatusError {
                down: true,
                message: "cv1touchd isn't running".into(),
            }),
            now,
        );
        let report = state.report();
        assert_eq!(
            (report.health, report.daemon),
            (Health::Stopped, Daemon::Stopped)
        );
        assert_eq!(offered(&report), [ServiceAction::Start]);
        assert_eq!(report.summary, "Daemon stopped");
        assert_eq!(report.health.tint(), Tint::Stopped);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn window_toggle_follows_the_window_process() {
        for (kind, [show, hide]) in [
            (Kind::Status, ["Show status window", "Hide status window"]),
            (Kind::Sensors, ["Show sensor views", "Hide sensor views"]),
        ] {
            let mut window = Window {
                kind,
                child: Some(Command::new("sleep").arg("5").spawn().unwrap()),
            };
            assert_eq!(window.toggle_label(), hide);
            let mut child = window.child.take().unwrap();
            child.kill().unwrap();
            window.child = Some(child);
            // Killed but not yet reaped: the label must not claim it is open.
            for _ in 0..100 {
                if window.toggle_label() == show {
                    break;
                }
                thread::sleep(std::time::Duration::from_millis(10));
            }
            assert_eq!(window.toggle_label(), show);
            assert!(window.child.is_none());
        }
    }
}
