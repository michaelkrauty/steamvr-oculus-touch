use super::{
    handle, poll_health, save_report, show_setup, Context, Kind, Message, Report, ServiceAction,
    Window,
};
use std::{
    cell::RefCell,
    error::Error,
    ptr,
    sync::{
        atomic::{AtomicU32, Ordering},
        mpsc::{self, Receiver, Sender},
    },
    thread,
    time::Duration,
};
use windows_sys::Win32::{
    Foundation::*,
    System::{
        LibraryLoader::GetModuleHandleW,
        Threading::{
            CreateEventW, CreateMutexW, OpenEventW, SetEvent, WaitForMultipleObjects,
            EVENT_MODIFY_STATE, INFINITE,
        },
    },
    UI::{Shell::*, WindowsAndMessaging::*},
};

static TASKBAR_CREATED: AtomicU32 = AtomicU32::new(0);

const TRAY: u32 = WM_APP + 1;
const OPEN: u32 = WM_APP + 2;
const POLL_TIMER: usize = 1;
const CLASS: &[u16] = &[
    99, 118, 49, 116, 111, 117, 99, 104, 45, 116, 114, 97, 121, 0,
];
struct Tray {
    window: Window,
    sensors: Window,
    report: Report,
    updates: Receiver<Report>,
    action_send: Sender<Option<String>>,
    action_results: Receiver<Option<String>>,
    action_error: Option<String>,
}

thread_local! {
    static STATE: RefCell<Option<Tray>> = const { RefCell::new(None) };
}

fn with_tray<R>(f: impl FnOnce(&mut Tray) -> R) -> Option<R> {
    STATE.with(|state| state.borrow_mut().as_mut().map(f))
}

// Each window (not the tray) owns its auto-reset events, which retain a
// request sent before eframe installs the listener.
fn events(kind: Kind) -> [&'static str; 2] {
    match kind {
        Kind::Status => [
            "Local\\cv1touch-gui-window",
            "Local\\cv1touch-gui-window-close",
        ],
        Kind::Sensors => [
            "Local\\cv1touch-gui-sensors",
            "Local\\cv1touch-gui-sensors-close",
        ],
        Kind::Setup => [
            "Local\\cv1touch-gui-setup",
            "Local\\cv1touch-gui-setup-close",
        ],
    }
}
const MESSAGES: [Message; 2] = [Message::Open, Message::Close];
thread_local! {
    static WINDOW_ACTIVATION: std::cell::Cell<[usize; 2]> = const { std::cell::Cell::new([0; 2]) };
}

pub(super) fn signal_window(kind: Kind, message: Message) -> bool {
    let name = events(kind)[MESSAGES.iter().position(|m| *m == message).unwrap()];
    unsafe {
        let event = OpenEventW(EVENT_MODIFY_STATE, 0, wide(name).as_ptr());
        if event.is_null() {
            return false;
        }
        let sent = SetEvent(event) != 0;
        CloseHandle(event);
        sent
    }
}

pub(super) fn run_window(kind: Kind) -> Result<(), Box<dyn Error>> {
    unsafe {
        let mut events = [ptr::null_mut(); 2];
        for (slot, name) in events.iter_mut().zip(self::events(kind)) {
            *slot = CreateEventW(ptr::null(), 0, 0, wide(name).as_ptr());
            if slot.is_null() {
                let error = std::io::Error::last_os_error();
                events.iter().filter(|e| !e.is_null()).for_each(|e| {
                    CloseHandle(*e);
                });
                return Err(error.into());
            }
            if GetLastError() == ERROR_ALREADY_EXISTS {
                // Another window runs: raise it instead.
                let sent = SetEvent(events[0]);
                let error = std::io::Error::last_os_error();
                events.iter().filter(|e| !e.is_null()).for_each(|e| {
                    CloseHandle(*e);
                });
                return if sent == 0 { Err(error.into()) } else { Ok(()) };
            }
        }
        WINDOW_ACTIVATION.with(|slot| slot.set(events.map(|e| e as usize)));
        kind.run();
        let left = WINDOW_ACTIVATION.with(|slot| slot.replace([0; 2]));
        if left[0] != 0 {
            left.iter().for_each(|e| {
                CloseHandle(*e as HANDLE);
            });
        }
        Ok(())
    }
}

pub(super) fn listen_for_activation(ctx: &Context) {
    let events = WINDOW_ACTIVATION.with(|slot| slot.replace([0; 2]));
    if events[0] != 0 {
        let ctx = ctx.clone();
        thread::spawn(move || unsafe {
            let handles = events.map(|e| e as HANDLE);
            loop {
                let index = WaitForMultipleObjects(2, handles.as_ptr(), 0, INFINITE)
                    .wrapping_sub(WAIT_OBJECT_0) as usize;
                match MESSAGES.get(index) {
                    Some(message) => handle(&ctx, *message),
                    None => break,
                }
            }
            handles.iter().for_each(|e| {
                CloseHandle(*e);
            });
        });
    }
}

fn wide(s: &str) -> Vec<u16> {
    s.encode_utf16().chain(Some(0)).collect()
}

/// The icon rendered at the shell's small-icon size, so that no scaling blurs it.
unsafe fn icon(report: &Report) -> HICON {
    let size = GetSystemMetrics(SM_CXSMICON).clamp(16, 64) as usize;
    let mut bgra = crate::icon::rgba(report.health.tint(), size);
    let mask_stride = size.div_ceil(16) * 2;
    let mut and = vec![0xffu8; size * mask_stride];
    for (i, pixel) in bgra.as_chunks_mut::<4>().0.iter_mut().enumerate() {
        pixel.swap(0, 2);
        for channel in 0..3 {
            pixel[channel] = ((pixel[channel] as u16 * pixel[3] as u16 + 127) / 255) as u8;
        }
        if pixel[3] != 0 {
            let (x, y) = (i % size, i / size);
            and[y * mask_stride + x / 8] &= !(0x80 >> (x % 8));
        }
    }
    CreateIcon(
        ptr::null_mut(),
        size as i32,
        size as i32,
        1,
        32,
        and.as_ptr(),
        bgra.as_ptr(),
    )
}

unsafe fn notify(hwnd: HWND, operation: u32, report: &Report) -> bool {
    let hicon = if operation == NIM_DELETE {
        ptr::null_mut()
    } else {
        icon(report)
    };
    if operation != NIM_DELETE && hicon.is_null() {
        return false;
    }
    let mut data = NOTIFYICONDATAW {
        cbSize: std::mem::size_of::<NOTIFYICONDATAW>() as u32,
        hWnd: hwnd,
        uID: 1,
        uFlags: NIF_MESSAGE | NIF_ICON | NIF_TIP,
        uCallbackMessage: TRAY,
        hIcon: hicon,
        ..Default::default()
    };
    let mut tip = wide(&format!("cv1touch: {}", report.summary));
    tip.truncate(data.szTip.len() - 1);
    tip.push(0);
    data.szTip[..tip.len()].copy_from_slice(&tip);
    let result = Shell_NotifyIconW(operation, &data) != 0;
    if !hicon.is_null() {
        DestroyIcon(hicon);
    }
    result
}

fn open_window() {
    with_tray(|tray| tray.window.open());
}

fn toggle_window(kind: Kind) {
    with_tray(|tray| match kind {
        Kind::Status => tray.window.toggle(),
        Kind::Sensors => tray.sensors.toggle(),
        Kind::Setup => {}
    });
}

fn control(action: ServiceAction) {
    with_tray(|tray| {
        let send = tray.action_send.clone();
        if let Err(error) = crate::actions::control(action, move |result| {
            let _ = send.send(
                result
                    .err()
                    .map(|error| format!("{} failed: {error}", action.label())),
            );
        }) {
            tray.action_error = Some(error.to_string());
        }
    });
}

unsafe extern "system" fn window(hwnd: HWND, msg: u32, w: WPARAM, l: LPARAM) -> LRESULT {
    match msg {
        message if message != 0 && message == TASKBAR_CREATED.load(Ordering::Relaxed) => {
            let report = with_tray(|tray| tray.report.clone()).unwrap_or_default();
            notify(hwnd, NIM_ADD, &report);
            0
        }
        OPEN => {
            open_window();
            0
        }
        WM_TIMER if w == POLL_TIMER => {
            if let Some(Some(report)) = with_tray(|tray| {
                tray.window.reap();
                tray.sensors.reap();
                if let Some(result) = tray.action_results.try_iter().last() {
                    tray.action_error = result;
                }
                let report = tray
                    .updates
                    .try_iter()
                    .last()
                    .filter(|r| *r != tray.report)?;
                tray.report = report.clone();
                Some(report)
            }) {
                notify(hwnd, NIM_MODIFY, &report);
            }
            0
        }
        TRAY if l as u32 == WM_LBUTTONUP => {
            toggle_window(Kind::Status);
            0
        }
        TRAY if l as u32 == WM_RBUTTONUP => {
            let menu = CreatePopupMenu();
            let (toggle, sensors, summary, report) = with_tray(|tray| {
                let summary = match &tray.action_error {
                    Some(error) => error.clone(),
                    None if crate::actions::busy() => "Working…".to_owned(),
                    None => tray.report.summary.clone(),
                };
                (
                    tray.window.toggle_label(),
                    tray.sensors.toggle_label(),
                    summary,
                    tray.report.clone(),
                )
            })
            .unwrap_or((
                Kind::Status.labels()[0],
                Kind::Sensors.labels()[0],
                String::new(),
                Report::default(),
            ));
            let service = |action| {
                if report.allows(action) {
                    MF_STRING
                } else {
                    MF_STRING | MF_GRAYED
                }
            };
            // Each entry's command is its position, from 1.
            let entries: [(u32, Option<&str>); 12] = [
                (MF_STRING, Some(toggle)),
                (MF_STRING, Some(sensors)),
                (MF_STRING | MF_GRAYED, Some(summary.as_str())),
                (MF_SEPARATOR, None),
                (
                    service(ServiceAction::Start),
                    Some(ServiceAction::Start.label()),
                ),
                (
                    service(ServiceAction::Restart),
                    Some(ServiceAction::Restart.label()),
                ),
                (
                    service(ServiceAction::Stop),
                    Some(ServiceAction::Stop.label()),
                ),
                (MF_SEPARATOR, None),
                (MF_STRING, Some("Set up…")),
                (MF_STRING, Some("Save diagnostic report…")),
                (MF_SEPARATOR, None),
                (MF_STRING, Some("Quit tray icon (cv1touchd keeps running)")),
            ];
            for (i, (flags, label)) in entries.iter().enumerate() {
                let text = label.map(wide);
                AppendMenuW(
                    menu,
                    *flags,
                    i + 1,
                    text.as_ref().map_or(ptr::null(), |t| t.as_ptr()),
                );
            }
            let mut point = POINT::default();
            GetCursorPos(&mut point);
            SetForegroundWindow(hwnd);
            TrackPopupMenu(
                menu,
                TPM_RIGHTBUTTON,
                point.x,
                point.y,
                0,
                hwnd,
                ptr::null(),
            );
            PostMessageW(hwnd, WM_NULL, 0, 0);
            DestroyMenu(menu);
            0
        }
        WM_COMMAND => {
            match w & 0xffff {
                1 => toggle_window(Kind::Status),
                2 => toggle_window(Kind::Sensors),
                5 => control(ServiceAction::Start),
                6 => control(ServiceAction::Restart),
                7 => control(ServiceAction::Stop),
                9 => {
                    if let Err(error) = show_setup(Default::default()) {
                        with_tray(|tray| {
                            tray.action_error = Some(format!("Could not open setup: {error}"))
                        });
                    }
                }
                10 => {
                    with_tray(|tray| {
                        tray.action_error = Some("Saving diagnostic report…".into());
                        let send = tray.action_send.clone();
                        save_report(move |line| {
                            let _ = send.send(Some(line));
                        });
                    });
                }
                12 => {
                    DestroyWindow(hwnd);
                }
                _ => {}
            }
            0
        }
        WM_DESTROY => {
            KillTimer(hwnd, POLL_TIMER);
            notify(hwnd, NIM_DELETE, &Report::default());
            PostQuitMessage(0);
            0
        }
        _ => DefWindowProcW(hwnd, msg, w, l),
    }
}
pub fn run() -> Result<(), Box<dyn Error>> {
    unsafe {
        let mutex = CreateMutexW(ptr::null(), 1, wide("Local\\cv1touch-gui-tray").as_ptr());
        if mutex.is_null() {
            return Err(std::io::Error::last_os_error().into());
        }
        if GetLastError() == ERROR_ALREADY_EXISTS {
            for _ in 0..100 {
                let hwnd = FindWindowW(CLASS.as_ptr(), ptr::null());
                if !hwnd.is_null() && PostMessageW(hwnd, OPEN, 0, 0) != 0 {
                    CloseHandle(mutex);
                    return Ok(());
                }
                thread::sleep(Duration::from_millis(50));
            }
            CloseHandle(mutex);
            return Err("Existing tray did not respond".into());
        }
        let taskbar_created = RegisterWindowMessageW(wide("TaskbarCreated").as_ptr());
        if taskbar_created == 0 {
            CloseHandle(mutex);
            return Err(std::io::Error::last_os_error().into());
        }
        TASKBAR_CREATED.store(taskbar_created, Ordering::Relaxed);
        let instance = GetModuleHandleW(ptr::null());
        let class = WNDCLASSW {
            lpfnWndProc: Some(window),
            hInstance: instance,
            lpszClassName: CLASS.as_ptr(),
            ..Default::default()
        };
        if RegisterClassW(&class) == 0 {
            CloseHandle(mutex);
            return Err(std::io::Error::last_os_error().into());
        }
        let hwnd = CreateWindowExW(
            0,
            CLASS.as_ptr(),
            CLASS.as_ptr(),
            0,
            0,
            0,
            0,
            0,
            ptr::null_mut(),
            ptr::null_mut(),
            instance,
            ptr::null(),
        );
        if hwnd.is_null() {
            CloseHandle(mutex);
            return Err(std::io::Error::last_os_error().into());
        }
        if !notify(hwnd, NIM_ADD, &Report::default()) {
            DestroyWindow(hwnd);
            CloseHandle(mutex);
            return Err(std::io::Error::last_os_error().into());
        }
        if SetTimer(hwnd, POLL_TIMER, 100, None) == 0 {
            let error = std::io::Error::last_os_error();
            DestroyWindow(hwnd);
            CloseHandle(mutex);
            return Err(error.into());
        }
        let (updates, _stop) = poll_health();
        let (action_send, action_results) = mpsc::channel();
        STATE.with(|state| {
            state.replace(Some(Tray {
                report: Report::default(),
                updates,
                action_send,
                action_results,
                action_error: None,
                window: Window::new(Kind::Status),
                sensors: Window::new(Kind::Sensors),
            }));
        });
        let mut msg = MSG::default();
        let result = loop {
            match GetMessageW(&mut msg, ptr::null_mut(), 0, 0) {
                -1 => {
                    let error = std::io::Error::last_os_error();
                    DestroyWindow(hwnd);
                    break Err(error.into());
                }
                0 => break Ok(()),
                _ => {
                    TranslateMessage(&msg);
                    DispatchMessageW(&msg);
                }
            }
        };
        STATE.with(|state| state.replace(None));
        CloseHandle(mutex);
        result
    }
}
