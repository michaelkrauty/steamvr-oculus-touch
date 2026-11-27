// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The Settings tab: the controller angle, desktop integration and quick maintenance
//! shortcuts. Every change goes through the daemon or the CLI, which save it.

use super::*;

/// What the window knows about the settings besides the daemon's status.
#[derive(Default)]
pub struct Local {
    /// Whether the tray starts at login; None where that is unknown.
    pub autostart: Option<bool>,
    /// Linux: whether the desktop entry and icons are installed.
    pub desktop: Option<bool>,
    /// A value just asked for, shown until the daemon's status confirms it.
    pub tilt: Option<(f64, Instant)>,
    /// The last change's error.
    pub message: Option<(Level, String)>,
    /// `finish-setup`: Some(None) while it runs, then its result.
    pub finish: Option<Option<Finish>>,
}

/// How long a value just set may wait for the status to show it.
const CONFIRM: Duration = Duration::from_secs(5);

impl Local {
    pub fn load() -> Self {
        Self {
            autostart: autostart::enabled(),
            desktop: desktop_installed(),
            ..Default::default()
        }
    }

    /// Drops the values just set once a status shows them, or once it is clear it will not.
    pub fn confirm(&mut self, status: Option<&Value>) {
        let Some(status) = status else { return };
        if self.tilt.is_some_and(|(deg, at)| {
            status["tilt_deg"]
                .as_f64()
                .is_some_and(|now| (now - deg).abs() < 0.05)
                || at.elapsed() > CONFIRM
        }) {
            self.tilt = None;
        }
    }
}

/// The angle shown: the one just set, the daemon's, or the saved one.
fn tilt(view: &View) -> f64 {
    view.local
        .tilt
        .map(|t| t.0)
        .or_else(|| view.snap.status.as_ref()?["tilt_deg"].as_f64())
        .unwrap_or_else(|| view.config.tilt_deg())
}

const TILT_TIP: &str = "How far the controllers are shown pitched from how they are tracked, with the front end up for positive angles. Some games hold a tool or a gun at an angle that feels wrong; tilting the controllers here changes how it sits in your hand. 0° shows them exactly as tracked.";
const AUTOSTART_TIP: &str = "Start cv1touch's tray icon when you log in, so that its status and menu are always one click away. The tracking itself does not need it.";

pub(super) fn tab(ui: &mut Ui, view: &View, action: &mut Option<Action>) {
    let local = view.local;
    if let Some((level, text)) = &local.message {
        ui.horizontal_wrapped(|ui| {
            dot(ui, *level);
            ui.label(RichText::new(text).color(color(ui, *level)));
        });
    }
    let running = view.snap.status.is_some();
    card(ui, |ui| {
        title(ui, "Controller angle");
        ui.horizontal(|ui| {
            let shown = tilt(view);
            let id = ui.make_persistent_id("tilt-drag");
            let mut value = ui.data(|d| d.get_temp::<f64>(id)).unwrap_or(shown);
            let slider = ui
                .add_enabled(
                    running,
                    egui::Slider::new(&mut value, -45.0..=45.0)
                        .step_by(0.5)
                        .fixed_decimals(1)
                        .suffix("°")
                        .text("Tilt"),
                )
                .on_hover_text(TILT_TIP)
                .on_disabled_hover_text(TILT_TIP);
            if slider.dragged() {
                ui.data_mut(|d| d.insert_temp(id, value));
            } else if slider.drag_stopped() || slider.changed() {
                ui.data_mut(|d| d.remove::<f64>(id));
                *action = Some(Action::Tilt(value));
            }
            if ui
                .add_enabled(running && shown != 0.0, egui::Button::new("Reset to 0°"))
                .on_hover_text("Show the controllers exactly as they are tracked.")
                .clicked()
            {
                *action = Some(Action::Tilt(0.0));
            }
        });
        if !running {
            ui.label(weak("Start the daemon to change the angle."));
        }
    });
    card(ui, |ui| {
        title(ui, "Desktop");
        match local.autostart {
            Some(mut on) => {
                if ui
                    .checkbox(&mut on, "Start tray icon when I log in")
                    .on_hover_text(AUTOSTART_TIP)
                    .changed()
                {
                    *action = Some(Action::Autostart(on));
                }
            }
            None => {
                ui.add_enabled(
                    false,
                    egui::Checkbox::new(&mut false, "Start tray icon when I log in"),
                )
                .on_disabled_hover_text("Not available on this system.");
            }
        }
        if let Some(installed) = local.desktop {
            ui.horizontal(|ui| {
                ui.label("Desktop shortcut and window icon");
                ui.label(weak(if installed { "installed" } else { "not installed" }));
                let (label, tip) = if installed {
                    ("Remove", "Remove cv1touch's menu entry and icons for your user.")
                } else {
                    ("Install", "Add a menu entry and icons for your user. Wayland desktops need them to show the window's icon.")
                };
                if ui.button(label).on_hover_text(tip).clicked() {
                    *action = Some(Action::Desktop(!installed));
                }
            });
        }
    });
    card(ui, |ui| {
        title(ui, "Maintenance");
        ui.horizontal_wrapped(|ui| {
            let mut shortcuts = vec![
                (
                    "Align controllers to headset (15 s)…",
                    Action::Setup(Request::QUICK_HEADSET),
                    "Wear the headset and hold both controllers to it for about 15 seconds to align the controllers with SteamVR's headset tracking.",
                ),
                (
                    "Calibrate buttons & sticks…",
                    Action::Setup(Request::stage(Stage::Buttons)),
                    "Measure your triggers, grips and sticks at rest and at their limits, so that games read their full range.",
                ),
            ];
            if cfg!(windows) {
                shortcuts.push((
                    "Finish driver & SteamVR setup",
                    Action::FinishSetup,
                    "Give the sensors their driver and add cv1touch to SteamVR, as the installer does. Close SteamVR first.",
                ));
            }
            for (label, value, tip) in shortcuts {
                let enabled = !(value == Action::FinishSetup && local.finish == Some(None));
                if ui
                    .add_enabled(enabled, egui::Button::new(label))
                    .on_hover_text(tip)
                    .clicked()
                {
                    *action = Some(value);
                }
            }
        });
        match &local.finish {
            Some(None) => {
                ui.horizontal(|ui| {
                    ui.add(egui::Spinner::new());
                    ui.label("Finishing driver and SteamVR setup…");
                });
            }
            Some(Some(finish)) => finish_checklist(ui, finish),
            None => {}
        }
    });
}

fn finish_checklist(ui: &mut Ui, finish: &Finish) {
    for (step, exit) in &finish.steps {
        // "Stopping cv1touch" exits 1 when nothing was running.
        let level = match (step.as_str(), exit) {
            (_, 0) | ("Stopping cv1touch", 1) => Level::Good,
            (_, 3) => Level::Warning,
            _ => Level::Bad,
        };
        ui.horizontal(|ui| {
            dot(ui, level);
            ui.label(step);
        });
    }
    let (level, text) = finish_notice(finish);
    ui.label(RichText::new(text).color(color(ui, level)));
    for line in finish.remaining.iter().skip(1) {
        ui.label(line);
    }
}

/// Linux: whether the desktop entry is installed; None elsewhere.
pub fn desktop_installed() -> Option<bool> {
    #[cfg(target_os = "linux")]
    return Some(crate::desktop::installed());
    #[cfg(not(target_os = "linux"))]
    None
}

pub fn set_desktop(install: bool) -> Result<(), String> {
    #[cfg(target_os = "linux")]
    return crate::desktop::set(install).map_err(|e| e.to_string());
    #[cfg(not(target_os = "linux"))]
    {
        let _ = install;
        Err("Not available on this system.".into())
    }
}

/// Starting the tray icon at login.
pub mod autostart {
    use std::io;

    /// Linux: an XDG autostart entry.
    #[cfg(target_os = "linux")]
    mod platform {
        use std::{
            ffi::OsString,
            io,
            path::{Path, PathBuf},
        };

        /// `$XDG_CONFIG_HOME/autostart/cv1touch-gui.desktop`, `$XDG_CONFIG_HOME` defaulting to
        /// `~/.config` when it is unset or not absolute.
        pub fn path(xdg: Option<OsString>, home: Option<OsString>) -> Option<PathBuf> {
            xdg.map(PathBuf::from)
                .filter(|p| p.is_absolute())
                .or_else(|| home.map(|h| PathBuf::from(h).join(".config")))
                .map(|config| config.join("autostart/cv1touch-gui.desktop"))
        }

        fn current() -> io::Result<PathBuf> {
            path(
                std::env::var_os("XDG_CONFIG_HOME"),
                std::env::var_os("HOME"),
            )
            .ok_or_else(|| io::Error::other("neither XDG_CONFIG_HOME nor HOME is set"))
        }

        fn entry(exe: &Path) -> String {
            format!(
                "[Desktop Entry]\nType=Application\nName=cv1touch\nComment=cv1touch tray icon\nExec={}\nIcon={}\nTerminal=false\nX-GNOME-Autostart-enabled=true\n",
                crate::desktop::exec_arg(exe),
                crate::window::APP_ID
            )
        }

        pub fn enabled_at(path: &Path) -> bool {
            path.is_file()
        }

        pub fn set_at(path: &Path, exe: &Path, on: bool) -> io::Result<()> {
            if on {
                std::fs::create_dir_all(path.parent().unwrap())?;
                std::fs::write(path, entry(exe))
            } else {
                match std::fs::remove_file(path) {
                    Err(e) if e.kind() != io::ErrorKind::NotFound => Err(e),
                    _ => Ok(()),
                }
            }
        }

        pub fn enabled() -> Option<bool> {
            current().ok().map(|p| enabled_at(&p))
        }

        pub fn set(on: bool) -> io::Result<()> {
            set_at(&current()?, &std::env::current_exe()?, on)
        }
    }

    /// Windows: a value under the user's `Run` key.
    #[cfg(windows)]
    mod platform {
        use std::{io, ptr};
        use windows_sys::Win32::{
            Foundation::{ERROR_FILE_NOT_FOUND, ERROR_SUCCESS},
            System::Registry::{
                RegCloseKey, RegDeleteValueW, RegOpenKeyExW, RegQueryValueExW, RegSetValueExW,
                HKEY, HKEY_CURRENT_USER, KEY_QUERY_VALUE, KEY_SET_VALUE, REG_SZ,
            },
        };

        fn wide(s: &str) -> Vec<u16> {
            s.encode_utf16().chain(Some(0)).collect()
        }

        /// The `Run` key, opened for `access`; closed when dropped.
        struct Key(HKEY);

        impl Key {
            fn open(access: u32) -> io::Result<Self> {
                let mut key = ptr::null_mut();
                let path = wide(r"Software\Microsoft\Windows\CurrentVersion\Run");
                // SAFETY: a NUL-terminated path and a place for the handle.
                let status =
                    unsafe { RegOpenKeyExW(HKEY_CURRENT_USER, path.as_ptr(), 0, access, &mut key) };
                if status != ERROR_SUCCESS {
                    return Err(io::Error::from_raw_os_error(status as i32));
                }
                Ok(Self(key))
            }
        }

        impl Drop for Key {
            fn drop(&mut self) {
                // SAFETY: the key is open and owned.
                unsafe { RegCloseKey(self.0) };
            }
        }

        const NAME: &str = "cv1touch-gui";

        pub fn enabled() -> Option<bool> {
            let key = Key::open(KEY_QUERY_VALUE).ok()?;
            let name = wide(NAME);
            // SAFETY: an open key and a NUL-terminated name; only the size is asked for.
            let status = unsafe {
                RegQueryValueExW(
                    key.0,
                    name.as_ptr(),
                    ptr::null(),
                    ptr::null_mut(),
                    ptr::null_mut(),
                    ptr::null_mut(),
                )
            };
            Some(status == ERROR_SUCCESS)
        }

        pub fn set(on: bool) -> io::Result<()> {
            let key = Key::open(KEY_SET_VALUE)?;
            let name = wide(NAME);
            let status = if on {
                let exe = std::env::current_exe()?;
                let value = wide(&format!("\"{}\"", exe.display()));
                // SAFETY: an open key, a NUL-terminated name and a REG_SZ value with its
                // terminator, sized in bytes.
                unsafe {
                    RegSetValueExW(
                        key.0,
                        name.as_ptr(),
                        0,
                        REG_SZ,
                        value.as_ptr().cast(),
                        (value.len() * 2) as u32,
                    )
                }
            } else {
                // SAFETY: an open key and a NUL-terminated name.
                match unsafe { RegDeleteValueW(key.0, name.as_ptr()) } {
                    ERROR_FILE_NOT_FOUND => ERROR_SUCCESS,
                    status => status,
                }
            };
            if status != ERROR_SUCCESS {
                return Err(io::Error::from_raw_os_error(status as i32));
            }
            Ok(())
        }
    }

    #[cfg(not(any(target_os = "linux", windows)))]
    mod platform {
        pub fn enabled() -> Option<bool> {
            None
        }

        pub fn set(_: bool) -> std::io::Result<()> {
            Err(std::io::Error::other("not available on this system"))
        }
    }

    #[cfg(all(target_os = "linux", test))]
    pub use platform::{enabled_at, path, set_at};

    pub fn enabled() -> Option<bool> {
        platform::enabled()
    }

    pub fn set(on: bool) -> io::Result<()> {
        platform::set(on)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn a_value_just_set_shows_until_the_status_has_it() {
        let mut local = Local {
            tilt: Some((5.0, Instant::now())),
            ..Default::default()
        };
        local.confirm(Some(&json!({"tilt_deg": 0.0})));
        assert!(local.tilt.is_some());
        local.confirm(Some(&json!({"tilt_deg": 5.0})));
        assert!(local.tilt.is_none());
        local.tilt = Some((5.0, Instant::now() - CONFIRM - Duration::from_secs(1)));
        local.confirm(Some(&json!({"tilt_deg": 0.0})));
        assert!(
            local.tilt.is_none(),
            "a stale request gives way to the status"
        );
    }

    #[test]
    fn settings_show_the_daemon_then_the_saved_file() {
        let icons = Icons::load(&egui::Context::default());
        let mut snap = Snapshot::default();
        let mut config = ConfigStats::default();
        let mut local = Local::default();
        let shown = |snap: &Snapshot, config: &ConfigStats, local: &Local| {
            let view = View {
                snap,
                config,
                daemon: Daemon::Stopped,
                notice: None,
                icons: &icons,
                report: None,
                local,
            };
            tilt(&view)
        };
        // No daemon and no file: the daemon's defaults.
        assert_eq!(shown(&snap, &config, &local), 0.0);
        config.settings = Some(json!({"format": "cv1touch-settings", "tilt_deg": -7.5}));
        assert_eq!(shown(&snap, &config, &local), -7.5);
        snap.status = Some(json!({"tilt_deg": 3.0}));
        assert_eq!(shown(&snap, &config, &local), 3.0);
        local.tilt = Some((10.5, Instant::now()));
        assert_eq!(shown(&snap, &config, &local), 10.5);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn autostart_entry_is_written_read_and_removed() {
        use std::path::Path;
        assert_eq!(
            autostart::path(Some("/x/config".into()), Some("/home/u".into())).unwrap(),
            Path::new("/x/config/autostart/cv1touch-gui.desktop")
        );
        assert_eq!(
            autostart::path(Some("relative".into()), Some("/home/u".into())).unwrap(),
            Path::new("/home/u/.config/autostart/cv1touch-gui.desktop")
        );
        let dir = std::env::temp_dir().join(format!("cv1touch-autostart-{}", std::process::id()));
        let path = autostart::path(Some(dir.clone().into()), None).unwrap();
        let exe = Path::new("/opt/cv1 touch/cv1touch-gui");
        assert!(!autostart::enabled_at(&path));
        autostart::set_at(&path, exe, true).unwrap();
        assert!(autostart::enabled_at(&path));
        let entry = std::fs::read_to_string(&path).unwrap();
        for line in [
            "Exec=\"/opt/cv1 touch/cv1touch-gui\"",
            "Icon=cv1touch-gui",
            "Terminal=false",
            "X-GNOME-Autostart-enabled=true",
        ] {
            assert!(entry.lines().any(|l| l == line), "{line} in {entry}");
        }
        autostart::set_at(&path, exe, false).unwrap();
        assert!(!autostart::enabled_at(&path));
        // Removing it twice is not an error.
        autostart::set_at(&path, exe, false).unwrap();
        std::fs::remove_dir_all(dir).unwrap();
    }
}
