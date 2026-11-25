#![cfg_attr(windows, windows_subsystem = "windows")]

mod actions;
mod cameras;
mod client;
mod config_stats;
#[cfg(target_os = "linux")]
mod desktop;
mod icon;
mod model;
#[cfg(any(target_os = "linux", test))]
mod png;
mod preview_client;
#[cfg(test)]
mod screenshots;
mod tray;
mod window;

const HELP: &str = "\
Usage: cv1touch-gui [OPTION]

Shows cv1touch's tray icon. Running it again opens the status window.

Options:
  --window             Open the status window only
  --cameras            Open the live camera views only
  --install-desktop    Add the desktop entry and icons for your user (Linux)
  --uninstall-desktop  Remove them again
  --help               Show this help";

fn main() {
    let result = match std::env::args().nth(1).as_deref() {
        None => tray::run(),
        Some("--window") => tray::run_window(tray::Kind::Status),
        Some("--cameras") => tray::run_window(tray::Kind::Cameras),
        #[cfg(target_os = "linux")]
        Some(flag @ ("--install-desktop" | "--uninstall-desktop")) => {
            desktop::run(flag == "--install-desktop")
        }
        Some("--help" | "-h") => {
            println!("{HELP}");
            Ok(())
        }
        Some(other) => Err(format!("unknown option {other}; see --help").into()),
    };
    if let Err(error) = result {
        eprintln!("cv1touch-gui: {error}");
        std::process::exit(1);
    }
}
