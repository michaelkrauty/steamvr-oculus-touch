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
mod setup_client;
mod setup_window;
mod tray;
mod window;

pub(crate) const HELP: &str = "\
Usage: cv1touch-gui [OPTION]

Shows cv1touch's tray icon. Running it again opens the status window.

Options:
  --window             Open the status window only
  --sensors            Open the live sensor views only
  --setup [--stage STAGE [--quick]]
                       Open the guided setup, or one stage of it: sensors, floor,
                       model, buttons or headset (--quick: the short headset
                       alignment)
  --install-desktop    Add the desktop entry and icons for your user (Linux)
  --uninstall-desktop  Remove them again
  --help               Show this help";

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = match args.first().map(String::as_str) {
        None => tray::run(),
        Some("--window") => tray::run_window(tray::Kind::Status),
        // --cameras is the older name.
        Some("--sensors" | "--cameras") => tray::run_window(tray::Kind::Sensors),
        Some("--setup") => setup_client::Request::parse(&args[1..])
            .map_err(Into::into)
            .and_then(|request| {
                setup_window::request(request);
                tray::run_window(tray::Kind::Setup)
            }),
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
