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
mod res;
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
  --quit               Close the windows and the tray icon (cv1touchd keeps running)
  --version            Show the version
  --help               Show this help";

/// What the command line asks for.
#[derive(Debug, PartialEq)]
enum Command {
    Tray,
    Window(tray::Kind),
    Setup(setup_client::Request),
    #[cfg(target_os = "linux")]
    Desktop(bool),
    Quit,
    Print(String),
}

fn parse(args: &[String]) -> Result<Command, String> {
    Ok(match args.first().map(String::as_str) {
        None => Command::Tray,
        Some("--window") => Command::Window(tray::Kind::Status),
        // --cameras is the older name.
        Some("--sensors" | "--cameras") => Command::Window(tray::Kind::Sensors),
        Some("--setup") => Command::Setup(setup_client::Request::parse(&args[1..])?),
        #[cfg(target_os = "linux")]
        Some(flag @ ("--install-desktop" | "--uninstall-desktop")) => {
            Command::Desktop(flag == "--install-desktop")
        }
        Some("--quit") => Command::Quit,
        Some("--version" | "-V") => {
            Command::Print(format!("cv1touch-gui {}", env!("CARGO_PKG_VERSION")))
        }
        Some("--help" | "-h") => Command::Print(HELP.into()),
        Some(other) => return Err(format!("unknown option {other}; see --help")),
    })
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = parse(&args)
        .map_err(Into::into)
        .and_then(|command| match command {
            Command::Tray => tray::run(),
            Command::Window(kind) => tray::run_window(kind),
            Command::Setup(request) => {
                setup_window::request(request);
                tray::run_window(tray::Kind::Setup)
            }
            #[cfg(target_os = "linux")]
            Command::Desktop(install) => desktop::run(install),
            Command::Quit => tray::quit(),
            Command::Print(text) => {
                println!("{text}");
                Ok(())
            }
        });
    if let Err(error) = result {
        eprintln!("cv1touch-gui: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> Result<Command, String> {
        super::parse(&args.iter().map(|a| a.to_string()).collect::<Vec<_>>())
    }

    #[test]
    fn parses_version_and_quit() {
        let version = format!("cv1touch-gui {}", env!("CARGO_PKG_VERSION"));
        for flag in ["--version", "-V"] {
            assert_eq!(parse(&[flag]), Ok(Command::Print(version.clone())));
        }
        assert_eq!(parse(&["--quit"]), Ok(Command::Quit));
        assert_eq!(parse(&[]), Ok(Command::Tray));
        assert_eq!(
            parse(&["--sensors"]),
            Ok(Command::Window(tray::Kind::Sensors))
        );
        assert!(parse(&["--stop"]).is_err());
    }
}
