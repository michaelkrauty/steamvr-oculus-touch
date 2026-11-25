// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The desktop entry and icons that give the window its icon on Wayland, where compositors
//! look the icon up by the window's app ID instead of taking it from the program.

use crate::{
    icon::{self, Tint},
    window::APP_ID,
};
use std::{
    ffi::OsString,
    io,
    path::{Path, PathBuf},
    process::{Command, Stdio},
};

const SIZES: [usize; 3] = [32, 64, 128];

/// `$XDG_DATA_HOME`, or `~/.local/share` when it is unset or not absolute.
fn data_home(xdg: Option<OsString>, home: Option<OsString>) -> io::Result<PathBuf> {
    xdg.map(PathBuf::from)
        .filter(|p| p.is_absolute())
        .or_else(|| home.map(|h| PathBuf::from(h).join(".local/share")))
        .ok_or_else(|| io::Error::other("neither XDG_DATA_HOME nor HOME is set"))
}

fn files(data: &Path) -> Vec<PathBuf> {
    let icons = data.join("icons/hicolor");
    SIZES
        .iter()
        .map(|n| icons.join(format!("{n}x{n}/apps/{APP_ID}.png")))
        .chain([data.join(format!("applications/{APP_ID}.desktop"))])
        .collect()
}

/// `exe` as one argument of a desktop entry's `Exec` key.
fn exec_arg(exe: &Path) -> String {
    let exe = exe.to_string_lossy();
    let plain = |c: char| c.is_ascii_alphanumeric() || "/._-+".contains(c);
    let arg = if exe.chars().all(plain) {
        exe.into_owned()
    } else {
        let mut quoted = String::from('"');
        for c in exe.chars() {
            if "\"`$\\".contains(c) {
                quoted.push('\\');
            }
            quoted.push(c);
        }
        quoted + "\""
    };
    // The desktop entry format unescapes backslashes once more, and `%` starts a field code.
    arg.replace('\\', "\\\\").replace('%', "%%")
}

fn install(data: &Path, exe: &Path) -> io::Result<Vec<PathBuf>> {
    let files = files(data);
    for (n, path) in SIZES.iter().zip(&files) {
        std::fs::create_dir_all(path.parent().unwrap())?;
        crate::png::write(path, &icon::rgba(Tint::Ready, *n), (*n, *n))?;
    }
    let entry = files.last().unwrap();
    std::fs::create_dir_all(entry.parent().unwrap())?;
    std::fs::write(
        entry,
        format!(
            "[Desktop Entry]\nType=Application\nName=cv1touch\nComment=Status tray and window for cv1touch\nExec={}\nIcon={APP_ID}\nStartupWMClass={APP_ID}\nTerminal=false\nCategories=Utility;\n",
            exec_arg(exe)
        ),
    )?;
    Ok(files)
}

fn uninstall(data: &Path) -> io::Result<Vec<PathBuf>> {
    let mut removed = Vec::new();
    for path in files(data) {
        match std::fs::remove_file(&path) {
            Ok(()) => removed.push(path),
            Err(e) if e.kind() == io::ErrorKind::NotFound => {}
            Err(e) => return Err(e),
        }
    }
    Ok(removed)
}

/// Asks the desktop to notice the change, with whichever cache tools this system has.
fn refresh(data: &Path) {
    let icons = data.join("icons/hicolor");
    let applications = data.join("applications");
    for (tool, args) in [
        (
            "gtk-update-icon-cache",
            vec![OsString::from("-f"), "-t".into(), icons.into()],
        ),
        ("update-desktop-database", vec![applications.into()]),
        ("kbuildsycoca6", vec![]),
    ] {
        let _ = Command::new(tool)
            .args(args)
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
    }
}

/// `--install-desktop` and `--uninstall-desktop`.
pub fn run(install_it: bool) -> Result<(), Box<dyn std::error::Error>> {
    let data = data_home(std::env::var_os("XDG_DATA_HOME"), std::env::var_os("HOME"))?;
    let paths = if install_it {
        install(&data, &std::env::current_exe()?)?
    } else {
        uninstall(&data)?
    };
    refresh(&data);
    let verb = if install_it { "Wrote" } else { "Removed" };
    if paths.is_empty() {
        println!("Nothing to remove.");
    }
    for path in paths {
        println!("{verb} {}", path.display());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn data_home_follows_the_base_directory_spec() {
        let dir = |xdg: Option<&str>, home: Option<&str>| {
            data_home(xdg.map(Into::into), home.map(Into::into)).ok()
        };
        assert_eq!(dir(Some("/x/data"), Some("/h")), Some("/x/data".into()));
        assert_eq!(
            dir(Some("relative"), Some("/h")),
            Some("/h/.local/share".into())
        );
        assert_eq!(dir(None, Some("/h")), Some("/h/.local/share".into()));
        assert_eq!(dir(None, None), None);
    }

    #[test]
    fn exec_quotes_what_the_desktop_would_split_or_expand() {
        assert_eq!(
            exec_arg(Path::new("/opt/cv1touch/cv1touch-gui")),
            "/opt/cv1touch/cv1touch-gui"
        );
        assert_eq!(exec_arg(Path::new("/a b/c%d$e")), r#""/a b/c%%d\\$e""#);
    }

    #[test]
    fn install_and_uninstall_round_trip() {
        let data =
            std::env::temp_dir().join(format!("cv1touch-gui-desktop-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&data);
        let written = install(&data, Path::new("/opt/cv1touch/cv1touch-gui")).unwrap();
        assert_eq!(written, files(&data));
        let entry = std::fs::read_to_string(written.last().unwrap()).unwrap();
        for line in [
            "[Desktop Entry]",
            "Exec=/opt/cv1touch/cv1touch-gui",
            "Icon=cv1touch-gui",
            "StartupWMClass=cv1touch-gui",
        ] {
            assert!(
                entry.lines().any(|l| l == line),
                "{line} missing from {entry}"
            );
        }
        for (n, path) in SIZES.iter().zip(&written) {
            assert!(path.ends_with(format!("icons/hicolor/{n}x{n}/apps/cv1touch-gui.png")));
            let png = std::fs::read(path).unwrap();
            assert_eq!(&png[..8], b"\x89PNG\r\n\x1a\n");
            assert_eq!(
                png[16..24],
                [(*n as u32).to_be_bytes(), (*n as u32).to_be_bytes()].concat()
            );
        }
        // Installing again overwrites; uninstalling removes exactly what was written.
        install(&data, Path::new("/opt/cv1touch/cv1touch-gui")).unwrap();
        std::fs::write(data.join("applications/other.desktop"), "").unwrap();
        assert_eq!(uninstall(&data).unwrap(), written);
        assert!(written.iter().all(|p| !p.exists()));
        assert!(data.join("applications/other.desktop").exists());
        assert!(uninstall(&data).unwrap().is_empty());
        std::fs::remove_dir_all(&data).unwrap();
    }
}
