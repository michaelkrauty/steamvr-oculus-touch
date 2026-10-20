// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! Private socket fixtures whose Linux endpoint spelling does not grow with TMPDIR.

use std::fs;
use std::io;
use std::path::{Path, PathBuf};

pub struct SocketDir {
    real: PathBuf,
    view: PathBuf,
    #[cfg(unix)]
    _directory: fs::File,
}

impl SocketDir {
    pub fn new() -> Self {
        Self::under(&std::env::temp_dir()).expect("private socket fixture")
    }

    fn under(base: &Path) -> io::Result<Self> {
        #[cfg(unix)]
        {
            use std::os::fd::AsRawFd;
            use std::os::unix::ffi::{OsStrExt, OsStringExt};
            use std::os::unix::fs::{DirBuilderExt, OpenOptionsExt, PermissionsExt};
            let mut template = base.join("cv1t-s-XXXXXX").as_os_str().as_bytes().to_vec();
            template.push(0);
            // SAFETY: a mutable, NUL-terminated template ending in six Xs.
            if unsafe { libc::mkdtemp(template.as_mut_ptr().cast()) }.is_null() {
                return Err(io::Error::last_os_error());
            }
            template.pop();
            let real = PathBuf::from(std::ffi::OsString::from_vec(template));
            let opened = (|| {
                fs::set_permissions(&real, fs::Permissions::from_mode(0o700))?;
                let directory = fs::OpenOptions::new()
                    .read(true)
                    .custom_flags(libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC)
                    .open(&real)?;
                // Use the owner's PID so exec'd clients reach the same directory. The real
                // child keeps final-component lstat checks meaningful: fd itself is a symlink.
                let view = PathBuf::from(format!(
                    "/proc/{}/fd/{}/r",
                    std::process::id(),
                    directory.as_raw_fd()
                ));
                fs::DirBuilder::new().mode(0o700).create(&view)?;
                fs::set_permissions(&view, fs::Permissions::from_mode(0o700))?;
                Ok(Self {
                    real: real.clone(),
                    view,
                    _directory: directory,
                })
            })();
            if opened.is_err() {
                let _ = fs::remove_dir_all(&real);
            }
            opened
        }
        #[cfg(windows)]
        {
            use std::sync::atomic::{AtomicU64, Ordering};
            static NEXT: AtomicU64 = AtomicU64::new(0);
            loop {
                let real = base.join(format!(
                    "cv1t-s-{}-{}",
                    std::process::id(),
                    NEXT.fetch_add(1, Ordering::Relaxed)
                ));
                match fs::create_dir(&real) {
                    Ok(()) => {
                        return Ok(Self {
                            view: real.clone(),
                            real,
                        })
                    }
                    Err(e) if e.kind() == io::ErrorKind::AlreadyExists => continue,
                    Err(e) => return Err(e),
                }
            }
        }
    }

    pub fn path(&self) -> &Path {
        &self.view
    }
}

impl Drop for SocketDir {
    fn drop(&mut self) {
        if let Err(e) = fs::remove_dir_all(&self.real) {
            if e.kind() != io::ErrorKind::NotFound && !std::thread::panicking() {
                panic!("removing socket fixture {}: {e}", self.real.display());
            }
        }
    }
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use std::os::unix::ffi::OsStrExt;
    use std::os::unix::fs::{MetadataExt, PermissionsExt};
    use std::os::unix::net::UnixListener;

    #[test]
    fn a_long_temp_base_keeps_a_private_short_socket_and_cleans_up_its_tree() {
        let parent = SocketDir::new();
        let base = parent.path().join("x".repeat(100));
        fs::create_dir(&base).unwrap();
        let fixture = SocketDir::under(&base).unwrap();
        let real = fixture.real.clone();
        let directory = fs::symlink_metadata(fixture.path()).unwrap();
        assert!(directory.is_dir() && !directory.file_type().is_symlink());
        assert_eq!(directory.permissions().mode() & 0o7777, 0o700);
        // SAFETY: geteuid has no arguments and cannot fail.
        assert_eq!(directory.uid(), unsafe { libc::geteuid() });
        assert!(fixture.path().as_os_str().as_bytes().len() <= 29);
        let endpoint = fixture.path().join("private/control.sock");
        fs::create_dir(endpoint.parent().unwrap()).unwrap();
        let listener = UnixListener::bind(&endpoint).unwrap();
        assert!(fs::canonicalize(&endpoint)
            .unwrap()
            .starts_with(fs::canonicalize(&base).unwrap()));
        drop(listener);
        drop(fixture);
        assert!(!real.exists());
        assert!(base.exists(), "cleanup removed the caller's directory");
    }
}
