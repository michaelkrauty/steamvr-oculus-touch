// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! What differs between Linux and Windows, behind one interface: the rest of the crate calls these
//! and never the operating system's own APIs directly.

#[cfg(unix)]
mod unix;
#[cfg(unix)]
use unix as os;
#[cfg(windows)]
mod windows;
#[cfg(windows)]
use windows as os;

pub mod fs {
    //! Files that hold the person's calibration and setup: never followed through links, private
    //! to them, and replaced atomically.
    use std::fs::File;
    use std::io;
    use std::path::Path;

    use super::os;

    /// Opens `path` for reading without following a link at its end and without blocking on a
    /// special file; the caller checks it is a regular file.
    pub fn open_read_nofollow(path: &Path) -> io::Result<File> {
        os::open_read_nofollow(path)
    }

    /// Opens `path` for reading without blocking on a special file (links are followed).
    pub fn open_read_nonblocking(path: &Path) -> io::Result<File> {
        os::open_read_nonblocking(path)
    }

    /// Creates `path`, which must not exist yet, readable and writable by this user only.
    pub fn create_private(path: &Path) -> io::Result<File> {
        os::create_private(path)
    }

    /// Creates `path` and its parents, new directories private to this user.
    pub fn create_private_dir_all(path: &Path) -> io::Result<()> {
        os::create_private_dir_all(path)
    }

    /// Flushes the file at `path` to disk.
    pub fn sync_file(path: &Path) -> io::Result<()> {
        os::sync_file(path)
    }

    /// Makes a rename into `dir` durable, where the platform can.
    pub fn sync_dir(dir: &Path) -> io::Result<()> {
        os::sync_dir(dir)
    }

    /// Renames `from` over `to`, replacing it.
    pub fn rename_replacing(from: &Path, to: &Path) -> io::Result<()> {
        os::rename_replacing(from, to)
    }

    /// Puts directory `stage` in place of `dir`. With `exists`, the two swap, so `stage` then holds
    /// the old contents; without, `dir` must not exist. Atomic where the platform allows it.
    pub fn swap_dir(stage: &Path, dir: &Path, exists: bool) -> io::Result<()> {
        os::swap_dir(stage, dir, exists)
    }
}

pub mod dylib {
    //! Libraries loaded at run time (the OpenVR client from the SteamVR installation).
    use std::ffi::{c_void, CStr};
    use std::path::Path;

    use super::os;

    /// A loaded library, unloaded when dropped.
    pub struct Library(*mut c_void);

    // SAFETY: a library handle is process-wide; loading and lookups are thread-safe on both
    // platforms.
    unsafe impl Send for Library {}

    impl Library {
        /// Loads the library at `path`, or by `path` as a bare name from the system's search path.
        pub fn open(path: &Path) -> Option<Library> {
            let handle = os::dylib_open(path);
            (!handle.is_null()).then(|| Library(handle))
        }

        /// A handle to nothing, for tests that build a client without a library; never drop it.
        #[cfg(test)]
        pub(crate) fn null() -> Library {
            Library(std::ptr::null_mut())
        }

        /// The address of `name`, or null.
        pub fn symbol(&self, name: &CStr) -> *mut c_void {
            os::dylib_symbol(self.0, name)
        }
    }

    impl Drop for Library {
        fn drop(&mut self) {
            os::dylib_close(self.0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::fs;

    #[test]
    fn a_system_library_loads_and_resolves_a_symbol() {
        #[cfg(unix)]
        let (lib, sym) = ("libc.so.6", c"getpid");
        #[cfg(windows)]
        let (lib, sym) = ("kernel32.dll", c"GetCurrentProcessId");
        let lib = super::dylib::Library::open(std::path::Path::new(lib)).unwrap();
        assert!(!lib.symbol(sym).is_null());
        assert!(lib.symbol(c"cv1touch_no_such_symbol").is_null());
        assert!(
            super::dylib::Library::open(std::path::Path::new("cv1touch-no-such-lib")).is_none()
        );
    }

    fn scratch(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("cv1t-platform-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn a_private_file_is_created_once_and_read_back() {
        let dir = scratch("private");
        let path = dir.join("a/b/c.json");
        fs::create_private_dir_all(path.parent().unwrap()).unwrap();
        use std::io::{Read, Write};
        fs::create_private(&path).unwrap().write_all(b"{}").unwrap();
        assert!(fs::create_private(&path).is_err(), "never overwrites");
        let mut s = String::new();
        fs::open_read_nofollow(&path)
            .unwrap()
            .read_to_string(&mut s)
            .unwrap();
        assert_eq!(s, "{}");
        fs::sync_dir(path.parent().unwrap()).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_rename_replaces_the_target() {
        let dir = scratch("rename");
        std::fs::write(dir.join("new"), "new").unwrap();
        std::fs::write(dir.join("old"), "old").unwrap();
        fs::rename_replacing(&dir.join("new"), &dir.join("old")).unwrap();
        assert_eq!(std::fs::read_to_string(dir.join("old")).unwrap(), "new");
        assert!(!dir.join("new").exists());
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_directory_swap_leaves_the_old_contents_in_the_stage() {
        let dir = scratch("swap");
        let (stage, live) = (dir.join("stage"), dir.join("live"));
        std::fs::create_dir(&stage).unwrap();
        std::fs::write(stage.join("v"), "2").unwrap();
        fs::swap_dir(&stage, &live, false).unwrap();
        assert_eq!(std::fs::read_to_string(live.join("v")).unwrap(), "2");
        std::fs::create_dir(&stage).unwrap();
        std::fs::write(stage.join("v"), "3").unwrap();
        fs::swap_dir(&stage, &live, true).unwrap();
        assert_eq!(std::fs::read_to_string(live.join("v")).unwrap(), "3");
        assert_eq!(std::fs::read_to_string(stage.join("v")).unwrap(), "2");
        // without `exists`, an existing target is never replaced
        std::fs::create_dir(dir.join("other")).unwrap();
        assert!(fs::swap_dir(&dir.join("other"), &live, false).is_err());
        assert_eq!(std::fs::read_to_string(live.join("v")).unwrap(), "3");
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
