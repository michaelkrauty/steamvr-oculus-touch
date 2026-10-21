// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Call only in an aborting child, after all setup and immediately before the fatal operation.

pub fn prepare() {
    let limit = libc::rlimit {
        rlim_cur: 0,
        rlim_max: 0,
    };
    // SAFETY: a valid limit structure, affecting only this subprocess.
    assert_eq!(unsafe { libc::setrlimit(libc::RLIMIT_CORE, &limit) }, 0);
    #[cfg(target_os = "linux")]
    {
        // A piped core_pattern invokes its handler even with a zero core limit. Do this
        // after exec, and never in a parent serving sockets through /proc/self/fd.
        assert_eq!(
            unsafe { libc::prctl(libc::PR_SET_DUMPABLE, 0 as libc::c_ulong) },
            0
        );
        let limits = std::fs::read_to_string("/proc/self/limits").unwrap();
        let dumpable = unsafe { libc::prctl(libc::PR_GET_DUMPABLE) };
        if let Some(directory) = std::env::var_os("CV1TOUCH_TEST_ABORT_SNAPSHOTS") {
            std::fs::write(
                std::path::Path::new(&directory).join(format!("{}.limits", std::process::id())),
                format!("dumpable: {dumpable}\n{limits}"),
            )
            .unwrap();
        }
        assert_eq!(dumpable, 0, "expected abort must not invoke a core handler");
        let core = limits
            .lines()
            .find(|line| line.starts_with("Max core file size"))
            .unwrap();
        assert_eq!(
            core.split_whitespace().skip(4).collect::<Vec<_>>(),
            ["0", "0", "bytes"],
            "{core}"
        );
    }
}
