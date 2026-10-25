// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! A detached supervisor must inherit no handles, including non-stdio handles forwarded by
//! an installer or command shell. Setting Command's stdio to null does not provide this on
//! our oldest supported Rust, whose Windows spawn inherits every inheritable handle.

use std::ffi::OsStr;
use std::io;
use std::os::windows::ffi::OsStrExt;
use std::process::Command;
use std::time::Duration;
use windows_sys::Win32::Foundation::{CloseHandle, WAIT_OBJECT_0, WAIT_TIMEOUT};
use windows_sys::Win32::System::Threading::{
    CreateProcessW, GetExitCodeProcess, WaitForSingleObject, CREATE_NO_WINDOW, PROCESS_INFORMATION,
    STARTUPINFOW,
};

fn quoted(value: &OsStr) -> Vec<u16> {
    let mut result = vec![b'"' as u16];
    let mut slashes = 0;
    for unit in value.encode_wide() {
        if unit == b'\\' as u16 {
            slashes += 1;
            continue;
        }
        let escaping_quote = unit == b'"' as u16;
        result.extend(std::iter::repeat_n(
            b'\\' as u16,
            if escaping_quote {
                slashes * 2 + 1
            } else {
                slashes
            },
        ));
        result.push(unit);
        slashes = 0;
    }
    result.extend(std::iter::repeat_n(b'\\' as u16, slashes * 2));
    result.push(b'"' as u16);
    result
}

/// Only the program and arguments from our private supervisor builder are used. The child
/// inherits the environment and working directory, but no console or kernel handles.
pub(super) fn launch(command: &Command, early: Duration) -> io::Result<Option<u32>> {
    let program: Vec<u16> = command.get_program().encode_wide().chain([0]).collect();
    let mut line = quoted(command.get_program());
    for argument in command.get_args() {
        line.push(b' ' as u16);
        line.extend(quoted(argument));
    }
    line.push(0);
    // SAFETY: correctly sized structures, terminated program/command strings, and null
    // optional parameters. FALSE disables inheritance of *all* parent handles. The two
    // returned handles are closed exactly once, including on wait/query failure.
    unsafe {
        let mut startup: STARTUPINFOW = std::mem::zeroed();
        startup.cb = std::mem::size_of_val(&startup) as u32;
        let mut process: PROCESS_INFORMATION = std::mem::zeroed();
        if CreateProcessW(
            program.as_ptr(),
            line.as_mut_ptr(),
            std::ptr::null(),
            std::ptr::null(),
            0,
            CREATE_NO_WINDOW,
            std::ptr::null(),
            std::ptr::null(),
            &startup,
            &mut process,
        ) == 0
        {
            return Err(io::Error::last_os_error());
        }
        CloseHandle(process.hThread);
        let result = match WaitForSingleObject(process.hProcess, early.as_millis() as u32) {
            WAIT_TIMEOUT => Ok(None),
            WAIT_OBJECT_0 => {
                let mut code = 0;
                if GetExitCodeProcess(process.hProcess, &mut code) == 0 {
                    Err(io::Error::last_os_error())
                } else {
                    Ok(Some(code))
                }
            }
            _ => Err(io::Error::last_os_error()),
        };
        CloseHandle(process.hProcess);
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use windows_sys::Win32::Foundation::LocalFree;
    use windows_sys::Win32::UI::Shell::CommandLineToArgvW;

    #[test]
    fn arguments_round_trip_through_windows_parsing() {
        let values = [
            "C:\\program ü & spaces\\cv1touchd.exe",
            "--supervise",
            "",
            "a b",
            "one\"two",
            "a\\\"b",
            "trailing\\",
            "two\\\\",
        ];
        let mut line = Vec::new();
        for value in values {
            if !line.is_empty() {
                line.push(b' ' as u16);
            }
            line.extend(quoted(OsStr::new(value)));
        }
        line.push(0);
        // SAFETY: a terminated command line; the API owns argc pointers to terminated
        // strings in one allocation, which is released with LocalFree after copying.
        unsafe {
            let mut count = 0;
            let args = CommandLineToArgvW(line.as_ptr(), &mut count);
            assert!(!args.is_null());
            let actual: Vec<_> = std::slice::from_raw_parts(args, count as usize)
                .iter()
                .map(|&arg| {
                    let mut length = 0;
                    while *arg.add(length) != 0 {
                        length += 1;
                    }
                    String::from_utf16(std::slice::from_raw_parts(arg, length)).unwrap()
                })
                .collect();
            LocalFree(args.cast());
            assert_eq!(actual, values);
        }
    }
}
