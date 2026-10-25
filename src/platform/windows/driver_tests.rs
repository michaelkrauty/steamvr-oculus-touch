// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! Exercise the real inbox-driver selection APIs without a physical USB device.
//! This creates only a uniquely named ROOT device on a disposable, elevated CI runner.
//! WinUSB cannot start on this synthetic bus: success proves binding, not USB I/O.

use super::*;
use windows_sys::Win32::Devices::DeviceAndDriverInstallation::{
    DiUninstallDevice, SetupDiCallClassInstaller, SetupDiCreateDeviceInfoW,
    SetupDiGetDeviceInstanceIdW, SetupDiSetDeviceRegistryPropertyW, DICD_GENERATE_ID,
    DIF_REGISTERDEVICE, SPDRP_HARDWAREID, SPDRP_SERVICE,
};

struct TestDevice {
    node: DeviceNode,
    registered: bool,
}

impl Drop for TestDevice {
    fn drop(&mut self) {
        if self.registered {
            // SAFETY: only the generated test node, never a physical device or a preexisting
            // instance. Best-effort cleanup also runs if an assertion unwinds.
            unsafe {
                let mut reboot = 0;
                DiUninstallDevice(
                    std::ptr::null_mut(),
                    self.node.set,
                    &self.node.data,
                    0,
                    &mut reboot,
                );
            }
        }
    }
}

#[test]
#[ignore = "creates a temporary ROOT device; needs a disposable elevated Windows runner"]
fn inbox_winusb_binds_to_a_synthetic_device() {
    assert_eq!(
        std::env::var("CV1TOUCH_TEST_ROOT_DEVICE").as_deref(),
        Ok("1")
    );
    assert!(elevated(), "the disposable runner must be elevated");
    let class = windows_sys::core::GUID::from_u128(0x88bae032_5a81_49f0_bc3d_a4ff138216d6);
    let name = wide(&format!("CV1TOUCH_WINUSB_TEST_{}", std::process::id()));
    // SAFETY: a new set and a generated instance ID. The set and registered device have
    // separate RAII owners; no device outside this set can be changed by this fixture.
    let mut device = unsafe {
        use windows_sys::Win32::Devices::DeviceAndDriverInstallation::{
            SetupDiCreateDeviceInfoList, SP_DEVINFO_DATA,
        };
        let set = SetupDiCreateDeviceInfoList(&class, std::ptr::null_mut());
        assert_ne!(set, INVALID_HANDLE_VALUE as isize);
        let mut device = TestDevice {
            node: DeviceNode {
                set,
                data: std::mem::zeroed(),
            },
            registered: false,
        };
        device.node.data.cbSize = std::mem::size_of::<SP_DEVINFO_DATA>() as u32;
        assert_ne!(
            SetupDiCreateDeviceInfoW(
                set,
                name.as_ptr(),
                &class,
                wide("cv1touch isolated driver-binding test").as_ptr(),
                std::ptr::null_mut(),
                DICD_GENERATE_ID,
                &mut device.node.data,
            ),
            0,
            "{}",
            io::Error::last_os_error()
        );
        // Deliberately not a real device's ID, and not the inbox driver's compatible ID:
        // cv1touch must select WinUSB even when Windows would not select it automatically.
        let hardware: Vec<u16> = "cv1touch-test-only".encode_utf16().chain([0, 0]).collect();
        assert_ne!(
            SetupDiSetDeviceRegistryPropertyW(
                set,
                &mut device.node.data,
                SPDRP_HARDWAREID,
                hardware.as_ptr().cast(),
                (hardware.len() * 2) as u32,
            ),
            0
        );
        assert_ne!(
            SetupDiCallClassInstaller(DIF_REGISTERDEVICE, set, &device.node.data),
            0,
            "{}",
            io::Error::last_os_error()
        );
        device.registered = true;
        device
    };
    let mut id = [0u16; 256];
    // SAFETY: a live registered node and an output buffer of the stated size.
    assert_ne!(
        unsafe {
            SetupDiGetDeviceInstanceIdW(
                device.node.set,
                &device.node.data,
                id.as_mut_ptr(),
                id.len() as u32,
                std::ptr::null_mut(),
            )
        },
        0
    );
    let end = id.iter().position(|c| *c == 0).unwrap();
    let id = String::from_utf16(&id[..end]).unwrap();
    assert!(id
        .to_ascii_uppercase()
        .starts_with("ROOT\\CV1TOUCH_WINUSB_TEST_"));
    assert!(!set_up_by_us(&id));
    assert!(!use_winusb(&id).expect("select and install inbox winusb.inf"));
    assert!(set_up_by_us(&id));
    let bound = DeviceNode::open(&id).unwrap();
    // SAFETY: query only this test's live node.
    let service = unsafe { device_property(bound.set, &bound.data, SPDRP_SERVICE) };
    assert!(service.is_some_and(|s| s.eq_ignore_ascii_case("WinUSB")));
    drop(bound);
    assert!(!remove_driver(&id).expect("remove the test's driver and node"));
    device.registered = false;
    assert!(!set_up_by_us(&id), "test node survived removal");
}
