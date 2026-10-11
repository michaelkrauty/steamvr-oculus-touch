// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! Minimal OpenVR client (a background application) over the C API, loaded at runtime from
//! libopenvr_api.so: device list, raw-universe poses and properties. Used by `cv1touch register`
//! and `cv1touch vr-devices`; SteamVR must be running.

use std::ffi::{c_char, c_void, CStr, CString};
use std::os::unix::ffi::OsStrExt;

use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion};

#[allow(
    non_upper_case_globals,
    non_camel_case_types,
    non_snake_case,
    dead_code,
    clippy::all
)]
mod sys {
    include!(concat!(env!("OUT_DIR"), "/openvr.rs"));
}

pub const CLASS_CONTROLLER: u32 = sys::ETrackedDeviceClass_TrackedDeviceClass_Controller;
pub const CLASS_HMD: u32 = sys::ETrackedDeviceClass_TrackedDeviceClass_HMD;
pub const CLASS_TRACKING_REFERENCE: u32 =
    sys::ETrackedDeviceClass_TrackedDeviceClass_TrackingReference;
pub const PROP_CURRENT_UNIVERSE_ID: u32 = sys::ETrackedDeviceProperty_Prop_CurrentUniverseId_Uint64;
const MAX_DEVICES: usize = 64;

#[derive(Debug, Clone, PartialEq)]
pub struct Device {
    pub index: u32,
    pub class: u32,
    pub serial: String,
    pub model: String,
    pub tracking_system: String,
    pub controller_type: String,
}

pub fn iso_from_matrix34(m: &[[f32; 4]; 3]) -> Isometry3<f64> {
    let r = Matrix3::from_fn(|i, j| m[i][j] as f64);
    let q = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix(&r));
    Isometry3::from_parts(
        Translation3::new(m[0][3] as f64, m[1][3] as f64, m[2][3] as f64),
        q,
    )
}

/// A controller tracked by Lighthouse that is an Index controller.
pub fn find_reference(devices: &[Device]) -> Option<&Device> {
    devices.iter().find(|d| {
        d.class == CLASS_CONTROLLER
            && d.tracking_system == "lighthouse"
            && d.controller_type == "knuckles"
    })
}

type InitFn = unsafe extern "C" fn(*mut u32, u32, *const c_char) -> u32;
type GetInterfaceFn = unsafe extern "C" fn(*const c_char, *mut u32) -> isize;
type ShutdownFn = unsafe extern "C" fn();

/// Pressed bits, touched bits and the five legacy controller axes.
pub type ControllerState = (u64, u64, [(f32, f32); 5]);

// The C API header omits the SDK's Linux/macOS pack(4) for VRControllerState001_t.
// Use the driver SDK's platform layout at this boundary instead of the generated C struct.
#[cfg_attr(any(target_os = "linux", target_os = "macos"), repr(C, packed(4)))]
#[cfg_attr(not(any(target_os = "linux", target_os = "macos")), repr(C))]
#[derive(Default)]
struct LegacyControllerState {
    packet_num: u32,
    button_pressed: u64,
    button_touched: u64,
    axes: [sys::VRControllerAxis_t; 5],
}

/// Raw-universe pose, linear velocity and angular velocity.
pub type PoseWithVelocity = (Isometry3<f64>, [f64; 3], [f64; 3]);

pub struct Client {
    lib: *mut c_void,
    sys: &'static sys::VR_IVRSystem_FnTable,
    shutdown: ShutdownFn,
}

// SAFETY: OpenVR's client interfaces may be called from any thread; the client is used by one
// thread at a time.
unsafe impl Send for Client {}

impl Client {
    pub fn connect() -> Result<Client, String> {
        // SAFETY: dlopen/dlsym with NUL-terminated names; symbols have the declared C signatures.
        unsafe {
            let bundled = crate::steamvr::runtime_root().join("bin/linux64/libopenvr_api.so");
            let mut lib = CString::new(bundled.as_os_str().as_bytes())
                .ok()
                .map_or(std::ptr::null_mut(), |p| {
                    libc::dlopen(p.as_ptr(), libc::RTLD_NOW | libc::RTLD_LOCAL)
                });
            if lib.is_null() {
                lib = libc::dlopen(
                    c"libopenvr_api.so".as_ptr(),
                    libc::RTLD_NOW | libc::RTLD_LOCAL,
                );
            }
            if lib.is_null() {
                return Err(format!("cannot load OpenVR client library at {} or libopenvr_api.so; set CV1TOUCH_STEAMVR to the SteamVR runtime directory", bundled.display()));
            }
            let sym = |name: &CStr| libc::dlsym(lib, name.as_ptr());
            let (init, get, shutdown) = (
                sym(c"VR_InitInternal2"),
                sym(c"VR_GetGenericInterface"),
                sym(c"VR_ShutdownInternal"),
            );
            if init.is_null() || get.is_null() || shutdown.is_null() {
                libc::dlclose(lib);
                return Err("libopenvr_api.so lacks VR_InitInternal2/VR_GetGenericInterface/VR_ShutdownInternal".into());
            }
            // SAFETY: the non-null dlsym results name OpenVR functions with these C ABIs;
            // the library remains loaded until Client is dropped.
            let (init, get, shutdown) = (
                std::mem::transmute::<*mut c_void, InitFn>(init),
                std::mem::transmute::<*mut c_void, GetInterfaceFn>(get),
                std::mem::transmute::<*mut c_void, ShutdownFn>(shutdown),
            );
            let mut err = 0u32;
            init(
                &mut err,
                sys::EVRApplicationType_VRApplication_Background,
                std::ptr::null(),
            );
            if err != 0 {
                libc::dlclose(lib);
                return Err(format!(
                    "OpenVR init failed (error {err}): is SteamVR running?"
                ));
            }
            let table = get(c"FnTable:IVRSystem_026".as_ptr(), &mut err)
                as *const sys::VR_IVRSystem_FnTable;
            if err != 0 || table.is_null() {
                shutdown();
                libc::dlclose(lib);
                return Err(format!("OpenVR has no FnTable:IVRSystem_026 (error {err})"));
            }
            Ok(Client {
                lib,
                sys: &*table,
                shutdown,
            })
        }
    }

    fn string_property(&self, index: u32, prop: u32) -> String {
        let mut buf = [0 as c_char; 256];
        let mut err = 0;
        // SAFETY: the function table comes from the runtime; buf outlives the call.
        unsafe {
            let f = self
                .sys
                .GetStringTrackedDeviceProperty
                .expect("GetStringTrackedDeviceProperty");
            f(
                index,
                prop as _,
                buf.as_mut_ptr(),
                buf.len() as u32,
                &mut err,
            );
            CStr::from_ptr(buf.as_ptr()).to_string_lossy().into_owned()
        }
    }

    pub fn uint64_property(&self, index: u32, prop: u32) -> Option<u64> {
        let mut err = 0;
        // SAFETY: as above.
        let v = unsafe {
            self.sys
                .GetUint64TrackedDeviceProperty
                .expect("GetUint64TrackedDeviceProperty")(index, prop as _, &mut err)
        };
        (err == 0).then_some(v)
    }

    pub fn float_property(&self, index: u32, prop: u32) -> Option<f32> {
        let mut err = 0;
        // SAFETY: as above.
        let v = unsafe {
            self.sys
                .GetFloatTrackedDeviceProperty
                .expect("GetFloatTrackedDeviceProperty")(index, prop as _, &mut err)
        };
        (err == 0).then_some(v)
    }

    /// A device's battery charge as SteamVR shows it (0-1), when it reports one.
    pub fn battery(&self, index: u32) -> Option<f32> {
        self.float_property(
            index,
            sys::ETrackedDeviceProperty_Prop_DeviceBatteryPercentage_Float,
        )
    }

    pub fn devices(&self) -> Vec<Device> {
        let class_of = self
            .sys
            .GetTrackedDeviceClass
            .expect("GetTrackedDeviceClass");
        (0..MAX_DEVICES as u32)
            // SAFETY: as above.
            .map(|i| (i, unsafe { class_of(i) }))
            .filter(|&(_, class)| class != sys::ETrackedDeviceClass_TrackedDeviceClass_Invalid)
            .map(|(index, class)| Device {
                index,
                class,
                serial: self
                    .string_property(index, sys::ETrackedDeviceProperty_Prop_SerialNumber_String),
                model: self
                    .string_property(index, sys::ETrackedDeviceProperty_Prop_ModelNumber_String),
                tracking_system: self.string_property(
                    index,
                    sys::ETrackedDeviceProperty_Prop_TrackingSystemName_String,
                ),
                controller_type: self.string_property(
                    index,
                    sys::ETrackedDeviceProperty_Prop_ControllerType_String,
                ),
            })
            .collect()
    }

    /// SteamVR's legacy view of a controller's inputs: (pressed bits, touched bits, five axes).
    pub fn controller_state(&self, index: u32) -> Option<ControllerState> {
        let mut st = LegacyControllerState::default();
        // SAFETY: st has the SDK's platform layout and its exact size is passed. The cast only
        // crosses the FFI boundary; no reference to the generated, differently aligned type is
        // formed. Packed fields below are copied by value, without unaligned references.
        let ok = unsafe {
            self.sys.GetControllerState.expect("GetControllerState")(
                index,
                std::ptr::addr_of_mut!(st).cast(),
                std::mem::size_of::<LegacyControllerState>() as u32,
            )
        };
        ok.then(|| {
            (
                st.button_pressed,
                st.button_touched,
                st.axes.map(|a| (a.x, a.y)),
            )
        })
    }

    /// A legacy haptic pulse (SteamVR routes it to the device's /output/haptic).
    pub fn haptic_pulse(&self, index: u32, duration_us: u16) {
        // SAFETY: as above.
        unsafe { self.sys.TriggerHapticPulse.expect("TriggerHapticPulse")(index, 0, duration_us) }
    }

    /// Raw-universe poses now (no prediction); `Some` when valid and tracking normally.
    /// The Lighthouse base stations tracked now, with their raw-universe poses.
    pub fn stations(&self) -> Vec<(String, Isometry3<f64>)> {
        let poses = self.raw_poses();
        self.devices()
            .into_iter()
            .filter(|d| d.class == CLASS_TRACKING_REFERENCE && d.tracking_system == "lighthouse")
            .filter_map(|d| Some((d.serial, poses.get(d.index as usize).copied().flatten()?)))
            .collect()
    }

    /// Raw-universe poses predicted `seconds` ahead, with SteamVR's velocities (world, rad/s).
    pub fn raw_poses_ahead(&self, seconds: f32) -> Vec<Option<PoseWithVelocity>> {
        let mut poses = [sys::TrackedDevicePose_t::default(); MAX_DEVICES];
        // SAFETY: the array holds MAX_DEVICES poses.
        unsafe {
            self.sys
                .GetDeviceToAbsoluteTrackingPose
                .expect("GetDeviceToAbsoluteTrackingPose")(
                sys::ETrackingUniverseOrigin_TrackingUniverseRawAndUncalibrated as _,
                seconds,
                poses.as_mut_ptr(),
                MAX_DEVICES as u32,
            );
        }
        poses
            .iter()
            .map(|p| {
                p.bPoseIsValid.then(|| {
                    let v = |a: [f32; 3]| [a[0] as f64, a[1] as f64, a[2] as f64];
                    (
                        iso_from_matrix34(&p.mDeviceToAbsoluteTracking.m),
                        v(p.vVelocity.v),
                        v(p.vAngularVelocity.v),
                    )
                })
            })
            .collect()
    }

    pub fn raw_poses(&self) -> Vec<Option<Isometry3<f64>>> {
        let mut poses = [sys::TrackedDevicePose_t::default(); MAX_DEVICES];
        // SAFETY: the array holds MAX_DEVICES poses.
        unsafe {
            self.sys
                .GetDeviceToAbsoluteTrackingPose
                .expect("GetDeviceToAbsoluteTrackingPose")(
                sys::ETrackingUniverseOrigin_TrackingUniverseRawAndUncalibrated as _,
                0.0,
                poses.as_mut_ptr(),
                MAX_DEVICES as u32,
            );
        }
        poses
            .iter()
            .map(|p| {
                (p.bPoseIsValid
                    && p.eTrackingResult == sys::ETrackingResult_TrackingResult_Running_OK)
                    .then(|| iso_from_matrix34(&p.mDeviceToAbsoluteTracking.m))
            })
            .collect()
    }
}

impl Drop for Client {
    fn drop(&mut self) {
        // SAFETY: initialised in connect; shut down once.
        unsafe {
            (self.shutdown)();
            libc::dlclose(self.lib);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[cfg(any(target_os = "linux", target_os = "macos"))]
    const PRESSED: u64 = 0x1234_5678_9abc_def0;
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    const TOUCHED: u64 = 0xfedc_ba98_7654_3210;
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    const AXES: [(f32, f32); 5] = [
        (-0.75, 0.25),
        (0.95, 0.0),
        (0.99, 0.0),
        (0.125, -0.375),
        (-0.625, 0.875),
    ];

    #[cfg(any(target_os = "linux", target_os = "macos"))]
    unsafe extern "C" fn packed_controller_state(
        index: u32,
        state: *mut sys::VRControllerState_t,
        size: u32,
    ) -> bool {
        // Device 9 models a runtime that requires the exact SDK size. Device 10 also accepts
        // larger buffers, exposing shifted field reads independently of size rejection.
        if !(index == 9 && size == 60 || index == 10 && size >= 60) || state.is_null() {
            return false;
        }
        let mut bytes = [0u8; 60];
        bytes[..4].copy_from_slice(&0x1020_3040u32.to_ne_bytes());
        bytes[4..12].copy_from_slice(&PRESSED.to_ne_bytes());
        bytes[12..20].copy_from_slice(&TOUCHED.to_ne_bytes());
        for (axis, (x, y)) in AXES.into_iter().enumerate() {
            let offset = 20 + axis * 8;
            bytes[offset..offset + 4].copy_from_slice(&x.to_ne_bytes());
            bytes[offset + 4..offset + 8].copy_from_slice(&y.to_ne_bytes());
        }
        // SAFETY: the caller provides at least 60 writable bytes. Copy bytes without assuming
        // the generated C header's incorrect eight-byte alignment for this Linux/macOS ABI.
        unsafe { std::ptr::copy_nonoverlapping(bytes.as_ptr(), state.cast::<u8>(), bytes.len()) };
        true
    }

    #[cfg(any(target_os = "linux", target_os = "macos"))]
    fn mock_controller_client() -> std::mem::ManuallyDrop<Client> {
        unsafe extern "C" fn unused_shutdown() {}
        let table = sys::VR_IVRSystem_FnTable {
            GetControllerState: Some(packed_controller_state),
            ..Default::default()
        };
        // No library was opened. ManuallyDrop avoids the real client's shutdown/dlclose path.
        std::mem::ManuallyDrop::new(Client {
            lib: std::ptr::null_mut(),
            sys: Box::leak(Box::new(table)),
            shutdown: unused_shutdown,
        })
    }

    #[test]
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    fn controller_state_passes_the_platform_legacy_abi_size() {
        let client = mock_controller_client();
        assert_eq!(client.controller_state(9), Some((PRESSED, TOUCHED, AXES)));
        assert_eq!(client.controller_state(11), None);
    }

    #[test]
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    fn controller_state_decodes_real_packed_buttons_and_all_axes() {
        assert_eq!(
            mock_controller_client().controller_state(10),
            Some((PRESSED, TOUCHED, AXES))
        );
    }

    #[test]
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    fn controller_state_layout_preserves_packet_and_sdk_offsets() {
        assert_eq!(std::mem::size_of::<LegacyControllerState>(), 60);
        assert_eq!(std::mem::align_of::<LegacyControllerState>(), 4);
        assert_eq!(std::mem::offset_of!(LegacyControllerState, packet_num), 0);
        assert_eq!(
            std::mem::offset_of!(LegacyControllerState, button_pressed),
            4
        );
        assert_eq!(
            std::mem::offset_of!(LegacyControllerState, button_touched),
            12
        );
        assert_eq!(std::mem::offset_of!(LegacyControllerState, axes), 20);
        let mut state = LegacyControllerState::default();
        // SAFETY: the mock writes exactly the SDK's 60-byte layout into this matching buffer.
        assert!(unsafe { packed_controller_state(9, std::ptr::addr_of_mut!(state).cast(), 60) });
        let packet = state.packet_num;
        assert_eq!(packet, 0x1020_3040);
    }

    #[test]
    fn matrix34_to_isometry() {
        // 90 degrees about +Y, translation (1, 2, 3); OpenVR matrices are row-major [row][col]
        let m = [
            [0.0, 0.0, 1.0, 1.0],
            [0.0, 1.0, 0.0, 2.0],
            [-1.0, 0.0, 0.0, 3.0],
        ];
        let t = iso_from_matrix34(&m);
        assert!((t.translation.vector - Vector3::new(1.0, 2.0, 3.0)).norm() < 1e-9);
        assert!((t.rotation * Vector3::x() - Vector3::new(0.0, 0.0, -1.0)).norm() < 1e-6);
    }

    #[test]
    fn reference_is_a_lighthouse_index_controller() {
        let d = |index, class, sys: &str, ty: &str| Device {
            index,
            class,
            serial: format!("S{index}"),
            model: String::new(),
            tracking_system: sys.into(),
            controller_type: ty.into(),
        };
        let devices = [
            d(0, 1, "lighthouse", ""),
            d(3, CLASS_CONTROLLER, "oculus", "oculus_touch"),
            d(4, CLASS_CONTROLLER, "lighthouse", "knuckles"),
        ];
        assert_eq!(find_reference(&devices).map(|d| d.index), Some(4));
        assert!(find_reference(&devices[..2]).is_none());
    }

    /// Needs SteamVR running: `cargo test --lib vr::tests::live -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn live_lists_devices() {
        let c = Client::connect().unwrap();
        let poses = c.raw_poses();
        for d in c.devices() {
            eprintln!(
                "{d:?} pose {:?}",
                poses[d.index as usize].map(|p| p.translation.vector)
            );
        }
    }
}
