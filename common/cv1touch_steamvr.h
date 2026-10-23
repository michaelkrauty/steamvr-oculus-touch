// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* What the SteamVR driver and `cv1touch steamvr install` must agree on outside shared memory.
 * This header is the single definition: Rust gets it through bindgen (src/ffi.rs), the driver
 * includes it. */
#ifndef CV1TOUCH_STEAMVR_H
#define CV1TOUCH_STEAMVR_H

/* The controllers' SteamVR controller type (Prop_ControllerType_String), and the controller_type
 * of the input profile `cv1touch steamvr install` makes for them. SteamVR saves the bindings a
 * user makes per controller type, so this must never change. */
#define CV1T_CONTROLLER_TYPE "cv1touch_touch"

#endif
