// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Raw bindings (bindgen) to OpenHMD, the cv1touch engine extensions and the shared-memory ABI.
#![allow(
    non_camel_case_types,
    non_upper_case_globals,
    non_snake_case,
    dead_code,
    clippy::all
)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
