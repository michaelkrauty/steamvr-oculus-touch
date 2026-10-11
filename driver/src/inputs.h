// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#pragma once
#include "cv1touch_shm.h"

namespace cv1t {

/* Grip "touch": the CV1 grip has no capacitive sensor, so a light squeeze counts. */
constexpr float kGripTouch = 0.05f;

struct InputValues {
	bool a_click = false, a_touch = false, b_click = false, b_touch = false, system_click = false;
	bool joy_click = false, joy_touch = false, trigger_touch = false, grip_touch = false, thumbrest_touch = false;
	float trigger = 0, grip = 0, joy_x = 0, joy_y = 0;
	bool operator==(const InputValues &) const = default;
};

/* Left hand: a/b are the X/Y buttons. No data, not connected, or dead sample: everything released. */
InputValues map_inputs(const cv1t_controller_state *st, int hand, uint64_t now_ns);

/* The face-button and system component paths per hand (touch_profile.json). system_click is
 * nullptr for the right hand: its Oculus button belongs to the runtime. */
struct FaceNames {
	const char *a_click, *a_touch, *b_click, *b_touch, *system_click;
};
FaceNames face_names(int hand);

} // namespace cv1t
