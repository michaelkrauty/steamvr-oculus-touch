// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

#include <cassert>
#include <cstdio>
#include <cstring>

#include "inputs.h"
#include "pose.h"

using cv1t::map_inputs;

static cv1t_controller_state awake()
{
	cv1t_controller_state s{};
	s.flags = CV1T_FLAG_CALIBRATED | CV1T_FLAG_CONNECTED;
	return s;
}

static void right_hand_bits()
{
	cv1t_controller_state s = awake();
	s.buttons = CV1T_BTN_A | CV1T_BTN_STICK;
	s.touches = CV1T_TOUCH_B | CV1T_TOUCH_TRIGGER | CV1T_TOUCH_THUMBREST;
	s.trigger = 0.7f;
	s.grip = 0.02f;
	s.stick[0] = -0.5f;
	s.stick[1] = 0.25f;
	const auto v = map_inputs(&s, CV1T_HAND_RIGHT, 0);
	assert(v.a_click && !v.a_touch && !v.b_click && v.b_touch);
	assert(v.joy_click && v.joy_touch); // a click implies a touch
	assert(v.trigger_touch && v.thumbrest_touch && !v.grip_touch);
	assert(v.trigger == 0.7f && v.grip == 0.02f && v.joy_x == -0.5f && v.joy_y == 0.25f);
}

static void left_hand_uses_x_and_y()
{
	cv1t_controller_state s = awake();
	s.buttons = CV1T_BTN_Y | CV1T_BTN_SYSTEM;
	s.touches = CV1T_TOUCH_X;
	s.grip = 0.3f;
	const auto v = map_inputs(&s, CV1T_HAND_LEFT, 0);
	assert(!v.a_click && v.a_touch && v.b_click && !v.b_touch && v.system_click && v.grip_touch);
	const auto n = cv1t::face_names(CV1T_HAND_LEFT);
	assert(std::strcmp(n.a_click, "/input/x/click") == 0 && std::strcmp(n.b_touch, "/input/y/touch") == 0);
	assert(std::strcmp(n.system_click, "/input/system/click") == 0);
	assert(cv1t::face_names(CV1T_HAND_RIGHT).system_click == nullptr);
	assert(std::strcmp(cv1t::face_names(CV1T_HAND_RIGHT).a_click, "/input/a/click") == 0);
}

static void asleep_or_missing_releases_everything()
{
	cv1t_controller_state s = awake();
	s.buttons = 0xff;
	s.touches = 0xff;
	s.trigger = 1;
	s.flags = CV1T_FLAG_CALIBRATED;
	assert(map_inputs(&s, CV1T_HAND_RIGHT, 0) == cv1t::InputValues{});
	assert(map_inputs(nullptr, CV1T_HAND_LEFT, 0) == cv1t::InputValues{});
}

// The control loop can keep heartbeating while the publishing thread hangs.
// Its last CONNECTED snapshot must not keep buttons held after the pose disconnects.
static void a_dead_sample_releases_everything()
{
    auto s = awake();
    s.sample_host_ns = 1'000'000'000;
    s.orient[3] = 1;
    s.buttons = CV1T_BTN_A | CV1T_BTN_STICK;
    s.touches = 0xff;
    s.trigger = s.grip = 1;
    s.stick[0] = s.stick[1] = 0.5f;
    const auto boundary = s.sample_host_ns + cv1t::kDeadSampleNs;
    assert(map_inputs(&s, CV1T_HAND_RIGHT, boundary).a_click);
    assert(map_inputs(&s, CV1T_HAND_RIGHT, s.sample_host_ns - 1).a_click);
    const auto pose = cv1t::make_pose(&s, nullptr, CV1T_HAND_RIGHT, boundary + 1);
    assert(!pose.deviceIsConnected);
    assert(map_inputs(&s, CV1T_HAND_RIGHT, boundary + 1) == cv1t::InputValues{});
}

int main()
{
	right_hand_bits();
	left_hand_uses_x_and_y();
	asleep_or_missing_releases_everything();
	a_dead_sample_releases_everything();
	printf("test_inputs ok\n");
	return 0;
}
