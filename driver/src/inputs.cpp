// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

#include "inputs.h"
#include "pose.h"

namespace cv1t {

InputValues map_inputs(const cv1t_controller_state *st, int hand, uint64_t now_ns)
{
	InputValues v;
	if (!st || !(st->flags & CV1T_FLAG_CONNECTED) ||
	    (now_ns > st->sample_host_ns && now_ns - st->sample_host_ns > kDeadSampleNs))
		return v;
	const bool left = hand == CV1T_HAND_LEFT;
	const uint32_t a_btn = left ? CV1T_BTN_X : CV1T_BTN_A, b_btn = left ? CV1T_BTN_Y : CV1T_BTN_B;
	const uint32_t a_tch = left ? CV1T_TOUCH_X : CV1T_TOUCH_A, b_tch = left ? CV1T_TOUCH_Y : CV1T_TOUCH_B;
	v.a_click = st->buttons & a_btn;
	v.a_touch = st->touches & a_tch;
	v.b_click = st->buttons & b_btn;
	v.b_touch = st->touches & b_tch;
	v.system_click = left && (st->buttons & CV1T_BTN_SYSTEM);
	v.joy_click = st->buttons & CV1T_BTN_STICK;
	v.joy_touch = (st->touches & CV1T_TOUCH_STICK) || v.joy_click;
	v.trigger_touch = st->touches & CV1T_TOUCH_TRIGGER;
	v.thumbrest_touch = st->touches & CV1T_TOUCH_THUMBREST;
	v.trigger = st->trigger;
	v.grip = st->grip;
	v.grip_touch = st->grip > kGripTouch;
	v.joy_x = st->stick[0];
	v.joy_y = st->stick[1];
	return v;
}

FaceNames face_names(int hand)
{
	if (hand == CV1T_HAND_LEFT)
		return {"/input/x/click", "/input/x/touch", "/input/y/click", "/input/y/touch", "/input/system/click"};
	return {"/input/a/click", "/input/a/touch", "/input/b/click", "/input/b/touch", nullptr};
}

} // namespace cv1t
