// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include "touch_device.h"


#include "log.h"
#include "pose.h"

namespace cv1t {

vr::EVRInitError TouchDevice::Activate(uint32_t object_id)
{
	auto *props = vr::VRProperties();
	container_ = props->TrackedDeviceToPropertyContainer(object_id);
	const bool left = hand_ == CV1T_HAND_LEFT;
	props->SetStringProperty(container_, vr::Prop_TrackingSystemName_String, "oculus");
	props->SetStringProperty(container_, vr::Prop_ModelNumber_String, left ? "Oculus Rift CV1 (Left Controller)" : "Oculus Rift CV1 (Right Controller)");
	props->SetStringProperty(container_, vr::Prop_SerialNumber_String, serial());
	props->SetStringProperty(container_, vr::Prop_ManufacturerName_String, "Oculus");
	props->SetStringProperty(container_, vr::Prop_RenderModelName_String, left ? "oculus_cv1_controller_left" : "oculus_cv1_controller_right");
	props->SetStringProperty(container_, vr::Prop_ControllerType_String, "oculus_touch");
	props->SetStringProperty(container_, vr::Prop_InputProfilePath_String, "{oculus}/input/touch_profile.json");
	props->SetStringProperty(container_, vr::Prop_RegisteredDeviceType_String, left ? "cv1touch/CV1T-LEFT" : "cv1touch/CV1T-RIGHT");
	props->SetInt32Property(container_, vr::Prop_ControllerRoleHint_Int32, left ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_RightHand);
	props->SetInt32Property(container_, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);
	props->SetBoolProperty(container_, vr::Prop_WillDriftInYaw_Bool, false);
	props->SetBoolProperty(container_, vr::Prop_DeviceProvidesBatteryStatus_Bool, false);

	auto *in = vr::VRDriverInput();
	const FaceNames f = face_names(hand_);
	in->CreateBooleanComponent(container_, f.a_click, &h_.a_click);
	in->CreateBooleanComponent(container_, f.a_touch, &h_.a_touch);
	in->CreateBooleanComponent(container_, f.b_click, &h_.b_click);
	in->CreateBooleanComponent(container_, f.b_touch, &h_.b_touch);
	h_.system_click = vr::k_ulInvalidInputComponentHandle;
	if (f.system_click)
		in->CreateBooleanComponent(container_, f.system_click, &h_.system_click);
	in->CreateBooleanComponent(container_, "/input/joystick/click", &h_.joy_click);
	in->CreateBooleanComponent(container_, "/input/joystick/touch", &h_.joy_touch);
	in->CreateScalarComponent(container_, "/input/joystick/x", &h_.joy_x, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
	in->CreateScalarComponent(container_, "/input/joystick/y", &h_.joy_y, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
	in->CreateScalarComponent(container_, "/input/trigger/value", &h_.trigger, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
	in->CreateBooleanComponent(container_, "/input/trigger/touch", &h_.trigger_touch);
	in->CreateScalarComponent(container_, "/input/grip/value", &h_.grip, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
	in->CreateBooleanComponent(container_, "/input/grip/touch", &h_.grip_touch);
	in->CreateBooleanComponent(container_, "/input/thumbrest/touch", &h_.thumbrest_touch);
	vr::VRInputComponentHandle_t haptic = vr::k_ulInvalidInputComponentHandle;
	in->CreateHapticComponent(container_, "/output/haptic", &haptic);
	haptic_ = haptic;

	object_id_.store(object_id, std::memory_order_release);
	cv1t_log("cv1touch: %s activated as device %u", serial(), object_id);
	return vr::VRInitError_None;
}

vr::DriverPose_t TouchDevice::GetPose()
{
	std::lock_guard<std::mutex> g(pose_mu_);
	return last_pose_;
}

void TouchDevice::send_inputs(const InputValues &v, double time_offset)
{
	if (inputs_sent_ && v == last_inputs_)
		return;
	auto *in = vr::VRDriverInput();
	const auto b = [&](vr::VRInputComponentHandle_t hnd, bool now, bool before) {
		if (hnd != vr::k_ulInvalidInputComponentHandle && (!inputs_sent_ || now != before))
			in->UpdateBooleanComponent(hnd, now, time_offset);
	};
	const auto s = [&](vr::VRInputComponentHandle_t hnd, float now, float before) {
		if (!inputs_sent_ || now != before)
			in->UpdateScalarComponent(hnd, now, time_offset);
	};
	const InputValues &o = last_inputs_;
	b(h_.a_click, v.a_click, o.a_click);
	b(h_.a_touch, v.a_touch, o.a_touch);
	b(h_.b_click, v.b_click, o.b_click);
	b(h_.b_touch, v.b_touch, o.b_touch);
	b(h_.system_click, v.system_click, o.system_click);
	b(h_.joy_click, v.joy_click, o.joy_click);
	b(h_.joy_touch, v.joy_touch, o.joy_touch);
	b(h_.trigger_touch, v.trigger_touch, o.trigger_touch);
	b(h_.grip_touch, v.grip_touch, o.grip_touch);
	b(h_.thumbrest_touch, v.thumbrest_touch, o.thumbrest_touch);
	s(h_.trigger, v.trigger, o.trigger);
	s(h_.grip, v.grip, o.grip);
	s(h_.joy_x, v.joy_x, o.joy_x);
	s(h_.joy_y, v.joy_y, o.joy_y);
	last_inputs_ = v;
	inputs_sent_ = true;
}

void TouchDevice::record_age(uint64_t age_ns, uint64_t handoff_ns, uint64_t now_ns)
{
	if (ages_since_ns_ == 0)
		ages_since_ns_ = now_ns;
	ages_.push_back(age_ns);
	if (handoff_ns != UINT64_MAX)
		handoffs_.push_back(handoff_ns);
	if (now_ns - ages_since_ns_ < 10'000'000'000ull)
		return;
	cv1t_log("%s", age_report(serial(), ages_, handoffs_).c_str());
	ages_.clear();
	handoffs_.clear();
	ages_since_ns_ = now_ns;
}

void TouchDevice::update(const cv1t_controller_state *st, const cv1t_alignment *al, uint64_t now_ns)
{
	const vr::DriverPose_t pose = make_pose(st, al, hand_, now_ns);
	{
		std::lock_guard<std::mutex> g(pose_mu_);
		last_pose_ = pose;
	}
	const uint32_t id = object_id_.load(std::memory_order_acquire);
	if (id == vr::k_unTrackedDeviceIndexInvalid)
		return;
	if (al && al->universe_id != 0 && al->universe_id != universe_) {
		universe_ = al->universe_id;
		vr::VRProperties()->SetUint64Property(container_, vr::Prop_CurrentUniverseId_Uint64, universe_);
	}
	const bool connected = pose.deviceIsConnected;
	const bool gap = connected && (st->flags & CV1T_FLAG_RADIO_GAP);
	const uint64_t sample = st ? st->sample_host_ns : 0;
	// a new sample, a connection change, a radio gap's start or end, or a 100 ms keep-alive
	if (sample == last_sample_ns_ && connected == last_connected_ && gap == last_gap_ &&
	    now_ns - last_submit_ns_ < 100'000'000)
		return;
	vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id, pose, sizeof pose);
	if (connected && sample != last_sample_ns_ && now_ns >= sample) {
		// the part of the age that passed after the daemon published: the hand-off to here (a
		// publish stamped just after this pass's "now" is a hand-off of nothing, not no hand-off)
		const uint64_t published = st->publish_host_ns;
		const uint64_t handoff = published == 0 ? UINT64_MAX : now_ns >= published ? now_ns - published : 0;
		record_age(now_ns - sample, handoff, now_ns);
	}
	if (connected != last_connected_)
		cv1t_log("cv1touch: %s %s", serial(), connected ? "connected" : "disconnected (standby, out of range or no daemon)");
	if (gap && !last_gap_)
		cv1t_log("cv1touch: %s radio silent: held as it was", serial());
	else if (!gap && last_gap_ && connected)
		cv1t_log("cv1touch: %s radio back", serial());
	last_sample_ns_ = sample;
	last_connected_ = connected;
	last_gap_ = gap;
	last_submit_ns_ = now_ns;
	send_inputs(map_inputs(st, hand_, now_ns), pose.poseTimeOffset);
	if (st && connected && st->battery_percent >= 0 && st->battery_percent != battery_) {
		auto props = vr::VRProperties();
		if (battery_ < 0)
			props->SetBoolProperty(container_, vr::Prop_DeviceProvidesBatteryStatus_Bool, true);
		battery_ = st->battery_percent;
		props->SetFloatProperty(container_, vr::Prop_DeviceBatteryPercentage_Float, battery_ / 100.0f);
		props->SetBoolProperty(container_, vr::Prop_DeviceIsCharging_Bool, false);
	}
}

} // namespace cv1t
