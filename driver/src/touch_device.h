// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

#include <openvr_driver.h>

#include "cv1touch_shm.h"
#include "inputs.h"

namespace cv1t {

class TouchDevice : public vr::ITrackedDeviceServerDriver {
public:
	explicit TouchDevice(int hand) : hand_(hand) {}
	const char *serial() const { return hand_ == CV1T_HAND_LEFT ? "CV1T-LEFT" : "CV1T-RIGHT"; }

	vr::EVRInitError Activate(uint32_t object_id) override;
	void Deactivate() override { object_id_ = vr::k_unTrackedDeviceIndexInvalid; }
	void EnterStandby() override {}
	void *GetComponent(const char *) override { return nullptr; }
	void DebugRequest(const char *, char *buf, uint32_t size) override
	{
		if (size)
			buf[0] = 0;
	}
	vr::DriverPose_t GetPose() override;

	/* Pose thread: the newest published state (nullptr: none) and alignment (nullptr: none). */
	void update(const cv1t_controller_state *st, const cv1t_alignment *al, uint64_t now_ns);
	vr::VRInputComponentHandle_t haptic_component() const { return haptic_; }

private:
	struct Handles {
		vr::VRInputComponentHandle_t a_click, a_touch, b_click, b_touch, system_click, joy_click, joy_touch, joy_x, joy_y, trigger,
			trigger_touch, grip, grip_touch, thumbrest_touch;
	};
	void send_inputs(const InputValues &v, double time_offset);
	void record_age(uint64_t age_ns, uint64_t now_ns);

	int hand_;
	std::atomic<uint32_t> object_id_{vr::k_unTrackedDeviceIndexInvalid};
	vr::PropertyContainerHandle_t container_ = 0;
	Handles h_{};
	std::atomic<vr::VRInputComponentHandle_t> haptic_{vr::k_ulInvalidInputComponentHandle};
	std::mutex pose_mu_;
	vr::DriverPose_t last_pose_{};
	InputValues last_inputs_{};
	bool inputs_sent_ = false;
	uint64_t last_sample_ns_ = 0, last_submit_ns_ = 0, universe_ = 0;
	bool last_connected_ = false;
	int battery_ = -1;
	std::vector<uint64_t> ages_;
	uint64_t ages_since_ns_ = 0;
};

} // namespace cv1t
