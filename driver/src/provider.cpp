// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include "provider.h"

#include <chrono>

#include "log.h"
#include "stations.h"

namespace cv1t {

vr::EVRInitError Provider::Init(vr::IVRDriverContext *ctx)
{
	VR_INIT_SERVER_DRIVER_CONTEXT(ctx);
	cv1t_log_ready();
	for (int h = 0; h < 2; h++) {
		devices_[h] = std::make_unique<TouchDevice>(h);
		vr::VRServerDriverHost()->TrackedDeviceAdded(devices_[h]->serial(), vr::TrackedDeviceClass_Controller, devices_[h].get());
	}
	haptics_.start();
	running_ = true;
	pose_thread_ = std::thread([this] {
		try {
			pose_loop();
		} catch (...) {
			cv1t_log("cv1touch: pose thread stopped by an exception");
		}
	});
	station_thread_ = std::thread([this] {
		try {
			station_loop();
		} catch (...) {
			cv1t_log("cv1touch: station thread stopped by an exception");
		}
	});
	cv1t_log("cv1touch: driver up (shm %s, ABI v%u)", CV1T_SHM_NAME, CV1T_SHM_VERSION);
	return vr::VRInitError_None;
}

Provider::~Provider()
{
	running_ = false;
	if (pose_thread_.joinable())
		pose_thread_.join();
	if (station_thread_.joinable())
		station_thread_.join();
	haptics_.stop();
}

void Provider::Cleanup()
{
	running_ = false;
	if (station_thread_.joinable())
		station_thread_.join();
	if (pose_thread_.joinable())
		pose_thread_.join();
	haptics_.stop();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

void Provider::RunFrame()
{
	vr::VREvent_t ev;
	while (vr::VRServerDriverHost()->PollNextEvent(&ev, sizeof ev)) {
		if (ev.eventType != vr::VREvent_Input_HapticVibration)
			continue;
		const auto &hv = ev.data.hapticVibration;
		for (int h = 0; h < 2; h++)
			if (devices_[h] && devices_[h]->haptic_component() == hv.componentHandle)
				haptics_.submit(h, hv.fDurationSeconds, hv.fFrequency, hv.fAmplitude);
	}
}

/* Every 2 s, the Lighthouse base stations' raw poses to the daemon, which follows the universe with
 * them: read here inside vrserver, so the daemon needs no SteamVR connection of its own (each one
 * costs vrserver an app connect, binding load and disconnect). */
void Provider::station_loop()
{
	ControlClient control;
	while (running_) {
		vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
		vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, poses, vr::k_unMaxTrackedDeviceCount);
		auto *props = vr::VRProperties();
		std::vector<StationPose> stations;
		for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
			if (!poses[i].bPoseIsValid)
				continue;
			const vr::PropertyContainerHandle_t c = props->TrackedDeviceToPropertyContainer(i);
			if (props->GetInt32Property(c, vr::Prop_DeviceClass_Int32) != vr::TrackedDeviceClass_TrackingReference ||
			    props->GetStringProperty(c, vr::Prop_TrackingSystemName_String) != "lighthouse")
				continue;
			stations.push_back(station_pose(props->GetStringProperty(c, vr::Prop_SerialNumber_String), poses[i].mDeviceToAbsoluteTracking));
		}
		if (!stations.empty())
			control.send(stations_line(stations));
		for (int k = 0; k < 20 && running_; k++)
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

void Provider::pose_loop()
{
	uint32_t seen = 0;
	while (running_) {
		uint64_t now = cv1t_now_ns();
		if (!shm_.ensure_open(now)) {
			for (auto &d : devices_)
				d->update(nullptr, nullptr, now);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}
		seen = shm_.wait(seen, 20'000'000);
		now = cv1t_now_ns();
		cv1t_alignment al{};
		const bool have_al = shm_.alignment(&al);
		const bool alive = shm_.alive(now);
		for (int h = 0; h < 2; h++) {
			cv1t_controller_state st{};
			const bool ok = alive && shm_.hand(h, &st);
			devices_[h]->update(ok ? &st : nullptr, have_al ? &al : nullptr, now);
		}
	}
}

} // namespace cv1t
