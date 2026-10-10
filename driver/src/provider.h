// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

#pragma once
#include <atomic>
#include <memory>
#include <thread>

#include <openvr_driver.h>

#include "haptics.h"
#include "shm_client.h"
#include "touch_device.h"

namespace cv1t {

class Provider : public vr::IServerTrackedDeviceProvider {
public:
	/* vrserver may exit without Cleanup(): a joinable thread in a static would std::terminate. */
	~Provider();
	vr::EVRInitError Init(vr::IVRDriverContext *ctx) override;
	void Cleanup() override;
	const char *const *GetInterfaceVersions() override { return vr::k_InterfaceVersions; }
	void RunFrame() override;
	bool ShouldBlockStandbyMode() override { return false; }
	void EnterStandby() override {}
	void LeaveStandby() override {}

private:
	void pose_loop();
	void station_loop();
	std::unique_ptr<TouchDevice> devices_[2];
	ShmClient shm_;
	HapticSender haptics_;
	std::atomic<bool> running_{false};
	std::thread pose_thread_;
	std::thread station_thread_;
};

} // namespace cv1t
