// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include "provider.h"

#include <chrono>

#include "log.h"
#include "stations.h"

namespace cv1t {

/* SteamVR's base-station setting and cv1touch's records of it (PinKeys), through SteamVR's own
 * settings interface: SteamVR saves a change at once, and its Lighthouse driver reads the setting
 * when SteamVR starts. */
static const char *const kPinKeys[3][2] = {
	{"driver_lighthouse", "primarybasestation"}, {"driver_cv1touch", "pinned_base"}, {"driver_cv1touch", "unpinned_base"}};

static PinKeys read_pin()
{
	int32_t v[3];
	for (int i = 0; i < 3; i++) {
		vr::EVRSettingsError e = vr::VRSettingsError_None;
		v[i] = vr::VRSettings()->GetInt32(kPinKeys[i][0], kPinKeys[i][1], &e);
		if (e != vr::VRSettingsError_None)
			v[i] = 0;
	}
	return {v[0], v[1], v[2]};
}

static void write_pin(const PinKeys &from, const PinKeys &to)
{
	const int32_t a[3] = {from.pin, from.ours, from.unpinned}, b[3] = {to.pin, to.ours, to.unpinned};
	for (int i = 0; i < 3; i++) {
		if (a[i] == b[i])
			continue;
		if (b[i])
			vr::VRSettings()->SetInt32(kPinKeys[i][0], kPinKeys[i][1], b[i]);
		else
			vr::VRSettings()->RemoveKeyInSection(kPinKeys[i][0], kPinKeys[i][1]);
	}
	if (to.pin != from.pin && to.pin)
		cv1t_log("cv1touch: base-station pin: %s.%s set to LHB-%08X for SteamVR's next start", kPinKeys[0][0], kPinKeys[0][1], static_cast<unsigned>(to.pin));
	else if (to.pin != from.pin)
		cv1t_log("cv1touch: base-station pin: %s.%s removed for SteamVR's next start", kPinKeys[0][0], kPinKeys[0][1]);
}

/* Reports the pin (the last report when `quit`) and writes what the daemon answers, only if no
 * one changed it since it was read. */
static void exchange_pin(ControlClient &control, bool quit)
{
	const PinKeys was = read_pin();
	std::string reply;
	PinKeys want;
	if (control.send(pin_line(was, quit), &reply) && pin_reply(reply, &want) && want != was && read_pin() == was)
		write_pin(was, want);
}

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
	// the last base-station pin report, so that a pin SteamVR is still waiting for goes with
	// this session (src/base_pin.rs, at_quit)
	ControlClient control;
	exchange_pin(control, true);
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

/* The Lighthouse base stations' raw poses to the daemon, which follows the universe with them:
 * read here inside vrserver, so the daemon needs no SteamVR connection of its own (each one costs
 * vrserver an app connect, binding load and disconnect). Looked at every 100 ms and sent when
 * they change, so a move SteamVR makes is followed at once, and every 2 s besides
 * (StationReports). SteamVR's headset likewise, for the daemon to tell whether it is the one the
 * controllers were lined up with. And every 2 s, the base-station pin (PinKeys). */
void Provider::station_loop()
{
	ControlClient control;
	StationReports reports;
	uint64_t pin_ns = 0;
	std::string headset_sent;
	uint64_t headset_ns = 0;
	bool headset_taken = false;
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
		const uint64_t now = cv1t_now_ns();
		// SteamVR's headset, always device 0; before the stations, whose report wakes the daemon's
		// universe watch, so that the watch knows the headset by then
		const vr::PropertyContainerHandle_t hmd = props->TrackedDeviceToPropertyContainer(vr::k_unTrackedDeviceIndex_Hmd);
		if (props->GetInt32Property(hmd, vr::Prop_DeviceClass_Int32) == vr::TrackedDeviceClass_HMD) {
			const std::string line = headset_line(props->GetStringProperty(hmd, vr::Prop_SerialNumber_String),
			                                      props->GetStringProperty(hmd, vr::Prop_TrackingSystemName_String),
			                                      props->GetUint64Property(hmd, vr::Prop_CurrentUniverseId_Uint64), !headset_taken);
			if (!line.empty() && (line != headset_sent || now - headset_ns >= StationReports::KEEP_ALIVE_NS)) {
				headset_sent = line;
				headset_ns = now;
				headset_taken = control.send(line) || headset_taken;
			}
		}
		if (!stations.empty() && reports.due(stations, now))
			control.send(stations_line(stations));
		// the base-station pin, every 2 s
		if (now - pin_ns >= StationReports::KEEP_ALIVE_NS) {
			pin_ns = now;
			exchange_pin(control, false);
		}
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
