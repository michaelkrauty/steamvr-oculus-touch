// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
// Fake runtime only: no daemon, sockets or shared memory.
#include "touch_device.h"
#include "log.h"

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

namespace cv1t {
// Observe admission closure, not an elapsed delay or provider Cleanup. Deactivate
// releases this mutex in its drain wait, so the worker can still finish its pass.
struct TouchDeviceTestAccess {
	static bool await_closing(TouchDevice &device)
	{
		std::unique_lock<std::mutex> lock(device.submission_mu_);
		return device.submission_cv_.wait_for(lock, std::chrono::seconds(5), [&] {
			return device.object_id_ == vr::k_unTrackedDeviceIndexInvalid;
		}) && device.submissions_ == 1;
	}
};
} // namespace cv1t

namespace {
using namespace vr;
using namespace std::chrono_literals;

void require(bool ok, const char *what)
{
	if (!ok) {
		std::fprintf(stderr, "harness error: %s\n", what);
		std::abort();
	}
}

struct Gate {
	std::mutex mu;
	std::condition_variable cv;
	bool arrived = false, released = false;
	void block()
	{
		std::unique_lock<std::mutex> lock(mu);
		arrived = true;
		cv.notify_all();
		require(cv.wait_for(lock, 5s, [&] { return released; }), "gate release timeout");
	}
	void await()
	{
		std::unique_lock<std::mutex> lock(mu);
		require(cv.wait_for(lock, 5s, [&] { return arrived; }), "submission arrival timeout");
	}
	void release()
	{
		std::lock_guard<std::mutex> lock(mu);
		released = true;
		cv.notify_all();
	}
};

// Abort rather than hanging the suite if a callback/drain lock order regresses.
struct Watchdog {
	std::mutex mu;
	std::condition_variable cv;
	bool done = false;
	std::thread thread{[this] {
		std::unique_lock<std::mutex> lock(mu);
		require(cv.wait_for(lock, 10s, [&] { return done; }), "test completion timeout");
	}};
	~Watchdog()
	{
		{
			std::lock_guard<std::mutex> lock(mu);
			done = true;
			cv.notify_all();
		}
		thread.join();
	}
};

struct State {
	Gate gate;
	bool block_properties;
	std::atomic<bool> deactivated{false};
	std::atomic<unsigned> poses{0}, inputs{0}, properties{0}, logs{0};
	std::atomic<unsigned> late_poses{0}, late_inputs{0}, late_properties{0}, late_logs{0};
	bool throw_pose = false;
	ITrackedDeviceServerDriver *device = nullptr;
	void callback() { if (device) device->GetPose(); }
	explicit State(bool p) : block_properties(p) {}
};

struct Properties final : IVRProperties {
	State &s;
	explicit Properties(State &s) : s(s) {}
	ETrackedPropertyError ReadPropertyBatch(PropertyContainerHandle_t, PropertyRead_t *, uint32_t) override
	{ std::abort(); }
	ETrackedPropertyError WritePropertyBatch(PropertyContainerHandle_t container, PropertyWrite_t *batch, uint32_t n) override
	{
		require(container == 1007, "property container");
		s.callback();
		++s.properties;
		if (s.deactivated.load()) ++s.late_properties;
		for (uint32_t i = 0; i < n; ++i) {
			batch[i].eError = TrackedProp_Success;
			if (s.block_properties && batch[i].prop == Prop_CurrentUniverseId_Uint64)
				s.gate.block();
		}
		return TrackedProp_Success;
	}
	const char *GetPropErrorNameFromEnum(ETrackedPropertyError) override { return "fake"; }
	PropertyContainerHandle_t TrackedDeviceToPropertyContainer(TrackedDeviceIndex_t id) override
	{ require(id == 7, "property device id"); return 1007; }
};

struct Input final : IVRDriverInput {
	State &s;
	VRInputComponentHandle_t next = 1;
	explicit Input(State &s) : s(s) {}
	EVRInputError create(PropertyContainerHandle_t c, VRInputComponentHandle_t *h)
	{ require(c == 1007, "input container"); *h = next++; return VRInputError_None; }
	EVRInputError update(VRInputComponentHandle_t h)
	{
		require(h > 0 && h < next, "input handle");
		s.callback();
		++s.inputs;
		if (s.deactivated.load()) ++s.late_inputs;
		return VRInputError_None;
	}
	EVRInputError CreateBooleanComponent(PropertyContainerHandle_t c, const char *, VRInputComponentHandle_t *h) override
	{ return create(c, h); }
	EVRInputError UpdateBooleanComponent(VRInputComponentHandle_t h, bool, double) override { return update(h); }
	EVRInputError CreateScalarComponent(PropertyContainerHandle_t c, const char *, VRInputComponentHandle_t *h, EVRScalarType, EVRScalarUnits) override
	{ return create(c, h); }
	EVRInputError UpdateScalarComponent(VRInputComponentHandle_t h, float, double) override { return update(h); }
	EVRInputError CreateHapticComponent(PropertyContainerHandle_t c, const char *, VRInputComponentHandle_t *h) override
	{ return create(c, h); }
	EVRInputError CreateSkeletonComponent(PropertyContainerHandle_t, const char *, const char *, const char *, EVRSkeletalTrackingLevel, const VRBoneTransform_t *, uint32_t, VRInputComponentHandle_t *) override
	{ std::abort(); }
	EVRInputError UpdateSkeletonComponent(VRInputComponentHandle_t, EVRSkeletalMotionRange, const VRBoneTransform_t *, uint32_t) override
	{ std::abort(); }
	EVRInputError CreatePoseComponent(PropertyContainerHandle_t, const char *, VRInputComponentHandle_t *) override { std::abort(); }
	EVRInputError UpdatePoseComponent(VRInputComponentHandle_t, const HmdMatrix34_t *, double) override { std::abort(); }
	EVRInputError CreateEyeTrackingComponent(PropertyContainerHandle_t, const char *, VRInputComponentHandle_t *) override { std::abort(); }
	EVRInputError UpdateEyeTrackingComponent(VRInputComponentHandle_t, const VREyeTrackingData_t *, double) override { std::abort(); }
};

struct SubmissionFailure {};

struct Host final : IVRServerDriverHost {
	State &s;
	explicit Host(State &s) : s(s) {}
	bool TrackedDeviceAdded(const char *, ETrackedDeviceClass c, ITrackedDeviceServerDriver *d) override
	{
		require(c == TrackedDeviceClass_Controller, "device class");
		// Synchronous activation is permitted; no shutdown/reactivation overlap is required.
		require(d->Activate(7) == VRInitError_None, "Activate");
		return true;
	}
	void TrackedDevicePoseUpdated(uint32_t id, const DriverPose_t &, uint32_t size) override
	{
		require(id == 7 && size == sizeof(DriverPose_t), "pose submission");
		s.callback();
		++s.poses;
		if (s.deactivated.load()) ++s.late_poses;
		if (s.throw_pose) throw SubmissionFailure{};
		if (!s.block_properties) s.gate.block();
	}
	void VsyncEvent(double) override { std::abort(); }
	void VendorSpecificEvent(uint32_t, EVREventType, const VREvent_Data_t &, double) override { std::abort(); }
	bool IsExiting() override { return false; }
	bool PollNextEvent(VREvent_t *, uint32_t) override { return false; }
	void GetRawTrackedDevicePoses(float, TrackedDevicePose_t *, uint32_t) override { std::abort(); }
	void RequestRestart(const char *, const char *, const char *, const char *) override { std::abort(); }
	uint32_t GetFrameTimings(Compositor_FrameTiming *, uint32_t) override { std::abort(); }
	void SetDisplayEyeToHead(uint32_t, const HmdMatrix34_t &, const HmdMatrix34_t &) override { std::abort(); }
	void SetDisplayProjectionRaw(uint32_t, const HmdRect2_t &, const HmdRect2_t &) override { std::abort(); }
	void SetRecommendedRenderTargetSize(uint32_t, uint32_t, uint32_t) override { std::abort(); }
};

struct DriverLog final : IVRDriverLog {
	State &s;
	explicit DriverLog(State &s) : s(s) {}
	void Log(const char *) override
	{
		s.callback();
		++s.logs;
		if (s.deactivated.load()) ++s.late_logs;
	}
};

struct Context final : IVRDriverContext {
	Properties properties;
	Input input;
	Host host;
	DriverLog log;
	explicit Context(State &s) : properties(s), input(s), host(s), log(s) {}
	void *GetGenericInterface(const char *version, EVRInitError *error) override
	{
		if (error) *error = VRInitError_None;
		if (!std::strcmp(version, IVRProperties_Version)) return &properties;
		if (!std::strcmp(version, IVRDriverInput_Version)) return &input;
		if (!std::strcmp(version, IVRServerDriverHost_Version)) return &host;
		if (!std::strcmp(version, IVRDriverLog_Version)) return &log;
		std::abort();
	}
	DriverHandle_t GetDriverHandle() override { return 2000; }
};

void run(bool block_properties)
{
	Watchdog watchdog;
	State s(block_properties);
	Context context(s);
	// Device-level test: install the SDK interfaces used by TouchDevice, including logging.
	// No full Provider::Init, because that opens the real daemon endpoints.
	vr::VRDriverContext() = &context;
	vr::OpenVRInternal_ModuleServerDriverContext().Clear();
	vr::VRDriverLog();
	cv1t_log_ready();
	vr::VRProperties();
	vr::VRDriverInput();
	vr::VRServerDriverHost();
	cv1t::TouchDevice device(CV1T_HAND_LEFT);
	require(context.host.TrackedDeviceAdded(device.serial(), TrackedDeviceClass_Controller, &device), "TrackedDeviceAdded");
	s.device = &device;
	s.properties = 0; // Do not count activation setup.
	constexpr uint64_t now = 1'000'000'000;
	cv1t_controller_state sample{};
	sample.flags = CV1T_FLAG_CONNECTED;
	sample.sample_host_ns = sample.publish_host_ns = now;
	sample.battery_percent = 50;
	sample.orient[3] = 1;
	cv1t_alignment alignment{};
	alignment.universe_id = 42;
	// Exactly one admitted worker, matching Provider::pose_loop; actual TouchDevice.
	Gate resume_worker;
	std::thread worker([&] {
		device.update(&sample, block_properties ? &alignment : nullptr, now);
		resume_worker.block();
		for (int i = 0; i < 32; ++i) {
			device.update(&sample, &alignment, now + 200'000'000 + i);
			device.update(nullptr, nullptr, now + 300'000'000 + i);
		}
	});
	s.gate.await(); // At this point update has already acquired a valid object_id_.
	std::thread lifecycle([&] {
		device.Deactivate();
		s.deactivated.store(true);
	});
	require(cv1t::TouchDeviceTestAccess::await_closing(device), "closed admission with a pass still in flight");
	require(!s.deactivated.load(), "Deactivate must drain the blocked pass");
	// Closing rejects even a new pass while the original worker remains blocked.
	// Only last_pose_ may change; the worker's input/property caches must not be touched.
	const unsigned closing_poses = s.poses, closing_inputs = s.inputs, closing_properties = s.properties, closing_logs = s.logs;
	device.update(nullptr, &alignment, now + 200'000'000);
	require(s.poses == closing_poses && s.inputs == closing_inputs && s.properties == closing_properties && s.logs == closing_logs,
		"update during closing must be rejected");
	s.gate.release(); // Always released before joins, including when Deactivate waits.
	lifecycle.join();
	// The actual pose worker continues after Deactivate returns, until the later
	// provider-style shutdown join. None of those later passes may enter the runtime.
	const unsigned poses = s.poses, inputs = s.inputs, properties = s.properties, logs = s.logs;
	resume_worker.release();
	worker.join();
	device.Deactivate(); // An already drained device remains closed.
	require(s.poses == poses && s.inputs == inputs && s.properties == properties && s.logs == logs, "fresh update after Deactivate must be suppressed");
	require(s.deactivated.load(), "deactivation completed");
	require(s.poses == 1 && s.inputs == 14 && s.properties == (block_properties ? 4u : 3u),
		"admitted pose/input/battery/universe pass completed");
	require(s.late_poses == 0 && s.late_inputs == 0 && s.late_properties == 0 && s.late_logs == 0,
		"no runtime calls may enter after Deactivate returns");
	std::printf("blocked=%s: drained; closing and post-return updates rejected\n", block_properties ? "property" : "host");
	vr::CleanupDriverContext(); // Context cleanup follows deactivation and worker join.
}

// Every admitted exit must release its ticket, including the keep-alive fast path
// and an exception (Provider catches exceptions outside pose_loop).
void run_idle_and_unwind()
{
	Watchdog watchdog;
	State s(false);
	Context context(s);
	vr::VRDriverContext() = &context;
	vr::OpenVRInternal_ModuleServerDriverContext().Clear();
	vr::VRDriverLog();
	cv1t_log_ready();
	cv1t::TouchDevice device(CV1T_HAND_LEFT);
	device.update(nullptr, nullptr, 1);
	require(s.poses == 0 && s.inputs == 0 && s.properties == 0, "inactive device is silent");
	require(context.host.TrackedDeviceAdded(device.serial(), TrackedDeviceClass_Controller, &device), "TrackedDeviceAdded");
	s.device = &device;
	s.properties = 0;
	s.gate.release();
	cv1t_controller_state sample{};
	constexpr uint64_t now = 1'000'000'000;
	sample.flags = CV1T_FLAG_CONNECTED;
	sample.sample_host_ns = sample.publish_host_ns = now;
	sample.orient[3] = 1;
	sample.battery_percent = 50;
	device.update(&sample, nullptr, now);
	device.update(&sample, nullptr, now + 1);
	require(s.poses == 1 && s.inputs == 14 && s.properties == 3, "unchanged sample stays throttled");
	s.throw_pose = true;
	bool caught = false;
	try {
		device.update(nullptr, nullptr, now + 2);
	} catch (const SubmissionFailure &) {
		caught = true;
	}
	require(caught, "runtime exception reached the worker boundary");
	device.Deactivate();
	s.deactivated = true;
	const unsigned logs = s.logs;
	device.update(&sample, nullptr, now + 200'000'000);
	require(s.poses == 2 && s.inputs == 14 && s.properties == 3 && s.logs == logs, "unwound and deactivated device is silent");
	vr::CleanupDriverContext();
	std::puts("inactive, throttled and exceptional exits: drained");
}
} // namespace

int main()
{
	run(false);
	run(true);
	run_idle_and_unwind();
}
