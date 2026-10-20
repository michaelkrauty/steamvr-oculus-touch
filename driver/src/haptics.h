// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#pragma once
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace cv1t {

struct HapticCommand {
	int hand;
	float duration_s, frequency_hz, amplitude;
};

/* Into the daemon's accepted ranges: duration (0, 10] s (0 or invalid: a 10 ms pulse), frequency
 * (0, 1000] Hz (0 or invalid: 320), amplitude [0, 1] (invalid: 0). */
HapticCommand clamp_haptic(int hand, float duration_s, float frequency_hz, float amplitude);
/* "haptic left|right <s> <hz> <amp>\n" */
std::string haptic_line(const HapticCommand &c);

/* One request line to the daemon's control socket, one reply line back ("ok..." is success);
 * 100 ms timeouts, so a stuck daemon cannot hold the caller. */
bool send_control_line(const std::string &path, const std::string &line);
#ifndef _WIN32
/* The same UID-scoped path and private-directory policy as src/platform/unix.rs. */
std::string container_socket_path(uint32_t uid);
bool control_directory_is_private(const std::string &dir, uint32_t uid);
#endif

/* One client per sending thread. Default clients remember the reachable endpoint; an explicit
 * single path never falls through to another daemon. */
class ControlClient {
public:
	ControlClient();
	explicit ControlClient(std::string path, std::string fallback = {});
	bool send(const std::string &line);
	const std::string &path() const { return paths_[last_]; }
#ifndef _WIN32
	/* The last send rejected an overlong address and already logged its cause. */
	bool path_too_long() const { return path_too_long_; }
#endif

private:
	std::string paths_[2];
	unsigned last_ = 0;
#ifndef _WIN32
	unsigned warned_long_paths_ = 0;
	bool path_too_long_ = false;
#endif
};
#ifdef _WIN32
/* The daemon's named pipe for a control endpoint path: \\.\pipe\cv1touch- and the path with "\"
 * as "/" (the daemon's platform::pipe_name); empty past the 256-character limit. */
std::wstring control_pipe_name(const std::string &path);
#endif

/* Sends haptic commands to the daemon's control socket from a worker thread. The newest command
 * per hand wins, and submit() never blocks the caller. */
class HapticSender {
public:
	HapticSender();
	explicit HapticSender(std::string socket_path, std::string fallback = {});
	~HapticSender();
	void start();
	void stop();
	void submit(int hand, float duration_s, float frequency_hz, float amplitude);
	/* Takes the pending command for a hand (tests; the worker uses it too). */
	bool take(int hand, HapticCommand *out);
	/* Claims a delivery-failure warning (worker thread only; tests supply the timestamp). */
	bool take_warning(uint64_t now_ns);
	static std::string default_socket_path();

private:
	void run();
	ControlClient control_;
	std::mutex mu_;
	std::condition_variable cv_;
	bool pending_[2] = {false, false};
	HapticCommand cmd_[2]{};
	bool running_ = false;
	std::thread worker_;
	uint64_t last_warn_ns_ = 0;
	bool warned_ = false;
};

} // namespace cv1t
