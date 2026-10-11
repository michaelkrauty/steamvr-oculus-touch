// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#pragma once
#include <condition_variable>
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

/* Sends haptic commands to the daemon's control socket from a worker thread. The newest command
 * per hand wins, and submit() never blocks the caller. */
class HapticSender {
public:
	explicit HapticSender(std::string socket_path = default_socket_path());
	~HapticSender();
	void start();
	void stop();
	void submit(int hand, float duration_s, float frequency_hz, float amplitude);
	/* Takes the pending command for a hand (tests; the worker uses it too). */
	bool take(int hand, HapticCommand *out);
	static std::string default_socket_path();

private:
	void run();
	std::string path_;
	std::mutex mu_;
	std::condition_variable cv_;
	bool pending_[2] = {false, false};
	HapticCommand cmd_[2]{};
	bool running_ = false;
	std::thread worker_;
	uint64_t last_warn_ns_ = 0;
};

} // namespace cv1t
