// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include "haptics.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

#include "cv1touch_shm.h"
#include "log.h"

namespace cv1t {

HapticCommand clamp_haptic(int hand, float d, float f, float a)
{
	HapticCommand c{hand, d, f, a};
	if (!std::isfinite(c.duration_s) || c.duration_s <= 0.0f)
		c.duration_s = 0.01f;
	if (c.duration_s < 0.001f)
		c.duration_s = 0.001f; // the command prints 4 decimals: shorter would read as 0
	if (c.duration_s > 10.0f)
		c.duration_s = 10.0f;
	if (!std::isfinite(c.frequency_hz) || c.frequency_hz <= 0.0f)
		c.frequency_hz = 320.0f;
	if (c.frequency_hz < 1.0f)
		c.frequency_hz = 1.0f; // printed with 1 decimal
	if (c.frequency_hz > 1000.0f)
		c.frequency_hz = 1000.0f;
	if (!std::isfinite(c.amplitude) || c.amplitude < 0.0f)
		c.amplitude = 0.0f;
	if (c.amplitude > 1.0f)
		c.amplitude = 1.0f;
	return c;
}

std::string haptic_line(const HapticCommand &c)
{
	char buf[96];
	snprintf(buf, sizeof buf, "haptic %s %.4f %.1f %.3f\n", c.hand == CV1T_HAND_LEFT ? "left" : "right", c.duration_s, c.frequency_hz, c.amplitude);
	return buf;
}

std::string HapticSender::default_socket_path()
{
	const char *rt = std::getenv("XDG_RUNTIME_DIR");
	std::string dir = rt && *rt ? rt : "/run/user/" + std::to_string(getuid());
	return dir + "/cv1touch/control.sock";
}

HapticSender::HapticSender(std::string socket_path) : path_(std::move(socket_path)) {}

HapticSender::~HapticSender() { stop(); }

void HapticSender::start()
{
	std::lock_guard<std::mutex> g(mu_);
	if (running_)
		return;
	running_ = true;
	worker_ = std::thread([this] {
		try {
			run();
		} catch (...) {
			cv1t_log("cv1touch: haptic worker stopped by an exception");
		}
	});
}

void HapticSender::stop()
{
	{
		std::lock_guard<std::mutex> g(mu_);
		running_ = false;
	}
	cv_.notify_all();
	if (worker_.joinable())
		worker_.join();
}

void HapticSender::submit(int hand, float d, float f, float a)
{
	if (hand < 0 || hand > 1)
		return;
	{
		std::lock_guard<std::mutex> g(mu_);
		cmd_[hand] = clamp_haptic(hand, d, f, a);
		pending_[hand] = true;
	}
	cv_.notify_one();
}

bool HapticSender::take(int hand, HapticCommand *out)
{
	std::lock_guard<std::mutex> g(mu_);
	if (!pending_[hand])
		return false;
	pending_[hand] = false;
	*out = cmd_[hand];
	return true;
}

/* One request, one reply line; 100 ms timeouts so a stuck daemon cannot pile commands up. */
bool send_control_line(const std::string &path, const std::string &line)
{
	const int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
	if (fd < 0)
		return false;
	timeval tv{0, 100'000};
	setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
	setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
	sockaddr_un addr{};
	addr.sun_family = AF_UNIX;
	std::strncpy(addr.sun_path, path.c_str(), sizeof addr.sun_path - 1);
	bool ok = connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0 &&
		  send(fd, line.data(), line.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(line.size());
	if (ok) {
		char reply[256];
		const ssize_t n = read(fd, reply, sizeof reply - 1);
		ok = n >= 2 && std::strncmp(reply, "ok", 2) == 0;
	}
	close(fd);
	return ok;
}

void HapticSender::run()
{
	for (;;) {
		{
			std::unique_lock<std::mutex> lk(mu_);
			cv_.wait(lk, [this] { return !running_ || pending_[0] || pending_[1]; });
			if (!running_)
				return;
		}
		for (int h = 0; h < 2; h++) {
			HapticCommand c{};
			if (!take(h, &c))
				continue;
			if (!send_control_line(path_, haptic_line(c))) {
				const uint64_t now = cv1t_now_ns();
				if (now - last_warn_ns_ > 60'000'000'000ull) {
					cv1t_log("cv1touch: haptic command not delivered to %s (daemon not running?)", path_.c_str());
					last_warn_ns_ = now;
				}
			}
		}
	}
}

} // namespace cv1t
