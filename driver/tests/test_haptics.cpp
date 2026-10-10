// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "cv1touch_shm.h"
#include "haptics.h"

using namespace cv1t;

/* Review Focus 5: what apps send, into what the daemon accepts. */
static void clamps_what_apps_send()
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	auto c = clamp_haptic(1, 0.0f, 0.0f, 0.5f);
	assert(c.duration_s == 0.01f && c.frequency_hz == 320.0f && c.amplitude == 0.5f);
	c = clamp_haptic(0, 30.0f, 5000.0f, 1.7f);
	assert(c.duration_s == 10.0f && c.frequency_hz == 1000.0f && c.amplitude == 1.0f);
	c = clamp_haptic(0, nan, nan, nan);
	assert(c.duration_s == 0.01f && c.frequency_hz == 320.0f && c.amplitude == 0.0f);
	c = clamp_haptic(0, 1e-5f, 0.04f, 0.5f); // review fix 4: too small to print would read as 0
	assert(c.duration_s == 0.001f && c.frequency_hz == 1.0f);
	assert(haptic_line(c) == "haptic left 0.0010 1.0 0.500\n");
	c = clamp_haptic(0, 0.2f, 160.0f, -0.3f);
	assert(c.duration_s == 0.2f && c.frequency_hz == 160.0f && c.amplitude == 0.0f);
}

static void formats_the_daemon_command()
{
	assert(haptic_line({CV1T_HAND_RIGHT, 0.25f, 160.0f, 1.0f}) == "haptic right 0.2500 160.0 1.000\n");
	assert(haptic_line({CV1T_HAND_LEFT, 0.01f, 320.0f, 0.5f}) == "haptic left 0.0100 320.0 0.500\n");
}

static void newest_command_per_hand_wins()
{
	HapticSender s("/nonexistent/control.sock");
	s.submit(0, 0.1f, 160, 0.2f);
	s.submit(0, 0.2f, 160, 0.9f);
	s.submit(1, 0.3f, 320, 0.4f);
	HapticCommand c{};
	assert(s.take(0, &c) && c.amplitude == 0.9f && c.duration_s == 0.2f);
	assert(!s.take(0, &c));
	assert(s.take(1, &c) && c.hand == 1 && c.frequency_hz == 320.0f);
}

/* The worker delivers the line to a control socket and reads the reply. */
static void sends_to_the_socket()
{
	const std::string path = "/tmp/cv1t-hapt-" + std::to_string(getpid()) + ".sock";
	unlink(path.c_str());
	int srv = socket(AF_UNIX, SOCK_STREAM, 0);
	sockaddr_un addr{};
	addr.sun_family = AF_UNIX;
	std::strncpy(addr.sun_path, path.c_str(), sizeof addr.sun_path - 1);
	assert(bind(srv, reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0 && listen(srv, 4) == 0);
	HapticSender s(path);
	s.start();
	s.submit(CV1T_HAND_RIGHT, 0.25f, 160.0f, 1.0f);
	int conn = accept(srv, nullptr, nullptr);
	assert(conn >= 0);
	char buf[128] = {};
	ssize_t n = read(conn, buf, sizeof buf - 1);
	assert(n > 0 && std::string(buf) == "haptic right 0.2500 160.0 1.000\n");
	assert(write(conn, "ok\n", 3) == 3);
	close(conn);
	s.stop();
	close(srv);
	unlink(path.c_str());
}

int main()
{
	clamps_what_apps_send();
	formats_the_daemon_command();
	newest_command_per_hand_wins();
	sends_to_the_socket();
	printf("test_haptics ok\n");
	return 0;
}
