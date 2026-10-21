// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include "control_test.h"
#include "haptics.h"
#include "socket_dir.h"
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <future>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

static int listener;
static uint32_t expected_ms;
static std::promise<int> *connected;
static std::string expected_path;

/* Check the requested values exactly: kernels can round the installed timeout to a tick. */
extern "C" int __real_setsockopt(int, int, int, const void *, socklen_t);
extern "C" int __wrap_setsockopt(int fd, int level, int option, const void *value, socklen_t length)
{
	if (level == SOL_SOCKET && (option == SO_SNDTIMEO || option == SO_RCVTIMEO)) {
		assert(length == sizeof(timeval));
		const auto &timeout = *static_cast<const timeval *>(value);
		assert(timeout.tv_sec * 1000 + timeout.tv_usec / 1000 == expected_ms);
	}
	return __real_setsockopt(fd, level, option, value, length);
}

/* Inspect the actual socket options, then queue the reply before the client's send/read.
 * The real connect and exchange still run; no peer thread has to win a short deadline. */
extern "C" int __real_connect(int, const sockaddr *, socklen_t);
extern "C" int __wrap_connect(int fd, const sockaddr *address, socklen_t length)
{
	assert(address->sa_family == AF_UNIX);
	assert(std::string(reinterpret_cast<const sockaddr_un *>(address)->sun_path) == expected_path);
	assert(connected != nullptr); // a default-path regression must never reach a live daemon
	const int result = __real_connect(fd, address, length);
	assert(result == 0);
	for (int option : {SO_SNDTIMEO, SO_RCVTIMEO}) {
		timeval timeout{};
		socklen_t size = sizeof timeout;
		assert(getsockopt(fd, SOL_SOCKET, option, &timeout, &size) == 0);
		const auto installed = timeout.tv_sec * 1000 + timeout.tv_usec / 1000;
		assert(installed >= expected_ms && installed < expected_ms + 100);
	}
	const int peer = accept(listener, nullptr, nullptr);
	assert(peer >= 0 && send(peer, "ok\n", 3, MSG_NOSIGNAL) == 3);
	connected->set_value(peer);
	return result;
}

static void received(int peer, const std::string &expected)
{
	char bytes[128]{};
	const auto count = read(peer, bytes, sizeof bytes);
	assert(count > 0 && std::string(bytes, count) == expected);
	close(peer);
}

template <class Send> static void direct(uint32_t timeout_ms, Send send)
{
	std::promise<int> ready;
	connected = &ready;
	expected_ms = timeout_ms;
	assert(send());
	received(ready.get_future().get(), "status\n");
	connected = nullptr;
}

static void worker(cv1t::HapticSender &sender, uint32_t timeout_ms)
{
	std::promise<int> ready;
	connected = &ready;
	expected_ms = timeout_ms;
	sender.start();
	sender.submit(0, 0.01f, 320.0f, 0.5f);
	received(ready.get_future().get(), "haptic left 0.0100 320.0 0.500\n");
	sender.stop();
	connected = nullptr;
}

int main()
{
	SocketDir fixture;
	const std::string directory = fixture.path() + "/cv1touch";
	assert(mkdir(directory.c_str(), 0700) == 0);
	const std::string path = directory + "/control.sock";
	expected_path = path;
	assert(setenv("XDG_RUNTIME_DIR", fixture.path().c_str(), 1) == 0);
	assert(setenv("CV1T_TEST_CONTROL_SHM_DIR", fixture.path().c_str(), 1) == 0);
	assert(cv1t::HapticSender::default_socket_path() == path);
	listener = socket(AF_UNIX, SOCK_STREAM, 0);
	assert(listener >= 0);
	sockaddr_un address{};
	address.sun_family = AF_UNIX;
	assert(path.size() < sizeof address.sun_path);
	std::strcpy(address.sun_path, path.c_str());
	assert(bind(listener, reinterpret_cast<sockaddr *>(&address), sizeof address) == 0);
	assert(listen(listener, 4) == 0);

	cv1t::ControlClient defaults;
	direct(100, [&] { return defaults.send("status\n"); });
	cv1t::ControlClient explicit_path(path);
	direct(100, [&] { return explicit_path.send("status\n"); });
	cv1t::ControlClient fixture_default(kControlTestTimeoutMs);
	direct(5000, [&] { return fixture_default.send("status\n"); });
	cv1t::ControlClient fixture_path(path, {}, kControlTestTimeoutMs);
	direct(5000, [&] { return fixture_path.send("status\n"); });
	direct(100, [&] { return cv1t::send_control_line(path, "status\n"); });
	direct(5000, [&] { return cv1t::send_control_line(path, "status\n", kControlTestTimeoutMs); });

	cv1t::HapticSender default_worker;
	worker(default_worker, 100);
	cv1t::HapticSender explicit_worker(path);
	worker(explicit_worker, 100);
	cv1t::HapticSender fixture_worker(kControlTestTimeoutMs);
	worker(fixture_worker, 5000);
	cv1t::HapticSender fixture_explicit_worker(path, {}, kControlTestTimeoutMs);
	worker(fixture_explicit_worker, 5000);
	assert(!cv1t::ControlClient(path, {}, 0).send("status\n"));
	assert(!cv1t::send_control_line(path, "status\n", 0));
	close(listener);
	std::puts("test_control_timeout ok");
}
