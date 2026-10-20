// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include "haptics.h"
#include "socket_dir.h"
#include <cassert>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

using cv1t::ControlClient;

struct Paths {
	SocketDir directory;
	std::string root, legacy, shared;
	Paths()
	{
		root = directory.path();
		legacy = root + "/legacy.sock";
		assert(mkdir((root + "/shared").c_str(), 0700) == 0);
		shared = root + "/shared/control.sock";
	}
};

struct Environment {
	std::string name, saved;
	bool existed;
	Environment(const char *key, const std::string &value) : name(key)
	{
		const char *old = std::getenv(key);
		existed = old != nullptr;
		saved = old ? old : "";
		assert(setenv(key, value.c_str(), 1) == 0);
	}
	~Environment()
	{
		if (existed)
			setenv(name.c_str(), saved.c_str(), 1);
		else
			unsetenv(name.c_str());
	}
};

struct Server {
	int fd;
	explicit Server(const std::string &path)
	{
		fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
		assert(fd >= 0);
		sockaddr_un addr{};
		addr.sun_family = AF_UNIX;
		assert(path.size() < sizeof addr.sun_path);
		std::strcpy(addr.sun_path, path.c_str());
		assert(bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0);
		assert(listen(fd, 4) == 0);
	}
	~Server() { close(fd); }
	bool pending() const
	{
		pollfd p{fd, POLLIN, 0};
		return poll(&p, 1, 0) > 0;
	}
	std::thread reply(const std::string &line = "ok\n", const std::string &request = "status\n") const
	{
		return std::thread([fd = fd, line, request] {
			pollfd p{fd, POLLIN, 0};
			assert(poll(&p, 1, 5000) == 1);
			const int conn = accept(fd, nullptr, nullptr);
			assert(conn >= 0);
			char buf[64]{};
			const auto n = read(conn, buf, sizeof buf);
			assert(n > 0 && std::string(buf, n) == request);
			if (!line.empty())
				assert(send(conn, line.data(), line.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(line.size()));
			close(conn);
		});
	}
};

static void resolves_the_runtime_and_shared_paths_for_the_same_user()
{
	const char *old = std::getenv("XDG_RUNTIME_DIR");
	const bool had_old = old != nullptr;
	const std::string saved = old ? old : "";
	setenv("XDG_RUNTIME_DIR", "/runtime/example", 1);
	assert(cv1t::HapticSender::default_socket_path() == "/runtime/example/cv1touch/control.sock");
	for (bool empty : {false, true}) {
		if (empty)
			setenv("XDG_RUNTIME_DIR", "", 1);
		else
			unsetenv("XDG_RUNTIME_DIR");
		assert(cv1t::HapticSender::default_socket_path() == "/run/user/" + std::to_string(getuid()) + "/cv1touch/control.sock");
	}
	if (had_old)
		setenv("XDG_RUNTIME_DIR", saved.c_str(), 1);
	else
		unsetenv("XDG_RUNTIME_DIR");
	assert(cv1t::container_socket_path(42) == "/dev/shm/cv1touch-control-42/control.sock");
	assert(cv1t::container_socket_path(43) == "/dev/shm/cv1touch-control-43/control.sock");
}

static void runtime_is_tried_first_and_an_explicit_path_stays_exact()
{
	Paths p;
	Server legacy(p.legacy), shared(p.shared);
	ControlClient c(p.legacy, p.shared);
	auto reply = legacy.reply();
	assert(c.send("status\n"));
	reply.join();
	assert(!shared.pending());
	assert(unlink(p.legacy.c_str()) == 0);
	assert(!ControlClient(p.legacy).send("status\n"));
	assert(!shared.pending());
}

static void a_missing_socket_falls_back_and_the_working_endpoint_is_remembered()
{
	Paths p;
	Server shared(p.shared);
	ControlClient c(p.legacy, p.shared);
	auto first = shared.reply();
	assert(c.send("status\n"));
	first.join();
	Server legacy(p.legacy);
	auto second = shared.reply();
	assert(c.send("status\n"));
	second.join();
	assert(!legacy.pending());
	assert(unlink(p.shared.c_str()) == 0);
	auto third = legacy.reply();
	assert(c.send("status\n"));
	third.join();
	assert(!shared.pending());
}

static void a_refused_socket_falls_back()
{
	Paths p;
	{ Server stale(p.legacy); }
	Server shared(p.shared);
	ControlClient c(p.legacy, p.shared);
	auto reply = shared.reply();
	assert(c.send("status\n"));
	reply.join();
}

static void a_missing_cached_directory_falls_back_but_an_unsafe_one_does_not()
{
	for (bool missing : {true, false}) {
		Paths p;
		Server shared(p.shared);
		ControlClient c(p.legacy, p.shared);
		auto first = shared.reply();
		assert(c.send("status\n"));
		first.join();
		Server legacy(p.legacy);
		if (missing) {
			assert(unlink(p.shared.c_str()) == 0);
			assert(rmdir((p.root + "/shared").c_str()) == 0);
			auto reply = legacy.reply();
			assert(c.send("status\n"));
			reply.join();
		} else {
			assert(chmod((p.root + "/shared").c_str(), 0755) == 0);
			errno = ENOENT;
			assert(!c.send("status\n"));
			assert(!legacy.pending() && !shared.pending());
			assert(chmod((p.root + "/shared").c_str(), 0700) == 0);
		}
	}
}

static void other_connection_errors_do_not_fall_back()
{
	Paths p;
	Server shared(p.shared);
	FILE *file = fopen(p.legacy.c_str(), "w");
	assert(file && fclose(file) == 0);
	assert(!ControlClient(p.legacy + "/socket", p.shared).send("status\n")); // ENOTDIR
	assert(!shared.pending());
	assert(!ControlClient(p.root + "/" + std::string(200, 'x'), p.shared).send("status\n")); // too long
	assert(!shared.pending());
	if (geteuid() != 0) {
		assert(mkdir((p.root + "/denied").c_str(), 0700) == 0);
		assert(chmod((p.root + "/denied").c_str(), 0000) == 0);
		assert(!ControlClient(p.root + "/denied/socket", p.shared).send("status\n")); // EACCES
		assert(!shared.pending());
		assert(chmod((p.root + "/denied").c_str(), 0700) == 0);
	}
}

static void a_failed_acknowledgement_does_not_resend_the_request()
{
	for (const std::string line : {"error rejected\n", "o", ""}) {
		Paths p;
		Server legacy(p.legacy), shared(p.shared);
		ControlClient c(p.legacy, p.shared);
		auto reply = legacy.reply(line);
		assert(!c.send("status\n"));
		reply.join();
		assert(!shared.pending());
	}
}

static void the_shared_socket_requires_a_private_real_directory()
{
	Paths p;
	Server shared(p.shared);
	assert(cv1t::control_directory_is_private(p.root + "/shared", geteuid()));
	assert(!cv1t::control_directory_is_private(p.root + "/shared", geteuid() + 1));
	for (mode_t mode : {0755, 0770, 01700}) {
		assert(chmod((p.root + "/shared").c_str(), mode) == 0);
		assert(!ControlClient(p.legacy, p.shared).send("status\n"));
		assert(!shared.pending());
	}
	assert(chmod((p.root + "/shared").c_str(), 0700) == 0);
	assert(rename((p.root + "/shared").c_str(), (p.root + "/real").c_str()) == 0);
	assert(symlink((p.root + "/real").c_str(), (p.root + "/shared").c_str()) == 0);
	assert(!ControlClient(p.legacy, p.shared).send("status\n"));
	assert(!shared.pending());
}

static void the_haptic_worker_uses_and_remembers_the_same_fallback()
{
	Paths p;
	Server shared(p.shared);
	cv1t::HapticSender haptics(p.legacy, p.shared);
	haptics.start();
	auto first = shared.reply("ok\n", "haptic right 0.2500 160.0 1.000\n");
	haptics.submit(1, 0.25f, 160.0f, 1.0f);
	first.join();
	Server legacy(p.legacy);
	auto second = shared.reply("ok\n", "haptic left 0.0100 320.0 0.500\n");
	haptics.submit(0, 0.01f, 320.0f, 0.5f);
	second.join();
	haptics.stop();
	assert(!legacy.pending());
}

static void default_constructors_reach_the_shared_socket(bool worker)
{
	Paths p;
	Environment runtime("XDG_RUNTIME_DIR", p.root + "/missing-runtime");
	Environment memory("CV1T_TEST_CONTROL_SHM_DIR", p.root);
	const std::string dir = p.root + "/cv1touch-control-" + std::to_string(geteuid());
	const std::string path = dir + "/control.sock";
	assert(mkdir(dir.c_str(), 0700) == 0);
	// Guard the fixture seam before a default constructor could contact the live daemon.
	assert(cv1t::container_socket_path(geteuid()) == path);
	assert(cv1t::HapticSender::default_socket_path() == p.root + "/missing-runtime/cv1touch/control.sock");
	Server shared(path);
	if (worker) {
		cv1t::HapticSender haptics;
		haptics.start();
		auto reply = shared.reply("ok\n", "haptic right 0.2500 160.0 1.000\n");
		haptics.submit(1, 0.25f, 160.0f, 1.0f);
		reply.join();
		haptics.stop();
	} else {
		ControlClient client;
		auto reply = shared.reply();
		assert(client.send("status\n"));
		reply.join();
		assert(client.path() == path);
	}
}

int main(int argc, char **argv)
{
	if (argc == 2) {
		assert(std::string(argv[1]) == "--default-client" || std::string(argv[1]) == "--default-worker");
		default_constructors_reach_the_shared_socket(std::string(argv[1]) == "--default-worker");
		return 0;
	}
	assert(argc == 1);
	resolves_the_runtime_and_shared_paths_for_the_same_user();
	runtime_is_tried_first_and_an_explicit_path_stays_exact();
	a_missing_socket_falls_back_and_the_working_endpoint_is_remembered();
	a_refused_socket_falls_back();
	a_missing_cached_directory_falls_back_but_an_unsafe_one_does_not();
	other_connection_errors_do_not_fall_back();
	a_failed_acknowledgement_does_not_resend_the_request();
	the_shared_socket_requires_a_private_real_directory();
	the_haptic_worker_uses_and_remembers_the_same_fallback();
	default_constructors_reach_the_shared_socket(false);
	default_constructors_reach_the_shared_socket(true);
	std::puts("test_control_client ok");
}
