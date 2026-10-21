// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* Run by examples/driver_contract.rs, which plays cv1touchd with the daemon's own Rust code: the
 * driver must find the shared memory and the control endpoint by the same names, read what was
 * published, run on the same clock, and wake on each publish. */
#include <cassert>
#include <cstdio>
#include <future>
#include <thread>
#ifndef _WIN32
#include <fstream>
#endif

#include "cv1touch_shm.h"
#include "fake_daemon.h"
#include "haptics.h"
#include "control_test.h"
#include "shm_client.h"

using namespace cv1t;

#ifndef _WIN32
/* Observe the actual futex wait, rather than assuming a thread ran after a fixed sleep. */
static void wait_until_armed(long tid, uint32_t seen)
{
	const auto until = cv1t_now_ns() + 5'000'000'000;
	while (cv1t_now_ns() < until) {
		std::ifstream state("/proc/self/task/" + std::to_string(tid) + "/syscall");
		long call = -1;
		uintptr_t address = 0, operation = 0, value = 0;
		if (state >> call >> std::hex >> address >> operation >> value)
			if (call == SYS_futex && (operation & FUTEX_CMD_MASK) == FUTEX_WAIT && value == seen)
				return;
		std::this_thread::yield();
	}
	assert(false && "SHM waiter did not enter its futex wait");
}
#endif

/* Use the observation made by the waiter, even if the socket reply arrives much later. */
static bool woke_promptly(uint64_t published, uint64_t observed)
{
	return published && observed >= published && observed - published < 2'000'000'000;
}

int main(int argc, char **argv)
{
	if (argc == 2 && std::string(argv[1]) == "--timing") {
		assert(woke_promptly(10'000'000'000, 10'010'000'000));
		assert(!woke_promptly(10'000'000'000, 12'010'000'000));
		assert(!woke_promptly(10'000'000'000, 9'000'000'000));
		assert(!woke_promptly(0, 10'010'000'000));
		return 0;
	}
	if (argc == 2 && std::string(argv[1]) == "--status") {
		ControlClient control;
		const bool ok = control.send("status\n");
		printf("status %s (%s)\n", ok ? "connected" : "unavailable", control.path().c_str());
		return ok ? 0 : 1;
	}
	if (argc != 3 && argc != 4) {
		fprintf(stderr, "usage: test_daemon_contract SHM_NAME CONTROL_PATH [FALLBACK_PATH] | --status\n");
		return 2;
	}
	ControlClient control(argv[2], argc == 4 ? argv[3] : "", kControlTestTimeoutMs);
	ShmClient c(argv[1], 2'000'000'000);
	const uint64_t now = cv1t_now_ns();
	assert(c.ensure_open(now));
	assert(c.generation() == 0xC0FFEE);
	// the daemon's heartbeat on our clock: now, give or take the stale window
	assert(c.alive(now) && !c.alive(now + 3'000'000'000));
	cv1t_controller_state st{};
	assert(c.hand(CV1T_HAND_RIGHT, &st) && st.pos[0] == 1.5f && (st.flags & CV1T_FLAG_CONNECTED));
	cv1t_alignment al{};
	assert(c.alignment(&al) && al.valid == CV1T_ALIGN_LIGHTHOUSE && al.lighthouse_pos[2] == -2.0f);

	uint32_t seen = c.wait(~0u, 0);
	for (int round = 0; round < 3; round++) {
		uint32_t woke = seen;
		uint64_t observed = 0;
		std::promise<long> ready;
		std::thread waiter([&] {
#ifdef _WIN32
			ready.set_value(0);
#else
			ready.set_value(syscall(SYS_gettid));
#endif
			woke = c.wait(seen, 5'000'000'000);
			observed = cv1t_now_ns();
		});
		[[maybe_unused]] const auto tid = ready.get_future().get();
#ifndef _WIN32
		wait_until_armed(tid, seen);
#endif
		assert(control.send("publish\n"));
		waiter.join();
		assert(woke == seen + 1);
#ifdef _WIN32
		// Manual-reset events retain their signal even if publication preceded the wait.
		HANDLE event = OpenEventW(SYNCHRONIZE, FALSE, fake_object(woke % 2 ? "update1" : "update0", argv[1]).c_str());
		assert(event && WaitForSingleObject(event, 0) == WAIT_OBJECT_0);
		CloseHandle(event);
#endif
		assert(c.hand(CV1T_HAND_RIGHT, &st));
		assert(woke_promptly(st.publish_host_ns, observed));
		seen = woke;
	}
	printf("test_daemon_contract ok\n");
	return 0;
}
