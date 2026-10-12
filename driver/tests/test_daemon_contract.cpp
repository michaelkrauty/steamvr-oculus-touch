// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* Run by examples/driver_contract.rs, which plays cv1touchd with the daemon's own Rust code: the
 * driver must find the shared memory and the control endpoint by the same names, read what was
 * published, run on the same clock, and wake on each publish. */
#include <cassert>
#include <chrono>
#include <cstdio>
#include <thread>

#include "cv1touch_shm.h"
#include "haptics.h"
#include "shm_client.h"

using namespace cv1t;

int main(int argc, char **argv)
{
	if (argc != 3) {
		fprintf(stderr, "usage: test_daemon_contract SHM_NAME CONTROL_PATH\n");
		return 2;
	}
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
		const auto before = std::chrono::steady_clock::now();
		std::thread waiter([&] { woke = c.wait(seen, 5'000'000'000); });
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		assert(send_control_line(argv[2], "publish\n"));
		waiter.join();
		assert(std::chrono::steady_clock::now() - before < std::chrono::seconds(2));
		assert(woke == seen + 1);
		seen = woke;
	}
	printf("test_daemon_contract ok\n");
	return 0;
}
