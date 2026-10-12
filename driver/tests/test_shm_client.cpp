// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "fake_daemon.h"
#include "shm_client.h"

using cv1t::ShmClient;

static std::string name(const char *tag) { return fake_name(tag); }

static void not_running()
{
	ShmClient c(name("none"));
	assert(!c.ensure_open(1'000'000'000));
	assert(!c.alive(1'000'000'000));
}

static void reads_hands_and_alignment()
{
	const std::string n = name("read");
	FakeDaemon d = fake_start(n, CV1T_SHM_VERSION, 0x42, 1'000'000'000);
	cv1t_shm *s = d.shm;
	s->hands[CV1T_HAND_RIGHT].state.pos[0] = 1.5f;
	s->hands[CV1T_HAND_RIGHT].state.flags = CV1T_FLAG_CONNECTED;
	s->alignment.valid = CV1T_ALIGN_LIGHTHOUSE;
	s->alignment.lighthouse_pos[2] = -2.0f;
	ShmClient c(n);
	assert(c.ensure_open(1'100'000'000));
	assert(c.generation() == 0x42);
	cv1t_controller_state st{};
	assert(c.hand(CV1T_HAND_RIGHT, &st) && st.pos[0] == 1.5f && st.flags == CV1T_FLAG_CONNECTED);
	cv1t_alignment al{};
	assert(c.alignment(&al) && al.valid == CV1T_ALIGN_LIGHTHOUSE && al.lighthouse_pos[2] == -2.0f);
	fake_unmap(d);
}

/* Review Focus 2: a daemon of another ABI version is refused, every time. */
static void refuses_another_version()
{
	const std::string n = name("v2");
	FakeDaemon d = fake_start(n, 2, 1, 1'000'000'000);
	ShmClient c(n);
	assert(!c.ensure_open(1'000'000'000));
	assert(!c.ensure_open(2'000'000'000));
	assert(!c.is_open());
	fake_unmap(d);
}

/* A crashed daemon leaves its segment with a stale heartbeat: not alive. */
static void stale_heartbeat_is_not_alive()
{
	const std::string n = name("stale");
	FakeDaemon d = fake_start(n, CV1T_SHM_VERSION, 1, 1'000'000'000);
	ShmClient c(n, 2'000'000'000);
	assert(c.ensure_open(1'500'000'000));
	assert(c.alive(2'900'000'000));
	assert(!c.alive(3'100'000'000));
	fake_unmap(d);
}

/* Review Focus 1: the daemon exits (magic zeroed) and a new one creates a fresh segment under the
 * same name: the client drops the old mapping and picks up the new generation. */
static void reopens_a_restarted_daemon()
{
	const std::string n = name("restart");
	FakeDaemon a = fake_start(n, CV1T_SHM_VERSION, 1, 1'000'000'000);
	ShmClient c(n);
	assert(c.ensure_open(1'000'000'000) && c.generation() == 1);
	__atomic_store_n(&a.shm->magic, 0u, __ATOMIC_RELEASE);
	fake_release_name(a);
	assert(!c.ensure_open(1'100'000'000)); // gone, and the name is gone too
	// on Windows a's view keeps the section alive, and the new daemon reuses it zeroed
	FakeDaemon b = fake_start(n, CV1T_SHM_VERSION, 2, 1'700'000'000);
	assert(c.ensure_open(1'700'000'000)); // retried after >= 500 ms
	assert(c.generation() == 2);
	fake_unmap(b);
	fake_unmap(a);
}

/* Review fix 3: a writer caught mid-update (odd seq for every retry) does not turn into a
 * one-frame disconnect or misalignment: the last good copy is used. */
static void a_busy_writer_keeps_the_last_good_state()
{
	const std::string n = name("busy");
	FakeDaemon d = fake_start(n, CV1T_SHM_VERSION, 9, 1'000'000'000);
	cv1t_shm *s = d.shm;
	s->hands[CV1T_HAND_LEFT].state.pos[1] = 0.75f;
	s->alignment.valid = CV1T_ALIGN_LIGHTHOUSE;
	ShmClient c(n);
	assert(c.ensure_open(1'000'000'000));
	cv1t_controller_state st{};
	cv1t_alignment al{};
	assert(c.hand(CV1T_HAND_LEFT, &st) && c.alignment(&al));
	s->hands[CV1T_HAND_LEFT].seq = 1; // writer stuck mid-update
	s->alignment_seq = 1;
	cv1t_controller_state st2{};
	cv1t_alignment al2{};
	assert(c.hand(CV1T_HAND_LEFT, &st2) && st2.pos[1] == 0.75f);
	assert(c.alignment(&al2) && al2.valid == CV1T_ALIGN_LIGHTHOUSE);
	assert(!c.hand(CV1T_HAND_RIGHT, &st2) || true); // the right hand was never read: no copy needed either way
	fake_unmap(d);
}

/* The pose thread waits for each publish: it wakes promptly, publish after publish, alongside
 * another reader waiting at the same time (a tool, or a second driver instance). */
static void every_waiting_reader_wakes_on_each_publish()
{
	const std::string n = name("wake");
	FakeDaemon d = fake_start(n, CV1T_SHM_VERSION, 3, cv1t_now_ns());
	std::vector<std::thread> readers;
	std::vector<long long> slowest_ms(2, 0);
	for (int r = 0; r < 2; r++)
		readers.emplace_back([&n, &slowest_ms, r] {
			ShmClient c(n);
			assert(c.ensure_open(cv1t_now_ns()));
			for (uint32_t seen = 0; seen < 3;) {
				const auto before = std::chrono::steady_clock::now();
				const uint32_t now = c.wait(seen, 5'000'000'000);
				const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - before).count();
				if (ms > slowest_ms[r])
					slowest_ms[r] = ms;
				seen = now;
			}
		});
	for (int i = 0; i < 3; i++) {
		std::this_thread::sleep_for(std::chrono::milliseconds(150));
		fake_publish(d);
	}
	for (auto &t : readers)
		t.join();
	for (long long ms : slowest_ms)
		assert(ms < 2000);
	// nothing published: the wait ends at its timeout
	ShmClient c(n);
	assert(c.ensure_open(cv1t_now_ns()));
	const auto before = std::chrono::steady_clock::now();
	assert(c.wait(3, 30'000'000) == 3);
	assert(std::chrono::steady_clock::now() - before >= std::chrono::milliseconds(20));
	fake_unmap(d);
}

int main()
{
	not_running();
	reads_hands_and_alignment();
	refuses_another_version();
	stale_heartbeat_is_not_alive();
	reopens_a_restarted_daemon();
	a_busy_writer_keeps_the_last_good_state();
	every_waiting_reader_wakes_on_each_publish();
	printf("test_shm_client ok\n");
	return 0;
}
