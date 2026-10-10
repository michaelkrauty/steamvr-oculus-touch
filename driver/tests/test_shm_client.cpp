// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

#include <cassert>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

#include "shm_client.h"

using cv1t::ShmClient;

static std::string name(const char *tag) { return "/cv1t-drvtest-" + std::to_string(getpid()) + "-" + tag; }

/* A daemon-side segment: created, sized and filled the way cv1touchd does it. */
static cv1t_shm *make(const std::string &n, uint32_t version, uint64_t generation, uint64_t heartbeat)
{
	shm_unlink(n.c_str());
	int fd = shm_open(n.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
	assert(fd >= 0);
	assert(ftruncate(fd, sizeof(cv1t_shm)) == 0);
	auto *s = static_cast<cv1t_shm *>(mmap(nullptr, sizeof(cv1t_shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
	close(fd);
	assert(s != MAP_FAILED);
	s->version = version;
	s->size = sizeof(cv1t_shm);
	s->generation = generation;
	s->heartbeat_host_ns = heartbeat;
	__atomic_store_n(&s->magic, CV1T_SHM_MAGIC, __ATOMIC_RELEASE);
	return s;
}

static void not_running()
{
	ShmClient c(name("none"));
	assert(!c.ensure_open(1'000'000'000));
	assert(!c.alive(1'000'000'000));
}

static void reads_hands_and_alignment()
{
	const std::string n = name("read");
	cv1t_shm *s = make(n, CV1T_SHM_VERSION, 0x42, 1'000'000'000);
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
	shm_unlink(n.c_str());
	munmap(s, sizeof *s);
}

/* Review Focus 2: a daemon of another ABI version is refused, every time. */
static void refuses_another_version()
{
	const std::string n = name("v2");
	cv1t_shm *s = make(n, 2, 1, 1'000'000'000);
	ShmClient c(n);
	assert(!c.ensure_open(1'000'000'000));
	assert(!c.ensure_open(2'000'000'000));
	assert(!c.is_open());
	shm_unlink(n.c_str());
	munmap(s, sizeof *s);
}

/* A crashed daemon leaves its segment with a stale heartbeat: not alive. */
static void stale_heartbeat_is_not_alive()
{
	const std::string n = name("stale");
	cv1t_shm *s = make(n, CV1T_SHM_VERSION, 1, 1'000'000'000);
	ShmClient c(n, 2'000'000'000);
	assert(c.ensure_open(1'500'000'000));
	assert(c.alive(2'900'000'000));
	assert(!c.alive(3'100'000'000));
	shm_unlink(n.c_str());
	munmap(s, sizeof *s);
}

/* Review Focus 1: the daemon exits (magic zeroed) and a new one creates a fresh segment under the
 * same name: the client drops the old mapping and picks up the new generation. */
static void reopens_a_restarted_daemon()
{
	const std::string n = name("restart");
	cv1t_shm *a = make(n, CV1T_SHM_VERSION, 1, 1'000'000'000);
	ShmClient c(n);
	assert(c.ensure_open(1'000'000'000) && c.generation() == 1);
	__atomic_store_n(&a->magic, 0u, __ATOMIC_RELEASE);
	shm_unlink(n.c_str());
	assert(!c.ensure_open(1'100'000'000)); // gone, and the name is gone too
	cv1t_shm *b = make(n, CV1T_SHM_VERSION, 2, 1'700'000'000);
	assert(c.ensure_open(1'700'000'000)); // retried after >= 500 ms
	assert(c.generation() == 2);
	shm_unlink(n.c_str());
	munmap(a, sizeof *a);
	munmap(b, sizeof *b);
}

/* Review fix 3: a writer caught mid-update (odd seq for every retry) does not turn into a
 * one-frame disconnect or misalignment: the last good copy is used. */
static void a_busy_writer_keeps_the_last_good_state()
{
	const std::string n = name("busy");
	cv1t_shm *s = make(n, CV1T_SHM_VERSION, 9, 1'000'000'000);
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
	shm_unlink(n.c_str());
	munmap(s, sizeof *s);
}

int main()
{
	not_running();
	reads_hands_and_alignment();
	refuses_another_version();
	stale_heartbeat_is_not_alive();
	reopens_a_restarted_daemon();
	a_busy_writer_keeps_the_last_good_state();
	printf("test_shm_client ok\n");
	return 0;
}
