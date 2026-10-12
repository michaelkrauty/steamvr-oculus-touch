// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#pragma once
/* The daemon's side of the shared memory, for the driver tests: created, sized, filled and woken
 * the way cv1touchd does it, on either platform. */
#include <cassert>
#include <climits>
#include <cstring>
#include <string>

#include "cv1touch_shm.h"

#ifndef _WIN32
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

struct FakeDaemon {
	std::string name;
	cv1t_shm *shm = nullptr;
#ifdef _WIN32
	HANDLE mapping = nullptr;
	HANDLE events[2] = {nullptr, nullptr};
#endif
};

static inline int fake_pid()
{
#ifdef _WIN32
	return static_cast<int>(GetCurrentProcessId());
#else
	return static_cast<int>(getpid());
#endif
}

static inline std::string fake_name(const char *tag) { return "/cv1t-drvtest-" + std::to_string(fake_pid()) + "-" + tag; }

#ifdef _WIN32
static inline std::wstring fake_object(const char *kind, const std::string &name)
{
	const std::string n = std::string("Local\\cv1touch-") + kind + "-" + name.substr(name.rfind('/') + 1);
	return std::wstring(n.begin(), n.end());
}
#endif

static inline FakeDaemon fake_start(const std::string &name, uint32_t version, uint64_t generation, uint64_t heartbeat)
{
	FakeDaemon d;
	d.name = name;
#ifdef _WIN32
	d.mapping = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, sizeof(cv1t_shm), fake_object("shm", name).c_str());
	assert(d.mapping);
	const bool reused = GetLastError() == ERROR_ALREADY_EXISTS; // a reader's view kept the old one
	d.shm = static_cast<cv1t_shm *>(MapViewOfFile(d.mapping, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(cv1t_shm)));
	assert(d.shm);
	if (reused)
		std::memset(d.shm, 0, sizeof(cv1t_shm));
	d.events[0] = CreateEventW(nullptr, TRUE, FALSE, fake_object("update0", name).c_str());
	d.events[1] = CreateEventW(nullptr, TRUE, FALSE, fake_object("update1", name).c_str());
	assert(d.events[0] && d.events[1]);
#else
	shm_unlink(name.c_str());
	int fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
	assert(fd >= 0);
	assert(ftruncate(fd, sizeof(cv1t_shm)) == 0);
	void *p = mmap(nullptr, sizeof(cv1t_shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	assert(p != MAP_FAILED);
	d.shm = static_cast<cv1t_shm *>(p);
#endif
	d.shm->version = version;
	d.shm->size = sizeof(cv1t_shm);
	d.shm->generation = generation;
	d.shm->heartbeat_host_ns = heartbeat;
	__atomic_store_n(&d.shm->magic, CV1T_SHM_MAGIC, __ATOMIC_RELEASE);
	return d;
}

/* One publish batch: bump the counter, wake every waiting reader. */
static inline void fake_publish(FakeDaemon &d)
{
	const uint32_t n = __atomic_add_fetch(&d.shm->update_futex, 1u, __ATOMIC_RELEASE);
#ifdef _WIN32
	SetEvent(d.events[n % 2]);
	ResetEvent(d.events[(n + 1) % 2]);
#else
	(void)n;
	syscall(SYS_futex, &d.shm->update_futex, FUTEX_WAKE, INT_MAX, nullptr, nullptr, 0);
#endif
}

/* The daemon lets go of the name (its exit); readers' mappings stay valid. */
static inline void fake_release_name(FakeDaemon &d)
{
#ifdef _WIN32
	for (HANDLE &h : d.events) {
		CloseHandle(h);
		h = nullptr;
	}
	CloseHandle(d.mapping);
	d.mapping = nullptr;
#else
	shm_unlink(d.name.c_str());
#endif
}

static inline void fake_unmap(FakeDaemon &d)
{
#ifdef _WIN32
	UnmapViewOfFile(d.shm);
	if (d.mapping)
		fake_release_name(d);
#else
	munmap(d.shm, sizeof(cv1t_shm));
#endif
	d.shm = nullptr;
}
