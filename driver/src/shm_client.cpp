// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include "shm_client.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "log.h"

namespace cv1t {

ShmClient::ShmClient(std::string name, uint64_t stale_ns) : name_(std::move(name)), stale_ns_(stale_ns) {}

ShmClient::~ShmClient() { close(); }

void ShmClient::close()
{
	if (shm_)
		munmap(const_cast<cv1t_shm *>(shm_), sizeof(cv1t_shm));
	shm_ = nullptr;
	have_hand_[0] = have_hand_[1] = have_alignment_ = false; // a new daemon starts afresh
}

bool ShmClient::alive(uint64_t now_ns) const
{
	if (!shm_ || __atomic_load_n(&shm_->magic, __ATOMIC_ACQUIRE) != CV1T_SHM_MAGIC)
		return false;
	const uint64_t hb = __atomic_load_n(&shm_->heartbeat_host_ns, __ATOMIC_ACQUIRE);
	return now_ns <= hb || now_ns - hb <= stale_ns_;
}

bool ShmClient::ensure_open(uint64_t now_ns)
{
	if (shm_ && !alive(now_ns)) {
		cv1t_log("cv1touch: daemon %s (generation %llx): controllers disconnected",
			 __atomic_load_n(&shm_->magic, __ATOMIC_ACQUIRE) == CV1T_SHM_MAGIC ? "stopped responding" : "exited",
			 (unsigned long long)generation_);
		close();
	}
	if (shm_)
		return true;
	if (tried_ && now_ns - last_try_ns_ < 500'000'000)
		return false;
	tried_ = true;
	last_try_ns_ = now_ns;
	const int fd = shm_open(name_.c_str(), O_RDONLY | O_CLOEXEC, 0);
	if (fd < 0)
		return false;
	struct stat st {};
	if (fstat(fd, &st) < 0 || static_cast<size_t>(st.st_size) < sizeof(cv1t_shm)) {
		::close(fd);
		return false; // not sized yet: mapping it would SIGBUS
	}
	void *p = mmap(nullptr, sizeof(cv1t_shm), PROT_READ, MAP_SHARED, fd, 0);
	::close(fd);
	if (p == MAP_FAILED)
		return false;
	const auto *s = static_cast<const cv1t_shm *>(p);
	if (__atomic_load_n(&s->magic, __ATOMIC_ACQUIRE) != CV1T_SHM_MAGIC) {
		munmap(p, sizeof(cv1t_shm));
		return false;
	}
	if (s->version != CV1T_SHM_VERSION || s->size != sizeof(cv1t_shm)) {
		if (!warned_version_)
			cv1t_log("cv1touch: daemon shared memory is ABI v%u (%u bytes); this driver needs v%u (%zu): restart cv1touchd from the same build",
				 s->version, s->size, CV1T_SHM_VERSION, sizeof(cv1t_shm));
		warned_version_ = true;
		munmap(p, sizeof(cv1t_shm));
		return false;
	}
	shm_ = s;
	generation_ = s->generation;
	if (!alive(now_ns)) {
		close(); // a crashed daemon's leftover segment
		return false;
	}
	cv1t_log("cv1touch: connected to cv1touchd (generation %llx)", (unsigned long long)generation_);
	return true;
}

bool ShmClient::hand(int hand, cv1t_controller_state *out) const
{
	if (!shm_ || hand < 0 || hand >= CV1T_HAND_COUNT)
		return false;
	if (cv1t_shm_read_hand(shm_, hand, out) == 0) {
		last_hand_[hand] = *out;
		have_hand_[hand] = true;
		return true;
	}
	if (!have_hand_[hand])
		return false;
	*out = last_hand_[hand];
	return true;
}

bool ShmClient::alignment(cv1t_alignment *out) const
{
	if (!shm_)
		return false;
	if (cv1t_shm_read_alignment(shm_, out) == 0) {
		last_alignment_ = *out;
		have_alignment_ = true;
		return true;
	}
	if (!have_alignment_)
		return false;
	*out = last_alignment_;
	return true;
}

uint32_t ShmClient::wait(uint32_t last_seen, int64_t timeout_ns) const
{
	return shm_ ? cv1t_shm_wait(shm_, last_seen, timeout_ns) : last_seen;
}

} // namespace cv1t
