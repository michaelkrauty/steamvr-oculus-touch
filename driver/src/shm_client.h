// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#pragma once
#include <cstdint>
#include <string>

#include "cv1touch_shm.h"

namespace cv1t {

/* The daemon's segment, read-only. ensure_open() maps it when needed, and drops a mapping whose
 * daemon went away (magic zeroed on exit, or heartbeat older than stale_ns after a crash). It
 * retries the name at most every 500 ms, so a restarted daemon's fresh segment is picked up
 * without restarting SteamVR. A segment of another ABI version is refused. */
class ShmClient {
public:
	explicit ShmClient(std::string name = CV1T_SHM_NAME, uint64_t stale_ns = 2'000'000'000);
	~ShmClient();
	ShmClient(const ShmClient &) = delete;
	ShmClient &operator=(const ShmClient &) = delete;
	bool ensure_open(uint64_t now_ns);
	bool alive(uint64_t now_ns) const;
	bool hand(int hand, cv1t_controller_state *out) const;
	bool alignment(cv1t_alignment *out) const;
	/* Blocks until the update counter differs from last_seen or timeout_ns passes. */
	uint32_t wait(uint32_t last_seen, int64_t timeout_ns) const;
	uint64_t generation() const { return generation_; }
	bool is_open() const { return shm_ != nullptr; }

private:
	void close();
	std::string name_;
	uint64_t stale_ns_;
	const cv1t_shm *shm_ = nullptr;
	uint64_t generation_ = 0;
	uint64_t last_try_ns_ = 0;
	bool tried_ = false;
	bool warned_version_ = false;
	/* Last consistent reads: a writer caught mid-update for every retry yields these instead. */
	mutable cv1t_controller_state last_hand_[CV1T_HAND_COUNT]{};
	mutable bool have_hand_[CV1T_HAND_COUNT] = {false, false};
	mutable cv1t_alignment last_alignment_{};
	mutable bool have_alignment_ = false;
};

} // namespace cv1t
