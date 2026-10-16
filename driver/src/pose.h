// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#pragma once
#include <cstdint>

#include <openvr_driver.h>

#include <string>
#include <vector>

#include "cv1touch_shm.h"

namespace cv1t {

/* IMU-only coasting after the last optical fix that still reports Running_OK. */
constexpr uint64_t kCoastNs = 250'000'000;
/* A published sample older than this is held still (no velocities) and out of range; older than
 * kDeadSampleNs the controller is reported disconnected (the daemon stopped publishing), unless
 * the daemon holds it through a radio gap (CV1T_FLAG_RADIO_GAP), which lasts 10 s at most:
 * kGapSampleNs bounds that should the daemon stop publishing during one. */
constexpr uint64_t kStaleSampleNs = 100'000'000;
constexpr uint64_t kDeadSampleNs = 1'000'000'000;
constexpr uint64_t kGapSampleNs = 11'000'000'000;

/* Whether a connected controller's published sample is too old to show: the daemon stopped
 * publishing it. */
inline bool sample_dead(const cv1t_controller_state &st, uint64_t now_ns)
{
	if (now_ns <= st.sample_host_ns)
		return false;
	const uint64_t age = now_ns - st.sample_host_ns;
	return age > ((st.flags & CV1T_FLAG_RADIO_GAP) ? kGapSampleNs : kDeadSampleNs);
}

/* The driver's statistics line on the poses it submitted in the last while: `ages` are their
 * sample-to-submission ages (ns, at least one) and `handoffs` the part of each that passed after
 * the daemon published it (ns); both are sorted in place. No hand-offs (an older daemon that
 * stamps no publish time) leaves that part out. */
std::string age_report(const char *serial, std::vector<uint64_t> &ages, std::vector<uint64_t> &handoffs);

/* The SteamVR pose for one hand: st == nullptr (no daemon, no data) or a controller that is not
 * connected gives a disconnected, invalid pose. al == nullptr means no alignment. */
vr::DriverPose_t make_pose(const cv1t_controller_state *st, const cv1t_alignment *al, int hand, uint64_t now_ns);

} // namespace cv1t
