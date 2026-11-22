// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include <openvr_driver.h>

namespace cv1t {

/* A base station's raw pose: position (m), orientation x y z w. */
struct StationPose {
	std::string serial;
	double pos[3];
	double quat[4];
};

StationPose station_pose(const std::string &serial, const vr::HmdMatrix34_t &m);

/* "stations SERIAL x y z qx qy qz qw ...\n" for the daemon's control socket (stations whose
 * serial would split the line are left out). */
std::string stations_line(const std::vector<StationPose> &stations);

/* When the station thread, looking every 100 ms, sends the stations to the daemon: at once when
 * the set of stations or a pose changed since the last report, and otherwise every 2 s, which
 * keeps the daemon's copy fresh (it trusts one for 10 s). Two poses within 0.1 mm and 0.01 deg
 * are the same, as in the daemon (align.rs same_pose). */
class StationReports {
public:
	static constexpr uint64_t KEEP_ALIVE_NS = 2'000'000'000;
	/* Whether to send `stations`, seen at `now_ns`; if so, they count as sent then. */
	bool due(const std::vector<StationPose> &stations, uint64_t now_ns);

private:
	std::vector<StationPose> sent_;
	uint64_t sent_ns_ = 0;
	bool any_ = false;
};

} // namespace cv1t
