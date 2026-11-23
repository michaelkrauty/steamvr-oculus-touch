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

/* "headset [start] UNIVERSE SYSTEM SERIAL\n": SteamVR's headset for the daemon's control socket,
 * the serial last since it may hold spaces; "start" until the daemon has taken one in this
 * SteamVR start; "-" for a headset without a tracking system (SteamVR's null driver sets none).
 * Empty for a tracking system that would split the line or a serial that would end it. */
std::string headset_line(const std::string &serial, const std::string &system, uint64_t universe, bool start);

/* SteamVR's base-station setting (driver_lighthouse.primarybasestation) and cv1touch's records of
 * it in its own section, 0 when not set: the driver reports them to the daemon every 2 s and
 * writes what the daemon answers (src/base_pin.rs decides). */
struct PinKeys {
	int32_t pin = 0, ours = 0, unpinned = 0;
	bool operator==(const PinKeys &) const = default;
};

/* "pin PIN OURS UNPINNED\n" for the daemon's control socket, with " quit" before the newline for
 * the last report, as SteamVR quits. */
std::string pin_line(const PinKeys &k, bool quit = false);
/* The values the daemon's reply asks for ("ok PIN OURS UNPINNED"); false for a plain "ok", which
 * leaves them as they are, and for anything else. */
bool pin_reply(const std::string &reply, PinKeys *want);

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
