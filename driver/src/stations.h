// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

#pragma once
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

} // namespace cv1t
