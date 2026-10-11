// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include "stations.h"

#include <cmath>
#include <cstdio>

namespace cv1t {

StationPose station_pose(const std::string &serial, const vr::HmdMatrix34_t &m)
{
	StationPose s{serial, {m.m[0][3], m.m[1][3], m.m[2][3]}, {0, 0, 0, 1}};
	const double r[3][3] = {{m.m[0][0], m.m[0][1], m.m[0][2]}, {m.m[1][0], m.m[1][1], m.m[1][2]}, {m.m[2][0], m.m[2][1], m.m[2][2]}};
	const double tr = r[0][0] + r[1][1] + r[2][2];
	double x, y, z, w;
	// from the largest of w, x, y, z, for precision near half turns
	if (tr > 0) {
		const double k = 2 * std::sqrt(1 + tr);
		w = k / 4, x = (r[2][1] - r[1][2]) / k, y = (r[0][2] - r[2][0]) / k, z = (r[1][0] - r[0][1]) / k;
	} else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
		const double k = 2 * std::sqrt(1 + r[0][0] - r[1][1] - r[2][2]);
		w = (r[2][1] - r[1][2]) / k, x = k / 4, y = (r[0][1] + r[1][0]) / k, z = (r[0][2] + r[2][0]) / k;
	} else if (r[1][1] > r[2][2]) {
		const double k = 2 * std::sqrt(1 + r[1][1] - r[0][0] - r[2][2]);
		w = (r[0][2] - r[2][0]) / k, x = (r[0][1] + r[1][0]) / k, y = k / 4, z = (r[1][2] + r[2][1]) / k;
	} else {
		const double k = 2 * std::sqrt(1 + r[2][2] - r[0][0] - r[1][1]);
		w = (r[1][0] - r[0][1]) / k, x = (r[0][2] + r[2][0]) / k, y = (r[1][2] + r[2][1]) / k, z = k / 4;
	}
	const double n = std::sqrt(x * x + y * y + z * z + w * w);
	s.quat[0] = x / n, s.quat[1] = y / n, s.quat[2] = z / n, s.quat[3] = w / n;
	return s;
}

std::string stations_line(const std::vector<StationPose> &stations)
{
	std::string line = "stations";
	char buf[160];
	for (const auto &s : stations) {
		if (s.serial.empty() || s.serial.find_first_of(" \t\r\n") != std::string::npos)
			continue;
		std::snprintf(buf, sizeof buf, " %s %.6f %.6f %.6f %.7f %.7f %.7f %.7f", s.serial.c_str(), s.pos[0], s.pos[1], s.pos[2], s.quat[0], s.quat[1], s.quat[2], s.quat[3]);
		line += buf;
	}
	return line + "\n";
}

} // namespace cv1t
