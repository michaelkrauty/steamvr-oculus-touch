// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

#include "stations.h"

using namespace cv1t;

/* A pose matrix (rotation 90 deg about y, then a translation) reads back as its quaternion. */
static void reads_a_pose_matrix()
{
	vr::HmdMatrix34_t m{};
	// x -> -z, z -> x: rotation of +90 deg about y
	m.m[0][0] = 0;  m.m[0][1] = 0; m.m[0][2] = 1;  m.m[0][3] = 1.85f;
	m.m[1][0] = 0;  m.m[1][1] = 1; m.m[1][2] = 0;  m.m[1][3] = 0.36f;
	m.m[2][0] = -1; m.m[2][1] = 0; m.m[2][2] = 0;  m.m[2][3] = -2.61f;
	StationPose s = station_pose("LHB-TEST-A", m);
	const double h = std::sqrt(0.5);
	assert(std::fabs(s.quat[0]) < 1e-6 && std::fabs(s.quat[1] - h) < 1e-6 && std::fabs(s.quat[2]) < 1e-6 && std::fabs(s.quat[3] - h) < 1e-6);
	assert(std::fabs(s.pos[0] - 1.85) < 1e-6 && std::fabs(s.pos[1] - 0.36) < 1e-6 && std::fabs(s.pos[2] + 2.61) < 1e-6);
	// a half turn about x: w = 0, the largest component is x
	vr::HmdMatrix34_t r{};
	r.m[0][0] = 1; r.m[1][1] = -1; r.m[2][2] = -1;
	s = station_pose("LHB-1", r);
	assert(std::fabs(std::fabs(s.quat[0]) - 1) < 1e-6 && std::fabs(s.quat[3]) < 1e-6);
}

/* What the daemon parses: "stations" then a serial and 7 numbers per station. */
static void formats_the_line_the_daemon_reads()
{
	StationPose a{"LHB-TEST-A", {1.85, 0.36, -2.61}, {0.01, 0.38, -0.02, 0.92}};
	StationPose b{"LHB-TEST-B", {-1.5, 0.35, -2.56}, {0, 0, 0, 1}};
	const std::string line = stations_line({a, b});
	assert(line == "stations LHB-TEST-A 1.850000 0.360000 -2.610000 0.0100000 0.3800000 -0.0200000 0.9200000 "
		       "LHB-TEST-B -1.500000 0.350000 -2.560000 0.0000000 0.0000000 0.0000000 1.0000000\n");
	assert(stations_line({}) == "stations\n");
	// a serial with a space would split the line: dropped
	StationPose bad{"LHB 1", {0, 0, 0}, {0, 0, 0, 1}};
	assert(stations_line({bad, b}).rfind("stations LHB-TEST-B ", 0) == 0);
}

int main()
{
	reads_a_pose_matrix();
	formats_the_line_the_daemon_reads();
	std::puts("test_stations: ok");
	return 0;
}
