// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include "stations.h"

#include <charconv>
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

std::string pin_line(const PinKeys &k, bool quit)
{
	char buf[64];
	std::snprintf(buf, sizeof buf, "pin %d %d %d%s\n", static_cast<int>(k.pin), static_cast<int>(k.ours), static_cast<int>(k.unpinned), quit ? " quit" : "");
	return buf;
}

bool pin_reply(const std::string &reply, PinKeys *want)
{
	if (reply.rfind("ok ", 0) != 0 || reply.back() != '\n')
		return false;
	const char *p = reply.data() + 3, *const end = reply.data() + reply.size() - 1;
	int32_t v[3];
	for (int i = 0; i < 3; i++) {
		if (i && (p == end || *p++ != ' '))
			return false;
		const auto [next, error] = std::from_chars(p, end, v[i]);
		if (error != std::errc())
			return false;
		p = next;
	}
	if (p != end)
		return false;
	*want = {v[0], v[1], v[2]};
	return true;
}

static bool same_pose(const StationPose &a, const StationPose &b)
{
	const double dx = a.pos[0] - b.pos[0], dy = a.pos[1] - b.pos[1], dz = a.pos[2] - b.pos[2];
	// the rotation from a to b: conj(a) * b
	const double *p = a.quat, *q = b.quat;
	const double w = p[3] * q[3] + p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
	const double x = p[3] * q[0] - p[0] * q[3] - p[1] * q[2] + p[2] * q[1];
	const double y = p[3] * q[1] + p[0] * q[2] - p[1] * q[3] - p[2] * q[0];
	const double z = p[3] * q[2] - p[0] * q[1] + p[1] * q[0] - p[2] * q[3];
	const double angle = 2 * std::atan2(std::sqrt(x * x + y * y + z * z), std::fabs(w));
	return std::sqrt(dx * dx + dy * dy + dz * dz) < 1e-4 && angle < 0.01 * 3.14159265358979323846 / 180;
}

bool StationReports::due(const std::vector<StationPose> &stations, uint64_t now_ns)
{
	bool same = any_ && now_ns - sent_ns_ < KEEP_ALIVE_NS && stations.size() == sent_.size();
	for (const auto &s : stations) {
		if (!same)
			break;
		same = false;
		for (const auto &o : sent_)
			if (o.serial == s.serial) {
				same = same_pose(o, s);
				break;
			}
	}
	if (same)
		return false;
	sent_ = stations;
	sent_ns_ = now_ns;
	any_ = true;
	return true;
}

} // namespace cv1t
