// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include "pose.h"

#include <algorithm>
#include <cstdio>

namespace cv1t {

static vr::HmdQuaternion_t quat(const float q[4]) { return {q[3], q[0], q[1], q[2]}; }

std::string age_report(const char *serial, std::vector<uint64_t> &ages, std::vector<uint64_t> &handoffs)
{
	std::sort(ages.begin(), ages.end());
	std::sort(handoffs.begin(), handoffs.end());
	const auto at = [](const std::vector<uint64_t> &v, double q) { return v[static_cast<size_t>(q * (v.size() - 1))] * 1e-6; };
	char buf[320];
	int n = snprintf(buf, sizeof buf, "cv1touch: %s pose age at submission over %zu poses: p50 %.2f ms p99 %.2f ms max %.2f ms", serial,
			 ages.size(), at(ages, 0.5), at(ages, 0.99), at(ages, 1.0));
	if (!handoffs.empty() && n > 0 && static_cast<size_t>(n) < sizeof buf)
		snprintf(buf + n, sizeof buf - n, "; of it after the daemon published: p50 %.2f ms p99 %.2f ms max %.2f ms", at(handoffs, 0.5),
			 at(handoffs, 0.99), at(handoffs, 1.0));
	return buf;
}

vr::DriverPose_t make_pose(const cv1t_controller_state *st, const cv1t_alignment *al, int hand, uint64_t now_ns)
{
	vr::DriverPose_t p{};
	p.qWorldFromDriverRotation = {1, 0, 0, 0};
	p.qDriverFromHeadRotation = {1, 0, 0, 0};
	p.qRotation = {1, 0, 0, 0};
	p.result = vr::TrackingResult_Uninitialized;
	if (!st || !(st->flags & CV1T_FLAG_CONNECTED))
		return p; // not connected, not valid
	const int64_t age = static_cast<int64_t>(now_ns - st->sample_host_ns);
	if (sample_dead(*st, now_ns))
		return p; // the daemon stopped publishing: no frozen poses
	p.deviceIsConnected = true;
	p.poseIsValid = true;
	if (al && (al->valid & CV1T_ALIGN_LIGHTHOUSE)) {
		p.qWorldFromDriverRotation = quat(al->lighthouse_orient);
		for (int i = 0; i < 3; i++)
			p.vecWorldFromDriverTranslation[i] = al->lighthouse_pos[i];
	}
	const uint32_t render_bit = hand == CV1T_HAND_LEFT ? CV1T_ALIGN_RENDER_LEFT : CV1T_ALIGN_RENDER_RIGHT;
	if (al && (al->valid & render_bit)) {
		p.qDriverFromHeadRotation = quat(al->render_orient[hand]);
		for (int i = 0; i < 3; i++)
			p.vecDriverFromHeadTranslation[i] = al->render_pos[hand][i];
	}
	p.qRotation = quat(st->orient);
	// SteamVR extrapolates with the share of the tracked motion `cv1touch prediction` set: all of
	// it until the daemon has said
	const bool set = al && (al->valid & CV1T_ALIGN_PREDICTION);
	const double position_share = set ? al->prediction_position / 100.0 : 1.0;
	const double rotation_share = set ? al->prediction_rotation / 100.0 : 1.0;
	// SteamVR integrates the angular velocity in the frame of qRotation (measured with a synthetic
	// pose; its documentation says driver world space), and neither the alignment nor the render
	// offset changes that frame: it gets the body rate with the bias removed. The linear velocity
	// is in driver world space, as documented.
	for (int i = 0; i < 3; i++) {
		p.vecPosition[i] = st->pos[i];
		p.vecVelocity[i] = st->lin_vel[i] * position_share;
		p.vecAngularVelocity[i] = ((double)st->ang_vel[i] - st->gyro_bias[i]) * rotation_share;
	}
	// a radio gap's pose is held, not placed in time: its sample can be seconds old
	const bool gap = st->flags & CV1T_FLAG_RADIO_GAP;
	p.poseTimeOffset = age > 0 && !gap ? -static_cast<double>(age) * 1e-9 : 0.0;
	const bool fixed = st->last_position_fix_host_ns != 0 &&
			   static_cast<int64_t>(now_ns - st->last_position_fix_host_ns) <= static_cast<int64_t>(kCoastNs);
	const bool stale = age > static_cast<int64_t>(kStaleSampleNs);
	// a radio gap: the controller as it was, where the game keeps it (the daemon sends no position)
	p.result = fixed && !stale && !gap ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange;
	if (stale) {
		for (int i = 0; i < 3; i++)
			p.vecVelocity[i] = p.vecAngularVelocity[i] = 0; // hold still rather than extrapolate an old motion
	}
	if (!(st->flags & CV1T_FLAG_POSITION)) {
		for (int i = 0; i < 3; i++)
			p.vecVelocity[i] = 0; // no position fix: the position is held, never extrapolated
	}
	p.willDriftInYaw = false;
	p.shouldApplyHeadModel = false;
	return p;
}

} // namespace cv1t
