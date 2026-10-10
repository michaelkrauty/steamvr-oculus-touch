// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

#include "pose.h"

namespace cv1t {

static vr::HmdQuaternion_t quat(const float q[4]) { return {q[3], q[0], q[1], q[2]}; }

/* v rotated by unit quaternion (x y z w). */
static void rotate(const float q[4], const double v[3], double out[3])
{
	const double x = q[0], y = q[1], z = q[2], w = q[3];
	const double tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
	out[0] = v[0] + w * tx + (y * tz - z * ty);
	out[1] = v[1] + w * ty + (z * tx - x * tz);
	out[2] = v[2] + w * tz + (x * ty - y * tx);
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
	if (age > static_cast<int64_t>(kDeadSampleNs))
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
	const double w_body[3] = {(double)st->ang_vel[0] - st->gyro_bias[0], (double)st->ang_vel[1] - st->gyro_bias[1], (double)st->ang_vel[2] - st->gyro_bias[2]};
	rotate(st->orient, w_body, p.vecAngularVelocity);
	for (int i = 0; i < 3; i++) {
		p.vecPosition[i] = st->pos[i];
		p.vecVelocity[i] = st->lin_vel[i];
	}
	p.poseTimeOffset = age > 0 ? -static_cast<double>(age) * 1e-9 : 0.0;
	const bool fixed = st->last_position_fix_host_ns != 0 &&
			   static_cast<int64_t>(now_ns - st->last_position_fix_host_ns) <= static_cast<int64_t>(kCoastNs);
	const bool stale = age > static_cast<int64_t>(kStaleSampleNs);
	p.result = fixed && !stale ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange;
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
