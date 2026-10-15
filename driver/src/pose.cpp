// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include "pose.h"

namespace cv1t {

static vr::HmdQuaternion_t quat(const float q[4]) { return {q[3], q[0], q[1], q[2]}; }

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
