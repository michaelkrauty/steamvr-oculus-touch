// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include <cassert>
#include <cmath>
#include <cstdio>

#include "pose.h"

using cv1t::make_pose;

static bool approx(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }

static cv1t_controller_state state()
{
	cv1t_controller_state s{};
	s.flags = CV1T_FLAG_CALIBRATED | CV1T_FLAG_CONNECTED | CV1T_FLAG_POSITION;
	s.sample_host_ns = 10'000'000'000;
	s.last_position_fix_host_ns = 9'990'000'000;
	const float h = std::sqrt(0.5f);
	s.orient[0] = h; s.orient[3] = h; // 90 degrees about +X
	s.pos[0] = 0.1f; s.pos[1] = 1.2f; s.pos[2] = -0.3f;
	s.lin_vel[0] = 0.5f;
	s.ang_vel[2] = 1.0f; s.gyro_bias[2] = 0.25f; // body +Z at 0.75 rad/s
	return s;
}

static void no_data_is_disconnected()
{
	const auto p = make_pose(nullptr, nullptr, CV1T_HAND_LEFT, 0);
	assert(!p.deviceIsConnected && !p.poseIsValid);
	cv1t_controller_state asleep = state();
	asleep.flags = CV1T_FLAG_CALIBRATED; // paired, not reporting: standby
	const auto q = make_pose(&asleep, nullptr, CV1T_HAND_LEFT, 10'001'000'000);
	assert(!q.deviceIsConnected && !q.poseIsValid);
}

static void connected_pose_and_timing()
{
	const cv1t_controller_state s = state();
	const auto p = make_pose(&s, nullptr, CV1T_HAND_RIGHT, 10'002'000'000);
	assert(p.deviceIsConnected && p.poseIsValid && p.result == vr::TrackingResult_Running_OK);
	assert(approx(p.poseTimeOffset, -0.002));
	assert(approx(p.qRotation.w, std::sqrt(0.5)) && approx(p.qRotation.x, std::sqrt(0.5)) && approx(p.qRotation.y, 0));
	assert(approx(p.vecPosition[1], 1.2f) && approx(p.vecVelocity[0], 0.5f));
	// body +Z rotated 90 degrees about +X is world -Y
	assert(approx(p.vecAngularVelocity[0], 0, 1e-6) && approx(p.vecAngularVelocity[1], -0.75, 1e-6) && approx(p.vecAngularVelocity[2], 0, 1e-6));
	assert(p.qWorldFromDriverRotation.w == 1 && p.qDriverFromHeadRotation.w == 1);
}

static void prediction_shares_scale_the_velocities()
{
	const cv1t_controller_state s = state();
	cv1t_alignment al{};
	// an alignment without the percents (from before they were published) gives all the motion
	const auto all = make_pose(&s, &al, CV1T_HAND_RIGHT, 10'002'000'000);
	assert(approx(all.vecVelocity[0], 0.5) && approx(all.vecAngularVelocity[1], -0.75));
	al.valid = CV1T_ALIGN_PREDICTION;
	al.prediction_position = 100;
	al.prediction_rotation = 0;
	const auto moves = make_pose(&s, &al, CV1T_HAND_RIGHT, 10'002'000'000);
	assert(approx(moves.vecVelocity[0], 0.5) && moves.vecAngularVelocity[1] == 0);
	al.prediction_position = 0;
	al.prediction_rotation = 100;
	const auto turns = make_pose(&s, &al, CV1T_HAND_RIGHT, 10'002'000'000);
	assert(turns.vecVelocity[0] == 0 && approx(turns.vecAngularVelocity[1], -0.75));
	al.prediction_position = 50;
	al.prediction_rotation = 50;
	const auto half = make_pose(&s, &al, CV1T_HAND_RIGHT, 10'002'000'000);
	assert(approx(half.vecVelocity[0], 0.25) && approx(half.vecAngularVelocity[1], -0.375));
	// the pose itself does not move with the shares
	assert(half.vecPosition[1] == moves.vecPosition[1] && half.qRotation.w == moves.qRotation.w);
}

static void coasting_then_out_of_range()
{
	cv1t_controller_state s = state(); // fix at 9.990 s; IMU samples keep arriving (coasting)
	const auto at = [&](uint64_t now) {
		s.sample_host_ns = now - 1'000'000;
		return make_pose(&s, nullptr, 0, now);
	};
	assert(at(9'990'000'000 + cv1t::kCoastNs).result == vr::TrackingResult_Running_OK);
	const auto late = at(9'990'000'000 + cv1t::kCoastNs + 1'000'000);
	assert(late.result == vr::TrackingResult_Running_OutOfRange && late.poseIsValid && late.deviceIsConnected);
	cv1t_controller_state never = state();
	never.flags &= ~CV1T_FLAG_POSITION;
	never.last_position_fix_host_ns = 0;
	assert(make_pose(&never, nullptr, 0, 10'001'000'000).result == vr::TrackingResult_Running_OutOfRange);
}

/* Without a position fix the position is held: no linear velocity for SteamVR to extrapolate,
 * while the angular velocity (the gyro still knows) stays. */
static void a_lost_position_is_not_extrapolated()
{
	cv1t_controller_state s = state();
	s.flags &= ~CV1T_FLAG_POSITION;
	const auto p = make_pose(&s, nullptr, CV1T_HAND_RIGHT, 10'001'000'000);
	assert(p.poseIsValid && p.vecVelocity[0] == 0 && p.vecVelocity[1] == 0 && p.vecVelocity[2] == 0);
	assert(approx(p.vecAngularVelocity[1], -0.75, 1e-6));
}

static void alignment_is_applied_per_hand()
{
	const cv1t_controller_state s = state();
	cv1t_alignment al{};
	al.valid = CV1T_ALIGN_LIGHTHOUSE | CV1T_ALIGN_RENDER_LEFT;
	al.lighthouse_orient[1] = 1.0f; // 180 degrees about +Y, x y z w
	al.lighthouse_pos[0] = 2.0f;
	al.render_orient[CV1T_HAND_LEFT][3] = 1.0f;
	al.render_pos[CV1T_HAND_LEFT][2] = 0.04f;
	const auto left = make_pose(&s, &al, CV1T_HAND_LEFT, 10'001'000'000);
	assert(left.qWorldFromDriverRotation.w == 0 && left.qWorldFromDriverRotation.y == 1 && left.vecWorldFromDriverTranslation[0] == 2.0);
	assert(approx(left.vecDriverFromHeadTranslation[2], 0.04f));
	const auto right = make_pose(&s, &al, CV1T_HAND_RIGHT, 10'001'000'000);
	assert(right.qDriverFromHeadRotation.w == 1 && right.vecDriverFromHeadTranslation[2] == 0); // no right offset
}

/* A sample stamped slightly after "now" (clocks read on two threads) is not a future pose. */
static void never_a_positive_time_offset()
{
	const cv1t_controller_state s = state();
	assert(make_pose(&s, nullptr, 0, 9'999'000'000).poseTimeOffset == 0.0);
}

/* Review fix 2: a daemon that stops publishing (hung engine) must not keep a controller moving on
 * its last velocities: past 100 ms it stops and is out of range, past 1 s it is disconnected. */
static void stale_sample_stops_then_disconnects()
{
	const cv1t_controller_state s = state(); // sample at 10.000 s, moving
	const auto aging = make_pose(&s, nullptr, CV1T_HAND_RIGHT, 10'150'000'000);
	assert(aging.deviceIsConnected && aging.poseIsValid);
	assert(aging.vecVelocity[0] == 0 && aging.vecAngularVelocity[1] == 0);
	assert(aging.result == vr::TrackingResult_Running_OutOfRange);
	const auto dead = make_pose(&s, nullptr, CV1T_HAND_RIGHT, 11'500'000'000);
	assert(!dead.deviceIsConnected && !dead.poseIsValid);
	assert(make_pose(&s, nullptr, CV1T_HAND_RIGHT, 10'050'000'000).vecVelocity[0] == 0.5); // still fresh
}

int main()
{
	no_data_is_disconnected();
	connected_pose_and_timing();
	prediction_shares_scale_the_velocities();
	coasting_then_out_of_range();
	a_lost_position_is_not_extrapolated();
	alignment_is_applied_per_hand();
	never_a_positive_time_offset();
	stale_sample_stops_then_disconnects();
	printf("test_pose ok\n");
	return 0;
}
