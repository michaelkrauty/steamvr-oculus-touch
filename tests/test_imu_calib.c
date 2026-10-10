// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

/* Factory IMU calibration: the offset must be subtracted (upstream computed raw - offset into a
 * variable it then overwrote, so offsets were never applied), then the 3x3 matrix applied. */
#include "omath.c"
#include "rift-imu-calib.h"
#include "test_util.h"

int main(void)
{
	float identity[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
	float swap_xy[3][3] = { { 0, 1, 0 }, { 1, 0, 0 }, { 0, 0, 1 } };
	vec3f raw = {{ 0.25f, -0.5f, 9.9f }};
	vec3f offset = {{ 0.05f, -0.1f, 0.1f }};
	vec3f out;

	rift_imu_apply_calibration(&raw, &offset, identity, &out);
	CHECK_NEAR(out.x, 0.2, 1e-6);
	CHECK_NEAR(out.y, -0.4, 1e-6);
	CHECK_NEAR(out.z, 9.8, 1e-5);

	/* Offset first, then the matrix: M * (raw - offset). */
	rift_imu_apply_calibration(&raw, &offset, swap_xy, &out);
	CHECK_NEAR(out.x, -0.4, 1e-6);
	CHECK_NEAR(out.y, 0.2, 1e-6);
	CHECK_NEAR(out.z, 9.8, 1e-5);
	return 0;
}
