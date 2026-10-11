// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* Fixed (calibrated) sensor poses are looked up by exact serial; orientation is normalised. */
#include <string.h>

#include "omath.c"
#include "rift-cv1touch-poses.h"
#include "test_util.h"

int main(void)
{
	ohmd_cv1touch_config cfg;
	posef p;

	memset(&cfg, 0, sizeof cfg);
	CHECK(!cv1touch_fixed_pose_lookup(&cfg, "WMTD000000002", &p)); /* empty table */

	snprintf(cfg.fixed_poses[1].serial, sizeof cfg.fixed_poses[1].serial, "%s", "WMTD000000002");
	cfg.fixed_poses[1].orient[3] = 2.0f; /* unnormalised identity */
	cfg.fixed_poses[1].pos[0] = 1.5f;
	cfg.fixed_poses[1].pos[1] = 2.25f;
	cfg.fixed_poses[1].pos[2] = -0.5f;

	CHECK(!cv1touch_fixed_pose_lookup(&cfg, "WMTD00000000", &p)); /* a prefix is not a match */
	CHECK(!cv1touch_fixed_pose_lookup(&cfg, "WMTD000000003", &p));
	CHECK(cv1touch_fixed_pose_lookup(&cfg, "WMTD000000002", &p));
	CHECK_NEAR(p.orient.w, 1.0, 1e-6);
	CHECK_NEAR(p.orient.x, 0.0, 1e-6);
	CHECK_NEAR(p.pos.x, 1.5, 1e-6);
	CHECK_NEAR(p.pos.y, 2.25, 1e-6);
	CHECK_NEAR(p.pos.z, -0.5, 1e-6);
	return 0;
}
