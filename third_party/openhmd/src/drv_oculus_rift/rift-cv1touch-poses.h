/* cv1touch: calibrated (fixed) sensor poses from the configuration, keyed by sensor serial. */
#ifndef RIFT_CV1TOUCH_POSES_H
#define RIFT_CV1TOUCH_POSES_H

#include <stdbool.h>
#include <string.h>

#include "openhmd_cv1touch.h"
#include "../omath.h"

static inline bool cv1touch_fixed_pose_lookup(const ohmd_cv1touch_config *cfg, const char *serial, posef *out)
{
	for (int i = 0; i < 8; i++) {
		const ohmd_cv1touch_sensor_pose *p = cfg->fixed_poses + i;

		if (p->serial[0] == '\0' || strncmp(p->serial, serial, sizeof p->serial) != 0)
			continue;
		out->orient.x = p->orient[0];
		out->orient.y = p->orient[1];
		out->orient.z = p->orient[2];
		out->orient.w = p->orient[3];
		oquatf_normalize_me(&out->orient);
		out->pos.x = p->pos[0];
		out->pos.y = p->pos[1];
		out->pos.z = p->pos[2];
		return true;
	}
	return false;
}

#endif
