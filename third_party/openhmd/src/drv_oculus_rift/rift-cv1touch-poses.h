/* cv1touch: calibrated (fixed) sensor poses from the configuration, keyed by sensor serial. */
#ifndef RIFT_CV1TOUCH_POSES_H
#define RIFT_CV1TOUCH_POSES_H

#include <stdbool.h>
#include <string.h>

#include "openhmd_cv1touch.h"
#include "../omath.h"

static inline bool cv1touch_fixed_pose_lookup_list(const ohmd_cv1touch_sensor_pose *poses, size_t count, const char *serial, posef *out)
{
	for (size_t i = 0; i < count; i++) {
		const ohmd_cv1touch_sensor_pose *p = poses + i;

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

static inline bool cv1touch_fixed_pose_lookup(const ohmd_cv1touch_config *cfg, const char *serial, posef *out) {
    return cv1touch_fixed_pose_lookup_list(cfg->fixed_poses, 8, serial, out);
}

#endif
