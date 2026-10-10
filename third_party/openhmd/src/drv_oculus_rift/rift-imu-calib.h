/* cv1touch: factory IMU calibration for the Rift headset and Touch controllers. Upstream wrote
 * raw - offset into a variable it then overwrote, so the factory offsets were never applied. The
 * factory data is described as offsets subtracted first, then the 3x3 rotation/cross-axis matrix. */
#ifndef RIFT_IMU_CALIB_H
#define RIFT_IMU_CALIB_H

#include "../omath.h"

static inline void rift_imu_apply_calibration(const vec3f *raw, const vec3f *offset, const float matrix[3][3], vec3f *out)
{
	vec3f centred;

	ovec3f_subtract(raw, offset, &centred);
	ovec3f_multiply_mat3x3(&centred, matrix, out);
}

#endif
