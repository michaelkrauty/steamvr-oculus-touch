/* SPDX-License-Identifier: BSL-1.0 */
#ifndef RIFT_SENSOR_POSE_H
#define RIFT_SENSOR_POSE_H

#include "rift-sensor-device.h"
#include "rift-sensor-maths.h"
#include "rift-sensor-blobwatch.h"

/* At most 64 distinct, in-range labels are used. Failure leaves pose intact;
 * num_leds_out counts the collected labels and num_inliers is zero on failure. */
bool estimate_initial_pose(struct blob *blobs, int num_blobs, int device_id,
    rift_led *leds, int num_led_pos, rift_sensor_camera_params *calib,
    posef *pose, int *num_leds_out, int *num_inliers, bool use_extrinsic_guess);

#endif
