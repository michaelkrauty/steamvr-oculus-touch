// SPDX-License-Identifier: Apache-2.0
/* Search scheduling tests deliberately use no camera calibration or pose solver. */
#include "rift-sensor-pose.h"

bool estimate_initial_pose(struct blob *blobs, int num_blobs, int device_id,
    rift_led *leds, int num_led_pos, rift_sensor_camera_params *calib,
    posef *pose, int *num_leds_out, int *num_inliers, bool use_extrinsic_guess)
{
    (void)blobs; (void)num_blobs; (void)device_id; (void)leds; (void)num_led_pos;
    (void)calib; (void)pose; (void)num_leds_out; (void)num_inliers; (void)use_extrinsic_guess;
    return false;
}
