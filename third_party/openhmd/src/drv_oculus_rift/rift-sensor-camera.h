/* SPDX-License-Identifier: Apache-2.0 */
#ifndef RIFT_SENSOR_CAMERA_H
#define RIFT_SENSOR_CAMERA_H

#include "rift-sensor-device.h"
#include "rift-sensor-blobwatch.h"

/* Pixel coordinates in, normalised camera rays (z = 1) out. */
void undistort_points(struct blob *blobs, int count, vec3f *out,
                      rift_sensor_camera_params *calib);
/* Projection does not reject points on or behind the camera plane. */
void rift_project_points(rift_led *leds, int count, rift_sensor_camera_params *calib,
                         posef *pose, vec3f *out);

#endif
