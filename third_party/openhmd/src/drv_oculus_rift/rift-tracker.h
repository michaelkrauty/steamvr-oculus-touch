// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift driver - positional tracking interface */

#include "rift.h"
#include "rift-kalman-6dof.h"
#include "rift-tracker-common.h"
#include "rift-sensor-pose-helper.h"
#include "openhmd_cv1touch.h"

#ifndef __RIFT_TRACKER_H__
#define __RIFT_TRACKER_H__

typedef struct rift_tracker_ctx_s rift_tracker_ctx;

rift_tracker_ctx *rift_tracker_new (ohmd_context* ohmd_ctx, const uint8_t radio_id[5]);

rift_tracked_device *rift_tracker_add_device (rift_tracker_ctx *ctx, int device_id, posef *imu_pose, posef *model_pose, rift_leds *leds, rift_tracked_device_imu_calibration *calib);
void rift_tracker_on_new_exposure (rift_tracker_ctx *ctx, uint32_t hmd_ts, uint16_t exposure_count, uint32_t exposure_hmd_ts, uint8_t led_pattern_phase);
uint8_t rift_tracker_get_device_list(rift_tracker_ctx *tracker_ctx, rift_tracked_device **dev_list);

bool rift_tracker_frame_captured (rift_tracker_ctx *ctx, uint64_t local_ts, uint64_t frame_start_local_ts, rift_tracker_exposure_info *info, const char *source);
void rift_tracker_frame_release (rift_tracker_ctx *ctx, uint64_t local_ts, uint64_t frame_local_ts, rift_tracker_exposure_info *info, const char *source);

void rift_tracker_free (rift_tracker_ctx *ctx);
/* Engine thread only, outside sensor/tracker locks. */
void rift_tracker_retry_video(rift_tracker_ctx *ctx, uint64_t now_ns);

void rift_tracked_device_imu_update(rift_tracked_device *dev, uint64_t local_ts, uint32_t device_ts, float dt, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field);
void rift_tracked_device_get_view_pose(rift_tracked_device *dev, posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel);

typedef struct rift_tracked_device_export {
	uint64_t device_time_ns;
	uint64_t last_observed_pose_ts;
	uint64_t last_observed_orient_ts;
	posef view_pose;
	vec3f view_lin_vel, view_ang_vel, view_lin_accel;
	posef raw_pose;
	vec3f pos_error, rot_error;
	vec3f gyro_raw, gyro_bias;
} rift_tracked_device_export;

void rift_tracked_device_get_export(rift_tracked_device *dev, rift_tracked_device_export *out);
int rift_tracker_get_sensor_count(rift_tracker_ctx *ctx);
/* cv1touch: the devices (bit 1 << device id) the brute-force search leaves out, because the
 * daemon's own tracker holds them or they are asleep (rift_pose_finder_process_blobs_fast).
 * Any thread. */
void rift_tracker_set_search_skip(rift_tracker_ctx *ctx, uint32_t device_mask);
uint32_t rift_tracker_get_search_skip(rift_tracker_ctx *ctx);
int rift_tracker_get_sensor_info(rift_tracker_ctx *ctx, int index, ohmd_cv1touch_sensor *out);
int rift_tracker_set_sensor_pose(rift_tracker_ctx *ctx, const ohmd_cv1touch_sensor_pose *pose);
int rift_tracker_clear_sensor_pose(rift_tracker_ctx *ctx, const char *serial);

bool rift_tracked_device_get_latest_exposure_info_pose (rift_tracked_device *dev, rift_tracked_device_exposure_info *dev_info);

bool rift_tracked_device_model_pose_update(rift_tracked_device *dev_base, uint64_t local_ts, uint64_t frame_start_local_ts, rift_tracker_exposure_info *exposure_info, rift_pose_metrics *score, posef *pose, const char *source);
void rift_tracked_device_frame_release (rift_tracked_device *dev_base, rift_tracker_exposure_info *info);

#endif
