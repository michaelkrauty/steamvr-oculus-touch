// Copyright 2021, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

#include <stdbool.h>
#include "openhmd_cv1touch.h"

#include "ohmd-video.h"
#include "rift-tracker-common.h"

/* Oculus Rift driver - interface for a Rift Sensor video device */
#ifndef __RIFT_SENSOR_DEVICE_H__
#define __RIFT_SENSOR_DEVICE_H__

typedef struct rift_sensor_camera_params rift_sensor_camera_params;
typedef struct rift_sensor_device rift_sensor_device;
typedef struct rift_sensor_device_funcs rift_sensor_device_funcs;

typedef void (*rift_sensor_device_frame_cb)(struct rift_sensor_device *dev, ohmd_video_frame *frame, void *cb_data);

struct rift_sensor_camera_params {
	bool is_cv1; /* CV1 has fisheye distortion, DK2 does not */

	/* Frame width and height */
	int width;
	int height;

	dmat3 camera_matrix;
	bool dist_fisheye;
	double dist_coeffs[5];
};

struct rift_sensor_device_funcs {
	bool (*start) (rift_sensor_device *device, uint8_t min_frames, rift_sensor_device_frame_cb frame_cb, void *cb_data);
	void (*stop) (rift_sensor_device *device);
	void (*free) (rift_sensor_device *device);
	/* Must return without waiting for callbacks, locks or device I/O. */
	void (*request_stop) (rift_sensor_device *device);
	/* Engine thread only, outside tracker/sensor locks. */
	void (*retry) (rift_sensor_device *device, uint64_t now_ns);
};

struct rift_sensor_device {
	rift_sensor_device_funcs *funcs;
	rift_sensor_camera_params calib;
	ohmd_cv1touch_sensor_stream stream_status;
};

#define rift_sensor_device_start_video(d,min_frames,frame_cb,frame_cb_data) \
		(d)->funcs->start((d), (min_frames), (frame_cb), (frame_cb_data))

#define rift_sensor_device_request_stop(d) (d)->funcs->request_stop((d))
#define rift_sensor_device_retry(d, now) (d)->funcs->retry((d), (now))
#define rift_sensor_device_stop_video(d) (d)->funcs->stop((d))
#define rift_sensor_device_free(d) (d)->funcs->free((d))

#endif
