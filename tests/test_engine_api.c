// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include <openhmd.h>
#include <stddef.h>
#include <stdio.h>

#include "openhmd_cv1touch.h"
#include "test_util.h"

static int frames_seen;

static void on_frame(void *user, const ohmd_cv1touch_frame_info *info, int n, const ohmd_cv1touch_blob *blobs)
{
	(void)user; (void)info; (void)n; (void)blobs;
	frames_seen++;
}

static void on_report(void *user, int source, uint64_t ts, const uint8_t *data, int len)
{
	(void)user; (void)source; (void)ts; (void)data; (void)len;
}

int main(void)
{
	ohmd_context *ctx = ohmd_ctx_create();
	CHECK(ctx != NULL);
	ohmd_cv1touch_config cfg = { .radio_wait_ms = 2, .hmd_tracking = false, .hmd_leds_off = true };
	snprintf(cfg.ignore_sensor_serials[0], sizeof cfg.ignore_sensor_serials[0], "%s", "WMTD000000000");
	ohmd_cv1touch_configure(ctx, &cfg);
    char ignored[65][32];
    ohmd_cv1touch_sensor_pose poses[65] = {0};
    for (int i = 0; i < 65; i++) {
        snprintf(ignored[i], sizeof ignored[i], "TEST%06d", i);
        snprintf(poses[i].serial, sizeof poses[i].serial, "TEST%06d", i);
        poses[i].orient[3] = 1;
    }
    CHECK(ohmd_cv1touch_configure_sensors(NULL, ignored, 65, poses, 65) == -1);
    CHECK(ohmd_cv1touch_configure_sensors(ctx, ignored, 65, poses, 65) == 0);
    CHECK(ohmd_cv1touch_configure_sensors(ctx, NULL, 1, NULL, 0) == -1);
    CHECK(ohmd_cv1touch_configure_sensors(ctx, NULL, 0, NULL, 0) == 0);
    CHECK(ohmd_cv1touch_configure_sensors(ctx, ignored, 65, poses, 65) == 0);
	ohmd_cv1touch_set_frame_callback(ctx, on_frame, NULL);
	ohmd_cv1touch_set_report_callback(ctx, on_report, NULL);
	CHECK(ohmd_cv1touch_get_io_errors(NULL) == 0);
	CHECK(sizeof(ohmd_cv1touch_blob) == 20);
	CHECK(frames_seen == 0);
	CHECK(sizeof(ohmd_cv1touch_sensor_video) == 40);
    CHECK(sizeof(ohmd_cv1touch_sensor_video_quality) == 56);
    CHECK(offsetof(ohmd_cv1touch_sensor_video_quality, frames_decoded) == 32);
    CHECK(offsetof(ohmd_cv1touch_sensor_video_quality, video_format) == 48);
    ohmd_cv1touch_sensor_video_quality quality;
    CHECK(ohmd_cv1touch_get_sensor_video_quality(NULL, 0, &quality) == -1);
    CHECK(ohmd_cv1touch_get_sensor_video_quality(NULL, -1, NULL) == -1);
	ohmd_cv1touch_sensor_video video;
	CHECK(ohmd_cv1touch_get_sensor_video(NULL, 0, &video) == -1);
	CHECK(sizeof(ohmd_cv1touch_sensor_stream) == 64);
	CHECK(sizeof(ohmd_cv1touch_sensor_attachment) == 16);
	CHECK(offsetof(ohmd_cv1touch_sensor_stream, attachment) == 48);
	ohmd_cv1touch_sensor_stream stream;
	CHECK(ohmd_cv1touch_get_sensor_stream(NULL, 0, &stream) == -1);
	CHECK(ohmd_cv1touch_forget_sensor_stream(NULL, "CAMERA1") == -1);
	CHECK(ohmd_cv1touch_forget_sensor_stream(NULL, NULL) == -1);
	CHECK(sizeof(ohmd_cv1touch_led) == 24);
	CHECK(sizeof(ohmd_cv1touch_frame_device) == 64);
	CHECK(sizeof(ohmd_cv1touch_frame_info) == 248);
	CHECK(offsetof(ohmd_cv1touch_frame_info, devices) == 56);
	ohmd_cv1touch_led leds[4];
	CHECK(ohmd_cv1touch_get_led_model(NULL, leds, 4) == -1);
	ohmd_cv1touch_controller c;
	CHECK(ohmd_cv1touch_get_controller(NULL, &c) == -1);
	char identity[33] = "sentinel";
	CHECK(ohmd_cv1touch_get_controller_identity(NULL, identity, sizeof identity) == -1);
	CHECK(identity[0] == '\0');
	CHECK(ohmd_cv1touch_get_controller_identity(NULL, NULL, 33) == -1);
	CHECK(ohmd_cv1touch_get_controller_identity(NULL, identity, 32) == -1);
	ohmd_ctx_destroy(ctx);
	return 0;
}
