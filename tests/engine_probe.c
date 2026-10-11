// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* Live hardware check for the cv1touch engine API. Not a unit test: needs the headset (USB),
 * Rift sensors and both Touch controllers awake.
 * usage: engine_probe [seconds] [--hmd-leds-off] [--ignore SERIAL] [--dump-blobs PATH] */
#include <openhmd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "openhmd_cv1touch.h"

#include <pthread.h>

/* --dump-blobs PATH: one line per released frame: "serial frame_start_ns x,y,w,h,led_id ..." */
static FILE *dump_file;
static pthread_mutex_t dump_lock = PTHREAD_MUTEX_INITIALIZER;

static void dump_frame(void *user, const ohmd_cv1touch_frame_info *info, int n, const ohmd_cv1touch_blob *blobs)
{
	(void)user;
	pthread_mutex_lock(&dump_lock);
	fprintf(dump_file, "%s %llu", info->serial, (unsigned long long)info->frame_start_host_ns);
	for (int i = 0; i < n; i++)
		fprintf(dump_file, " %.1f,%.1f,%u,%u,%d", blobs[i].x, blobs[i].y, blobs[i].width, blobs[i].height, blobs[i].led_id);
	fputc('\n', dump_file);
	pthread_mutex_unlock(&dump_lock);
}

static double now_s(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int main(int argc, char **argv)
{
	double seconds = argc > 1 ? atof(argv[1]) : 30.0;
	ohmd_cv1touch_config cfg = { .radio_wait_ms = 2 };
	int n_ignore = 0;
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--hmd-leds-off") == 0)
			cfg.hmd_leds_off = true;
		else if (strcmp(argv[i], "--ignore") == 0 && i + 1 < argc && n_ignore < 8)
			snprintf(cfg.ignore_sensor_serials[n_ignore++], 32, "%s", argv[++i]);
		else if (strcmp(argv[i], "--dump-blobs") == 0 && i + 1 < argc)
			dump_file = fopen(argv[++i], "w");
	}

	ohmd_context *ctx = ohmd_ctx_create();
	ohmd_cv1touch_configure(ctx, &cfg);
	if (dump_file)
		ohmd_cv1touch_set_frame_callback(ctx, dump_frame, NULL);
	int n = ohmd_ctx_probe(ctx);
	ohmd_device_settings *settings = ohmd_device_settings_create(ctx);
	int manual = 0;
	ohmd_device_settings_seti(settings, OHMD_IDS_AUTOMATIC_UPDATE, &manual);
	ohmd_device *hand[2] = { NULL, NULL }; /* 0 left, 1 right */
	for (int i = 0; i < n; i++) {
		int flags = 0;
		ohmd_list_geti(ctx, i, OHMD_DEVICE_FLAGS, &flags);
		if (flags & OHMD_DEVICE_FLAGS_NULL_DEVICE)
			continue;
		int h = (flags & OHMD_DEVICE_FLAGS_LEFT_CONTROLLER) ? 0 : (flags & OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER) ? 1 : -1;
		if (h >= 0 && !hand[h])
			hand[h] = ohmd_list_open_device_s(ctx, i, settings);
	}
	ohmd_device_settings_destroy(settings);
	if (!hand[0] || !hand[1]) {
		fprintf(stderr, "controllers not found: %s\n", ohmd_ctx_get_error(ctx));
		return 1;
	}

	uint32_t prev_fix[8][2] = { { 0 } };
	uint64_t prev_dev_ns[2] = { 0, 0 };
	unsigned updates = 0;
	double start = now_s(), next = start + 1.0;
	while (now_s() - start < seconds) {
		ohmd_ctx_update(ctx);
		updates++;
		if (now_s() < next)
			continue;
		next += 1.0;
		printf("t=%5.1f loop/s=%u", now_s() - start, updates);
		updates = 0;
		for (int h = 0; h < 2; h++) {
			ohmd_cv1touch_controller c;
			ohmd_cv1touch_get_controller(hand[h], &c);
			double fix_age_ms = c.last_pose_fix_device_ns ? (c.device_time_ns - c.last_pose_fix_device_ns) / 1e6 : -1.0;
			printf(" | %s cal=%d adv=%d raw=(%+.3f %+.3f %+.3f) view=(%+.3f %+.3f %+.3f) fix_age_ms=%.0f btn=%x trig=%.2f",
			       h ? "R" : "L", c.calibrated, c.device_time_ns != prev_dev_ns[h], c.raw_pos[0], c.raw_pos[1],
			       c.raw_pos[2], c.view_pos[0], c.view_pos[1], c.view_pos[2], fix_age_ms, c.buttons, c.trigger);
			prev_dev_ns[h] = c.device_time_ns;
		}
		int ns = ohmd_cv1touch_get_sensor_count(hand[1]);
		for (int s = 0; s < ns && s < 8; s++) {
			ohmd_cv1touch_sensor si;
			ohmd_cv1touch_get_sensor(hand[1], s, &si);
			printf(" | %s placed=%d fixes/s L=%u R=%u", si.serial, si.have_camera_pose,
			       si.pose_fixes[2] - prev_fix[s][0], si.pose_fixes[1] - prev_fix[s][1]);
			prev_fix[s][0] = si.pose_fixes[2];
			prev_fix[s][1] = si.pose_fixes[1];
		}
		printf("\n");
		fflush(stdout);
	}
	ohmd_close_device(hand[0]);
	ohmd_close_device(hand[1]);
	ohmd_ctx_destroy(ctx);
	if (dump_file)
		fclose(dump_file);
	return 0;
}
