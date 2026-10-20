/*
 * Rift sensor interface. Processing task.
 * Copyright 2014-2015 Philipp Zabel
 * Copyright 2019 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */

#define _GNU_SOURCE
#define __STDC_FORMAT_MACROS

#include "../log-output.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../ohmd-assert.h"
#include <inttypes.h>

#include "rift-sensor.h"
#include "rift-sensor-pose-search.h"
#include "rift-cv1touch-poses.h"

#include "rift-sensor-blobwatch.h"

#include "sensor/ar0134.h"
#include "sensor/mt9v034.h"
#include "sensor/esp770u.h"
#include "sensor/esp570.h"
#include "sensor/uvc.h"

#include "rift-debug-draw.h"

#include "ohmd-jpeg.h"

#include "ohmd-pipewire.h"

#define ASSERT_MSG(_v, label, ...) if(!(_v)){ ohmd_log_fprintf(stderr, __VA_ARGS__); goto label; }

/* We need 4 capture buffers:
 * 	1 to be capturing into
 * 	1 captured, in the fast analysis thread
 * 	1 possibly undergoing long analysis
 * 	1 pending long analysis
 */
#define NUM_CAPTURE_BUFFERS 4

#define QUEUE_SIZE (NUM_CAPTURE_BUFFERS+1)

typedef struct rift_sensor_frame_queue rift_sensor_frame_queue;

struct rift_sensor_frame_queue {
	rift_sensor_analysis_frame *data[QUEUE_SIZE];
	unsigned int head, tail;
};

struct rift_sensor_ctx_s
{
	ohmd_context* ohmd_ctx;
	int id;
	char serial_no[32];

	rift_tracker_ctx *tracker;

	rift_sensor_device *dev;

	rift_sensor_analysis_frame *frames;
	ohmd_jpeg_decoder *jpeg_decoder;
	/* JPEG images discarded or kept despite warnings since jpeg_window_start, logged at
	 * most every 10 s. Fast analysis thread only. */
	uint32_t jpeg_failed, jpeg_warned;
	uint64_t jpeg_window_start;

	blobwatch* bw;
	rift_pose_finder pf;

	/* cv1touch counters (atomic) */
	uint32_t cv1touch_frames;
	uint64_t cv1touch_frames_received;
	uint32_t cv1touch_pose_fixes[3];

	ohmd_mutex *sensor_lock;

	/* Protected by sensor_lock */
	rift_tracked_device *devices[RIFT_MAX_TRACKED_DEVICES];
	uint8_t n_devices;

	/* Queue of frames being returned to the capture thread */
	rift_sensor_frame_queue capture_frame_q;
	int dropped_frames;
	/* queue of frames awaiting fast analysis */
	rift_sensor_frame_queue fast_analysis_q;
	/* cond that's signalled when a frame is put in the fast queue */
	ohmd_cond *fast_analysis_q_cond;

	/* queue of frames awaiting long analysis */
	rift_sensor_frame_queue long_analysis_q;
	/* cond that's signalled when a frame is put in the long queue */
	ohmd_cond *long_analysis_q_cond;

	int shutdown;

	bool long_analysis_busy;

	/* cv1touch: a camera pose given (or taken away: pending_clear) at runtime, applied by the
	 * fast analysis thread */
	bool have_pending_pose;
	bool pending_clear;
	posef pending_pose;

	/* End protected by sensor_lock */

	ohmd_thread* fast_analysis_thread;
	ohmd_thread* long_analysis_thread;

	/* Pipewire output streams */
	ohmd_pw_video_stream *debug_vid_raw;
	ohmd_pw_video_stream *debug_vid;
	uint8_t *debug_frame;
};

#define INIT_QUEUE(q) \
		(q)->head = (q)->tail = 0;

#define PUSH_QUEUE(q,f) do { \
	unsigned int next = ((q)->tail+1) % QUEUE_SIZE; \
	OHMD_ASSERT(next != (q)->head); /* Check there's room */ \
	OHMD_ASSERT((f) != NULL); \
	(q)->data[(q)->tail] = (f); \
	(q)->tail = next; \
} while(0)

static rift_sensor_analysis_frame *POP_QUEUE(rift_sensor_frame_queue *q) {
	rift_sensor_analysis_frame *f;
	unsigned int next_head = (q->head+1) % QUEUE_SIZE;

	if ((q)->tail == (q)->head) /* Check there's something in the queue */
		return NULL;

	f = q->data[q->head];
	q->head = next_head;

	return f;
}

/* Rewind the queue and un-push the last element */
static rift_sensor_analysis_frame *REWIND_QUEUE(rift_sensor_frame_queue *q) {
	rift_sensor_analysis_frame *f;
	unsigned int prev_tail = (q->tail-1);

	if (prev_tail >= QUEUE_SIZE)
		prev_tail = QUEUE_SIZE-1;

	if (q->tail == q->head) /* Check there's something in the queue */
		return NULL;

	f = q->data[prev_tail];
	q->tail = prev_tail;

	return f;
}

bool
rift_sensor_add_device(rift_sensor_ctx *sensor, rift_tracked_device *device)
{
	bool ret;

	ohmd_lock_mutex(sensor->sensor_lock);
	OHMD_ASSERT(sensor->n_devices < RIFT_MAX_TRACKED_DEVICES);

	ret	= correspondence_search_set_model (sensor->pf.cs, device->id, device->led_search);

	if (ret) {
		sensor->devices[sensor->n_devices] = device;
		sensor->n_devices++;
	}

	ohmd_unlock_mutex(sensor->sensor_lock);

	return ret;
}

/* Called with sensor_lock held. Releases a frame back to the capture thread */
static void
release_capture_frame(rift_sensor_ctx *sensor, rift_sensor_analysis_frame *frame)
{
	uint64_t now = ohmd_monotonic_get(sensor->ohmd_ctx);

	LOGD("Sensor %d Frame %d analysis done after %u ms. Captured %" PRIu64 " capture %u ms fast: queued %f ms analysis %u ms (%ums blob extraction) long: queued %f ms analysis %u ms",
		 sensor->id, frame->id,
		 (uint32_t) (now - frame->vframe->start_ts) / 1000000,
		 frame->vframe->start_ts,
		 (uint32_t) (frame->frame_delivered_ts - frame->vframe->start_ts) / 1000000,
		 (double) (frame->image_analysis_start_ts - frame->frame_delivered_ts) / 1000000.0,
		 (uint32_t) (frame->image_analysis_finish_ts - frame->image_analysis_start_ts) / 1000000,
		 (uint32_t) (frame->blob_extract_finish_ts - frame->image_analysis_start_ts) / 1000000,
		 (double) (frame->long_analysis_start_ts - frame->image_analysis_finish_ts) / 1000000.0,
		 (uint32_t) (frame->long_analysis_finish_ts - frame->long_analysis_start_ts) / 1000000);

	rift_tracker_frame_release (sensor->tracker, now,
			frame->vframe->start_ts, frame->exposure_info_valid ? &frame->exposure_info : NULL,
			sensor->serial_no);

	__atomic_fetch_add(&sensor->cv1touch_frames, 1, __ATOMIC_RELAXED);
	if (frame->bwobs && sensor->ohmd_ctx->cv1touch_frame_cb) {
		ohmd_cv1touch_frame_info info;
		ohmd_cv1touch_blob blobs[MAX_BLOBS_PER_FRAME];
		int n = frame->bwobs->num_blobs;

		memset(&info, 0, sizeof info);
		snprintf(info.serial, sizeof info.serial, "%s", sensor->serial_no);
		info.frame_start_host_ns = frame->vframe->start_ts;
		info.exposure_valid = frame->exposure_info_valid;
		info.exposure_count = frame->exposure_info.count;
		info.exposure_hmd_ts = frame->exposure_info.hmd_ts;
		info.n_devices = frame->n_devices;
		if (info.n_devices > 3)
			info.n_devices = 3;
		if (info.n_devices > sensor->n_devices)
			info.n_devices = sensor->n_devices;
		for (int d = 0; d < (int) info.n_devices; d++) {
			const rift_sensor_frame_device_state *ds = frame->capture_state + d;
			ohmd_cv1touch_frame_device *od = info.devices + d;

			od->device_id = sensor->devices[d]->id;
			od->device_time_ns = frame->exposure_info.devices[d].device_time_ns;
			od->match_flags = ds->score.match_flags;
			od->matched_blobs = ds->score.matched_blobs;
			od->visible_leds = ds->score.visible_leds;
			if (ds->cv1touch_pose_valid) {
				od->pose_valid = 1;
				od->cam_orient[0] = ds->final_cam_pose.orient.x;
				od->cam_orient[1] = ds->final_cam_pose.orient.y;
				od->cam_orient[2] = ds->final_cam_pose.orient.z;
				od->cam_orient[3] = ds->final_cam_pose.orient.w;
				od->cam_pos[0] = ds->final_cam_pose.pos.x;
				od->cam_pos[1] = ds->final_cam_pose.pos.y;
				od->cam_pos[2] = ds->final_cam_pose.pos.z;
			}
		}
		if (n > MAX_BLOBS_PER_FRAME)
			n = MAX_BLOBS_PER_FRAME;
		for (int i = 0; i < n; i++) {
			const struct blob *b = frame->bwobs->blobs + i;
			blobs[i] = (ohmd_cv1touch_blob) { b->x, b->y, b->width, b->height, b->area, b->led_id };
		}
		if (sensor->ohmd_ctx->cv1touch_image_cb && frame->vframe->data && frame->vframe->format == OHMD_VIDEO_FRAME_FORMAT_GRAY8)
			sensor->ohmd_ctx->cv1touch_image_cb(sensor->ohmd_ctx->cv1touch_image_user, &info, frame->vframe->width,
				frame->vframe->height, frame->vframe->stride, frame->vframe->data);
		sensor->ohmd_ctx->cv1touch_frame_cb(sensor->ohmd_ctx->cv1touch_frame_user, &info, n, blobs);
	}
	if (frame->bwobs) {
		blobwatch_release_observation(sensor->bw, frame->bwobs);
		frame->bwobs = NULL;
	}

	ohmd_video_frame_release(frame->vframe);
	frame->vframe = NULL;

	PUSH_QUEUE(&sensor->capture_frame_q, frame);
}

/* Blob extraction reads a packed, complete GRAY8 image. A short image can still
 * contain old pixels in its reused buffer; padding cannot recover missing pixels. */
static bool valid_raw_frame(const rift_sensor_device *dev, const ohmd_video_frame *frame)
{
	if (frame->format != OHMD_VIDEO_FRAME_FORMAT_GRAY8 || frame->data == NULL ||
			frame->width <= 0 || frame->height <= 0 ||
			frame->width != dev->calib.width || frame->height != dev->calib.height ||
			frame->stride != frame->width || (size_t)frame->height > SIZE_MAX / frame->width)
		return false;
	size_t size = (size_t)frame->width * frame->height;
	return frame->data_size == size && frame->data_block_size >= size;
}

/* Called by the rift_sensor_device when a frame is captured. Attach
 * it to a rift_sensor_analysis_frame, retrieve the exposure info
 * from the tracker, and push it on the fast analysis queue
 */
static void frame_captured_cb(rift_sensor_device *dev, ohmd_video_frame *vframe, void *cb_data)
{
	rift_sensor_ctx *sensor = cb_data;
	rift_sensor_analysis_frame *frame;
	OHMD_ASSERT(sensor);
	/* Acquisition progresses even when analysis is still releasing older frames. */
	__atomic_fetch_add(&sensor->cv1touch_frames_received, 1, __ATOMIC_RELAXED);
	uint64_t now = ohmd_monotonic_get(sensor->ohmd_ctx);

	/* The UVC assembler checks raw frames before delivery. Also protect callers
	 * using a different capture backend before taking ownership or queueing work. */
	if (vframe->format != OHMD_VIDEO_FRAME_FORMAT_JPEG && !valid_raw_frame(dev, vframe)) {
		ohmd_video_frame_release(vframe);
		return;
	}

	ohmd_lock_mutex (sensor->sensor_lock);

	frame = POP_QUEUE(&sensor->capture_frame_q);
	/* If there is no queue on the capture frame queue,
	 * then the analysis threads got blocked for too long. Find
	 * and reclaim a frame from the one of the analysis queues */

	/* Take from the fast queue first - it should have woken
	 * up and processed any frame we gave it by now, so if there's
	 * one pending things are quite blocked */
	if (frame == NULL) {
		if ((frame = REWIND_QUEUE(&sensor->fast_analysis_q)) != NULL) {
			release_capture_frame(sensor, frame);
			frame = POP_QUEUE(&sensor->capture_frame_q);
		}
	}

	/* Otherwise try and claim a pending frame from the long
	 * analysis queue */
	if (frame == NULL) {
		if ((frame = REWIND_QUEUE(&sensor->long_analysis_q)) != NULL) {
			release_capture_frame(sensor, frame);
			frame = POP_QUEUE(&sensor->capture_frame_q);
		}
	}

	if (frame == NULL)
		goto no_frame;

	frame->frame_delivered_ts = now;
	frame->vframe = vframe;
	frame->bwobs = NULL;

	frame->exposure_info_valid = rift_tracker_frame_captured (sensor->tracker, now,
		vframe->start_ts, &frame->exposure_info, sensor->serial_no);

	PUSH_QUEUE(&sensor->fast_analysis_q, frame);
	ohmd_cond_signal(sensor->fast_analysis_q_cond);
	ohmd_unlock_mutex(sensor->sensor_lock);
	return;

no_frame:
	ohmd_video_frame_release(vframe);
	ohmd_unlock_mutex(sensor->sensor_lock);
}

static void analyse_frame_fast(rift_sensor_ctx *sensor, rift_sensor_analysis_frame *frame)
{
	uint64_t now = ohmd_monotonic_get(sensor->ohmd_ctx);
	ohmd_video_frame *vframe = frame->vframe;
	int width = vframe->width;
	int height = vframe->height;

	LOGD("Sensor %d Frame %d - starting fast analysis", sensor->id, frame->id);

	frame->image_analysis_start_ts = now;

	blobwatch_process(sensor->bw, vframe->data, width, height,
		frame->exposure_info.led_pattern_phase, NULL, 0, &frame->bwobs);

	frame->blob_extract_finish_ts = ohmd_monotonic_get(sensor->ohmd_ctx);

	if (frame->bwobs && frame->bwobs->num_blobs > 0) {
		rift_pose_finder_process_blobs_fast(&sensor->pf, frame, sensor->devices,
			rift_tracker_get_search_skip(sensor->tracker));
#if 0
		ohmd_log_printf("Sensor %d phase %d Blobs: %d\n", sensor->id, led_pattern_phase, sensor->bwobs->num_blobs);

		for (int index = 0; index < sensor->bwobs->num_blobs; index++)
		{
			ohmd_log_printf("Sensor %d Blob[%d]: %d,%d %dx%d id %d pattern %x age %u\n", sensor->id,
				index,
				sensor->bwobs->blobs[index].x,
				sensor->bwobs->blobs[index].y,
				sensor->bwobs->blobs[index].width,
				sensor->bwobs->blobs[index].height,
				sensor->bwobs->blobs[index].led_id,
				sensor->bwobs->blobs[index].pattern,
				sensor->bwobs->blobs[index].pattern_age);
		}
#endif
	}

	frame->image_analysis_finish_ts = ohmd_monotonic_get(sensor->ohmd_ctx);

	if (ohmd_pw_video_stream_connected(sensor->debug_vid_raw))
		ohmd_pw_video_stream_push (sensor->debug_vid_raw, vframe->start_ts, vframe->data);

	if (ohmd_pw_video_stream_connected(sensor->debug_vid)) {
		rift_debug_draw_frame (sensor->debug_frame, frame->bwobs, sensor->pf.cs, frame,
			frame->n_devices, sensor->devices, &sensor->dev->calib, &sensor->pf.camera_pose);
		ohmd_pw_video_stream_push (sensor->debug_vid, vframe->start_ts, sensor->debug_frame);
	}
}

static unsigned int fast_analysis_thread(void *arg);
static unsigned int long_analysis_thread(void *arg);
static bool handle_found_pose (rift_sensor_ctx *sensor_ctx,
	rift_tracked_device *dev, rift_sensor_analysis_frame *frame,
	posef *obj_world_pose, rift_pose_metrics *score);

rift_sensor_ctx *
rift_sensor_new(ohmd_context* ohmd_ctx, int id, const char *serial_no,
	rift_sensor_device *dev, rift_tracker_ctx *tracker)
{
	rift_sensor_ctx *sensor_ctx = NULL;
	char stream_id[64];
	int i;

	sensor_ctx = ohmd_alloc(ohmd_ctx, sizeof (rift_sensor_ctx));
	if (!sensor_ctx) { rift_sensor_device_free(dev); return NULL; }

	sensor_ctx->ohmd_ctx = ohmd_ctx;
	sensor_ctx->dev = dev;

	sensor_ctx->pf.sensor_id = sensor_ctx->id = id;
	strcpy ((char *) sensor_ctx->serial_no, (char *) serial_no);
	sensor_ctx->tracker = tracker;

	sensor_ctx->long_analysis_busy = false;

	sensor_ctx->sensor_lock = ohmd_create_mutex(ohmd_ctx);
	if (!sensor_ctx->sensor_lock) goto fail;

	/* Initialise frame queues */
	INIT_QUEUE(&sensor_ctx->capture_frame_q);
	INIT_QUEUE(&sensor_ctx->fast_analysis_q);
	INIT_QUEUE(&sensor_ctx->long_analysis_q);

	/* Conds we use to wake queue waiters */
	sensor_ctx->fast_analysis_q_cond = ohmd_create_cond(ohmd_ctx);
	sensor_ctx->long_analysis_q_cond = ohmd_create_cond(ohmd_ctx);
	if (!sensor_ctx->fast_analysis_q_cond || !sensor_ctx->long_analysis_q_cond) goto fail;

	/* Allocate capture frame buffers */
	sensor_ctx->frames = ohmd_alloc(ohmd_ctx, NUM_CAPTURE_BUFFERS * sizeof (rift_sensor_analysis_frame));
	if (!sensor_ctx->frames) goto fail;
	for (i = 0; i < NUM_CAPTURE_BUFFERS; i++) {
			rift_sensor_analysis_frame *frame = sensor_ctx->frames + i;
			frame->id = i;
			/* Send all frames to the capture queue to start */
			PUSH_QUEUE(&sensor_ctx->capture_frame_q, frame);
	}
	sensor_ctx->dropped_frames = 0;

	snprintf(stream_id,64,"openhmd-rift-sensor-%s", sensor_ctx->serial_no);
	stream_id[63] = 0;

	rift_sensor_camera_params *calib = &sensor_ctx->dev->calib;

	sensor_ctx->bw = blobwatch_new(calib->is_cv1 ? BLOB_THRESHOLD_CV1 : BLOB_THRESHOLD_DK2);
	if (!sensor_ctx->bw) goto fail;

	rift_pose_finder_init(&sensor_ctx->pf, calib, (rift_pose_finder_cb) handle_found_pose, sensor_ctx);
	if (!sensor_ctx->pf.cs) goto fail;

	/* cv1touch: a calibrated sensor starts with its fixed pose; with fixed_poses_only, the others
	 * never place themselves and only observe (calibration capture). */
	{
		posef fixed;
		sensor_ctx->pf.cv1touch_allow_placement = !ohmd_ctx->cv1touch_config.fixed_poses_only;
		if (cv1touch_fixed_pose_lookup_list(ohmd_ctx->cv1touch_sensors.configured ? ohmd_ctx->cv1touch_sensors.poses : ohmd_ctx->cv1touch_config.fixed_poses,
                ohmd_ctx->cv1touch_sensors.configured ? ohmd_ctx->cv1touch_sensors.pose_count : 8, sensor_ctx->serial_no, &fixed)) {
			rift_pose_finder_set_camera_pose(&sensor_ctx->pf, &fixed);
			LOGI("Sensor %d (%s) uses its calibrated pose", sensor_ctx->id, sensor_ctx->serial_no);
		}
	}

	/* Raw debug video stream */
	sensor_ctx->debug_vid_raw = ohmd_pw_video_stream_new (stream_id, "Rift Sensor", OHMD_PW_VIDEO_FORMAT_GRAY8,
			calib->width, calib->height, 625, 12);

	/* Annotated debug video stream */
	sensor_ctx->debug_vid = ohmd_pw_video_stream_new (stream_id, "Rift Tracking", OHMD_PW_VIDEO_FORMAT_RGB,
			calib->width * 2, calib->height, 625, 12);

	if (sensor_ctx->debug_vid) {
		/* Allocate an RGB debug frame, twice the width of the input */
		sensor_ctx->debug_frame = ohmd_alloc(ohmd_ctx, 2 * 3 * calib->width * calib->height);
		if (!sensor_ctx->debug_frame) goto fail;
	}

	/* Start analysis threads */
	sensor_ctx->shutdown = false;
	sensor_ctx->fast_analysis_thread = ohmd_create_thread (ohmd_ctx, fast_analysis_thread, sensor_ctx);
	sensor_ctx->long_analysis_thread = ohmd_create_thread (ohmd_ctx, long_analysis_thread, sensor_ctx);
	if (!sensor_ctx->fast_analysis_thread || !sensor_ctx->long_analysis_thread) goto fail;


	LOGV("Sensor %d ready\n", id);

	return sensor_ctx;
fail:
	rift_sensor_free(sensor_ctx);
	return NULL;
}

bool
rift_sensor_start(rift_sensor_ctx *sensor)
{
	return rift_sensor_device_start_video (sensor->dev, NUM_CAPTURE_BUFFERS + 1, frame_captured_cb, sensor);
}

void rift_sensor_request_stop(rift_sensor_ctx *sensor)
{
	rift_sensor_device_request_stop(sensor->dev);
}

void rift_sensor_retry(rift_sensor_ctx *sensor, uint64_t now_ns)
{
	rift_sensor_device_retry(sensor->dev, now_ns);
}

void
rift_sensor_free (rift_sensor_ctx *sensor_ctx)
{
	if (sensor_ctx == NULL)
		return;

	if (sensor_ctx->dev)
		rift_sensor_device_stop_video(sensor_ctx->dev);

	/* Shut down analysis threads */
    if (sensor_ctx->sensor_lock) {
        ohmd_lock_mutex(sensor_ctx->sensor_lock);
        sensor_ctx->shutdown = true;
        if (sensor_ctx->fast_analysis_q_cond) ohmd_cond_broadcast(sensor_ctx->fast_analysis_q_cond);
        if (sensor_ctx->long_analysis_q_cond) ohmd_cond_broadcast(sensor_ctx->long_analysis_q_cond);
        ohmd_unlock_mutex(sensor_ctx->sensor_lock);
    }

	if (sensor_ctx->fast_analysis_thread)
		ohmd_destroy_thread(sensor_ctx->fast_analysis_thread);

	if (sensor_ctx->long_analysis_thread)
		ohmd_destroy_thread(sensor_ctx->long_analysis_thread);

	if (sensor_ctx->sensor_lock)
		ohmd_destroy_mutex(sensor_ctx->sensor_lock);

	if (sensor_ctx->fast_analysis_q_cond)
		ohmd_destroy_cond(sensor_ctx->fast_analysis_q_cond);
	if (sensor_ctx->long_analysis_q_cond)
		ohmd_destroy_cond(sensor_ctx->long_analysis_q_cond);

#if HAVE_LIBJPEG
	if (sensor_ctx->jpeg_decoder)
		ohmd_jpeg_decoder_free(sensor_ctx->jpeg_decoder);
#endif

	if (sensor_ctx->bw)
		blobwatch_free (sensor_ctx->bw);
	if (sensor_ctx->debug_vid_raw != NULL)
		ohmd_pw_video_stream_free (sensor_ctx->debug_vid_raw);
	if (sensor_ctx->debug_vid != NULL)
		ohmd_pw_video_stream_free (sensor_ctx->debug_vid);
	if (sensor_ctx->debug_frame != NULL)
		free (sensor_ctx->debug_frame);

	if (sensor_ctx->dev)
		rift_sensor_device_free(sensor_ctx->dev);

	rift_pose_finder_clear (&sensor_ctx->pf);

	free(sensor_ctx->frames);
	free (sensor_ctx);
}

static rift_sensor_analysis_frame *
get_latest_long_analysis_frame(rift_sensor_ctx *ctx) {
	/* Always drain all pending frames and only analyse the newest */
	rift_sensor_analysis_frame *frame = POP_QUEUE(&ctx->long_analysis_q);

	do {
		rift_sensor_analysis_frame *another_frame = POP_QUEUE(&ctx->long_analysis_q);
		if (another_frame == NULL) {
			break;
		}

		release_capture_frame(ctx, frame);
		frame = another_frame;
	} while (true);

	return frame;
}

static unsigned int long_analysis_thread(void *arg)
{
	rift_sensor_ctx *ctx = arg;
	char name[16];
	snprintf(name, sizeof name, "cv1t-long%d", ctx->id);
	ohmd_set_thread_name(name); /* cv1touch: findable in profiles */

	ohmd_lock_mutex (ctx->sensor_lock);

	rift_sensor_analysis_frame *frame = get_latest_long_analysis_frame(ctx);
	while (!ctx->shutdown) {
			if (frame != NULL) {
				ctx->long_analysis_busy = true;
				ohmd_unlock_mutex (ctx->sensor_lock);

				frame->long_analysis_start_ts = ohmd_monotonic_get(ctx->ohmd_ctx);
				rift_pose_finder_process_blobs_long(&ctx->pf, frame, ctx->devices,
					rift_tracker_get_search_skip(ctx->tracker));
				frame->long_analysis_finish_ts = ohmd_monotonic_get(ctx->ohmd_ctx);

				ohmd_lock_mutex (ctx->sensor_lock);
				ctx->long_analysis_busy = false;

				/* Done with this frame - send it back to the capture thread */
				release_capture_frame(ctx, frame);
				frame = NULL;
			}
			while (!ctx->shutdown && frame == NULL) {
				/* Try and fetch another frame, otherwise sleep */
				if ((frame = get_latest_long_analysis_frame(ctx)) != NULL)
					break;
				ohmd_cond_wait(ctx->long_analysis_q_cond, ctx->sensor_lock);
			}
	}
	ohmd_unlock_mutex (ctx->sensor_lock);

	return 0;
}

#if HAVE_LIBJPEG
#define JPEG_REPORT_NS 10000000000ULL

/* Times come from the captured images, so this reads no clock. */
static void note_jpeg_decode(rift_sensor_ctx *sensor, uint64_t ts, bool decoded)
{
	if (!decoded || ohmd_jpeg_decoder_warnings(sensor->jpeg_decoder) > 0) {
		if (sensor->jpeg_failed + sensor->jpeg_warned == 0)
			sensor->jpeg_window_start = ts;
		if (decoded)
			sensor->jpeg_warned++;
		else
			sensor->jpeg_failed++;
	}
	if (sensor->jpeg_failed + sensor->jpeg_warned == 0 || ts < sensor->jpeg_window_start + JPEG_REPORT_NS)
		return;

	unsigned secs = (unsigned)((ts - sensor->jpeg_window_start + 500000000) / 1000000000);
	const char *message = ohmd_jpeg_decoder_message(sensor->jpeg_decoder);
	if (sensor->jpeg_failed != 0) {
		char kept[64] = "";
		if (sensor->jpeg_warned != 0)
			snprintf(kept, sizeof kept, ", and kept %u despite decoder warnings", sensor->jpeg_warned);
		LOGW("Sensor %d video: discarded %u damaged image%s in the last %u s (%u could not be decoded)%s; latest decoder message: %s",
				sensor->id, sensor->jpeg_failed, sensor->jpeg_failed == 1 ? "" : "s", secs, sensor->jpeg_failed, kept, message);
	} else {
		LOGW("Sensor %d video: kept %u image%s despite decoder warnings in the last %u s; latest decoder message: %s",
				sensor->id, sensor->jpeg_warned, sensor->jpeg_warned == 1 ? "" : "s", secs, message);
	}
	sensor->jpeg_failed = sensor->jpeg_warned = 0;
}

bool decode_frame(rift_sensor_ctx *sensor, rift_sensor_analysis_frame *frame)
{
	ohmd_video_frame *vframe = frame->vframe;
	ohmd_video_frame *out_vframe = NULL;

	if (vframe->format != OHMD_VIDEO_FRAME_FORMAT_JPEG)
		return true;

	if (sensor->jpeg_decoder == NULL) {
		sensor->jpeg_decoder = ohmd_jpeg_decoder_new(sensor->ohmd_ctx);
		if (sensor->jpeg_decoder == NULL) {
			LOGE("Failed to create JPEG decoder");
			return false;
		}
	}

	bool decoded = ohmd_jpeg_decoder_decode(sensor->jpeg_decoder, vframe, &out_vframe);
	note_jpeg_decode(sensor, vframe->start_ts, decoded);
	if (!decoded)
		return false;

	ohmd_video_frame_release(frame->vframe);
	frame->vframe = out_vframe;

	return true;
}
#endif

static bool init_frame_analyse(rift_sensor_ctx *sensor, rift_sensor_analysis_frame *frame)
{

	/* Do initial prep of the frame before we start analysis */
	LOGD ("Sensor %d captured frame %d exposure counter %u phase %d", sensor->id,
		frame->id, frame->exposure_info.count, frame->exposure_info.led_pattern_phase);

	int d;
	const rift_tracker_exposure_info *exposure_info = &frame->exposure_info;

	frame->need_long_analysis = false;
	frame->long_analysis_found_new_blobs = false;
	frame->long_analysis_start_ts = frame->long_analysis_finish_ts = 0;

	for (d = 0; d < exposure_info->n_devices; d++) {
		rift_sensor_frame_device_state *dev_state = frame->capture_state + d;
		const rift_tracked_device_exposure_info *exp_dev_info = exposure_info->devices + d;

		/* Mark the score as un-evaluated to start */
		dev_state->score.match_flags = 0;
		dev_state->found_device_pose = false;
		dev_state->cv1touch_pose_valid = false;

		rift_pose_finder_exp_info_to_dev_state (&sensor->pf, exp_dev_info, dev_state);
	}
	frame->n_devices = exposure_info->n_devices;

#if HAVE_LIBJPEG
	if (!decode_frame(sensor, frame))
		return false;
#endif

	return valid_raw_frame(sensor->dev, frame->vframe);
}

static unsigned int fast_analysis_thread(void *arg)
{
	rift_sensor_ctx *sensor = arg;
	char name[16];
	snprintf(name, sizeof name, "cv1t-fast%d", sensor->id);
	ohmd_set_thread_name(name); /* cv1touch: findable in profiles */

	ohmd_lock_mutex (sensor->sensor_lock);
	rift_sensor_analysis_frame *frame = POP_QUEUE(&sensor->fast_analysis_q);
	while (!sensor->shutdown) {
			if (frame != NULL) {
				/* cv1touch: a pose given at runtime, taken between frames while the long
				 * analysis is not reading the pose finder */
				if ((sensor->have_pending_pose || sensor->pending_clear) && !sensor->long_analysis_busy) {
					if (sensor->pending_clear) {
						sensor->pf.have_camera_pose = false;
						LOGI("Sensor %d (%s) has no pose for now: it labels blobs only", sensor->id, sensor->serial_no);
					} else {
						rift_pose_finder_set_camera_pose(&sensor->pf, &sensor->pending_pose);
						LOGI("Sensor %d (%s) uses a new pose", sensor->id, sensor->serial_no);
					}
					sensor->have_pending_pose = false;
					sensor->pending_clear = false;
				}
				ohmd_unlock_mutex (sensor->sensor_lock);
				if (init_frame_analyse(sensor, frame))
					analyse_frame_fast(sensor, frame);
				ohmd_lock_mutex (sensor->sensor_lock);

				/* Done with this frame - either send it back to the capture thread,
				 * or to the long analysis thread (unless the thread is still busy
				 * processing something else
				 */
				if (frame->need_long_analysis && !sensor->long_analysis_busy) {
					/* If there is an un-fetched frame in the long analysis
					 * queue, steal it back and return that to the capture thread,
					 * then replace it with the new one */
					rift_sensor_analysis_frame *old_frame = REWIND_QUEUE(&sensor->long_analysis_q);
					if (old_frame) {
						uint64_t now = ohmd_monotonic_get(sensor->ohmd_ctx);
						LOGD("Sensor %d reclaimed frame %d from long analysis queue",
							sensor->id, old_frame->id);
						old_frame->long_analysis_start_ts = old_frame->long_analysis_finish_ts = now;
						release_capture_frame(sensor, old_frame);
					}

					PUSH_QUEUE(&sensor->long_analysis_q, frame);
					ohmd_cond_signal(sensor->long_analysis_q_cond);
				} else {
					frame->long_analysis_start_ts = frame->long_analysis_finish_ts = frame->image_analysis_finish_ts;
					release_capture_frame(sensor, frame);
				}

				frame = NULL;
			}

			while (!sensor->shutdown && frame == NULL) {
				/* Try and fetch another frame, otherwise sleep */
				if ((frame = POP_QUEUE(&sensor->fast_analysis_q)) != NULL)
					break;
				ohmd_cond_wait(sensor->fast_analysis_q_cond, sensor->sensor_lock);
			}
	}
	ohmd_unlock_mutex (sensor->sensor_lock);

	return 0;
}

static bool handle_found_pose (rift_sensor_ctx *sensor_ctx,
	rift_tracked_device *dev, rift_sensor_analysis_frame *frame,
	posef *obj_world_pose, rift_pose_metrics *score)
{
	uint64_t now = ohmd_monotonic_get(sensor_ctx->ohmd_ctx);
	LOGD("Sensor %d TS %llu Updating fusion for device %d pose quat %f %f %f %f  pos %f %f %f",
		sensor_ctx->id, (unsigned long long)(now),
		dev->id, obj_world_pose->orient.x, obj_world_pose->orient.y, obj_world_pose->orient.z, obj_world_pose->orient.w,
		obj_world_pose->pos.x, obj_world_pose->pos.y, obj_world_pose->pos.z);

	bool ret = rift_tracked_device_model_pose_update(dev, now, frame->vframe->start_ts, &frame->exposure_info,
	       score, obj_world_pose, sensor_ctx->serial_no);

	if (ret) {
		if (dev->id >= 0 && dev->id < 3)
			__atomic_fetch_add(&sensor_ctx->cv1touch_pose_fixes[dev->id], 1, __ATOMIC_RELAXED);
		/* If this pose was accepted by the tracker, transfer these blob labels to the blobwatch object */
		ohmd_lock_mutex(sensor_ctx->sensor_lock);
		blobwatch_update_labels (sensor_ctx->bw, frame->bwobs, dev->id);
		ohmd_unlock_mutex(sensor_ctx->sensor_lock);
	} else { /* FIXME: Only do this after a few failures? */
		rift_clear_blob_labels (frame->bwobs->blobs, frame->bwobs->num_blobs, dev->id);
	}

	rift_tracked_device_frame_release(dev, &frame->exposure_info);

	return ret;
}

const char *rift_sensor_serial_no (rift_sensor_ctx *sensor)
{
	return sensor->serial_no;
}

bool rift_sensor_set_cv1touch_pose(rift_sensor_ctx *sensor, const ohmd_cv1touch_sensor_pose *pose)
{
	posef fixed;

	if (!cv1touch_fixed_pose_lookup_list(pose, 1, sensor->serial_no, &fixed))
		return false;
	ohmd_lock_mutex(sensor->sensor_lock);
	sensor->pending_pose = fixed;
	sensor->have_pending_pose = true;
	sensor->pending_clear = false;
	ohmd_unlock_mutex(sensor->sensor_lock);
	return true;
}

bool rift_sensor_clear_cv1touch_pose(rift_sensor_ctx *sensor, const char *serial)
{
	if (!cv1touch_sensor_serial_equal(sensor->serial_no, serial))
		return false;
	/* a sensor that may place itself would do so from its own fusion at once: not for it */
	if (sensor->pf.cv1touch_allow_placement)
		return false;
	ohmd_lock_mutex(sensor->sensor_lock);
	sensor->pending_clear = true;
	sensor->have_pending_pose = false;
	ohmd_unlock_mutex(sensor->sensor_lock);
	return true;
}

void rift_sensor_get_video_info(rift_sensor_ctx *sensor, ohmd_cv1touch_sensor_video *out)
{
	memset(out, 0, sizeof *out);
	snprintf(out->serial, sizeof out->serial, "%s", sensor->serial_no);
	out->frames_received = __atomic_load_n(&sensor->cv1touch_frames_received, __ATOMIC_RELAXED);
}

void rift_sensor_get_cv1touch_info(rift_sensor_ctx *sensor, ohmd_cv1touch_sensor *out)
{
	posef pose;

	memset(out, 0, sizeof *out);
	snprintf(out->serial, sizeof out->serial, "%s", sensor->serial_no);
	/* The provisional placement writes pf without this lock; Phase 3 replaces it with a fixed pose. */
	ohmd_lock_mutex(sensor->sensor_lock);
	out->have_camera_pose = sensor->pf.have_camera_pose;
	pose = sensor->pf.camera_pose;
	ohmd_unlock_mutex(sensor->sensor_lock);
	out->camera_orient[0] = pose.orient.x;
	out->camera_orient[1] = pose.orient.y;
	out->camera_orient[2] = pose.orient.z;
	out->camera_orient[3] = pose.orient.w;
	out->camera_pos[0] = pose.pos.x;
	out->camera_pos[1] = pose.pos.y;
	out->camera_pos[2] = pose.pos.z;
	const rift_sensor_camera_params *calib = &sensor->dev->calib;
	out->width = calib->width;
	out->height = calib->height;
	out->fx = calib->camera_matrix.m[0];
	out->fy = calib->camera_matrix.m[4];
	out->cx = calib->camera_matrix.m[2];
	out->cy = calib->camera_matrix.m[5];
	for (int i = 0; i < 4; i++)
		out->k[i] = calib->dist_coeffs[i];
	out->frames = __atomic_load_n(&sensor->cv1touch_frames, __ATOMIC_RELAXED);
	for (int i = 0; i < 3; i++)
		out->pose_fixes[i] = __atomic_load_n(&sensor->cv1touch_pose_fixes[i], __ATOMIC_RELAXED);
}

void rift_sensor_get_stream_info(rift_sensor_ctx *sensor, ohmd_cv1touch_sensor_stream *out)
{
	/* Starts and diagnostic reads both run on the engine thread. */
	*out = sensor->dev->stream_status;
	snprintf(out->serial, sizeof out->serial, "%s", sensor->serial_no);
}
