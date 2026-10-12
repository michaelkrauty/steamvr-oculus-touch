// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* The engine's brute-force search leaves out the devices in the search-skip mask (held by
 * cv1touch's tracker, or asleep): a frame goes to long analysis only for the other devices, and
 * long analysis searches only for them. A unity build of the pose finder whose pose evaluation and
 * correspondence search are stubs: no device matches its prior and every search fails, as for a
 * frame of stray blobs. */
#include <stdlib.h>
#include <string.h>

#include "omath.c"
#include "rift-sensor-pose-search.c"
#include "test_util.h"

static int searches[RIFT_MAX_TRACKED_DEVICES];

bool rift_tracked_device_get_latest_exposure_info_pose(rift_tracked_device *dev, rift_tracked_device_exposure_info *dev_info)
{
	(void)dev;
	dev_info->fusion_slot = 0;
	dev_info->capture_pose.orient.w = 1.0f;
	return true;
}

void rift_evaluate_pose_with_prior(rift_pose_metrics *score, posef *pose, bool prior_must_match, posef *pose_prior,
	const vec3f *pos_error_thresh, const vec3f *rot_error_thresh, struct blob *blobs, int num_blobs, int device_id,
	rift_leds *leds_model, rift_sensor_camera_params *calib, rift_rect_t *out_bounds)
{
	(void)pose, (void)prior_must_match, (void)pose_prior, (void)pos_error_thresh, (void)rot_error_thresh;
	(void)blobs, (void)num_blobs, (void)device_id, (void)leds_model, (void)calib, (void)out_bounds;
	memset(score, 0, sizeof *score);
}

void rift_evaluate_pose(rift_pose_metrics *score, posef *pose, struct blob *blobs, int num_blobs, int device_id,
	rift_leds *leds_model, rift_sensor_camera_params *calib, rift_rect_t *out_bounds)
{
	(void)pose, (void)blobs, (void)num_blobs, (void)device_id, (void)leds_model, (void)calib, (void)out_bounds;
	memset(score, 0, sizeof *score);
}

void correspondence_search_set_blobs(correspondence_search_t *cs, struct blob *blobs, int num_blobs)
{
	(void)cs, (void)blobs, (void)num_blobs;
}

bool correspondence_search_find_one_pose(correspondence_search_t *cs, int model_id, CorrespondenceSearchFlags search_flags,
	posef *pose, vec3f *pos_error_thresh, vec3f *rot_error_thresh, vec3f *gravity_vector, float gravity_tolerance_rad,
	rift_pose_metrics *score)
{
	(void)cs, (void)search_flags, (void)pose, (void)pos_error_thresh, (void)rot_error_thresh, (void)gravity_vector;
	(void)gravity_tolerance_rad;
	searches[model_id]++;
	memset(score, 0, sizeof *score);
	return false;
}

/* Reached only through a found pose, which the stubs never give. */
void rift_clear_blob_labels(struct blob *blobs, int num_blobs, int device_id)
{
	(void)blobs, (void)num_blobs, (void)device_id;
	abort();
}

void rift_mark_matching_blobs(posef *pose, struct blob *blobs, int num_blobs, int device_id, rift_leds *leds_model,
	rift_sensor_camera_params *calib)
{
	(void)pose, (void)blobs, (void)num_blobs, (void)device_id, (void)leds_model, (void)calib;
	abort();
}

/* The left (2) and the right (1) controller, in that order (the mask goes by device id, not by
 * slot), neither found by its prior, and six unlabelled blobs. */
static rift_tracked_device devices[2] = {{.id = 2}, {.id = 1}};
static rift_tracked_device *devs[2] = {&devices[0], &devices[1]};
static blobservation blobs;
static rift_sensor_analysis_frame frame;
static rift_pose_finder pf;

static void reset(void)
{
	memset(&frame, 0, sizeof frame);
	memset(searches, 0, sizeof searches);
	frame.n_devices = 2;
	frame.bwobs = &blobs;
}

int main(void)
{
	blobs.num_blobs = 6;
	for (int i = 0; i < blobs.num_blobs; i++)
		blobs.blobs[i].led_id = -1;
	/* a sensor with no pose yet: every frame goes to long analysis, which searches for every
	 * device, held or not (its blob labels place the sensor) */
	reset();
	rift_pose_finder_process_blobs_fast(&pf, &frame, devs, 0x6);
	CHECK(frame.need_long_analysis);
	rift_pose_finder_process_blobs_long(&pf, &frame, devs, 0x6);
	CHECK(searches[1] > 0 && searches[2] > 0);

	posef camera = {.orient = {{0, 0, 0, 1}}};
	rift_pose_finder_set_camera_pose(&pf, &camera);

	/* nothing skipped: long analysis, which searches for both */
	reset();
	rift_pose_finder_process_blobs_fast(&pf, &frame, devs, 0);
	CHECK(frame.need_long_analysis);
	rift_pose_finder_process_blobs_long(&pf, &frame, devs, 0);
	CHECK(searches[1] > 0 && searches[2] > 0);

	/* the right controller held: still long analysis, for the left only */
	reset();
	rift_pose_finder_process_blobs_fast(&pf, &frame, devs, 1u << 1);
	CHECK(frame.need_long_analysis);
	rift_pose_finder_process_blobs_long(&pf, &frame, devs, 1u << 1);
	CHECK(searches[1] == 0 && searches[2] > 0);

	/* both held: no long analysis, and no search if it ran anyway */
	reset();
	rift_pose_finder_process_blobs_fast(&pf, &frame, devs, 0x6);
	CHECK(!frame.need_long_analysis);
	rift_pose_finder_process_blobs_long(&pf, &frame, devs, 0x6);
	CHECK(searches[1] == 0 && searches[2] == 0);

	printf("test_search_skip ok\n");
	return 0;
}
