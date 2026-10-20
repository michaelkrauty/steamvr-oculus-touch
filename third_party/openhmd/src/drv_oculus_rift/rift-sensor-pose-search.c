/*
 * Rift sensor interface
 * Copyright 2014-2015 Philipp Zabel
 * Copyright 2019 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */

#define _GNU_SOURCE
#define __STDC_FORMAT_MACROS

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../ohmd-assert.h"
#include <inttypes.h>

#include "rift-tracker.h"
#include "rift-sensor-device.h"
#include "rift-sensor-pose-search.h"

#include "rift-sensor-blobwatch.h"

#include "rift-sensor-maths.h"
#include "rift-sensor-opencv.h"

/* cv1touch provisional sensor placement (replaced by fixed calibration in Phase 3). The headset is
 * normally untracked, so a strongly matched controller places a sensor. Only the first sensor may
 * use an unanchored controller; later sensors need a controller that currently has optical pose
 * lock, so every sensor lands in the same world frame. */
static int cv1touch_placed_sensors;

/* Called when a new tracker is created (e.g. after the daemon reopens the headset). */
void rift_pose_finder_cv1touch_reset(void)
{
	__atomic_store_n(&cv1touch_placed_sensors, 0, __ATOMIC_RELEASE);
}

static bool cv1touch_may_place(const rift_tracked_device_exposure_info *exp_dev_info)
{
	int expected = 0;

	if (__atomic_compare_exchange_n(&cv1touch_placed_sensors, &expected, 1, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return true;
	if (exp_dev_info->had_pose_lock) {
		__atomic_fetch_add(&cv1touch_placed_sensors, 1, __ATOMIC_ACQ_REL);
		return true;
	}
	return false;
}

static int update_device_and_blobs (rift_pose_finder *pf, rift_sensor_analysis_frame *frame,
	rift_tracked_device *dev, rift_sensor_frame_device_state *dev_state,
	rift_tracked_device_exposure_info *exp_dev_info,
	posef *obj_cam_pose);

void rift_pose_finder_init(rift_pose_finder *pf, rift_sensor_camera_params *calib,
		rift_pose_finder_cb pose_cb, void *pose_cb_data)
{
	pf->have_camera_pose = false;
	pf->cv1touch_allow_placement = true;
	pf->calib = calib;
	pf->cs = correspondence_search_new(calib);

	pf->pose_cb = pose_cb;
	pf->pose_cb_data = pose_cb_data;
}

/* Sets the sensor's camera -> world pose (provisional placement or a calibrated fixed pose). */
void rift_pose_finder_set_camera_pose(rift_pose_finder *pf, const posef *camera_pose)
{
	const vec3f gravity_vector = {{ 0.0, 1.0, 0.0 }};
	quatf cam_orient = camera_pose->orient;

	pf->camera_pose = *camera_pose;
	oquatf_inverse(&cam_orient);
	oquatf_get_rotated(&cam_orient, &gravity_vector, &pf->cam_gravity_vector);
	pf->have_camera_pose = true;
}

static float inspection_turn(quatf rotation)
{
	oquatf_normalize_me(&rotation);
	return 2.0f * acosf(OHMD_MIN(1.0f, fabsf(rotation.w)));
}

/* A thirty-degree turn leaves headroom for PnP noise. Three exposures within three seconds
 * must each follow this hand's IMU to a tenth of its turn. */
#define IDENTITY_WINDOW_NS 3000000000ULL
#define IDENTITY_MIN_TURN DEG_TO_RAD(30)
#define IDENTITY_AGREEING 3
#define IDENTITY_TOLERANCE 0.1f
/* A fixed alias of the other hand, at whatever offset, turns by exactly the other hand's
 * angle. Its turn must differ from this hand's by half of this hand's turn, which leaves
 * at least four tenths of it between the agreeing camera turn and any such alias. */
#define IDENTITY_DISTINCT 0.5f
/* In replay, genuine exposures followed their hand's IMU to 9.4 degrees at most (4,989 of
 * them, 1,410 on identified streams), and beyond 7 degrees at most twice in a row. A mirror
 * alias is about 120 degrees off. */
#define IDENTITY_CONTRADICTION_FLOOR DEG_TO_RAD(10)
#define IDENTITY_CONTRADICTIONS 3

/* Blobwatch may transfer an ID across any distance. Inspection lineage also requires every
 * raw-frame step to stay within 32 px: 99.84% of 3.16 million same-labelled replay pairs did.
 * This rejects jumps, not physical swaps at an image overlap. Those are caught only once
 * the hands' rotations diverge; identical motion can preserve the swap until it diverges.
 * A frame gap over three nominal camera periods starts a new epoch. */
#define INSPECTION_FRAME_GAP_NS 60000000ULL
#define INSPECTION_STEP_PX 32.0f

static void inspection_raw_tracks(rift_pose_finder *pf, rift_sensor_analysis_frame *frame)
{
	const blobservation *raw = frame->bwobs;
	for (int b = 0; b < raw->num_blobs; b++) {
		const struct blob *blob = &raw->blobs[b];
		uint64_t track = 0;
		for (int p = 0; p < pf->inspection_previous_raw.num_blobs; p++) {
			const struct blob *previous = &pf->inspection_previous_raw.blobs[p];
			if (blob->blob_id != previous->blob_id) continue;
			float dx = blob->x - previous->x, dy = blob->y - previous->y;
			if (dx * dx + dy * dy <= INSPECTION_STEP_PX * INSPECTION_STEP_PX)
				track = pf->inspection_tracks[p];
			break;
		}
		frame->inspection_tracks[b] = track ? track : ++pf->inspection_next_track;
	}
	pf->inspection_previous_raw = *raw;
	memcpy(pf->inspection_tracks, frame->inspection_tracks, sizeof pf->inspection_tracks);
}

rift_correspondence_verdict rift_pose_finder_check_inspection_blobs(rift_pose_finder *pf, int device_id,
    const rift_sensor_analysis_frame *frame)
{
	rift_inspection_identity *id = &pf->inspection_identity[device_id];
	unsigned common = 0, n = 0;
	uint32_t blobs[MAX_BLOBS_PER_FRAME];
	int leds[MAX_BLOBS_PER_FRAME];
	uint64_t tracks[MAX_BLOBS_PER_FRAME];
	for (int b = 0; b < frame->bwobs->num_blobs; b++) {
		const struct blob *blob = &frame->bwobs->blobs[b];
		if (LED_OBJECT_ID(blob->led_id) != device_id)
			continue;
		blobs[n] = blob->blob_id;
		tracks[n] = frame->inspection_tracks[b];
		leds[n++] = blob->led_id;
		for (unsigned prev = 0; prev < id->n_blobs; prev++) {
			if (blob->blob_id == id->blob_ids[prev] && blob->led_id == id->led_ids[prev] &&
			    frame->inspection_tracks[b] == id->tracks[prev]) {
				common++;
				break;
			}
		}
	}
	/* A failed fit or a cold search can propose the other hand while this hand's trusted
	 * blob tracks are still present. Reject that alternative without replacing the stream.
	 * Once those raw tracks are lost, any reacquisition must identify itself from scratch. */
	if (id->validated && common < 5 && id->epoch == frame->inspection_epoch && frame->inspection_raw_valid) {
		unsigned trusted = 0;
		for (int b = 0; b < frame->inspection_raw.num_blobs; b++) {
			const struct blob *blob = &frame->inspection_raw.blobs[b];
			for (unsigned prev = 0; prev < id->n_blobs; prev++) {
				if (blob->blob_id == id->blob_ids[prev] && blob->led_id == id->led_ids[prev] &&
				    frame->inspection_tracks[b] == id->tracks[prev]) {
					trusted++;
					break;
				}
			}
		}
		if (trusted >= 5)
			return RIFT_CORRESPONDENCE_UNRELATED;
	}
	bool broken = id->first_ns && (common < 5 || id->epoch != frame->inspection_epoch);
	if (broken) {
		/* Preserve last_ns: an older in-flight search still cannot replace this stream. */
		id->first_ns = 0;
		id->validated = false;
		id->agreeing = id->contradicting = 0;
	}
	id->epoch = frame->inspection_epoch;
	id->n_blobs = n;
	memcpy(id->blob_ids, blobs, n * sizeof blobs[0]);
	memcpy(id->led_ids, leds, n * sizeof leds[0]);
	memcpy(id->tracks, tracks, n * sizeof tracks[0]);
	return broken ? RIFT_CORRESPONDENCE_BROKEN : RIFT_CORRESPONDENCE_CONTINUOUS;
}

/* Called under the sensor lock by either analysis worker, before exporting or transferring
 * labels. Both IMU poses refer to this same exposure, not the time the search completed.
 * Every call consumes its exposure, so a later search of the same exposure is stale. */
rift_identity_verdict rift_pose_finder_identify_inspection(rift_pose_finder *pf, int device_id,
    uint64_t exposure_ns, const quatf *own, const quatf *other, const quatf *camera, bool tracked,
    bool *judgeable)
{
	rift_inspection_identity *id = &pf->inspection_identity[device_id];
	rift_identity_verdict verdict = RIFT_IDENTITY_UNKNOWN;
	*judgeable = false;
	if (!exposure_ns || exposure_ns <= id->last_ns)
		return RIFT_IDENTITY_UNKNOWN;
	id->last_ns = exposure_ns;
	float own_norm = oquatf_get_dot(own, own), seen_norm = oquatf_get_dot(camera, camera);
	float other_norm = other ? oquatf_get_dot(other, other) : 1.0f;
	if (!isfinite(own_norm) || !isfinite(seen_norm) || !isfinite(other_norm) ||
	    own_norm < 0.5f || seen_norm < 0.5f || other_norm < 0.5f)
		return RIFT_IDENTITY_UNKNOWN;
	if (!id->first_ns)
		goto anchor;
	quatf own_delta, seen_delta, error;
	oquatf_diff(&id->own, own, &own_delta);
	oquatf_diff(&id->camera, camera, &seen_delta);
	oquatf_diff(&own_delta, &seen_delta, &error);
	float span = inspection_turn(own_delta);
	float own_error = inspection_turn(error);
	bool tight = own_error <= IDENTITY_TOLERANCE * OHMD_MAX(span, IDENTITY_MIN_TURN);
	if (id->validated) {
		/* Stillness, a slow turn or a return toward the anchor is no new evidence. Only
		 * the tracked correspondences disagreeing with this hand's IMU, repeatedly, revoke
		 * the identity: a cold search finding the other hand while this one is out of view
		 * says nothing about them. */
		*judgeable = true;
		if (own_error > OHMD_MAX(IDENTITY_TOLERANCE * span, IDENTITY_CONTRADICTION_FLOOR)) {
			if (!tracked || ++id->contradicting < IDENTITY_CONTRADICTIONS)
				return RIFT_IDENTITY_UNKNOWN;
			verdict = RIFT_IDENTITY_CONTRADICTS;
			goto anchor;
		}
		id->contradicting = 0;
		/* Bound IMU drift: roll the anchor onto an exposure that agrees as tightly as a
		 * validating one. An outlier never becomes the anchor. */
		if (tight && exposure_ns - id->first_ns > IDENTITY_WINDOW_NS) {
			id->first_ns = exposure_ns;
			id->own = *own;
			id->camera = *camera;
		}
		return RIFT_IDENTITY_AGREES;
	}
	if (exposure_ns - id->first_ns > IDENTITY_WINDOW_NS || id->have_other != (other != NULL))
		goto anchor;
	bool distinct = true;
	if (other) {
		quatf other_delta;
		oquatf_diff(&id->other, other, &other_delta);
		distinct = fabsf(span - inspection_turn(other_delta)) >= IDENTITY_DISTINCT * span;
	}
	*judgeable = span >= IDENTITY_MIN_TURN && distinct;
	if (!tight) {
		/* An untrusted fast track that fails the motion check must make room for another
		 * cold candidate. Otherwise a persistent alias can occupy the whole inspection.
		 * Only the fast worker owns this private track; cold results cannot stop it. */
		if (tracked) pf->inspection_candidate[device_id].active = false;
		goto anchor;
	}
	if (span < IDENTITY_MIN_TURN)
		return RIFT_IDENTITY_UNKNOWN;
	/* Even in a rigid rifle grip, the other hand must have turned clearly differently. */
	if (!distinct)
		goto anchor;
	if (++id->agreeing < IDENTITY_AGREEING)
		return RIFT_IDENTITY_UNKNOWN;
	id->validated = true;
	id->contradicting = 0;
	return RIFT_IDENTITY_AGREES;
anchor:
	id->first_ns = exposure_ns;
	id->own = *own;
	id->camera = *camera;
	id->have_other = other != NULL;
	if (other) id->other = *other;
	id->validated = false;
	id->agreeing = id->contradicting = 0;
	return verdict;
}

void rift_pose_finder_set_inspection_pose(rift_pose_finder *pf, int device_id,
	const posef *capture_world_pose, const posef *observed_cam_pose)
{
	if (!(pf->inspection_mask & (1u << device_id)))
		return;
	posef camera_object_pose = *observed_cam_pose;
	oposef_inverse(&camera_object_pose);
	oposef_apply(&camera_object_pose, capture_world_pose, &pf->inspection_poses[device_id]);
	pf->inspection_pose_mask |= 1u << device_id;
}

void rift_pose_finder_set_inspection_candidate(rift_pose_finder *pf, int device_id,
    const blobservation *labels, uint64_t epoch, const uint64_t *tracks,
    const posef *capture_world_pose, const posef *observed_cam_pose)
{
	if (!(pf->inspection_mask & (1u << device_id)))
		return;
	rift_inspection_candidate *candidate = &pf->inspection_candidate[device_id];
	candidate->labels = *labels;
	memcpy(candidate->tracks, tracks, sizeof candidate->tracks);
	candidate->epoch = epoch;
	posef camera_object_pose = *observed_cam_pose;
	oposef_inverse(&camera_object_pose);
	oposef_apply(&camera_object_pose, capture_world_pose, &candidate->world_from_cam);
	candidate->active = true;
}

/* A quarantined track is usable only while five of its exact blob/LED pairs persist. Blob
 * IDs are propagated by consecutive raw observations; a new camera epoch breaks the chain. */
static bool inspection_candidate_labels(rift_inspection_candidate *candidate,
    const blobservation *raw, uint64_t epoch, const uint64_t *tracks, int device_id, blobservation *out)
{
	if (candidate->epoch != epoch)
		return false;
	*out = *raw;
	rift_clear_blob_labels(out->blobs, out->num_blobs, device_id);
	unsigned common = 0;
	for (int b = 0; b < out->num_blobs; b++) {
		struct blob *blob = &out->blobs[b];
		if (blob->led_id >= 0) continue; /* another hand's live labels take precedence */
		for (int prev = 0; prev < candidate->labels.num_blobs; prev++) {
			const struct blob *old = &candidate->labels.blobs[prev];
			if (blob->blob_id == old->blob_id && LED_OBJECT_ID(old->led_id) == device_id &&
			    tracks[b] == candidate->tracks[prev]) {
				blob->led_id = old->led_id;
				common++;
				break;
			}
		}
	}
	return common >= 5;
}

bool rift_pose_finder_filter_inspection_labels(rift_pose_finder *pf, int device_id,
    blobservation *labels, const uint64_t *tracks)
{
	unsigned common = 0;
	for (int b = 0; b < labels->num_blobs; b++) {
		struct blob *blob = &labels->blobs[b];
		if (LED_OBJECT_ID(blob->led_id) != device_id) continue;
		bool survives = false;
		for (int p = 0; p < pf->inspection_previous_raw.num_blobs; p++)
			if (blob->blob_id == pf->inspection_previous_raw.blobs[p].blob_id &&
			    tracks[b] == pf->inspection_tracks[p]) {
				survives = true;
				break;
			}
		if (survives) common++;
		else blob->led_id = LED_INVALID_ID;
	}
	return common >= 5;
}

void rift_pose_finder_clear(rift_pose_finder *pf)
{
	if (pf->cs != NULL)
		correspondence_search_free(pf->cs);
}

void rift_pose_finder_exp_info_to_dev_state (rift_pose_finder *pf,
    const rift_tracked_device_exposure_info *exp_dev_info, rift_sensor_frame_device_state *dev_state)
{
		oposef_apply_inverse(&exp_dev_info->capture_pose, &pf->camera_pose, &dev_state->capture_cam_pose);

		/* Compute gravity error from XZ error range */
		const vec3f *rot_error = &exp_dev_info->rot_error;
		dev_state->gravity_error_rad = OHMD_MAX(rot_error->x, rot_error->z);

		/* Calculate camera-relative prior error bounds */
		quatf cam_orient_inverse = pf->camera_pose.orient;
		oquatf_inverse(&cam_orient_inverse);

		oquatf_get_rotated_abs(&cam_orient_inverse, &exp_dev_info->rot_error, &dev_state->obj_cam_rot_error);
#if 0
		oquatf_get_rotated_abs(&cam_orient_inverse, &exp_dev_info->pos_error, &dev_state->obj_cam_pos_error);
#else
		dev_state->obj_cam_pos_error = exp_dev_info->pos_error;
#endif

#if 0
		/* TEST: rot error to at least 20 degrees */
		int a;
		for (a = 0; a < 3; a++) {
			if (dev_state->obj_cam_rot_error.arr[a] < DEG_TO_RAD(20))
				dev_state->obj_cam_rot_error.arr[a] = DEG_TO_RAD(20);
		}
#endif
}

void rift_pose_finder_process_blobs_fast(rift_pose_finder *pf,
        rift_sensor_analysis_frame *frame,
        rift_tracked_device **devs, uint32_t skip_mask)
{
	rift_tracker_exposure_info *exposure_info = &frame->exposure_info;
	blobservation* bwobs = frame->bwobs;
	if (pf->inspection_mask) {
		/* Include empty observations: a later reacquisition cannot bridge a camera outage. */
		uint64_t at = frame->vframe ? frame->vframe->start_ts : 0;
		if (at && (at <= pf->inspection_frame_ns || at - pf->inspection_frame_ns > INSPECTION_FRAME_GAP_NS))
			pf->inspection_epoch++;
		pf->inspection_frame_ns = at;
		frame->inspection_epoch = pf->inspection_epoch;
		frame->inspection_raw_valid = bwobs != NULL;
		if (bwobs) {
			inspection_raw_tracks(pf, frame);
			frame->inspection_raw = *bwobs;
		}
	} else {
		frame->inspection_raw_valid = false;
	}
	if (!bwobs || bwobs->num_blobs == 0)
		return;
	bool have_unfound_devices = false;
	int used_blobs = 0; /* Track how many blobs were matched to devices */
	int d;

	/* If we don't have a camera pose yet, skip straight to correspondence
	 * search */
	if (!pf->have_camera_pose) {
		LOGV("Need camera pose - marking frame for long analysis");
		frame->need_long_analysis = true;
		return;
	}

	/* Only process the devices that were available when this frame was captured */
	for (d = 0; d < frame->n_devices; d++) {
		rift_tracked_device_exposure_info *exp_dev_info = exposure_info->devices + d;
		rift_tracked_device *dev = devs[d];
		rift_sensor_frame_device_state *dev_state = frame->capture_state + d;

		/* Get the latest pose estimate for this device */
		if (!rift_tracked_device_get_latest_exposure_info_pose(dev, exp_dev_info)) {
			LOGV ("Skipping fast analysis of device %d. No fusion slot assigned", d);
			continue;
		}
		if (exp_dev_info->fusion_slot == -1) {
			LOGV ("Sensor %d Skipping fast analysis of device %d. No fusion slot assigned", pf->sensor_id, dev->id);
			continue;
		}
		blobservation quarantined;
		blobservation *raw = frame->bwobs;
		rift_inspection_candidate *candidate = &pf->inspection_candidate[dev->id];
		if ((pf->inspection_mask & (1u << dev->id)) && candidate->active) {
			if (inspection_candidate_labels(candidate, raw, frame->inspection_epoch, frame->inspection_tracks, dev->id, &quarantined)) {
				frame->bwobs = bwobs = &quarantined;
				frame->quarantined_observation = true;
			} else {
				candidate->active = false;
			}
		}
		rift_pose_finder_exp_info_to_dev_state (pf, exp_dev_info, dev_state);
		if (pf->inspection_pose_mask & pf->inspection_mask & (1u << dev->id))
			oposef_apply_inverse(&exp_dev_info->capture_pose, &pf->inspection_poses[dev->id], &dev_state->capture_cam_pose);

		if (frame->quarantined_observation)
			oposef_apply_inverse(&exp_dev_info->capture_pose, &candidate->world_from_cam, &dev_state->capture_cam_pose);

		/* Start from the capture_cam_pose */
		posef obj_cam_pose = dev_state->capture_cam_pose;

		LOGV ("Sensor %d Fusion provided world pose for device %d, %f %f %f %f pos %f %f %f "
			"pos_err_limit %f %f %f rot_err_limit %f %f %f had_pose_lock %d for %" PRId64 "ms",
			pf->sensor_id, dev->id,
			exp_dev_info->capture_pose.orient.x, exp_dev_info->capture_pose.orient.y,
			exp_dev_info->capture_pose.orient.z, exp_dev_info->capture_pose.orient.w,
			exp_dev_info->capture_pose.pos.x, exp_dev_info->capture_pose.pos.y, exp_dev_info->capture_pose.pos.z,
			exp_dev_info->pos_error.x, exp_dev_info->pos_error.y, exp_dev_info->pos_error.z,
			exp_dev_info->rot_error.x, exp_dev_info->rot_error.y, exp_dev_info->rot_error.z,
			exp_dev_info->had_pose_lock,
			exp_dev_info->had_pose_lock ? (exp_dev_info->device_time_ns - exp_dev_info->last_acquired_pose_lock_ts) / 1000000 : -1);

		LOGD ("Sensor %d Frame %d searching for matching cam pose for device %d, prior quat %f %f %f %f pos %f %f %f "
			"pos_err_limit %f %f %f rot_err_limit %f %f %f",
			pf->sensor_id, frame->id, dev->id,
			obj_cam_pose.orient.x, obj_cam_pose.orient.y, obj_cam_pose.orient.z, obj_cam_pose.orient.w,
			obj_cam_pose.pos.x, obj_cam_pose.pos.y, obj_cam_pose.pos.z,
			dev_state->obj_cam_pos_error.x, dev_state->obj_cam_pos_error.y, dev_state->obj_cam_pos_error.z,
			dev_state->obj_cam_rot_error.x, dev_state->obj_cam_rot_error.y, dev_state->obj_cam_rot_error.z);

		rift_evaluate_pose_with_prior(&dev_state->score, &obj_cam_pose,
			true, &dev_state->capture_cam_pose, &dev_state->obj_cam_pos_error, &dev_state->obj_cam_rot_error,
			frame->bwobs->blobs, frame->bwobs->num_blobs,
			dev->id, dev->leds, pf->calib, NULL);

		if (POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_GOOD|RIFT_POSE_MATCH_LED_IDS)) {
			LOGD("Sensor %d already had good pose match for device %d match_flags 0x%x matched %u blobs of %u visible LEDs",
			    pf->sensor_id, dev->id, dev_state->score.match_flags,
			    dev_state->score.matched_blobs, dev_state->score.visible_leds);
		} else {
			int num_blobs = 0;

			LOGD("Sensor %d no expected pose match for device %d match_flags 0x%x matched %u blobs of %u visible LEDs",
			    pf->sensor_id, dev->id, dev_state->score.match_flags,
			    dev_state->score.matched_blobs, dev_state->score.visible_leds);

			/* See if we still have enough labelled blobs to try to re-acquire the pose without a
			 * full search */
			for (int index = 0; index < bwobs->num_blobs; index++) {
				struct blob *b = bwobs->blobs + index;
				if (LED_OBJECT_ID (b->led_id) == dev->id) {
						num_blobs++;
				}
			}

			if (num_blobs > 4) {
				LOGD("Sensor %d trying to reacquire device %d from %u blobs (of %u expected). Prior only matched %d",
					pf->sensor_id, dev->id, num_blobs, dev_state->score.visible_leds, dev_state->score.matched_blobs);

				estimate_initial_pose (bwobs->blobs, bwobs->num_blobs,
					dev->id, dev->leds->points, dev->leds->num_points, pf->calib,
					&obj_cam_pose, NULL, NULL, true);

				if (pf->inspection_mask & (1u << dev->id))
					rift_evaluate_pose(&dev_state->score, &obj_cam_pose,
						bwobs->blobs, bwobs->num_blobs, dev->id, dev->leds, pf->calib, NULL);
				else
					rift_evaluate_pose_with_prior(&dev_state->score, &obj_cam_pose,
						true, &dev_state->capture_cam_pose, &dev_state->obj_cam_pos_error, &dev_state->obj_cam_rot_error,
						bwobs->blobs, bwobs->num_blobs,
						dev->id, dev->leds, pf->calib,
						NULL);

				if (POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_GOOD)) {
					LOGD("Sensor %d re-acquired match for device %d matched %u blobs of %u - quat %f %f %f %f pos %f %f %f",
						pf->sensor_id, dev->id, dev_state->score.matched_blobs, dev_state->score.visible_leds,
						obj_cam_pose.orient.x, obj_cam_pose.orient.y, obj_cam_pose.orient.z, obj_cam_pose.orient.w,
						obj_cam_pose.pos.x, obj_cam_pose.pos.y, obj_cam_pose.pos.z
					);
				}
#if LOGLEVEL <= 1
				else {
					quatf orient_diff;

					oquatf_diff(&obj_cam_pose.orient, &dev_state->capture_cam_pose.orient, &orient_diff);
					oquatf_normalize_me(&orient_diff);

					LOGD("Sensor %d device %d had %d prior blobs, but failed match with flags 0x%x. Yielded pose %f %f %f %f pos %f %f %f (match %d of %d visible) rot_error %f %f %f pos_error %f %f %f orient_diff %f %f %f %f",
						pf->sensor_id, dev->id, num_blobs, dev_state->score.match_flags,
						obj_cam_pose.orient.x, obj_cam_pose.orient.y, obj_cam_pose.orient.z, obj_cam_pose.orient.w,
						obj_cam_pose.pos.x, obj_cam_pose.pos.y, obj_cam_pose.pos.z,
						dev_state->score.matched_blobs, dev_state->score.visible_leds,
						dev_state->score.orient_error.x, dev_state->score.orient_error.y,
						dev_state->score.orient_error.z,
						dev_state->score.pos_error.x, dev_state->score.pos_error.y, dev_state->score.pos_error.z,
						orient_diff.x, orient_diff.y, orient_diff.z, orient_diff.w);
				}
#endif
			}
		}

		/* If we got at least a good match, pass it on for consideration by the fusion */
		if (POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_GOOD)) {
			used_blobs += update_device_and_blobs (pf, frame, dev, dev_state, exp_dev_info, &obj_cam_pose);
		}

		if (frame->quarantined_observation) {
			if (dev_state->found_device_pose) {
				candidate->labels = quarantined;
				memcpy(candidate->tracks, frame->inspection_tracks, sizeof candidate->tracks);
			}
			else candidate->active = false;
			frame->bwobs = bwobs = raw;
			frame->quarantined_observation = false;
		}

		if (!dev_state->found_device_pose &&
		    (!pf->inspection_mask || (pf->inspection_mask & (1u << dev->id))) &&
		    (!(skip_mask & (1u << dev->id)) || (pf->inspection_mask & (1u << dev->id))))
			have_unfound_devices = true;
	}

	if (have_unfound_devices) {
		/* If we have enough unmatched blobs, and unfound online devices,
		 * send the frame for long analysis */
		int unused_blobs = bwobs->num_blobs - used_blobs;

		if (unused_blobs > 4) {
			LOGD("Sensor %d - frame has %d unused blobs and needs long analysis - sending",
				pf->sensor_id, unused_blobs);

			frame->need_long_analysis = true;

#if LOGLEVEL == 0
			/* Log which device(s) we didn't find */
			for (d = 0; d < frame->n_devices; d++) {
				rift_sensor_frame_device_state *dev_state = frame->capture_state + d;

				if (dev_state->found_device_pose)
					continue;

				LOGD("Sensor %d frame %d with %d unmatched blobs needs full search for device %d",
					pf->sensor_id, frame->id, unused_blobs, devs[d]->id);
			}
#endif
		}
		else {
			LOGD("Sensor %d - frame has only %d unused blobs (need at least 5) and needs long analysis - not sending", pf->sensor_id, unused_blobs);
		}
	}
}

void rift_pose_finder_process_blobs_long(rift_pose_finder *pf, rift_sensor_analysis_frame *frame,
        rift_tracked_device **devs, uint32_t skip_mask)
{
	rift_tracker_exposure_info *exposure_info = &frame->exposure_info;
	blobservation* bwobs = frame->bwobs;
	int d, pass;
	bool had_strong_matches = false;

	LOGD("Sensor %d Frame %d - starting long search for devices", pf->sensor_id, frame->id);

	correspondence_search_set_blobs (pf->cs, bwobs->blobs, bwobs->num_blobs);

	for (pass = 0; pass < 2; pass++) {
		/* Only process the devices that were available when this frame was captured */
		for (d = 0; d < frame->n_devices; d++) {
			if (correspondence_search_should_stop(pf->cs))
				return;
			rift_tracked_device *dev = devs[d];
			rift_sensor_frame_device_state *dev_state = frame->capture_state + d;
			rift_tracked_device_exposure_info *exp_dev_info = exposure_info->devices + d;
			bool do_aligned_checks = false;
			float pose_tolerance = 0.0;
			CorrespondenceSearchFlags search_flags = CS_FLAG_STOP_FOR_STRONG_MATCH;

			if (dev_state->found_device_pose)
				continue; /* We already found a pose for this device */
			if (pf->inspection_mask && !(pf->inspection_mask & (1u << dev->id)))
				continue;
			/* held by cv1touch's tracker, or asleep; but a sensor with no pose yet labels blobs only
			 * here (calibrate place), so it searches for every device */
			if (pf->have_camera_pose && (skip_mask & (1u << dev->id)) &&
			    !(pf->inspection_mask & (1u << dev->id)))
				continue;

			LOGV("*** Starting long search for device %d pass %d", dev->id, pass);

			if (pass == 0) {
				if (dev->id == 0)
					search_flags |= CS_FLAG_MATCH_ALL_BLOBS; /* Let the HMD match whatever it can on the first pass */

				search_flags |= CS_FLAG_SHALLOW_SEARCH;
			} else
				search_flags |= CS_FLAG_DEEP_SEARCH;

			if (!rift_tracked_device_get_latest_exposure_info_pose(dev, exp_dev_info)) {
				LOGV ("Skipping long analysis of device %d. No fusion slot assigned", dev->id);
				continue;
			}

			if (exp_dev_info->fusion_slot == -1) {
				LOGV ("Sensor %d Skipping long analysis of device %d. No fusion slot assigned", pf->sensor_id, dev->id);
				continue;
			}

			rift_pose_finder_exp_info_to_dev_state(pf, exp_dev_info, dev_state);

			/* Start from the capture_cam_pose */
			posef obj_cam_pose = dev_state->capture_cam_pose;

			/* If the gravity vector error standard deviation is small enough, try for an
			 * aligned pose from the prior, within 2 standard deviation */
			if (pf->have_camera_pose && !(pf->inspection_mask & (1u << dev->id))) {
				if (dev_state->gravity_error_rad < DEG_TO_RAD(30)) {
					search_flags |= CS_FLAG_MATCH_GRAVITY;
					do_aligned_checks = true;
					pose_tolerance = OHMD_MAX(2 * dev_state->gravity_error_rad, DEG_TO_RAD(30));
				}

				search_flags |= CS_FLAG_HAVE_POSE_PRIOR;
			}

			if (POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_GOOD) && had_strong_matches) {
				/* We have a good pose match for this device, that we found on the first
				 * pass. If any other device found a strong match though, then that may
				 * have claimed blobs we were relying on - so re-check our pose and possibly start again */
				if (search_flags & CS_FLAG_HAVE_POSE_PRIOR) {
					rift_evaluate_pose_with_prior(&dev_state->score, &obj_cam_pose,
						do_aligned_checks, &dev_state->capture_cam_pose,
						&dev_state->obj_cam_pos_error, &dev_state->obj_cam_rot_error,
						bwobs->blobs, bwobs->num_blobs,
						dev->id, dev->leds, pf->calib, NULL);
				}
				else {
					rift_evaluate_pose (&dev_state->score, &obj_cam_pose,
						frame->bwobs->blobs, frame->bwobs->num_blobs,
						dev->id, dev->leds, pf->calib, NULL);
				}

				/* If we don't have a good match any more, do another shallow search */
				if (!POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_GOOD)) {
					LOGD("Sensor %d device %d - lost candidate pose after checking other devices. Searching again", pf->sensor_id, dev->id);
					search_flags |= CS_FLAG_SHALLOW_SEARCH;
				}
			}

			if (search_flags & CS_FLAG_DEEP_SEARCH) {
				LOGD("Sensor %d deep search for device %d", pf->sensor_id, dev->id);
			}

			if (correspondence_search_find_one_pose (pf->cs, dev->id, search_flags, &obj_cam_pose,
					&dev_state->obj_cam_pos_error, &dev_state->obj_cam_rot_error,
					&pf->cam_gravity_vector, pose_tolerance, &dev_state->score)) {
				if (do_aligned_checks) {
					LOGD("Got aligned pose %f, %f, %f, %f for device %d with tolerance %f deg",
						obj_cam_pose.orient.x, obj_cam_pose.orient.y, obj_cam_pose.orient.z, obj_cam_pose.orient.w,
						dev->id, RAD_TO_DEG(pose_tolerance));
				}
			}
			else if (do_aligned_checks) {
				LOGD("No aligned pose for device %d with tolerance %f deg", d, RAD_TO_DEG(pose_tolerance));
			}

			if (correspondence_search_should_stop(pf->cs))
				return;

			LOGV("Sensor %d Frame %d - doing long search with flags 0x%x for device %d match_flags 0x%x matched %u blobs of %u from %d (%s match)",
				pf->sensor_id, frame->id, search_flags, dev->id, dev_state->score.match_flags,
				dev_state->score.matched_blobs, dev_state->score.visible_leds, bwobs->num_blobs,
				POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_GOOD) ? "good" : "bad");

			/* Require a strong pose match on the quick loop */
			if (pass == 0 && !POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_STRONG))
				continue;

			if (POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_GOOD)) {
				had_strong_matches |= POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_STRONG);

				update_device_and_blobs (pf, frame, dev, dev_state, exp_dev_info, &obj_cam_pose);
				frame->long_analysis_found_new_blobs = true;
			}
		}
	}
}

static int
update_device_and_blobs (rift_pose_finder *pf, rift_sensor_analysis_frame *frame,
	rift_tracked_device *dev, rift_sensor_frame_device_state *dev_state,
	rift_tracked_device_exposure_info *exp_dev_info,
	posef *obj_cam_pose)
{
	blobservation* bwobs = frame->bwobs;
	int ret = 0;
	bool preserve_labels = (pf->inspection_mask & (1u << dev->id)) && !frame->detached_observation &&
		!frame->quarantined_observation;
	int previous_labels[MAX_BLOBS_PER_FRAME];
	if (preserve_labels)
		for (int b = 0; b < bwobs->num_blobs; b++) previous_labels[b] = bwobs->blobs[b].led_id;

	/* Clear existing blob IDs for this device, then
	 * back project LED ids into blobs if we find them and the dot product
	 * shows them pointing strongly to the camera */
	rift_clear_blob_labels (bwobs->blobs, bwobs->num_blobs, dev->id);

	rift_mark_matching_blobs (obj_cam_pose, bwobs->blobs, bwobs->num_blobs, dev->id,
			dev->leds, pf->calib);

	/* Refine the pose with PnP now that we've labelled the blobs */
	estimate_initial_pose (bwobs->blobs, bwobs->num_blobs,
		dev->id, dev->leds->points, dev->leds->num_points, pf->calib,
		obj_cam_pose, NULL, NULL, true);

	/* And label the blobs again in case we collected any more */
	rift_mark_matching_blobs (obj_cam_pose, bwobs->blobs, bwobs->num_blobs, dev->id,
			dev->leds, pf->calib);

	dev_state->final_cam_pose = *obj_cam_pose;

	LOGD ("sensor %d PnP for device %d yielded quat %f %f %f %f pos %f %f %f",
		pf->sensor_id, dev->id,
		dev_state->final_cam_pose.orient.x, dev_state->final_cam_pose.orient.y, dev_state->final_cam_pose.orient.z, dev_state->final_cam_pose.orient.w,
		dev_state->final_cam_pose.pos.x, dev_state->final_cam_pose.pos.y, dev_state->final_cam_pose.pos.z);

	posef pose = dev_state->final_cam_pose;
	rift_pose_metrics *score = &dev_state->score;

	if (pf->have_camera_pose && !(pf->inspection_mask & (1u << dev->id))) {
		rift_evaluate_pose_with_prior(&dev_state->score, &pose,
			false, &dev_state->capture_cam_pose, &dev_state->obj_cam_pos_error, &dev_state->obj_cam_rot_error,
			frame->bwobs->blobs, frame->bwobs->num_blobs,
			dev->id, dev->leds, pf->calib, NULL);
	} else {
		rift_evaluate_pose (score, &pose,
			frame->bwobs->blobs, frame->bwobs->num_blobs,
			dev->id, dev->leds, pf->calib, NULL);
	}

	if (!POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_GOOD)) {
		LOGV("Sensor %d Failed pose match for dev %d with match_flags 0x%x - only %u LEDs matched %u visible ones",
			pf->sensor_id, dev->id, score->match_flags, score->matched_blobs, score->visible_leds);
		if (preserve_labels)
			for (int b = 0; b < bwobs->num_blobs; b++) bwobs->blobs[b].led_id = previous_labels[b];
		else
			rift_clear_blob_labels (bwobs->blobs, bwobs->num_blobs, dev->id);
		return 0;
	}

	LOGV("Sensor %d Found %s pose match - %u LEDs matched %u visible ones flags 0x%x",
		pf->sensor_id, POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_STRONG) ? "strong" : "good",
		score->matched_blobs, score->visible_leds, score->match_flags);
	dev_state->cv1touch_pose_valid = true;

	/* cv1touch: see cv1touch_may_place(). Upstream required the headset (dev->id == 0); any device
	 * may place a sensor here. The gravity gate stays: the reported error is floored at 25 degrees,
	 * so passing it means the device's fusion has converged its tilt (a sensor placed from an
	 * unconverged device lands in a world tilted against gravity). */
	if (!pf->have_camera_pose && pf->cv1touch_allow_placement && POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_STRONG) &&
	    dev_state->gravity_error_rad <= DEG_TO_RAD(25.0) && cv1touch_may_place(exp_dev_info)) {
		/* No camera pose yet. If this is the HMD, we had an IMU pose at capture time,
		 * and the fusion has a good gravity vector from the IMU, use it to
		 * initialise the camera (world->camera) pose using the current headset pose.
		 * Calculate the xform from camera->world by applying
		 * the observed pose (object->camera), inverted (so camera->object) to our found
		 * fusion pose (object->world) to yield camera->world xform
		 */
		posef camera_object_pose = pose;
		oposef_inverse(&camera_object_pose);

		posef *capture_pose = &exp_dev_info->capture_pose;
		oposef_apply(&camera_object_pose, capture_pose, &pf->camera_pose);

		LOGI("Set sensor %d pose from device %d - tracker pose quat %f %f %f %f  pos %f %f %f"
				" fusion pose quat %f %f %f %f  pos %f %f %f gravity error %f degrees yielded"
				" world->camera pose quat %f %f %f %f  pos %f %f %f",
			pf->sensor_id, dev->id,
			pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w, pose.pos.x, pose.pos.y, pose.pos.z,
			capture_pose->orient.x, capture_pose->orient.y, capture_pose->orient.z, capture_pose->orient.w,
			capture_pose->pos.x, capture_pose->pos.y, capture_pose->pos.z, RAD_TO_DEG(dev_state->gravity_error_rad),
			pf->camera_pose.orient.x, pf->camera_pose.orient.y, pf->camera_pose.orient.z, pf->camera_pose.orient.w,
			pf->camera_pose.pos.x, pf->camera_pose.pos.y, pf->camera_pose.pos.z);

		rift_pose_finder_set_camera_pose(pf, &pf->camera_pose);
	}

	if (!pf->have_camera_pose) {
		LOGD("Sensor %d No camera pose yet - gravity error is %f degrees rot_error (%f, %f, %f). Not fusing pose for device %d",
		    pf->sensor_id, RAD_TO_DEG(dev_state->gravity_error_rad),
		    exp_dev_info->rot_error.x, exp_dev_info->rot_error.y, exp_dev_info->rot_error.z,
				dev->id);
		/* cv1touch: the camera-relative pose is found (and exported); without this the long search
		 * re-ran a deep search for the device on pass 1, which cost frame rate in capture mode and
		 * could overwrite the exported match score. */
		dev_state->found_device_pose = true;
		return score->matched_blobs;
	}

	// rift_dump_pose_leds (&pose, dev->id, dev->leds, pf->calib);

	/* The pose we found is the transform from object coords to camera-relative coords.
	 * Our camera pose stores the transform from camera to world, and what we
	 * need to give the fusion is transform from object->world.
	 *
	 * To get the transform from object->world, take object->camera pose, and apply
	 * the camera->world pose. */
	oposef_apply(&pose, &pf->camera_pose, &pose);

	OHMD_ASSERT(pf->pose_cb != NULL);

	if (pf->pose_cb (pf->pose_cb_data, dev, frame, &pose, score)) {
		dev_state->found_device_pose = true;
		ret = score->matched_blobs;
	} else if (pf->inspection_mask & (1u << dev->id)) {
		/* The identity check consumed this exposure: a cold search of it, on the long path
		 * or after this fast match, could only be discarded as stale. */
		dev_state->found_device_pose = true;
		if (preserve_labels)
			for (int b = 0; b < bwobs->num_blobs; b++) bwobs->blobs[b].led_id = previous_labels[b];
	}

	return ret;
}

