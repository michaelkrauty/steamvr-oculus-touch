// Copyright 2020 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#ifndef __POSE_SEARCH_H__
#define __POSE_SEARCH_H__

#include "rift.h"
#include "rift-tracker-common.h"
#include "rift-sensor.h"
#include "rift-sensor-maths.h"
#include "rift-sensor-blobwatch.h"

#include "correspondence_search.h"

typedef struct rift_pose_finder rift_pose_finder;

typedef bool (*rift_pose_finder_cb) (void *cb_data,
	rift_tracked_device *dev, rift_sensor_analysis_frame *frame,
	posef *obj_world_pose, rift_pose_metrics *score);

/* Callback-serialized identity evidence. Relative camera/IMU rotations do not depend on
 * the sensor's disputed absolute pose. Indistinguishable two-hand motion proves nothing.
 * Evidence belongs to a continuous correspondence, not merely a device ID. */
typedef struct {
	uint64_t first_ns, last_ns;
	uint64_t evidence_epoch; /* lifetime of an exported identity, distinct from raw-frame continuity */
	quatf own, other, camera;
	bool have_other;
	bool validated;
	unsigned agreeing;
	unsigned contradicting;
	uint64_t epoch;
	unsigned n_blobs;
	uint32_t blob_ids[MAX_BLOBS_PER_FRAME];
	int led_ids[MAX_BLOBS_PER_FRAME];
	uint64_t tracks[MAX_BLOBS_PER_FRAME];
} rift_inspection_identity;

typedef enum {
	RIFT_IDENTITY_UNKNOWN,     /* no new evidence: keep whatever is established */
	RIFT_IDENTITY_AGREES,      /* this hand, validated and consistent with that identity */
	RIFT_IDENTITY_CONTRADICTS, /* a validated identity was contradicted: revoke it */
} rift_identity_verdict;

/* Fast-worker-only quarantine. These labels and its camera transform never enter blobwatch
 * or the trusted fast priors before independent motion establishes the candidate's identity. */
typedef struct {
	bool active;
	uint64_t epoch;
	blobservation labels;
	uint64_t tracks[MAX_BLOBS_PER_FRAME];
	posef world_from_cam;
} rift_inspection_candidate;

struct rift_pose_finder {
	int sensor_id;
	rift_sensor_camera_params *calib;

	/* Updated from fast_analysis_thread */
	bool have_camera_pose;
	posef camera_pose;
	vec3f cam_gravity_vector;
	uint32_t inspection_mask; /* label-only checks, by device id; keep the calibrated pose */
	uint32_t inspection_pose_mask;
	posef inspection_poses[RIFT_MAX_TRACKED_DEVICES];
	rift_inspection_identity inspection_identity[RIFT_MAX_TRACKED_DEVICES];
	uint64_t next_identity_epoch; /* sensor-lock serialized, never reused during this engine lifetime */
	rift_inspection_candidate inspection_candidate[RIFT_MAX_TRACKED_DEVICES];
	uint64_t inspection_frame_ns, inspection_epoch; /* fast worker only */
	blobservation inspection_previous_raw;
	uint64_t inspection_tracks[MAX_BLOBS_PER_FRAME], inspection_next_track;
	bool cv1touch_allow_placement; /* provisional runtime placement allowed (cv1touch) */

	/* Brute force search */
	correspondence_search_t *cs;

	rift_pose_finder_cb pose_cb;
	void *pose_cb_data;
};

void rift_pose_finder_init(rift_pose_finder *pf, rift_sensor_camera_params *calib,
		rift_pose_finder_cb pose_cb, void *pose_cb_data);
void rift_pose_finder_clear(rift_pose_finder *pf);
void rift_pose_finder_set_camera_pose(rift_pose_finder *pf, const posef *camera_pose);

/* Called under the callback lock before judging a new exposure. At least five identical
 * blob/LED pairs must persist through raw-frame steps within 32 px, with no gap over 60 ms.
 * A broken correspondence revokes shared labels. An unrelated fit cannot replace a still
 * continuous trusted stream. */
typedef enum {
	RIFT_CORRESPONDENCE_CONTINUOUS,
	RIFT_CORRESPONDENCE_BROKEN,
	RIFT_CORRESPONDENCE_UNRELATED, /* a cold alternative while the trusted tracks still exist */
} rift_correspondence_verdict;
rift_correspondence_verdict rift_pose_finder_check_inspection_blobs(rift_pose_finder *pf, int device_id,
    const rift_sensor_analysis_frame *frame);

/* `tracked`: the correspondence came from the fast path's prior and labels, which follow the
 * identified hand; a cold search's candidate that disagrees is only rejected. `judgeable` is
 * set when this exposure's IMU motion could establish identity (or the identity is already
 * validated), whatever the correspondence's own verdict. */
rift_identity_verdict rift_pose_finder_identify_inspection(rift_pose_finder *pf, int device_id,
    uint64_t exposure_ns, const quatf *own, const quatf *other, const quatf *camera, bool tracked,
    bool *judgeable);

/* Fast worker only: seed an untrusted candidate from a completed cold search. */
void rift_pose_finder_set_inspection_candidate(rift_pose_finder *pf, int device_id,
    const blobservation *labels, uint64_t epoch, const uint64_t *tracks,
    const posef *capture_world_pose, const posef *observed_cam_pose);

/* Fast worker: a delayed label update must still belong to the live image-motion lineage. */
bool rift_pose_finder_filter_inspection_labels(rift_pose_finder *pf, int device_id,
    blobservation *labels, const uint64_t *tracks);

/* A validated cold correspondence supplies a private camera prior for this hand's labels.
 * This never changes the calibrated camera pose or the fusion's world frame. Fast worker only. */
void rift_pose_finder_set_inspection_pose(rift_pose_finder *pf, int device_id,
    const posef *capture_world_pose, const posef *observed_cam_pose);

/* `skip_mask` (bit 1 << device id): devices another tracker holds or that are asleep. The fast
 * path still matches them against their priors, but a frame goes to long analysis only for the
 * others, and long analysis searches only for the others, except on a sensor with no pose yet. */
void rift_pose_finder_process_blobs_fast(rift_pose_finder *pf, rift_sensor_analysis_frame *frame,
	rift_tracked_device **devs, uint32_t skip_mask);
void rift_pose_finder_process_blobs_long(rift_pose_finder *pf, rift_sensor_analysis_frame *frame,
	rift_tracked_device **devs, uint32_t skip_mask);
void rift_pose_finder_exp_info_to_dev_state (rift_pose_finder *pf,
  const rift_tracked_device_exposure_info *exp_dev_info, rift_sensor_frame_device_state *dev_state);

void rift_pose_finder_cv1touch_reset(void);

#endif
