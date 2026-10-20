#define _GNU_SOURCE
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* Identity quarantine and the fast/long observation handoff use the actual sensor runtime
 * and blobwatch. Controlled geometry isolates label ownership from PnP accuracy; the separate
 * inspection-tracking test drives real correspondence search and PnP. No devices are opened. */
#define POP_QUEUE bw_pop_queue
#define blobwatch_update_labels native_blobwatch_update_labels
#include "rift-sensor-blobwatch.c"
#undef blobwatch_update_labels
void blobwatch_update_labels(blobwatch *bw, blobservation *ob, int device_id);
#undef POP_QUEUE
#undef PUSH_QUEUE
#undef INIT_QUEUE
#include "correspondence_search.c"
#include "rift-sensor-flicker.c"
#include "omath.c"
#include "rift-sensor-pose-search.c"
#include "rift-sensor.c"
#include "test_util.h"
#include <pthread.h>
#include <time.h>
#include <sched.h>


static pthread_t fast_worker;
void blobwatch_update_labels(blobwatch *bw, blobservation *ob, int device_id)
{
    CHECK(pthread_equal(pthread_self(), fast_worker));
    native_blobwatch_update_labels(bw, ob, device_id);
}
void ohmd_cond_signal(ohmd_cond *cond) { (void)cond; }

static int awake_device;
static int matching_first_blob = 1;
static bool retain_exposure;
static bool crossing_trace;

/* Six known blob IDs stand in for this scheduling fixture's projected other-hand LEDs. */
void rift_evaluate_pose_with_prior(rift_pose_metrics *score, posef *pose, bool prior_must_match, posef *prior,
    const vec3f *pos_error, const vec3f *rot_error, struct blob *blobs, int n, int device,
    rift_leds *leds, rift_sensor_camera_params *calib, rift_rect_t *bounds)
{
    (void)score; (void)pose; (void)prior_must_match; (void)prior; (void)pos_error; (void)rot_error;
    (void)blobs; (void)n; (void)device; (void)leds; (void)calib; (void)bounds;
    CHECK(device == awake_device);
    int matched = 0;
    for (int i = 0; i < n; i++) matched += LED_OBJECT_ID(blobs[i].led_id) == device;
    score->match_flags = matched >= 5 ? RIFT_POSE_MATCH_GOOD | RIFT_POSE_MATCH_STRONG | RIFT_POSE_MATCH_LED_IDS : 0;
    score->matched_blobs = matched;
    score->visible_leds = 6;
}
/* An inspected hand is evaluated without the disputed calibration's prior, by the same rule. */
void rift_evaluate_pose(rift_pose_metrics *score, posef *pose, struct blob *blobs, int n, int device,
    rift_leds *leds, rift_sensor_camera_params *calib, rift_rect_t *bounds)
{
    rift_evaluate_pose_with_prior(score, pose, false, NULL, NULL, NULL, blobs, n, device, leds, calib, bounds);
}
bool rift_score_is_better_pose(rift_pose_metrics *old, rift_pose_metrics *new_score)
{
    (void)old; (void)new_score;
    abort();
}

/* The engine's mutex API is hidden in its shared library. Use real pthread mutexes here. */
ohmd_mutex *ohmd_create_mutex(ohmd_context *ctx)
{
    (void)ctx;
    pthread_mutex_t *mutex = malloc(sizeof *mutex);
    CHECK(mutex && pthread_mutex_init(mutex, NULL) == 0);
    return (ohmd_mutex *)mutex;
}
void ohmd_destroy_mutex(ohmd_mutex *mutex)
{
    CHECK(pthread_mutex_destroy((pthread_mutex_t *)mutex) == 0);
    free(mutex);
}
void ohmd_lock_mutex(ohmd_mutex *mutex) { CHECK(pthread_mutex_lock((pthread_mutex_t *)mutex) == 0); }
void ohmd_unlock_mutex(ohmd_mutex *mutex) { CHECK(pthread_mutex_unlock((pthread_mutex_t *)mutex) == 0); }
uint64_t ohmd_monotonic_per_sec(ohmd_context *ctx) { return ctx->monotonic_ticks_per_sec; }
uint64_t ohmd_monotonic_conv(uint64_t ticks, uint64_t src, uint64_t dst)
{
    return ticks / src * dst + ticks % src * dst / src;
}

static uint64_t now_ticks = 1000000000;
uint64_t ohmd_monotonic_get(ohmd_context *ctx)
{
	(void)ctx;
	return __atomic_load_n(&now_ticks, __ATOMIC_RELAXED);
}

static int fusion_updates;
bool rift_tracked_device_model_pose_update(rift_tracked_device *dev, uint64_t now, uint64_t captured,
    rift_tracker_exposure_info *info, rift_pose_metrics *score, posef *pose, const char *source)
{
    (void)dev; (void)now; (void)captured; (void)info; (void)score; (void)pose; (void)source;
    fusion_updates++;
    return dev->id == awake_device;
}
void rift_tracked_device_frame_release(rift_tracked_device *dev, rift_tracker_exposure_info *info)
{
    (void)dev; (void)info;
}
bool rift_tracked_device_get_latest_exposure_info_pose(rift_tracked_device *dev,
    rift_tracked_device_exposure_info *info)
{
    CHECK(dev->id == awake_device);
    info->fusion_slot = 0;
    if (!retain_exposure) info->capture_pose.orient.w = 1;
    return true;
}
void rift_clear_blob_labels(struct blob *blobs, int n, int dev)
{
    for (int i = 0; i < n; i++)
        if (LED_OBJECT_ID(blobs[i].led_id) == dev) blobs[i].led_id = -1;
}
void rift_mark_matching_blobs(posef *pose, struct blob *blobs, int n, int dev,
    rift_leds *leds, rift_sensor_camera_params *calib)
{
    (void)pose; (void)leds; (void)calib;
    CHECK(dev == awake_device && n >= 6);
    /* These six blob IDs are this fixture's projected LEDs. Respect another device's labels,
     * as the real matcher does; the throughput test supplies actual CS/PnP geometry. */
    for (int i = 0; i < n; i++)
        if (blobs[i].blob_id >= matching_first_blob && blobs[i].blob_id < matching_first_blob + 6 &&
            (blobs[i].led_id < 0 || LED_OBJECT_ID(blobs[i].led_id) == dev))
            blobs[i].led_id = LED_MAKE_ID(dev, blobs[i].blob_id - matching_first_blob);
}

static int released;
void rift_tracker_frame_release(rift_tracker_ctx *ctx, uint64_t now, uint64_t exposure,
    rift_tracker_exposure_info *info, const char *source)
{
    (void)ctx; (void)now; (void)exposure; (void)info; (void)source;
}
void ohmd_video_frame_release(ohmd_video_frame *frame)
{
    CHECK(frame != NULL);
    released++;
}


static quatf turn(float degrees)
{
    quatf q;
    vec3f axis = {{0, 1, 0}};
    oquatf_init_axis(&q, &axis, DEG_TO_RAD(degrees));
    return q;
}

static rift_identity_verdict identify(rift_pose_finder *pf, int dev, uint64_t at, quatf own,
    const quatf *other, quatf seen, bool *judgeable)
{
    bool unused;
    return rift_pose_finder_identify_inspection(pf, dev, at, &own, other, &seen, true,
        judgeable ? judgeable : &unused);
}

/* Anchor at zero, then three exposures turned 40 degrees: validated on the third. */
static void validate(rift_pose_finder *pf, int dev, uint64_t at, const quatf *other)
{
    CHECK(identify(pf, dev, at, turn(0), other, turn(45), NULL) == RIFT_IDENTITY_UNKNOWN);
    for (int k = 1; k <= 3; k++)
        CHECK(identify(pf, dev, at + k * 20000000ULL, turn(40), other, turn(85), NULL) ==
            (k == 3 ? RIFT_IDENTITY_AGREES : RIFT_IDENTITY_UNKNOWN));
}

static void identity_cases(void)
{
    for (int dev = 1; dev <= 2; dev++) {
        for (int camera_degrees = 5; camera_degrees <= 45; camera_degrees += 5) {
            for (int together = 0; together <= 1; together++) {
                rift_pose_finder pf = {0};
                quatf camera = turn(camera_degrees), offset;
                vec3f x = {{1, 0, 0}};
                oquatf_init_axis(&offset, &x, DEG_TO_RAD(60));
                bool accepted = false;
                for (int k = 0; k < 8; k++) {
                    quatf motion = turn(k * 10), own, other = offset, own_offset, seen;
                    oquatf_init_axis(&own_offset, &x, DEG_TO_RAD(-40));
                    oquatf_mult(&motion, &own_offset, &own);
                    if (together) oquatf_mult(&motion, &offset, &other);
                    oquatf_mult(&camera, &own, &seen);
                    rift_identity_verdict id = identify(&pf, dev,
                        1000000000ULL + k * 20000000ULL, own, &other, seen, NULL);
                    CHECK(id != RIFT_IDENTITY_CONTRADICTS);
                    if (together) CHECK(id == RIFT_IDENTITY_UNKNOWN); /* A rigid grip is ambiguous. */
                    accepted |= id == RIFT_IDENTITY_AGREES;
                }
                CHECK(accepted == !together);
            }
        }
    }
    /* Measured wrong-hand case: about 120 degrees of absolute alias, while the requested
     * hand turns less than 0.08 degrees in three seconds. Repeated labels add no identity. */
    rift_pose_finder pf = {0};
    for (int k = 0; k < 150; k++) {
        quatf other = turn(0);
        bool judgeable = true;
        CHECK(identify(&pf, 1, 1000000000ULL + k * 20000000ULL, turn(k * 0.0005f), &other,
            turn(120 + k * 0.0005f), &judgeable) == RIFT_IDENTITY_UNKNOWN);
        CHECK(!judgeable); /* Too still: an inspection seeing only this would be starved. */
    }
    /* Replay pairs had p95 own-hand error below four percent of their turn. The nearest
     * alias-like pair that also passed the other-hand test was 16.8 percent. Keep headroom
     * on both sides of the ten-percent bound; noisy or ambiguous evidence stays unknown. */
    for (int alias = 0; alias <= 1; alias++) {
        memset(&pf, 0, sizeof pf);
        quatf zero = turn(0);
        CHECK(identify(&pf, 1, 1000000000, zero, &zero, zero, NULL) == RIFT_IDENTITY_UNKNOWN);
        for (int k = 0; k < 3; k++) {
            float span = (alias ? 90 : 40) + k * 10;
            bool judgeable = false;
            CHECK(identify(&pf, 1, 1020000000 + k * 20000000, turn(span), &zero,
                turn(span * (alias ? 1.168f : 1.08f)), &judgeable) ==
                (!alias && k == 2 ? RIFT_IDENTITY_AGREES : RIFT_IDENTITY_UNKNOWN));
            /* Enough motion either way: a wrong candidate is a verdict. It also re-anchors, so
             * the next turn counts from it. */
            CHECK(judgeable == (!alias || k == 0));
        }
    }
    /* While validating, a large earlier rotation cannot relax the bound on the current turn:
     * seven degrees is too much for a forty-degree turn. */
    memset(&pf, 0, sizeof pf);
    quatf zero = turn(0);
    CHECK(identify(&pf, 1, 1000000000, zero, &zero, zero, NULL) == RIFT_IDENTITY_UNKNOWN);
    for (int k = 0; k < 2; k++)
        CHECK(identify(&pf, 1, 1020000000 + k * 20000000, turn(100), &zero, turn(100), NULL) ==
            RIFT_IDENTITY_UNKNOWN);
    CHECK(identify(&pf, 1, 1100000000, turn(40), &zero, turn(47), NULL) == RIFT_IDENTITY_UNKNOWN);
    CHECK(!pf.inspection_identity[1].validated && pf.inspection_identity[1].first_ns == 1100000000);
    /* An exposure is consumed once: a later search of the same or an older exposure is stale. */
    memset(&pf, 0, sizeof pf);
    validate(&pf, 1, 1000000000, &zero);
    CHECK(identify(&pf, 1, 1060000000, turn(40), &zero, turn(205), NULL) == RIFT_IDENTITY_UNKNOWN);
    CHECK(identify(&pf, 1, 1040000000, turn(40), &zero, turn(205), NULL) == RIFT_IDENTITY_UNKNOWN);
    CHECK(pf.inspection_identity[1].contradicting == 0);
}

/* A fixed alias of the other hand turns by exactly the other hand's angle, whatever its
 * offset. One clause tells the hands apart: their turns differ by at least half of the
 * requested hand's. The former second clause (the other hand's error beyond the own error
 * by a quarter of the turn) was implied by the first once the own error is within a tenth. */
static void discrimination_cases(void)
{
    vec3f z = {{0, 0, 1}};
    for (int side = 0; side <= 1; side++) {
        for (int beyond = 0; beyond <= 1; beyond++) {
            /* The requested hand turns 40 degrees about y; the other hand about z, by just
             * under or over 20 (or 60) degrees: 0.5 of 40 is the bound. */
            float other_degrees = side ? (beyond ? 60.5f : 59.5f) : (beyond ? 19.5f : 20.5f);
            rift_pose_finder pf = {0};
            quatf other0, other1;
            oquatf_init_axis(&other0, &z, 0);
            CHECK(identify(&pf, 2, 1000000000, turn(0), &other0, turn(45), NULL) == RIFT_IDENTITY_UNKNOWN);
            oquatf_init_axis(&other1, &z, DEG_TO_RAD(other_degrees));
            bool accepted = false, judgeable = false;
            for (int k = 1; k <= 3; k++) {
                rift_identity_verdict id = identify(&pf, 2, 1000000000 + k * 20000000ULL,
                    turn(40), &other1, turn(85), &judgeable);
                CHECK(judgeable == beyond);
                accepted |= id == RIFT_IDENTITY_AGREES;
            }
            CHECK(accepted == beyond);
        }
    }
    /* The other hand disconnected or asleep (no IMU exposure): own motion alone identifies. */
    rift_pose_finder pf = {0};
    validate(&pf, 1, 1000000000, NULL);
    /* Before identity, the other hand appearing or vanishing starts over. */
    memset(&pf, 0, sizeof pf);
    quatf still = turn(0);
    CHECK(identify(&pf, 1, 1000000000, turn(0), NULL, turn(45), NULL) == RIFT_IDENTITY_UNKNOWN);
    CHECK(identify(&pf, 1, 1020000000, turn(40), NULL, turn(85), NULL) == RIFT_IDENTITY_UNKNOWN);
    CHECK(identify(&pf, 1, 1040000000, turn(40), &still, turn(85), NULL) == RIFT_IDENTITY_UNKNOWN);
    CHECK(pf.inspection_identity[1].first_ns == 1040000000 && pf.inspection_identity[1].have_other);
    /* Once identified, it does not matter. */
    memset(&pf, 0, sizeof pf);
    validate(&pf, 1, 1000000000, NULL);
    CHECK(identify(&pf, 1, 1100000000, turn(40), &still, turn(85), NULL) == RIFT_IDENTITY_AGREES);
    CHECK(identify(&pf, 1, 1120000000, turn(40), NULL, turn(85), NULL) == RIFT_IDENTITY_AGREES);
}

/* A requested hand's stream through the runtime callback, which exports or revokes labels. */
struct stream {
    rift_sensor_ctx *sensor;
    rift_tracked_device *dev;
    rift_sensor_analysis_frame frame;
    blobservation labels;
    ohmd_video_frame video;
    bool reverse_labels;
};

static bool expose(struct stream *s, uint64_t at, float own_degrees, float seen_degrees, bool detached)
{
    int requested = s->dev->id;
    for (int i = 6; i < 12; i++) s->labels.blobs[i].led_id = LED_MAKE_ID(requested, s->reverse_labels ? 11 - i : i - 6);
    s->frame.vframe = &s->video;
    s->frame.bwobs = &s->labels;
    s->frame.detached_observation = detached;
    s->frame.exposure_info.devices[0].device_time_ns = at;
    s->frame.exposure_info.devices[0].capture_pose.orient = turn(own_degrees);
    s->frame.exposure_info.devices[1].device_time_ns = at;
    s->frame.exposure_info.devices[1].capture_pose.orient = turn(0);
    s->frame.capture_state[0].final_cam_pose.orient = turn(45 + seen_degrees);
    s->frame.capture_state[0].cv1touch_pose_valid = true;
    rift_pose_metrics score = {.match_flags = RIFT_POSE_MATCH_GOOD | RIFT_POSE_MATCH_STRONG};
    posef pose = s->frame.capture_state[0].final_cam_pose;
    return handle_found_pose(s->sensor, s->dev, &s->frame, &pose, &score);
}

static bool revoking(struct stream *s)
{
    return s->sensor->pending_label_revoke & (1u << s->dev->id);
}

static void apply(rift_sensor_ctx *sensor)
{
    ohmd_lock_mutex(sensor->sensor_lock);
    apply_pending_pose(sensor);
    ohmd_unlock_mutex(sensor->sensor_lock);
}

/* Validates with a forty-degree turn at `at`, through the callback, then applies its labels. */
static uint64_t validate_stream(struct stream *s, uint64_t at)
{
    memset(s->sensor->pf.inspection_identity, 0, sizeof s->sensor->pf.inspection_identity);
    /* A fresh validation's detached results refer to the raw tracks currently live in
     * blobwatch, including after another case changed their IDs or image coordinates. */
    *s->sensor->bw->last_observation = s->labels;
    s->video.start_ts = at;
    rift_sensor_analysis_frame raw = {.bwobs = &s->labels, .vframe = &s->video};
    rift_pose_finder_process_blobs_fast(&s->sensor->pf, &raw, NULL, 0);
    s->frame.inspection_epoch = raw.inspection_epoch;
    memcpy(s->frame.inspection_tracks, raw.inspection_tracks, sizeof raw.inspection_tracks);
    s->frame.inspection_raw_valid = false;
    CHECK(!expose(s, at, 0, 0, true));
    for (int k = 1; k <= 3; k++) {
        s->video.start_ts = at + k * 20000000ULL;
        rift_pose_finder_process_blobs_fast(&s->sensor->pf, &raw, NULL, 0);
        s->frame.inspection_epoch = raw.inspection_epoch;
        memcpy(s->frame.inspection_tracks, raw.inspection_tracks, sizeof raw.inspection_tracks);
        CHECK(expose(s, s->video.start_ts, 40, 40, true) == (k == 3));
    }
    apply(s->sensor);
    CHECK(s->sensor->pf.inspection_pose_mask & (1u << s->dev->id));
    CHECK(!revoking(s));
    return at + 60000000ULL;
}

#define TICK 20000000ULL

static struct stream new_stream(rift_sensor_ctx *sensor, rift_tracked_device *dev, blobservation *latest)
{
    struct stream s = {.sensor = sensor, .dev = dev};
    s.labels = *latest;
    sensor->pf.inspection_previous_raw = *latest;
    memset(sensor->pf.inspection_tracks, 0, sizeof sensor->pf.inspection_tracks);
    s.frame = (rift_sensor_analysis_frame){ .n_devices = 2, .long_analysis_start_ts = 1,
        .inspection_epoch = sensor->pf.inspection_epoch};
    return s;
}

/* One cold candidate supplies a private, untrusted track, not shared labels or a fast
 * prior. Consecutive raw frames validate motion at camera rate instead of serial searches. */
static void quarantined_candidate(struct stream *s)
{
    uint64_t at = 400000000000ULL;
    memset(s->sensor->pf.inspection_identity, 0, sizeof s->sensor->pf.inspection_identity);
    s->video.start_ts = at;
    rift_sensor_analysis_frame raw = {.bwobs = &s->labels, .vframe = &s->video};
    rift_pose_finder_process_blobs_fast(&s->sensor->pf, &raw, NULL, 0);
    s->frame.inspection_epoch = raw.inspection_epoch;
    memcpy(s->frame.inspection_tracks, raw.inspection_tracks, sizeof raw.inspection_tracks);
    CHECK(!expose(s, at, 0, 0, true));
    apply(s->sensor);
    CHECK(!(s->sensor->pf.inspection_pose_mask & (1u << s->dev->id)));
    for (int i = 6; i < 12; i++) CHECK(s->sensor->bw->last_observation->blobs[i].led_id == -1);
    awake_device = s->dev->id;
    matching_first_blob = s->labels.blobs[6].blob_id;
    retain_exposure = true;
    rift_led points[6] = {0};
    rift_leds model = {.points = points, .num_points = 6};
    s->dev->leds = &model;
    for (int k = 1; k <= 6; k++) {
        s->video.start_ts = at + k * TICK;
        rift_sensor_analysis_frame frame = {.bwobs = s->sensor->bw->last_observation,
            .vframe = &s->video, .n_devices = 1};
        frame.exposure_info.devices[0].device_time_ns = s->video.start_ts;
        frame.exposure_info.devices[0].capture_pose.orient = turn(k * 12);
        rift_pose_finder_process_blobs_fast(&s->sensor->pf, &frame, s->sensor->devices, 0x6);
        CHECK(frame.capture_state[0].found_device_pose && !frame.need_long_analysis);
        if (k < 6) {
            CHECK(!frame.capture_state[0].cv1touch_pose_valid);
            CHECK(!(s->sensor->pf.inspection_pose_mask & (1u << s->dev->id)));
            for (int i = 6; i < 12; i++) CHECK(frame.bwobs->blobs[i].led_id == -1);
        } else {
            CHECK(frame.capture_state[0].cv1touch_pose_valid);
            CHECK(s->sensor->pf.inspection_pose_mask & (1u << s->dev->id));
        }
    }
    matching_first_blob = 1;
    retain_exposure = false;
    s->dev->leds = NULL;
}

/* An untrusted candidate that follows the wrong motion must stop monopolising fast
 * matching. It stays unknown, and a later frame may search for another correspondence. */
static void candidate_wrong_motion(struct stream *s)
{
    memset(s->sensor->pf.inspection_identity, 0, sizeof s->sensor->pf.inspection_identity);
    s->sensor->pf.inspection_pose_mask = 0;
    blobwatch_clear_labels(s->sensor->bw, s->dev->id);
    CHECK(!expose(s, 500000000000ULL, 0, 0, true));
    apply(s->sensor);
    CHECK(s->sensor->pf.inspection_candidate[s->dev->id].active);
    s->frame.quarantined_observation = true;
    CHECK(!expose(s, 500020000000ULL, 12, 0, false));
    CHECK(!s->sensor->pf.inspection_candidate[s->dev->id].active);
    CHECK(!s->sensor->pf.inspection_pose_mask);
    for (int i = 6; i < 12; i++) CHECK(s->sensor->bw->last_observation->blobs[i].led_id == -1);
    awake_device = s->dev->id;
    rift_led points[6] = {0};
    rift_leds model = {.points = points, .num_points = 6};
    s->dev->leds = &model;
    rift_sensor_analysis_frame next = {.bwobs = s->sensor->bw->last_observation,
        .vframe = &s->video, .n_devices = 1};
    next.exposure_info.devices[0].device_time_ns = 500040000000ULL;
    rift_pose_finder_process_blobs_fast(&s->sensor->pf, &next, s->sensor->devices, 6);
    CHECK(next.need_long_analysis);
    s->dev->leds = NULL;
}

static void candidate_image_jump(struct stream *s)
{
    memset(s->sensor->pf.inspection_identity, 0, sizeof s->sensor->pf.inspection_identity);
    s->sensor->pf.inspection_pose_mask = 0;
    blobwatch_clear_labels(s->sensor->bw, s->dev->id);
    s->labels = *s->sensor->bw->last_observation;
    s->video.start_ts = 620000000000ULL;
    rift_sensor_analysis_frame raw = {.bwobs = &s->labels, .vframe = &s->video};
    rift_pose_finder_process_blobs_fast(&s->sensor->pf, &raw, NULL, 0);
    s->frame.inspection_epoch = raw.inspection_epoch;
    memcpy(s->frame.inspection_tracks, raw.inspection_tracks, sizeof raw.inspection_tracks);
    CHECK(!expose(s, s->video.start_ts, 0, 0, true));
    apply(s->sensor);
    CHECK(s->sensor->pf.inspection_candidate[s->dev->id].active);
    awake_device = s->dev->id;
    matching_first_blob = s->labels.blobs[6].blob_id;
    retain_exposure = true;
    rift_led points[6] = {0};
    rift_leds model = {.points = points, .num_points = 6};
    s->dev->leds = &model;
    for (int b = 6; b < 12; b++) s->sensor->bw->last_observation->blobs[b].x += 200;
    s->video.start_ts += TICK;
    rift_sensor_analysis_frame next = {.bwobs = s->sensor->bw->last_observation,
        .vframe = &s->video, .n_devices = 1};
    next.exposure_info.devices[0].device_time_ns = s->video.start_ts;
    next.exposure_info.devices[0].capture_pose.orient = turn(12);
    rift_pose_finder_process_blobs_fast(&s->sensor->pf, &next, s->sensor->devices, 6);
    CHECK(!s->sensor->pf.inspection_candidate[s->dev->id].active && next.need_long_analysis);
    CHECK(!next.capture_state[0].cv1touch_pose_valid && !s->sensor->pf.inspection_pose_mask);
    for (int b = 6; b < 12; b++) CHECK(next.bwobs->blobs[b].led_id == -1);
    matching_first_blob = 1;
    retain_exposure = false;
    s->dev->leds = NULL;
}

/* Aiming steadily past the anchor's three seconds, then a second anchor: no new evidence,
 * never a revocation. Labels keep flowing while nothing contradicts them. */
static void still_beyond_anchor(struct stream *s)
{
    uint64_t at = validate_stream(s, 10000000000ULL);
    for (int k = 1; k <= 500; k++) {
        CHECK(expose(s, at + k * TICK, 40, 40, false));
        CHECK(!revoking(s));
    }
}

/* Returning toward the anchor, and on through it. */
static void return_toward_anchor(struct stream *s)
{
    uint64_t at = validate_stream(s, 30000000000ULL);
    for (int k = 1; k <= 60; k++) {
        CHECK(expose(s, at + k * TICK, 40 - k, 40 - k, false));
        CHECK(!revoking(s));
    }
}

/* A still hand's measured PnP noise (up to 9.4 degrees per exposure in replay) is far above a
 * tenth of its small turn; the ten-degree floor keeps it agreeing. */
static void noisy_still_hand(struct stream *s)
{
    uint64_t at = validate_stream(s, 20000000000ULL);
    for (int k = 1; k <= 400; k++) {
        CHECK(expose(s, at + k * TICK, 40, 40 + (k & 1 ? 9 : -9), false));
        CHECK(!revoking(s));
    }
}

/* Only an exposure agreeing as tightly as a validating one becomes the next anchor: one nine
 * degrees off, though agreeing, would put later honest exposures eighteen degrees off it. */
static void loose_exposure_never_anchors(struct stream *s)
{
    uint64_t at = validate_stream(s, 40000000000ULL);
    for (int k = 1; k <= 160; k++) CHECK(expose(s, at + k * TICK, 40, 40, false));
    CHECK(expose(s, at + 161 * TICK, 40, 49, false));
    for (int k = 162; k <= 170; k++) {
        CHECK(expose(s, at + k * TICK, 40, 31, false));
        CHECK(!revoking(s));
    }
}

/* Exposures whose motion could establish identity are flagged for the consumer, whatever the
 * candidate's verdict; a still hand's are not. */
static void motion_flags(struct stream *s)
{
    memset(s->sensor->pf.inspection_identity, 0, sizeof s->sensor->pf.inspection_identity);
    uint64_t at = 60000000000ULL;
    rift_pose_metrics *score = &s->frame.capture_state[0].score;
    const uint32_t motion = OHMD_CV1TOUCH_MATCH_INSPECTION_MOTION, identity = OHMD_CV1TOUCH_MATCH_INSPECTION_IDENTITY;
    struct { float own, seen; bool exported; uint32_t flags; } steps[] = {
        {0, 0, false, 0},           /* the anchor */
        {10, 10, false, 0},         /* too little turn */
        {40, 70, false, motion},    /* enough turn, a wrong candidate: a verdict */
        {80, 110, false, motion},   /* counted from that candidate: forty degrees, agreeing */
        {120, 150, false, motion},
        {125, 155, true, motion | identity},
        {125, 155, true, motion | identity}, /* identified and still */
    };
    for (size_t k = 0; k < sizeof steps / sizeof steps[0]; k++) {
        for (int i = 6; i < 12; i++) s->labels.blobs[i].led_id = LED_MAKE_ID(s->dev->id, i - 6);
        s->frame.bwobs = &s->labels;
        s->frame.exposure_info.devices[0].device_time_ns = at + k * TICK;
        s->frame.exposure_info.devices[0].capture_pose.orient = turn(steps[k].own);
        s->frame.exposure_info.devices[1].device_time_ns = at + k * TICK;
        s->frame.exposure_info.devices[1].capture_pose.orient = turn(0);
        s->frame.capture_state[0].final_cam_pose.orient = turn(45 + steps[k].seen);
        s->frame.detached_observation = true;
        *score = (rift_pose_metrics){.match_flags = RIFT_POSE_MATCH_GOOD};
        posef pose = s->frame.capture_state[0].final_cam_pose;
        CHECK(handle_found_pose(s->sensor, s->dev, &s->frame, &pose, score) == steps[k].exported);
        CHECK((score->match_flags & (motion | identity)) == steps[k].flags);
    }
}

/* A single PnP outlier is not exported, revokes nothing and does not become the anchor. */
static void single_outlier(struct stream *s)
{
    uint64_t at = validate_stream(s, 50000000000ULL);
    CHECK(!expose(s, at + TICK, 40, 160, false));
    CHECK(!revoking(s));
    for (int k = 2; k <= 400; k++) {
        CHECK(expose(s, at + k * TICK, 40, 40, false));
        CHECK(!revoking(s));
    }
}

/* Two outliers in a row are still no contradiction; the third in a row is. A swap to the
 * mirror alias, about 120 degrees off, is revoked on its third exposure, and identity then
 * starts over: agreeing exposures are no longer enough by themselves. */
static void sustained_contradiction(struct stream *s)
{
    uint64_t at = validate_stream(s, 70000000000ULL);
    for (int run = 1; run <= 2; run++) {
        for (int k = 0; k < run; k++) CHECK(!expose(s, at += TICK, 40, 160, false));
        CHECK(!revoking(s));
        CHECK(expose(s, at += TICK, 40, 40, false));
    }
    CHECK(!expose(s, at += TICK, 40, 160, false) && !revoking(s));
    CHECK(!expose(s, at += TICK, 40, 160, false) && !revoking(s));
    CHECK(!expose(s, at += TICK, 40, 160, false) && revoking(s));
    apply(s->sensor);
    CHECK(!(s->sensor->pf.inspection_pose_mask & (1u << s->dev->id)));
    for (int i = 6; i < 12; i++) CHECK(s->sensor->bw->last_observation->blobs[i].led_id == -1);
    for (int k = 1; k <= 10; k++) CHECK(!expose(s, at + k * TICK, 40, 40, false));
    CHECK(!revoking(s));
}

/* While the hand is out of view, cold searches may find the other hand's LEDs again and
 * again. Those candidates are rejected, but they are no evidence against the tracked hand's
 * labels and prior, which find it again when it returns. */
static void cold_aliases(struct stream *s)
{
    uint64_t at = validate_stream(s, 80000000000ULL);
    for (int k = 1; k <= 20; k++) {
        CHECK(!expose(s, at + k * TICK, 40, 160, true));
        CHECK(!revoking(s));
    }
    CHECK(expose(s, at + 21 * TICK, 40, 40, false));
    apply(s->sensor);
    CHECK(s->sensor->pf.inspection_pose_mask & (1u << s->dev->id));
}

/* Documented: before identity, a slow turn (8 degrees per second) never covers thirty degrees
 * within three seconds, so it never establishes identity; once identified, it keeps agreeing. */
static void slow_turn(struct stream *s)
{
    memset(s->sensor->pf.inspection_identity, 0, sizeof s->sensor->pf.inspection_identity);
    uint64_t at = 90000000000ULL;
    for (int k = 0; k <= 500; k++) CHECK(!expose(s, at + k * TICK, k * 0.16f, k * 0.16f, false));
    at = validate_stream(s, 110000000000ULL);
    for (int k = 1; k <= 500; k++) CHECK(expose(s, at + k * TICK, 40 + k * 0.16f, 40 + k * 0.16f, false));
}

/* Documented: a 20-degree wiggle (1 Hz) whose anchor falls at its middle never reaches thirty
 * degrees; anchored at an extreme it spans forty and does. Once identified, the wiggle keeps
 * agreeing. */
static void wiggle(struct stream *s)
{
    memset(s->sensor->pf.inspection_identity, 0, sizeof s->sensor->pf.inspection_identity);
    uint64_t at = 130000000000ULL;
    for (int k = 0; k <= 500; k++) {
        float a = 20 * sinf(2 * (float)M_PI * k * 0.02f);
        CHECK(!expose(s, at + k * TICK, a, a, false));
    }
    memset(s->sensor->pf.inspection_identity, 0, sizeof s->sensor->pf.inspection_identity);
    at = 150000000000ULL;
    bool identified = false;
    for (int k = 0; k <= 50; k++) {
        float a = 20 * cosf(2 * (float)M_PI * k * 0.02f);
        identified |= expose(s, at + k * TICK, a, a, false);
    }
    CHECK(identified);
    for (int k = 51; k <= 500; k++) {
        float a = 20 * cosf(2 * (float)M_PI * k * 0.02f);
        CHECK(expose(s, at + k * TICK, a, a, false));
    }
}

/* A genuine anchor must not identify a later cold alias of the other physical hand,
 * even when the hidden requested hand has meanwhile reached exactly that alias attitude. */
static void reacquired_alias(struct stream *s, bool validated)
{
    for (int changed_labels = 0; changed_labels <= 1; changed_labels++) {
        for (int angle = 120; angle <= 180; angle += 30) {
            uint64_t at = 170000000000ULL;
            s->reverse_labels = false;
            if (validated) at = validate_stream(s, at);
            else {
                memset(s->sensor->pf.inspection_identity, 0, sizeof s->sensor->pf.inspection_identity);
                CHECK(!expose(s, at, 0, 0, true));
                for (int k = 1; k <= 2; k++) CHECK(!expose(s, at += TICK, 40, 40, true));
            }
            /* Occlusion ends the old correspondence. The stationary other hand now supplies
             * new tracks (or different LED IDs on surviving tracks), with an alias that
             * happens to equal the hidden hand. Both endpoints must see the same hand. */
            if (changed_labels) s->reverse_labels = true;
            else for (int i = 6; i < 12; i++) s->labels.blobs[i].blob_id += 100;
            for (int k = 1; k <= 6; k++) {
                CHECK(!expose(s, at + k * TICK, angle, angle, true));
                CHECK(!s->sensor->pf.inspection_identity[s->dev->id].validated);
                CHECK(!(s->sensor->pending_label_updates & (1u << s->dev->id)));
            }
        }
    }
    s->reverse_labels = false;
}

/* A cold alternative is not a reacquisition if the original trusted raw tracks remain.
 * It must not export a new mapping or erase that still-continuous identity. */
static void unrelated_cold_fit(struct stream *s)
{
    s->labels = *s->sensor->bw->last_observation;
    uint64_t at = validate_stream(s, 180000000000ULL);
    s->frame.inspection_raw = *s->sensor->bw->last_observation;
    s->frame.inspection_raw_valid = true;
    uint64_t anchor = s->sensor->pf.inspection_identity[s->dev->id].first_ns;
    s->reverse_labels = true;
    for (int k = 1; k <= 4; k++) {
        CHECK(!expose(s, at + k * TICK, 160, 160, true));
        CHECK(s->sensor->pf.inspection_identity[s->dev->id].validated);
        CHECK(s->sensor->pf.inspection_identity[s->dev->id].first_ns == anchor);
        CHECK(!revoking(s));
        CHECK(!(s->sensor->pending_candidate & (1u << s->dev->id)));
    }
    s->reverse_labels = false;
    CHECK(expose(s, at + 5 * TICK, 40, 40, false));
}

/* Blobwatch can transfer all six software IDs to a different physical controller. The
 * source array belongs only to the fixture: the runtime must reject the image-space jump
 * before those inherited IDs can preserve an identity or complete its pending proof. */
static void migrated_source(struct stream *s, bool validated, int displacement, bool diverge, bool unjudged_jump)
{
    enum { WIDTH = 512, HEIGHT = 80 };
    uint8_t image[WIDTH * HEIGHT];
    int physical_source[6];
    uint32_t original_ids[6] = {0};
    blobwatch_free(s->sensor->bw);
    s->sensor->bw = blobwatch_new(BLOB_THRESHOLD_CV1);
    memset(s->sensor->pf.inspection_identity, 0, sizeof s->sensor->pf.inspection_identity);
    int change = validated ? 4 : 3;
    int exposures = displacement == 1 ? (diverge ? 4 : 200) : 5;
    for (int k = 0; k < change + exposures; k++) {
        bool migrated = k >= change;
        memset(image, 0, sizeof image);
        for (int b = 0; b < 6; b++) {
            int x = 20 + b * 30 + (migrated ? displacement : 0);
            physical_source[b] = migrated ? 3 - s->dev->id : s->dev->id;
            for (int y = 20; y < 25; y++)
                for (int dx = 0; dx < 5; dx++) image[y * WIDTH + x + dx] = 255;
        }
        blobservation *ob;
        blobwatch_process(s->sensor->bw, image, WIDTH, HEIGHT, 0, NULL, 0, &ob);
        CHECK(ob->num_blobs == 6);
        for (int b = 0; b < 6; b++) {
            if (!k) original_ids[b] = ob->blobs[b].blob_id;
            bool inherited = false;
            for (int j = 0; j < 6; j++) inherited |= ob->blobs[b].blob_id == original_ids[j];
            CHECK(inherited); /* The nearest-neighbour tracker really retained all six IDs. */
        }
        uint64_t at = 600000000000ULL + k * TICK;
        s->video.start_ts = at;
        rift_sensor_analysis_frame frame = {.bwobs = ob, .vframe = &s->video};
        rift_pose_finder_process_blobs_fast(&s->sensor->pf, &frame, NULL, 0);
        if (migrated && k == change && unjudged_jump) {
            /* No pose is found on the jump itself. Its lineage break must survive until
             * the later, stationary fit, whose own one-frame displacement is zero. */
            blobwatch_release_observation(s->sensor->bw, ob);
            continue;
        }
        frame.n_devices = 2;
        float later_turn = migrated ? (k - change) * 20 : 0;
        float angle = migrated ? 120 + later_turn : (k ? 40 : 0);
        frame.exposure_info.devices[0].device_time_ns = at;
        frame.exposure_info.devices[0].capture_pose.orient = turn(angle);
        frame.exposure_info.devices[1].device_time_ns = at;
        frame.exposure_info.devices[1].capture_pose.orient = turn(diverge ? 0 : later_turn);
        frame.capture_state[0].final_cam_pose.orient = turn(45 + (migrated && diverge ? 120 : angle));
        frame.capture_state[0].cv1touch_pose_valid = true;
        frame.quarantined_observation = !s->sensor->pf.inspection_identity[s->dev->id].validated;
        for (int b = 0; b < 6; b++) ob->blobs[b].led_id = LED_MAKE_ID(s->dev->id, ob->blobs[b].blob_id % 6);
        rift_pose_metrics score = {.match_flags = RIFT_POSE_MATCH_GOOD | RIFT_POSE_MATCH_STRONG};
        posef pose = frame.capture_state[0].final_cam_pose;
        bool accepted = handle_found_pose(s->sensor, s->dev, &frame, &pose, &score);
        if (!migrated) CHECK(accepted == (k == 3));
        if (displacement > 32 || !migrated || (diverge && k > change)) {
            for (int b = 0; b < 6; b++) {
                int exported = LED_OBJECT_ID(rift_sensor_frame_blob_label(&frame, &ob->blobs[b]));
                CHECK(exported != s->dev->id || physical_source[b] == s->dev->id);
            }
        } else {
            /* Document the residual: a one-pixel physical swap is indistinguishable from
             * tracking when both hands keep turning alike, even past the anchor's lifetime. */
            CHECK(accepted && s->sensor->pf.inspection_identity[s->dev->id].validated);
            for (int b = 0; b < 6; b++) {
                CHECK(physical_source[b] != s->dev->id);
                CHECK(LED_OBJECT_ID(rift_sensor_frame_blob_label(&frame, &ob->blobs[b])) == s->dev->id);
            }
        }
        if (displacement == 1 && diverge && k == change + 3) {
            CHECK(!accepted && revoking(s));
            CHECK(!s->sensor->pf.inspection_identity[s->dev->id].validated);
        }
        if (crossing_trace && migrated) {
            unsigned exported = 0;
            for (int b = 0; b < 6; b++)
                exported += LED_OBJECT_ID(rift_sensor_frame_blob_label(&frame, &ob->blobs[b])) == s->dev->id;
            printf("crossing %d %d %u %u\n", s->dev->id, accepted, score.match_flags, exported);
        }
        apply(s->sensor);
        if (displacement == 1 && diverge && k >= change + 3) {
            CHECK(!(s->sensor->pf.inspection_pose_mask & (1u << s->dev->id)));
            CHECK(!s->sensor->pf.inspection_candidate[s->dev->id].active);
            for (int b = 0; b < 6; b++) CHECK(LED_OBJECT_ID(ob->blobs[b].led_id) != s->dev->id);
        }
        blobwatch_release_observation(s->sensor->bw, ob);
    }
}

/* A cold result captured before the jump may finish afterward. The copied update cannot
 * relabel the new physical source or install the old prior when the fast worker applies it. */
static void pending_migration(struct stream *s, bool gap)
{
    s->video.start_ts = 610000000000ULL;
    rift_sensor_analysis_frame raw = {.bwobs = &s->labels, .vframe = &s->video};
    rift_pose_finder_process_blobs_fast(&s->sensor->pf, &raw, NULL, 0);
    s->frame.inspection_epoch = raw.inspection_epoch;
    memcpy(s->frame.inspection_tracks, raw.inspection_tracks, sizeof raw.inspection_tracks);
    uint64_t at = validate_stream(s, s->video.start_ts);
    s->video.start_ts = at + TICK;
    rift_pose_finder_process_blobs_fast(&s->sensor->pf, &raw, NULL, 0);
    s->frame.inspection_epoch = raw.inspection_epoch;
    memcpy(s->frame.inspection_tracks, raw.inspection_tracks, sizeof raw.inspection_tracks);
    CHECK(expose(s, at + TICK, 40, 40, true));
    CHECK(s->sensor->pending_label_updates & (1u << s->dev->id));
    raw.bwobs = s->sensor->bw->last_observation;
    if (!gap) for (int b = 6; b < 12; b++) raw.bwobs->blobs[b].x += 200;
    s->video.start_ts = at + (gap ? 6 : 2) * TICK;
    rift_pose_finder_process_blobs_fast(&s->sensor->pf, &raw, NULL, 0);
    apply(s->sensor);
    CHECK(!(s->sensor->pf.inspection_pose_mask & (1u << s->dev->id)));
    CHECK(!s->sensor->pf.inspection_identity[s->dev->id].validated);
    for (int b = 6; b < 12; b++) CHECK(LED_OBJECT_ID(raw.bwobs->blobs[b].led_id) != s->dev->id);
}

/* Starting placement extends the inspection, preserving the hand already identified.
 * The other hand can join it, but must establish its own correspondence and identity. */
static void placement_handoff(struct stream *s)
{
    uint64_t at = validate_stream(s, 210000000000ULL);
    uint64_t anchor = s->sensor->pf.inspection_identity[s->dev->id].first_ns;
    s->sensor->long_analysis_busy = true;
    CHECK(rift_sensor_clear_cv1touch_pose(s->sensor, "CameraA", 180000000000ULL, 6));
    apply(s->sensor);
    CHECK(s->sensor->pending_clear && s->sensor->search_cancelled);
    CHECK(s->sensor->pf.inspection_identity[s->dev->id].validated);
    s->sensor->long_analysis_busy = false;
    apply(s->sensor);
    CHECK(!s->sensor->pending_clear && !s->sensor->search_cancelled);
    CHECK(s->sensor->pf.inspection_identity[s->dev->id].validated);
    CHECK(s->sensor->pf.inspection_identity[s->dev->id].first_ns == anchor);
    CHECK(s->sensor->pf.inspection_pose_mask & (1u << s->dev->id));
    CHECK(!s->sensor->pf.inspection_identity[3 - s->dev->id].validated);
    CHECK(expose(s, at + TICK, 40, 40, false));
    /* Retaining identity across the phase change cannot retain it across track loss. */
    for (int i = 6; i < 12; i++) s->labels.blobs[i].blob_id += 100;
    CHECK(!expose(s, at + 2 * TICK, 150, 150, true));
    CHECK(!s->sensor->pf.inspection_identity[s->dev->id].validated);
    /* A retry after expiry is a fresh inspection even if it requests the same hands. */
    validate_stream(s, at + 1000000000);
    s->sensor->search_deadline = now_ticks;
    CHECK(rift_sensor_clear_cv1touch_pose(s->sensor, "CameraA", 180000000000ULL, 6));
    apply(s->sensor);
    CHECK(!s->sensor->pf.inspection_identity[s->dev->id].validated);
    CHECK(!s->sensor->pf.inspection_pose_mask);
}

/* Even unchanged raw IDs cannot bridge a camera outage; all intermediate observations
 * must be consecutive, not merely the two frames whose poses happened to be judged. */
static void frame_gap(struct stream *s)
{
    rift_sensor_analysis_frame raw = {.bwobs = &s->labels, .vframe = &s->video};
    s->video.start_ts = 1000000000;
    rift_pose_finder_process_blobs_fast(&s->sensor->pf, &raw, NULL, 0);
    s->frame.inspection_epoch = raw.inspection_epoch;
    uint64_t at = validate_stream(s, 1000000000);
    /* An intervening empty exposure still participates in the raw-frame clock. */
    blobservation empty = {0}; raw.bwobs = &empty;
    s->video.start_ts = at + 100000000;
    rift_pose_finder_process_blobs_fast(&s->sensor->pf, &raw, NULL, 0);
    s->frame.inspection_epoch = raw.inspection_epoch;
    CHECK(!expose(s, at + 120000000, 40, 40, true));
    CHECK(!s->sensor->pf.inspection_identity[s->dev->id].validated);
    CHECK(revoking(s));
}

/* UNKNOWN must suppress output, not erase the live labels the next raw frame inherits.
 * Use blobwatch's real image extraction and propagation, not a copied fast observation. */
static void live_outlier(struct stream *s)
{
    enum { WIDTH = 160, HEIGHT = 80 };
    uint8_t image[WIDTH * HEIGHT] = {0};
    for (int b = 0; b < 12; b++) {
        int x = 10 + (b % 6) * 24, y = 15 + (b / 6) * 35;
        for (int dy = 0; dy < 5; dy++)
            for (int dx = 0; dx < 5; dx++) image[(y + dy) * WIDTH + x + dx] = 255;
    }
    blobwatch_free(s->sensor->bw);
    s->sensor->bw = blobwatch_new(BLOB_THRESHOLD_CV1);
    blobservation *ob;
    blobwatch_process(s->sensor->bw, image, WIDTH, HEIGHT, 0, NULL, 0, &ob);
    CHECK(ob == s->sensor->bw->last_observation && ob->num_blobs == 12);
    s->labels = *ob;
    for (int i = 0; i < 6; i++) s->labels.blobs[i].led_id = LED_MAKE_ID(3 - s->dev->id, i);
    uint64_t at = validate_stream(s, 190000000000ULL);
    s->frame.bwobs = ob;
    s->frame.detached_observation = false;
    s->frame.exposure_info.devices[0].device_time_ns = at + TICK;
    s->frame.exposure_info.devices[1].device_time_ns = at + TICK;
    s->frame.capture_state[0].final_cam_pose.orient = turn(205);
    s->frame.capture_state[0].cv1touch_pose_valid = true;
    rift_pose_metrics score = {.match_flags = RIFT_POSE_MATCH_GOOD};
    posef pose = s->frame.capture_state[0].final_cam_pose;
    CHECK(!handle_found_pose(s->sensor, s->dev, &s->frame, &pose, &score));
    CHECK(!s->frame.capture_state[0].cv1touch_pose_valid && !revoking(s));
    for (int i = 6; i < 12; i++) {
        CHECK(LED_OBJECT_ID(ob->blobs[i].led_id) == s->dev->id);
        CHECK(rift_sensor_frame_blob_label(&s->frame, &ob->blobs[i]) == LED_INVALID_ID);
    }
    blobwatch_release_observation(s->sensor->bw, ob);
    blobwatch_process(s->sensor->bw, image, WIDTH, HEIGHT, 0, NULL, 0, &ob);
    CHECK(ob == s->sensor->bw->last_observation && ob->num_blobs == 12);
    for (int i = 6; i < 12; i++) CHECK(LED_OBJECT_ID(ob->blobs[i].led_id) == s->dev->id);
    blobwatch_release_observation(s->sensor->bw, ob);
}

static ohmd_cv1touch_inspection_identity identity_notices[8];
static unsigned n_identity_notices;
static void identity_notice(void *data, const ohmd_cv1touch_inspection_identity *event, uint64_t at)
{
    (void)data; (void)at;
    CHECK(n_identity_notices < 8);
    identity_notices[n_identity_notices++] = *event;
}

static ohmd_cv1touch_frame_info released_identity_frame;
static void identity_frame(void *data, const ohmd_cv1touch_frame_info *info, int n,
    const ohmd_cv1touch_blob *blobs)
{
    (void)data; (void)n; (void)blobs;
    released_identity_frame = *info;
}

static void epoch_lifetimes(struct stream *s)
{
    n_identity_notices = 0;
    s->sensor->ohmd_ctx->cv1touch_identity_cb = identity_notice;
    s->sensor->pf.next_identity_epoch = (1ULL << 32);
    uint64_t at = validate_stream(s, 240000000000ULL);
    uint64_t first = s->frame.capture_state[0].identity_epoch;
    CHECK(first == (1ULL << 32) + 1 && n_identity_notices == 1);
    rift_sensor_analysis_frame delayed = s->frame;
    delayed.long_observation = *s->frame.bwobs;
    delayed.bwobs = &delayed.long_observation;
    delayed.detached_observation = true;
    CHECK(identity_notices[0].state == OHMD_CV1TOUCH_IDENTITY_STARTED);
    CHECK(identity_notices[0].epoch == first && identity_notices[0].device_id == s->dev->id);
    /* A gap revokes future use of these labels but says nothing against earlier checks. */
    s->frame.inspection_epoch++;
    CHECK(!expose(s, at + TICK, 40, 40, true));
    CHECK(n_identity_notices == 2 && identity_notices[1].epoch == first);
    CHECK(identity_notices[1].state == OHMD_CV1TOUCH_IDENTITY_ENDED);
    apply(s->sensor);
    at = validate_stream(s, 250000000000ULL);
    uint64_t second = s->frame.capture_state[0].identity_epoch;
    CHECK(second > first && n_identity_notices == 3);
    for (int k = 1; k <= 3; k++) {
        CHECK(!expose(s, at + k * TICK, 40, 160, false));
        CHECK(n_identity_notices == (k == 3 ? 4u : 3u));
    }
    CHECK(identity_notices[3].epoch == second);
    CHECK(identity_notices[3].state == OHMD_CV1TOUCH_IDENTITY_INVALIDATED);
    CHECK(s->frame.capture_state[0].identity_epoch == 0);
    apply(s->sensor);
    CHECK(n_identity_notices == 4); /* label cleanup cannot reclassify it as a normal end */
    /* An old completed frame keeps its own full-width tag after a newer epoch ended. */
    s->sensor->ohmd_ctx->cv1touch_frame_cb = identity_frame;
    ohmd_lock_mutex(s->sensor->sensor_lock);
    release_capture_frame(s->sensor, &delayed);
    CHECK(POP_QUEUE(&s->sensor->capture_frame_q) == &delayed);
    ohmd_unlock_mutex(s->sensor->sensor_lock);
    CHECK(released_identity_frame.n_devices > 0);
    CHECK(released_identity_frame.devices[0].device_id == s->dev->id);
    CHECK(released_identity_frame.devices[0].identity_epoch_lo == 1);
    CHECK(released_identity_frame.devices[0].identity_epoch_hi == 1);
    s->sensor->ohmd_ctx->cv1touch_frame_cb = NULL;
    s->sensor->ohmd_ctx->cv1touch_identity_cb = NULL;
}

static void latched_cases(rift_sensor_ctx *sensor, rift_tracked_device *dev, blobservation *latest)
{
    void (*cases[])(struct stream *) = {still_beyond_anchor, return_toward_anchor, noisy_still_hand,
        loose_exposure_never_anchors, motion_flags, single_outlier, sustained_contradiction,
        cold_aliases, slow_turn, wiggle, epoch_lifetimes};
    for (size_t k = 0; k < sizeof cases / sizeof cases[0]; k++) {
        struct stream s = new_stream(sensor, dev, latest);
        cases[k](&s);
        apply(sensor);
    }
}

/* An older detached result queued before a newer fast-path validation must not overwrite or
 * revoke the newer labels when the fast worker applies its queue. */
static void newer_fast_labels(rift_sensor_ctx *sensor, rift_tracked_device *dev, blobservation *latest)
{
    struct stream s = new_stream(sensor, dev, latest);
    uint64_t at = validate_stream(&s, 200000000000ULL);
    CHECK(expose(&s, at + 20000000, 40, 40, true));
    CHECK(sensor->pending_label_updates & (1u << dev->id));
    for (int i = 6; i < 12; i++) s.labels.blobs[i].led_id = LED_MAKE_ID(dev->id, i == 11 ? 6 : i - 6);
    s.frame.bwobs = &s.labels;
    s.frame.detached_observation = false;
    s.frame.exposure_info.devices[0].device_time_ns = at + 40000000;
    s.frame.exposure_info.devices[1].device_time_ns = at + 40000000;
    rift_pose_metrics score = {.match_flags = RIFT_POSE_MATCH_GOOD | RIFT_POSE_MATCH_STRONG};
    posef pose = s.frame.capture_state[0].final_cam_pose;
    CHECK(handle_found_pose(sensor, dev, &s.frame, &pose, &score));
    apply(sensor);
    for (int i = 6; i < 12; i++) CHECK(latest->blobs[i].led_id == LED_MAKE_ID(dev->id, i == 11 ? 6 : i - 6));
    CHECK(sensor->pf.inspection_pose_mask & (1u << dev->id));
}

/* A fast match whose identity is still pending consumed its exposure: a cold search of it
 * could only be discarded, so the frame must not go to long analysis for that hand. */
static void pending_fast_match(rift_sensor_ctx *sensor, rift_tracked_device *dev, blobservation *latest)
{
    memset(sensor->pf.inspection_identity, 0, sizeof sensor->pf.inspection_identity);
    awake_device = dev->id;
    rift_led points[6] = {0};
    rift_leds model = {.points = points, .num_points = 6};
    rift_tracked_device inspected = {.id = dev->id, .leds = &model};
    rift_tracked_device *devices[] = {&inspected};
    blobservation obs = *latest;
    for (int i = 0; i < 6; i++) obs.blobs[i].led_id = LED_MAKE_ID(dev->id, i);
    rift_sensor_analysis_frame fast = {.bwobs = &obs, .n_devices = 1};
    fast.exposure_info.devices[0].capture_pose.orient.w = 1;
    fast.exposure_info.devices[0].device_time_ns = 300000000000ULL;
    rift_pose_finder_process_blobs_fast(&sensor->pf, &fast, devices, 0x6);
    CHECK(POSE_HAS_FLAGS(&fast.capture_state[0].score, RIFT_POSE_MATCH_GOOD));
    CHECK(!fast.capture_state[0].cv1touch_pose_valid);
    CHECK(sensor->pf.inspection_identity[dev->id].last_ns == 300000000000ULL);
    CHECK(fast.capture_state[0].found_device_pose);
    CHECK(!fast.need_long_analysis);
}

struct transfer_case {
    rift_sensor_ctx *sensor;
    rift_tracked_device *device;
    rift_sensor_analysis_frame frame;
    blobservation labels;
    ohmd_video_frame video;
    bool done;
    bool started;
    bool fast_read;
};

static void *produce_labels(void *data)
{
    struct transfer_case *c = data;
    __atomic_store_n(&c->started, true, __ATOMIC_RELEASE);
    while (!__atomic_load_n(&c->fast_read, __ATOMIC_ACQUIRE)) sched_yield();
    for (int k = 0; k < 1000; k++) {
        c->frame.exposure_info.devices[0].device_time_ns = 1100000000 + k * 1000000;
        c->frame.capture_state[0].cv1touch_pose_valid = true;
        rift_pose_metrics *score = &c->frame.capture_state[0].score;
        score->match_flags = RIFT_POSE_MATCH_GOOD | RIFT_POSE_MATCH_STRONG;
        posef world = c->frame.capture_state[0].final_cam_pose;
        CHECK(handle_found_pose(c->sensor, c->device, &c->frame, &world, score));
        sched_yield();
    }
    __atomic_store_n(&c->done, true, __ATOMIC_RELEASE);
    return NULL;
}

static void check_other_hand(rift_sensor_ctx *sensor, blobservation *latest, int other_id)
{
    for (int i = 0; i < 6; i++) CHECK(LED_OBJECT_ID(latest->blobs[i].led_id) == other_id);
    awake_device = other_id;
    rift_led points[6] = {0};
    rift_leds model = {.points = points, .num_points = 6};
    rift_tracked_device dev = {.id = other_id, .leds = &model};
    rift_tracked_device *devices[] = {&dev};
    blobservation obs = *latest;
    rift_sensor_analysis_frame fast = {.bwobs = &obs, .n_devices = 1};
    fast.exposure_info.devices[0].capture_pose.orient.w = 1;
    rift_pose_finder_process_blobs_fast(&sensor->pf, &fast, devices, 0x6);
    CHECK(fast.capture_state[0].found_device_pose);
}

int main(int argc, char **argv)
{
    fast_worker = pthread_self();
    identity_cases();
    discrimination_cases();
    for (int requested = 1; requested <= 2; requested++) {
        int other = 3 - requested;
        ohmd_context ctx = {.monotonic_ticks_per_sec = 1000000000};
        rift_sensor_device usb = {0}; /* a pose commit checks that its USB device is still there */
        rift_sensor_ctx sensor = {.ohmd_ctx = &ctx, .dev = &usb};
        snprintf(sensor.serial_no, sizeof sensor.serial_no, "CameraA");
        sensor.sensor_lock = ohmd_create_mutex(&ctx);
        sensor.bw = blobwatch_new(BLOB_THRESHOLD_CV1);
        blobservation *latest = bw_pop_queue(&sensor.bw->observation_q);
        sensor.bw->last_observation = latest;
        latest->num_blobs = 12;
        for (int i = 0; i < 12; i++) latest->blobs[i] = (struct blob){
            .blob_id = i + 1, .led_id = i < 6 ? LED_MAKE_ID(other, i) : -1};
        blobservation found = *latest;
        found.num_blobs = 6;
        for (int i = 0; i < 6; i++) found.blobs[i].led_id = LED_MAKE_ID(requested, i);
        rift_tracked_device dev = {.id = requested}, opposite = {.id = other};
        sensor.devices[0] = &dev; sensor.devices[1] = &opposite; sensor.n_devices = 2;
        sensor.pf.inspection_mask = 1u << requested;
        sensor.pf.have_camera_pose = true;
        sensor.pf.camera_pose.orient.w = 1;
        sensor.pf.inspection_previous_raw = *latest;
        sensor.pf.pose_cb = (rift_pose_finder_cb)handle_found_pose; sensor.pf.pose_cb_data = &sensor;
        if (argc > 1) {
            struct stream s = new_stream(&sensor, &dev, latest);
            if (!strcmp(argv[1], "reacquired-before")) reacquired_alias(&s, false);
            else if (!strcmp(argv[1], "reacquired-after")) reacquired_alias(&s, true);
            else if (!strcmp(argv[1], "live-outlier")) live_outlier(&s);
            else if (!strcmp(argv[1], "frame-gap")) frame_gap(&s);
            else if (!strcmp(argv[1], "candidate")) quarantined_candidate(&s);
            else if (!strcmp(argv[1], "unrelated")) unrelated_cold_fit(&s);
            else if (!strcmp(argv[1], "placement")) placement_handoff(&s);
            else if (!strcmp(argv[1], "wrong-candidate")) candidate_wrong_motion(&s);
            else if (!strcmp(argv[1], "candidate-jump")) candidate_image_jump(&s);
            else if (!strcmp(argv[1], "pending-migration")) pending_migration(&s, false);
            else if (!strcmp(argv[1], "pending-gap")) pending_migration(&s, true);
            else if (!strcmp(argv[1], "migration-unjudged")) migrated_source(&s, true, 200, false, true);
            else if (!strcmp(argv[1], "migration-before")) migrated_source(&s, false, 200, false, false);
            else if (!strcmp(argv[1], "migration-after")) migrated_source(&s, true, 200, false, false);
            else if (!strcmp(argv[1], "epoch-lifetimes")) epoch_lifetimes(&s);
            else if (!strcmp(argv[1], "migration-crossing")) migrated_source(&s, true, 1, true, false);
            else if (!strcmp(argv[1], "migration-rigid")) migrated_source(&s, true, 1, false, false);
            else if (!strcmp(argv[1], "migration-trace")) {
                crossing_trace = true;
                migrated_source(&s, true, 1, true, false);
            }
            else abort();
            ohmd_destroy_mutex(sensor.sensor_lock);
            blobwatch_free(sensor.bw);
            continue;
        }

        ohmd_video_frame video = {.start_ts = 1};
        rift_sensor_analysis_frame frame = {.vframe = &video, .bwobs = &found, .n_devices = 2,
            .long_analysis_start_ts = 1, .detached_observation = true};
        frame.exposure_info.devices[0].capture_pose.orient.w = 1;
        frame.exposure_info.devices[0].device_time_ns = 1000000000;
        frame.exposure_info.devices[1].capture_pose.orient.w = 1;
        frame.exposure_info.devices[1].device_time_ns = 1000000000;
        frame.capture_state[0].final_cam_pose.orient = turn(120);
        posef pose = frame.capture_state[0].final_cam_pose;
        rift_pose_metrics score = {.match_flags = RIFT_POSE_MATCH_GOOD | RIFT_POSE_MATCH_STRONG};
        CHECK(!handle_found_pose(&sensor, &dev, &frame, &pose, &score));
        CHECK(sensor.pending_label_pose == 0 && sensor.pf.inspection_pose_mask == 0);
        check_other_hand(&sensor, latest, other);

        /* The long worker may read and label only its detached observation. */
        rift_sensor_analysis_frame queued = {.vframe = &video, .bwobs = latest};
        queue_long_frame(&sensor, &queued);
        CHECK(queued.bwobs != latest && queued.detached_observation);
        CHECK(POP_QUEUE(&sensor.long_analysis_q) == &queued);
        queued.bwobs->blobs[0].led_id = LED_MAKE_ID(requested, 0);
        CHECK(LED_OBJECT_ID(latest->blobs[0].led_id) == other);

        /* Seed independently discriminating motion, never an absolute camera agreement. */
        memset(sensor.pf.inspection_identity, 0, sizeof sensor.pf.inspection_identity);
        quatf still = turn(0);
        struct stream seeded = new_stream(&sensor, &dev, latest);
        validate_stream(&seeded, 1000000000);
        struct transfer_case c = {.sensor = &sensor, .device = &dev};
        c.labels = *latest;
        for (int i = 6; i < 12; i++) c.labels.blobs[i].led_id = LED_MAKE_ID(requested, i - 6);
        c.frame = (rift_sensor_analysis_frame){.bwobs = &c.labels, .vframe = &c.video, .n_devices = 2,
            .detached_observation = true, .long_analysis_start_ts = 1};
        c.frame.inspection_epoch = seeded.frame.inspection_epoch;
        memcpy(c.frame.inspection_tracks, seeded.frame.inspection_tracks, sizeof c.frame.inspection_tracks);
        c.frame.exposure_info.devices[0].capture_pose.orient = turn(40);
        c.frame.exposure_info.devices[1].capture_pose.orient = still;
        c.frame.exposure_info.devices[1].device_time_ns = 1000000000;
        c.frame.capture_state[0].final_cam_pose.orient = turn(85);
        /* A real pthread producer runs the runtime callback while the fast worker reads and
         * applies actual blobwatch transfers. TSan covers both loops when enabled externally. */
        sensor.long_analysis_busy = true;
        pthread_t worker;
        CHECK(pthread_create(&worker, NULL, produce_labels, &c) == 0);
        while (!__atomic_load_n(&c.started, __ATOMIC_ACQUIRE)) sched_yield();
        unsigned reads = 0;
        while (!__atomic_load_n(&c.done, __ATOMIC_ACQUIRE)) {
            for (int i = 0; i < 12; i++) CHECK(latest->blobs[i].led_id >= -1);
            ohmd_lock_mutex(sensor.sensor_lock);
            apply_pending_pose(&sensor);
            ohmd_unlock_mutex(sensor.sensor_lock);
            reads++;
            __atomic_store_n(&c.fast_read, true, __ATOMIC_RELEASE);
            sched_yield();
        }
        CHECK(pthread_join(worker, NULL) == 0);
        CHECK(reads > 0);
        sensor.long_analysis_busy = false;
        ohmd_lock_mutex(sensor.sensor_lock); apply_pending_pose(&sensor); ohmd_unlock_mutex(sensor.sensor_lock);
        CHECK(sensor.pf.inspection_pose_mask & (1u << requested));
        check_other_hand(&sensor, latest, other);

        /* An older cold result is unknown, not a contradiction of newer validated frames.
         * It must not revoke the fast prior established while that search was running. */
        c.frame.exposure_info.devices[0].device_time_ns = 1200000000;
        c.frame.capture_state[0].final_cam_pose.orient = turn(170);
        CHECK(!handle_found_pose(&sensor, &dev, &c.frame, &pose, &c.frame.capture_state[0].score));
        ohmd_lock_mutex(sensor.sensor_lock); apply_pending_pose(&sensor); ohmd_unlock_mutex(sensor.sensor_lock);
        CHECK(sensor.pf.inspection_pose_mask & (1u << requested));
        for (int i = 6; i < 12; i++) CHECK(LED_OBJECT_ID(latest->blobs[i].led_id) == requested);
        check_other_hand(&sensor, latest, other);

        /* The tracked correspondences losing independent identity, three exposures in a row,
         * revoke labels and the private fast prior. It must not leave the other controller
         * hidden behind a rejected correspondence. */
        c.frame.capture_state[0].final_cam_pose.orient = turn(170);
        c.frame.detached_observation = false;
        c.frame.bwobs = latest;
        for (int k = 0; k < 3; k++) {
            c.frame.exposure_info.devices[0].device_time_ns = 2200000000 + k * 20000000;
            CHECK(!handle_found_pose(&sensor, &dev, &c.frame, &pose, &c.frame.capture_state[0].score));
            CHECK(!(sensor.pending_label_revoke & (1u << requested)) == (k < 2));
        }
        c.frame.detached_observation = true;
        c.frame.bwobs = &c.labels;
        ohmd_lock_mutex(sensor.sensor_lock); apply_pending_pose(&sensor); ohmd_unlock_mutex(sensor.sensor_lock);
        CHECK(sensor.pf.inspection_pose_mask == 0);
        for (int i = 6; i < 12; i++) CHECK(latest->blobs[i].led_id == -1);
        check_other_hand(&sensor, latest, other);

        /* Independently identify it again, so restoration must clear a live prior and labels. */
        memset(sensor.pf.inspection_identity, 0, sizeof sensor.pf.inspection_identity);
        seeded = new_stream(&sensor, &dev, latest);
        CHECK(!expose(&seeded, 2300000000, 0, 0, true));
        c.frame.inspection_epoch = seeded.frame.inspection_epoch;
        memcpy(c.frame.inspection_tracks, seeded.frame.inspection_tracks, sizeof c.frame.inspection_tracks);
        c.frame.capture_state[0].final_cam_pose.orient = turn(85);
        for (int i = 6; i < 12; i++) c.labels.blobs[i].led_id = LED_MAKE_ID(requested, i - 6);
        for (int k = 0; k < 3; k++) {
            c.frame.exposure_info.devices[0].device_time_ns = 2320000000 + k * 20000000;
            CHECK(handle_found_pose(&sensor, &dev, &c.frame, &pose,
                &c.frame.capture_state[0].score) == (k == 2));
            if (k < 2)
                for (int i = 6; i < 12; i++) c.labels.blobs[i].led_id = LED_MAKE_ID(requested, i - 6);
        }
        ohmd_lock_mutex(sensor.sensor_lock); apply_pending_pose(&sensor); ohmd_unlock_mutex(sensor.sensor_lock);
        CHECK(sensor.pf.inspection_pose_mask & (1u << requested));
        {
            struct stream s = new_stream(&sensor, &dev, latest);
            reacquired_alias(&s, false);
            reacquired_alias(&s, true);
            unrelated_cold_fit(&s);
            s.frame.inspection_raw_valid = false;
            frame_gap(&s);
            s.frame.inspection_epoch = 0;
        }
        latched_cases(&sensor, &dev, latest);
        check_other_hand(&sensor, latest, other);
        newer_fast_labels(&sensor, &dev, latest);
        check_other_hand(&sensor, latest, other);
        pending_fast_match(&sensor, &dev, latest);
        /* A former label of the requested hand cannot erase the other hand's current label. */
        latest->blobs[0].prev_led_id = LED_MAKE_ID(requested, 0);
        ohmd_cv1touch_sensor_pose restored = {.orient = {0, 0, 0, 1}};
        snprintf(restored.serial, sizeof restored.serial, "CameraA");
        CHECK(rift_sensor_set_cv1touch_pose(&sensor, &restored));
        CHECK(next_fast_frame(&sensor) == NULL);
        CHECK(sensor.pf.inspection_pose_mask == 0);
        for (int i = 6; i < 12; i++) CHECK(latest->blobs[i].led_id == -1);
        check_other_hand(&sensor, latest, other);
        sensor.pf.inspection_mask = 1u << requested;
        {
            struct stream s = new_stream(&sensor, &dev, latest);
            pending_migration(&s, false);
            s = new_stream(&sensor, &dev, sensor.bw->last_observation);
            pending_migration(&s, true);
            s = new_stream(&sensor, &dev, sensor.bw->last_observation);
            candidate_image_jump(&s);
            s = new_stream(&sensor, &dev, sensor.bw->last_observation);
            quarantined_candidate(&s);
            live_outlier(&s);
            s = new_stream(&sensor, &dev, sensor.bw->last_observation);
            placement_handoff(&s);
            s = new_stream(&sensor, &dev, sensor.bw->last_observation);
            candidate_wrong_motion(&s);
        }
        {
            struct stream s = new_stream(&sensor, &dev, sensor.bw->last_observation);
            migrated_source(&s, false, 200, false, false);
            migrated_source(&s, true, 200, false, false);
            migrated_source(&s, true, 200, false, true);
            migrated_source(&s, true, 1, true, false);
            migrated_source(&s, true, 1, false, false);
            s = new_stream(&sensor, &dev, sensor.bw->last_observation);
        }
        ohmd_destroy_mutex(sensor.sensor_lock);
        blobwatch_free(sensor.bw);
    }
    puts("test_inspection_handoff ok: identity, latching, isolation, and interleaved label transfer");
    return 0;
}
