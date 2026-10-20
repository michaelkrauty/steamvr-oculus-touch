// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* Actual correspondence search and OpenCV PnP, with fictional geometry. A cold match must
 * supply subsequent independent frames at camera rate despite the disputed calibration;
 * forty serial cold searches per five seconds are not assumed. No devices or recorded labels. */
#define _GNU_SOURCE
#include "omath.c"
#include "rift-sensor-maths.c"
#include "rift-sensor-pose-helper.c"
#include "led_search.c"
#include "correspondence_search.c"
#include "rift-sensor-pose-search.c"
#include "test_util.h"
#include <time.h>

#define LEDS 12
static int deliveries;
/* The other hand's IMU orientation at the exposure being searched; NULL when it is absent. */
static const quatf *other_hand;
static rift_identity_verdict last_verdict;
static int agreements;

/* Every cooperative stop poll, with the trials the search counts between its resets, and
 * how often consecutive polls each follow a trial: a trial preceded by a poll of its own,
 * after its blobs were selected, never follows the previous trial's poll directly. */
static unsigned long polls, trials, trials_seen, back_to_back;
static bool counting_stop(void *data)
{
    static unsigned long last;
    correspondence_search_t *cs = data;
    unsigned long delta = cs->num_trials >= trials_seen ? cs->num_trials - trials_seen : cs->num_trials;
    if (cs->num_trials < trials_seen)
        trials += trials_seen;
    trials_seen = cs->num_trials;
    back_to_back += delta && last;
    last = delta;
    polls++;
    return false;
}

bool rift_tracked_device_get_latest_exposure_info_pose(rift_tracked_device *dev,
    rift_tracked_device_exposure_info *info)
{
    (void)dev;
    return info->fusion_slot >= 0;
}

static bool receive(void *data, rift_tracked_device *dev, rift_sensor_analysis_frame *frame,
    posef *world, rift_pose_metrics *score)
{
    (void)world;
    CHECK(POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_GOOD));
    rift_pose_finder *pf = data;
    bool judgeable;
    rift_correspondence_verdict correspondence = rift_pose_finder_check_inspection_blobs(pf, dev->id, frame);
    last_verdict = correspondence == RIFT_CORRESPONDENCE_UNRELATED ? RIFT_IDENTITY_UNKNOWN :
        rift_pose_finder_identify_inspection(pf, dev->id,
        frame->exposure_info.devices[0].device_time_ns,
        &frame->exposure_info.devices[0].capture_pose.orient, other_hand,
        &frame->capture_state[0].final_cam_pose.orient, !frame->detached_observation, &judgeable);
    if (last_verdict != RIFT_IDENTITY_AGREES) {
        frame->capture_state[0].cv1touch_pose_valid = false;
        if (frame->detached_observation) {
            rift_pose_finder_set_inspection_candidate(pf, dev->id, frame->bwobs, frame->inspection_epoch, frame->inspection_tracks,
                &frame->exposure_info.devices[0].capture_pose, &frame->capture_state[0].final_cam_pose);
            rift_clear_blob_labels(frame->bwobs->blobs, frame->bwobs->num_blobs, dev->id);
        }
        return false;
    }
    if (frame->quarantined_observation) {
        rift_pose_finder_set_inspection_pose(pf, dev->id, &frame->exposure_info.devices[0].capture_pose,
            &frame->capture_state[0].final_cam_pose);
        pf->inspection_candidate[dev->id].active = false;
    }
    deliveries++;
    agreements++;
    return true;
}

static quatf axis_turn(float x, float y, float z, float degrees)
{
    vec3f axis = {{x, y, z}};
    ovec3f_normalize_me(&axis);
    quatf q;
    oquatf_init_axis(&q, &axis, DEG_TO_RAD(degrees));
    return q;
}

static void irregular_leds(rift_led *points)
{
    for (int i = 0; i < LEDS; i++) {
        points[i] = (rift_led){.id = i};
        /* An irregular constellation avoids ring aliases without copying factory geometry. */
        points[i].pos = (vec3f){{0.08f * cosf(i * 2.4f), 0.07f * sinf(i * 1.7f), 0.012f * sinf(i * 0.9f)}};
        points[i].dir.z = -1;
    }
}

static void project(rift_led *points, rift_sensor_camera_params *calib, posef *pose, blobservation *obs)
{
    vec3f pixels[LEDS];
    rift_project_points(points, LEDS, calib, pose, pixels);
    *obs = (blobservation){.num_blobs = LEDS};
    for (int i = 0; i < LEDS; i++)
        obs->blobs[i] = (struct blob){.blob_id = i + 1, .x = pixels[i].x, .y = pixels[i].y,
            .width = 3, .height = 3, .area = 9, .led_id = -1};
}

static void expose_frame(rift_sensor_analysis_frame *frame, blobservation *obs, posef own, uint64_t at)
{
    *frame = (rift_sensor_analysis_frame){.bwobs = obs, .n_devices = 1};
    frame->exposure_info.devices[0].capture_pose = own;
    frame->exposure_info.devices[0].device_time_ns = at;
    frame->exposure_info.devices[0].fusion_slot = 0;
    frame->exposure_info.devices[0].pos_error = (vec3f){{.05f, .05f, .05f}};
    frame->exposure_info.devices[0].rot_error = (vec3f){{.25f, .25f, .25f}};
}

/* The idealised mirror alias through the real search: the right model fits the left hand's
 * LEDs exactly at a fixed 120-degree offset (the factory models' mirror images admit only an
 * approximate rigid fit). Only the left hand is in view. Its right-model pose turns as the
 * left hand does, conjugated by that offset. When the right hand's IMU turn is that same
 * conjugate (turns of equal angle about axes the offset relates), the alias agrees with the
 * requested hand's IMU exactly, while the hands' world turns differ by more than half the
 * span, which a comparison of world turns accepted. Equal turn angles reveal it. A rigid
 * turn about the offset's own axis stays unknown too. */
static void alias_through_search(rift_sensor_camera_params *calib, rift_led *right_points)
{
    quatf offset = axis_turn(1, 1, 0, 120), inverse = offset;
    oquatf_inverse(&inverse);
    rift_led left_points[LEDS];
    for (int i = 0; i < LEDS; i++) {
        left_points[i] = right_points[i];
        oquatf_get_rotated(&offset, &right_points[i].pos, &left_points[i].pos);
        oquatf_get_rotated(&offset, &right_points[i].dir, &left_points[i].dir);
    }
    rift_leds right = {.points = right_points, .num_points = LEDS, .radius_mm = .004};
    led_search_model_t *model = led_search_model_new(&right);
    /* The left hand starts at the offset's inverse, so its LEDs face the camera like the right
     * model at rest. The right hand is held a quarter turn about x from it. */
    quatf right0 = axis_turn(1, 0, 0, 90), right0_inverse = right0;
    oquatf_inverse(&right0_inverse);
    for (int rigid = 0; rigid <= 1; rigid++) {
        rift_tracked_device dev = {.id = 1, .leds = &right};
        rift_tracked_device *devices[] = {&dev};
        rift_pose_finder pf = {0};
        rift_pose_finder_init(&pf, calib, receive, &pf);
        pf.cv1touch_allow_placement = false;
        pf.inspection_mask = 1u << dev.id;
        posef camera = {.orient = {{0, 0, 0, 1}}};
        rift_pose_finder_set_camera_pose(&pf, &camera);
        correspondence_search_set_model(pf.cs, dev.id, model);
        agreements = 0;
        int found = 0;
        for (int k = 0; k <= 7; k++) {
            /* Correlated turns: the left hand about y, the right about R0 y; or both rigidly
             * about x, which commutes with the hands' quarter-turn offset. */
            quatf world_left = rigid ? axis_turn(1, 0, 0, k * 10) : axis_turn(0, 1, 0, k * 10), world_right;
            if (rigid) {
                world_right = world_left;
            } else {
                quatf tmp;
                oquatf_mult(&right0, &world_left, &tmp);
                oquatf_mult(&tmp, &right0_inverse, &world_right);
            }
            quatf left, own;
            oquatf_mult(&world_left, &inverse, &left);
            oquatf_mult(&world_right, &right0, &own);
            posef left_pose = {.orient = left, .pos = {{.05f, -.1f, 1.6f}}};
            blobservation obs;
            project(left_points, calib, &left_pose, &obs);
            rift_sensor_analysis_frame frame;
            /* The right hand's IMU pose; out of view, its position is never searched from. */
            expose_frame(&frame, &obs, (posef){.orient = own, .pos = {{-.3f, -.1f, 1.6f}}},
                1000000000 + k * 20000000ULL);
            other_hand = &left;
            last_verdict = RIFT_IDENTITY_UNKNOWN;
            rift_pose_finder_process_blobs_fast(&pf, &frame, devices, 0x6);
            if (frame.need_long_analysis) {
                frame.long_analysis_start_ts = 1;
                frame.detached_observation = true;
                rift_pose_finder_process_blobs_long(&pf, &frame, devices, 0x6);
            }
            CHECK(pf.inspection_pose_mask == 0);
            CHECK(!frame.capture_state[0].cv1touch_pose_valid);
            for (int i = 0; i < obs.num_blobs; i++) CHECK(obs.blobs[i].led_id == LED_INVALID_ID);
            /* The right model was fitted to the left hand's LEDs: the alias is found. */
            if (POSE_HAS_FLAGS(&frame.capture_state[0].score, RIFT_POSE_MATCH_GOOD)) {
                found++;
                quatf error;
                oquatf_diff(&world_left, &frame.capture_state[0].final_cam_pose.orient, &error);
                CHECK(inspection_turn(error) < DEG_TO_RAD(1));
            }
            CHECK(last_verdict == RIFT_IDENTITY_UNKNOWN);
        }
        CHECK(found == 8);
        CHECK(agreements == 0);
        rift_pose_finder_clear(&pf);
    }
    other_hand = NULL;
    led_search_model_free(model);
}

/* An unmoved sensor whose hand is identified only after a thirty-degree turn: while the hand
 * is still, every exposure matches on the fast path through the calibrated pose, and none
 * may queue a cold search whose result could only be discarded as stale. */
static void unmoved_still_sensor(rift_sensor_camera_params *calib, led_search_model_t *model, rift_leds *leds)
{
    rift_tracked_device dev = {.id = 2, .leds = leds};
    rift_tracked_device *devices[] = {&dev};
    rift_pose_finder pf = {0};
    rift_pose_finder_init(&pf, calib, receive, &pf);
    pf.cv1touch_allow_placement = false;
    pf.inspection_mask = 1u << dev.id;
    posef camera = {.orient = {{0, 0, 0, 1}}};
    rift_pose_finder_set_camera_pose(&pf, &camera);
    correspondence_search_set_model(pf.cs, dev.id, model);
    int matched = 0;
    for (int tick = 0; tick < 100; tick++) {
        posef truth = {.orient = axis_turn(0, 1, 0, -20), .pos = {{.1f, -.15f, 2}}};
        blobservation obs;
        project(leds->points, calib, &truth, &obs);
        rift_sensor_analysis_frame frame;
        expose_frame(&frame, &obs, truth, 1000000000 + tick * 20000000ULL);
        rift_pose_finder_process_blobs_fast(&pf, &frame, devices, 0x6);
        matched += POSE_HAS_FLAGS(&frame.capture_state[0].score, RIFT_POSE_MATCH_GOOD);
        CHECK(!frame.need_long_analysis);
    }
    CHECK(matched == 100);
    /* The rigid replay's third inspection starts on an empty raw frame with an awake hand.
     * Empty observations advance continuity but must never reach the non-empty PnP helper. */
    blobservation empty = {0};
    ohmd_video_frame video = {.start_ts = 4000000000};
    rift_sensor_analysis_frame frame;
    expose_frame(&frame, &empty, (posef){.orient = {{0, 0, 0, 1}}}, 4000000000);
    frame.vframe = &video;
    uint64_t epoch = pf.inspection_epoch;
    rift_pose_finder_process_blobs_fast(&pf, &frame, devices, 0x6);
    CHECK(frame.inspection_epoch > epoch);
    CHECK(!frame.need_long_analysis && !frame.capture_state[0].found_device_pose);

    rift_pose_finder_clear(&pf);
}

int main(void)
{
    rift_sensor_camera_params calib = {.is_cv1 = true, .dist_fisheye = true, .width = 1280, .height = 960};
    calib.camera_matrix.m[0] = calib.camera_matrix.m[4] = 715;
    calib.camera_matrix.m[2] = 640; calib.camera_matrix.m[5] = 480; calib.camera_matrix.m[8] = 1;
    calib.dist_coeffs[0] = 0.01;
    rift_led points[LEDS];
    irregular_leds(points);
    rift_leds leds = {.points = points, .num_points = LEDS, .radius_mm = .004};
    led_search_model_t *model = led_search_model_new(&leds);
    alias_through_search(&calib, points);
    unmoved_still_sensor(&calib, model, &leds);
    static const quatf still = {{0, 0, 0, 1}};
    other_hand = &still;
    for (int device = 1; device <= 2; device++) {
    for (int angle = 5; angle <= 45; angle += 5) {
    rift_tracked_device dev = {.id = device, .leds = &leds};
    rift_tracked_device *devices[] = {&dev};
    rift_pose_finder pf = {0};
    rift_pose_finder_init(&pf, &calib, receive, &pf);
    pf.cv1touch_allow_placement = false;
    pf.inspection_mask = 1u << dev.id;
    posef camera = {.orient = {{0, 0, 0, 1}}, .pos = {{.4f, 0, 0}}};
    vec3f camera_axis = {{0, 0, 1}};
    oquatf_init_axis(&camera.orient, &camera_axis, DEG_TO_RAD(angle));
    rift_pose_finder_set_camera_pose(&pf, &camera);
    correspondence_search_set_model(pf.cs, dev.id, model);
    pf.cs->should_stop = counting_stop;
    pf.cs->stop_data = pf.cs;
    int cold = 0;
    deliveries = 0;
    for (int tick = 0; tick < 145 && deliveries < 100; tick++) {
        posef truth = {.pos = {{.1f + tick * .005f, -.15f, 2}}};
        vec3f axis = {{0, 1, 0}};
        oquatf_init_axis(&truth.orient, &axis, -.9f + tick * .013f);
        vec3f pixels[LEDS];
        rift_project_points(points, LEDS, &calib, &truth, pixels);
        blobservation obs = {.num_blobs = LEDS};
        for (int i = 0; i < LEDS; i++) {
            obs.blobs[i] = (struct blob){.blob_id = i + 1, .x = pixels[i].x, .y = pixels[i].y,
                .width = 3, .height = 3, .area = 9, .led_id = -1};
        }
        rift_sensor_analysis_frame frame = {.bwobs = &obs, .n_devices = 1};
        frame.exposure_info.devices[0].capture_pose = truth;
        frame.exposure_info.devices[0].device_time_ns = 1000000000 + tick * 20000000;
        frame.long_analysis_start_ts = 0;
        frame.exposure_info.devices[0].fusion_slot = 0;
        frame.exposure_info.devices[0].pos_error = (vec3f){{.05f, .05f, .05f}};
        frame.exposure_info.devices[0].rot_error = (vec3f){{.25f, .25f, .25f}};
        rift_pose_finder_process_blobs_fast(&pf, &frame, devices, 0x6);
        if (frame.need_long_analysis) {
            frame.long_analysis_start_ts = 1;
            frame.detached_observation = true;
            rift_pose_finder_process_blobs_long(&pf, &frame, devices, 0x6);
            cold++;
            if (frame.capture_state[0].cv1touch_pose_valid)
                rift_pose_finder_set_inspection_pose(&pf, dev.id, &truth, &frame.capture_state[0].final_cam_pose);
        }
        if (!frame.capture_state[0].cv1touch_pose_valid) continue;
        CHECK(frame.capture_state[0].found_device_pose);
        vec3f error;
        ovec3f_subtract(&truth.pos, &frame.capture_state[0].final_cam_pose.pos, &error);
        if (ovec3f_get_length(&error) >= .0001f)
            fprintf(stderr, "device %d calibration %d tick %d cold %d error %.8f m, match flags %x\n",
                device, angle, tick, cold, ovec3f_get_length(&error), frame.capture_state[0].score.match_flags);
        CHECK(ovec3f_get_length(&error) < .0001f);

    }
    CHECK(cold == 1 && deliveries >= 100);
    rift_pose_finder_clear(&pf);
    }
    }
    /* Selecting a trial's blobs polls once; the trial itself adds no poll of its own. */
    trials += trials_seen;
    fprintf(stderr, "cold-search stop polls %lu for %lu trials (%.2f per trial), %lu trials polled once\n",
        polls, trials, (double)polls / trials, back_to_back);
    CHECK(trials > 0 && back_to_back > 0);
    led_search_model_free(model);
    puts("test_inspection_tracking ok: camera rotations, native identity, mirror alias, 100 fast frames");
    return 0;
}
