// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* A real correspondence-search loop is parked while the engine restores its pose. The
 * restoration must cancel that loop, and cannot be acknowledged until the fast worker applies
 * it. No camera, USB or elapsed-time sleeps: the search callback and clock are controlled. */
#include "correspondence_search.c"
#include "omath.c"
#include "rift-sensor-pose-search.c"
#include "rift-sensor.c"
#include "test_util.h"
#include <pthread.h>
#include <time.h>

static void stop_device(rift_sensor_device *dev) { (void)dev; }

static int awake_device;

/* The real loop is parked before any pose evaluation (the anchors have no neighbours).
 * These paths must never be reached by this scheduling test. */
void rift_evaluate_pose_with_prior(rift_pose_metrics *score, posef *pose, bool prior_must_match, posef *prior,
    const vec3f *pos_error, const vec3f *rot_error, struct blob *blobs, int n, int device,
    rift_leds *leds, rift_sensor_camera_params *calib, rift_rect_t *bounds)
{
    (void)score; (void)pose; (void)prior_must_match; (void)prior; (void)pos_error; (void)rot_error;
    (void)blobs; (void)n; (void)device; (void)leds; (void)calib; (void)bounds;
    CHECK(device == awake_device);
    score->match_flags = RIFT_POSE_MATCH_GOOD | RIFT_POSE_MATCH_STRONG | RIFT_POSE_MATCH_LED_IDS;
    score->matched_blobs = score->visible_leds = 6;
}
void rift_evaluate_pose(rift_pose_metrics *score, posef *pose, struct blob *blobs, int n, int device,
    rift_leds *leds, rift_sensor_camera_params *calib, rift_rect_t *bounds)
{
    (void)score; (void)pose; (void)blobs; (void)n; (void)device; (void)leds; (void)calib; (void)bounds;
    abort();
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

static int fusion_updates, label_updates, starts;
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
void blobwatch_clear_labels(blobwatch *bw, int dev) { (void)bw; (void)dev; }
void blobwatch_update_labels(blobwatch *bw, blobservation *ob, int dev)
{
    (void)bw; (void)ob; (void)dev;
    label_updates++;
}
static void search_started(void *data, const ohmd_cv1touch_frame_info *info, uint64_t remaining)
{
    (void)data;
    CHECK(strcmp(info->serial, "CameraA") == 0);
    CHECK(info->exposure_hmd_ts == 1234);
    CHECK(remaining > 0);
    starts++;
}

bool rift_tracked_device_get_latest_exposure_info_pose(rift_tracked_device *dev,
    rift_tracked_device_exposure_info *info)
{
    CHECK(dev->id == awake_device);
    info->fusion_slot = 0;
    info->capture_pose.orient.w = 1;
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
    CHECK(dev == awake_device && n == 6);
    for (int i = 0; i < n; i++) blobs[i].led_id = LED_MAKE_ID(dev, i);
}

static int released;
void rift_tracker_frame_release(rift_tracker_ctx *ctx, uint64_t now, uint64_t exposure,
    rift_tracker_exposure_info *info, const char *source)
{
    (void)ctx; (void)now; (void)exposure; (void)info; (void)source;
}
void blobwatch_release_observation(blobwatch *bw, blobservation *ob)
{
    (void)bw; (void)ob;
}
void ohmd_video_frame_release(ohmd_video_frame *frame)
{
    CHECK(frame != NULL);
    released++;
}

static pthread_mutex_t gate_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t gate_cond = PTHREAD_COND_INITIALIZER;
static bool entered, resume_search, finished;
static int polls, wakeups;
void ohmd_cond_signal(ohmd_cond *cond)
{
    (void)cond;
    wakeups++;
}
static rift_sensor_ctx sensor;
static correspondence_search_t search;

static bool blocked_stop(void *data)
{
	pthread_mutex_lock(&gate_lock);
	entered = true;
	pthread_cond_broadcast(&gate_cond);
	while (!resume_search)
		pthread_cond_wait(&gate_cond, &gate_lock);
	pthread_mutex_unlock(&gate_lock);
	polls++;
	return sensor_search_should_stop(data);
}

static void *run_search(void *data)
{
	(void)data;
	cs_model_info_t model = {0};
	check_led_match(&search, &model, NULL, 0);
	pthread_mutex_lock(&gate_lock);
	finished = true;
	pthread_cond_broadcast(&gate_cond);
	pthread_mutex_unlock(&gate_lock);
	ohmd_lock_mutex(sensor.sensor_lock);
	long_analysis_finished(&sensor);
	ohmd_unlock_mutex(sensor.sensor_lock);
	return NULL;
}

static void wait_until_entered(void)
{
	struct timespec until;
	clock_gettime(CLOCK_REALTIME, &until);
	until.tv_sec += 2;
	pthread_mutex_lock(&gate_lock);
	while (!entered && !finished) {
		CHECK(pthread_cond_timedwait(&gate_cond, &gate_lock, &until) == 0);
	}
	CHECK(entered);
	pthread_mutex_unlock(&gate_lock);
}

int main(void)
{
	ohmd_context ctx = {.monotonic_ticks_per_sec = 1000000000};
	cs_image_point_t points[3] = {0};
	rift_sensor_device_funcs funcs = {.request_stop = stop_device};
	rift_sensor_device device = {.funcs = &funcs};
	sensor.dev = &device;
	sensor.ohmd_ctx = &ctx;
	sensor.sensor_lock = ohmd_create_mutex(&ctx);
	sensor.fast_analysis_q_cond = (ohmd_cond *)&gate_cond;
	CHECK(sensor.sensor_lock);
	strcpy(sensor.serial_no, "CameraA");
	sensor.pf.cs = &search;
	sensor.pf.have_camera_pose = true;
	search.num_points = 3;
	search.points = points;
	search.should_stop = blocked_stop;
	search.stop_data = &sensor;
	ohmd_cv1touch_sensor_pose pose = {.serial = "CameraA", .orient = {0, 0, 0, 1}};

    /* Restoration races with a queued search which the long worker has not taken yet. The
     * actual queue must be drained, not just a manually marked busy search cancelled. */
    ohmd_video_frame video = {0};
    rift_sensor_analysis_frame queued = {.vframe = &video};
    ctx.cv1touch_search_cb = search_started;
    queued.exposure_info.hmd_ts = 1234;
    sensor.pf.inspection_mask = 0x2;
    sensor.search_deadline = now_ticks + 30000000000;
    blobservation queued_observation = {0};
    queued.bwobs = &queued_observation;
    queue_long_frame(&sensor, &queued);
    CHECK(starts == 1);
    CHECK(rift_sensor_set_cv1touch_pose(&sensor, &pose));
    CHECK(rift_sensor_cv1touch_pose_applied(&sensor, "CameraA") == 0);
    CHECK(next_fast_frame(&sensor) == NULL);
    CHECK(released == 1);
    CHECK(POP_QUEUE(&sensor.long_analysis_q) == NULL);
    CHECK(POP_QUEUE(&sensor.capture_frame_q) == &queued);
    CHECK(rift_sensor_cv1touch_pose_applied(&sensor, "CameraA") == 1);

    /* Unknown first sightings stay out of blobwatch. Runtime label propagation and
     * interleaved fast/long workers are exercised with real blobwatch in inspection_handoff. */
    blobservation labelled = {.num_blobs = 6};
    rift_sensor_analysis_frame found = {.vframe = &video, .bwobs = &labelled, .n_devices = 1};
    posef found_pose = {.orient = {{0, 0, 0, 1}}};
    rift_pose_metrics score = {0};
    for (int hand = 1; hand <= 2; hand++) {
        sensor.pf.inspection_mask = 1u << hand;
        rift_tracked_device dev = {.id = hand};
        sensor.devices[0] = &dev;
        found.exposure_info.devices[0].capture_pose.orient.w = 1;
        found.exposure_info.devices[0].device_time_ns = 1000000000;
        found.capture_state[0].final_cam_pose.orient.w = 1;
        CHECK(!handle_found_pose(&sensor, &dev, &found, &found_pose, &score));
        CHECK(fusion_updates == hand - 1 && label_updates == 0);
        dev.id = 3 - hand;
        CHECK(!handle_found_pose(&sensor, &dev, &found, &found_pose, &score));
        CHECK(fusion_updates == hand && label_updates == 0);
    }
    sensor.devices[0] = NULL;
    sensor.pf.inspection_mask = 0;


	for (int mode = 0; mode < 4; mode++) {
        int requested = mode % 2 + 1;
		CHECK(rift_sensor_clear_cv1touch_pose(&sensor, "CameraA", 30000000000, 1u << requested));
		apply_pending_pose(&sensor);
		CHECK(sensor.pf.have_camera_pose); /* Inspection keeps the other hand's fast path. */
		entered = resume_search = finished = false;
		polls = wakeups = 0;
		sensor.search_polls = 0;
		sensor.long_analysis_busy = true;
		pthread_t worker;
		CHECK(pthread_create(&worker, NULL, run_search, NULL) == 0);
        wait_until_entered();
        /* The other controller wakes while the real cold-search loop is still blocked. Its
         * calibrated fast path must acquire now, in both hand orders. */
        awake_device = 3 - requested;
        rift_led leds[6] = {0};
        rift_leds model = {.points = leds, .num_points = 6};
        rift_tracked_device other = {.id = awake_device, .leds = &model};
        rift_tracked_device *devices[] = {&other};
        blobservation observation = {.num_blobs = 6};
        for (int i = 0; i < 6; i++) observation.blobs[i].led_id = -1;
        rift_sensor_analysis_frame waking = {.vframe = &video, .bwobs = &observation, .n_devices = 1};
        sensor.devices[0] = &other;
        sensor.n_devices = 1;
        sensor.pf.pose_cb = (rift_pose_finder_cb)handle_found_pose;
        sensor.pf.pose_cb_data = &sensor;
        rift_pose_finder_process_blobs_fast(&sensor.pf, &waking, devices, 0x6);
        CHECK(waking.capture_state[0].found_device_pose);
        CHECK(!waking.need_long_analysis && sensor.long_analysis_busy);
        CHECK(!finished);
        awake_device = 0;
		if (mode == 1) {
			__atomic_fetch_add(&now_ticks, 30000000000, __ATOMIC_RELAXED);
		} else if (mode == 2) {
			/* The sensor is unplugged: retiring it stops its search like a restoration. */
			rift_sensor_retire(&sensor);
		} else if (mode == 3) {
			rift_sensor_request_stop(&sensor);
		} else {
			CHECK(rift_sensor_set_cv1touch_pose(&sensor, &pose));
			CHECK(wakeups == 1);
			CHECK(rift_sensor_cv1touch_pose_applied(&sensor, "CameraA") == 0);
			apply_pending_pose(&sensor);
			CHECK(rift_sensor_cv1touch_pose_applied(&sensor, "CameraA") == 0);
		}
		pthread_mutex_lock(&gate_lock);
		resume_search = true;
		pthread_cond_broadcast(&gate_cond);
		pthread_mutex_unlock(&gate_lock);
		CHECK(pthread_join(worker, NULL) == 0);
		CHECK(polls == 1); /* The loop did not advance to a second anchor after stopping. */
		if (mode == 3) {
			CHECK(sensor.search_stopping);
			break;
		}
		if (mode == 0) CHECK(wakeups == 2);
		if (mode != 0) {
			CHECK(mode == 2 || search_deadline_expired(&sensor));
			/* A retired sensor still applies and acknowledges a pose restored before the
			 * daemon forgot it: the engine thread does not wait for it forever. */
			CHECK(rift_sensor_set_cv1touch_pose(&sensor, &pose));
		}
		CHECK(rift_sensor_cv1touch_pose_applied(&sensor, "CameraA") == 0);
		CHECK(next_fast_frame(&sensor) == NULL); /* A restored pose needs no camera frame. */
		CHECK(rift_sensor_cv1touch_pose_applied(&sensor, "CameraA") == 1);
		if (mode == 2) {
			/* It starts no new search, though: a frame that needs one goes back to capture.
			 * A restored pose has no deadline to end such a search. */
			blobservation late_observation = {0};
			rift_sensor_analysis_frame late = {.vframe = &video, .bwobs = &late_observation,
				.need_long_analysis = true};
			int freed = released;
			finish_fast_frame(&sensor, &late);
			CHECK(released == freed + 1);
			CHECK(POP_QUEUE(&sensor.long_analysis_q) == NULL);
			CHECK(POP_QUEUE(&sensor.capture_frame_q) == &late);
			CHECK(__atomic_load_n(&sensor.search_cancelled, __ATOMIC_RELAXED));
		}
		sensor.retired = false; /* the next mode inspects a sensor that is still plugged in */
        CHECK(sensor.pf.inspection_pose_mask == 0);
		CHECK(!search_deadline_expired(&sensor));
	}
	CHECK(rift_sensor_cv1touch_pose_applied(&sensor, "CameraB") == -1);
	ohmd_destroy_mutex(sensor.sensor_lock);
	puts("test_search_cancel ok");
	return 0;
}
