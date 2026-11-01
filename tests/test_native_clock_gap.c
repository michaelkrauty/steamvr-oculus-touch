// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* Real tracker owner and UKF, no hardware. Unity seams count actual numerical
 * return values and allocations; only logging, sensor registration and locks
 * are supplied by the fixture. Old-camera callbacks run on a real second thread. */
#define _GNU_SOURCE
#define OHMD_NONBLOCK_LOG
#include <pthread.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include "test_util.h"
#include "ukf.h"

static atomic_uint allocations;
static size_t live_matrices;
static void *counted_malloc(size_t n) { allocations++; return malloc(n); }
static void *counted_calloc(size_t n, size_t size) { allocations++; return calloc(n, size); }
static void *counted_realloc(void *p, size_t n) { allocations++; return realloc(p, n); }
static void *matrix_malloc(size_t n)
{
	void *p = counted_malloc(n);
	CHECK(p);
	live_matrices++;
	return p;
}
static void matrix_free(void *p)
{
	if (p) {
		CHECK(live_matrices > 0);
		live_matrices--;
	}
	free(p);
}
#define malloc matrix_malloc
#define free matrix_free
#include "matrices.c"
#undef free
#undef malloc
#include "omath.c"
#include "exponential-filter.c"
#define print_mat ukf_print_mat
#define ukf_base_predict native_predict
#define ukf_base_update native_update
#include "ukf.c"
#undef ukf_base_predict
#undef ukf_base_update
#undef print_mat
#define print_mat unscented_print_mat
#include "unscented.c"
#undef print_mat

static unsigned predictions, updates, prediction_failures, update_failures;
static double last_prediction_dt;
bool ukf_base_predict(ukf_base *u, double dt)
{
	predictions++;
	last_prediction_dt = dt;
	bool ok = native_predict(u, dt);
	prediction_failures += !ok;
	return ok;
}
bool ukf_base_update(ukf_base *u, ukf_measurement *m)
{
	updates++;
	bool ok = native_update(u, m);
	update_failures += !ok;
	return ok;
}
#define malloc counted_malloc
#define calloc counted_calloc
#define realloc counted_realloc
#include "rift-kalman-6dof.c"
#include "rift-tracker.c"
#undef malloc
#undef calloc
#undef realloc
#include "rift-cv1touch-touch.h"

int ohmd_log_printf(const char *format, ...) { (void)format; return 0; }
int ohmd_log_fprintf(FILE *stream, const char *format, ...) { (void)stream; (void)format; return 0; }
int ohmd_log_puts(const char *text) { (void)text; return 0; }
void ohmd_assert_fail(const char *expr, const char *file, int line, const char *fn)
{
	fprintf(stderr, "%s:%d %s: assertion %s\n", file, line, fn, expr);
	abort();
}

/* A recursive acquisition or tracker_lock acquisition while holding device_lock
 * fails rather than hiding an owner lock-order regression. */
static _Thread_local int locks_held;
ohmd_mutex *ohmd_create_mutex(ohmd_context *ctx)
{
	(void)ctx;
	pthread_mutex_t *m = malloc(sizeof *m);
	CHECK(m && pthread_mutex_init(m, NULL) == 0);
	return (ohmd_mutex *)m;
}
void ohmd_destroy_mutex(ohmd_mutex *m) { CHECK(pthread_mutex_destroy((pthread_mutex_t *)m) == 0); free(m); }
void ohmd_lock_mutex(ohmd_mutex *m) { CHECK(locks_held == 0); CHECK(pthread_mutex_lock((pthread_mutex_t *)m) == 0); locks_held++; }
void ohmd_unlock_mutex(ohmd_mutex *m) { CHECK(locks_held == 1); locks_held--; CHECK(pthread_mutex_unlock((pthread_mutex_t *)m) == 0); }
uint64_t ohmd_monotonic_get(ohmd_context *ctx) { (void)ctx; return 10000000000ull; }
led_search_model_t *led_search_model_new(rift_leds *leds) { (void)leds; return NULL; }
bool rift_sensor_add_device(rift_sensor_ctx *ctx, rift_tracked_device *dev) { (void)ctx; (void)dev; return true; }

static const vec3f accel = {{0, 9.80665f, 0}}, mag = {{0}};
static const posef identity = {.orient = {.w = 1}};
struct fixture {
	rift_tracker_ctx ctx;
	ohmd_context ohmd;
	rift_touch_controller_t touch;
	rift_tracked_device_imu_calibration calibration;
	rift_leds leds;
	rift_sensor_ctx *sensor;
	rift_tracked_device_priv *dev;
	uint32_t message_ts;
};

static struct fixture *create_fixture(int slots, uint32_t first_message)
{
	struct fixture *f = calloc(1, sizeof *f);
	CHECK(f);
	/* Canaries: no reset should touch factory calibration or persisted configuration. */
	memset(&f->calibration, 0x35, sizeof f->calibration);
	memset(&f->ohmd.cv1touch_config, 0x53, sizeof f->ohmd.cv1touch_config);
	memset(&f->ohmd.cv1touch_sensors, 0x27, sizeof f->ohmd.cv1touch_sensors);
	f->ctx.ohmd_ctx = &f->ohmd;
	f->ctx.tracker_lock = ohmd_create_mutex(NULL);
	f->ctx.devices[0].device_lock = ohmd_create_mutex(NULL);
	f->ctx.n_sensors = slots ? 1 : 0;
	f->ctx.sensors = &f->sensor;
	posef imu_pose = {.pos = {{.01f, .02f, .03f}}, .orient = {.w = 1}};
	posef model_pose = {.pos = {{.04f, .05f, .06f}}, .orient = {.w = 1}};
	f->dev = (rift_tracked_device_priv *)rift_tracker_add_device(&f->ctx, 2,
		&imu_pose, &model_pose, &f->leds, &f->calibration);
	CHECK(f->dev->n_delay_slots == slots && f->dev->index == 0);
	f->touch.have_calibration = true;
	for (int i = 0; i < 3; i++) {
		f->touch.calibration.accel_matrix[i][i] = 1;
		f->touch.calibration.gyro_matrix[i][i] = 1;
	}
	f->message_ts = first_message;
	return f;
}

static void destroy_fixture(struct fixture *f)
{
	rift_kalman_6dof_clear(&f->dev->ukf_fusion);
	ohmd_destroy_mutex(f->dev->device_lock);
	ohmd_destroy_mutex(f->ctx.tracker_lock);
	free(f);
}

static void equal_matrix(const matrix2d *a, const matrix2d *b)
{
	CHECK(a->rows == b->rows && a->cols == b->cols);
	for (unsigned r = 0; r < a->rows; r++)
		for (unsigned c = 0; c < a->cols; c++)
			CHECK(MATRIX2D_XY(a, r, c) == MATRIX2D_XY(b, r, c));
}

static void healthy(rift_tracked_device_priv *d)
{
	matrix2d *root = matrix2d_alloc(d->ukf_fusion.ukf.N_cov, d->ukf_fusion.ukf.N_cov);
	CHECK(matrix2d_cholesky(root, d->ukf_fusion.ukf.P_prior) == MATRIX_RESULT_OK);
	matrix2d_free(root);
	rift_tracked_device_export out;
	rift_tracked_device_get_export(&d->base, &out);
	for (int i = 0; i < 3; i++) {
		CHECK(isfinite(out.pos_error.arr[i]) && isfinite(out.rot_error.arr[i]));
		CHECK(isfinite(out.raw_pose.pos.arr[i]));
	}
	CHECK(isfinite(out.raw_pose.orient.w));
}

/* Exercise the real non-empty decoded path and the owner independently. The
 * sensor calibration is irrelevant to the clock; feed known calibrated IMUs to UKF. */
static cv1touch_sample_time report(struct fixture *f, uint32_t delta_us, float gyro_y)
{
	f->message_ts += delta_us;
	pkt_rift_radio_message m = {.valid = true, .device_type = RIFT_TOUCH_CONTROLLER_RIGHT};
	m.touch.timestamp = f->message_ts;
	m.touch.accel[1] = 2048;
	rift_touch_controller_t timing = f->touch;
	cv1touch_sample_time t = touch_sample_time(&timing, &m);
	ohmd_cv1touch_controller out;
	CHECK(touch_decode_sample(NULL, &f->touch, &m, 10000000000ull + (uint64_t)f->message_ts * 1000, &out));
	CHECK(out.device_time_ns == t.device_ns);
	vec3f gyro = {{0, gyro_y, 0}};
	unsigned before = allocations;
	rift_tracked_device_imu_update(&f->dev->base, out.received_host_ns,
		t.device_ts, t.dt * 1e-6f, &gyro, &accel, &mag);
	/* The existing UKF measurement has one temporary matrix allocation. A reset
	 * must add none; changing that pre-existing hot-path allocation is out of scope. */
	CHECK(allocations <= before + 1);
	if (!prediction_failures && !update_failures) CHECK(allocations == before + 1);
	CHECK(f->dev->device_time_ns == t.device_ns && f->dev->last_device_ts == t.device_ts);
	CHECK(f->dev->ukf_fusion.current_ts == t.device_ns);
	return t;
}

static rift_tracker_exposure_info exposure(rift_tracked_device_priv *d)
{
	rift_tracker_exposure_info e = {.n_devices = 1};
	ohmd_lock_mutex(d->device_lock);
	rift_tracked_device_on_new_exposure(d, &e.devices[0]);
	if (d->n_delay_slots) CHECK(rift_tracked_device_exposure_claim(d, &e.devices[0]) >= 0);
	ohmd_unlock_mutex(d->device_lock);
	return e;
}

static void optical(rift_tracked_device_priv *d)
{
	rift_tracker_exposure_info e = exposure(d);
	posef imu_pose, pose;
	rift_kalman_6dof_get_pose_at(&d->ukf_fusion, d->device_time_ns, &imu_pose, NULL, NULL, NULL, NULL, NULL);
	oposef_apply(&d->model_from_fusion, &imu_pose, &pose); /* Current native pose, not the held position. */
	rift_pose_metrics score = {.match_flags = RIFT_POSE_MATCH_POSITION | RIFT_POSE_MATCH_ORIENT};
	CHECK(rift_tracked_device_model_pose_update(&d->base, 0, 0, &e, &score, &pose, "test"));
	CHECK(d->last_observed_pose_ts == d->device_time_ns);
	CHECK(d->last_observed_orient_ts == d->device_time_ns);
	CHECK(d->ukf_fusion.saw_pose_update);
	rift_tracked_device_frame_release(&d->base, &e);
}

/* Compare the owner reset with a separately allocated fresh filter, not another
 * reset call. Different configured noise must survive, while learned biases and
 * all delayed state/cross-covariance must be discarded. */
static void reset_matches_fresh(struct fixture *f)
{
	rift_tracked_device_priv *d = f->dev;
	rift_kalman_6dof_filter *u = &d->ukf_fusion, fresh = {0};
	posef held;
	rift_kalman_6dof_get_pose_at(u, d->device_time_ns, &held, NULL, NULL, NULL, NULL, NULL);
	rift_kalman_6dof_init(&fresh, &held, d->n_delay_slots);
	MATRIX2D_XY(u->Q_noise, 0, 0) *= 1.1;
	MATRIX2D_XY(u->m1.R, 0, 0) *= 1.1;
	MATRIX2D_XY(u->m2.R, 0, 0) *= 1.1;
	MATRIX2D_XY(u->m_position.R, 0, 0) *= 1.1;
	matrix2d_copy(fresh.Q_noise, u->Q_noise);
	matrix2d_copy(fresh.m1.R, u->m1.R);
	matrix2d_copy(fresh.m2.R, u->m2.R);
	matrix2d_copy(fresh.m_position.R, u->m_position.R);
	ukf_base allocation_identity = u->ukf;
	ukf_measurement imu_allocation = u->m1, pose_allocation = u->m2, position_allocation = u->m_position;
	posef reported = d->reported_pose, model = d->model_pose;
	posef device_from_fusion = d->device_from_fusion, model_from_fusion = d->model_from_fusion;
	posef fusion_from_model = d->fusion_from_model;
	ohmd_cv1touch_config config = f->ohmd.cv1touch_config;
	cv1touch_sensor_config sensors = f->ohmd.cv1touch_sensors;
	rift_tracked_device_imu_calibration calibration = f->calibration;
	rift_touch_calibration touch_calibration = f->touch.calibration;
	d->pose_output_filter.pos_filter.fc_min = 17;
	d->pose_output_filter.orient_filter.beta = .7;
	d->pose_output_filter.pos_filter.have_prev_y = true;
	d->pose_output_filter.orient_filter.have_prev_y = true;
	d->last_reported_pose = d->device_time_ns;
	d->reported_lin_vel = d->reported_lin_accel = d->reported_ang_vel = (vec3f){{1, 2, 3}};
	d->last_no_free_delay_slot = d->device_time_ns;
	uint64_t previous = d->device_time_ns;
	uint32_t sample_gap_us = f->message_ts - d->last_device_ts;
	report(f, 2000, .05f);
	CHECK(d->device_time_ns - previous == (uint64_t)sample_gap_us * 1000);
	CHECK(last_prediction_dt == 0); /* Fresh epoch, not propagation across missing IMUs. */
	vec3f gyro = {{0, .05f, 0}};
	rift_kalman_6dof_imu_update(&fresh, d->device_time_ns, &gyro, &accel, &mag);
	equal_matrix(u->ukf.x_prior, fresh.ukf.x_prior);
	equal_matrix(u->ukf.P_prior, fresh.ukf.P_prior);
	CHECK(!u->saw_pose_update && u->reset_slot == -1 && u->pose_slot == -1);
	CHECK(!d->last_observed_pose_ts && !d->last_observed_orient_ts && !d->last_acquired_pose_lock_ts);
	CHECK(!d->last_reported_pose && !d->last_no_free_delay_slot);
	CHECK(!d->pose_output_filter.pos_filter.have_prev_y && !d->pose_output_filter.orient_filter.have_prev_y);
	CHECK(d->pose_output_filter.pos_filter.fc_min == 17 && d->pose_output_filter.orient_filter.beta == .7);
	for (int i = 0; i < 3; i++) CHECK(d->reported_lin_vel.arr[i] == 0 && d->reported_lin_accel.arr[i] == 0 && d->reported_ang_vel.arr[i] == 0);
	CHECK(memcmp(&reported, &d->reported_pose, sizeof reported) == 0);
	CHECK(memcmp(&model, &d->model_pose, sizeof model) == 0);
	CHECK(memcmp(&device_from_fusion, &d->device_from_fusion, sizeof device_from_fusion) == 0);
	CHECK(memcmp(&model_from_fusion, &d->model_from_fusion, sizeof model_from_fusion) == 0);
	CHECK(memcmp(&fusion_from_model, &d->fusion_from_model, sizeof fusion_from_model) == 0);
	CHECK(memcmp(&config, &f->ohmd.cv1touch_config, sizeof config) == 0);
	CHECK(memcmp(&sensors, &f->ohmd.cv1touch_sensors, sizeof sensors) == 0);
	CHECK(memcmp(&calibration, &f->calibration, sizeof calibration) == 0);
	CHECK(memcmp(&touch_calibration, &f->touch.calibration, sizeof touch_calibration) == 0);
	CHECK(d->base.leds == &f->leds);
	CHECK(memcmp(&allocation_identity, &u->ukf, sizeof allocation_identity) == 0);
	CHECK(memcmp(&imu_allocation, &u->m1, sizeof imu_allocation) == 0);
	CHECK(memcmp(&pose_allocation, &u->m2, sizeof pose_allocation) == 0);
	CHECK(memcmp(&position_allocation, &u->m_position, sizeof position_allocation) == 0);
	equal_matrix(u->Q_noise, fresh.Q_noise);
	equal_matrix(u->m1.R, fresh.m1.R);
	equal_matrix(u->m2.R, fresh.m2.R);
	equal_matrix(u->m_position.R, fresh.m_position.R);
	for (int i = 0; i < d->n_delay_slots; i++) {
		CHECK(!d->delay_slots[i].valid && !d->delay_slots[i].use_count && !d->delay_slots[i].had_used_report);
		CHECK(!u->slot_inuse[i]);
	}
	/* More updates exercise recycled scratch matrices against fresh scratch. */
	for (int i = 0; i < 5; i++) {
		report(f, 2000, .05f);
		rift_kalman_6dof_imu_update(&fresh, d->device_time_ns, &gyro, &accel, &mag);
		equal_matrix(u->ukf.x_prior, fresh.ukf.x_prior);
		equal_matrix(u->ukf.P_prior, fresh.ukf.P_prior);
	}
	rift_kalman_6dof_clear(&fresh);
}

static void gap_sequence(int slots, uint32_t gap, uint32_t start)
{
	struct fixture *f = create_fixture(slots, start);
	rift_tracked_device_priv *d = f->dev;
	for (int i = 0; i < (slots ? 2500 : 100); i++) report(f, i ? 2000 : 0, .05f);
	if (slots) optical(d);
	healthy(d);
	uint64_t previous = d->device_time_ns;
	/* This receive gap is deliberately NOT yet a sample gap. */
	report(f, gap, .05f);
	CHECK(d->device_time_ns == previous + 2000000);
	CHECK_NEAR(last_prediction_dt, .002, 1e-12);
	reset_matches_fresh(f); /* The next received report exposes the sample gap. */
	for (int i = 0; i < 5000; i++) {
		double before = MATRIX2D_Y(d->ukf_fusion.ukf.x_prior, STATE_ORIENTATION + 1);
		report(f, 2000, (i & 1) ? .07f : .09f);
		CHECK(MATRIX2D_Y(d->ukf_fusion.ukf.x_prior, STATE_ORIENTATION + 1) != before);
		if (slots && i % 25 == 0) optical(d);
		if (i % 100 == 0) healthy(d);
	}
	healthy(d);
	CHECK(!prediction_failures && !update_failures);
	/* A second gap must reset using the same storage and clear reacquired optical evidence. */
	report(f, gap, .05f);
	reset_matches_fresh(f);
	printf("native gap: slots=%d gap_us=%u start_us=%u predictions=%u updates=%u ok\n", slots, gap, start, predictions, updates);
	destroy_fixture(f);
}

static void short_gaps_and_early_epoch(void)
{
	const uint32_t gaps[] = {80000, 99999, 100000, 100001};
	for (unsigned j = 0; j < sizeof gaps / sizeof gaps[0]; j++) {
		/* Small initial epoch catches zero-sentinel freshness errors below 500 ms. */
		struct fixture *f = create_fixture(3, 10000);
		rift_tracked_device_priv *d = f->dev;
		report(f, 0, .05f);
		rift_tracker_exposure_info untracked = exposure(d);
		CHECK(!untracked.devices[0].had_pose_lock);
		rift_tracked_device_frame_release(&d->base, &untracked);
		/* Even a nonzero predicted position is held without a first optical fix. */
		MATRIX2D_Y(d->ukf_fusion.ukf.x_prior, STATE_POSITION) = 3;
		rift_tracked_device_get_view_pose(&d->base, NULL, NULL, NULL, NULL);
		posef model;
		ohmd_lock_mutex(d->device_lock);
		rift_tracked_device_get_model_pose_locked(d, 0, &model, NULL, NULL);
		ohmd_unlock_mutex(d->device_lock);
		CHECK(d->reported_pose.pos.x == 0 && d->model_pose.pos.x == 0);
		optical(d);
		uint64_t observed = d->last_observed_pose_ts;
		report(f, gaps[j], .05f);
		CHECK(d->last_observed_pose_ts == observed);
		/* For <=100 ms compare directly with the unchanged UKF continuity path. */
		rift_kalman_6dof_filter control = {0};
		posef initial = {.orient = {.y = 1}};
		rift_kalman_6dof_init(&control, &initial, 3);
		matrix2d_copy(control.ukf.x_prior, d->ukf_fusion.ukf.x_prior);
		matrix2d_copy(control.ukf.P_prior, d->ukf_fusion.ukf.P_prior);
		control.first_update = d->ukf_fusion.first_update;
		control.current_ts = d->ukf_fusion.current_ts;
		control.saw_pose_update = d->ukf_fusion.saw_pose_update;
		memcpy(control.slot_inuse, d->ukf_fusion.slot_inuse, sizeof control.slot_inuse);
		report(f, 2000, .05f);
		if (gaps[j] <= 100000) {
			vec3f gyro = {{0, .05f, 0}};
			rift_kalman_6dof_imu_update(&control, d->device_time_ns, &gyro, &accel, &mag);
			equal_matrix(control.ukf.x_prior, d->ukf_fusion.ukf.x_prior);
			equal_matrix(control.ukf.P_prior, d->ukf_fusion.ukf.P_prior);
			CHECK(d->last_observed_pose_ts == observed && d->ukf_fusion.saw_pose_update);
			CHECK_NEAR(last_prediction_dt, gaps[j] * 1e-6, 1e-12);
		} else {
			CHECK(d->last_observed_pose_ts == 0 && !d->ukf_fusion.saw_pose_update);
			CHECK(last_prediction_dt == 0 && d->device_time_ns < 500000000);
			rift_tracker_exposure_info e = exposure(d);
			CHECK(!e.devices[0].had_pose_lock);
			rift_tracked_device_frame_release(&d->base, &e);
			/* Position remains held despite the reset's nonzero held raw pose. */
			posef held = d->reported_pose, held_model = d->model_pose;
			MATRIX2D_Y(d->ukf_fusion.ukf.x_prior, STATE_POSITION) += 5;
			rift_tracked_device_get_view_pose(&d->base, NULL, NULL, NULL, NULL);
			posef model;
			ohmd_lock_mutex(d->device_lock);
			rift_tracked_device_get_model_pose_locked(d, 0, &model, NULL, NULL);
			ohmd_unlock_mutex(d->device_lock);
			CHECK(memcmp(&held.pos, &d->reported_pose.pos, sizeof held.pos) == 0);
			CHECK(memcmp(&held_model.pos, &d->model_pose.pos, sizeof held_model.pos) == 0);
			optical(d);
			e = exposure(d);
			CHECK(e.devices[0].had_pose_lock);
			rift_tracked_device_frame_release(&d->base, &e);
		}
		rift_kalman_6dof_clear(&control);
		destroy_fixture(f);
	}
}

struct late_callback {
	rift_tracked_device_priv *d;
	rift_tracker_exposure_info old;
	pthread_barrier_t ready, resume;
};

static void barrier(pthread_barrier_t *b)
{
	int r = pthread_barrier_wait(b);
	CHECK(r == 0 || r == PTHREAD_BARRIER_SERIAL_THREAD);
}

static void *late_camera(void *arg)
{
	struct late_callback *c = arg;
	/* A camera obtains its prior, then spends time searching while IMUs resume. */
	CHECK(rift_tracked_device_get_latest_exposure_info_pose(&c->d->base, &c->old.devices[0]));
	barrier(&c->ready);
	barrier(&c->resume);
	rift_pose_metrics score = {.match_flags = RIFT_POSE_MATCH_POSITION | RIFT_POSE_MATCH_ORIENT};
	posef pose = c->old.devices[0].capture_pose;
	pose.pos.x += 10;
	CHECK(!rift_tracked_device_model_pose_update(&c->d->base, 0, 0, &c->old, &score, &pose, "stale"));
	rift_tracked_device_frame_release(&c->d->base, &c->old);
	ohmd_lock_mutex(c->d->device_lock);
	CHECK(rift_tracked_device_exposure_claim(c->d, &c->old.devices[0]) == -1);
	ohmd_unlock_mutex(c->d->device_lock);
	CHECK(!rift_tracked_device_get_latest_exposure_info_pose(&c->d->base, &c->old.devices[0]));
	return NULL;
}

static void stale_camera_cannot_touch_reused_slots(void)
{
	struct fixture *f = create_fixture(3, 1000000);
	rift_tracked_device_priv *d = f->dev;
	report(f, 0, .05f);
	optical(d);
	rift_tracker_exposure_info old[3];
	for (int i = 0; i < 3; i++) {
		report(f, 2000, .05f);
		old[i] = exposure(d);
		CHECK(d->delay_slots[old[i].devices[0].fusion_slot].use_count == 1);
	}
	struct late_callback c = {.d = d, .old = old[0]};
	CHECK(pthread_barrier_init(&c.ready, NULL, 2) == 0);
	CHECK(pthread_barrier_init(&c.resume, NULL, 2) == 0);
	pthread_t worker;
	CHECK(pthread_create(&worker, NULL, late_camera, &c) == 0);
	barrier(&c.ready);
	report(f, 5000000, .05f);
	report(f, 2000, .05f);
	/* Reject stale callbacks even before any new slot has been populated. */
	for (int i = 0; i < 3; i++) {
		rift_tracked_device_exposure_info copy = old[i].devices[0];
		CHECK(!rift_tracked_device_get_latest_exposure_info_pose(&d->base, &copy));
		CHECK(copy.fusion_slot == -1);
		rift_tracker_exposure_info stale = old[i];
		rift_pose_metrics score = {.match_flags = RIFT_POSE_MATCH_POSITION | RIFT_POSE_MATCH_ORIENT};
		posef pose = identity;
		CHECK(!rift_tracked_device_model_pose_update(&d->base, 0, 0, &stale, &score, &pose, "stale"));
		rift_tracked_device_frame_release(&d->base, &stale);
		ohmd_lock_mutex(d->device_lock);
		CHECK(rift_tracked_device_exposure_claim(d, &stale.devices[0]) == -1);
		ohmd_unlock_mutex(d->device_lock);
	}
	rift_tracker_exposure_info fresh[3];
	for (int i = 0; i < 3; i++) fresh[i] = exposure(d);
	matrix2d *snapshot = matrix2d_dup(d->ukf_fusion.ukf.x_prior);
	matrix2d *covariance = matrix2d_dup(d->ukf_fusion.ukf.P_prior);
	unsigned before_predictions = predictions;
	barrier(&c.resume);
	CHECK(pthread_join(worker, NULL) == 0);
	/* Other outstanding cameras may arrive later, after the worker already finished. */
	for (int i = 0; i < 3; i++) {
		rift_pose_metrics score = {.match_flags = RIFT_POSE_MATCH_POSITION | RIFT_POSE_MATCH_ORIENT};
		posef pose = identity;
		CHECK(!rift_tracked_device_model_pose_update(&d->base, 0, 0, &old[i], &score, &pose, "stale"));
		rift_tracked_device_frame_release(&d->base, &old[i]);
		ohmd_lock_mutex(d->device_lock);
		CHECK(rift_tracked_device_exposure_claim(d, &old[i].devices[0]) == -1);
		ohmd_unlock_mutex(d->device_lock);
	}
	CHECK(predictions == before_predictions);
	equal_matrix(snapshot, d->ukf_fusion.ukf.x_prior);
	equal_matrix(covariance, d->ukf_fusion.ukf.P_prior);
	CHECK(!d->last_observed_pose_ts && !d->last_observed_orient_ts && !d->ukf_fusion.saw_pose_update);
	for (int i = 0; i < 3; i++) {
		CHECK(d->delay_slots[i].valid && d->delay_slots[i].use_count == 1);
		CHECK(!fresh[i].devices[0].had_pose_lock);
	}
	/* A current camera still fuses and owns its reference after all stale releases. */
	rift_pose_metrics score = {.match_flags = RIFT_POSE_MATCH_POSITION | RIFT_POSE_MATCH_ORIENT};
	posef pose = fresh[0].devices[0].capture_pose;
	CHECK(rift_tracked_device_model_pose_update(&d->base, 0, 0, &fresh[0], &score, &pose, "fresh"));
	CHECK(d->last_observed_pose_ts == d->device_time_ns);
	for (int i = 0; i < 3; i++) rift_tracked_device_frame_release(&d->base, &fresh[i]);
	for (int i = 0; i < 3; i++) CHECK(d->delay_slots[i].use_count == 0);
	matrix2d_free(snapshot);
	matrix2d_free(covariance);
	CHECK(pthread_barrier_destroy(&c.ready) == 0);
	CHECK(pthread_barrier_destroy(&c.resume) == 0);
	destroy_fixture(f);
}

/* Keep the independently validated failure sequence separate from reset-policy
 * assertions so fail-before evidence measures the actual poisoned posterior. */
static void posterior_regression(int slots)
{
	struct fixture *f = create_fixture(slots, 1000000);
	rift_tracked_device_priv *d = f->dev;
	for (int i = 0; i < (slots ? 2500 : 100); i++) report(f, i ? 2000 : 0, .05f);
	if (slots) optical(d);
	report(f, 536872912, .05f);
	report(f, 2000, .05f);
	unsigned frozen = 0;
	for (int i = 0; i < 5000; i++) {
		double before = MATRIX2D_Y(d->ukf_fusion.ukf.x_prior, STATE_ORIENTATION + 1);
		report(f, 2000, (i & 1) ? .07f : .09f);
		frozen += MATRIX2D_Y(d->ukf_fusion.ukf.x_prior, STATE_ORIENTATION + 1) == before;
		if (slots && i % 25 == 0) optical(d);
	}
	printf("posterior regression: slots=%d prediction_failures=%u update_failures=%u frozen=%u/5000\n",
		slots, prediction_failures, update_failures, frozen);
	CHECK(!prediction_failures && !update_failures && !frozen);
	healthy(d);
	destroy_fixture(f);
}

/* These resources are never created by the hardware-free lifecycle fixture. */
void rift_sensor_request_stop(rift_sensor_ctx *s) { (void)s; abort(); }
void rift_sensor_free(rift_sensor_ctx *s) { (void)s; abort(); }
void led_search_model_free(led_search_model_t *s) { (void)s; abort(); }
void ohmd_destroy_thread(ohmd_thread *t) { (void)t; abort(); }

/* Exercise the production owner, including unused slots and construction cleanup.
 * Sensor presence selects delay slots only; no sensor resources are created. */
static int ownership_regression(void)
{
	rift_tracker_free(NULL);
	rift_tracker_free(calloc(1, sizeof(rift_tracker_ctx)));
	rift_tracker_ctx *partial = calloc(1, sizeof *partial);
	partial->devices[0].ukf_fusion.m_position.z = matrix2d_alloc(3, 1);
	rift_tracker_free(partial);
	size_t retained = live_matrices;
	printf("partial tracker retained_matrices=%zu\n", live_matrices);
	for (int slots = 0; slots <= 3; slots += 3) {
		for (int cycle = 0; cycle < 4; cycle++) {
			rift_tracker_ctx *ctx = calloc(1, sizeof *ctx);
			ctx->tracker_lock = ohmd_create_mutex(NULL);
			ctx->n_sensors = slots ? 1 : 0;
			ctx->sensors = slots ? calloc(1, sizeof *ctx->sensors) : NULL;
			posef pose = {.orient = {.w = 1}};
			rift_leds leds = {0};
			rift_tracked_device_imu_calibration cal = {0};
			/* Alternate one and two registrations, leaving unused slots. */
			for (int i = 0; i < 1 + cycle % 2; i++) {
				ctx->devices[i].device_lock = ohmd_create_mutex(NULL);
				rift_tracked_device_priv *dev = (void *)rift_tracker_add_device(ctx,
					i, &pose, &pose, &leds, &cal);
				CHECK(dev->n_delay_slots == slots);
			}
			ctx->n_sensors = 0; /* No actual sensor ownership to drain. */
			rift_tracker_free(ctx);
			printf("ownership slots=%d cycle=%d retained_matrices=%zu\n",
				slots, cycle + 1, live_matrices);
			retained |= live_matrices;
		}
	}
	return retained ? 1 : 0;
}

int main(int argc, char **argv)
{
	if (argc == 2 && strcmp(argv[1], "ownership") == 0)
		return ownership_regression();
	if (argc == 2 && strcmp(argv[1], "continuity") == 0) {
		short_gaps_and_early_epoch();
		stale_camera_cannot_touch_reused_slots();
	} else if (argc == 2 && strcmp(argv[1], "stale-camera") == 0) {
		stale_camera_cannot_touch_reused_slots();
	} else if (argc == 2 && strcmp(argv[1], "posterior-0") == 0) {
		posterior_regression(0);
	} else if (argc == 2 && strcmp(argv[1], "posterior-3") == 0) {
		posterior_regression(3);
	} else {
		CHECK(argc == 1);
		for (int slots = 0; slots <= 3; slots += 3) {
			gap_sequence(slots, 536872912, 1000000);
			gap_sequence(slots, 5000000, 0xfffff000u);
			gap_sequence(slots, UINT32_MAX, 1000000);
		}
	}
	CHECK(predictions && updates && !prediction_failures && !update_failures);
	CHECK(live_matrices == 0);
	puts("test_native_clock_gap: ok");
	return 0;
}
