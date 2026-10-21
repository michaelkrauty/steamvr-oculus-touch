// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
/* Touch activity reaches the daemon from both production readers: the engine's radio report
 * handler (reader 0) and the radio thread's decoder (reader 1), empty sleep beacons included.
 * A unity build of the engine with its tracker, radio commands and HID stubbed; no devices. */
#include "rift.c"
#include "test_util.h"
#include "omath.c"
#include "packet.c"
#include "openhmd.c"

double ohmd_get_tick(void) { return 0; }
uint64_t ohmd_monotonic_get(ohmd_context *ctx) { (void)ctx; return 0; }
rift_tracked_device *rift_tracker_add_device(rift_tracker_ctx *tracker, int id,
    posef *imu, posef *model, rift_leds *leds, rift_tracked_device_imu_calibration *calibration)
{
	(void)tracker; (void)id; (void)imu; (void)model; (void)leds; (void)calibration;
	CHECK(false);
	return NULL;
}
int rift_touch_get_calibration(ohmd_context *ctx, rift_hmd_radio_state *radio, int device_num,
    rift_touch_calibration *calibration, rift_touch_calibration_state *state)
{
	(void)ctx; (void)radio; (void)device_num; (void)calibration; (void)state;
	return -1; /* no answer from the controller: nothing decoded but the beacon */
}
static int imu_updates;
void rift_tracked_device_imu_update(rift_tracked_device *dev, uint64_t local_ts, uint32_t device_ts,
    float dt, const vec3f *ang_vel, const vec3f *accel, const vec3f *mag_field)
{
	(void)dev; (void)local_ts; (void)device_ts; (void)dt; (void)ang_vel; (void)accel; (void)mag_field;
	imu_updates++;
}

static struct {
	int n, reader[8], device[8];
	uint64_t host[8];
	bool asleep[8];
	float gyro_squared[8];
} seen;

static void record(void *user, int reader, int device, uint64_t device_ns, uint64_t host, bool asleep,
    float gyro_squared)
{
	(void)device_ns;
	CHECK(user == &seen && seen.n < 8);
	int n = seen.n++;
	seen.reader[n] = reader;
	seen.device[n] = device;
	seen.host[n] = host;
	seen.asleep[n] = asleep;
	seen.gyro_squared[n] = gyro_squared;
}

/* A radio report holding one Touch message: empty (a sleep beacon) or an IMU sample turning
 * about z. */
static void report(unsigned char out[RIFT_RADIO_REPORT_SIZE], int device_type, bool sample)
{
	memset(out, 0, RIFT_RADIO_REPORT_SIZE);
	out[0] = RIFT_RADIO_REPORT_ID;
	out[3] = 0x1c;
	out[5] = device_type;
	if (sample) {
		out[7] = 0x10;  /* timestamp 4096 µs */
		out[15] = 0x08; /* accel[2] 2048: 1 g */
		out[21] = 0x40; /* gyro[2] 16384: about 17 rad/s */
	}
}

static void calibrate(rift_touch_controller_t *t)
{
	t->have_calibration = true;
	for (int i = 0; i < 3; i++) {
		t->calibration.accel_matrix[i][i] = 1.0f;
		t->calibration.gyro_matrix[i][i] = 1.0f;
	}
}

static void engine_reader_reports_beacons_and_samples(void)
{
	ohmd_context ctx = {0};
	ohmd_cv1touch_set_activity_callback(&ctx, record, &seen);
	rift_hmd_t hmd = {0};
	hmd.ctx = &ctx;
	hmd.touch_dev[0].base.id = 1;
	hmd.touch_dev[1].base.id = 2;
	memset(&seen, 0, sizeof seen);
	unsigned char buf[RIFT_RADIO_REPORT_SIZE];
	report(buf, RIFT_TOUCH_CONTROLLER_LEFT, false);
	handle_rift_radio_report(&hmd, 100, buf, sizeof buf);
	CHECK(seen.n == 1 && seen.reader[0] == 0 && seen.device[0] == 2 && seen.asleep[0]);
	CHECK(seen.host[0] == 100);
	/* Without its calibration a sample is not decoded: no activity. */
	report(buf, RIFT_TOUCH_CONTROLLER_RIGHT, true);
	handle_rift_radio_report(&hmd, 102, buf, sizeof buf);
	CHECK(seen.n == 1);
	calibrate(&hmd.touch_dev[0]);
	handle_rift_radio_report(&hmd, 104, buf, sizeof buf);
	CHECK(seen.n == 2 && seen.reader[1] == 0 && seen.device[1] == 1 && !seen.asleep[1]);
	CHECK(seen.host[1] == 104 && seen.gyro_squared[1] > 0.0f && imu_updates == 1);
}

static void radio_decoder_reports_beacons_before_the_calibration(void)
{
	ohmd_context ctx = {0};
	ohmd_cv1touch_set_activity_callback(&ctx, record, &seen);
	rift_hmd_t hmd = {0};
	hmd.ctx = &ctx;
	struct ohmd_cv1touch_radio_decoder dec = {.hmd = &hmd};
	dec.touch[0].base.id = 1;
	dec.touch[1].base.id = 2;
	memset(&seen, 0, sizeof seen);
	unsigned char buf[RIFT_RADIO_REPORT_SIZE];
	ohmd_cv1touch_controller out[2];
	/* A beacon is activity even before the engine has the controller's calibration. */
	report(buf, RIFT_TOUCH_CONTROLLER_RIGHT, false);
	CHECK(ohmd_cv1touch_radio_decode(&dec, buf, sizeof buf, 200, out) == 0);
	CHECK(seen.n == 1 && seen.reader[0] == 1 && seen.device[0] == 1 && seen.asleep[0]);
	CHECK(seen.host[0] == 200);
	report(buf, RIFT_TOUCH_CONTROLLER_LEFT, true);
	CHECK(ohmd_cv1touch_radio_decode(&dec, buf, sizeof buf, 202, out) == 0);
	CHECK(seen.n == 1);
	calibrate(&hmd.touch_dev[1]);
	CHECK(ohmd_cv1touch_radio_decode(&dec, buf, sizeof buf, 204, out) == 1);
	CHECK(seen.n == 2 && seen.reader[1] == 1 && seen.device[1] == 2 && !seen.asleep[1]);
	CHECK(seen.host[1] == 204 && seen.gyro_squared[1] > 0.0f);
	CHECK(out[0].received_host_ns == 204 && out[0].device_id == 2);
}

int main(void)
{
	engine_reader_reports_beacons_and_samples();
	radio_decoder_reports_beacons_before_the_calibration();
	puts("test_radio_activity: ok");
	return 0;
}
