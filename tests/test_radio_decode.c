// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* A Touch message decoded the way the engine decodes it (rift-cv1touch-touch.h), which the radio
 * thread's decoder repeats on its own copy of the controller's state: sample timing (stamped with
 * the previous message's timestamp, the first with its own less 2 ms), empty messages, the factory
 * calibration, and state kept apart from the engine's. */
#include <string.h>

#include "omath.c"
#include "rift-cv1touch-touch.h"
#include "test_util.h"

static pkt_rift_radio_message touch_msg(uint32_t timestamp, int16_t accel_x, int16_t gyro_z, uint8_t buttons)
{
	pkt_rift_radio_message m;
	memset(&m, 0, sizeof m);
	m.valid = true;
	m.device_type = RIFT_TOUCH_CONTROLLER_RIGHT;
	m.touch.timestamp = timestamp;
	m.touch.accel[0] = accel_x;
	m.touch.accel[2] = 2048; /* 1 g */
	m.touch.gyro[2] = gyro_z;
	m.touch.buttons = buttons;
	return m;
}

static void calibrated(rift_touch_controller_t *t)
{
	memset(t, 0, sizeof *t);
	t->base.id = 2;
	t->have_calibration = true;
	for (int i = 0; i < 3; i++) {
		t->calibration.accel_matrix[i][i] = 1.0f;
		t->calibration.gyro_matrix[i][i] = 1.0f;
	}
	/* ranges that make the inputs well defined */
	t->calibration.trigger_min_range = 0;
	t->calibration.trigger_mid_range = 512;
	t->calibration.trigger_max_range = 1023;
	t->calibration.middle_min_range = 0;
	t->calibration.middle_mid_range = 512;
	t->calibration.middle_max_range = 1023;
	t->calibration.joy_x_range_min = 0;
	t->calibration.joy_x_range_max = 1023;
	t->calibration.joy_y_range_min = 0;
	t->calibration.joy_y_range_max = 1023;
}

static void times_samples_like_the_engine(void)
{
	rift_touch_controller_t t;
	calibrated(&t);
	pkt_rift_radio_message a = touch_msg(1000000, 0, 0, 0), b = touch_msg(1002000, 0, 0, 0), c = touch_msg(1004100, 0, 0, 0);

	/* the first sample: its own timestamp less 2 ms */
	cv1touch_sample_time ta = touch_sample_time(&t, &a);
	CHECK(ta.dt == 2000 && ta.device_ts == 998000);
	CHECK(ta.device_ns == rift_device_time_next(0, 0, 998000));
	/* then each one is stamped with the previous message's timestamp */
	cv1touch_sample_time tb = touch_sample_time(&t, &b);
	CHECK(tb.dt == 2000 && tb.device_ts == 1000000);
	CHECK(tb.device_ns == ta.device_ns + 2000000);
	cv1touch_sample_time tc = touch_sample_time(&t, &c);
	CHECK(tc.dt == 2100 && tc.device_ts == 1002000);
	CHECK(tc.device_ns == tb.device_ns + 2000000);
}

static void an_empty_message_carries_only_the_buttons(void)
{
	rift_touch_controller_t t;
	calibrated(&t);
	pkt_rift_radio_message m = touch_msg(0, 0, 0, 0x13); /* top bits are not buttons */
	m.touch.accel[2] = 0;
	ohmd_cv1touch_controller out;
	CHECK(!touch_decode_sample(NULL, &t, &m, 5, &out));
	CHECK(t.buttons == 0x3);
	CHECK(!t.time_valid); /* and no time passes */
}

static void decodes_the_imu_sample_and_times(void)
{
	rift_touch_controller_t t;
	calibrated(&t);
	ohmd_cv1touch_controller out;
	pkt_rift_radio_message m = touch_msg(500000, 1024, 16384, 0x1);
	CHECK(touch_decode_sample(NULL, &t, &m, 777, &out));
	CHECK(out.calibrated && out.device_id == 2 && out.received_host_ns == 777);
	CHECK(out.device_time_ns == rift_device_time_next(0, 0, 498000));
	CHECK(out.buttons == 0x1);
	CHECK_NEAR(out.imu_accel[0], OHMD_GRAVITY_EARTH / 2, 1e-4); /* 1024 of 2048 per g */
	CHECK_NEAR(out.imu_accel[2], OHMD_GRAVITY_EARTH, 1e-4);
	CHECK_NEAR(out.imu_gyro[2], 1000.0 / 180.0 * M_PI, 1e-3); /* half of the 2000 deg/s range */
}

static void the_decoders_state_is_its_own(void)
{
	/* the engine's controller and a decoder's copy decode the same messages; neither moves the
	 * other's clock, and both time them alike from the same start */
	rift_touch_controller_t engine, copy;
	calibrated(&engine);
	calibrated(&copy);
	ohmd_cv1touch_controller out;
	pkt_rift_radio_message a = touch_msg(3000000, 0, 0, 0), b = touch_msg(3002000, 0, 0, 0);
	CHECK(touch_decode_sample(NULL, &copy, &a, 1, &out));
	CHECK(touch_decode_sample(NULL, &copy, &b, 2, &out));
	CHECK(!engine.time_valid && engine.cv1touch_clock_ns == 0);
	cv1touch_sample_time ea = touch_sample_time(&engine, &a);
	cv1touch_sample_time eb = touch_sample_time(&engine, &b);
	(void) ea;
	CHECK(eb.device_ns == out.device_time_ns);
}

static struct {
	int n, reader[4], device[4];
	uint64_t host[4], device_ns[4];
	bool asleep[4];
	float gyro_squared[4];
} activity;

static void ordered_activity(void *user, int reader, int device, uint64_t device_ns, uint64_t host, bool asleep, float gyro_squared)
{
	CHECK(user == &activity && activity.n < 4);
	int n = activity.n++;
	activity.reader[n] = reader;
	activity.device[n] = device;
	activity.host[n] = host;
	activity.device_ns[n] = device_ns;
	activity.asleep[n] = asleep;
	activity.gyro_squared[n] = gyro_squared;
}

static void activity_follows_decoding_on_each_reader(void)
{
	for (int reader = 0; reader < 2; reader++) {
		ohmd_context ctx = {0};
		ctx.cv1touch_activity_cb = ordered_activity;
		ctx.cv1touch_activity_user = &activity;
		memset(&activity, 0, sizeof activity);
		rift_touch_controller_t t;
		calibrated(&t);
		pkt_rift_radio_message data = touch_msg(1000000, 0, 0, 0), beacon = {0};
		ohmd_cv1touch_controller out;
		for (int i = 0; i < 4; i++) {
			bool empty = i == 0 || i == 2;
			pkt_rift_radio_message *m = empty ? &beacon : &data;
			data.touch.gyro[2] = i == 1 ? 16384 : 0;
			uint64_t host = 750 + i * 2;
			if (reader == 0) {
				if (!touch_empty_activity(&ctx, reader, &t, m, host)) {
					vec3f gyro, accel;
					touch_sample_imu(&ctx, reader, &t, m, host, &gyro, &accel);
				}
			} else {
				CHECK(touch_decode_sample(&ctx, &t, m, host, &out) == !empty);
			}
			CHECK(activity.n == i + 1 && activity.reader[i] == reader);
			CHECK(activity.device[i] == 2 && activity.host[i] == host);
			CHECK(activity.asleep[i] == empty);
			CHECK((activity.gyro_squared[i] > 0.25f) == (i == 1));
			CHECK(activity.device_ns[i] == t.cv1touch_clock_ns);
			if (!empty) data.touch.timestamp += 2000;
		}
		CHECK(activity.device_ns[0] == 0);
		CHECK(activity.device_ns[1] == activity.device_ns[2]);
		CHECK(activity.device_ns[3] > activity.device_ns[2]);
	}
}

int main(void)
{
	activity_follows_decoding_on_each_reader();
	times_samples_like_the_engine();
	an_empty_message_carries_only_the_buttons();
	decodes_the_imu_sample_and_times();
	the_decoders_state_is_its_own();
	puts("test_radio_decode: ok");
	return 0;
}
