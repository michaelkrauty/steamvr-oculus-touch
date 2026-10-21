/* cv1touch: decoding a Touch controller's radio message, shared by the engine
 * (handle_touch_controller_message) and the daemon's radio thread (ohmd_cv1touch_radio_decode),
 * which decodes the same messages on its own copy of the controller's state. */
#ifndef RIFT_CV1TOUCH_TOUCH_H
#define RIFT_CV1TOUCH_TOUCH_H

#include <math.h>
#include <string.h>

#include "rift.h"
#include "rift-imu-calib.h"
#include "rift-cv1touch-clock.h"
#include "openhmd_cv1touch.h"

#ifndef OHMD_GRAVITY_EARTH
#define OHMD_GRAVITY_EARTH 9.80665 // m/s²
#endif

static bool touch_message_empty(const pkt_rift_radio_message *msg)
{
	return !(msg->touch.timestamp ||
	      msg->touch.accel[0] || msg->touch.accel[1] || msg->touch.accel[2] ||
	      msg->touch.gyro[0] || msg->touch.gyro[1] || msg->touch.gyro[2]);
}

/* Both readers notify in decoded message order, using only that reader's clock. */
static void touch_activity(ohmd_context *ctx, int reader, const rift_touch_controller_t *touch,
        uint64_t received_ns, bool asleep, float gyro_squared)
{
	if (ctx && ctx->cv1touch_activity_cb)
		ctx->cv1touch_activity_cb(ctx->cv1touch_activity_user, reader, touch->base.id,
		        touch->cv1touch_clock_ns, received_ns, asleep, gyro_squared);
}

static bool touch_empty_activity(ohmd_context *ctx, int reader, const rift_touch_controller_t *touch,
        const pkt_rift_radio_message *msg, uint64_t received_ns)
{
	if (!touch_message_empty(msg))
		return false;
	touch_activity(ctx, reader, touch, received_ns, true, 0.0f);
	return true;
}

/* The message's IMU sample with the factory calibration applied: rad/s and m/s^2, IMU frame. */
static void touch_decode_imu(const rift_touch_calibration *c, const pkt_rift_radio_message *msg, vec3f *gyro_out, vec3f *accel_out)
{
	vec3f raw_accel = {{
		OHMD_GRAVITY_EARTH / 2048 * msg->touch.accel[0],
		OHMD_GRAVITY_EARTH / 2048 * msg->touch.accel[1],
		OHMD_GRAVITY_EARTH / 2048 * msg->touch.accel[2],
	}};

	/* Gyro is MPU 6500, configured for 2000°/s,
	 * The datasheet has 16.4 LSB/°/s, but I'm using
	 * 32768 / 2000 = 16.384 here because that actually
	 * yields a 2000°/s full range, then converting to
	 * radians for the fusion. */
	vec3f raw_gyro = {{
		msg->touch.gyro[0] / (16.384 * 180.0) * M_PI,
		msg->touch.gyro[1] / (16.384 * 180.0) * M_PI,
		msg->touch.gyro[2] / (16.384 * 180.0) * M_PI,
	}};

	/* Factory calibration: subtract the offsets, then apply the rotation / cross-axis matrix */
	rift_imu_apply_calibration(&raw_gyro, &c->gyro_offset, c->gyro_matrix, gyro_out);
	rift_imu_apply_calibration(&raw_accel, &c->accel_offset, c->accel_matrix, accel_out);
}

/* The message's trigger, grip, stick and the one capacitive channel it carries. */
static void touch_decode_inputs(rift_touch_controller_t *touch, const pkt_rift_radio_message *msg)
{
	rift_touch_calibration *c = &touch->calibration;

	float t;
	if (msg->touch.trigger < c->trigger_mid_range) {
		t = 1.0f - ((float)msg->touch.trigger - c->trigger_min_range) /
		    (c->trigger_mid_range - c->trigger_min_range) * 0.5f;
	} else {
		t = 0.5f - ((float)msg->touch.trigger - c->trigger_mid_range) /
		    (c->trigger_max_range - c->trigger_mid_range) * 0.5f;
	}
	touch->trigger = t;

	float gr;
	if (msg->touch.grip < c->middle_mid_range) {
		gr = 1.0f - ((float)msg->touch.grip - c->middle_min_range) /
		     (c->middle_mid_range - c->middle_min_range) * 0.5f;
	} else {
		gr = 0.5f - ((float)msg->touch.grip - c->middle_mid_range) /
		     (c->middle_max_range - c->middle_mid_range) * 0.5f;
	}
	touch->grip = gr;

	float joy[2];
	if (msg->touch.stick[0] >= c->joy_x_dead_min && msg->touch.stick[0] <= c->joy_x_dead_max &&
	    msg->touch.stick[1] >= c->joy_y_dead_min && msg->touch.stick[1] <= c->joy_y_dead_max) {
		joy[0] = 0.0f;
		joy[1] = 0.0f;
	} else {
		joy[0] = ((float)msg->touch.stick[0] - c->joy_x_range_min) /
			 (c->joy_x_range_max - c->joy_x_range_min) * 2.0f - 1.0f;
		joy[1] = ((float)msg->touch.stick[1] - c->joy_y_range_min) /
			 (c->joy_y_range_max - c->joy_y_range_min) * 2.0f - 1.0f;
	}
	touch->stick[0] = joy[0];
	touch->stick[1] = joy[1];

	switch (msg->touch.adc_channel) {
	case RIFT_TOUCH_CONTROLLER_HAPTIC_COUNTER:
		/*
		 * The haptic counter seems to be used as read pointer into a
		 * 256-byte ringbuffer. It is incremented 320 times per second:
		 *
		 * https://developer.oculus.com/documentation/pcsdk/latest/concepts/dg-input-touch-haptic/
		 */
		touch->haptic_counter = msg->touch.adc_value;
		break;
	case RIFT_TOUCH_CONTROLLER_ADC_STICK:
		touch->cap_stick = ((float)msg->touch.adc_value - c->cap_sense_min[0]) /
				   (c->cap_sense_touch[0] - c->cap_sense_min[0]);
		break;
	case RIFT_TOUCH_CONTROLLER_ADC_B_Y:
		touch->cap_b_y = ((float)msg->touch.adc_value - c->cap_sense_min[1]) /
				 (c->cap_sense_touch[1] - c->cap_sense_min[1]);
		break;
	case RIFT_TOUCH_CONTROLLER_ADC_TRIGGER:
		touch->cap_trigger = ((float)msg->touch.adc_value - c->cap_sense_min[2]) /
				     (c->cap_sense_touch[2] - c->cap_sense_min[2]);
		break;
	case RIFT_TOUCH_CONTROLLER_ADC_A_X:
		touch->cap_a_x = ((float)msg->touch.adc_value - c->cap_sense_min[3]) /
				 (c->cap_sense_touch[3] - c->cap_sense_min[3]);
		break;
	case RIFT_TOUCH_CONTROLLER_ADC_REST:
		touch->cap_rest = ((float)msg->touch.adc_value - c->cap_sense_min[7]) /
				  (c->cap_sense_touch[7] - c->cap_sense_min[7]);
		break;
	}
}

/* A message's sample time as the engine keeps it. */
typedef struct {
	int32_t dt;         /* µs since the previous message (2000 for the first) */
	uint32_t device_ts; /* the sample's timestamp: the previous message's (the first's own less dt) */
	uint64_t device_ns; /* on the controller's device clock (rift-cv1touch-clock.h) */
} cv1touch_sample_time;

/* Times a non-empty message's sample and advances `touch`'s timing state past it. */
static cv1touch_sample_time touch_sample_time(rift_touch_controller_t *touch, const pkt_rift_radio_message *msg)
{
	cv1touch_sample_time t;
	t.dt = touch->time_valid ? (int32_t)(msg->touch.timestamp - touch->last_timestamp) : 2000; /* 500Hz updates = 2000µS spacing */
	t.device_ts = msg->touch.timestamp - t.dt;
	t.device_ns = rift_device_time_next(touch->cv1touch_clock_ns, touch->cv1touch_clock_ts, t.device_ts);
	touch->cv1touch_clock_ns = t.device_ns;
	touch->cv1touch_clock_ts = t.device_ts;
	touch->last_timestamp = msg->touch.timestamp;
	touch->time_valid = true;
	return t;
}

/* Notify after factory calibration, before a later sample can replace this motion evidence. */
static cv1touch_sample_time touch_sample_imu(ohmd_context *ctx, int reader,
        rift_touch_controller_t *touch, const pkt_rift_radio_message *msg, uint64_t received_ns,
        vec3f *gyro, vec3f *accel)
{
	cv1touch_sample_time t = touch_sample_time(touch, msg);
	touch_decode_imu(&touch->calibration, msg, gyro, accel);
	touch_activity(ctx, reader, touch, received_ns, false,
	        gyro->x * gyro->x + gyro->y * gyro->y + gyro->z * gyro->z);
	return t;
}

/* A calibrated controller's message decoded on `touch`'s own state into `out` (IMU sample,
 * inputs, times), received at host time `received_ns`; false for an empty message, which only
 * carries the buttons. */
static bool touch_decode_sample(ohmd_context *ctx, rift_touch_controller_t *touch, const pkt_rift_radio_message *msg, uint64_t received_ns, ohmd_cv1touch_controller *out)
{
	touch->buttons = msg->touch.buttons & 0xf;
	if (touch_empty_activity(ctx, 1, touch, msg, received_ns))
		return false;
	vec3f gyro, accel;
	cv1touch_sample_time t = touch_sample_imu(ctx, 1, touch, msg, received_ns, &gyro, &accel);
	touch_decode_inputs(touch, msg);

	memset(out, 0, sizeof *out);
	out->calibrated = true;
	out->device_id = touch->base.id;
	out->device_time_ns = t.device_ns;
	out->received_host_ns = received_ns;
	out->buttons = touch->buttons;
	out->trigger = touch->trigger;
	out->grip = touch->grip;
	out->stick[0] = touch->stick[0];
	out->stick[1] = touch->stick[1];
	out->cap[0] = touch->cap_stick;
	out->cap[1] = touch->cap_b_y;
	out->cap[2] = touch->cap_trigger;
	out->cap[3] = touch->cap_a_x;
	out->cap[4] = touch->cap_rest;
	out->imu_accel[0] = accel.x;
	out->imu_accel[1] = accel.y;
	out->imu_accel[2] = accel.z;
	out->imu_gyro[0] = gyro.x;
	out->imu_gyro[1] = gyro.y;
	out->imu_gyro[2] = gyro.z;
	return true;
}

#endif
