/*
 * Copyright 2013, Fredrik Hultin.
 * Copyright 2013, Jakob Bornecrantz.
 * Copyright 2016 Philipp Zabel
 * Copyright 2019 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift Driver - HID/USB Driver Implementation */


#include <stdlib.h>
#include <hidapi.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <assert.h>
#include <errno.h>

#include "rift.h"
#include "rift-hmd-radio.h"
#include "rift-tracker.h"
#include "rift-imu-calib.h"
#include "rift-cv1touch-clock.h"
#include "rift-cv1touch-touch.h"
#include "../hid.h"


#define UDEV_WIKI_URL "https://github.com/OpenHMD/OpenHMD/wiki/Udev-rules-list"
#define OCULUS_VR_INC_ID 0x2833
#define SAMSUNG_ELECTRONICS_CO_ID 0x04e8
#define RIFT_CV1_PID 0x0031

#define TICK_LEN (1000000 / 1000) // 1000 Hz ticks, in uS
#define TICK_US_TO_NS(t) ((t) * 1000)
#define TICK_US_TO_SEC(t) ((float)(t) / 1000000.0)
#define TICK_DIFF(a,b) ((int32_t)((a) - (b)))

#define KEEP_ALIVE_VALUE (10 * 1000)
#define SETFLAG(_s, _flag, _val) (_s) = ((_s) & ~(_flag)) | ((_val) ? (_flag) : 0)

typedef enum {
	REV_DK1,
	REV_DK2,
	REV_CV1,

	REV_GEARVR_GEN1
} rift_revision;

struct rift_hmd_s {
	ohmd_context* ctx;
	rift_revision revision;
	int use_count;

	hid_device* handle;
	hid_device* radio_handle;
	rift_hmd_radio_state radio;

	pkt_sensor_range sensor_range;
	pkt_imu_calibration imu_calibration;
	pkt_sensor_display_info display_info;
	rift_coordinate_frame coordinate_frame, hw_coordinate_frame;
	pkt_sensor_config sensor_config;
	pkt_tracker_sensor sensor;
	bool have_imu_timestamp;
	uint32_t last_imu_timestamp;
	double last_keep_alive;
	rift_tracked_device *tracked_dev;

	struct {
		vec3f pos;
	} imu;

	uint8_t radio_address[5];
	uint32_t io_errors; /* cv1touch: HID read failures (headset unplugged) */
	rift_leds leds;

	uint16_t remote_buttons_state;

	rift_tracker_ctx *tracker_ctx;

	/* OpenHMD output devices */
	rift_device_priv hmd_dev;
	rift_touch_controller_t touch_dev[2];
};

typedef struct device_list_s device_list_t;
struct device_list_s {
	char path[OHMD_STR_SIZE];
	rift_hmd_t *hmd;

	device_list_t* next;
};

typedef struct {
	const char* name;
	int company;
	int id;
	int iface;
	rift_revision rev;
} rift_devices;

/* Global list of (probably 1) active HMD devices */
static device_list_t* rift_hmds;

static hid_device* open_hid_dev (ohmd_context* ctx, int vid, int pid, int iface_num);
static void close_hmd (rift_hmd_t *hmd);

static rift_hmd_t *find_hmd(char *hid_path)
{
	device_list_t* current = rift_hmds;

	while (current != NULL) {
		if (strcmp(current->path, hid_path)==0) {
			current->hmd->use_count++;
			return current->hmd;
		}
		current = current->next;
	}
	return NULL;
}

static void push_hmd(rift_hmd_t *hmd, char *hid_path)
{
	device_list_t* d = calloc(1, sizeof(device_list_t));
	d->hmd = hmd;
	strcpy (d->path, hid_path);

	d->next = rift_hmds;
	rift_hmds = d;
}

static void release_hmd(rift_hmd_t *hmd)
{
	device_list_t* current, *prev;

	if (hmd->use_count > 1) {
		hmd->use_count--;
		return;
	}

	/* Use count on the HMD device hit 0, release it
	 * and remove from the list */
	current = rift_hmds;
	prev = NULL;
	while (current != NULL) {
		if (current->hmd == hmd) {
			close_hmd (current->hmd);

			if (prev == NULL)
				rift_hmds = current->next;
			else
				prev->next = current->next;
			free (current);
			return;
		}
		prev = current;
		current = current->next;
	}

	LOGE("Failed to find HMD in the active device list");
}

static rift_device_priv* rift_device_priv_get(ohmd_device* device)
{
	return (rift_device_priv*)device;
}

static int get_feature_report(rift_hmd_t* priv, rift_sensor_feature_cmd cmd, unsigned char* buf)
{
	memset(buf, 0, FEATURE_BUFFER_SIZE);
	buf[0] = (unsigned char)cmd;
	return hid_get_feature_report(priv->handle, buf, FEATURE_BUFFER_SIZE);
}

static int send_feature_report(rift_hmd_t* priv, const unsigned char *data, size_t length)
{
	return hid_send_feature_report(priv->handle, data, length);
}

static void set_coordinate_frame(rift_hmd_t* priv, rift_coordinate_frame coordframe)
{
	priv->coordinate_frame = coordframe;

	// set the RIFT_SCF_SENSOR_COORDINATES in the sensor config to match whether coordframe is hmd or sensor
	SETFLAG(priv->sensor_config.flags, RIFT_SCF_SENSOR_COORDINATES, coordframe == RIFT_CF_SENSOR);

	// encode send the new config to the Rift
	unsigned char buf[FEATURE_BUFFER_SIZE];
	int size = encode_sensor_config(buf, &priv->sensor_config);
	if(send_feature_report(priv, buf, size) == -1){
		ohmd_set_error(priv->ctx, "send_feature_report failed in set_coordinate frame");
		return;
	}

	// read the state again, set the hw_coordinate_frame to match what
	// the hardware actually is set to just in case it doesn't stick.
	size = get_feature_report(priv, RIFT_CMD_SENSOR_CONFIG, buf);
	if(size <= 0){
		LOGW("could not set coordinate frame");
		priv->hw_coordinate_frame = RIFT_CF_HMD;
		return;
	}

	decode_sensor_config(&priv->sensor_config, buf, size);
	priv->hw_coordinate_frame = (priv->sensor_config.flags & RIFT_SCF_SENSOR_COORDINATES) ? RIFT_CF_SENSOR : RIFT_CF_HMD;

	if(priv->hw_coordinate_frame != coordframe) {
		LOGW("coordinate frame didn't stick");
	}
}

static void handle_tracker_sensor_msg(rift_hmd_t* priv, uint64_t local_ts, unsigned char* buffer, int size)
{
	if (buffer[0] == RIFT_IRQ_SENSORS_DK1
	  && !decode_tracker_sensor_msg_dk1(&priv->sensor, buffer, size)){
		LOGE("couldn't decode tracker sensor message");
	}
	else if (buffer[0] == RIFT_IRQ_SENSORS_DK2 /* DK2 and CV1 variant */
	  && !decode_tracker_sensor_msg_dk2(&priv->sensor, buffer, size)){
		LOGE("couldn't decode tracker sensor message");
	}

	pkt_tracker_sensor* s = &priv->sensor;
	vec3f raw_mag;

	dump_packet_tracker_sensor(s);

	int32_t mag32[] = { s->mag[0], s->mag[1], s->mag[2] };
	vec3f_from_rift_vec(mag32, &raw_mag);

	uint32_t dt = TICK_LEN; // TODO: query the Rift for the sample rate

	/* If we have a gap since the last sample handled, treat the
	 * first sample here as having the full dt */
	if (priv->have_imu_timestamp) {
		dt = (s->timestamp - priv->last_imu_timestamp);
		dt -= (s->num_samples - 1) * TICK_LEN; // TODO: query the Rift for the sample rate
	}

	/* Compute the starting timestamps, in local system time and device time */
	uint32_t total_dt = (s->num_samples-1)*TICK_LEN + dt;
	uint32_t device_ts = s->timestamp - total_dt;
	bool sent_exposure_update = false;

	local_ts -= TICK_US_TO_NS(total_dt);

	for(int i = 0; i < s->num_samples; i++){
		vec3f raw_gyro, gyro;
		vec3f raw_accel, accel;

		/* if the exposure timestamp is earlier than this sample, report it now */
		if (!sent_exposure_update && TICK_DIFF(device_ts, s->exposure_timestamp) >= 0) { 
			rift_tracker_on_new_exposure (priv->tracker_ctx, device_ts, s->exposure_count,
					s->exposure_timestamp, s->led_pattern_phase);
			sent_exposure_update = true;
		}

		vec3f_from_rift_vec(s->samples[i].accel, &raw_accel);
		vec3f_from_rift_vec(s->samples[i].gyro, &raw_gyro);

		/* If the rift isn't applying calibration, we should */
		if (!(priv->sensor_config.flags & RIFT_SCF_USE_CALIBRATION)) {
				rift_imu_apply_calibration(&raw_gyro, &priv->imu_calibration.gyro_offset,
						priv->imu_calibration.gyro_matrix, &gyro);
				rift_imu_apply_calibration(&raw_accel, &priv->imu_calibration.accel_offset,
						priv->imu_calibration.accel_matrix, &accel);
		}
		else {
				gyro = raw_gyro;
				accel = raw_accel;
		}

		/* cv1touch: the headset may be radio-only (not a tracked device) */
		if (priv->tracked_dev)
			rift_tracked_device_imu_update(priv->tracked_dev, local_ts, device_ts, TICK_US_TO_SEC(dt), &gyro, &accel, &raw_mag);

		device_ts += dt;
		local_ts += TICK_US_TO_NS(dt);
		dt = TICK_LEN; // TODO: query the Rift for the sample rate
	}
	priv->last_imu_timestamp = s->timestamp;
	priv->have_imu_timestamp = true;

	if (!sent_exposure_update) {
		rift_tracker_on_new_exposure (priv->tracker_ctx, s->timestamp, s->exposure_count,
				s->exposure_timestamp, s->led_pattern_phase);
	}
}

static void dump_controller_calibration(rift_touch_controller_t *touch)
{
	rift_touch_calibration *c = &touch->calibration;
	int i;

	LOGI ("%s Controller IMU calibration  gyro_offset [ %f, %f, %f ]  accel_offset [ %f, %f, %f ]",
		touch->base.id == 1 ? "Right" : "Left",
		c->gyro_offset.x, c->gyro_offset.y, c->gyro_offset.z,
		c->accel_offset.x, c->accel_offset.y, c->accel_offset.z);

	LOGI("  gyro_calib = ");
	for (i = 0; i < 3; i++) {
		LOGI("  [ %8.3f, %8.3f, %8.3f ]", c->gyro_matrix[i][0],
			c->gyro_matrix[i][1], c->gyro_matrix[i][2]);
	}

	LOGI("  accel_calib = ");
	for (i = 0; i < 3; i++) {
		LOGI("  [ %8.3f, %8.3f, %8.3f ]", c->accel_matrix[i][0],
			c->accel_matrix[i][1], c->accel_matrix[i][2]);
	}
}

static void handle_touch_controller_message(rift_hmd_t *hmd, uint64_t local_ts,
		rift_touch_controller_t *touch, pkt_rift_radio_message *msg)
{
	uint64_t cv1touch_received_ns = local_ts;
	// The top bits are carrying something unknown. Ignore them
	uint8_t buttons = msg->touch.buttons & 0xf;
	rift_touch_calibration *c = &touch->calibration;

	if (touch->buttons != buttons) {
		LOGV ("touch controller %d buttons now %x",
				touch->base.id, buttons);
	}
	touch->buttons = buttons;

	if (touch_message_empty(msg))
		return;

	if (!touch->have_calibration) {
		rift_tracked_device_imu_calibration imu_calibration;
		int i;

		/* We need calibration data to do any more */
		if (rift_touch_get_calibration (hmd->ctx, &hmd->radio, touch->device_num,
				&touch->calibration) < 0)
			return;

		quatf imu_orient = {{ 0.0, 0.0, 0.0, 1.0 }};
		posef imu_pose;
		oposef_init(&imu_pose, &touch->calibration.imu_position, &imu_orient);

		quatf model_orient = {{ 0.0, 0.0, 0.0, 1.0 }};
		vec3f model_pos = {{ 0.0, 0.0, 0.0 }};
		posef model_pose;
		oposef_init(&model_pose, &model_pos, &model_orient);

		/* Copy into imu_calibration array */
		imu_calibration.accel_offset = touch->calibration.accel_offset;
		imu_calibration.gyro_offset = touch->calibration.gyro_offset;
		for (i = 0; i < 9; i++) {
			imu_calibration.accel_matrix[i] = touch->calibration.accel_matrix[i/3][i%3];
			imu_calibration.gyro_matrix[i] = touch->calibration.gyro_matrix[i/3][i%3];
		}

		touch->tracked_dev = rift_tracker_add_device (hmd->tracker_ctx, touch->base.id, &imu_pose, &model_pose, &touch->calibration.leds, &imu_calibration);
		memcpy(touch->cv1touch_calibration_hash, hmd->radio.calibration_hash, 16);
		/* release: the radio thread's decoder copies the calibration once it sees the flag */
		__atomic_store_n(&touch->have_calibration, true, __ATOMIC_RELEASE);
		dump_controller_calibration(touch);
	}

	// time in microseconds
	cv1touch_sample_time t = touch_sample_time(touch, msg);
	int32_t dt = t.dt;
	uint32_t device_ts = t.device_ts;
	local_ts -= TICK_US_TO_NS(dt);

	const double dt_s = 1e-6 * dt;
	vec3f mag = {{0.0f, 0.0f, 0.0f}};
	vec3f gyro;
	vec3f accel;
	touch_decode_imu(c, msg, &gyro, &accel);
	touch->cv1touch_accel = accel;
	touch->cv1touch_gyro = gyro;

	uint64_t filter_start = ohmd_monotonic_get(hmd->ctx);
	rift_tracked_device_imu_update(touch->tracked_dev, local_ts, device_ts, dt_s, &gyro, &accel, &mag);
	touch->cv1touch_filter_ns = ohmd_monotonic_get(hmd->ctx) - filter_start;
	touch->cv1touch_received_ns = cv1touch_received_ns;

	touch_decode_inputs(touch, msg);
}

static void handle_rift_radio_message(rift_hmd_t *hmd, uint64_t ts, pkt_rift_radio_message *msg)
{
	switch (msg->device_type) {
		case RIFT_REMOTE:
			if (hmd->remote_buttons_state != msg->remote.buttons) {
				LOGV ("Remote buttons state 0x%02x", msg->remote.buttons);
			}
			hmd->remote_buttons_state = msg->remote.buttons;
			break;
		case RIFT_TOUCH_CONTROLLER_RIGHT:
			handle_touch_controller_message (hmd, ts, &hmd->touch_dev[0], msg);
			break;
		case RIFT_TOUCH_CONTROLLER_LEFT:
			handle_touch_controller_message (hmd, ts, &hmd->touch_dev[1], msg);
			break;
	}
}

static void rift_send_keepalive(rift_hmd_t *priv)
{
	unsigned char buffer[FEATURE_BUFFER_SIZE];

	// send keep alive message
	pkt_keep_alive keep_alive = { 0, priv->sensor_config.keep_alive_interval };
	int ka_size;
	if (priv->revision == REV_DK1)
		ka_size = encode_dk1_keep_alive(buffer, &keep_alive);
	else
		ka_size = encode_dk2_keep_alive(buffer, &keep_alive);
	if (send_feature_report(priv, buffer, ka_size) == -1)
		LOGW("error sending keepalive");
}


static void handle_rift_radio_report(rift_hmd_t* hmd, uint64_t ts, unsigned char* buffer, int size)
{
	pkt_rift_radio_report r;

	if (!decode_rift_radio_report(&r, buffer, size))
		return;

	if (r.message[0].valid)
		handle_rift_radio_message(hmd, ts, &r.message[0]);
	if (r.message[1].valid)
		handle_rift_radio_message(hmd, ts, &r.message[1]);
}

/* cv1touch: the outcome of a haptics write. Confirmed, it is what the controller now has, and
 * a pulse's time starts. Otherwise the controller may or may not have it: the wanted state is
 * sent again shortly. A pulse that cannot get through is given up after 0.2 s, and an off after
 * 0.5 s (a controller that stopped answering is not vibrating either). */
static void haptics_write_done(rift_hmd_t *hmd, uint64_t ts, rift_touch_controller_t *touch, int ret)
{
	rift_haptic_state *hs = &touch->haptic_state;
	const uint64_t ms = ohmd_monotonic_per_sec(hmd->ctx) / 1000;

	if (ret == 0) {
		hs->sent_amplitude = hs->inflight_amplitude;
		hs->sent_low_freq = hs->inflight_low_freq;
		hs->failures = 0;
		if (hs->haptics_on && hs->inflight_amplitude != 0 && hs->end_time == (uint64_t)(-1))
			hs->end_time = ts + roundf(hs->duration * ohmd_monotonic_per_sec(hmd->ctx));
		return;
	}
	rift_touch_cancel_in_progress(&hmd->radio, touch->device_num);
	hs->dirty = true;
	hs->failures++;
	hs->retry_time = ts + 10 * ms;
	if (hs->failures == 1)
		LOGW("cv1touch: haptics write to device %d failed (%d); retrying", touch->device_num, ret);
	if (hs->inflight_amplitude != 0 && hs->failures >= 20) {
		hs->haptics_on = false; /* the pulse never started: drop it */
	} else if (hs->inflight_amplitude == 0 && hs->failures >= 50) {
		LOGW("cv1touch: device %d does not confirm haptics off; giving up", touch->device_num);
		hs->sent_amplitude = 0;
		hs->dirty = false;
		hs->failures = 0;
	}
}

/* cv1touch: brings the controller's vibration to the wanted state. A write in flight is
 * finished before anything else is sent: cancelling it leaves the radio waiting on its result,
 * and the next write would be skipped while the old one is taken as its confirmation. */
static void check_haptics_state(rift_hmd_t *hmd, uint64_t ts, rift_touch_controller_t *touch)
{
	rift_haptic_state *hs = &touch->haptic_state;

	/* A pulse ends on its own timer */
	if (hs->haptics_on && hs->end_time < ts)
		hs->haptics_on = false;

	if (hs->in_progress) {
		int ret = rift_touch_send_haptics(&hmd->radio, touch->device_num, hs->inflight_low_freq, hs->inflight_amplitude);
		if (ret == -EINPROGRESS || ret == -EBUSY)
			return;
		hs->in_progress = false;
		haptics_write_done(hmd, ts, touch, ret);
	}

	const uint8_t want = hs->haptics_on ? hs->amplitude : 0;
	const bool want_lf = hs->low_freq;
	const bool same = want == hs->sent_amplitude && (want == 0 || want_lf == hs->sent_low_freq);

	/* A new pulse at what the controller is already doing only needs its timer */
	if (hs->dirty && same && want != 0 && hs->end_time == (uint64_t)(-1)) {
		hs->end_time = ts + roundf(hs->duration * ohmd_monotonic_per_sec(hmd->ctx));
		hs->dirty = false;
		return;
	}
	if (same) {
		hs->dirty = false;
		return;
	}
	if (ts < hs->retry_time)
		return;

	hs->dirty = false;
	hs->inflight_amplitude = want;
	hs->inflight_low_freq = want_lf;
	int ret = rift_touch_send_haptics(&hmd->radio, touch->device_num, want_lf, want);
	if (ret == -EINPROGRESS) {
		hs->in_progress = true;
	} else if (ret == -EBUSY) {
		hs->dirty = true; /* the radio is busy with something else: next pass */
	} else {
		haptics_write_done(hmd, ts, touch, ret);
	}
}

/* Keep-alive and haptics: what an update sends before it reads anything. */
static void update_hmd_outputs(rift_hmd_t *priv, uint64_t now)
{
	// Handle keep alive messages
	double t = ohmd_get_tick();
	if(t - priv->last_keep_alive >= (double)priv->sensor_config.keep_alive_interval / 1000.0 - .2){
		// send keep alive message
		rift_send_keepalive(priv);
		// Update the time of the last keep alive we have sent.
		priv->last_keep_alive = t;
	}

	/* Update any haptics state first */
	check_haptics_state(priv, now, &priv->touch_dev[0]);
	check_haptics_state(priv, now, &priv->touch_dev[1]);
}

static void update_hmd(rift_hmd_t *priv)
{
	unsigned char buffer[FEATURE_BUFFER_SIZE];
	bool got_a_msg = false;
	uint64_t start = ohmd_monotonic_get(priv->ctx);

	update_hmd_outputs(priv, start);

	/* cv1touch: wait briefly for the next radio report so a dedicated update thread wakes as
	 * soon as controller data lands, instead of sleeping a fixed interval between polls. */
	if (priv->radio_handle != NULL && priv->ctx->cv1touch_config.radio_wait_ms > 0) {
		/* then every report queued behind it, so a slow pass does not leave the engine's
		 * controller state further behind each time */
		int wait_ms = priv->ctx->cv1touch_config.radio_wait_ms;
		for (;;) {
			int size = hid_read_timeout(priv->radio_handle, buffer, FEATURE_BUFFER_SIZE, wait_ms);
			uint64_t ts = ohmd_monotonic_get(priv->ctx);
			if (size < 0) {
				priv->io_errors++;
				break;
			}
			if (size == 0)
				break;
			if (buffer[0] == RIFT_RADIO_REPORT_ID) {
				if (priv->ctx->cv1touch_report_cb)
					priv->ctx->cv1touch_report_cb(priv->ctx->cv1touch_report_user, OHMD_CV1TOUCH_REPORT_RADIO, ts, buffer, size);
				handle_rift_radio_report (priv, ts, buffer, size);
			}
			wait_ms = 0;
		}
	}

	// Read all the messages from the device.
	do {
		int size;
		/* FIXME: Collect all HID messages that are pending, then work backward to calculate capture timestamps? */
		uint64_t ts = ohmd_monotonic_get(priv->ctx);

		got_a_msg = false;

		size = hid_read(priv->handle, buffer, FEATURE_BUFFER_SIZE);
		if(size < 0){
			LOGE("error reading from HMD");
			priv->io_errors++;
			break;
		} else if(size > 0) {
			// currently the only message type the hardware supports (I think)
			if(buffer[0] == RIFT_IRQ_SENSORS_DK1 || buffer[0] == RIFT_IRQ_SENSORS_DK2) {
				if (priv->ctx->cv1touch_report_cb)
					priv->ctx->cv1touch_report_cb(priv->ctx->cv1touch_report_user, OHMD_CV1TOUCH_REPORT_HMD, ts, buffer, size);
				handle_tracker_sensor_msg(priv, ts, buffer, size);
				ts = ohmd_monotonic_get(priv->ctx);
				got_a_msg = true;
			}else{
				LOGE("unknown HMD message type: %u", buffer[0]);
			}
		}

		if (priv->radio_handle == NULL)
			continue;

		// Read all the controller messages from the radio device.
		size = hid_read(priv->radio_handle, buffer, FEATURE_BUFFER_SIZE);
		if(size < 0){
			LOGE("error reading from controllers");
			priv->io_errors++;
			break;
		} else if(size > 0) {
			if (buffer[0] == RIFT_RADIO_REPORT_ID) {
				if (priv->ctx->cv1touch_report_cb)
					priv->ctx->cv1touch_report_cb(priv->ctx->cv1touch_report_user, OHMD_CV1TOUCH_REPORT_RADIO, ts, buffer, size);
				handle_rift_radio_report (priv, ts, buffer, size);
				got_a_msg = true;
				ts = ohmd_monotonic_get(priv->ctx);
			}
		}


		/* Don't loop for more than 5ms, or the app doesn't get a chance to draw anything */
		if (got_a_msg) {
			uint64_t now = ohmd_monotonic_get(priv->ctx);
			if (now - start > 5000000)
				break;
		}
	} while(got_a_msg);
}

static void update_device(ohmd_device* device)
{
	rift_device_priv* dev_priv = rift_device_priv_get(device);
	rift_hmd_t *hmd = dev_priv->hmd;

	/* Update on whichever is the lowest open id device */
	if (dev_priv->id == 2) {
		if (hmd->hmd_dev.opened)
			return;
		if (hmd->touch_dev[0].base.opened)
			return;
	}
	else if (dev_priv->id == 1) {
		if (hmd->hmd_dev.opened)
			return;
	}
	update_hmd (dev_priv->hmd);
}

static int getf_hmd(rift_hmd_t *hmd, ohmd_float_value type, float* out)
{
	posef pose = { 0, };

	switch(type){
	case OHMD_DISTORTION_K: {
			for (int i = 0; i < 6; i++) {
				out[i] = hmd->display_info.distortion_k[i];
			}
			break;
		}

	case OHMD_ROTATION_QUAT: {
			if (hmd->tracked_dev) {
				rift_tracked_device_get_view_pose(hmd->tracked_dev, &pose, NULL, NULL, NULL);
			}
			*(quatf*)out = pose.orient;
			break;
		}

	case OHMD_POSITION_VECTOR:
		if (hmd->tracked_dev) {
			rift_tracked_device_get_view_pose(hmd->tracked_dev, &pose, NULL, NULL, NULL);
		}
		*(vec3f*)out = pose.pos;
		break;

	case OHMD_VELOCITY_VECTOR: {
		vec3f vel = { 0, };

		if (hmd->tracked_dev) {
			rift_tracked_device_get_view_pose(hmd->tracked_dev, NULL, &vel, NULL, NULL);
		}
		*(vec3f*)out = vel;
		break;
	}
	case OHMD_ACCELERATION_VECTOR: {
		vec3f accel = { 0, };

		if (hmd->tracked_dev) {
			rift_tracked_device_get_view_pose(hmd->tracked_dev, NULL, NULL, &accel, NULL);
		}
		*(vec3f*)out = accel;
		break;
	}
	case OHMD_ANGULAR_VELOCITY_VECTOR: {
		vec3f ang_vel = { 0, };

		if (hmd->tracked_dev) {
			rift_tracked_device_get_view_pose(hmd->tracked_dev, NULL, NULL, NULL, &ang_vel);
		}
		*(vec3f*)out = ang_vel;
		break;
	}
	case OHMD_CONTROLS_STATE:
		out[0] = (hmd->remote_buttons_state & RIFT_REMOTE_BUTTON_UP) != 0;
		out[1] = (hmd->remote_buttons_state & RIFT_REMOTE_BUTTON_DOWN) != 0;
		out[2] = (hmd->remote_buttons_state & RIFT_REMOTE_BUTTON_LEFT) != 0;
		out[3] = (hmd->remote_buttons_state & RIFT_REMOTE_BUTTON_RIGHT) != 0;
		out[4] = (hmd->remote_buttons_state & RIFT_REMOTE_BUTTON_OK) != 0;
		out[5] = (hmd->remote_buttons_state & RIFT_REMOTE_BUTTON_PLUS) != 0;
		out[6] = (hmd->remote_buttons_state & RIFT_REMOTE_BUTTON_MINUS) != 0;
		out[7] = (hmd->remote_buttons_state & RIFT_REMOTE_BUTTON_OCULUS) != 0;
		out[8] = (hmd->remote_buttons_state & RIFT_REMOTE_BUTTON_BACK) != 0;
		break;

	default:
		ohmd_set_error(hmd->ctx, "invalid type given to getf (%ud)", type);
		return -1;
		break;
	}

	return 0;
}

static int getf_touch_controller(rift_device_priv* dev_priv, ohmd_float_value type, float* out)
{
	rift_touch_controller_t *touch = (rift_touch_controller_t *)(dev_priv);
	posef pose = { 0, };

	switch(type){
	case OHMD_ROTATION_QUAT: {
			if (touch->tracked_dev) {
				rift_tracked_device_get_view_pose(touch->tracked_dev, &pose, NULL, NULL, NULL);
			}
			*(quatf*)out = pose.orient;
			break;
		}
	case OHMD_POSITION_VECTOR:
		if (touch->tracked_dev) {
			rift_tracked_device_get_view_pose(touch->tracked_dev, &pose, NULL, NULL, NULL);
		}
		*(vec3f*)out = pose.pos;
		break;
	case OHMD_VELOCITY_VECTOR: {
		vec3f vel = { 0, };

		if (touch->tracked_dev) {
			rift_tracked_device_get_view_pose(touch->tracked_dev, NULL, &vel, NULL, NULL);
		}
		*(vec3f*)out = vel;
		break;
	}
	case OHMD_ACCELERATION_VECTOR: {
		vec3f accel = { 0, };

		if (touch->tracked_dev) {
			rift_tracked_device_get_view_pose(touch->tracked_dev, NULL, NULL, &accel, NULL);
		}
		*(vec3f*)out = accel;
		break;
	}
	case OHMD_ANGULAR_VELOCITY_VECTOR: {
		vec3f ang_vel = { 0, };

		if (touch->tracked_dev) {
			rift_tracked_device_get_view_pose(touch->tracked_dev, NULL, NULL, NULL, &ang_vel);
		}
		*(vec3f*)out = ang_vel;
		break;
	}
	case OHMD_DISTORTION_K:
		return -1;
	case OHMD_CONTROLS_STATE:
		if (dev_priv->id == 1) { // right control
			out[0] = (touch->buttons & RIFT_TOUCH_CONTROLLER_BUTTON_A) != 0;
			out[1] = (touch->buttons & RIFT_TOUCH_CONTROLLER_BUTTON_B) != 0;
			out[2] = (touch->buttons & RIFT_TOUCH_CONTROLLER_BUTTON_OCULUS) != 0;
			out[3] = (touch->buttons & RIFT_TOUCH_CONTROLLER_BUTTON_STICK) != 0;

		}
		else { // left control, id == 2
			out[0] = (touch->buttons & RIFT_TOUCH_CONTROLLER_BUTTON_X) != 0;
			out[1] = (touch->buttons & RIFT_TOUCH_CONTROLLER_BUTTON_Y) != 0;
			out[2] = (touch->buttons & RIFT_TOUCH_CONTROLLER_BUTTON_MENU) != 0;
			out[3] = (touch->buttons & RIFT_TOUCH_CONTROLLER_BUTTON_STICK) != 0;
		}
		out[4] = touch->trigger;
		out[5] = touch->grip;
		out[6] = touch->stick[0];
		out[7] = touch->stick[1];
		break;
	default:
		ohmd_set_error(dev_priv->hmd->ctx, "invalid type given to getf (%u)", type);
		return -1;
	}

	return 0;
}

static int getf(ohmd_device* device, ohmd_float_value type, float* out)
{
	rift_device_priv* dev_priv = rift_device_priv_get(device);
	if (dev_priv->id == 0)
		return getf_hmd (dev_priv->hmd, type, out);
	else if (dev_priv->id == 1 || dev_priv->id == 2)
		return getf_touch_controller (dev_priv, type, out);

	return -1;
}

static int set_touch_haptics(ohmd_device *device, bool enable, float duration, float frequency, float amplitude)
{
	const float MIN_HAPTIC_DURATION = 0.01;

	rift_device_priv* dev_priv = rift_device_priv_get(device);
	rift_touch_controller_t *touch = (rift_touch_controller_t *)(dev_priv);

	/* Change the haptics state. It will actually be sent to the controllers
	 * in the update loop */
	LOGD("New Haptics event, dev %d duration %f freq %f amplitude %f\n", touch->device_num, duration, frequency, amplitude);
	if (enable) {
			touch->haptic_state.haptics_on = true;
			touch->haptic_state.dirty = true;
			touch->haptic_state.low_freq = (frequency <= 160.0);
			touch->haptic_state.amplitude = roundf(0xff * amplitude);
			touch->haptic_state.duration = OHMD_MAX(duration, MIN_HAPTIC_DURATION);

			touch->haptic_state.end_time = (uint64_t)(-1);
	}
	else if (touch->haptic_state.haptics_on) {
			touch->haptic_state.haptics_on = false;
			touch->haptic_state.dirty = true;
	}

	return 0;
}

static void close_device(ohmd_device* device)
{
	LOGD("closing device");
	rift_device_priv* dev_priv = rift_device_priv_get(device);
	dev_priv->opened = false;
	release_hmd (dev_priv->hmd);
}

void
rift_leds_init (rift_leds *leds, uint8_t num_points)
{
	leds->points = calloc(num_points, sizeof(rift_led));
	leds->num_points = num_points;
	leds->radius_mm = 4 / 1000.0;
}

void
rift_leds_dump (rift_leds *leds, const char *desc)
{
	int i;
	printf ("LED model: %s\n", desc);
	for (i = 0; i < leds->num_points; i++) {
		rift_led *p = leds->points + i;
		printf ("{ .pos = {%f,%f,%f}, .dir={%f,%f,%f}, .pattern=0x%x },\n",
		    p->pos.x, p->pos.y, p->pos.z,
		    p->dir.x, p->dir.y, p->dir.z,
		    p->pattern);
	}
}

void
rift_leds_clear (rift_leds *leds)
{
	free (leds->points);
	leds->points = NULL;
}

/*
 * Obtains the positions and blinking patterns of the IR LEDs from the Rift.
 */
static int rift_get_led_info(rift_hmd_t *priv)
{
	int first_index = -1;
	unsigned char buf[FEATURE_BUFFER_SIZE];
	int size;
	int num_leds = 0;

	//Get LED positions
	while (true) {
		pkt_position_info pos;

		size = get_feature_report(priv, RIFT_CMD_POSITION_INFO, buf);
		if (size <= 0 || !decode_position_info(&pos, buf, size) ||
		    first_index == pos.index) {
			break;
		}

		if (first_index < 0) {
			first_index = pos.index;
			rift_leds_init (&priv->leds, pos.num);
		}

		if (pos.flags == 1) { //reports 0's
			priv->imu.pos.x = (float)pos.pos_x;
			priv->imu.pos.y = (float)pos.pos_y;
			priv->imu.pos.z = (float)pos.pos_z;
			LOGV ("IMU index %d pos x/y/x %d/%d/%d\n", pos.index, pos.pos_x, pos.pos_y, pos.pos_z);
			ovec3f_multiply_scalar(&priv->imu.pos, 1.0/1000000.0, &priv->imu.pos); /* convert to metres */
		} else if (pos.flags == 2) {
			rift_led *led = &priv->leds.points[pos.index];
			led->id = pos.index;
			led->pos.x = (float)pos.pos_x;
			led->pos.y = (float)pos.pos_y;
			led->pos.z = (float)pos.pos_z;
			led->dir.x = (float)pos.dir_x;
			led->dir.y = (float)pos.dir_y;
			led->dir.z = (float)pos.dir_z;
			led->pattern = 0xff;
			ovec3f_multiply_scalar(&led->pos, 1.0/1000000.0, &led->pos); /* convert to metres */
			ovec3f_normalize_me(&led->dir);
			if (pos.index >= num_leds)
				num_leds = pos.index + 1;
			LOGV ("LED index %d pos x/y/x %d/%d/%d\n", pos.index, pos.pos_x, pos.pos_y, pos.pos_z);
		}
	}
	priv->leds.num_points = num_leds;

	// Get LED patterns
	first_index = -1;
	while (true) {
		pkt_led_pattern_report pkt;
		int8_t pattern_length;
		int32_t pattern;

		size = get_feature_report(priv, RIFT_CMD_PATTERN_INFO, buf);
		if (size <= 0 || !decode_led_pattern_info(&pkt, buf, size) ||
		    first_index == pkt.index) {
			break;
		}

		if (first_index < 0) {
			first_index = pkt.index;
			if (priv->leds.num_points != pkt.num) {
				LOGE("LED positions count doesn't match pattern count - got %d patterns for %d LEDs", pkt.num, priv->leds.num_points);
				return -1;
			}
		}
		if (pkt.index >= priv->leds.num_points) {
			LOGE("Invalid LED pattern index %d (%d LEDs)", pkt.index, priv->leds.num_points);
			return -1;
		}

		pattern_length = pkt.pattern_length;
		pattern = pkt.pattern;

		/* pattern_length should be 10 */
		if (pattern_length != 10) {
			LOGE("Rift: Unexpected LED pattern length: %d\n",
				pattern_length);
			return -1;
		}

		LOGV ("LED index %d pattern 0x%08x\n", pkt.index, pkt.pattern);
		/*
		 * pattern should consist of 10 2-bit values that are either
		 * 1 (dark) or 3 (bright).
		 */
		if ((pattern & ~0xaaaaa) != 0x55555) {
			LOGE("Rift: Unexpected pattern: 0x%x", pattern);
			return -1;
		}

		/* Convert into 10 single-bit values 1 -> 0, 3 -> 1 */
		pattern &= 0xaaaaa;
		pattern |= pattern >> 1;
		pattern &= 0x66666;
		pattern |= pattern >> 2;
		pattern &= 0xe1e1e;
		pattern |= pattern >> 4;
		pattern &= 0xe01fe;
		pattern |= pattern >> 8;
		pattern = (pattern >> 1) & 0x3ff;

		priv->leds.points[pkt.index].pattern = pattern;
	}

	/* FIXME: Filter out the LEDs on the back of the headset strap for now, until the positional tracking copes with the
	 * device articulation. At the moment, a camera that sees the back LEDs will extract the wrong position.
	 * Headset LEDs have a Z < -100mm */
	{
		int in_index, out_index = 0;
		for (in_index = 0; in_index < priv->leds.num_points; in_index++) {
			rift_led *led = &priv->leds.points[in_index];
			if (led->pos.z < -0.1) {
				printf ("Dropping headband LED { .pos = {%f,%f,%f}, .dir={%f,%f,%f}, .pattern=0x%x },\n",
					led->pos.x, led->pos.y, led->pos.z,
					led->dir.x, led->dir.y, led->dir.z,
					led->pattern);
				continue;
			}
			if (in_index != out_index)
				priv->leds.points[out_index] = *led;
			out_index++;
		}
		priv->leds.num_points = out_index;
	}

	rift_leds_dump (&priv->leds, "HMD LEDs");

	return 0;
}

/*
 * Sends a tracking report to enable the IR tracking LEDs.
 */
static int rift_send_tracking_config(rift_hmd_t *rift, bool blink,
    uint16_t exposure_us, uint16_t period_us)
{
	pkt_tracking_config tracking_config = { 0, };
	unsigned char buf[FEATURE_BUFFER_SIZE];
	int size;

	tracking_config.vsync_offset = RIFT_TRACKING_VSYNC_OFFSET;
	tracking_config.duty_cycle = RIFT_TRACKING_DUTY_CYCLE;
	tracking_config.exposure_us = exposure_us;
	tracking_config.period_us = period_us;

	if (blink) {
		tracking_config.pattern = 0;
		tracking_config.flags = RIFT_TRACKING_ENABLE |
		                        RIFT_TRACKING_USE_CARRIER |
		                        RIFT_TRACKING_AUTO_INCREMENT;
	} else {
		tracking_config.pattern = 0xff;
		tracking_config.flags = RIFT_TRACKING_ENABLE |
		                        RIFT_TRACKING_USE_CARRIER;
	}

	size = encode_tracking_config(buf, &tracking_config);
	if (send_feature_report(rift, buf, size) == -1) {
		LOGE("Error sending LED tracking config");
	  return -1;
	}

	return 0;
}

static void init_touch_device(rift_touch_controller_t *touch, int id, int device_num)
{
	ohmd_device *ohmd_dev = &touch->base.base;

	touch->base.id = id;
	touch->device_num = device_num;
	touch->time_valid = false;

	ohmd_set_default_device_properties(&ohmd_dev->properties);

	ohmd_dev->properties.control_count = 8;

	if (id == 1) {
		ohmd_dev->properties.controls_hints[0] = OHMD_BUTTON_A;
		ohmd_dev->properties.controls_hints[1] = OHMD_BUTTON_B;
		ohmd_dev->properties.controls_hints[2] = OHMD_HOME; // Oculus button
		ohmd_dev->properties.controls_hints[3] = OHMD_ANALOG_PRESS; // stick button
	} else {
		ohmd_dev->properties.controls_hints[0] = OHMD_BUTTON_X;
		ohmd_dev->properties.controls_hints[1] = OHMD_BUTTON_Y;
		ohmd_dev->properties.controls_hints[2] = OHMD_MENU;
		ohmd_dev->properties.controls_hints[3] = OHMD_ANALOG_PRESS; // stick button
	}
	ohmd_dev->properties.controls_types[0] = OHMD_DIGITAL;
	ohmd_dev->properties.controls_types[1] = OHMD_DIGITAL;
	ohmd_dev->properties.controls_types[2] = OHMD_DIGITAL;
	ohmd_dev->properties.controls_types[3] = OHMD_DIGITAL;

	ohmd_dev->properties.controls_hints[4] = OHMD_TRIGGER;
	ohmd_dev->properties.controls_hints[5] = OHMD_SQUEEZE;
	ohmd_dev->properties.controls_hints[6] = OHMD_ANALOG_X;
	ohmd_dev->properties.controls_hints[7] = OHMD_ANALOG_Y;
	ohmd_dev->properties.controls_types[4] = OHMD_ANALOG;
	ohmd_dev->properties.controls_types[5] = OHMD_ANALOG;
	ohmd_dev->properties.controls_types[6] = OHMD_ANALOG;
	ohmd_dev->properties.controls_types[7] = OHMD_ANALOG;
}

static rift_hmd_t *open_hmd(ohmd_driver* driver, ohmd_device_desc* desc)
{
	rift_hmd_t* priv = ohmd_alloc(driver->ctx, sizeof(rift_hmd_t));
	rift_device_priv *hmd_dev;
	if(!priv)
		goto cleanup;

	hmd_dev = &priv->hmd_dev;

	priv->revision = desc->revision;
	priv->use_count = 1;
	priv->ctx = driver->ctx;

	priv->have_imu_timestamp = false;
	priv->last_imu_timestamp = -1;

	// Open the HID device
	priv->handle = hid_open_path(desc->path);

	if(!priv->handle) {
		char* path = _hid_to_unix_path(desc->path);
		ohmd_set_error(driver->ctx, "Could not open %s.\n"
		                            "Check your permissions: "
		                            UDEV_WIKI_URL, path);
		free(path);
		goto cleanup;
	}

	if(hid_set_nonblocking(priv->handle, 1) == -1){
		ohmd_set_error(driver->ctx, "failed to set non-blocking on device");
		goto cleanup;
	}

	/* Send a keepalive msg first, to wake up the headset */
	rift_send_keepalive(priv);
	// Update the time of the last keep alive we have sent.
	priv->last_keep_alive = ohmd_get_tick();

	/* For the CV1, try and open the radio HID device */
	if (desc->revision == REV_CV1) {
		priv->radio_handle = open_hid_dev (driver->ctx, OCULUS_VR_INC_ID, RIFT_CV1_PID, 1);
		if (priv->radio_handle == NULL)
			goto cleanup;
		if(hid_set_nonblocking(priv->radio_handle, 1) == -1){
			ohmd_set_error(driver->ctx, "Failed to set non-blocking on radio device");
			goto cleanup;
		}
		rift_hmd_radio_init(&priv->radio, priv->radio_handle);
	}

	unsigned char buf[FEATURE_BUFFER_SIZE];

	int size;

	// Read and decode the sensor range
	size = get_feature_report(priv, RIFT_CMD_RANGE, buf);
	decode_sensor_range(&priv->sensor_range, buf, size);
	dump_packet_sensor_range(&priv->sensor_range);

	size = get_feature_report(priv, RIFT_CMD_IMU_CALIBRATION, buf);
	decode_imu_calibration(&priv->imu_calibration, buf, size);
	dump_packet_imu_calibration(&priv->imu_calibration);

	// Read and decode display information
	size = get_feature_report(priv, RIFT_CMD_DISPLAY_INFO, buf);
	decode_sensor_display_info(&priv->display_info, buf, size);
	dump_packet_sensor_display_info(&priv->display_info);

	// Read and decode the sensor config
	size = get_feature_report(priv, RIFT_CMD_SENSOR_CONFIG, buf);
	decode_sensor_config(&priv->sensor_config, buf, size);
	dump_packet_sensor_config(&priv->sensor_config);

	// if the sensor has display info data, use HMD coordinate frame
	priv->coordinate_frame = priv->display_info.distortion_type != RIFT_DT_NONE ? RIFT_CF_HMD : RIFT_CF_SENSOR;

	// enable calibration, but these don't seem to stick. We check later if
	// we need to do it manually
	SETFLAG(priv->sensor_config.flags, RIFT_SCF_USE_CALIBRATION, 1);
	SETFLAG(priv->sensor_config.flags, RIFT_SCF_AUTO_CALIBRATION, 1);

	// apply sensor config
	set_coordinate_frame(priv, priv->coordinate_frame);

	// Turn the screens on
	if (desc->revision == REV_CV1)
	{
		size = encode_enable_components(buf, true, true, !driver->ctx->cv1touch_config.hmd_leds_off);
		if (send_feature_report(priv, buf, size) == -1)
			LOGE("error turning the screens on");

		rift_send_tracking_config (priv, false, RIFT_TRACKING_EXPOSURE_US_CV1,
				RIFT_TRACKING_PERIOD_US_CV1);

		/* Read the radio ID for CV1 to enable camera sensor sync */
		rift_hmd_radio_get_address(priv->handle, priv->radio_address);
	}
	else if (desc->revision == REV_DK2)
	{
		rift_send_tracking_config (priv, false, RIFT_TRACKING_EXPOSURE_US_DK2,
				RIFT_TRACKING_PERIOD_US_DK2);
	}

	/* We only need the LED info if we have a sensor to observe them with,
	   so we could skip this */
	if (rift_get_led_info (priv) < 0) {
		ohmd_set_error(driver->ctx, "failed to read LED info from device");
		goto cleanup;
	}

	// update sensor settings with new keep alive value
	// (which will have been ignored in favor of the default 1000 ms one)
	size = get_feature_report(priv, RIFT_CMD_SENSOR_CONFIG, buf);
	decode_sensor_config(&priv->sensor_config, buf, size);
	dump_packet_sensor_config(&priv->sensor_config);

	// Set default device properties
	ohmd_set_default_device_properties(&hmd_dev->base.properties);

	// Set device properties
	hmd_dev->base.properties.hsize = priv->display_info.h_screen_size;
	hmd_dev->base.properties.vsize = priv->display_info.v_screen_size;
	hmd_dev->base.properties.hres = priv->display_info.h_resolution;
	hmd_dev->base.properties.vres = priv->display_info.v_resolution;
	hmd_dev->base.properties.lens_sep = priv->display_info.lens_separation;
	hmd_dev->base.properties.lens_vpos = priv->display_info.v_center;
	hmd_dev->base.properties.ratio = ((float)priv->display_info.h_resolution / (float)priv->display_info.v_resolution) / 2.0f;

	if (desc->revision == REV_CV1) {
		/* On the CV1, add some control mappings for the simple Oculus remote control buttons */
		hmd_dev->base.properties.control_count = 9;
		hmd_dev->base.properties.controls_hints[0] = OHMD_BUTTON_Y; // UP
		hmd_dev->base.properties.controls_hints[1] = OHMD_BUTTON_A; // DOWN
		hmd_dev->base.properties.controls_hints[2] = OHMD_BUTTON_X; // LEFT
		hmd_dev->base.properties.controls_hints[3] = OHMD_BUTTON_B; // RIGHT
		hmd_dev->base.properties.controls_hints[4] = OHMD_GENERIC; // OK button
		hmd_dev->base.properties.controls_hints[5] = OHMD_VOLUME_PLUS;
		hmd_dev->base.properties.controls_hints[6] = OHMD_VOLUME_MINUS;
		hmd_dev->base.properties.controls_hints[7] = OHMD_MENU; // OCULUS button
		hmd_dev->base.properties.controls_hints[8] = OHMD_HOME; // Back button

		hmd_dev->base.properties.controls_types[0] = OHMD_DIGITAL;
		hmd_dev->base.properties.controls_types[1] = OHMD_DIGITAL;
		hmd_dev->base.properties.controls_types[2] = OHMD_DIGITAL;
		hmd_dev->base.properties.controls_types[3] = OHMD_DIGITAL;
		hmd_dev->base.properties.controls_types[4] = OHMD_DIGITAL;
		hmd_dev->base.properties.controls_types[5] = OHMD_DIGITAL;
		hmd_dev->base.properties.controls_types[6] = OHMD_DIGITAL;
		hmd_dev->base.properties.controls_types[7] = OHMD_DIGITAL;
		hmd_dev->base.properties.controls_types[8] = OHMD_DIGITAL;

		/* And initialise state trackers for the 2 touch controllers */
		init_touch_device (&priv->touch_dev[0], 1, RIFT_TOUCH_CONTROLLER_RIGHT);
		init_touch_device (&priv->touch_dev[1], 2, RIFT_TOUCH_CONTROLLER_LEFT);
	}

	//setup generic distortion coeffs, from hand-calibration
	switch (desc->revision) {
		case REV_DK2:
			ohmd_set_universal_distortion_k(&(hmd_dev->base.properties), 0.247, -0.145, 0.103, 0.795);
			ohmd_set_universal_aberration_k(&(hmd_dev->base.properties), 0.985, 1.000, 1.015);
			break;
		case REV_DK1:
			ohmd_set_universal_distortion_k(&(hmd_dev->base.properties), 1.003, -1.005, 0.403, 0.599);
			ohmd_set_universal_aberration_k(&(hmd_dev->base.properties), 0.985, 1.000, 1.015);
			break;
		case REV_CV1:
			ohmd_set_universal_distortion_k(&(hmd_dev->base.properties), 0.269, -0.25, 0.178, 0.803);
			ohmd_set_universal_aberration_k(&(hmd_dev->base.properties), 0.9992107, 1.0, 1.0120361);
			/* CV1 reports IPD, but not lens center, at least not anywhere I could find, so use the manually measured value of 0.054 */
			priv->display_info.lens_separation = 0.054;
			hmd_dev->base.properties.lens_sep = priv->display_info.lens_separation;
		default:
			break;
	}

	// calculate projection eye projection matrices from the device properties
	//ohmd_calc_default_proj_matrices(&hmd_dev->base.properties);
	float l,r,t,b,n,f;
	// left eye screen bounds
	l = -1.0f * (priv->display_info.h_screen_size/2 - priv->display_info.lens_separation/2);
	r = priv->display_info.lens_separation/2;
	t = priv->display_info.v_screen_size - priv->display_info.v_center;
	b = -1.0f * priv->display_info.v_center;
	n = priv->display_info.eye_to_screen_distance[0];
	f = n*10e6;
	//LOGD("l: %0.3f, r: %0.3f, b: %0.3f, t: %0.3f, n: %0.3f, f: %0.3f", l,r,b,t,n,f);
	/* eye separation is handled by IPD in the Modelview matrix */
	omat4x4f_init_frustum(&hmd_dev->base.properties.proj_left, l, r, b, t, n, f);
	//right eye screen bounds
	l = -1.0f * priv->display_info.lens_separation/2;
	r = priv->display_info.h_screen_size/2 - priv->display_info.lens_separation/2;
	n = priv->display_info.eye_to_screen_distance[1];
	f = n*10e6;
	//LOGD("l: %0.3f, r: %0.3f, b: %0.3f, t: %0.3f, n: %0.3f, f: %0.3f", l,r,b,t,n,f);
	/* eye separation is handled by IPD in the Modelview matrix */
	omat4x4f_init_frustum(&hmd_dev->base.properties.proj_right, l, r, b, t, n, f);

	hmd_dev->base.properties.fov = 2 * atan2f(
			priv->display_info.h_screen_size/2 - priv->display_info.lens_separation/2,
			priv->display_info.eye_to_screen_distance[0]);
	hmd_dev->id = 0;
	hmd_dev->hmd = priv;

	// Find and attach Rift sensors if available
	priv->tracker_ctx = rift_tracker_new (driver->ctx, priv->radio_address);
	if (priv->tracker_ctx == NULL) {
		ohmd_set_error(driver->ctx, "Failed to create Rift sensor tracker");
		goto cleanup;
	}

	/* Configure the rift device in the tracker */
	quatf imu_orient = {{ 0.0, 0.0, 0.0, 1.0 }};
	posef imu_pose;
	oposef_init(&imu_pose, &priv->imu.pos, &imu_orient);

	/* The HMD LED model is rotated 180 deg around Y */
	quatf model_orient = {{ 0.0, 1.0, 0.0, 0.0 }};
	vec3f model_pos = {{ 0.0, 0.0, 0.0 }};
	posef model_pose;
	oposef_init(&model_pose, &model_pos, &model_orient);

	rift_tracked_device_imu_calibration imu_calibration;
	int i;

	imu_calibration.accel_offset = priv->imu_calibration.accel_offset;
	imu_calibration.gyro_offset = priv->imu_calibration.gyro_offset;
	for (i = 0; i < 9; i++) {
		imu_calibration.accel_matrix[i] = priv->imu_calibration.accel_matrix[i/3][i%3];
		imu_calibration.gyro_matrix[i] = priv->imu_calibration.gyro_matrix[i/3][i%3];
	}
	/* cv1touch: by default the headset is the controllers' radio and the sensors' exposure-sync
	 * source only; tracking it would require it to be in view. */
	if (driver->ctx->cv1touch_config.hmd_tracking)
		priv->tracked_dev = rift_tracker_add_device (priv->tracker_ctx, 0, &imu_pose, &model_pose, &priv->leds, &imu_calibration);

	return priv;

cleanup:
	if (priv)
		close_hmd (priv);
	return NULL;
}

static void close_hmd(rift_hmd_t *hmd)
{
	if (hmd->tracker_ctx)
		rift_tracker_free(hmd->tracker_ctx);
	if (hmd->radio_handle) {
		rift_hmd_radio_clear(&hmd->radio);
		hid_close(hmd->radio_handle);
	}
	hid_close(hmd->handle);

	rift_touch_clear_calibration (&hmd->touch_dev[0].calibration);
	rift_touch_clear_calibration (&hmd->touch_dev[1].calibration);
	rift_leds_clear (&hmd->leds);
	free(hmd);
}

/* FIXME: This opens the first device that matches the
 * requested VID/PID/interface, which works fine if there's
 * 1 rift attached. To support multiple rift, we need to
 * match parent USB devices like ouvrt does */
static hid_device* open_hid_dev(ohmd_context* ctx,
		int vid, int pid, int iface_num)
{
	struct hid_device_info* devs = hid_enumerate(vid, pid);
	struct hid_device_info* cur_dev = devs;
	hid_device *handle = NULL;

	if(devs == NULL)
		return NULL;

	while (cur_dev) {
		if (cur_dev->interface_number == iface_num) {
			handle = hid_open_path(cur_dev->path);
			if (handle)
				break;
			else {
				char* path = _hid_to_unix_path(cur_dev->path);
				ohmd_set_error(ctx, "Could not open %s.\n"
				               "Check your permissions: "
				               UDEV_WIKI_URL, path);
				free(path);
			}
		}
		cur_dev = cur_dev->next;
	}

	hid_free_enumeration(devs);
	return handle;
}

static ohmd_device* open_device(ohmd_driver* driver, ohmd_device_desc* desc)
{
	rift_device_priv *dev = NULL;
	rift_hmd_t *hmd = find_hmd(desc->path);
	bool is_controller = false;

	if (hmd == NULL) {
		hmd = open_hmd (driver, desc);
		if (hmd == NULL)
			return NULL;
		push_hmd (hmd, desc->path);
	}

	if (desc->id == 0)
		dev = &hmd->hmd_dev;
	else if (desc->id == 1) {
		dev = &hmd->touch_dev[0].base;
		is_controller = true;
	} else if (desc->id == 2) {
		dev = &hmd->touch_dev[1].base;
		is_controller = true;
	}
	else {
		LOGE ("Invalid device description passed to open_device()");
		release_hmd(hmd);
		return NULL;
	}

	// set up device callbacks
	dev->hmd = hmd;
	dev->id = desc->id;
	dev->opened = true;

	dev->base.update = update_device;
	dev->base.close = close_device;
	dev->base.getf = getf;

	if (is_controller)
		dev->base.set_haptics = set_touch_haptics;

	return &dev->base;
}

#define RIFT_ID_COUNT 5

static void get_device_list(ohmd_driver* driver, ohmd_device_list* list)
{
	// enumerate HID devices and add any Rifts found to the device list

	rift_devices rd[RIFT_ID_COUNT] = {
		{ "Rift (DK1)", OCULUS_VR_INC_ID, 0x0001,	-1, REV_DK1 },
		{ "Rift (DK2)", OCULUS_VR_INC_ID, 0x0021,	-1, REV_DK2 },
		{ "Rift (DK2)", OCULUS_VR_INC_ID, 0x2021,	-1, REV_DK2 },
		{ "Rift (CV1)", OCULUS_VR_INC_ID, RIFT_CV1_PID,	 0, REV_CV1 },

		{ "GearVR (Gen1)", SAMSUNG_ELECTRONICS_CO_ID, 0xa500,	 0, REV_GEARVR_GEN1 },
	};

	for(int i = 0; i < RIFT_ID_COUNT; i++){
		struct hid_device_info* devs = hid_enumerate(rd[i].company, rd[i].id);
		struct hid_device_info* cur_dev = devs;

		if(devs == NULL)
			continue;

		while (cur_dev) {
			// We need to check the manufacturer because other companies (eg: VR-Tek)
			// are reusing the Oculus DK1 USB ID for their own HMDs
			if(ohmd_wstring_match(cur_dev->manufacturer_string, L"Oculus VR, Inc.") &&
			   (rd[i].iface == -1 || cur_dev->interface_number == rd[i].iface)) {
				int id = 0;
				ohmd_device_desc* desc = &list->devices[list->num_devices++];

				strcpy(desc->driver, "OpenHMD Rift Driver");
				strcpy(desc->vendor, "Oculus VR, Inc.");
				strcpy(desc->product, rd[i].name);

				desc->revision = rd[i].rev;
		
				desc->device_class = OHMD_DEVICE_CLASS_HMD;
				desc->device_flags = OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING;

				strcpy(desc->path, cur_dev->path);

				desc->driver_ptr = driver;
				desc->id = id++;

				/* For CV1, publish devices for the remote and touch controllers */
				if (desc->revision == REV_CV1) {
					// Remote Control
					desc = &list->devices[list->num_devices++];
					desc->revision = rd[i].rev;
					desc->driver_ptr = driver;
					desc->device_class = OHMD_DEVICE_CLASS_CONTROLLER;
					desc->device_flags = 0;
					desc->id = 0;

					strcpy(desc->driver, "OpenHMD Rift Driver");
					strcpy(desc->vendor, "Oculus VR, Inc.");
					sprintf(desc->product, "%s: Remote Control", rd[i].name);

					strcpy(desc->path, cur_dev->path);

					//Controller 0 (right)
					desc = &list->devices[list->num_devices++];
					desc->revision = rd[i].rev;

					strcpy(desc->driver, "OpenHMD Rift Driver");
					strcpy(desc->vendor, "Oculus VR, Inc.");
					sprintf(desc->product, "%s: Right Controller", rd[i].name);

					strcpy(desc->path, cur_dev->path);

					desc->device_flags =
						OHMD_DEVICE_FLAGS_POSITIONAL_TRACKING |
						OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING |
						OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER |
						OHMD_DEVICE_FLAGS_HAPTIC_FEEDBACK;

					desc->device_class = OHMD_DEVICE_CLASS_CONTROLLER;
					desc->driver_ptr = driver;
					desc->id = id++;

					// Controller 1 (left)
					desc = &list->devices[list->num_devices++];
					desc->revision = rd[i].rev;

					strcpy(desc->driver, "OpenHMD Rift Driver");
					strcpy(desc->vendor, "Oculus VR, Inc.");
					sprintf(desc->product, "%s: Left Controller", rd[i].name);

					strcpy(desc->path, cur_dev->path);

					desc->device_flags =
						OHMD_DEVICE_FLAGS_POSITIONAL_TRACKING |
						OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING |
						OHMD_DEVICE_FLAGS_LEFT_CONTROLLER |
						OHMD_DEVICE_FLAGS_HAPTIC_FEEDBACK;

					desc->device_class = OHMD_DEVICE_CLASS_CONTROLLER;
					desc->driver_ptr = driver;
					desc->id = id++;
				}
			}
			cur_dev = cur_dev->next;
		}

		hid_free_enumeration(devs);
	}
}

static void destroy_driver(ohmd_driver* drv)
{
	LOGD("shutting down driver");

	hid_exit();
	free(drv);

	ohmd_toggle_ovr_service(1); //re-enable OVRService if previously running
}

ohmd_driver* ohmd_create_oculus_rift_drv(ohmd_context* ctx)
{
	ohmd_driver* drv = ohmd_alloc(ctx, sizeof(ohmd_driver));
	if(drv == NULL)
		return NULL;

	ohmd_toggle_ovr_service(0); //disable OVRService if running

	drv->get_device_list = get_device_list;
	drv->open_device = open_device;
	drv->destroy = destroy_driver;
	drv->ctx = ctx;

	return drv;
}

OHMD_APIENTRYDLL uint32_t OHMD_APIENTRY ohmd_cv1touch_get_io_errors(ohmd_device *device)
{
	if (device == NULL || device->getf != getf)
		return 0;
	return rift_device_priv_get(device)->hmd->io_errors;
}

static void cv1touch_copy_quat(float *out, const quatf *q)
{
	out[0] = q->x; out[1] = q->y; out[2] = q->z; out[3] = q->w;
}

static void cv1touch_copy_vec(float *out, const vec3f *v)
{
	out[0] = v->x; out[1] = v->y; out[2] = v->z;
}

/* cv1touch: the radio thread's decoder. It reads the radio through a hidraw handle of its own
 * (each reader gets every report) and decodes on its own copy of each controller's state, so it
 * never waits on the engine or touches its state; only a controller's factory calibration comes
 * from the engine, once the engine has fetched it. Its device clocks are its own: they start at
 * its first sample, not the engine's, and only its samples should be timed against them. */
struct ohmd_cv1touch_radio_decoder {
	rift_hmd_t *hmd;
	rift_touch_controller_t touch[2]; /* as hmd->touch_dev: [0] right, [1] left */
};

OHMD_APIENTRYDLL ohmd_cv1touch_radio_decoder *OHMD_APIENTRY ohmd_cv1touch_radio_decoder_new(ohmd_device *device)
{
	if (device == NULL || device->getf != getf)
		return NULL;
	ohmd_cv1touch_radio_decoder *dec = calloc(1, sizeof *dec);
	if (dec == NULL)
		return NULL;
	dec->hmd = rift_device_priv_get(device)->hmd;
	for (int i = 0; i < 2; i++)
		dec->touch[i].base.id = dec->hmd->touch_dev[i].base.id;
	return dec;
}

OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_cv1touch_radio_decoder_free(ohmd_cv1touch_radio_decoder *dec)
{
	free(dec);
}

OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_radio_decode(ohmd_cv1touch_radio_decoder *dec, const unsigned char *buf, int size,
		uint64_t received_ns, ohmd_cv1touch_controller *out)
{
	pkt_rift_radio_report r;
	if (dec == NULL || size <= 0 || buf[0] != RIFT_RADIO_REPORT_ID || !decode_rift_radio_report(&r, buf, size))
		return 0;
	int n = 0;
	for (int i = 0; i < 2; i++) {
		pkt_rift_radio_message *msg = &r.message[i];
		int t;
		if (!msg->valid)
			continue;
		if (msg->device_type == RIFT_TOUCH_CONTROLLER_RIGHT)
			t = 0;
		else if (msg->device_type == RIFT_TOUCH_CONTROLLER_LEFT)
			t = 1;
		else
			continue;
		rift_touch_controller_t *touch = &dec->touch[t];
		if (!touch->have_calibration) {
			rift_touch_controller_t *engine = &dec->hmd->touch_dev[t];
			if (!__atomic_load_n(&engine->have_calibration, __ATOMIC_ACQUIRE))
				continue;
			touch->calibration = engine->calibration;
			touch->have_calibration = true;
		}
		if (touch_decode_sample(touch, msg, received_ns, &out[n]))
			n++;
	}
	return n;
}

/* Factory calibration hash is stable across reconnects; no personal serial is exposed.
 * Call on the engine thread, like get_controller. Never read the shared radio scratch hash. */
OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_get_controller_identity(ohmd_device *device, char *out, int size)
{
	if (out == NULL || size < 33)
		return -1;
	out[0] = '\0';
	if (device == NULL || device->getf != getf)
		return -1;
	rift_device_priv *priv = rift_device_priv_get(device);
	if (priv->id != 1 && priv->id != 2)
		return -1;
	rift_touch_controller_t *touch = (rift_touch_controller_t *)priv;
	if (!__atomic_load_n(&touch->have_calibration, __ATOMIC_ACQUIRE))
		return -1;
	static const char hex[] = "0123456789abcdef";
	for (int i = 0; i < 16; i++) {
		out[2*i] = hex[touch->cv1touch_calibration_hash[i] >> 4];
		out[2*i+1] = hex[touch->cv1touch_calibration_hash[i] & 15];
	}
	out[32] = '\0';
	return 0;
}

OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_get_controller(ohmd_device *device, ohmd_cv1touch_controller *out)
{
	if (device == NULL || device->getf != getf)
		return -1;
	rift_device_priv *dev_priv = rift_device_priv_get(device);
	if (dev_priv->id != 1 && dev_priv->id != 2)
		return -1;
	rift_touch_controller_t *touch = (rift_touch_controller_t *) dev_priv;

	memset(out, 0, sizeof *out);
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
	out->received_host_ns = touch->cv1touch_received_ns;
	out->filter_ns = touch->cv1touch_filter_ns;
	out->device_id = dev_priv->id;
	cv1touch_copy_vec(out->imu_accel, &touch->cv1touch_accel);
	cv1touch_copy_vec(out->imu_gyro, &touch->cv1touch_gyro);
	if (touch->tracked_dev == NULL)
		return 0;

	rift_tracked_device_export ex;
	rift_tracked_device_get_export(touch->tracked_dev, &ex);
	out->calibrated = true;
	out->device_time_ns = ex.device_time_ns;
	out->last_pose_fix_device_ns = ex.last_observed_pose_ts;
	out->last_orient_fix_device_ns = ex.last_observed_orient_ts;
	cv1touch_copy_quat(out->view_orient, &ex.view_pose.orient);
	cv1touch_copy_vec(out->view_pos, &ex.view_pose.pos);
	cv1touch_copy_vec(out->view_lin_vel, &ex.view_lin_vel);
	cv1touch_copy_vec(out->view_ang_vel, &ex.view_ang_vel);
	cv1touch_copy_vec(out->view_lin_acc, &ex.view_lin_accel);
	cv1touch_copy_quat(out->raw_orient, &ex.raw_pose.orient);
	cv1touch_copy_vec(out->raw_pos, &ex.raw_pose.pos);
	cv1touch_copy_vec(out->pos_stddev, &ex.pos_error);
	cv1touch_copy_vec(out->rot_stddev, &ex.rot_error);
	cv1touch_copy_vec(out->gyro_raw, &ex.gyro_raw);
	cv1touch_copy_vec(out->gyro_bias, &ex.gyro_bias);
	return 0;
}

OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_get_led_model(ohmd_device *device, ohmd_cv1touch_led *out, int max)
{
	if (device == NULL || device->getf != getf || out == NULL || max <= 0)
		return -1;
	rift_device_priv *dev_priv = rift_device_priv_get(device);
	if (dev_priv->id != 1 && dev_priv->id != 2)
		return -1;
	rift_touch_controller_t *touch = (rift_touch_controller_t *) dev_priv;
	if (!touch->have_calibration)
		return 0;
	int n = touch->calibration.leds.num_points < max ? touch->calibration.leds.num_points : max;
	for (int i = 0; i < n; i++) {
		const rift_led *led = touch->calibration.leds.points + i;
		cv1touch_copy_vec(out[i].pos, &led->pos);
		cv1touch_copy_vec(out[i].dir, &led->dir);
	}
	return n;
}

OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_get_sensor_count(ohmd_device *device)
{
	if (device == NULL || device->getf != getf)
		return 0;
	rift_hmd_t *hmd = rift_device_priv_get(device)->hmd;
	return hmd->tracker_ctx ? rift_tracker_get_sensor_count(hmd->tracker_ctx) : 0;
}

OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_get_sensor(ohmd_device *device, int index, ohmd_cv1touch_sensor *out)
{
	if (device == NULL || device->getf != getf)
		return -1;
	rift_hmd_t *hmd = rift_device_priv_get(device)->hmd;
	if (hmd->tracker_ctx == NULL)
		return -1;
	return rift_tracker_get_sensor_info(hmd->tracker_ctx, index, out);
}
