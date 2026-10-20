/*
 * Oculus Rift CV1 Radio
 * Copyright 2016 Philipp Zabel
 * Copyright 2019 Jan Schmidt
 * SPDX-License-Identifier:	BSL-1.0
 */
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <float.h>
#include <math.h>

#include "rift-hmd-radio.h"
#include "../ext_deps/nxjson.h"

static int get_feature_report(hid_device *handle, rift_sensor_feature_cmd cmd, unsigned char* buf)
{
	memset(buf, 0, FEATURE_BUFFER_SIZE);
	buf[0] = (unsigned char)cmd;
	int ret = hid_get_feature_report(handle, buf, FEATURE_BUFFER_SIZE);
	return ret < 0 ? -EIO : ret;
}

static int send_feature_report(hid_device *handle, unsigned char* buf, int length)
{
	int ret = hid_send_feature_report(handle, buf, length);
	return ret < 0 ? -EIO : ret;
}

static int rift_hmd_radio_get_cmd_response(hid_device *handle, bool wait)
{
	unsigned char buffer[FEATURE_BUFFER_SIZE];
	int ret_size;

	do {
		ret_size = get_feature_report(handle, RIFT_CMD_RADIO_CONTROL, buffer);
		if (ret_size < 4) {
			/* Asynchronous calibration/haptics callers report failure transitions. */
			if (wait)
				LOGE("HMD radio command failed - response too small");
			return -EIO;
		}

		/* If this isn't a blocking wait, return EAGAIN */
		if (!wait && (buffer[3] & 0x80)) {
			return -EINPROGRESS;
		}
	} while (buffer[3] & 0x80);

	/* 0x08 means the device isn't responding */
	if (buffer[3] & 0x08)
		return -ETIMEDOUT;

	return 0;
}

static int rift_hmd_radio_send_cmd(hid_device *handle, uint8_t a, uint8_t b, uint8_t c)
{
	unsigned char buffer[FEATURE_BUFFER_SIZE];
	int cmd_size = encode_radio_control_cmd(buffer, a, b, c);
	int ret_size;

	ret_size = send_feature_report(handle, buffer, cmd_size);
	return ret_size;

}

/* There is no validated transaction tag or abort command. Drain an abandoned
 * command before sending anything else; sending a replacement and then resending
 * it would leave a second result to be mistaken for a later command. A radio that
 * stays busy forever cannot safely be reused. Space polls by 100 ms across both
 * hands and haptics: at most ten status reads per second while stuck. Once it
 * finishes, reuse waits for the next poll. A synchronous HID request can still block. */
static int rift_radio_drain_abandoned(ohmd_context *ctx, rift_hmd_radio_state *radio)
{
	if (!radio->discard_reply)
		return 0;
	const uint64_t now = ohmd_monotonic_get(ctx);
	const uint64_t second = ohmd_monotonic_per_sec(ctx);
	if (!radio->drain_next_poll)
		radio->drain_started = now;
	if (now < radio->drain_next_poll)
		return -EBUSY;
	int ret = rift_hmd_radio_get_cmd_response(radio->handle, false);
	if (ret == 0 || ret == -ETIMEDOUT) {
		unsigned char discarded[FEATURE_BUFFER_SIZE];
		if (get_feature_report(radio->handle, RIFT_CMD_RADIO_READ_DATA, discarded) >= 7) {
			radio->discard_reply = false;
			radio->drain_next_poll = 0;
			radio->drain_warned = false;
			return 0;
		}
		ret = -EIO;
	}
	/* A slow or failed HID read must still leave a gap before the next poll. */
	const uint64_t completed = ohmd_monotonic_get(ctx);
	radio->drain_next_poll = completed + second / 10;
	if (!radio->drain_warned && completed - radio->drain_started >= 10 * second) {
		LOGW("The radio is still finishing an earlier request; controllers cannot load until it does.");
		radio->drain_warned = true;
	}
	return ret == -EINPROGRESS ? -EBUSY : ret;
}

/* A command whose status could not be read may be running or finished. Give up on it, but
 * drain its result before anything else is sent: the next command must never take it as its own. */
static void rift_radio_abandon_command(rift_hmd_radio_state *radio)
{
	if (radio->command_result_pending)
		radio->discard_reply = true;
	radio->command_result_pending = false;
}

static int rift_radio_command_complete(rift_hmd_radio_state *radio)
{
	int ret = rift_hmd_radio_get_cmd_response(radio->handle, false);
	/* An unread/invalid status is not evidence that this command completed. */
	radio->command_result_pending = ret != 0 && ret != -ETIMEDOUT;
	return ret;
}

static int rift_radio_read_flash(rift_hmd_radio_state *radio, uint8_t device_type,
				uint16_t offset, uint16_t length, uint8_t *flash_data)
{
	int ret;
	unsigned char buffer[FEATURE_BUFFER_SIZE];
	int cmd_size;

	if (!radio->command_result_pending) {
		LOGV("Reading FW calibration block for controller %d at offset %u", device_type, offset);
		cmd_size = encode_radio_data_read_cmd(buffer, offset, length);
		if ((ret = send_feature_report(radio->handle, buffer, cmd_size)) < 0)
			goto done;

		if ((ret = rift_hmd_radio_send_cmd (radio->handle, 0x03, RIFT_HMD_RADIO_READ_FLASH_CONTROL,
		    device_type)) < 0)
			goto done;
	}

	if ((ret = rift_radio_command_complete(radio)) < 0)
		goto done;

	ret = get_feature_report(radio->handle, RIFT_CMD_RADIO_READ_DATA, buffer);
	if (ret < 0)
		goto done;
	if (ret < length + 7) {
		ret = -EIO;
		goto done;
	}

	memcpy (flash_data, buffer+7, length);
	radio->calibration_progress++;

done:
	return ret;
}

static int rift_radio_write_complete(rift_hmd_radio_state *radio)
{
	return rift_radio_command_complete(radio);
}

static int rift_radio_write(rift_hmd_radio_state *radio, uint8_t cmd_byte1, uint8_t cmd_byte2, uint8_t device_type,
	unsigned char *buf, int length)
{
	int ret;

	if (buf[0] != RIFT_CMD_RADIO_READ_DATA)
		return -EINVAL;

	if (!radio->command_result_pending) {
		ret = send_feature_report(radio->handle, buf, length);
		if (ret < 0)
			return ret;

		if ((ret = rift_hmd_radio_send_cmd(radio->handle, cmd_byte1, cmd_byte2, device_type)) < 0)
			goto done;
	}

	ret = rift_radio_write_complete(radio);
done:
	return ret;
}


static int rift_radio_read_calibration_hash(rift_hmd_radio_state *radio, uint8_t device_type,
					    uint8_t hash[16])
{
	if (radio->cur_read_cmd == RIFT_RADIO_CMD_NONE) {
		LOGV("Starting FW hash read for controller %d\n", device_type);
		radio->cur_read_cmd = RIFT_FW_READ_CMD_CALIBRATION_HASH;
	}
	return rift_radio_read_flash(radio, device_type, 0x1bf0, 16, hash);
}

static int rift_radio_read_calibration(rift_hmd_radio_state *radio, uint8_t device_type,
		char **json_out, uint16_t *length, rift_touch_calibration_info *info)
{
	int ret;

	/* First, read the flash header to get the block size */
	if (radio->cur_read_cmd == RIFT_RADIO_CMD_NONE ||
	    radio->cur_read_cmd == RIFT_FW_READ_CMD_CALIBRATION_HDR) {
		uint16_t json_length;
		uint8_t flash_data[20];

		if (radio->cur_read_cmd == RIFT_RADIO_CMD_NONE) {
			LOGV("Starting FW calibration hdr read for controller %d\n", device_type);
			radio->cur_read_cmd = RIFT_FW_READ_CMD_CALIBRATION_HDR;
		}

		info->field = "flash header";
		ret = rift_radio_read_flash(radio, device_type, 0, 20, flash_data);
		if (ret < 0) {
			if (ret != -EINPROGRESS)
				radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;
			return ret;
		}

		if (flash_data[0] != 1 || flash_data[1] != 0) {
			info->reason = "expected calibration header version 1";
			radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;
			return -EINVAL; /* Invalid data */
		}
		json_length = (flash_data[3] << 8) | flash_data[2];

		if (radio->read_block_alloc < json_length + 1) {
			unsigned char *block = realloc(radio->read_block, json_length + 1);
			if (block == NULL) {
				info->field = "flash buffer"; info->reason = "not enough memory";
				return -ENOMEM;
			}
			radio->read_block = block;
			radio->read_block_alloc = json_length + 1;
		}

		radio->read_data_offset = 20;
		radio->read_data_size = json_length;

		memcpy(radio->read_block, flash_data + 4, OHMD_MIN(16, json_length));
		radio->read_block[json_length] = '\0';

		LOGV("Starting FW calibration block read for controller %d\n", device_type);
		radio->cur_read_cmd = RIFT_FW_READ_CMD_CALIBRATION;
	}

	info->field = "flash block";
	for (; radio->read_data_offset < radio->read_data_size + 4; radio->read_data_offset += 20) {
		uint16_t offset = radio->read_data_offset;
		uint16_t json_offset = offset - 4;

		ret = rift_radio_read_flash(radio, device_type, offset,
				OHMD_MIN (20, radio->read_data_size - json_offset),
				radio->read_block + json_offset);
		if (ret < 0) {
			if (ret != -EINPROGRESS)
				radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;
			return ret;
		}

	}

	*json_out = (char *) radio->read_block;
	*length = radio->read_data_size;
	radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;

	return 0;
}

static bool json_is_numeric_array(const nx_json *array, unsigned int count)
{
	if (array->type != NX_JSON_ARRAY || array->length < (int)count)
		return false;
	const nx_json *value = array->child;
	for (unsigned int i = 0; i < count; i++, value = value->next) {
		if ((value->type != NX_JSON_INTEGER && value->type != NX_JSON_DOUBLE) ||
		    !isfinite(value->dbl_value) || fabs(value->dbl_value) > FLT_MAX)
			return false;
	}
	return true;
}

static bool json_read_uint16(const nx_json *obj, const char *key, uint16_t *out)
{
	const nx_json *value = nx_json_get(obj, key);
	if (value->type != NX_JSON_INTEGER || value->int_value < 0 || value->int_value > UINT16_MAX)
		return false;
	*out = value->int_value;
	return true;
}

static bool json_read_vec3(const nx_json *nxj, const char *key, vec3f *out)
{
	const nx_json *member = nx_json_get (nxj, key);
	if (!json_is_numeric_array(member, 3))
		return false;

	out->x = nx_json_item (member, 0)->dbl_value;
	out->y = nx_json_item (member, 1)->dbl_value;
	out->z = nx_json_item (member, 2)->dbl_value;
	return true;
}

static bool json_read_imu_calibration(const nx_json *obj, const char *key,
		float matrix[3][3], vec3f *offset, unsigned int *defaults, unsigned int flag)
{
	const nx_json *array = nx_json_get(obj, key);
	/* Upstream defaults absent offsets to zero; the nine matrix entries are required. */
	if (array->type != NX_JSON_ARRAY || array->length < 9 ||
	    !json_is_numeric_array(array, OHMD_MIN(array->length, 12)))
		return false;
	for (unsigned int i = 0; i < 9; i++)
		matrix[i/3][i%3] = nx_json_item(array, i)->dbl_value;
	for (unsigned int i = 0; i < 3; i++)
		offset->arr[i] = nx_json_item(array, i + 9)->dbl_value;
	if (array->length < 12)
		*defaults |= flag;

	/* A singular matrix loses an IMU axis, including an all-zero model. */
	double determinant = (double)matrix[0][0] * ((double)matrix[1][1] * matrix[2][2] - (double)matrix[1][2] * matrix[2][1]) -
		(double)matrix[0][1] * ((double)matrix[1][0] * matrix[2][2] - (double)matrix[1][2] * matrix[2][0]) +
		(double)matrix[0][2] * ((double)matrix[1][0] * matrix[2][1] - (double)matrix[1][1] * matrix[2][0]);
	return determinant != 0;
}

static int rift_touch_parse_calibration(char *json_in,
		rift_touch_calibration *calibration, rift_touch_calibration_info *info)
{
	rift_touch_calibration_info ignored;
	if (!info) info = &ignored;
	*info = (rift_touch_calibration_info){ .field = "JSON", .reason = "malformed JSON or missing root" };
	rift_touch_calibration parsed = {0};
	rift_touch_calibration *c = &parsed;
	const nx_json* nxj = NULL, *obj, *version, *array;
	int ret = -EINVAL;
	unsigned int i;
	/* nxjson modifies its input. Keep cached and radio-owned bytes intact. */
	char *json = malloc (strlen (json_in) + 1);
	if (json == NULL) {
		ret = -ENOMEM;
		info->reason = "not enough memory";
		goto out;
	}
	strcpy (json, json_in);

	nxj = nx_json_parse_silent (json, NULL, &ret);
	if (nxj == NULL) {
		if (ret == -ENOMEM) info->reason = "not enough memory";
		if (ret == 0)
			ret = -EINVAL;
		goto out;
	}
	ret = -EINVAL;
	info->field = "root"; info->reason = "expected an object";
	if (nxj->type != NX_JSON_OBJECT)
		goto out;

	info->field = "TrackedObject";
	obj = nx_json_get(nxj, "TrackedObject");
	if (obj->type != NX_JSON_OBJECT)
		goto out;

	info->field = "JsonVersion"; info->reason = "expected integer version 2";
	version = nx_json_get (obj, "JsonVersion");
	if (version->type != NX_JSON_INTEGER || version->int_value != 2)
		goto out;

	info->field = "ImuPosition"; info->reason = "expected three finite coordinates";
	if (!json_read_vec3 (obj, "ImuPosition", &c->imu_position))
		goto out;

	const struct { const char *key; uint16_t *value; } required[] = {
		{"JoyXRangeMin", &c->joy_x_range_min},
		{"JoyXRangeMax", &c->joy_x_range_max},
		{"JoyYRangeMin", &c->joy_y_range_min},
		{"JoyYRangeMax", &c->joy_y_range_max},
		{"TriggerMinRange", &c->trigger_min_range},
		{"TriggerMidRange", &c->trigger_mid_range},
		{"TriggerMaxRange", &c->trigger_max_range},
		{"MiddleMinRange", &c->middle_min_range},
		{"MiddleMidRange", &c->middle_mid_range},
		{"MiddleMaxRange", &c->middle_max_range},
	};
	info->reason = "missing or invalid unsigned 16-bit endpoint";
	for (i = 0; i < sizeof required / sizeof required[0]; i++) {
		info->field = required[i].key;
		if (!json_read_uint16(obj, required[i].key, required[i].value))
			goto out;
	}
	info->reason = "range endpoints must be strictly increasing";
	info->field = "JoyXRange";
	if (c->joy_x_range_min >= c->joy_x_range_max) goto out;
	info->field = "JoyYRange";
	if (c->joy_y_range_min >= c->joy_y_range_max) goto out;
	info->field = "TriggerRange";
	if (!(c->trigger_min_range < c->trigger_mid_range && c->trigger_mid_range < c->trigger_max_range)) goto out;
	info->field = "MiddleRange";
	if (!(c->middle_min_range < c->middle_mid_range && c->middle_mid_range < c->middle_max_range)) goto out;

	/* Missing comparison-only dead zones disable the clamp, as upstream permits. */
	if (!json_read_uint16(obj, "JoyXDeadMin", &c->joy_x_dead_min) ||
	    !json_read_uint16(obj, "JoyXDeadMax", &c->joy_x_dead_max)) {
		c->joy_x_dead_min = 1; c->joy_x_dead_max = 0;
		info->defaults |= 4;
	}
	if (!json_read_uint16(obj, "JoyYDeadMin", &c->joy_y_dead_min) ||
	    !json_read_uint16(obj, "JoyYDeadMax", &c->joy_y_dead_max)) {
		c->joy_y_dead_min = 1; c->joy_y_dead_max = 0;
		info->defaults |= 8;
	}
	info->reason = "expected a finite nonsingular 3x3 matrix and numeric offsets";
	info->field = "GyroCalibration";
	if (!json_read_imu_calibration(obj, "GyroCalibration", c->gyro_matrix, &c->gyro_offset, &info->defaults, 1)) goto out;
	info->field = "AccCalibration";
	if (!json_read_imu_calibration(obj, "AccCalibration", c->accel_matrix, &c->accel_offset, &info->defaults, 2)) goto out;

	c->middle_flipped = nx_json_get (obj, "MiddleFlipped")->int_value;

	/* Slots 4-6 are unused by the decoder: neither require nor inspect their values. */
	const unsigned int channels[] = {0, 1, 2, 3, 7};
	const char *cap_keys[] = {"CapSenseMin", "CapSenseTouch"};
	for (unsigned int k = 0; k < 2; k++) {
		info->field = cap_keys[k]; info->reason = "missing or invalid consumed capacitive value";
		array = nx_json_get(obj, cap_keys[k]);
		if (array->type != NX_JSON_ARRAY || array->length < 8) goto out;
		for (i = 0; i < sizeof channels / sizeof channels[0]; i++) {
			unsigned int channel = channels[i];
			const nx_json *value = nx_json_item(array, channel);
			if (value->type != NX_JSON_INTEGER || value->int_value < 0 || value->int_value > UINT16_MAX) goto out;
			if (k) c->cap_sense_touch[channel] = value->int_value;
			else c->cap_sense_min[channel] = value->int_value;
		}
	}
	info->field = "CapSenseTouch"; info->reason = "consumed capacitive range must be nonzero";
	for (i = 0; i < sizeof channels / sizeof channels[0]; i++) {
		if (c->cap_sense_touch[channels[i]] == c->cap_sense_min[channels[i]]) goto out;
	}

	info->field = "ModelPoints"; info->reason = "expected nonempty finite LED geometry with nonzero normals";
	/* LED model points */
	const nx_json *led_model = nx_json_get (obj, "ModelPoints");
	if (led_model->type != NX_JSON_OBJECT) {
		goto out;
	}

	if (led_model->length == 0 || led_model->length > 255) {
		goto out;
	}

	rift_leds_init (&c->leds, led_model->length);
	if (c->leds.num_points && c->leds.points == NULL) {
		ret = -ENOMEM;
		info->reason = "not enough memory";
		goto out;
	}

	for (i = 0; i < c->leds.num_points; i++) {
		rift_led *led = &c->leds.points[i];
		const nx_json* point[6];
		char name[9];
		int j;

		snprintf(name, 9, "Point%d", i);
		array = nx_json_get (led_model, name);
		if (!json_is_numeric_array(array, 6)) {
			goto out;
		}

		for (j = 0; j < 6; j++)
			point[j] = nx_json_item (array, j);

		led->id = i;
		led->pos.x = point[0]->dbl_value;
		led->pos.y = point[1]->dbl_value;
		led->pos.z = point[2]->dbl_value;
		led->dir.x = point[3]->dbl_value;
		led->dir.y = point[4]->dbl_value;
		led->dir.z = point[5]->dbl_value;
		if (led->dir.x == 0 && led->dir.y == 0 && led->dir.z == 0)
			goto out;
		led->pattern = 0xff;
	}

	ret = 0;
	goto out;
out:
	if (nxj != NULL)
		nx_json_free (nxj);
	free (json);
	if (ret == 0) {
		rift_leds_clear (&calibration->leds);
		*calibration = parsed;
	} else {
		rift_leds_clear (&parsed.leds);
	}
	return ret;
}

static void
make_calibration_hash_key(rift_hmd_radio_state *radio, char *config_block_key)
{
	char hash_hex[64], *cur;
	int i;

	for (i = 0, cur = hash_hex; i < 16; i++) {
		sprintf(cur, "%02x", radio->calibration_hash[i]);
		cur += 2;
	}
	snprintf(config_block_key, 255, "rift-touch-config-%s.bin", hash_hex);
	config_block_key[255] = '0';
}

int rift_touch_get_calibration(ohmd_context* ctx, rift_hmd_radio_state *radio,
	int device_id, rift_touch_calibration *calibration, rift_touch_calibration_state *state)
{
	unsigned long length;
	char *json = NULL, *cached_json = NULL;
	int ret = -1;
	bool had_config_cache = false;

	if (radio->cur_read_cmd != RIFT_RADIO_CMD_NONE && radio->device_in_progress != device_id)
		return -EBUSY; /* Another device is being read right now */

	if (radio->cur_read_cmd != RIFT_RADIO_CMD_NONE &&
	    radio->cur_read_cmd != RIFT_FW_READ_CMD_CALIBRATION_HASH &&
	    radio->cur_read_cmd != RIFT_FW_READ_CMD_CALIBRATION_HDR &&
	    radio->cur_read_cmd != RIFT_FW_READ_CMD_CALIBRATION)
		return -EBUSY; /* A different command is in progress */

	state->source = "radio";
	state->failure = (rift_touch_calibration_info){ .field = "previous radio command" };
	ret = rift_radio_drain_abandoned(ctx, radio);
	if (ret < 0) return ret;
	radio->device_in_progress = device_id;
	state->failure = (rift_touch_calibration_info){ .field = "calibration hash" };

	if (radio->cur_read_cmd == RIFT_RADIO_CMD_NONE ||
	    radio->cur_read_cmd == RIFT_FW_READ_CMD_CALIBRATION_HASH) {
		/* If the controller isn't on yet, we might fail to read the calibration data */
		ret = rift_radio_read_calibration_hash(radio, device_id, radio->calibration_hash);
		if (ret < 0) {
			if (ret != -EINPROGRESS) {
				LOGV ("Failed to read calibration hash from device %d", device_id);
			}
			goto out;
		}
		/* Silence is not an identity change. Preserve failures until a different
		 * hash is actually read, including across slow I/O and sleeping reports. */
		if (state->have_hash && memcmp(state->hash, radio->calibration_hash, 16) != 0)
			memset(state, 0, sizeof *state);
		memcpy(state->hash, radio->calibration_hash, 16);
		state->have_hash = true;
		radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;
	}

	/* Only malformed data quarantines this hash's cache entry. A temporary
	 * allocation failure is retried without bypassing a potentially valid cache. */
	if (radio->cur_read_cmd == RIFT_RADIO_CMD_NONE && !state->cache_rejected) {
		char config_block_key[256];
		make_calibration_hash_key(radio, config_block_key);
		state->source = "cache";
		state->failure = (rift_touch_calibration_info){ .field = "cache read" };
		ret = ohmd_get_config(ctx, config_block_key, &cached_json, &length);
		if (ret == ENOMEM || ret == -ENOMEM) {
			ret = -ENOMEM;
			goto out;
		}
		if (ret == 0) {
			ret = rift_touch_parse_calibration(cached_json, calibration, &state->failure);
			if (ret == 0) {
				had_config_cache = true;
			} else if (ret == -EINVAL) {
				state->cache_rejected = true;
				LOGW("%s controller cache calibration: %s: %s; reading from the radio",
					device_id == RIFT_TOUCH_CONTROLLER_RIGHT ? "Right" : "Left", state->failure.field, state->failure.reason);
			} else {
				goto out;
			}
		}
	}

	if (!had_config_cache) {
		uint16_t short_len;
		state->source = "radio";
		state->failure = (rift_touch_calibration_info){ .field = "flash read" };
		ret = rift_radio_read_calibration(radio, device_id, &json, &short_len, &state->failure);
		if (ret < 0)
			goto out;
		length = (unsigned long) short_len;
		ret = rift_touch_parse_calibration(json, calibration, &state->failure);
		if (ret < 0)
			goto out;
	}

	if (!had_config_cache) {
		char config_block_key[256];
		make_calibration_hash_key(radio, config_block_key);

		/* Store the freshly read config block.
		 * We don't need to free on this branch - our json block is in the radio read store */
		LOGD("Storing touch controller calibration cache file %s", config_block_key);
		ret = ohmd_set_config(ctx, config_block_key, json, length);
		if (ret < 0) {
			state->failure = (rift_touch_calibration_info){ .field = "cache write" };
			goto out;
		}
	}

	unsigned int defaults = state->failure.defaults & ~state->defaults_warned;
	if (defaults) {
		LOGW("%s controller %s calibration: using defaults for%s%s%s%s (zero offsets; missing dead zones disable the clamp)",
			device_id == RIFT_TOUCH_CONTROLLER_RIGHT ? "Right" : "Left", state->source,
			defaults & 1 ? " GyroCalibration offsets" : "", defaults & 2 ? " AccCalibration offsets" : "",
			defaults & 4 ? " JoyX dead zone" : "", defaults & 8 ? " JoyY dead zone" : "");
		state->defaults_warned |= defaults;
	}

	char control_name[20];
	snprintf (control_name, 20, "Controller %u", device_id);
	rift_leds_dump (&calibration->leds, control_name);

	radio->device_in_progress = -1;
	radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;

	ret = 0;
out:
	if (ret != -EINPROGRESS) {
		rift_radio_abandon_command(radio);
		radio->device_in_progress = -1;
		radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;
	}
	free(cached_json);
	return ret;
}

void
rift_touch_clear_calibration(rift_touch_calibration *calibration)
{
	rift_leds_clear (&calibration->leds);
}

void rift_hmd_radio_init(rift_hmd_radio_state *radio, hid_device *handle)
{
	radio->handle = handle;
}

void rift_hmd_radio_clear(rift_hmd_radio_state *radio)
{
	if (radio->read_block) {
		free(radio->read_block);
		radio->read_block = NULL;
		radio->read_block_alloc = 0;
	}
}

bool rift_hmd_radio_get_address(hid_device *handle, uint8_t radio_address[5])
{
	unsigned char buf[FEATURE_BUFFER_SIZE];
	int ret_size;

	ret_size = rift_hmd_radio_send_cmd (handle, 0x05, 0x03, 0x05);
	if (ret_size < 0)
		return false;

	if ((ret_size = rift_hmd_radio_get_cmd_response(handle, true)) < 0)
		return false;

	ret_size = get_feature_report(handle, RIFT_CMD_RADIO_READ_DATA, buf);
	if (ret_size < 0)
		return false;

	if (!decode_radio_address (radio_address, buf, ret_size)) {
		LOGE("Failed to decode received radio address");
		return false;
	}

	return true;
}

int rift_touch_send_haptics(ohmd_context *ctx, rift_hmd_radio_state *radio, int device_id, bool low_freq, uint8_t amplitude)
{
	int ret = -1;

	if (radio->cur_read_cmd != RIFT_RADIO_CMD_NONE && radio->device_in_progress != device_id)
		return -EBUSY; /* Another device is being read right now */

	if (radio->cur_read_cmd != RIFT_RADIO_CMD_NONE &&
	    radio->cur_read_cmd != RIFT_RADIO_WRITE_CMD_HAPTICS)
		return -EBUSY; /* A different command is in progress */

	ret = rift_radio_drain_abandoned(ctx, radio);
	if (ret < 0) return ret;
	radio->device_in_progress = device_id;

	if (radio->cur_read_cmd == RIFT_RADIO_CMD_NONE) {
		/* Prepare command buffer then dispatch and await */
		unsigned char buf[31];
		int buf_size = 31;

		memset(buf, 0, sizeof(buf));
		if (amplitude != 0) {
			if (low_freq)
				buf[3] = 0xa0; /* 160Hz vibration */
			else
				buf[4] = 0xa0; /* 320Hz vibration */
			buf[7] = amplitude; /* Intensity byte */
		}

		buf[0] = RIFT_CMD_RADIO_READ_DATA;

		radio->cur_read_cmd = RIFT_RADIO_WRITE_CMD_HAPTICS;
		ret = rift_radio_write(radio, 0x02, 0x03, device_id, buf, buf_size);
	}
	else {
		/* Just waiting for command result */
		ret = rift_radio_write_complete(radio);
	}

	if (ret < 0) {
		/* Only -EINPROGRESS leaves the write running for the caller to finish. Any other
		 * failure ends it, and an unreadable status leaves its result behind. */
		if (ret != -EINPROGRESS)
			rift_radio_abandon_command(radio);
		return ret;
	}

	radio->device_in_progress = -1;
	radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;

	return 0;
}

void rift_touch_cancel_in_progress(rift_hmd_radio_state *radio, int device_id)
{
	if (radio->cur_read_cmd != RIFT_RADIO_CMD_NONE && radio->device_in_progress != device_id)
		return; /* Another device is being read right now */

	radio->device_in_progress = -1;
	radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;
}
