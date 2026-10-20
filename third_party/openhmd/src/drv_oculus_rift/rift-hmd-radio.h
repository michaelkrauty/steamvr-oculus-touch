/*
 * Oculus Rift CV1 Radio
 * Copyright 2016 Philipp Zabel
 * Copyright 2019 Jan Schmidt
 * SPDX-License-Identifier:	BSL-1.0
 */
#ifndef RIFT_HMD_RADIO_H
#define RIFT_HMD_RADIO_H

#include <hidapi.h>
#include "rift.h"

typedef struct rift_hmd_radio_state rift_hmd_radio_state;

enum rift_radio_cmd_state {
	RIFT_RADIO_CMD_NONE = 0,
	RIFT_FW_READ_CMD_CALIBRATION_HASH,
	RIFT_FW_READ_CMD_CALIBRATION_HDR,
	RIFT_FW_READ_CMD_CALIBRATION,
	RIFT_RADIO_WRITE_CMD_HAPTICS,
};

struct rift_hmd_radio_state {
	hid_device *handle;

	enum rift_radio_cmd_state cur_read_cmd; /* Which radio read/write command is in progress */
	bool discard_reply; /* drain an abandoned command before submitting another */
	uint64_t drain_started, drain_next_poll; /* shared by calibration and haptics */
	bool drain_warned;
	bool command_result_pending; /* We are waiting for a response to a read/write command */
	int device_in_progress; /* Device ID we're currently reading from */
	uint64_t calibration_deadline; /* bounds one command without validated progress */
	uint64_t calibration_progress; /* validated flash replies, never busy status polls */

	uint32_t read_block_alloc; /* Allocated size of the read_block */
	uint32_t read_data_offset; /* current read offset */
	uint32_t read_data_size;   /* Amount of data in the read block */
	unsigned char *read_block;

	uint8_t calibration_hash[16];
};

void rift_hmd_radio_init(rift_hmd_radio_state *radio, hid_device *handle);
void rift_hmd_radio_clear(rift_hmd_radio_state *radio);
int rift_touch_get_calibration(ohmd_context* ctx,
		rift_hmd_radio_state *radio,
		int device_id,
		rift_touch_calibration *calibration, rift_touch_calibration_state *state);
void rift_touch_clear_calibration(rift_touch_calibration *calibration);
bool rift_hmd_radio_get_address(hid_device *handle, uint8_t address[5]);
int rift_touch_send_haptics(ohmd_context *ctx, rift_hmd_radio_state *radio, int device_id, bool low_freq, uint8_t amplitude);
void rift_touch_cancel_in_progress(rift_hmd_radio_state *radio, int device_id);

#endif /* RIFT_HMD_RADIO_H */
