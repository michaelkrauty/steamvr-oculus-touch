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
typedef struct rift_hmd_radio_ops rift_hmd_radio_ops;

enum rift_radio_cmd_state {
	RIFT_RADIO_CMD_NONE = 0,
	RIFT_FW_READ_CMD_CALIBRATION_HASH,
	RIFT_FW_READ_CMD_CALIBRATION_HDR,
	RIFT_FW_READ_CMD_CALIBRATION,
	RIFT_RADIO_WRITE_CMD_HAPTICS,
};

/* What carries the controllers' radio. Commands are polled: -EINPROGRESS asks the
 * caller to call again with the same arguments until the command ends. */
struct rift_hmd_radio_ops {
	/* The next controller report into buf (its size), 0 if none came within wait_ms. */
	int (*read_report)(rift_hmd_radio_state *radio, unsigned char *buf, int size, int wait_ms);
	/* length bytes of a controller's flash at offset */
	int (*read_flash)(rift_hmd_radio_state *radio, uint8_t device_type,
			uint16_t offset, uint16_t length, uint8_t *data);
	int (*send_haptics)(rift_hmd_radio_state *radio, uint8_t device_type, bool low_freq, uint8_t amplitude);
	/* 0 once no abandoned command can answer a new one, -EBUSY while one still may */
	int (*drain_abandoned)(ohmd_context *ctx, rift_hmd_radio_state *radio);
	/* give up on the command in progress */
	void (*abandon_command)(rift_hmd_radio_state *radio);
	/* the address the sensors synchronise to */
	bool (*get_address)(rift_hmd_radio_state *radio, uint8_t address[5]);
};

extern const rift_hmd_radio_ops rift_hmd_radio_headset_ops;
extern const rift_hmd_radio_ops rift_hmd_radio_external_ops;

struct rift_hmd_radio_state {
	const rift_hmd_radio_ops *ops;
	hid_device *handle; /* the headset's radio interface: reports and commands */
	hid_device *hmd_handle; /* the headset's own interface: the radio address */
	const ohmd_cv1touch_radio *external; /* or a radio the caller serves */

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

void rift_hmd_radio_init(rift_hmd_radio_state *radio, hid_device *handle, hid_device *hmd_handle);
void rift_hmd_radio_init_external(rift_hmd_radio_state *radio, const ohmd_cv1touch_radio *external);
void rift_hmd_radio_clear(rift_hmd_radio_state *radio);
int rift_touch_get_calibration(ohmd_context* ctx,
		rift_hmd_radio_state *radio,
		int device_id,
		rift_touch_calibration *calibration, rift_touch_calibration_state *state);
void rift_touch_clear_calibration(rift_touch_calibration *calibration);
int rift_touch_send_haptics(ohmd_context *ctx, rift_hmd_radio_state *radio, int device_id, bool low_freq, uint8_t amplitude);
void rift_touch_cancel_in_progress(rift_hmd_radio_state *radio, int device_id);

#endif /* RIFT_HMD_RADIO_H */
