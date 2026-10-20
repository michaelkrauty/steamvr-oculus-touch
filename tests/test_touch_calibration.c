// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
/* Exercise malformed cached and radio calibration, with explicit ownership counts.
 * HID and cache operations are synthetic; no devices or user files are accessed. */
#include <errno.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "test_util.h"

static void *held[8];
static size_t owned;
static int fail_alloc, fail_radio_alloc;
static void *cal_realloc(void *p, size_t size)
{
	if (fail_radio_alloc) { fail_radio_alloc = 0; return NULL; }
	return realloc(p, size);
}
static void *cal_malloc(size_t size)
{
	if (fail_alloc) { fail_alloc = 0; return NULL; }
	void *p = malloc(size);
	CHECK(p != NULL && owned < sizeof held / sizeof held[0]);
	held[owned++] = p;
	return p;
}
static void cal_free(void *p)
{
	if (!p) return;
	size_t i;
	for (i = 0; i < owned && held[i] != p; i++) {}
	CHECK(i < owned);
	held[i] = held[--owned];
	free(p);
}

static int warning_logs, syntax_logs;
static char diagnostics[4096];
static int count_printf(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	if (strcmp(format, "[%s] ") == 0) {
		const char *level = va_arg(args, const char *);
		if (strcmp(level, "WW") == 0 || strcmp(level, "EE") == 0) warning_logs++;
	}
	va_end(args);
	va_start(args, format);
	size_t used = strlen(diagnostics);
	vsnprintf(diagnostics + used, sizeof diagnostics - used, format, args);
	va_end(args);
	return 0;
}
static int count_fprintf(FILE *stream, const char *format, ...)
{
	(void)stream; (void)format;
	syntax_logs++;
	return 0;
}
#define printf count_printf
#define realloc cal_realloc
#define malloc cal_malloc
#define free cal_free
#include "rift-hmd-radio.c"
#undef realloc
#undef malloc
#undef free
#include "packet.c"
static size_t live_nodes;
static int fail_node_after = -1, node_allocations;
static void *node_calloc(void)
{
	if (fail_node_after == 0) { fail_node_after = -1; return NULL; }
	if (fail_node_after > 0) fail_node_after--;
	void *p = calloc(1, sizeof(nx_json));
	CHECK(p != NULL);
	live_nodes++;
	node_allocations++;
	return p;
}
static void node_free(void *p)
{
	CHECK(p != NULL && live_nodes > 0);
	live_nodes--;
	free(p);
}
#define NX_JSON_CALLOC() node_calloc()
#define NX_JSON_FREE(json) node_free((void *)(json))
#define fprintf count_fprintf
#include "ext_deps/nxjson.c"
#undef fprintf
#undef NX_JSON_CALLOC
#undef NX_JSON_FREE

static size_t live_leds;
static int fail_led_alloc;
static void *led_calloc(size_t count, size_t size)
{
	CHECK(size == sizeof(rift_led));
	if (fail_led_alloc) { fail_led_alloc = 0; return NULL; }
	void *p = calloc(count, size);
	if (p) live_leds++;
	return p;
}
static void led_free(void *p)
{
	if (p) {
		CHECK(live_leds > 0);
		live_leds--;
		free(p);
	}
}
/* Use the real LED constructor/clear functions, counting their real heap allocations. */
#define get_feature_report hmd_get_feature_report
#define send_feature_report hmd_send_feature_report
#define calloc led_calloc
#define free led_free
#include "rift.c"
#undef get_feature_report
#undef send_feature_report
#undef calloc
#undef free
#include "omath.c"
#undef printf

/* Complete synthetic factory data, including every decoder input. */
#define INPUT_CALIBRATION "\"JoyXRangeMin\":0,\"JoyXRangeMax\":100,\"JoyXDeadMin\":40,\"JoyXDeadMax\":60," \
	"\"JoyYRangeMin\":0,\"JoyYRangeMax\":100,\"JoyYDeadMin\":40,\"JoyYDeadMax\":60," \
	"\"TriggerMinRange\":0,\"TriggerMidRange\":50,\"TriggerMaxRange\":100," \
	"\"MiddleMinRange\":0,\"MiddleMidRange\":50,\"MiddleMaxRange\":100," \
	"\"GyroCalibration\":[1,0,0,0,1,0,0,0,1,0,0,0]," \
	"\"AccCalibration\":[1,0,0,0,1,0,0,0,1,0,0,0]," \
	"\"CapSenseMin\":[0,0,0,0,0,0,0,0],\"CapSenseTouch\":[100,100,100,100,0,0,0,100],"
static const char valid_json[] = "{\"TrackedObject\":{\"JsonVersion\":2,\"ImuPosition\":[0,0,0],"
	INPUT_CALIBRATION "\"ModelPoints\":{\"Point0\":[0,0,0,0,1,0]}}}";
static const char *cached_json;
static int cache_writes, cache_reads, fail_cached_copy;
static uint8_t flash_hash[16];
static uint64_t hid_delay_ns;
static int flash_error, short_data, get_error, cache_error, update_fixture, pending_header;
static int data_fetches, haptic_writes, last_haptic;
static int fail_status_reads, set_config_error, hash_timeouts, status_reads;
static int simulate_flash, flash_offset, flash_length, hash_reads, flash_headers, hid_sends, pending_polls, short_response;
static const char *radio_json;
static uint64_t fake_time;
static int tracker_creations, imu_updates;
uint64_t ohmd_monotonic_get(ohmd_context *ctx) { (void)ctx; return fake_time; }
uint64_t ohmd_monotonic_per_sec(ohmd_context *ctx) { (void)ctx; return 1000000000; }
rift_tracked_device *rift_tracker_add_device(rift_tracker_ctx *ctx, int id,
    posef *imu, posef *model, rift_leds *leds, rift_tracked_device_imu_calibration *calibration)
{
	CHECK(ctx && id > 0 && imu && model && leds && calibration);
	tracker_creations++;
	return (rift_tracked_device *)(uintptr_t)1;
}
void rift_tracked_device_imu_update(rift_tracked_device *device, uint64_t host_ns,
    uint32_t device_ts, float dt, const vec3f *gyro, const vec3f *accel, const vec3f *mag)
{
	CHECK(device && host_ns > 0 && device_ts > 0 && dt >= 0 && gyro && accel && mag);
	imu_updates++;
}

struct radio_transaction {
	uint64_t ready;
	bool status_read, data_read;
	unsigned char data[FEATURE_BUFFER_SIZE];
};
static struct radio_transaction transactions[2048];
static int independent_radio, transaction_count, transaction_data;
static unsigned char staged_data[FEATURE_BUFFER_SIZE];
static void queue_transaction(const unsigned char *control)
{
	CHECK(transaction_count < 2048);
	struct radio_transaction *tx = &transactions[transaction_count++];
	memset(tx, 0, sizeof *tx);
	tx->data[0] = RIFT_CMD_RADIO_READ_DATA;
	int offset = staged_data[3] | staged_data[4] << 8;
	int count = staged_data[5] | staged_data[6] << 8;
	bool flash = control[3] == 0x03;
	if (independent_radio == 2) {
		tx->ready = fake_time + 30000000;
	} else if (transaction_count == 1) {
		CHECK(flash && offset == 0x1bf0);
		tx->ready = fake_time;
	} else if (transaction_count == 2) {
		CHECK(flash && offset == 0);
		tx->ready = independent_radio == 3 ? 6600000000 : 3100000000; /* completes after timeout and radio reuse */
	} else {
		tx->ready = independent_radio == 3 ? fake_time + 30000000 : 3300000000;
	}
	if (!flash) {
		memcpy(tx->data, staged_data, 31);
	} else if (offset == 0x1bf0) {
		memcpy(tx->data + 7, flash_hash, 16);
	} else if (offset == 0) {
		size_t size = strlen(radio_json);
		tx->data[7] = 1;
		tx->data[9] = size & 0xff;
		tx->data[10] = size >> 8;
		memcpy(tx->data + 11, radio_json, size < 16 ? size : 16);
	} else {
		CHECK(offset >= 20 && count <= 20);
		memcpy(tx->data + 7, radio_json + offset - 4, count);
	}
}
static int transaction_response(unsigned char *data)
{
	if (data[0] == RIFT_CMD_RADIO_CONTROL) {
		for (int i = 0; i < transaction_count; i++) {
			struct radio_transaction *tx = &transactions[i];
			if (tx->ready <= fake_time && !tx->status_read) {
				tx->status_read = true;
				transaction_data = i;
				return FEATURE_BUFFER_SIZE;
			}
		}
		data[3] = 0x80;
		return FEATURE_BUFFER_SIZE;
	}
	CHECK(data[0] == RIFT_CMD_RADIO_READ_DATA && transaction_data >= 0);
	struct radio_transaction *tx = &transactions[transaction_data];
	CHECK(tx->status_read && tx->ready <= fake_time);
	tx->data_read = true;
	memcpy(data, tx->data, FEATURE_BUFFER_SIZE);
	data_fetches++;
	return FEATURE_BUFFER_SIZE;
}
int hid_send_feature_report(hid_device *handle, const unsigned char *data, size_t length)
{
	CHECK(handle != NULL && data != NULL);
	fake_time += hid_delay_ns;
	hid_sends++;
	if (independent_radio) {
		if (data[0] == RIFT_CMD_RADIO_READ_DATA) memcpy(staged_data, data, length);
		else { CHECK(data[0] == RIFT_CMD_RADIO_CONTROL); queue_transaction(data); }
		return (int)length;
	}
	if (simulate_flash && data[0] == RIFT_CMD_RADIO_CONTROL && data[3] == 0x02 && data[4] == 0x03) {
		haptic_writes++;
		last_haptic = 1;
		/* A fresh control command supersedes the stalled flash command. Merely
		 * polling its old result must remain busy until that happens. */
		pending_polls = 0;
	}
	if (simulate_flash && data[0] == RIFT_CMD_RADIO_CONTROL && data[3] == 0x03) last_haptic = 0;
	if (simulate_flash && data[0] == RIFT_CMD_RADIO_READ_DATA) {
		flash_offset = data[3] | data[4] << 8;
		flash_length = data[5] | data[6] << 8;
		if (flash_offset == 0 && pending_header) {
			pending_polls = pending_header;
			pending_header = 0;
		}
		if (flash_error && flash_offset != 0x1bf0) return -1;
	}
	return (int)length;
}
int hid_get_feature_report(hid_device *handle, unsigned char *data, size_t length)
{
	CHECK(handle != NULL && length == FEATURE_BUFFER_SIZE);
	fake_time += hid_delay_ns;
	if (data[0] == RIFT_CMD_RADIO_CONTROL) status_reads++;
	if (get_error && data[0] == RIFT_CMD_RADIO_READ_DATA) return -1;
	/* Each fixture controls the controller identity independently of its JSON. */
	memset(data + 1, 0, length - 1);
	/* A status read that fails does not consume the command's completion. */
	if (data[0] == RIFT_CMD_RADIO_CONTROL && fail_status_reads > 0) { fail_status_reads--; return 1; }
	if (independent_radio) return transaction_response(data);
	if (simulate_flash && data[0] == RIFT_CMD_RADIO_CONTROL && flash_offset == 0x1bf0 && hash_timeouts) {
		hash_timeouts--;
		data[3] = 0x08;
		return (int)length;
	}
	if (simulate_flash && data[0] == RIFT_CMD_RADIO_READ_DATA && short_data) return 1;
	if (simulate_flash && data[0] == RIFT_CMD_RADIO_CONTROL && short_response)
		return short_response - 1;
	if (simulate_flash && data[0] == RIFT_CMD_RADIO_CONTROL && pending_polls) {
		pending_polls--;
		data[3] = 0x80;
	}
	if (simulate_flash && data[0] == RIFT_CMD_RADIO_READ_DATA) {
		data_fetches++;
		if (last_haptic) return (int)length;
		size_t size = strlen(radio_json);
		if (flash_offset == 0x1bf0) {
			hash_reads++;
			memcpy(data + 7, flash_hash, sizeof flash_hash);
		} else if (flash_offset == 0) {
			flash_headers++;
			data[7] = 1;
			data[9] = size & 0xff;
			data[10] = size >> 8;
			memcpy(data + 11, radio_json, size < 16 ? size : 16);
		} else {
			CHECK(flash_offset >= 20 && (size_t)(flash_offset - 4 + flash_length) <= size);
			memcpy(data + 7, radio_json + flash_offset - 4, flash_length);
		}
	}
	return (int)length;
}
int ohmd_get_config(ohmd_context *ctx, const char *key, char **buf, unsigned long *length)
{
	CHECK(ctx != NULL && key != NULL && cached_json != NULL);
	cache_reads++;
	if (cache_error) { cache_error = 0; return ENOMEM; }
	if (simulate_flash && !independent_radio) {
		for (size_t i = 0; i < sizeof flash_hash; i++) {
			char hex[3];
			snprintf(hex, sizeof hex, "%02x", flash_hash[i]);
			CHECK(strncmp(key + 18 + 2 * i, hex, 2) == 0);
		}
	}
	*length = strlen(cached_json);
	*buf = cal_malloc(*length + 1);
	CHECK(*buf != NULL);
	memcpy(*buf, cached_json, *length + 1);
	if (fail_cached_copy) { fail_cached_copy = 0; fail_alloc = 1; }
	return 0;
}
int ohmd_set_config(ohmd_context *ctx, const char *key, char *buf, unsigned long length)
{
	CHECK(ctx != NULL && key != NULL && buf != NULL && length == strlen(buf));
	cache_writes++;
	return set_config_error;
}

/* Optimized unity builds may retain the driver's callback table with constant test
 * data. These callbacks are outside this fixture and must never execute. */
uint64_t ohmd_monotonic_conv(uint64_t ticks, uint64_t source, uint64_t destination)
{
	(void)ticks; (void)source; (void)destination;
	CHECK(update_fixture);
	return ticks * destination / source;
}
double ohmd_get_tick(void) { CHECK(update_fixture); return (double)fake_time / 1000000000; }
void rift_tracker_retry_video(rift_tracker_ctx *ctx, uint64_t now_ns)
{
	CHECK(update_fixture && ctx && now_ns == fake_time);
}
void rift_tracker_on_new_exposure(rift_tracker_ctx *ctx, uint32_t hmd_ts,
    uint16_t exposure_count, uint32_t exposure_hmd_ts, uint8_t led_pattern_phase)
{
	(void)ctx; (void)hmd_ts; (void)exposure_count; (void)exposure_hmd_ts; (void)led_pattern_phase;
	CHECK(false);
}

int hid_read(hid_device *handle, unsigned char *data, size_t length)
{
	CHECK(update_fixture && handle == (hid_device *)(uintptr_t)1 && data && length == FEATURE_BUFFER_SIZE);
	return 0; /* This fixture deliberately has no incoming reports. */
}
static void parser_failures_release_the_copy(void)
{
	const char *bad[] = {"{", "[x]", "\"unterminated", "/*", "{\"x\" /*", "{foo:1}"};
	for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++) {
		char text[64];
		strcpy(text, bad[i]);
		int before = syntax_logs, error;
		CHECK(nx_json_parse_silent(text, NULL, &error) == NULL && syntax_logs == before);
		strcpy(text, bad[i]);
		CHECK(nx_json_parse(text, NULL) == NULL && syntax_logs > before);
	}
	rift_touch_calibration calibration = {0};
	CHECK(rift_touch_parse_calibration("{", &calibration, NULL) < 0);
	CHECK(owned == 0);
	CHECK(rift_touch_parse_calibration("{\"TrackedObject\":{\"JsonVersion\":99}}", &calibration, NULL) < 0);
	CHECK(owned == 0);
	fail_alloc = 1;
	CHECK(rift_touch_parse_calibration((char *)valid_json, &calibration, NULL) < 0);
	CHECK(owned == 0 && fail_alloc == 0);
	CHECK(rift_touch_parse_calibration((char *)valid_json, &calibration, NULL) == 0);
	CHECK(owned == 0);
	rift_touch_clear_calibration(&calibration);
	CHECK(live_leds == 0);
}
static void rejected_cache_and_invalid_radio_release_buffers(void)
{
	const char *cases[] = {"{", "{\"TrackedObject\":{\"JsonVersion\":99}}"};
	for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
		ohmd_context ctx = {0};
		rift_touch_calibration_state state = {0};
		rift_touch_calibration calibration = {0};
		rift_hmd_radio_state radio = {.handle = (hid_device *)(uintptr_t)1};
		cached_json = cases[i];
		CHECK(rift_touch_get_calibration(&ctx, &radio, 1, &calibration, &state) < 0);
		CHECK(state.cache_rejected && owned == 0 && cache_writes == 0);
	}
}
static void radio_storage_is_borrowed_even_on_failure(void)
{
	ohmd_context ctx = {0};
	rift_touch_calibration_state state = {0};
	rift_touch_calibration calibration = {0};
	char json[sizeof valid_json];
	memcpy(json, "{", 2);
	rift_hmd_radio_state radio = {
		.cur_read_cmd = RIFT_FW_READ_CMD_CALIBRATION,
		.device_in_progress = 1,
		.read_block = (uint8_t *)json,
		.read_data_size = 1,
		.read_data_offset = 5,
	};
	CHECK(rift_touch_get_calibration(&ctx, &radio, 1, &calibration, &state) < 0);
	CHECK(owned == 0 && strcmp(json, "{") == 0 && cache_writes == 0);
	memcpy(json, valid_json, sizeof json);
	radio.cur_read_cmd = RIFT_FW_READ_CMD_CALIBRATION;
	radio.device_in_progress = 1;
	radio.read_data_size = strlen(json);
	radio.read_data_offset = radio.read_data_size + 4;
	CHECK(rift_touch_get_calibration(&ctx, &radio, 1, &calibration, &state) == 0);
	CHECK(owned == 0 && strcmp(json, valid_json) == 0 && radio.device_in_progress == -1 && cache_writes == 1);
	rift_touch_clear_calibration(&calibration);
	CHECK(live_leds == 0);
}
static void cached_success_releases_both_buffers(void)
{
	ohmd_context ctx = {0};
	rift_touch_calibration_state state = {0};
	rift_touch_calibration calibration = {0};
	rift_hmd_radio_state radio = {.handle = (hid_device *)(uintptr_t)1};
	cached_json = valid_json;
	CHECK(rift_touch_get_calibration(&ctx, &radio, 1, &calibration, &state) == 0);
	CHECK(owned == 0 && radio.device_in_progress == -1 && cache_writes == 0);
	rift_touch_clear_calibration(&calibration);
	CHECK(live_leds == 0);
}
static void late_failures_leave_the_previous_calibration_unchanged(void)
{
	const char *bad[] = {
		"{\"TrackedObject\":{\"JsonVersion\":2,\"ImuPosition\":[1,2,3]," INPUT_CALIBRATION "\"ModelPoints\":{\"Point0\":null}}}",
		"{\"TrackedObject\":{\"JsonVersion\":2,\"ImuPosition\":[1,2,3]," INPUT_CALIBRATION "\"ModelPoints\":{\"Point0\":0}}}",
	};
	char good[] = "{\"TrackedObject\":{\"JsonVersion\":2,\"ImuPosition\":[1,2,3]," INPUT_CALIBRATION "\"ModelPoints\":{\"Point0\":[1,2,3,0,1,0]}}}";
	rift_touch_calibration calibration = {0};
	calibration.imu_position.x = 17;
	calibration.joy_x_range_min = 999;
	rift_leds_init(&calibration.leds, 1);
	CHECK(calibration.leds.points != NULL && live_leds == 1);
	calibration.leds.points[0].id = 42;
	unsigned char before[sizeof calibration];
	memcpy(before, &calibration, sizeof before);
	for (size_t k = 0; k < sizeof bad / sizeof bad[0]; k++) {
		for (int n = 0; n < 32; n++)
			CHECK(rift_touch_parse_calibration((char *)bad[k], &calibration, NULL) < 0);
	}
	CHECK(owned == 0 && live_leds == 1);
	CHECK(memcmp(before, &calibration, sizeof before) == 0);
	CHECK(calibration.leds.points[0].id == 42);
	fail_led_alloc = 1;
	CHECK(rift_touch_parse_calibration(good, &calibration, NULL) < 0);
	CHECK(fail_led_alloc == 0 && owned == 0 && live_leds == 1);
	CHECK(memcmp(before, &calibration, sizeof before) == 0);
	CHECK(rift_touch_parse_calibration(good, &calibration, NULL) == 0);
	CHECK(owned == 0 && live_leds == 1);
	CHECK(calibration.imu_position.x == 1 && calibration.leds.num_points == 1);
	CHECK(calibration.leds.points[0].id == 0 && calibration.leds.points[0].pos.z == 3);
	rift_touch_clear_calibration(&calibration);
	CHECK(live_leds == 0);
}
static void retry_fixture(rift_hmd_t *hmd, ohmd_context *ctx)
{
	memset(hmd, 0, sizeof *hmd);
	hmd->ctx = ctx;
	hmd->tracker_ctx = (rift_tracker_ctx *)(uintptr_t)1;
	hmd->radio.handle = (hid_device *)(uintptr_t)1;
	hmd->touch_dev[0].base.id = 1;
	hmd->touch_dev[0].device_num = RIFT_TOUCH_CONTROLLER_RIGHT;
	hmd->touch_dev[1].base.id = 2;
	hmd->touch_dev[1].device_num = RIFT_TOUCH_CONTROLLER_LEFT;
	simulate_flash = 1;
	fake_time = hid_delay_ns = 0;
	flash_error = short_data = fail_cached_copy = fail_alloc = fail_led_alloc = 0;
	get_error = cache_error = update_fixture = data_fetches = haptic_writes = pending_header = last_haptic = 0;
	diagnostics[0] = 0;
	fail_node_after = -1;
	CHECK(live_nodes == 0);
	independent_radio = transaction_count = 0;
	fail_status_reads = set_config_error = hash_timeouts = status_reads = 0;
	transaction_data = -1;
	memset(flash_hash, 0, sizeof flash_hash);
	cache_reads = cache_writes = hash_reads = flash_headers = hid_sends = pending_polls = short_response = 0;
	warning_logs = syntax_logs = tracker_creations = imu_updates = 0;
	cached_json = "{\"TrackedObject\":{\"JsonVersion\":2,\"ImuPosition\":[0,0,0],\"ModelPoints\":{\"Point0\":null}}}";
}
static void report_frame(rift_hmd_t *hmd, int hand, uint64_t time)
{
	if (fake_time < time) fake_time = time;
	pkt_rift_radio_message message = {0};
	message.touch.timestamp = (uint32_t)(time / 1000);
	handle_touch_controller_message(hmd, time, &hmd->touch_dev[hand], &message);
}
static void clear_retry_fixture(rift_hmd_t *hmd)
{
	free(hmd->radio.read_block);
	rift_touch_clear_calibration(&hmd->touch_dev[0].calibration);
	rift_touch_clear_calibration(&hmd->touch_dev[1].calibration);
	CHECK(owned == 0 && live_leds == 0 && live_nodes == 0);
	for (int hand = 0; hand < 2; hand++) {
		if (hmd->touch_dev[hand].have_calibration) {
			rift_touch_calibration_state *state = &hmd->touch_dev[hand].calibration_state;
			CHECK(state->retry_time == 0 && state->retry_delay == 0 && state->error == 0);
		}
	}
}
static void flash_allocation_failure_keeps_the_previous_buffer(void)
{
	ohmd_context ctx = {0};
	rift_hmd_t hmd;
	retry_fixture(&hmd, &ctx);
	radio_json = valid_json;
	unsigned char *old = malloc(8);
	CHECK(old != NULL);
	hmd.radio.read_block = old;
	hmd.radio.read_block_alloc = 8;
	fail_radio_alloc = 1;
	rift_touch_calibration_state state = {0};
	CHECK(rift_touch_get_calibration(&ctx, &hmd.radio, RIFT_TOUCH_CONTROLLER_RIGHT,
		&hmd.touch_dev[0].calibration, &state) == -ENOMEM);
	CHECK(fail_radio_alloc == 0 && hmd.radio.read_block == old && hmd.radio.read_block_alloc == 8);
	CHECK(owned == 0 && live_leds == 0 && hmd.radio.cur_read_cmd == RIFT_RADIO_CMD_NONE);
	clear_retry_fixture(&hmd);
}
static void transport_failures_report_each_change_once(void)
{
	ohmd_context ctx = {0};
	rift_hmd_t hmd;
	retry_fixture(&hmd, &ctx);
	radio_json = cached_json;
	short_response = 1;
	for (uint64_t frame = 0; frame <= 15500; frame++)
		report_frame(&hmd, 0, 1000000000 + frame * 2000000);
	/* The short status never confirms completion: only the original command
	 * was sent. Further attempts drain it instead of piling up replacements.
	 * The first hash failure stays quiet. The failed drain is reported on
	 * the second failure, then once more when it has stayed stuck for 10 s. */
	CHECK(hid_sends == 2 && warning_logs == 2 && cache_reads == 0);
	short_response = 0;
	/* A different error is a new state. Repeated malformed data stays quiet. */
	for (uint64_t frame = 15501; frame < 100000; frame++)
		report_frame(&hmd, 0, 1000000000 + frame * 2000000);
	/* One abandoned hash payload was discarded, then three fresh hashes read. */
	CHECK(hash_reads == 4 && flash_headers == 3 && cache_reads == 1 && warning_logs == 4);
	clear_retry_fixture(&hmd);
}
static void corrupt_cache_falls_back_to_the_radio(void)
{
	ohmd_context ctx = {0};
	rift_hmd_t hmd;
	retry_fixture(&hmd, &ctx);
	radio_json = valid_json;
	report_frame(&hmd, 0, 1000000000);
	CHECK(hmd.touch_dev[0].have_calibration && tracker_creations == 1 && imu_updates == 1);
	CHECK(cache_reads == 1 && hash_reads == 1 && flash_headers == 1 && cache_writes == 1);
	clear_retry_fixture(&hmd);
}
static void terminal_failures_back_off_and_recover(void)
{
	const char *bad[] = { "{", "{                  ",
		"{\"TrackedObject\":{\"JsonVersion\":2,\"ImuPosition\":[0,0,0],\"ModelPoints\":{\"Point0\":0}}}" };
	for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++) {
		ohmd_context ctx = {0};
		rift_hmd_t hmd;
		retry_fixture(&hmd, &ctx);
		radio_json = bad[i];
		for (uint64_t frame = 0; frame < 100000; frame++)
			report_frame(&hmd, 0, 1000000000 + frame * 2000000);
		CHECK(!hmd.touch_dev[0].have_calibration && tracker_creations == 0);
		CHECK(hash_reads == 9 && flash_headers == 9 && cache_reads == 1 && cache_writes == 0);
		CHECK(hid_sends < 200 && warning_logs == 2 && syntax_logs == 0);
		CHECK(hmd.radio.cur_read_cmd == RIFT_RADIO_CMD_NONE && !hmd.radio.command_result_pending);
		radio_json = valid_json;
		/* Keep reports flowing: a valid firmware block loads at the next capped retry. */
		for (uint64_t frame = 100000; frame <= 121500; frame++) {
			report_frame(&hmd, 0, 1000000000 + frame * 2000000);
			CHECK(hmd.touch_dev[0].have_calibration == (frame == 121500));
		}
		CHECK(hash_reads == 10 && flash_headers == 10 && cache_reads == 1 && cache_writes == 1);
		CHECK(warning_logs == 2 && syntax_logs == 0 && tracker_creations == 1 && imu_updates == 1);
		clear_retry_fixture(&hmd);
	}
}
static void the_other_hand_can_load_while_a_failed_hand_waits(void)
{
	ohmd_context ctx = {0};
	rift_hmd_t hmd;
	retry_fixture(&hmd, &ctx);
	radio_json = cached_json;
	for (uint64_t frame = 0; frame <= 15500; frame++)
		report_frame(&hmd, 0, 1000000000 + frame * 2000000);
	CHECK(hash_reads == 6 && flash_headers == 6 && cache_reads == 1);
	/* The other hand can load while this one's next retry is still at 64 s. */
	radio_json = valid_json;
	report_frame(&hmd, 1, 32002000000);
	CHECK(hmd.touch_dev[1].have_calibration && !hmd.touch_dev[0].have_calibration);
	/* An unconfirmed return keeps this hand's failure history and deadline. */
	report_frame(&hmd, 0, 33100000000);
	CHECK(!hmd.touch_dev[0].have_calibration && hash_reads == 7);
	report_frame(&hmd, 0, 64000000000);
	CHECK(hmd.touch_dev[0].have_calibration && tracker_creations == 2 && imu_updates == 2);
	CHECK(hash_reads == 8 && flash_headers == 8 && cache_reads == 2);
	clear_retry_fixture(&hmd);
}
static void pending_and_busy_commands_continue_on_the_next_report(void)
{
	ohmd_context ctx = {0};
	rift_hmd_t hmd;
	retry_fixture(&hmd, &ctx);
	radio_json = valid_json;
	hmd.radio.cur_read_cmd = RIFT_RADIO_WRITE_CMD_HAPTICS;
	hmd.radio.device_in_progress = RIFT_TOUCH_CONTROLLER_LEFT;
	report_frame(&hmd, 0, 1000000000);
	CHECK(hid_sends == 0 && warning_logs == 0);
	/* A return after silence must not cancel this hand's pending haptics either. */
	hmd.radio.device_in_progress = RIFT_TOUCH_CONTROLLER_RIGHT;
	hmd.radio.command_result_pending = true;
	report_frame(&hmd, 0, 2100000000);
	CHECK(hid_sends == 0 && warning_logs == 0 && hmd.radio.command_result_pending);
	CHECK(hmd.radio.cur_read_cmd == RIFT_RADIO_WRITE_CMD_HAPTICS);
	hmd.radio.cur_read_cmd = RIFT_RADIO_CMD_NONE;
	hmd.radio.command_result_pending = false;
	pending_polls = 2;
	report_frame(&hmd, 0, 2102000000);
	CHECK(hid_sends == 2 && cache_reads == 0 && !hmd.touch_dev[0].have_calibration);
	report_frame(&hmd, 0, 2104000000);
	CHECK(hid_sends == 2 && cache_reads == 0 && !hmd.touch_dev[0].have_calibration);
	report_frame(&hmd, 0, 2106000000);
	CHECK(hmd.touch_dev[0].have_calibration && cache_reads == 1 && hash_reads == 1 && flash_headers == 1);
	clear_retry_fixture(&hmd);
}
static void slow_failures_wait_from_completion(void)
{
	ohmd_context ctx = {0};
	rift_hmd_t hmd;
	retry_fixture(&hmd, &ctx);
	radio_json = cached_json;
	hid_delay_ns = 300000000;
	uint64_t next = 1000000000, received = 1000000000;
	unsigned int delay = 1;
	for (int attempt = 1; attempt <= 9; attempt++) {
		fake_time = next;
		report_frame(&hmd, 0, received);
		received += 2000000;
		CHECK(hash_reads == attempt && flash_headers == attempt);
		uint64_t completed = fake_time;
		CHECK(completed > next + 1000000000);
		int sends = hid_sends;
		/* Queued timestamps remain old while the fresh monotonic clock advances. */
		report_frame(&hmd, 0, received);
		received += 2000000;
		CHECK(hid_sends == sends);
		next = completed + (uint64_t)delay * 1000000000;
		fake_time = next - 1;
		report_frame(&hmd, 0, received);
		received += 2000000;
		CHECK(hid_sends == sends);
		delay = delay < 30 ? delay * 2 : 60;
	}
	CHECK(cache_reads == 1 && warning_logs == 2 && !hmd.touch_dev[0].have_calibration);
	clear_retry_fixture(&hmd);
}
static void naps_do_not_reset_failure_history(void)
{
	ohmd_context ctx = {0};
	rift_hmd_t hmd;
	retry_fixture(&hmd, &ctx);
	radio_json = cached_json;
	for (uint64_t wake = 1000000000; wake <= 301000000000; wake += 3000000000) {
		if (wake > 1000000000) {
			pkt_rift_radio_message beacon = {0};
			fake_time = wake - 2000000000;
			handle_touch_controller_message(&hmd, fake_time, &hmd.touch_dev[0], &beacon);
			fake_time = wake - 1000000000;
			handle_touch_controller_message(&hmd, fake_time, &hmd.touch_dev[0], &beacon);
		}
		report_frame(&hmd, 0, wake);
	}
	CHECK(hash_reads == 10 && flash_headers == 10 && cache_reads == 1);
	CHECK(warning_logs == 2 && !hmd.touch_dev[0].have_calibration);
	clear_retry_fixture(&hmd);
}
static void another_hash_can_use_its_valid_cache(void)
{
	ohmd_context ctx = {0};
	rift_hmd_t hmd;
	retry_fixture(&hmd, &ctx);
	flash_hash[0] = 0x11;
	radio_json = cached_json;
	report_frame(&hmd, 0, 1000000000);
	CHECK(!hmd.touch_dev[0].have_calibration && cache_reads == 1 && flash_headers == 1);
	flash_hash[0] = 0x22;
	cached_json = valid_json;
	flash_error = 1;
	/* Exactly one second later: no silence-based reset can help this case. */
	report_frame(&hmd, 0, 2000000000);
	CHECK(hmd.touch_dev[0].have_calibration && cache_reads == 2 && flash_headers == 1);
	CHECK(hash_reads == 2 && warning_logs == 1 && tracker_creations == 1);
	clear_retry_fixture(&hmd);
}
static void a_new_hash_is_observed_at_the_capped_retry(void)
{
	for (int valid = 0; valid <= 1; valid++) {
		ohmd_context ctx = {0};
		rift_hmd_t hmd;
		retry_fixture(&hmd, &ctx);
		flash_hash[0] = 0x31;
		radio_json = cached_json;
		for (uint64_t frame = 0; frame <= 31500; frame++)
			report_frame(&hmd, 0, 1000000000 + frame * 2000000);
		CHECK(hash_reads == 7 && cache_reads == 1 && flash_headers == 7);
		flash_hash[0] = 0x42;
		if (valid) { cached_json = valid_json; flash_error = 1; }
		/* Replacement is discovered at the next scheduled read, 60 s later. */
		for (uint64_t frame = 31501; frame < 61500; frame++) {
			report_frame(&hmd, 0, 1000000000 + frame * 2000000);
			CHECK(hash_reads == 7 && !hmd.touch_dev[0].have_calibration);
		}
		report_frame(&hmd, 0, 124000000000);
		CHECK(hash_reads == 8 && cache_reads == 2);
		CHECK(hmd.touch_dev[0].have_calibration == (bool)valid);
		if (valid) {
			CHECK(flash_headers == 7 && warning_logs == 2 && tracker_creations == 1);
		} else {
			CHECK(warning_logs == 3); /* only the new rejected cache: its first terminal failure stays quiet */
			report_frame(&hmd, 0, 124999999999);
			CHECK(hash_reads == 8);
			report_frame(&hmd, 0, 125000000000);
			CHECK(hash_reads == 9 && cache_reads == 2 && warning_logs == 4);
		}
		clear_retry_fixture(&hmd);
	}
}
static void a_truncated_hash_reply_does_not_confirm_an_identity_change(void)
{
	ohmd_context ctx = {0};
	rift_hmd_t hmd;
	retry_fixture(&hmd, &ctx);
	flash_hash[0] = 0x51;
	radio_json = cached_json;
	report_frame(&hmd, 0, 1000000000);
	short_data = 1;
	report_frame(&hmd, 0, 2000000000);
	CHECK(cache_reads == 1 && hash_reads == 1 && flash_headers == 1 && warning_logs == 2);
	short_data = 0;
	report_frame(&hmd, 0, 3999999999);
	CHECK(hash_reads == 1);
	report_frame(&hmd, 0, 4000000000);
	CHECK(cache_reads == 1 && hash_reads == 2 && flash_headers == 2 && warning_logs == 3);
	clear_retry_fixture(&hmd);
}
static void a_cache_allocation_failure_is_not_quarantined(void)
{
	for (int leds = 0; leds <= 1; leds++) {
		ohmd_context ctx = {0};
		rift_hmd_t hmd;
		retry_fixture(&hmd, &ctx);
		radio_json = cached_json;
		cached_json = valid_json;
		if (leds) fail_led_alloc = 1; else fail_cached_copy = 1;
		report_frame(&hmd, 0, 1000000000);
		CHECK(!hmd.touch_dev[0].have_calibration && owned == 0 && live_leds == 0);
		CHECK(hash_reads == 1 && cache_reads == 1 && flash_headers == 0 && warning_logs == 0);
		report_frame(&hmd, 0, 1999999999);
		CHECK(hash_reads == 1);
		report_frame(&hmd, 0, 2000000000);
		CHECK(hmd.touch_dev[0].have_calibration && cache_reads == 2 && hash_reads == 2);
		CHECK(flash_headers == 0 && warning_logs == 0 && tracker_creations == 1);
		clear_retry_fixture(&hmd);
	}
}
/* Exercise the engine's actual no-report update path, including haptics. */
static void engine_frame(rift_hmd_t *hmd, uint64_t time)
{
	fake_time = time;
	update_fixture = 1;
	hmd->handle = (hid_device *)(uintptr_t)1;
	hmd->sensor_config.keep_alive_interval = 60000;
	update_hmd(hmd);
}
static void pending_reads_finish_or_expire_without_reports(int expire)
{
	ohmd_context ctx = {0};
	rift_hmd_t hmd;
	retry_fixture(&hmd, &ctx);
	radio_json = valid_json;
	/* A bad cache starts a flash read; only its header remains pending. */
	pending_header = expire ? 1000000 : 2;
	report_frame(&hmd, 0, 1000000000);
	CHECK(hmd.radio.command_result_pending && !hmd.touch_dev[0].have_calibration);
	CHECK(cache_reads == 1 && hmd.touch_dev[0].calibration_state.cache_rejected);
	for (int hand = 0; hand < 2; hand++) {
		hmd.touch_dev[hand].haptic_state.haptics_on = true;
		hmd.touch_dev[hand].haptic_state.amplitude = 17 + hand;
		hmd.touch_dev[hand].haptic_state.end_time = UINT64_MAX;
		hmd.touch_dev[hand].haptic_state.duration = 0.1f;
	}
	/* No right-hand reports follow. The original command remains independently busy. */
	uint64_t deadline = expire ? 6000000000 : 2000000000;
	for (uint64_t time = 1002000000; time < deadline; time += 2000000)
		engine_frame(&hmd, time);
	if (expire) {
		CHECK(!hmd.touch_dev[0].have_calibration && haptic_writes == 0 && pending_polls > 0);
		engine_frame(&hmd, deadline);
		CHECK(haptic_writes == 0 && hmd.touch_dev[0].calibration_state.error == -ETIMEDOUT);
		CHECK(hmd.touch_dev[0].calibration_state.retry_time == 7000000000);
		int sent = hid_sends;
		for (uint64_t time = deadline + 2000000; time < deadline + 100000000; time += 2000000)
			engine_frame(&hmd, time);
		CHECK(hid_sends == sent && haptic_writes == 0); /* no unsafe replacement */
		pending_polls = 0; /* The original radio command finally completes. */
		engine_frame(&hmd, deadline + 100000000);
		CHECK(haptic_writes == 2);
	} else {
		CHECK(hmd.touch_dev[0].have_calibration && tracker_creations == 1);
	}
	cached_json = valid_json;
	report_frame(&hmd, 1, deadline + 102000000);
	CHECK(hmd.touch_dev[1].have_calibration);
	engine_frame(&hmd, deadline + 300000000);
	CHECK(haptic_writes == 4); /* one on and one off per hand; no duplicate sends */
	CHECK(hmd.radio.cur_read_cmd == RIFT_RADIO_CMD_NONE && !hmd.radio.command_result_pending);
	clear_retry_fixture(&hmd);
}
static void short_status_does_not_fetch_stale_data_or_confirm_haptics(void)
{
	for (int size = 1; size <= 3; size++) {
		ohmd_context ctx = {0};
		rift_hmd_t hmd;
		retry_fixture(&hmd, &ctx);
		radio_json = cached_json;
		flash_hash[0] = 0x51;
		report_frame(&hmd, 0, 1000000000);
		CHECK(hmd.touch_dev[0].calibration_state.hash[0] == 0x51);
		flash_hash[0] = 0x72;
		cached_json = valid_json;
		short_response = size + 1;
		int fetched = data_fetches;
		report_frame(&hmd, 0, 2000000000);
		CHECK(data_fetches == fetched && !hmd.touch_dev[0].have_calibration);
		CHECK(hmd.touch_dev[0].calibration_state.hash[0] == 0x51);
		CHECK(hmd.touch_dev[0].calibration_state.retry_delay == 2);
		CHECK(rift_touch_send_haptics(&ctx, &hmd.radio, RIFT_TOUCH_CONTROLLER_LEFT, false, 12) == -EIO);
		CHECK(data_fetches == fetched);
		clear_retry_fixture(&hmd);
	}
}
static void stale_numeric_errno_does_not_poison_valid_calibration(void)
{
	rift_touch_calibration calibration = {0};
	errno = ERANGE;
	CHECK(rift_touch_parse_calibration((char *)valid_json, &calibration, NULL) == 0);
	rift_touch_clear_calibration(&calibration);
	char number[] = "0.5";
	errno = ERANGE;
	int error;
	const nx_json *value = nx_json_parse_silent(number, NULL, &error);
	CHECK(value && value->type == NX_JSON_DOUBLE && value->dbl_value == 0.5);
	nx_json_free(value);
	ohmd_context ctx = {0};
	rift_hmd_t hmd;
	retry_fixture(&hmd, &ctx);
	cached_json = "{\"TrackedObject\":{\"JsonVersion\":2,\"ImuPosition\":[1e999,0,0]}}";
	radio_json = valid_json;
	report_frame(&hmd, 0, 1000000000);
	CHECK(hmd.touch_dev[0].have_calibration && hash_reads == 1 && flash_headers == 1);
	CHECK(cache_reads == 1 && cache_writes == 1);
	clear_retry_fixture(&hmd);
}
static void cache_reader_oom_uses_the_backoff(void)
{
	ohmd_context ctx = {0};
	rift_hmd_t hmd;
	retry_fixture(&hmd, &ctx);
	cached_json = radio_json = valid_json;
	cache_error = 1;
	report_frame(&hmd, 0, 1000000000);
	CHECK(!hmd.touch_dev[0].have_calibration && flash_headers == 0);
	CHECK(hmd.touch_dev[0].calibration_state.error == -ENOMEM);
	CHECK(!hmd.touch_dev[0].calibration_state.cache_rejected);
	report_frame(&hmd, 0, 1999999999);
	CHECK(hash_reads == 1 && cache_reads == 1);
	report_frame(&hmd, 0, 2000000000);
	CHECK(hmd.touch_dev[0].have_calibration && cache_reads == 2 && flash_headers == 0);
	clear_retry_fixture(&hmd);
}
static void hid_failures_report_io_instead_of_permissions(void)
{
	for (int get = 0; get <= 1; get++) {
		ohmd_context ctx = {0};
		rift_hmd_t hmd;
		retry_fixture(&hmd, &ctx);
		radio_json = valid_json;
		if (get) get_error = 1;
		else {
			hmd.radio.cur_read_cmd = RIFT_FW_READ_CMD_CALIBRATION_HDR;
			hmd.radio.device_in_progress = RIFT_TOUCH_CONTROLLER_RIGHT;
			flash_error = 1;
		}
		report_frame(&hmd, 0, 1000000000);
		CHECK(hmd.touch_dev[0].calibration_state.error == -EIO);
		CHECK(warning_logs == 0);
		report_frame(&hmd, 0, 2000000000);
		CHECK(strstr(diagnostics, strerror(EIO)) != NULL);
		CHECK(strstr(diagnostics, strerror(EPERM)) == NULL);
		clear_retry_fixture(&hmd);
	}
}
static void node_allocation_failures_unwind_and_retry_the_cache(void)
{
	rift_touch_calibration calibration = {0};
	node_allocations = 0;
	CHECK(rift_touch_parse_calibration((char *)valid_json, &calibration, NULL) == 0);
	int allocations = node_allocations;
	CHECK(allocations > 20);
	unsigned char before[sizeof calibration];
	memcpy(before, &calibration, sizeof before);
	for (int node = 0; node < allocations; node++) {
		fail_node_after = node;
		CHECK(rift_touch_parse_calibration((char *)valid_json, &calibration, NULL) == -ENOMEM);
		CHECK(fail_node_after == -1 && owned == 0 && live_nodes == 0 && live_leds == 1);
		CHECK(memcmp(before, &calibration, sizeof before) == 0);
	}
	rift_touch_clear_calibration(&calibration);
	/* Also cover every nxjson node type, including types absent from this schema. */
	const char *types[] = {"{}", "[]", "\"text\"", "1", "true", "false", "null"};
	for (size_t i = 0; i < sizeof types / sizeof types[0]; i++) {
		char text[16]; strcpy(text, types[i]);
		int error;
		fail_node_after = 0;
		CHECK(nx_json_parse_silent(text, NULL, &error) == NULL && error == -ENOMEM);
		CHECK(live_nodes == 0);
		strcpy(text, types[i]);
		fail_node_after = 0;
		CHECK(nx_json_parse(text, NULL) == NULL && live_nodes == 0);
	}
	/* A root node already exists when this child's allocation fails. A missing
	 * null child must not turn into a successful partial object. */
	char nested[] = "{\"unused\":null}";
	int error;
	fail_node_after = 1;
	CHECK(nx_json_parse_silent(nested, NULL, &error) == NULL && error == -ENOMEM);
	CHECK(live_nodes == 0);
	ohmd_context ctx = {0};
	rift_hmd_t hmd;
	retry_fixture(&hmd, &ctx);
	cached_json = radio_json = valid_json;
	fail_node_after = 4;
	report_frame(&hmd, 0, 1000000000);
	CHECK(!hmd.touch_dev[0].have_calibration && owned == 0 && live_nodes == 0);
	CHECK(hmd.touch_dev[0].calibration_state.error == -ENOMEM);
	CHECK(!hmd.touch_dev[0].calibration_state.cache_rejected && flash_headers == 0);
	report_frame(&hmd, 0, 2000000000);
	CHECK(hmd.touch_dev[0].have_calibration && cache_reads == 2 && flash_headers == 0);
	clear_retry_fixture(&hmd);
}
static void replace_fixture(char *out, const char *needle, const char *replacement)
{
	const char *p = strstr(valid_json, needle);
	CHECK(p != NULL);
	int result = snprintf(out, sizeof valid_json + 64, "%.*s%s%s",
		(int)(p - valid_json), valid_json, replacement, p + strlen(needle));
	CHECK(result >= 0 && result < (int)sizeof valid_json + 64);
}
static void a_missing_root_is_not_a_successful_parse(void)
{
	const char *empty[] = { "]", " ]", ",]", "/**/]" };
	for (size_t i = 0; i < sizeof empty / sizeof empty[0]; i++) {
		char text[32]; strcpy(text, empty[i]);
		int error = 0;
		CHECK(nx_json_parse_silent(text, NULL, &error) == NULL && error == -EINVAL);
		CHECK(live_nodes == 0);
	}
	rift_touch_calibration calibration = {0};
	CHECK(rift_touch_parse_calibration((char *)valid_json, &calibration, NULL) == 0);
	unsigned char before[sizeof calibration]; memcpy(before, &calibration, sizeof before);
	const char *bad[] = { "]", "{}", "[]", "null", "0", "true", "\"text\"",
		"{\"TrackedObject\":{}}", "[ {\"TrackedObject\":{}} ]" };
	for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++) {
		CHECK(rift_touch_parse_calibration((char *)bad[i], &calibration, NULL) == -EINVAL);
		CHECK(memcmp(before, &calibration, sizeof before) == 0);
		CHECK(owned == 0 && live_nodes == 0 && live_leds == 1);
	}
	rift_touch_clear_calibration(&calibration);
}
static void required_decoder_fields_are_validated(void)
{
	const char *keys[] = { "TrackedObject", "JsonVersion", "ImuPosition",
		"JoyXRangeMin", "JoyXRangeMax",
		"JoyYRangeMin", "JoyYRangeMax",
		"TriggerMinRange", "TriggerMidRange", "TriggerMaxRange",
		"MiddleMinRange", "MiddleMidRange", "MiddleMaxRange",
		"GyroCalibration", "AccCalibration", "CapSenseMin", "CapSenseTouch", "ModelPoints", "Point0" };
	rift_touch_calibration calibration = {0};
	CHECK(rift_touch_parse_calibration((char *)valid_json, &calibration, NULL) == 0);
	unsigned char before[sizeof calibration]; memcpy(before, &calibration, sizeof before);
	for (size_t i = 0; i < sizeof keys / sizeof keys[0]; i++) {
		char text[sizeof valid_json]; strcpy(text, valid_json);
		char needle[64]; snprintf(needle, sizeof needle, "\"%s\":", keys[i]);
		char *field = strstr(text, needle); CHECK(field != NULL);
		field[1] = '!'; /* Missing the named field; the rest of the object is unchanged. */
		CHECK(rift_touch_parse_calibration(text, &calibration, NULL) == -EINVAL);
		CHECK(memcmp(before, &calibration, sizeof before) == 0);
	}
	const char *changes[][2] = {
		{ "\"JoyXRangeMin\":0,\"JoyXRangeMax\":100,\"JoyXDeadMin\":40,\"JoyXDeadMax\":60", "\"JoyXRangeMin\":0,\"JoyXRangeMax\":0,\"JoyXDeadMin\":0,\"JoyXDeadMax\":0" },
		{ "\"JoyYRangeMin\":0,\"JoyYRangeMax\":100,\"JoyYDeadMin\":40,\"JoyYDeadMax\":60", "\"JoyYRangeMin\":0,\"JoyYRangeMax\":0,\"JoyYDeadMin\":0,\"JoyYDeadMax\":0" },
		{ "\"CapSenseMin\":[0,0,0,0,0,0,0,0]", "\"CapSenseMin\":[100,0,0,0,0,0,0,0]" },
		{ "\"JsonVersion\":2", "\"JsonVersion\":4294967298" },
		{ "\"JsonVersion\":2", "\"JsonVersion\":2.0" },
		{ "\"Point0\":[0,0,0,0,1,0]", "\"Point0\":[false,0,0,0,1,0]" },
		{ "\"JoyXRangeMax\":100", "\"JoyXRangeMax\":65636" },
		{ "\"TriggerMinRange\":0", "\"TriggerMinRange\":-65536" },
		{ "\"CapSenseMin\":[0,0,0,0,0,0,0,0]", "\"CapSenseMin\":[false,0,0,0,0,0,0,0]" },
		{ "\"CapSenseTouch\":[100,100,100,100,0,0,0,100]", "\"CapSenseTouch\":[65636,100,100,100,0,0,0,100]" },
		{ "\"JoyXRangeMin\":0", "\"JoyXRangeMin\":100" },
		{ "\"JoyYRangeMin\":0", "\"JoyYRangeMin\":100" },
		{ "\"TriggerMinRange\":0", "\"TriggerMinRange\":50" },
		{ "\"TriggerMaxRange\":100", "\"TriggerMaxRange\":50" },
		{ "\"MiddleMinRange\":0", "\"MiddleMinRange\":50" },
		{ "\"MiddleMaxRange\":100", "\"MiddleMaxRange\":50" },
		{ "\"JoyXRangeMax\":100", "\"JoyXRangeMax\":65536" },
		{ "\"TriggerMinRange\":0", "\"TriggerMinRange\":-1" },
		{ "\"MiddleMidRange\":50", "\"MiddleMidRange\":true" },
		{ "\"ImuPosition\":[0,0,0]", "\"ImuPosition\":[0,0]" },
		{ "\"ImuPosition\":[0,0,0]", "\"ImuPosition\":[0,0,1e100]" },
		{ "\"GyroCalibration\":[1,0,0,0,1,0,0,0,1,0,0,0]", "\"GyroCalibration\":[1,0,0]" },
		{ "\"AccCalibration\":[1,0,0,0,1,0,0,0,1,0,0,0]", "\"AccCalibration\":[0,0,0,0,0,0,0,0,0,0,0,0]" },
		{ "\"CapSenseMin\":[0,0,0,0,0,0,0,0]", "\"CapSenseMin\":[0,0,0,0,0,0,0]" },
		{ "\"CapSenseTouch\":[100,100,100,100,0,0,0,100]", "\"CapSenseTouch\":[0,100,100,100,0,0,0,100]" },
		{ "\"CapSenseTouch\":[100,100,100,100,0,0,0,100]", "\"CapSenseTouch\":[100,100,100,100,0,0,0,0]" },
		{ "\"ModelPoints\":{\"Point0\":[0,0,0,0,1,0]}", "\"ModelPoints\":{}" },
		{ "\"Point0\":[0,0,0,0,1,0]", "\"Point0\":[0,0,0,0,1]" },
		{ "\"Point0\":[0,0,0,0,1,0]", "\"Point0\":[0,0,0,0,0,0]" },
		{ "\"Point0\":[0,0,0,0,1,0]", "\"Point0\":[0,0,0,0,true,0]" },
	};
	for (size_t i = 0; i < sizeof changes / sizeof changes[0]; i++) {
		char text[sizeof valid_json + 64]; replace_fixture(text, changes[i][0], changes[i][1]);
		CHECK(rift_touch_parse_calibration(text, &calibration, NULL) == -EINVAL);
		CHECK(memcmp(before, &calibration, sizeof before) == 0);
		CHECK(owned == 0 && live_nodes == 0 && live_leds == 1);
	}
	/* Every consumed capacitive channel needs a nonzero denominator. Unused
	 * channels 4–6 may retain the factory's zero values, as in valid_json. */
	for (int channel = 0; channel < 8; channel++) {
		if (channel >= 4 && channel <= 6) continue;
		char array[80];
		snprintf(array, sizeof array, "\"CapSenseTouch\":[%d,%d,%d,%d,0,0,0,%d]",
			channel == 0 ? 0 : 100, channel == 1 ? 0 : 100, channel == 2 ? 0 : 100,
			channel == 3 ? 0 : 100, channel == 7 ? 0 : 100);
		char text[sizeof valid_json + 64]; replace_fixture(text, "\"CapSenseTouch\":[100,100,100,100,0,0,0,100]", array);
		CHECK(rift_touch_parse_calibration(text, &calibration, NULL) == -EINVAL);
	}
	rift_touch_clear_calibration(&calibration);
}
static void an_empty_cached_root_preserves_the_model_and_falls_back(void)
{
	ohmd_context ctx = {0}; rift_hmd_t hmd; retry_fixture(&hmd, &ctx);
	CHECK(rift_touch_parse_calibration((char *)valid_json, &hmd.touch_dev[0].calibration, NULL) == 0);
	unsigned char before[sizeof hmd.touch_dev[0].calibration];
	memcpy(before, &hmd.touch_dev[0].calibration, sizeof before);
	cached_json = "]"; radio_json = valid_json; pending_header = 1;
	report_frame(&hmd, 0, 1000000000);
	CHECK(!hmd.touch_dev[0].have_calibration && hmd.touch_dev[0].calibration_state.cache_rejected);
	CHECK(memcmp(before, &hmd.touch_dev[0].calibration, sizeof before) == 0);
	engine_frame(&hmd, 1002000000);
	CHECK(hmd.touch_dev[0].have_calibration && cache_reads == 1 && cache_writes == 1);
	CHECK(hash_reads == 1 && flash_headers == 1);
	clear_retry_fixture(&hmd);
}
static void an_empty_radio_root_uses_the_backoff(void)
{
	ohmd_context ctx = {0}; rift_hmd_t hmd; retry_fixture(&hmd, &ctx);
	radio_json = "]";
	report_frame(&hmd, 0, 1000000000);
	CHECK(!hmd.touch_dev[0].have_calibration && hmd.touch_dev[0].calibration_state.error == -EINVAL);
	CHECK(hmd.touch_dev[0].calibration_state.retry_time == 2000000000);
	report_frame(&hmd, 0, 1999999999); CHECK(hash_reads == 1);
	radio_json = valid_json;
	report_frame(&hmd, 0, 2000000000);
	CHECK(hmd.touch_dev[0].have_calibration && hash_reads == 2);
	clear_retry_fixture(&hmd);
}
static void a_long_download_keeps_making_validated_progress(void)
{
	ohmd_context ctx = {0}; rift_hmd_t hmd; retry_fixture(&hmd, &ctx);
	char padded[8192];
	memset(padded, ' ', 4096);
	memcpy(padded + 4096, valid_json, sizeof valid_json);
	radio_json = padded; cached_json = "]"; independent_radio = 2;
	report_frame(&hmd, 0, 1000000000);
	for (uint64_t time = 1002000000; time <= 11000000000 && !hmd.touch_dev[0].have_calibration; time += 2000000)
		engine_frame(&hmd, time);
	CHECK(hmd.touch_dev[0].have_calibration && fake_time > 7000000000);
	CHECK(transaction_count > 200 && cache_reads == 1 && cache_writes == 1);
	fprintf(stderr, "synthetic download: %zu bytes, %d commands, %.3f s virtual time (30 ms per command)\n",
		strlen(padded), transaction_count, (double)(fake_time - 1000000000) / 1000000000);
	CHECK(hmd.touch_dev[0].calibration_state.error == 0 && tracker_creations == 1);
	clear_retry_fixture(&hmd);
}
static void recovery_drains_before_sending_and_leaves_no_extra_reply(void)
{
	for (int haptic = 0; haptic <= 1; haptic++) {
		for (int hand = 0; hand < 2; hand++) {
			ohmd_context ctx = {0}; rift_hmd_t hmd; retry_fixture(&hmd, &ctx);
			independent_radio = 3; flash_hash[0] = 0x51; radio_json = valid_json;
			report_frame(&hmd, 0, 1000000000);
			CHECK(transaction_count == 2);
			engine_frame(&hmd, 6000000000);
			CHECK(hmd.touch_dev[0].calibration_state.error == -ETIMEDOUT);
			flash_hash[0] = 0x72; cached_json = valid_json;
			rift_touch_controller_t *touch = &hmd.touch_dev[hand];
			fake_time = 6500000000;
			if (haptic) CHECK(rift_touch_send_haptics(&ctx, &hmd.radio, touch->device_num, false, 37) == -EBUSY);
			else if (hand) report_frame(&hmd, hand, fake_time);
			else CHECK(rift_touch_get_calibration(&ctx, &hmd.radio, touch->device_num,
				&touch->calibration, &touch->calibration_state) == -EBUSY);
			CHECK(transaction_count == 2 && !transactions[1].status_read);
			fake_time = 6600000000;
			int ret = haptic ? rift_touch_send_haptics(&ctx, &hmd.radio, touch->device_num, false, 37) :
				rift_touch_get_calibration(&ctx, &hmd.radio, touch->device_num, &touch->calibration, &touch->calibration_state);
			CHECK(ret == -EINPROGRESS && transaction_count == 3);
			CHECK(transactions[1].status_read && transactions[1].data_read);
			fake_time = 6630000000;
			ret = haptic ? rift_touch_send_haptics(&ctx, &hmd.radio, touch->device_num, false, 37) :
				rift_touch_get_calibration(&ctx, &hmd.radio, touch->device_num, &touch->calibration, &touch->calibration_state);
			CHECK(ret == 0 && transaction_count == 3);
			if (!haptic) CHECK(touch->calibration_state.hash[0] == 0x72);
			/* A distinct next hand/hash must not consume a duplicate replacement. */
			flash_hash[0] = 0x93;
			rift_touch_controller_t *other = &hmd.touch_dev[1 - hand];
			CHECK(rift_touch_get_calibration(&ctx, &hmd.radio, other->device_num,
				&other->calibration, &other->calibration_state) == -EINPROGRESS);
			CHECK(transaction_count == 4);
			fake_time = 6660000000;
			CHECK(rift_touch_get_calibration(&ctx, &hmd.radio, other->device_num,
				&other->calibration, &other->calibration_state) == 0);
			CHECK(other->calibration_state.hash[0] == 0x93 && transaction_count == 4);
			CHECK(rift_touch_send_haptics(&ctx, &hmd.radio, other->device_num, false, 19) == -EINPROGRESS);
			fake_time = 6690000000;
			CHECK(rift_touch_send_haptics(&ctx, &hmd.radio, other->device_num, false, 19) == 0);
			CHECK(transaction_count == 5);
			clear_retry_fixture(&hmd);
		}
	}
}
static void optional_factory_fields_keep_the_upstream_defaults(void)
{
	char json[sizeof valid_json + 64];
	replace_fixture(json, "\"GyroCalibration\":[1,0,0,0,1,0,0,0,1,0,0,0]", "\"GyroCalibration\":[1,0,0,0,1,0,0,0,1]");
	const char *optional[] = {"JoyXDeadMin", "JoyXDeadMax", "JoyYDeadMin", "JoyYDeadMax"};
	for (size_t i = 0; i < sizeof optional / sizeof optional[0]; i++) {
		char *key = strstr(json, optional[i]); CHECK(key); key[0] = '!';
	}
	/* Ignored capacitive slots can contain any JSON type. */
	replace_fixture(json, "\"CapSenseMin\":[0,0,0,0,0,0,0,0]", "\"CapSenseMin\":[0,0,0,0,null,false,{},0]");
	rift_touch_calibration calibration = {0};
	CHECK(rift_touch_parse_calibration(json, &calibration, NULL) == 0);
	rift_touch_clear_calibration(&calibration);
	/* Both arrays ignore their unused channels; partial offsets retain numeric
	 * entries and default only the missing suffix to zero. */
	replace_fixture(json, "\"CapSenseTouch\":[100,100,100,100,0,0,0,100]", "\"CapSenseTouch\":[100,100,100,100,null,{},false,100]");
	CHECK(rift_touch_parse_calibration(json, &calibration, NULL) == 0);
	rift_touch_clear_calibration(&calibration);
	const char *matrices[] = {"GyroCalibration", "AccCalibration"};
	for (unsigned int k = 0; k < 2; k++) {
		char old[128], replacement[128];
		snprintf(old, sizeof old, "\"%s\":[1,0,0,0,1,0,0,0,1,0,0,0]", matrices[k]);
		for (unsigned int offsets = 0; offsets < 3; offsets++) {
			snprintf(replacement, sizeof replacement, "\"%s\":[1,0,0,0,1,0,0,0,1%s]", matrices[k], offsets == 0 ? "" : offsets == 1 ? ",3" : ",3,4");
			replace_fixture(json, old, replacement);
			rift_touch_calibration_info info;
			CHECK(rift_touch_parse_calibration(json, &calibration, &info) == 0);
			CHECK(info.defaults & (1u << k));
			vec3f offset = k ? calibration.accel_offset : calibration.gyro_offset;
			CHECK(offset.x == (offsets ? 3 : 0) && offset.y == (offsets == 2 ? 4 : 0) && offset.z == 0);
			rift_touch_clear_calibration(&calibration);
		}
	}
	/* Now exercise the warnings through the production loader. */
	replace_fixture(json, "\"GyroCalibration\":[1,0,0,0,1,0,0,0,1,0,0,0]", "\"GyroCalibration\":[1,0,0,0,1,0,0,0,1]");
	for (size_t i = 0; i < sizeof optional / sizeof optional[0]; i++) {
		char *key = strstr(json, optional[i]); CHECK(key); key[0] = '!';
	}
	ohmd_context ctx = {0}; rift_hmd_t hmd; retry_fixture(&hmd, &ctx);
	cached_json = radio_json = json;
	report_frame(&hmd, 0, 1000000000);
	CHECK(hmd.touch_dev[0].have_calibration && warning_logs == 1);
	CHECK(strstr(diagnostics, "Right") && strstr(diagnostics, "cache") && strstr(diagnostics, "default"));
	CHECK(hmd.touch_dev[0].calibration.gyro_offset.x == 0);
	CHECK(hmd.touch_dev[0].calibration.joy_x_dead_min > hmd.touch_dev[0].calibration.joy_x_dead_max);
	CHECK(hmd.touch_dev[0].calibration.joy_y_dead_min > hmd.touch_dev[0].calibration.joy_y_dead_max);
	for (uint64_t time = 1002000000; time < 1100000000; time += 2000000) report_frame(&hmd, 0, time);
	CHECK(warning_logs == 1);
	hmd.touch_dev[0].have_calibration = false;
	report_frame(&hmd, 0, 1200000000);
	CHECK(hmd.touch_dev[0].have_calibration && warning_logs == 1);
	clear_retry_fixture(&hmd);
}
static void failure_diagnostics_name_the_hand_source_field_and_next_step(void)
{
	char bad[sizeof valid_json + 64];
	replace_fixture(bad, "\"TriggerMidRange\":50", "\"TriggerMidRange\":0");
	ohmd_context ctx = {0}; rift_hmd_t hmd; retry_fixture(&hmd, &ctx);
	cached_json = radio_json = bad;
	report_frame(&hmd, 0, 1000000000);
	CHECK(!hmd.touch_dev[0].have_calibration);
	CHECK(strstr(diagnostics, "cache"));
	CHECK(!strstr(diagnostics, "cv1touch report"));
	report_frame(&hmd, 0, 2000000000);
	const char *message = diagnostics, *next;
	while ((next = strstr(message + 1, "[WW] ")) != NULL) message = next;
	CHECK(strstr(message, "Right") && strstr(message, "radio"));
	CHECK(strstr(message, "Trigger") && strstr(message, "range") && strstr(message, "2 s"));
	CHECK(strstr(message, "cv1touch report") && strstr(message, "attach"));
	CHECK(!strstr(diagnostics, "TrackedObject") && !strstr(diagnostics, "\""));
	int warnings = warning_logs;
	for (uint64_t time = 2002000000; time < 10000000000; time += 2000000) report_frame(&hmd, 0, time);
	CHECK(warning_logs == warnings);
	/* A different field with the same errno is a real failure transition. */
	replace_fixture(bad, "\"MiddleMidRange\":50", "\"MiddleMidRange\":0");
	diagnostics[0] = 0;
	report_frame(&hmd, 0, 16000000000);
	CHECK(warning_logs == warnings + 1 && strstr(diagnostics, "MiddleRange"));
	clear_retry_fixture(&hmd);
}
/* A haptics write whose status cannot be read may still complete later. The next command,
 * a haptics write or the other hand's hash read, waits for that completion and is sent itself:
 * it must never be skipped and given the old command's result. */
static void a_failed_haptics_status_leaves_no_reply_for_the_next_command(void)
{
	for (int other_hand = 0; other_hand <= 1; other_hand++) {
		ohmd_context ctx = {0}; rift_hmd_t hmd; retry_fixture(&hmd, &ctx);
		independent_radio = 2; /* every command completes 30 ms after it is sent */
		flash_hash[0] = 0x51; cached_json = valid_json;
		rift_haptic_state *hs = &hmd.touch_dev[0].haptic_state;
		hs->sent_amplitude = 20; /* the controller is vibrating */
		hs->haptics_on = true; hs->amplitude = 37; hs->duration = 0.1f; hs->end_time = UINT64_MAX;
		fail_status_reads = 1;
		engine_frame(&hmd, 1000000000);
		CHECK(transaction_count == 1 && transactions[0].data[7] == 37);
		CHECK(hs->failures == 1 && hs->sent_amplitude == 20 && hmd.radio.cur_read_cmd == RIFT_RADIO_CMD_NONE);
		if (!other_hand) hs->haptics_on = false; /* the vibration is stopped before the retry */
		/* The first write completes at 1.030 s; until its result is drained, send nothing. */
		if (other_hand) report_frame(&hmd, 1, 1012000000); else engine_frame(&hmd, 1012000000);
		CHECK(transaction_count == 1 && hs->sent_amplitude == 20);
		if (other_hand) report_frame(&hmd, 1, 1112000000); else engine_frame(&hmd, 1112000000);
		CHECK(transactions[0].status_read && transactions[0].data_read);
		CHECK(transaction_count == 2); /* the next command, sent by itself */
		CHECK(hs->sent_amplitude == 20 && !hmd.touch_dev[1].have_calibration);
		if (other_hand) {
			CHECK(transactions[1].data[7] == 0x51); /* its own hash read */
			report_frame(&hmd, 1, 1147000000);
			CHECK(hmd.touch_dev[1].have_calibration && hmd.touch_dev[1].calibration_state.hash[0] == 0x51);
			engine_frame(&hmd, 1180000000); /* now the interrupted write, sent again */
			CHECK(transaction_count == 3 && transactions[2].data[7] == 37);
			engine_frame(&hmd, 1220000000);
			CHECK(hs->sent_amplitude == 37);
		} else {
			CHECK(transactions[1].data[7] == 0); /* its own off write */
			engine_frame(&hmd, 1147000000);
			CHECK(hs->sent_amplitude == 0 && hs->failures == 0 && !hs->dirty);
		}
		CHECK(transactions[1].status_read && hmd.radio.cur_read_cmd == RIFT_RADIO_CMD_NONE);
		CHECK(!hmd.radio.command_result_pending && !hmd.radio.discard_reply);
		clear_retry_fixture(&hmd);
	}
}
/* The engine can stall (a blocked USB call, a suspend). A command that finished meanwhile is not
 * abandoned for having waited: it is read, and the next block gets its own 5 s. */
static void a_command_that_finished_during_a_stall_is_not_abandoned(void)
{
	ohmd_context ctx = {0}; rift_hmd_t hmd; retry_fixture(&hmd, &ctx);
	radio_json = valid_json; cached_json = "]"; independent_radio = 2;
	report_frame(&hmd, 0, 1000000000);
	CHECK(transaction_count == 1 && !hmd.touch_dev[0].have_calibration);
	engine_frame(&hmd, 7000000000);
	CHECK(hmd.touch_dev[0].calibration_state.error == 0 && transaction_count == 2);
	CHECK(hmd.touch_dev[0].calibration_state.hash[0] == flash_hash[0] && !hmd.radio.discard_reply);
	for (uint64_t time = 7002000000; time < 12000000000 && !hmd.touch_dev[0].have_calibration; time += 2000000)
		engine_frame(&hmd, time);
	CHECK(hmd.touch_dev[0].have_calibration && tracker_creations == 1);
	CHECK(hmd.touch_dev[0].calibration_state.error == 0 && warning_logs == 1 && !strstr(diagnostics, "within 5 s"));
	clear_retry_fixture(&hmd);
}
/* A read that gave up leaves a reply to drain. If draining fails, that is what is reported,
 * not the timeout that came before it. The messages are meant to be read by the user. */
static void a_failed_drain_is_reported_as_itself(void)
{
	ohmd_context ctx = {0}; rift_hmd_t hmd; retry_fixture(&hmd, &ctx);
	radio_json = valid_json; hash_timeouts = 1;
	report_frame(&hmd, 0, 1000000000);
	CHECK(warning_logs == 0);
	pending_header = 1000000;
	report_frame(&hmd, 0, 2000000000);
	CHECK(hmd.radio.command_result_pending && !hmd.touch_dev[0].have_calibration);
	engine_frame(&hmd, 7000000000);
	rift_touch_calibration_state *state = &hmd.touch_dev[0].calibration_state;
	CHECK(state->error == -ETIMEDOUT && state->retry_time == 9000000000);
	const char *message = diagnostics, *next;
	while ((next = strstr(message + 1, "[WW] ")) != NULL) message = next;
	CHECK(strstr(message, "Right") && strstr(message, "radio") && strstr(message, "no reply from the controller within 5 s"));
	CHECK(strstr(message, "cv1touch report") && strstr(message, "bug report"));
	CHECK(!strstr(message, "validated") && !strstr(message, "attach it"));
	short_response = 2; /* the radio cannot be asked whether the old command finished */
	int warnings = warning_logs;
	diagnostics[0] = 0;
	report_frame(&hmd, 0, 9000000000);
	CHECK(state->error == -EIO && warning_logs == warnings + 1 && hmd.radio.discard_reply);
	CHECK(strstr(diagnostics, "Right controller radio calibration: previous radio command") && !strstr(diagnostics, "5 s"));
	clear_retry_fixture(&hmd);
}
/* Saving the cache is optional: when it fails, the calibration just read stays in use and is not read again. */
static void a_failed_cache_save_keeps_the_calibration(void)
{
	ohmd_context ctx = {0}; rift_hmd_t hmd; retry_fixture(&hmd, &ctx);
	radio_json = valid_json; set_config_error = -ENOSPC;
	report_frame(&hmd, 0, 1000000000);
	CHECK(hmd.touch_dev[0].have_calibration && cache_writes == 1 && tracker_creations == 1);
	CHECK(hmd.touch_dev[0].calibration_state.error == 0 && warning_logs == 1); /* only the rejected cache */
	CHECK(hash_reads == 1 && flash_headers == 1);
	report_frame(&hmd, 0, 1002000000);
	CHECK(hash_reads == 1 && flash_headers == 1 && cache_writes == 1 && imu_updates == 2);
	clear_retry_fixture(&hmd);
}
/* Exercise the shared drain through both calibration and haptics, at report rate. */
static void stuck_drain_fixture(rift_hmd_t *hmd, ohmd_context *ctx)
{
	retry_fixture(hmd, ctx);
	radio_json = valid_json;
	pending_header = 1000000;
	report_frame(hmd, 0, 1000000000);
	engine_frame(hmd, 6000000000);
	CHECK(hmd->radio.discard_reply && !hmd->radio.command_result_pending);
	cached_json = valid_json;
	warning_logs = 0;
	diagnostics[0] = 0;
}
static void a_busy_drain_is_shared_and_spaced_between_reports(void)
{
	ohmd_context ctx = {0}; rift_hmd_t hmd; stuck_drain_fixture(&hmd, &ctx);
	int before = status_reads, sent = hid_sends;
	for (uint64_t time = 7000000000; time < 9000000000; time += 2000000) {
		for (int hand = 0; hand < 2; hand++) {
			report_frame(&hmd, hand, time);
			CHECK(rift_touch_send_haptics(&ctx, &hmd.radio, hmd.touch_dev[hand].device_num, false, 17) == -EBUSY);
		}
	}
	CHECK(status_reads - before == 20 && hid_sends == sent && warning_logs == 0);
	/* Completing just after a poll waits at most another 100 ms, then resumes safely. */
	pending_polls = 0;
	report_frame(&hmd, 1, 8999999999);
	CHECK(!hmd.touch_dev[1].have_calibration && status_reads - before == 20);
	report_frame(&hmd, 1, 9000000000);
	CHECK(hmd.touch_dev[1].have_calibration && !hmd.radio.discard_reply);
	for (int hand = 0; hand < 2; hand++)
		CHECK(rift_touch_send_haptics(&ctx, &hmd.radio, hmd.touch_dev[hand].device_num, false, 17) == 0);
	CHECK(haptic_writes == 2);
	clear_retry_fixture(&hmd);
}
static void slow_or_failed_drain_polls_wait_from_completion(void)
{
	for (int error = 0; error <= 1; error++) {
		ohmd_context ctx = {0}; rift_hmd_t hmd; stuck_drain_fixture(&hmd, &ctx);
		short_response = error ? 2 : 0;
		hid_delay_ns = 300000000;
		fake_time = 7000000000;
		int before = status_reads;
		CHECK(rift_touch_send_haptics(&ctx, &hmd.radio, RIFT_TOUCH_CONTROLLER_LEFT, false, 17) == (error ? -EIO : -EBUSY));
		uint64_t completed = fake_time;
		for (int poll = 0; poll < 100; poll++) {
			report_frame(&hmd, poll % 2, completed);
			CHECK(rift_touch_send_haptics(&ctx, &hmd.radio, RIFT_TOUCH_CONTROLLER_LEFT, false, 17) == -EBUSY);
		}
		CHECK(status_reads == before + 1 && fake_time == completed);
		fake_time = completed + 99999999;
		CHECK(rift_touch_send_haptics(&ctx, &hmd.radio, RIFT_TOUCH_CONTROLLER_RIGHT, false, 17) == -EBUSY);
		CHECK(status_reads == before + 1);
		fake_time++;
		CHECK(rift_touch_send_haptics(&ctx, &hmd.radio, RIFT_TOUCH_CONTROLLER_RIGHT, false, 17) == (error ? -EIO : -EBUSY));
		CHECK(status_reads == before + 2);
		clear_retry_fixture(&hmd);
	}
}
static void a_long_drain_warns_once_and_a_later_drain_can_warn_again(void)
{
	ohmd_context ctx = {0}; rift_hmd_t hmd; stuck_drain_fixture(&hmd, &ctx);
	for (int episode = 0; episode < 2; episode++) {
		uint64_t start = 7000000000 + (uint64_t)episode * 30000000000;
		for (uint64_t time = start; time < start + 10000000000; time += 2000000) {
			fake_time = time;
			CHECK(rift_touch_send_haptics(&ctx, &hmd.radio, RIFT_TOUCH_CONTROLLER_LEFT, false, 17) == -EBUSY);
		}
		CHECK(warning_logs == episode);
		for (uint64_t time = start + 10000000000; time < start + 20000000000; time += 2000000) {
			fake_time = time;
			CHECK(rift_touch_send_haptics(&ctx, &hmd.radio, RIFT_TOUCH_CONTROLLER_RIGHT, false, 17) == -EBUSY);
		}
		CHECK(warning_logs == episode + 1);
		CHECK(strstr(diagnostics, "radio is still finishing an earlier request"));
		CHECK(strstr(diagnostics, "controllers cannot load until it does"));
		CHECK(!strstr(diagnostics, "TrackedObject") && !strstr(diagnostics, "cv1touch report"));
		pending_polls = 0;
		fake_time = start + 20000000000;
		CHECK(rift_touch_send_haptics(&ctx, &hmd.radio, RIFT_TOUCH_CONTROLLER_RIGHT, false, 17) == 0);
		CHECK(!hmd.radio.discard_reply);
		/* Another actual write fails to read its status and leaves a new reply to drain. */
		fake_time = start + 29999999999;
		fail_status_reads = 1;
		CHECK(rift_touch_send_haptics(&ctx, &hmd.radio, RIFT_TOUCH_CONTROLLER_RIGHT, false, 0) == -EIO);
		rift_touch_cancel_in_progress(&hmd.radio, RIFT_TOUCH_CONTROLLER_RIGHT);
		pending_polls = 1000000;
	}
	clear_retry_fixture(&hmd);
}
static void one_wake_up_timeout_recovers_without_a_warning(void)
{
	ohmd_context ctx = {0}; rift_hmd_t hmd; retry_fixture(&hmd, &ctx);
	cached_json = radio_json = valid_json;
	hash_timeouts = 1;
	report_frame(&hmd, 0, 1000000000);
	CHECK(!hmd.touch_dev[0].have_calibration && hmd.touch_dev[0].calibration_state.retry_time == 2000000000);
	CHECK(warning_logs == 0 && !strstr(diagnostics, "cv1touch report"));
	report_frame(&hmd, 0, 1999999999);
	CHECK(hash_timeouts == 0 && hid_sends == 2);
	report_frame(&hmd, 0, 2000000000);
	CHECK(hmd.touch_dev[0].have_calibration && hash_reads == 1 && cache_reads == 1 && warning_logs == 0);
	clear_retry_fixture(&hmd);
}
static void repeated_timeouts_warn_after_the_second_failure_in_plain_words(void)
{
	ohmd_context ctx = {0}; rift_hmd_t hmd; retry_fixture(&hmd, &ctx);
	cached_json = radio_json = valid_json;
	hash_timeouts = 10;
	report_frame(&hmd, 1, 1000000000);
	report_frame(&hmd, 1, 2000000000);
	CHECK(!hmd.touch_dev[1].have_calibration && warning_logs == 1);
	CHECK(strstr(diagnostics, "Left controller radio calibration: calibration hash: the controller did not answer"));
	CHECK(strstr(diagnostics, "retrying in 2 s") && strstr(diagnostics, "cv1touch report"));
	CHECK(!strstr(diagnostics, strerror(ETIMEDOUT)));
	for (uint64_t time = 2002000000; time <= 32000000000; time += 2000000)
		report_frame(&hmd, 1, time);
	CHECK(warning_logs == 1 && hash_timeouts == 4);
	hash_timeouts = 0;
	report_frame(&hmd, 1, 64000000000);
	CHECK(hmd.touch_dev[1].have_calibration && warning_logs == 1);
	/* Success resets the count: a later isolated timeout is quiet again. */
	hmd.touch_dev[1].have_calibration = false;
	hash_timeouts = 1;
	report_frame(&hmd, 1, 65000000000);
	CHECK(warning_logs == 1 && !hmd.touch_dev[1].have_calibration);
	clear_retry_fixture(&hmd);
}
int main(int argc, char **argv)
{
	CHECK(argc <= 2);
	if (argc == 1 || strcmp(argv[1], "drain") == 0) a_busy_drain_is_shared_and_spaced_between_reports();
	if (argc == 1 || strcmp(argv[1], "drain_slow") == 0) slow_or_failed_drain_polls_wait_from_completion();
	if (argc == 1 || strcmp(argv[1], "drain_notice") == 0) a_long_drain_warns_once_and_a_later_drain_can_warn_again();
	if (argc == 1 || strcmp(argv[1], "wake") == 0) one_wake_up_timeout_recovers_without_a_warning();
	if (argc == 1 || strcmp(argv[1], "wake_reason") == 0) repeated_timeouts_warn_after_the_second_failure_in_plain_words();
	if (argc == 1 || strcmp(argv[1], "parser") == 0)
		parser_failures_release_the_copy();
	if (argc == 1 || strcmp(argv[1], "cache") == 0) {
		rejected_cache_and_invalid_radio_release_buffers();
		cached_success_releases_both_buffers();
	}
	if (argc == 1 || strcmp(argv[1], "radio") == 0)
		radio_storage_is_borrowed_even_on_failure();
	if (argc == 1 || strcmp(argv[1], "leds") == 0)
		late_failures_leave_the_previous_calibration_unchanged();
	if (argc == 1 || strcmp(argv[1], "fallback") == 0)
		corrupt_cache_falls_back_to_the_radio();
	if (argc == 1 || strcmp(argv[1], "retry") == 0) {
		flash_allocation_failure_keeps_the_previous_buffer();
		terminal_failures_back_off_and_recover();
		transport_failures_report_each_change_once();
		the_other_hand_can_load_while_a_failed_hand_waits();
		pending_and_busy_commands_continue_on_the_next_report();
	}
	if (argc == 1 || strcmp(argv[1], "slow") == 0) slow_failures_wait_from_completion();
	if (argc == 1 || strcmp(argv[1], "naps") == 0) naps_do_not_reset_failure_history();
	if (argc == 1 || strcmp(argv[1], "identity") == 0) {
		another_hash_can_use_its_valid_cache();
		a_truncated_hash_reply_does_not_confirm_an_identity_change();
		a_new_hash_is_observed_at_the_capped_retry();
	}
	if (argc == 1 || strcmp(argv[1], "cache_alloc") == 0) a_cache_allocation_failure_is_not_quarantined();
	if (argc == 1 || strcmp(argv[1], "pending") == 0) pending_reads_finish_or_expire_without_reports(0);
	if (argc == 1 || strcmp(argv[1], "deadline") == 0) {
		pending_reads_finish_or_expire_without_reports(1);
		a_command_that_finished_during_a_stall_is_not_abandoned();
		a_failed_drain_is_reported_as_itself();
	}
	if (argc == 1 || strcmp(argv[1], "status") == 0) {
		short_status_does_not_fetch_stale_data_or_confirm_haptics();
		a_failed_haptics_status_leaves_no_reply_for_the_next_command();
	}
	if (argc == 1 || strcmp(argv[1], "errno") == 0) stale_numeric_errno_does_not_poison_valid_calibration();
	if (argc == 1 || strcmp(argv[1], "reader_oom") == 0) cache_reader_oom_uses_the_backoff();
	if (argc == 1 || strcmp(argv[1], "hid_error") == 0) hid_failures_report_io_instead_of_permissions();
	if (argc == 1 || strcmp(argv[1], "nodes") == 0) node_allocation_failures_unwind_and_retry_the_cache();
	if (argc == 1 || strcmp(argv[1], "roots") == 0) a_missing_root_is_not_a_successful_parse();
	if (argc == 1 || strcmp(argv[1], "schema") == 0) required_decoder_fields_are_validated();
	if (argc == 1 || strcmp(argv[1], "root_cache") == 0) an_empty_cached_root_preserves_the_model_and_falls_back();
	if (argc == 1 || strcmp(argv[1], "root_radio") == 0) an_empty_radio_root_uses_the_backoff();
	if (argc == 1 || strcmp(argv[1], "download") == 0) a_long_download_keeps_making_validated_progress();
	if (argc == 1 || strcmp(argv[1], "resync") == 0) recovery_drains_before_sending_and_leaves_no_extra_reply();
	if (argc == 1 || strcmp(argv[1], "optional") == 0) optional_factory_fields_keep_the_upstream_defaults();
	if (argc == 1 || strcmp(argv[1], "diagnostics") == 0) {
		failure_diagnostics_name_the_hand_source_field_and_next_step();
		a_failed_cache_save_keeps_the_calibration();
	}
	CHECK(argc == 1 || strcmp(argv[1], "leds") == 0 || strcmp(argv[1], "parser") == 0 ||
	      strcmp(argv[1], "cache") == 0 || strcmp(argv[1], "radio") == 0 ||
	      strcmp(argv[1], "fallback") == 0 || strcmp(argv[1], "retry") == 0 ||
	      strcmp(argv[1], "slow") == 0 || strcmp(argv[1], "naps") == 0 ||
	      strcmp(argv[1], "identity") == 0 || strcmp(argv[1], "cache_alloc") == 0 ||
	      strcmp(argv[1], "pending") == 0 || strcmp(argv[1], "deadline") == 0 ||
	      strcmp(argv[1], "status") == 0 ||
	      strcmp(argv[1], "errno") == 0 ||
	      strcmp(argv[1], "reader_oom") == 0 ||
	      strcmp(argv[1], "hid_error") == 0 ||
	      strcmp(argv[1], "nodes") == 0 ||
	      strcmp(argv[1], "roots") == 0 ||
	      strcmp(argv[1], "schema") == 0 ||
	      strcmp(argv[1], "root_cache") == 0 ||
	      strcmp(argv[1], "root_radio") == 0 ||
	      strcmp(argv[1], "download") == 0 ||
	      strcmp(argv[1], "resync") == 0 ||
	      strcmp(argv[1], "optional") == 0 ||
	      strcmp(argv[1], "diagnostics") == 0 ||
	      strcmp(argv[1], "drain") == 0 || strcmp(argv[1], "drain_slow") == 0 || strcmp(argv[1], "drain_notice") == 0 ||
	      strcmp(argv[1], "wake") == 0 || strcmp(argv[1], "wake_reason") == 0);
	puts("test_touch_calibration: ok");
	return 0;
}
