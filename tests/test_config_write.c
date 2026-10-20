// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
/* The real cache writer and its controller caller with synthetic filesystem/HID failures. */
#define _DEFAULT_SOURCE
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include "test_util.h"
#include "openhmdi.h" /* Keep device member names outside the syscall shims. */

/* The first group is the directory phase; the others fail while saving. A directory another process
 * creates first is fine (RACE_PREFIX, RACE_FINAL) unless what it made is not a directory (RACE_FILE). */
enum cache_case {
	CACHE_OK, CACHE_DIR_ACCESS, CACHE_DIR_FILE, CACHE_MKDIR_PREFIX, CACHE_MKDIR_FINAL,
	CACHE_RACE_PREFIX, CACHE_RACE_FINAL, CACHE_RACE_FILE,
	CACHE_OPEN, CACHE_WRITE, CACHE_SHORT, CACHE_CLOSE, CACHE_WRITE_AND_CLOSE, CACHE_FDOPEN, CACHE_FLUSH, CACHE_RENAME,
	CACHE_NAME_TAKEN, CACHE_NAMES_ALL_TAKEN, CACHE_REAL, CACHE_REAL_SHORT
};
static enum cache_case scenario;
static int opens, closes, logs, warnings, errors, descriptor_closes, renames, unlinks, temporary_exists;
static int syncs, flushes, prefix_stats, final_stats;
static char log_text[2048], temporary_name[OHMD_STR_SIZE + 64], taken_names[2][OHMD_STR_SIZE + 64];
static FILE synthetic_stream;
static FILE *stream = &synthetic_stream;

static int filename_error;
static int cache_snprintf(char *buf, size_t size, const char *format, ...)
{
	/* Legacy Windows _snprintf reports truncation with a negative result. */
	if (filename_error && strcmp(format, "%s/%s") == 0) {
		buf[0] = 0;
		return -1;
	}
	va_list args;
	va_start(args, format);
	int ret = vsnprintf(buf, size, format, args);
	va_end(args);
	return ret;
}
static int cache_printf(const char *format, ...)
{
	va_list args, copy;
	va_start(args, format);
	va_copy(copy, args);
	if (strcmp(format, "[%s] ") == 0) {
		const char *level = va_arg(copy, const char *);
		if (strcmp(level, "EE") == 0) { logs++; errors++; }
		if (strcmp(level, "WW") == 0) { logs++; warnings++; }
	}
	va_end(copy);
	va_list again;
	va_copy(again, args);
	size_t used = strlen(log_text);
	if (strcmp(format, "[%s] ") == 0) snprintf(log_text + used, sizeof log_text - used, "[%s] ", va_arg(again, const char *));
	else vsnprintf(log_text + used, sizeof log_text - used, format, again);
	va_end(again);
	int ret = vprintf(format, args);
	va_end(args);
	return ret;
}
static int cache_stat(const char *path, struct stat *info)
{
	if (scenario >= CACHE_REAL) return stat(path, info);
	memset(info, 0, sizeof *info);
	info->st_mode = S_IFDIR | 0700;
	if (scenario == CACHE_DIR_ACCESS) { errno = EACCES; return -1; }
	if (scenario == CACHE_DIR_FILE) { info->st_mode = S_IFREG | 0600; return 0; }
	if (scenario == CACHE_MKDIR_PREFIX ||
	    (scenario == CACHE_MKDIR_FINAL && strcmp(path, "synthetic/cache") == 0)) {
		errno = ENOENT;
		return -1;
	}
	/* The racing creator's directory (or file) appears after our mkdir found it exists. */
	if (scenario == CACHE_RACE_PREFIX && strcmp(path, "synthetic") == 0 && prefix_stats++ == 0) {
		errno = ENOENT;
		return -1;
	}
	if ((scenario == CACHE_RACE_FINAL || scenario == CACHE_RACE_FILE) && strcmp(path, "synthetic/cache") == 0) {
		if (final_stats++ == 0) { errno = ENOENT; return -1; }
		if (scenario == CACHE_RACE_FILE) info->st_mode = S_IFREG | 0600;
	}
	if (scenario == CACHE_RACE_PREFIX && strcmp(path, "synthetic/cache") == 0 && final_stats++ == 0) {
		errno = ENOENT;
		return -1;
	}
	return 0;
}
static int cache_mkdir(const char *path, mode_t mode)
{
	CHECK(mode == 0777);
	CHECK(scenario >= CACHE_MKDIR_PREFIX && scenario <= CACHE_RACE_FILE);
	if (scenario == CACHE_RACE_PREFIX && strcmp(path, "synthetic/cache") == 0)
		return 0; /* only the prefix was lost to the other process */
	CHECK(strcmp(path, scenario == CACHE_MKDIR_PREFIX || scenario == CACHE_RACE_PREFIX ? "synthetic" : "synthetic/cache") == 0);
	errno = scenario == CACHE_MKDIR_PREFIX ? EROFS : scenario == CACHE_MKDIR_FINAL ? ENOSPC : EEXIST;
	return -1;
}
static FILE *cache_open(const char *path, const char *mode)
{
	if (scenario >= CACHE_REAL) return fopen(path, mode);
	CHECK(strncmp(path, "synthetic/cache/", strlen("synthetic/cache/")) == 0);
	CHECK(strcmp(mode, "w") == 0 || strcmp(mode, "wb") == 0);
	opens++;
	if (scenario == CACHE_OPEN) { errno = EACCES; return NULL; }
	return stream;
}
static size_t cache_write(const void *buf, size_t size, size_t count, FILE *f)
{
	if (scenario >= CACHE_REAL) {
		errno = 0;
		return fwrite(buf, size, scenario == CACHE_REAL_SHORT ? 1 : count, f);
	}
	CHECK(f == stream && size == 1 && buf != NULL);
	if (scenario == CACHE_WRITE || scenario == CACHE_WRITE_AND_CLOSE) {
		errno = ENOSPC;
		return 0;
	}
	if (scenario == CACHE_SHORT) { errno = 0; return 1; }
	return count;
}
static inline int cache_error(FILE *f)
{
	if (scenario >= CACHE_REAL) return ferror(f);
	CHECK(f == stream);
	return scenario == CACHE_WRITE || scenario == CACHE_WRITE_AND_CLOSE;
}
static int cache_close(FILE *f)
{
	if (scenario >= CACHE_REAL) return fclose(f);
	CHECK(f == stream);
	closes++;
	if (scenario == CACHE_CLOSE || scenario == CACHE_WRITE_AND_CLOSE) { errno = EIO; return -1; }
	return 0;
}

/* The temporary file: a unique sibling, created exclusively with the mode fopen("w") would use. */
static int cache_open_fd(const char *path, int flags, ...)
{
	va_list args; va_start(args, flags);
	mode_t mode = (flags & O_CREAT) ? (mode_t)va_arg(args, int) : 0;
	va_end(args);
	if (scenario >= CACHE_REAL) return open(path, flags, mode);
	CHECK(strncmp(path, "synthetic/cache/", strlen("synthetic/cache/")) == 0 && strstr(path, ".tmp-") != NULL);
	CHECK((flags & (O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC)) == (O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC));
	CHECK(mode == 0666 && !temporary_exists);
	opens++;
	if (scenario == CACHE_OPEN) { errno = EACCES; return -1; }
	if (scenario == CACHE_NAMES_ALL_TAKEN) { errno = EEXIST; return -1; }
	if (scenario == CACHE_NAME_TAKEN && opens <= 2) {
		strcpy(taken_names[opens - 1], path);
		errno = EEXIST;
		return -1;
	}
	strcpy(temporary_name, path);
	temporary_exists = 1;
	return 42;
}
static FILE *cache_fdopen(int fd, const char *mode)
{
	if (scenario >= CACHE_REAL) return fdopen(fd, mode);
	CHECK(fd == 42 && strcmp(mode, "wb") == 0);
	if (scenario == CACHE_FDOPEN) { errno = ENOMEM; return NULL; }
	return stream;
}
static int cache_flush(FILE *f)
{
	if (scenario >= CACHE_REAL) return fflush(f);
	CHECK(f == stream); flushes++;
	if (scenario == CACHE_FLUSH) { errno = ENOSPC; return -1; }
	return 0;
}
/* Nothing is synced: the engine thread saves here, and must not wait for the disk. */
__attribute__((unused)) static int cache_sync(int fd)
{
	syncs++;
	return scenario >= CACHE_REAL ? fsync(fd) : 0;
}
static int cache_close_fd(int fd)
{
	if (scenario >= CACHE_REAL) return close(fd);
	CHECK(fd == 42); descriptor_closes++;
	return 0;
}
static int cache_rename(const char *from, const char *to)
{
	if (scenario >= CACHE_REAL) return rename(from, to);
	CHECK(strcmp(from, temporary_name) == 0 && temporary_exists);
	CHECK(strncmp(to, "synthetic/cache/", strlen("synthetic/cache/")) == 0);
	CHECK(strncmp(from, to, strlen(to)) == 0 && strncmp(from + strlen(to), ".tmp-", 5) == 0); /* a sibling */
	renames++;
	if (scenario == CACHE_RENAME) { errno = EACCES; return -1; }
	temporary_exists = 0;
	return 0;
}
static int cache_unlink(const char *path)
{
	if (scenario >= CACHE_REAL) return unlink(path);
	CHECK(strcmp(path, temporary_name) == 0 && temporary_exists);
	unlinks++; temporary_exists = 0;
	return 0;
}

#define stat(path, info) cache_stat(path, info)
#define mkdir(path, mode) cache_mkdir(path, mode)
#define fopen cache_open
#define fwrite cache_write
#define ferror cache_error
#define fclose cache_close
#define fdopen cache_fdopen
#define fflush cache_flush
#define fsync cache_sync
#define open cache_open_fd
#define close cache_close_fd
#define rename cache_rename
#define unlink cache_unlink
#define printf cache_printf
#undef _POSIX_C_SOURCE
#include "platform-posix.c"
#undef stat
#undef mkdir
#undef fopen
#undef fwrite
#undef ferror
#undef fclose
#undef fdopen
#undef fflush
#undef fsync
#undef open
#undef close
#undef rename
#undef unlink
/* The synthetic cases bypass reads; fresh-load tests read only their private directory. */
#define snprintf cache_snprintf
#define ohmd_get_config unused_cache_read
#include "openhmd.c"
#undef ohmd_get_config
#undef snprintf
#include "rift-hmd-radio.c"
#include "packet.c"
#include "ext_deps/nxjson.c"
#define get_feature_report hmd_get_feature_report
#define send_feature_report hmd_send_feature_report
#include "rift.c"
#undef get_feature_report
#undef send_feature_report
#include "omath.c"
#undef printf

int ohmd_get_config(ohmd_context *ctx, const char *key, char **buf, unsigned long *length)
{
	if (scenario >= CACHE_REAL) return unused_cache_read(ctx, key, buf, length);
	(void)ctx; (void)key; (void)buf; (void)length;
	return ENOENT;
}
int hid_send_feature_report(hid_device *handle, const unsigned char *data, size_t length)
{
	if (scenario >= CACHE_REAL) {
		CHECK(handle && data && length >= 6);
		if (data[0] == RIFT_CMD_RADIO_READ_DATA)
			CHECK((data[3] | data[4] << 8) == 0x1bf0 && data[5] == 16);
		else CHECK(data[0] == RIFT_CMD_RADIO_CONTROL);
		return (int)length;
	}
	(void)handle; (void)data; (void)length;
	CHECK(false); return -1;
}
int hid_get_feature_report(hid_device *handle, unsigned char *data, size_t length)
{
	if (scenario >= CACHE_REAL) {
		CHECK(handle && length == FEATURE_BUFFER_SIZE);
		CHECK(data[0] == RIFT_CMD_RADIO_CONTROL || data[0] == RIFT_CMD_RADIO_READ_DATA);
		memset(data + 1, 0, length - 1); /* fictional all-zero factory hash */
		return (int)length;
	}
	(void)handle; (void)data; (void)length;
	CHECK(false); return -1;
}
static int tracker_creations, imu_updates;
rift_tracked_device *rift_tracker_add_device(rift_tracker_ctx *ctx, int id,
    posef *imu, posef *model, rift_leds *leds, rift_tracked_device_imu_calibration *calibration)
{
	CHECK(ctx && id == 1 && imu && model && leds && calibration);
	tracker_creations++;
	return (rift_tracked_device *)(uintptr_t)1;
}
void rift_tracked_device_imu_update(rift_tracked_device *device, uint64_t host_ns,
    uint32_t device_ts, float dt, const vec3f *gyro, const vec3f *accel, const vec3f *mag)
{
	CHECK(device && host_ns > 0 && device_ts > 0 && dt >= 0 && gyro && accel && mag);
	imu_updates++;
}
/* Unused driver callbacks can be retained in optimized unity builds. */
void rift_tracker_retry_video(rift_tracker_ctx *ctx, uint64_t now_ns)
{
	(void)ctx; (void)now_ns;
	CHECK(false);
}
void rift_tracker_on_new_exposure(rift_tracker_ctx *ctx, uint32_t hmd_ts,
    uint16_t exposure_count, uint32_t exposure_hmd_ts, uint8_t led_pattern_phase)
{
	(void)ctx; (void)hmd_ts; (void)exposure_count; (void)exposure_hmd_ts; (void)led_pattern_phase;
	CHECK(false);
}
static void reset(enum cache_case failure, ohmd_context *ctx)
{
	scenario = failure;
	opens = closes = logs = warnings = errors = filename_error = 0;
	descriptor_closes = renames = unlinks = temporary_exists = 0;
	syncs = flushes = prefix_stats = final_stats = 0;
	log_text[0] = temporary_name[0] = taken_names[0][0] = taken_names[1][0] = 0;
	errno = 0;
	memset(ctx, 0, sizeof *ctx);
	strcpy(ctx->config_dir, "synthetic/cache");
}
static void cache_api_reports_write_errors(void)
{
	const struct { enum cache_case failure; int error; } cases[] = {
		{CACHE_OPEN, EACCES}, {CACHE_DIR_ACCESS, EACCES}, {CACHE_DIR_FILE, ENOTDIR},
		{CACHE_MKDIR_PREFIX, EROFS}, {CACHE_MKDIR_FINAL, ENOSPC}, {CACHE_RACE_FILE, ENOTDIR},
		{CACHE_WRITE, ENOSPC}, {CACHE_SHORT, EIO}, {CACHE_CLOSE, EIO},
		{CACHE_WRITE_AND_CLOSE, ENOSPC}, {CACHE_FDOPEN, ENOMEM}, {CACHE_FLUSH, ENOSPC},
		{CACHE_RENAME, EACCES}, {CACHE_NAMES_ALL_TAKEN, EEXIST},
	};
	char data[] = "abc";
	for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
		ohmd_context ctx;
		reset(cases[i].failure, &ctx);
		CHECK(ohmd_set_config(&ctx, "fixture.json", data, 3) == -cases[i].error);
		/* One warning, not an error: it names the place and says the calibration is still in use. */
		CHECK(logs == 1 && warnings == 1 && errors == 0 && ctx.error_msg[0] != 0);
		const int directory = scenario >= CACHE_DIR_ACCESS && scenario <= CACHE_RACE_FILE;
		CHECK(strstr(ctx.error_msg, directory ? "create the folder synthetic/cache" : "save the calibration to synthetic/cache/fixture.json") != NULL);
		CHECK(strstr(ctx.error_msg, strerror(cases[i].error)) != NULL);
		CHECK(strstr(log_text, "[WW] ") != NULL && strstr(log_text, ctx.error_msg) != NULL);
		CHECK(strstr(log_text, "still in use and will be read from the controller again next time") != NULL);
		CHECK(opens == (directory ? 0 : scenario == CACHE_NAMES_ALL_TAKEN ? 100 : 1));
		CHECK(closes == (directory || scenario == CACHE_OPEN || scenario == CACHE_FDOPEN || scenario == CACHE_NAMES_ALL_TAKEN ? 0 : 1));
		CHECK(temporary_exists == 0 && syncs == 0);
		CHECK(renames == (scenario == CACHE_RENAME ? 1 : 0));
		CHECK(unlinks == (!directory && scenario != CACHE_OPEN && scenario != CACHE_NAMES_ALL_TAKEN ? 1 : 0));
		CHECK(descriptor_closes == (scenario == CACHE_FDOPEN ? 1 : 0));
	}
	for (unsigned long length = 0; length <= 3; length += 3) {
		ohmd_context ctx;
		reset(CACHE_OK, &ctx);
		CHECK(ohmd_set_config(&ctx, "fixture.json", data, length) == 0);
		CHECK(opens == 1 && closes == 1 && logs == 0 && ctx.error_msg[0] == 0);
		/* No sync of the file or the directory: the engine thread must not wait for the disk. */
		CHECK(flushes == 1 && syncs == 0 && renames == 1);
		CHECK(unlinks == 0 && temporary_exists == 0 && descriptor_closes == 0);
	}
	/* A directory another process creates first is not a failure, if it made a directory. */
	for (int race = CACHE_RACE_PREFIX; race <= CACHE_RACE_FINAL; race++) {
		ohmd_context ctx; reset((enum cache_case)race, &ctx);
		CHECK(ohmd_set_config(&ctx, "fixture.json", data, 3) == 0 && logs == 0);
		CHECK(renames == 1 && temporary_exists == 0);
	}
	/* A temporary name that is taken (another process, or a leftover) is replaced by another. */
	ohmd_context ctx;
	reset(CACHE_NAME_TAKEN, &ctx);
	CHECK(ohmd_set_config(&ctx, "fixture.json", data, 3) == 0 && logs == 0);
	CHECK(opens == 3 && renames == 1 && unlinks == 0 && temporary_exists == 0);
	CHECK(strcmp(taken_names[0], taken_names[1]) != 0 && strcmp(taken_names[1], temporary_name) != 0);
	CHECK(strcmp(taken_names[0], temporary_name) != 0);
	reset(CACHE_OK, &ctx);
	char key[OHMD_STR_SIZE + 1];
	memset(key, 'x', sizeof key - 1); key[sizeof key - 1] = 0;
	CHECK(ohmd_set_config(&ctx, key, data, 3) == -ENAMETOOLONG);
	CHECK(logs == 1 && warnings == 1 && opens == 0 && strstr(ctx.error_msg, "save the calibration to xxx") != NULL);
	reset(CACHE_OK, &ctx);
	filename_error = 1;
	CHECK(ohmd_set_config(&ctx, "fixture.json", data, 3) == -ENAMETOOLONG);
	CHECK(logs == 1 && opens == 0);
	filename_error = 0;
	CHECK(ohmd_ensure_path("x") == -EINVAL);
	CHECK(ohmd_ensure_path(key) == 0); /* exactly the supported path length; already a dir */
	char too_long[OHMD_STR_SIZE + 2];
	memset(too_long, 'x', sizeof too_long - 1); too_long[sizeof too_long - 1] = 0;
	CHECK(ohmd_ensure_path(too_long) == -EINVAL);
}
static char factory_json[] = "{\"TrackedObject\":{\"JsonVersion\":2,\"ImuPosition\":[0,0,0],"
		"\"JoyXRangeMin\":0,\"JoyXRangeMax\":100,\"JoyXDeadMin\":40,\"JoyXDeadMax\":60,"
		"\"JoyYRangeMin\":0,\"JoyYRangeMax\":100,\"JoyYDeadMin\":40,\"JoyYDeadMax\":60,"
		"\"TriggerMinRange\":0,\"TriggerMidRange\":50,\"TriggerMaxRange\":100,"
		"\"MiddleMinRange\":0,\"MiddleMidRange\":50,\"MiddleMaxRange\":100,"
		"\"GyroCalibration\":[1,0,0,0,1,0,0,0,1,0,0,0],"
		"\"AccCalibration\":[1,0,0,0,1,0,0,0,1,0,0,0],\"CapSenseMin\":[0,0,0,0,0,0,0,0],"
		"\"CapSenseTouch\":[1,1,1,1,1,1,1,1],\"ModelPoints\":{\"Point0\":[0,0,0,0,1,0]}}}";

static void a_failed_cache_write_does_not_restart_calibration(void)
{
	for (int load = 0; load < 2; load++) {
		ohmd_context ctx;
		reset(CACHE_OPEN, &ctx);
		ohmd_monotonic_init(&ctx);
		tracker_creations = imu_updates = 0;
		rift_hmd_t hmd = {
			.ctx = &ctx,
			.tracker_ctx = (rift_tracker_ctx *)(uintptr_t)1,
			.radio = {
				.cur_read_cmd = RIFT_FW_READ_CMD_CALIBRATION,
				.device_in_progress = RIFT_TOUCH_CONTROLLER_RIGHT,
				.read_block = (uint8_t *)factory_json,
				.read_data_size = strlen(factory_json),
				.read_data_offset = strlen(factory_json) + 4,
			},
		};
		rift_touch_controller_t *touch = &hmd.touch_dev[0];
		touch->base.id = 1;
		touch->device_num = RIFT_TOUCH_CONTROLLER_RIGHT;
		pkt_rift_radio_message message = {0};
		for (int frame = 1; frame <= 16; frame++) {
			message.touch.timestamp = (uint32_t)frame * 1000;
			handle_touch_controller_message(&hmd, 1000000000 + (uint64_t)frame * 1000000, touch, &message);
			CHECK(touch->have_calibration && tracker_creations == 1 && imu_updates == frame);
			CHECK(opens == 1 && closes == 0 && logs == 1 && warnings == 1 && errors == 0);
		}
		CHECK(strstr(ctx.error_msg, "save the calibration to synthetic/cache/rift-touch-config-") != NULL);
		CHECK(strstr(log_text, "will be read from the controller again next time") != NULL);
		CHECK(hmd.radio.cur_read_cmd == RIFT_RADIO_CMD_NONE && hmd.radio.device_in_progress == -1);
		rift_touch_clear_calibration(&touch->calibration);
	}
}

/* A config path, file name included, must fit OHMD_STR_SIZE. Fixtures are named relative to TMPDIR,
 * so a long one (scripts/check.sh runs the suites under one) still leaves room for the name. */
static char *enter_tmpdir(void)
{
	char *previous = getcwd(NULL, 0); CHECK(previous);
	const char *tmp = getenv("TMPDIR");
	CHECK(chdir(tmp ? tmp : "/tmp") == 0);
	return previous;
}

static void leave_tmpdir(char *previous)
{
	CHECK(chdir(previous) == 0);
	free(previous);
}

static void a_short_write_cannot_poison_a_fresh_calibration_load(void)
{
	char *cwd = enter_tmpdir();
	for (int had_cache = 0; had_cache <= 1; had_cache++) {
		char directory[OHMD_STR_SIZE] = "cv1t-cache-atomic-XXXXXX", filename[OHMD_STR_SIZE];
		CHECK(mkdtemp(directory));
		const char key[] = "rift-touch-config-00000000000000000000000000000000.bin";
		CHECK(snprintf(filename, sizeof filename, "%s/%s", directory, key) < (int)sizeof filename);
		if (had_cache) {
			FILE *seed = fopen(filename, "wb"); CHECK(seed);
			CHECK(fwrite(factory_json, 1, strlen(factory_json), seed) == strlen(factory_json));
			CHECK(fclose(seed) == 0);
		}
		ohmd_context ctx; reset(CACHE_REAL_SHORT, &ctx);
		strcpy(ctx.config_dir, directory);
		char replacement[sizeof factory_json]; strcpy(replacement, factory_json);
		char *position = strstr(replacement, "\"ImuPosition\":[0"); CHECK(position);
		position[strlen("\"ImuPosition\":[")] = '1';
		CHECK(ohmd_set_config(&ctx, key, replacement, strlen(replacement)) == -EIO);
		char *loaded = NULL; unsigned long length = 0;
		int ret = ohmd_get_config(&ctx, key, &loaded, &length);
		if (had_cache) {
			CHECK(ret == 0 && length == strlen(factory_json) && memcmp(loaded, factory_json, length) == 0);
			free(loaded);
			rift_touch_calibration calibration = {0}; rift_touch_calibration_state state = {0};
			rift_hmd_radio_state radio = {.handle = (hid_device *)(uintptr_t)1};
			CHECK(rift_touch_get_calibration(&ctx, &radio, RIFT_TOUCH_CONTROLLER_RIGHT, &calibration, &state) == 0);
			CHECK(calibration.imu_position.x == 0 && !state.cache_rejected);
			rift_touch_clear_calibration(&calibration);
		} else CHECK(ret == ENOENT && loaded == NULL);
		DIR *dir = opendir(directory); CHECK(dir);
		struct dirent *entry; int files = 0;
		while ((entry = readdir(dir)) != NULL) {
			if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;
			CHECK(strcmp(entry->d_name, key) == 0); files++;
		}
		CHECK(closedir(dir) == 0 && files == had_cache); /* no partial temp files */
		/* A later complete save replaces the cache and is usable on a fresh load. */
		scenario = CACHE_REAL;
		CHECK(ohmd_set_config(&ctx, key, replacement, strlen(replacement)) == 0);
		rift_touch_calibration calibration = {0}; rift_touch_calibration_state state = {0};
		rift_hmd_radio_state radio = {.handle = (hid_device *)(uintptr_t)1};
		CHECK(rift_touch_get_calibration(&ctx, &radio, RIFT_TOUCH_CONTROLLER_RIGHT, &calibration, &state) == 0);
		CHECK(calibration.imu_position.x == 1 && !state.cache_rejected);
		rift_touch_clear_calibration(&calibration);
		CHECK(syncs == 0); /* neither the file nor the directory is synced */
		CHECK(unlink(filename) == 0 && rmdir(directory) == 0);
	}
	leave_tmpdir(cwd);
}
/* The saved file has the mode fopen("w") gave it before (0666 less the umask), not mkstemp's 0600. */
static void a_saved_cache_has_the_mode_fopen_gave_it(void)
{
	const mode_t masks[] = {022, 077, 002, 0};
	char *cwd = enter_tmpdir();
	for (size_t i = 0; i < sizeof masks / sizeof masks[0]; i++) {
		char directory[OHMD_STR_SIZE] = "cv1t-cache-mode-XXXXXX", filename[OHMD_STR_SIZE], reference[OHMD_STR_SIZE];
		CHECK(mkdtemp(directory));
		CHECK(snprintf(filename, sizeof filename, "%s/fixture.json", directory) < (int)sizeof filename);
		CHECK(snprintf(reference, sizeof reference, "%s/reference", directory) < (int)sizeof reference);
		ohmd_context ctx; reset(CACHE_REAL, &ctx);
		strcpy(ctx.config_dir, directory);
		const mode_t previous = umask(masks[i]);
		char data[] = "abc";
		CHECK(ohmd_set_config(&ctx, "fixture.json", data, 3) == 0);
		FILE *plain = fopen(reference, "w"); CHECK(plain && fclose(plain) == 0);
		umask(previous);
		struct stat saved, expected;
		CHECK(stat(filename, &saved) == 0 && stat(reference, &expected) == 0);
		CHECK((saved.st_mode & 0777) == (0666 & ~masks[i]));
		CHECK((saved.st_mode & 0777) == (expected.st_mode & 0777));
		CHECK(unlink(filename) == 0 && unlink(reference) == 0 && rmdir(directory) == 0);
	}
	leave_tmpdir(cwd);
}
int main(int argc, char **argv)
{
	CHECK(argc <= 2);
	if (argc == 1 || strcmp(argv[1], "api") == 0)
		cache_api_reports_write_errors();
	if (argc == 1 || strcmp(argv[1], "radio") == 0)
		a_failed_cache_write_does_not_restart_calibration();
	if (argc == 1 || strcmp(argv[1], "fresh") == 0) a_short_write_cannot_poison_a_fresh_calibration_load();
	if (argc == 1 || strcmp(argv[1], "mode") == 0) a_saved_cache_has_the_mode_fopen_gave_it();
	CHECK(argc == 1 || strcmp(argv[1], "api") == 0 || strcmp(argv[1], "radio") == 0 || strcmp(argv[1], "fresh") == 0 ||
	      strcmp(argv[1], "mode") == 0);
	puts("test_config_write: ok");
	return 0;
}
