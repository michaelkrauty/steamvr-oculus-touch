/*
 * Rift position tracking
 * Copyright 2014-2015 Philipp Zabel
 * Copyright 2019 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */

#define _GNU_SOURCE

#include "../log-output.h"
#include <libusb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../ohmd-assert.h"
#include <inttypes.h>
#include <stdarg.h>
#include <time.h>
#ifndef _WIN32
#include <fcntl.h>
#include <sys/stat.h>
#include <limits.h>
#endif

#include "ohmd-video.h"

#include "../exponential-filter.h"
#include "rift-cv1touch-clock.h"
#include "rift-tracker.h"
#include "rift-sensor.h"
#include "rift-sensor-pose-search.h"
#include "rift-sensor-usb.h"

#include "rift-sensor-maths.h"
#include "rift-sensor-camera.h"
#include "rift-sensor-pose-helper.h"

#include "rift-debug-draw.h"

#include "ohmd-pipewire.h"

#define ASSERT_MSG(_v, label, ...) if(!(_v)){ ohmd_log_fprintf(stderr, __VA_ARGS__); goto label; }


/* Number of IMU observations we accumulate before output */
#define RIFT_MAX_PENDING_IMU_OBSERVATIONS 1000

/* Number of state slots to use for quat/position updates */
#define NUM_POSE_DELAY_SLOTS 3

/* Number of exposure history slots to keep */
#define NUM_EXPOSURE_HISTORY 3

/* Length of time (milliseconds) we will interpolate position before declaring
 * tracking lost */
#define POSE_LOST_THRESHOLD 500

/* Continuity policy, matching the Rust tracker default, not a UKF stability bound. */
#define MAX_IMU_GAP_NS 100000000ull

/* Length of time (milliseconds) we can ignore orientation from cameras before
 * we force an update */
#define POSE_LOST_ORIENT_THRESHOLD 100

/* Length of time (milliseconds) that we are allowed to lose tracking because
 * there's no free delay slots before we'll warn about it. 60ms is ~3 frames
 */
#define NO_FREE_DELAY_SLOT_THRESHOLD 60

#define MIN_ROT_ERROR DEG_TO_RAD(25)
#define MIN_POS_ERROR 0.1

typedef struct rift_tracked_device_priv rift_tracked_device_priv;
typedef struct rift_tracker_pose_delay_slot rift_tracker_pose_delay_slot;

typedef struct rift_tracked_device_imu_observation rift_tracked_device_imu_observation;

struct rift_tracked_device_imu_observation {
	uint64_t local_ts;
	uint64_t device_ts;
	float dt;

	vec3f ang_vel;
	vec3f accel;
	vec3f mag;
};

struct rift_tracker_pose_delay_slot {
	int slot_id;		/* Index of the slot */
	bool valid;			/* true if the exposure info was set */
	int use_count;	/* Number of frames using this slot */

	uint64_t device_time_ns; /* Device time this slot is currently tracking */

	/* Whether any camera supplied a pose used by this delayed state. */
	bool had_used_report;
};

/* Internal full tracked device struct */
struct rift_tracked_device_priv {
	rift_tracked_device base;

	int index; /* Index of this entry in the devices array for the tracker and exposures */

	ohmd_mutex *device_lock;

	/* 6DOF Kalman Filter */
	rift_kalman_6dof_filter ukf_fusion;

	/* Account keeping for UKF fusion slots */
	int n_delay_slots;
	int delay_slot_index;
	rift_tracker_pose_delay_slot delay_slots[NUM_POSE_DELAY_SLOTS];
	/* Track the time we last started having no free delay slots (or 0 if never) */
	uint64_t last_no_free_delay_slot;

	/* The pose of the device relative to the IMU 3D space */
	posef device_from_fusion;

	/* The pose of the IMU relative to the LED model space */
	posef fusion_from_model;
	posef model_from_fusion;

	uint32_t last_device_ts;
	uint64_t device_time_ns;

	uint64_t last_observed_orient_ts;
	uint64_t last_observed_pose_ts;
	posef last_observed_pose;

	uint64_t last_acquired_pose_lock_ts;

	/* Reported view pose (to the user) and model pose (for the tracking) respectively */
	uint64_t last_reported_pose;
	posef reported_pose;
	vec3f reported_ang_vel;
	vec3f reported_lin_vel;
	vec3f reported_lin_accel;

	posef model_pose;

	exp_filter_pose pose_output_filter;

	int num_pending_imu_observations;
	rift_tracked_device_imu_observation pending_imu_observations[RIFT_MAX_PENDING_IMU_OBSERVATIONS];

	ohmd_pw_debug_stream *debug_metadata;
	FILE *debug_file;
};

struct sensor_stream_record {
    ohmd_cv1touch_sensor_stream stream;
    int pin_fd; /* one O_PATH|O_DIRECTORY|O_CLOEXEC metadata pin, closed on forget/replacement/close */
};

/* cv1touch: a sensor the engine thread asked the hot-plug worker to open and start */
struct sensor_add_request {
	char serial[32];
	ohmd_cv1touch_sensor_attachment key;
	bool have_pose;
	ohmd_cv1touch_sensor_pose pose;
};

/* cv1touch: a finished add, waiting for the engine thread to poll it */
struct sensor_add_result {
	ohmd_cv1touch_sensor_added added;
	rift_sensor_ctx *sensor;  /* STARTED: started, frames held back; in done or adopting until the roster */
	int pin_fd;               /* STARTED, START_FAILED: the start record's attachment pin */
	uint64_t start_done_ns;   /* when its start returned (host clock) */
};

/* cv1touch: the live sensor lifecycle's worker (cv1t-sensors). Every blocking USB step of an
 * add or a release runs there, never on the engine thread. It frees released sensors before it
 * opens the next one: on Windows a sensor's old handle must close before its new arrival opens.
 * `lock` is a leaf: nothing else is locked while it is held. The worker starts with the first
 * add or release, from the engine thread, whose scheduling, affinity and signal mask it keeps. */
struct sensor_hotplug {
	ohmd_thread *thread;
	ohmd_mutex *lock;
	ohmd_cond *cond;
	bool quit;
	struct sensor_add_request *requests;
	size_t n_requests, request_capacity;
	bool busy;                         /* the worker is adding `current` */
	struct sensor_add_request current;
	struct sensor_add_result *done;    /* capacity covers every request queued or running */
	size_t n_done, done_capacity;
	/* STARTED, taken out of done, not yet on the roster or in frees. The Windows search
	 * treats its device as held; cleared once the roster has it, or as it is queued to free. */
	rift_sensor_ctx *adopting;
	rift_sensor_ctx **frees;
	size_t n_frees, free_capacity;
};

struct rift_tracker_ctx_s
{
	ohmd_context* ohmd_ctx;
	libusb_context *usb_ctx;
	ohmd_mutex *tracker_lock;

	ohmd_thread* usb_thread;
	int usb_completed;

	int exposure_history_index;
	int exposure_history_size;
	rift_tracker_exposure_info exposure_history[NUM_EXPOSURE_HISTORY];

	/* cv1touch: only the engine thread changes the roster, and once the tracker is open it does
	 * so holding roster_lock; its own reads take no lock. Other threads (retirement, the hot-plug
	 * worker) read it holding roster_lock. Lock order: roster_lock -> sensor_lock ->
	 * tracker_lock -> device_lock; no sensor_lock holder takes roster_lock. */
	ohmd_mutex *roster_lock;
	rift_sensor_ctx **sensors;
	size_t n_sensors, sensor_capacity;
	struct sensor_stream_record *sensor_streams;
	size_t n_sensor_streams, sensor_stream_capacity;

	/* cv1touch: what a sensor added later needs: the headset's radio address, and a sensor id
	 * (logs, thread names) not used before */
	uint8_t radio_id[5];
	uint32_t next_sensor_id;
	struct sensor_hotplug hotplug;

	rift_tracked_device_priv devices[RIFT_MAX_TRACKED_DEVICES];
	uint8_t n_devices;

	uint32_t cv1touch_search_skip; /* atomic: rift_tracker_set_search_skip */
};

static void rift_tracked_device_send_imu_debug(rift_tracked_device_priv *dev);
static void rift_tracked_device_send_debug_printf(rift_tracked_device_priv *dev, uint64_t local_ts, const char *fmt, ...);

static void rift_tracked_device_on_new_exposure (rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info);
static int rift_tracked_device_exposure_claim(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info);
static void rift_tracked_device_exposure_release_locked(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info);

rift_tracked_device *
rift_tracker_add_device (rift_tracker_ctx *ctx, int device_id, posef *imu_pose, posef *model_pose, rift_leds *leds,
      rift_tracked_device_imu_calibration *imu_calib)
{
	int i, s;
	rift_tracked_device_priv *next_dev;
	char device_name[64];
	/* Rotate our initial pose 180 deg to point along the -Z axis */
	posef init_pose = { .pos = {{ 0.0, 0.0, 0.0 }}, .orient = {{ 0.0, 1.0, 0.0, 0.0 }}};

	snprintf(device_name,64,"openhmd-rift-device-%d", device_id);
	device_name[63] = 0;

	OHMD_ASSERT(ctx->n_devices < RIFT_MAX_TRACKED_DEVICES);

	ohmd_lock_mutex (ctx->tracker_lock);
	next_dev = ctx->devices + ctx->n_devices;

	next_dev->base.id = device_id;
	/* cv1touch: always, so a sensor plugged in after this device came online can feed its fusion */
	next_dev->n_delay_slots = NUM_POSE_DELAY_SLOTS;
	rift_kalman_6dof_init(&next_dev->ukf_fusion, &init_pose, next_dev->n_delay_slots);
	next_dev->last_acquired_pose_lock_ts = next_dev->last_reported_pose = next_dev->last_observed_orient_ts = next_dev->last_observed_pose_ts = next_dev->device_time_ns = 0;

	exp_filter_pose_init(&next_dev->pose_output_filter);

	/* Init delay slot bookkeeping */
	for (s = 0; s < next_dev->n_delay_slots; s++) {
		rift_tracker_pose_delay_slot *slot = next_dev->delay_slots + s;

		slot->slot_id = s;
		slot->valid = false;
	}

	/* Compute the device->IMU conversion from the imu->device pose passed */
	next_dev->device_from_fusion = *imu_pose;
	oposef_inverse(&next_dev->device_from_fusion);

	/* Compute the IMU->model transform by composing imu->device->model */
	oposef_apply(imu_pose, model_pose, &next_dev->fusion_from_model);
	/* And the inverse fusion->model conversion */
	next_dev->model_from_fusion = next_dev->fusion_from_model;
	oposef_inverse(&next_dev->model_from_fusion);

	next_dev->debug_metadata = ohmd_pw_debug_stream_new (device_name, "Rift Device");

	uint64_t now = ohmd_monotonic_get(ctx->ohmd_ctx);
	rift_tracked_device_send_debug_printf (next_dev, now, "{ \"type\": \"device\", "
		 "\"device-id\": %d,"
		"\"imu-calibration\": { \"accel-offset\": [ %f, %f, %f ], "
		"\"accel-matrix\": [ %f, %f, %f, %f, %f, %f, %f, %f, %f ], "
		"\"gyro_offset\": [ %f, %f, %f ], "
		"\"gyro-matrix\": [ %f, %f, %f, %f, %f, %f, %f, %f, %f ] } },", device_id,
		imu_calib->accel_offset.x, imu_calib->accel_offset.y, imu_calib->accel_offset.z,
		imu_calib->accel_matrix[0], imu_calib->accel_matrix[1], imu_calib->accel_matrix[2],
		imu_calib->accel_matrix[3], imu_calib->accel_matrix[4], imu_calib->accel_matrix[5],
		imu_calib->accel_matrix[6], imu_calib->accel_matrix[7], imu_calib->accel_matrix[8],
		imu_calib->gyro_offset.x, imu_calib->gyro_offset.y, imu_calib->gyro_offset.z,
		imu_calib->gyro_matrix[0], imu_calib->gyro_matrix[1], imu_calib->gyro_matrix[2],
		imu_calib->gyro_matrix[3], imu_calib->gyro_matrix[4], imu_calib->gyro_matrix[5],
		imu_calib->gyro_matrix[6], imu_calib->gyro_matrix[7], imu_calib->gyro_matrix[8]);

	next_dev->base.leds = leds;
	next_dev->base.led_search = led_search_model_new (leds);
	ctx->n_devices++;
	ohmd_unlock_mutex (ctx->tracker_lock);

	/* Tell the sensors about the new device */
	for (i = 0; i < ctx->n_sensors; i++) {
		rift_sensor_ctx *sensor_ctx = ctx->sensors[i];
		if (!rift_sensor_add_device (sensor_ctx, (rift_tracked_device *) next_dev)) {
			LOGE("Failed to configure object tracking for device %d\n", device_id);
		}
	}

	ohmd_log_printf("device %d online. Now tracking.\n", device_id);
	return (rift_tracked_device *) next_dev;
}

static unsigned int uvc_handle_events(void *arg)
{
	rift_tracker_ctx *tracker_ctx = arg;

	/* cv1touch: the daemon finds this thread by its name to make it real-time: it resubmits every
	 * sensor's video transfers, and a stall of it loses video on all of them at once */
	ohmd_set_thread_name("cv1t-usb");

	while (!tracker_ctx->usb_completed) {
		struct timeval timeout;

		timeout.tv_sec = 0;
		timeout.tv_usec = 100000;

		libusb_handle_events_timeout_completed(tracker_ctx->usb_ctx, &timeout, &tracker_ctx->usb_completed);
	}

	return 0;
}

static bool record_sensor_stream(rift_tracker_ctx *ctx,
    const ohmd_cv1touch_sensor_stream *stream, int pin_fd);

#ifndef _WIN32
#ifndef CV1TOUCH_SYSFS_USB_ROOT
#define CV1TOUCH_SYSFS_USB_ROOT "/sys/bus/usb/devices"
#endif

static bool sysfs_usb_number(int dir, const char *field, unsigned int expected)
{
    int fd = openat(dir, field, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return false;
    char text[32] = {0}, extra;
    ssize_t n = read(fd, text, sizeof text - 1);
    close(fd);
    unsigned int value;
    return n > 0 && sscanf(text, "%u %c", &value, &extra) == 1 && value == expected;
}

static int pin_sensor_attachment(libusb_device *device,
    ohmd_cv1touch_sensor_attachment *attachment)
{
    *attachment = (ohmd_cv1touch_sensor_attachment){0};
    unsigned int bus = libusb_get_bus_number(device), address = libusb_get_device_address(device);
    uint8_t ports[7];
    int count = libusb_get_port_numbers(device, ports, sizeof ports);
    if (!bus || !address || count < 1 || count > 7) return -1;
    char path[PATH_MAX];
    int pos = snprintf(path, sizeof path, "%s/%u-", CV1TOUCH_SYSFS_USB_ROOT, bus);
    if (pos < 0 || (size_t)pos >= sizeof path) return -1;
    for (int p = 0; p < count; p++) {
        int n = snprintf(path + pos, sizeof path - (size_t)pos, "%s%u", p ? "." : "", ports[p]);
        if (n < 0 || (size_t)n >= sizeof path - (size_t)pos) return -1;
        pos += n;
    }
    /* Pin BEFORE libusb_open, never after a possibly failed old handle. The path
     * comes from its bus/port chain; attributes are read relative to that same
     * directory so replacement cannot mix an old inode with new USB metadata.
     * kernfs holds the node until inode eviction, preventing ID reuse while our
     * O_PATH reference lives: linux fs/kernfs/inode.c, kernfs_init_inode and
     * kernfs_evict_inode:
     * https://github.com/torvalds/linux/blob/v6.17/fs/kernfs/inode.c#L188-L256
     * Retain this one pin with the start record, including after sensor free. */
    int fd = open(path, O_PATH | O_DIRECTORY | O_CLOEXEC);
    if (fd < 0) return -1;
    struct stat info;
    if (fstat(fd, &info) != 0 || !info.st_ino ||
        !sysfs_usb_number(fd, "busnum", bus) || !sysfs_usb_number(fd, "devnum", address)) {
        close(fd);
        return -1;
    }
    uint64_t inode = (uint64_t)info.st_ino;
    *attachment = (ohmd_cv1touch_sensor_attachment){.kind = OHMD_CV1TOUCH_ATTACHMENT_SYSFS,
        .data = {bus | (address << 8), (uint32_t)inode, (uint32_t)(inode >> 32)}};
    return fd;
}
#endif

/* cv1touch: a Rift Sensor (CV1 or DK2 camera), and its descriptor */
static bool is_rift_sensor(libusb_device *dev, struct libusb_device_descriptor *desc)
{
	return libusb_get_device_descriptor(dev, desc) >= 0 && desc->idVendor == 0x2833 &&
		(desc->idProduct == CV1_PID || desc->idProduct == DK2_PID);
}

/* Room for one more roster entry. Pointer-array growth never moves sensor objects. Once the
 * tracker is open, the caller holds roster_lock: other threads read the array under it. */
static bool reserve_sensor(rift_tracker_ctx *ctx)
{
	if (ctx->n_sensors >= INT_MAX) return false;
	if (ctx->n_sensors < ctx->sensor_capacity) return true;
	size_t capacity = ctx->sensor_capacity ? ctx->sensor_capacity * 2 : 4;
	if (capacity > INT_MAX) capacity = INT_MAX;
	if (capacity > SIZE_MAX / sizeof(*ctx->sensors)) return false;
	void *next = realloc(ctx->sensors, capacity * sizeof(*ctx->sensors));
	if (!next) return false;
	ctx->sensors = next; ctx->sensor_capacity = capacity;
	return true;
}

/* cv1touch: how opening a sensor ended (try_open_sensor) */
enum sensor_open {
	SENSOR_OPENED,
	SENSOR_OPEN_FAILED, /* libusb_open, or setting up the USB device or the analyser */
	SENSOR_REJECTED,    /* unreadable serial, not the one asked for, already open, or ignored */
	SENSOR_NO_MEMORY,   /* the roster could not grow */
};

/* cv1touch: opens the Rift Sensor dev and creates its analyser, both at engine open and for a
 * sensor added later. A duplicate is an open sensor of the same serial that is not retired.
 * expected: the serial asked for, or NULL. reserve: make room in the roster first (engine open;
 * a sensor added later joins the roster on the engine thread). Closes pin_fd unless it returns
 * SENSOR_OPENED; *error_code is libusb's code (or a negative errno) of a failed open or setup. */
static enum sensor_open try_open_sensor(rift_tracker_ctx *ctx, libusb_device *dev,
	const struct libusb_device_descriptor *desc, int pin_fd, int id, const char *expected,
	const ohmd_cv1touch_sensor_pose *pose, bool reserve, rift_sensor_ctx **out, int *error_code)
{
	ohmd_context *ohmd_ctx = ctx->ohmd_ctx;
	libusb_device_handle *usb_devh;
	unsigned char serial[33];
	int ret;

	*out = NULL;
	*error_code = 0;
	ret = libusb_open(dev, &usb_devh);
	if (ret) {
		ohmd_log_fprintf (stderr, "Failed to open Rift Sensor device. Check permissions\n");
		rift_sensor_close_attachment_pin(pin_fd);
		*error_code = ret;
		return SENSOR_OPEN_FAILED;
	}

	memset(serial, 0, sizeof serial);
	ret = desc->iSerialNumber ? libusb_get_string_descriptor_ascii(
		usb_devh, desc->iSerialNumber, serial, 32) : -1;
	if (ret <= 0 || ret >= 32 || !cv1touch_sensor_serial_valid((const char *)serial)) {
		LOGW("Ignoring Rift Sensor with unreadable or invalid serial");
		libusb_close(usb_devh); rift_sensor_close_attachment_pin(pin_fd);
		return SENSOR_REJECTED;
	}
	if (expected && !cv1touch_sensor_serial_equal(expected, (const char *)serial)) {
		LOGW("Ignoring Rift Sensor %s: sensor %s was asked for", serial, expected);
		libusb_close(usb_devh); rift_sensor_close_attachment_pin(pin_fd);
		return SENSOR_REJECTED;
	}
	bool duplicate = false;
	ohmd_lock_mutex(ctx->roster_lock);
	for (size_t k = 0; k < ctx->n_sensors; k++)
		if (cv1touch_sensor_serial_equal(rift_sensor_serial_no(ctx->sensors[k]), (const char *)serial) &&
		    !rift_sensor_retired(ctx->sensors[k]))
			duplicate = true;
	ohmd_unlock_mutex(ctx->roster_lock);
	if (duplicate) {
		LOGW("Ignoring duplicate Rift Sensor identity");
		libusb_close(usb_devh); rift_sensor_close_attachment_pin(pin_fd);
		return SENSOR_REJECTED;
	}
	if (cv1touch_sensor_ignored(&ohmd_ctx->cv1touch_sensors,
			&ohmd_ctx->cv1touch_config, (const char *)serial)) {
		LOGI("Ignoring Rift Sensor %s (configured ignore list)", serial);
		libusb_close(usb_devh); rift_sensor_close_attachment_pin(pin_fd);
		return SENSOR_REJECTED;
	}
	/* Reserve before creating workers. */
	if (reserve && !reserve_sensor(ctx)) {
		libusb_close(usb_devh); rift_sensor_close_attachment_pin(pin_fd);
		return SENSOR_NO_MEMORY;
	}
	rift_sensor_device *sensor_device = rift_sensor_usb_new (ohmd_ctx, id, (char *) serial,
		ctx->usb_ctx, usb_devh, ctx->radio_id, error_code);
	if (sensor_device == NULL) {
		LOGW("Failed to open Rift Sensor USB device %s\n", serial);
		rift_sensor_close_attachment_pin(pin_fd);
		return SENSOR_OPEN_FAILED;
	}

	*out = rift_sensor_new (ohmd_ctx, id, (char *) serial, sensor_device, ctx, pose);
	if (*out == NULL) {
		LOGW("Failed to open Rift Sensor analyser for %s\n", serial);
		rift_sensor_close_attachment_pin(pin_fd);
		*error_code = LIBUSB_ERROR_NO_MEM;
		return SENSOR_OPEN_FAILED;
	}
	return SENSOR_OPENED;
}

rift_tracker_ctx *
rift_tracker_new (ohmd_context* ohmd_ctx,
		const uint8_t radio_id[5])
{
	rift_tracker_ctx *tracker_ctx = NULL;
	int ret, i;
	libusb_device **devs = NULL;

	tracker_ctx = ohmd_alloc(ohmd_ctx, sizeof (rift_tracker_ctx));
	if (!tracker_ctx) return NULL;
	tracker_ctx->ohmd_ctx = ohmd_ctx;
	memcpy(tracker_ctx->radio_id, radio_id, sizeof tracker_ctx->radio_id);
	tracker_ctx->tracker_lock = ohmd_create_mutex(ohmd_ctx);
	if (!tracker_ctx->tracker_lock) goto fail;
	tracker_ctx->roster_lock = ohmd_create_mutex(ohmd_ctx);
	if (!tracker_ctx->roster_lock) goto fail;

	for (i = 0; i < RIFT_MAX_TRACKED_DEVICES; i++) {
		rift_tracked_device_priv *dev = tracker_ctx->devices + i;
		dev->index = i;
		dev->device_lock = ohmd_create_mutex(ohmd_ctx);
		if (!dev->device_lock) goto fail;
	}

	ret = libusb_init(&tracker_ctx->usb_ctx);
	ASSERT_MSG(ret >= 0, fail, "could not initialize libusb\n");

	ret = libusb_get_device_list(tracker_ctx->usb_ctx, &devs);
	ASSERT_MSG(ret >= 0, fail, "Could not get USB device list\n");

	/* cv1touch: provisional sensor placement starts over for every tracker */
	rift_pose_finder_cv1touch_reset();

	/* Start USB event thread */
	tracker_ctx->usb_completed = false;
	tracker_ctx->usb_thread = ohmd_create_thread (ohmd_ctx, uvc_handle_events, tracker_ctx);
	if (!tracker_ctx->usb_thread) goto fail;

	/* Until this returns, no other thread can reach the tracker or its roster. */
	for (i = 0; devs[i]; ++i) {
		struct libusb_device_descriptor desc;
		rift_sensor_ctx *sensor_ctx;
		int error_code;

		if (!is_rift_sensor(devs[i], &desc))
			continue; /* Can't access this device, or it is something else */

        ohmd_cv1touch_sensor_attachment attachment = {0};
        int attachment_pin = -1;
#ifndef _WIN32
        if (!ohmd_ctx->cv1touch_sensor_attachment_cb)
            attachment_pin = pin_sensor_attachment(devs[i], &attachment);
#endif
		enum sensor_open opened = try_open_sensor(tracker_ctx, devs[i], &desc, attachment_pin,
			(int)tracker_ctx->n_sensors, NULL, NULL, true, &sensor_ctx, &error_code);
		if (opened == SENSOR_NO_MEMORY)
			goto fail;
		if (opened != SENSOR_OPENED)
			continue;

        if (ohmd_ctx->cv1touch_sensor_attachment_cb)
            ohmd_ctx->cv1touch_sensor_attachment_cb(ohmd_ctx->cv1touch_sensor_attachment_user,
                rift_sensor_serial_no(sensor_ctx), &attachment);
        rift_sensor_set_attachment(sensor_ctx, &attachment, attachment_pin);
		tracker_ctx->sensors[tracker_ctx->n_sensors] = sensor_ctx;
		tracker_ctx->n_sensors++;

	}
	libusb_free_device_list(devs, 1);
	devs = NULL;
	tracker_ctx->next_sensor_id = (uint32_t)tracker_ctx->n_sensors;

	/* Loop over the sensors we found and start the video flowing */
    for (i = 0; i < (int)tracker_ctx->n_sensors;) {
        rift_sensor_ctx *sensor = tracker_ctx->sensors[i];
        bool started = rift_sensor_start(sensor);
        ohmd_cv1touch_sensor_stream stream;
        rift_sensor_get_stream_info(sensor, &stream);
        /* Copy before a failed start frees the object. Upsert success too, so
         * reconnects/fallback never preserve an earlier failure for this serial. */
        if (!record_sensor_stream(tracker_ctx, &stream, rift_sensor_take_attachment_pin(sensor))) {
            LOGE("Could not retain sensor stream diagnostics");
            goto fail;
        }
        if (!started) {
            LOGW("Failed to start video stream for sensor %s\n", rift_sensor_serial_no(sensor));
            rift_sensor_free(sensor);
            tracker_ctx->n_sensors--;
            memmove(tracker_ctx->sensors+i, tracker_ctx->sensors+i+1,
                    (tracker_ctx->n_sensors-(size_t)i)*sizeof(*tracker_ctx->sensors));
        } else i++;
    }

	ohmd_log_printf ("Started %zu Rift Sensor cameras\n", tracker_ctx->n_sensors);
	return tracker_ctx;

fail:
	if (devs) libusb_free_device_list(devs, 1);
	if (tracker_ctx)
		rift_tracker_free (tracker_ctx);
	return NULL;
}

/* cv1touch: the live sensor lifecycle (openhmd_cv1touch.h, "Live sensor lifecycle"). The engine
 * thread queues adds and releases and adopts started sensors; the hot-plug worker opens, starts
 * and frees them. Neither holds roster, sensor or tracker lock across a USB start, stop or
 * retry, nor any lock while it waits for the other. */

static uint32_t elapsed_ms(rift_tracker_ctx *ctx, uint64_t since)
{
	uint64_t ms = ohmd_monotonic_conv(ohmd_monotonic_get(ctx->ohmd_ctx) - since,
		ohmd_monotonic_per_sec(ctx->ohmd_ctx), 1000);
	return ms > UINT32_MAX ? UINT32_MAX : (uint32_t)ms;
}

/* items (capacity entries of size bytes) with room for needed entries, or NULL when it cannot
 * grow, which leaves items as it was. */
static void *reserve_entries(void *items, size_t *capacity, size_t needed, size_t size)
{
	if (needed <= *capacity)
		return items;
	size_t next = *capacity ? *capacity : 4;
	while (next < needed) {
		if (next > SIZE_MAX / 2)
			return NULL;
		next *= 2;
	}
	if (next > SIZE_MAX / size)
		return NULL;
	void *grown = realloc(items, next * size);
	if (grown)
		*capacity = next;
	return grown;
}

#ifndef _WIN32
/* The device at the key's bus/address, pinned and still the incarnation the key names. */
static uint32_t find_pinned_sensor(libusb_device **list, const ohmd_cv1touch_sensor_attachment *key,
	libusb_device **out, struct libusb_device_descriptor *desc, int *pin_fd,
	ohmd_cv1touch_sensor_attachment *pinned)
{
	for (size_t i = 0; list[i]; i++) {
		uint32_t place = (uint32_t)libusb_get_bus_number(list[i]) |
			((uint32_t)libusb_get_device_address(list[i]) << 8);
		if (place != key->data[0] || !is_rift_sensor(list[i], desc))
			continue;
		*pin_fd = pin_sensor_attachment(list[i], pinned);
		if (*pin_fd < 0)
			return OHMD_CV1TOUCH_SENSOR_ADD_NOT_FOUND;
		if (memcmp(pinned, key, sizeof *key) != 0) {
			/* Another connection took that place: never open it for this one */
			rift_sensor_close_attachment_pin(*pin_fd);
			*pin_fd = -1;
			return OHMD_CV1TOUCH_SENSOR_ADD_STALE;
		}
		*out = list[i];
		return 0;
	}
	return OHMD_CV1TOUCH_SENSOR_ADD_NOT_FOUND;
}
#endif

/* WinUSB lets one handle at a time open a device, and libusb keeps the libusb_device of a device
 * a handle holds open: a device a sensor of this engine holds is never a candidate for another.
 * That includes one the engine thread has taken out of `done` and is adopting onto the roster. */
static bool sensor_device_held(rift_tracker_ctx *ctx, const libusb_device *dev)
{
	struct sensor_hotplug *h = &ctx->hotplug;
	bool held = false;

	ohmd_lock_mutex(ctx->roster_lock);
	for (size_t i = 0; i < ctx->n_sensors && !held; i++)
		held = rift_sensor_get_usb_device(ctx->sensors[i]) == dev;
	ohmd_unlock_mutex(ctx->roster_lock);
	ohmd_lock_mutex(h->lock);
	for (size_t i = 0; i < h->n_done && !held; i++)
		held = h->done[i].sensor && rift_sensor_get_usb_device(h->done[i].sensor) == dev;
	for (size_t i = 0; i < h->n_frees && !held; i++)
		held = rift_sensor_get_usb_device(h->frees[i]) == dev;
	if (!held && h->adopting)
		held = rift_sensor_get_usb_device(h->adopting) == dev;
	ohmd_unlock_mutex(h->lock);
	return held;
}

/* A Windows key cannot be verified natively: the sensor is the unheld Rift Sensor of that
 * serial. Unverified on Windows hardware. */
static uint32_t find_unheld_sensor(rift_tracker_ctx *ctx, libusb_device **list, const char *serial,
	libusb_device **out, struct libusb_device_descriptor *desc, int *error_code)
{
	int open_error = 0;
	for (size_t i = 0; list[i]; i++) {
		libusb_device_handle *devh;
		unsigned char found[33] = {0};
		if (!is_rift_sensor(list[i], desc) || sensor_device_held(ctx, list[i]))
			continue;
		int ret = libusb_open(list[i], &devh);
		if (ret) {
			open_error = ret;
			continue;
		}
		ret = desc->iSerialNumber ? libusb_get_string_descriptor_ascii(
			devh, desc->iSerialNumber, found, 32) : -1;
		libusb_close(devh);
		if (ret > 0 && ret < 32 && cv1touch_sensor_serial_equal((const char *)found, serial)) {
			*out = list[i];
			return 0;
		}
	}
	/* one it could not open may well have been it */
	*error_code = open_error;
	return open_error ? OHMD_CV1TOUCH_SENSOR_ADD_OPEN_FAILED : OHMD_CV1TOUCH_SENSOR_ADD_NOT_FOUND;
}

/* Worker: opens and starts one sensor. Nothing else can reach it until the engine thread adopts
 * it, and its frames are held back until then. */
static void add_one_sensor(rift_tracker_ctx *ctx, const struct sensor_add_request *req,
	struct sensor_add_result *res)
{
	ohmd_cv1touch_sensor_added *added = &res->added;
	uint64_t began = ohmd_monotonic_get(ctx->ohmd_ctx);
	ohmd_cv1touch_sensor_attachment attachment = req->key;
	struct libusb_device_descriptor desc;
	libusb_device **list = NULL, *dev = NULL;
	rift_sensor_ctx *sensor = NULL;
	int pin_fd = -1, error_code = 0;

	*res = (struct sensor_add_result){.pin_fd = -1};
	memcpy(added->stream.serial, req->serial, sizeof added->stream.serial);
	added->stream.attachment = req->key;

	ssize_t listed = libusb_get_device_list(ctx->usb_ctx, &list);
	if (listed < 0) {
		added->outcome = OHMD_CV1TOUCH_SENSOR_ADD_OPEN_FAILED;
		added->error_code = (int32_t)listed;
		return;
	}
#ifndef _WIN32
	if (req->key.kind == OHMD_CV1TOUCH_ATTACHMENT_SYSFS)
		added->outcome = find_pinned_sensor(list, &req->key, &dev, &desc, &pin_fd, &attachment);
	else
#endif
		added->outcome = find_unheld_sensor(ctx, list, req->serial, &dev, &desc, &error_code);
	if (added->outcome == 0) {
		enum sensor_open opened = try_open_sensor(ctx, dev, &desc, pin_fd, (int)ctx->next_sensor_id++,
			req->serial, req->have_pose ? &req->pose : NULL, false, &sensor, &error_code);
		if (opened != SENSOR_OPENED)
			added->outcome = opened == SENSOR_REJECTED ? OHMD_CV1TOUCH_SENSOR_ADD_REJECTED :
				OHMD_CV1TOUCH_SENSOR_ADD_OPEN_FAILED;
	}
	libusb_free_device_list(list, 1);
	added->open_ms = elapsed_ms(ctx, began);
	added->error_code = error_code;
	if (sensor == NULL)
		return;

	rift_sensor_set_attachment(sensor, &attachment, pin_fd);
	rift_sensor_accept_frames_from(sensor, UINT64_MAX);
	if (__atomic_load_n(&ctx->hotplug.quit, __ATOMIC_ACQUIRE)) {
		/* the engine is closing: not worth a second of start */
		rift_sensor_free(sensor);
		added->outcome = OHMD_CV1TOUCH_SENSOR_ADD_OPEN_FAILED;
		added->error_code = LIBUSB_ERROR_INTERRUPTED;
		return;
	}
	uint64_t starting = ohmd_monotonic_get(ctx->ohmd_ctx);
	bool started = rift_sensor_start(sensor);
	res->start_done_ns = ohmd_monotonic_get(ctx->ohmd_ctx);
	added->start_ms = elapsed_ms(ctx, starting);
	rift_sensor_get_stream_info(sensor, &added->stream);
	res->pin_fd = rift_sensor_take_attachment_pin(sensor);
	if (!started) {
		LOGW("Failed to start video stream for sensor %s\n", rift_sensor_serial_no(sensor));
		rift_sensor_free(sensor);
		added->outcome = OHMD_CV1TOUCH_SENSOR_ADD_START_FAILED;
		added->error_code = added->stream.error_code;
		return;
	}
	added->outcome = OHMD_CV1TOUCH_SENSOR_ADD_STARTED;
	res->sensor = sensor;
}

static unsigned int sensor_hotplug_thread(void *arg)
{
	rift_tracker_ctx *ctx = arg;
	struct sensor_hotplug *h = &ctx->hotplug;

	/* cv1touch: findable in profiles and the daemon's thread list */
	ohmd_set_thread_name("cv1t-sensors");
	ohmd_lock_mutex(h->lock);
	while (!h->quit) {
		if (h->n_frees > 0) {
			/* frees first: a sensor's new arrival may need its old handle closed */
			rift_sensor_ctx *sensor = h->frees[0];
			h->n_frees--;
			memmove(h->frees, h->frees + 1, h->n_frees * sizeof *h->frees);
			ohmd_unlock_mutex(h->lock);
			rift_sensor_free(sensor);
			ohmd_lock_mutex(h->lock);
		} else if (h->n_requests > 0) {
			struct sensor_add_request req = h->requests[0];
			struct sensor_add_result res;
			h->n_requests--;
			memmove(h->requests, h->requests + 1, h->n_requests * sizeof *h->requests);
			h->current = req;
			h->busy = true;
			ohmd_unlock_mutex(h->lock);
			add_one_sensor(ctx, &req, &res);
			ohmd_lock_mutex(h->lock);
			h->done[h->n_done++] = res; /* room was made when it was queued */
			h->busy = false;
		} else {
			ohmd_cond_wait(h->cond, h->lock);
		}
	}
	ohmd_unlock_mutex(h->lock);
	return 0;
}

/* Engine thread. */
static bool start_hotplug_worker(rift_tracker_ctx *ctx)
{
	struct sensor_hotplug *h = &ctx->hotplug;

	if (h->thread)
		return true;
	if (!h->lock)
		h->lock = ohmd_create_mutex(ctx->ohmd_ctx);
	if (h->lock && !h->cond)
		h->cond = ohmd_create_cond(ctx->ohmd_ctx);
	if (h->cond)
		h->thread = ohmd_create_thread(ctx->ohmd_ctx, sensor_hotplug_thread, ctx);
	if (!h->thread)
		LOGE("Could not start the sensor hot-plug worker");
	return h->thread != NULL;
}

/* Engine thread: the worker frees the sensor, or this thread does when it cannot. */
static void free_on_worker(rift_tracker_ctx *ctx, rift_sensor_ctx *sensor)
{
	struct sensor_hotplug *h = &ctx->hotplug;
	rift_sensor_ctx **frees = NULL;

	if (start_hotplug_worker(ctx)) {
		ohmd_lock_mutex(h->lock);
		/* Same critical section as the frees insert: the worker cannot free it while the
		 * Windows search could still see it, and it is not in both places. */
		if (h->adopting == sensor)
			h->adopting = NULL;
		frees = reserve_entries(h->frees, &h->free_capacity, h->n_frees + 1, sizeof *h->frees);
		if (frees) {
			h->frees = frees;
			h->frees[h->n_frees++] = sensor;
			ohmd_cond_signal(h->cond);
		}
		ohmd_unlock_mutex(h->lock);
	} else if (h->adopting == sensor) {
		h->adopting = NULL;
	}
	if (!frees) {
		LOGW("Freeing Rift Sensor %s on the engine thread", rift_sensor_serial_no(sensor));
		rift_sensor_free(sensor);
	}
}

/* Engine thread: retires every roster sensor of this serial, takes it out of the roster under
 * roster_lock (so retirement elsewhere never reaches a freed sensor) and has it freed. */
static void release_serial(rift_tracker_ctx *ctx, const char *serial)
{
	for (size_t i = 0; i < ctx->n_sensors;) {
		rift_sensor_ctx *sensor = ctx->sensors[i];
		if (!cv1touch_sensor_serial_equal(rift_sensor_serial_no(sensor), serial)) {
			i++;
			continue;
		}
		rift_sensor_retire(sensor);
		ohmd_lock_mutex(ctx->roster_lock);
		ctx->n_sensors--;
		memmove(ctx->sensors + i, ctx->sensors + i + 1, (ctx->n_sensors - i) * sizeof *ctx->sensors);
		ohmd_unlock_mutex(ctx->roster_lock);
		free_on_worker(ctx, sensor);
	}
}

/* Engine thread: this serial's open sensor that is not retired, or NULL. */
static rift_sensor_ctx *sensor_open(rift_tracker_ctx *ctx, const char *serial)
{
	for (size_t i = 0; i < ctx->n_sensors; i++)
		if (cv1touch_sensor_serial_equal(rift_sensor_serial_no(ctx->sensors[i]), serial) &&
		    !rift_sensor_retired(ctx->sensors[i]))
			return ctx->sensors[i];
	return NULL;
}

int rift_tracker_get_sensor_lifetime(rift_tracker_ctx *ctx, const char *serial, uint64_t *out)
{
	rift_sensor_ctx *sensor = sensor_open(ctx, serial);
	if (!sensor)
		return -1;
	*out = rift_sensor_lifetime(sensor);
	return 0;
}

int rift_tracker_add_sensor(rift_tracker_ctx *ctx, const char *serial,
	const ohmd_cv1touch_sensor_attachment *key, const ohmd_cv1touch_sensor_pose *pose)
{
	struct sensor_hotplug *h = &ctx->hotplug;
	struct sensor_add_request req = {0};
	size_t length = strnlen(serial, sizeof req.serial);

	if (length == 0 || length >= sizeof req.serial)
		return -1;
	memcpy(req.serial, serial, length);
	if (!cv1touch_sensor_serial_valid(req.serial))
		return -1;
	switch (key->kind) {
#ifndef _WIN32
	case OHMD_CV1TOUCH_ATTACHMENT_SYSFS:
		if ((key->data[0] & 0xff) == 0 || (key->data[0] & 0xff00) == 0 || (key->data[1] | key->data[2]) == 0)
			return -1;
		break;
#endif
	case OHMD_CV1TOUCH_ATTACHMENT_WINDOWS:
		if ((key->data[0] | key->data[1]) == 0)
			return -1;
		break;
	default:
		return -1;
	}
	req.key = *key;
	if (pose) {
		if (!cv1touch_sensor_serial_equal(pose->serial, req.serial))
			return -1;
		req.have_pose = true;
		req.pose = *pose;
	}
	if (sensor_open(ctx, req.serial))
		return 1;
	if (!start_hotplug_worker(ctx))
		return -1;

	ohmd_lock_mutex(h->lock);
	bool queued = h->busy && cv1touch_sensor_serial_equal(h->current.serial, req.serial);
	for (size_t i = 0; i < h->n_requests; i++)
		queued |= cv1touch_sensor_serial_equal(h->requests[i].serial, req.serial);
	for (size_t i = 0; i < h->n_done; i++)
		queued |= cv1touch_sensor_serial_equal(h->done[i].added.stream.serial, req.serial);
	int ret = 1;
	if (!queued) {
		struct sensor_add_request *requests = reserve_entries(h->requests, &h->request_capacity,
			h->n_requests + 1, sizeof *h->requests);
		if (requests)
			h->requests = requests;
		/* the worker never waits for room for a result */
		struct sensor_add_result *done = requests ? reserve_entries(h->done, &h->done_capacity,
			h->n_done + h->n_requests + (h->busy ? 1 : 0) + 1, sizeof *h->done) : NULL;
		if (done)
			h->done = done;
		ret = -1;
		if (requests && done) {
			h->requests[h->n_requests++] = req;
			ohmd_cond_signal(h->cond);
			ret = 0;
		}
	}
	ohmd_unlock_mutex(h->lock);
	return ret;
}

/* Engine thread: a started sensor joins the roster, unless it duplicates an open one. */
static void adopt_sensor(rift_tracker_ctx *ctx, struct sensor_add_result *res)
{
	rift_sensor_ctx *sensor = res->sensor;
	const char *serial = rift_sensor_serial_no(sensor);
	bool joined = false;

	if (sensor_open(ctx, serial)) {
		LOGW("Ignoring duplicate Rift Sensor identity");
		res->added.outcome = OHMD_CV1TOUCH_SENSOR_ADD_REJECTED;
		res->added.error_code = 0;
	} else {
		/* An earlier connection's retired object goes first, if its release has not come yet */
		release_serial(ctx, serial);
		/* Every device the tracker has, before any other thread can see the sensor; this thread
		 * is the only one that adds devices */
		for (int i = 0; i < ctx->n_devices; i++) {
			if (!rift_sensor_add_device(sensor, &ctx->devices[i].base))
				LOGE("Failed to configure object tracking for device %d\n", ctx->devices[i].base.id);
		}
		ohmd_lock_mutex(ctx->roster_lock);
		joined = reserve_sensor(ctx);
		if (joined)
			ctx->sensors[ctx->n_sensors++] = sensor;
		ohmd_unlock_mutex(ctx->roster_lock);
		if (!joined) {
			res->added.outcome = OHMD_CV1TOUCH_SENSOR_ADD_OPEN_FAILED;
			res->added.error_code = LIBUSB_ERROR_NO_MEM;
		}
	}
	if (!joined) {
		rift_sensor_close_attachment_pin(res->pin_fd);
		free_on_worker(ctx, sensor);
		return;
	}
	if (!record_sensor_stream(ctx, &res->added.stream, res->pin_fd))
		LOGE("Could not retain sensor stream diagnostics");
	/* Its earlier frames were exposed before its radio followed the headset */
	rift_sensor_accept_frames_from(sensor, res->start_done_ns);
	res->added.lifetime = rift_sensor_lifetime(sensor);
}

int rift_tracker_poll_sensor_adds(rift_tracker_ctx *ctx, ohmd_cv1touch_sensor_added *out, int max)
{
	struct sensor_hotplug *h = &ctx->hotplug;
	int n = 0;

	if (!h->lock)
		return 0;
	while (n < max) {
		struct sensor_add_result res;
		ohmd_lock_mutex(h->lock);
		bool finished = h->n_done > 0;
		if (finished) {
			res = h->done[0];
			h->n_done--;
			memmove(h->done, h->done + 1, h->n_done * sizeof *h->done);
			/* Visible to sensor_device_held until adopt joins the roster or frees it.
			 * h->lock stays a leaf: the roster lock is taken only after this unlock. */
			if (res.sensor && res.added.outcome == OHMD_CV1TOUCH_SENSOR_ADD_STARTED)
				h->adopting = res.sensor;
		}
		ohmd_unlock_mutex(h->lock);
		if (!finished)
			break;
		if (res.added.outcome == OHMD_CV1TOUCH_SENSOR_ADD_STARTED)
			adopt_sensor(ctx, &res);
		else if (res.added.outcome == OHMD_CV1TOUCH_SENSOR_ADD_START_FAILED &&
		         !record_sensor_stream(ctx, &res.added.stream, res.pin_fd))
			LOGE("Could not retain sensor stream diagnostics");
		if (res.sensor) {
			ohmd_lock_mutex(h->lock);
			if (h->adopting == res.sensor)
				h->adopting = NULL;
			ohmd_unlock_mutex(h->lock);
		}
		out[n++] = res.added;
	}
	return n;
}

int rift_tracker_release_sensor(rift_tracker_ctx *ctx, const char *serial)
{
	release_serial(ctx, serial);
	return 0;
}

/* Engine thread, at close: the worker can be in a sensor's start or free. */
static void stop_hotplug_worker(rift_tracker_ctx *ctx)
{
	struct sensor_hotplug *h = &ctx->hotplug;

	if (!h->thread)
		return;
	ohmd_lock_mutex(h->lock);
	__atomic_store_n(&h->quit, true, __ATOMIC_RELEASE);
	ohmd_cond_broadcast(h->cond);
	ohmd_unlock_mutex(h->lock);
	ohmd_destroy_thread(h->thread);
	h->thread = NULL;
}

/* Called from the rift IMU / packet handling loop
 * when processing an IMU update from the HMD. If the
 * packet signalled a new camera exposure, we take
 * a snapshot of the predicted state of each device
 * into a lagged fusion slot */
void rift_tracker_on_new_exposure (rift_tracker_ctx *ctx, uint32_t hmd_ts, uint16_t exposure_count, uint32_t exposure_hmd_ts, uint8_t led_pattern_phase)
{
	rift_tracker_exposure_info *info;
	bool is_new_exposure = false;
	int i;

	ohmd_lock_mutex (ctx->tracker_lock);

	if (ctx->exposure_history_size > 0) {
		if (ctx->exposure_history[ctx->exposure_history_index].count != exposure_count) {
			is_new_exposure = true;

			ctx->exposure_history_index = (ctx->exposure_history_index + 1) % NUM_EXPOSURE_HISTORY;
			info = ctx->exposure_history + ctx->exposure_history_index;

			if (ctx->exposure_history_size < NUM_EXPOSURE_HISTORY)
				ctx->exposure_history_size++;
		}
	}
	else {
		info = ctx->exposure_history;
		ctx->exposure_history_index = 0;
		ctx->exposure_history_size = 1;
		is_new_exposure = true;
	}

	if (!is_new_exposure)
		goto done;

	if (info->led_pattern_phase != led_pattern_phase) {
		LOGD ("%f LED pattern phase changed to %d",
			(double) (ohmd_monotonic_get(ctx->ohmd_ctx)) / 1000000.0, led_pattern_phase);
		info->led_pattern_phase = led_pattern_phase;
	}

	uint64_t now = ohmd_monotonic_get(ctx->ohmd_ctx);

	info->local_ts = now;
	info->count = exposure_count;
	info->hmd_ts = exposure_hmd_ts;
	info->led_pattern_phase = led_pattern_phase;

	LOGD ("%f Have new exposure TS %u count %u LED pattern phase %d",
		(double) (now) / 1000000.0, exposure_count, exposure_hmd_ts, led_pattern_phase);

	if ((int32_t)(exposure_hmd_ts - hmd_ts) < -1500) {
		LOGV("Exposure timestamp %u was more than 1.5 IMU samples earlier than IMU ts %u by %u µS",
				exposure_hmd_ts, hmd_ts, hmd_ts - exposure_hmd_ts);
	}

	info->n_devices = ctx->n_devices;

	for (i = 0; i < ctx->n_devices; i++) {
		rift_tracked_device_priv *dev = ctx->devices + i;
		rift_tracked_device_exposure_info *dev_info = info->devices + i;

		dev_info->device_index = dev->index;

		ohmd_lock_mutex (dev->device_lock);
		rift_tracked_device_on_new_exposure(dev, dev_info);

		rift_tracked_device_send_imu_debug(dev);

		rift_tracked_device_send_debug_printf(dev, now,
				",\n{ \"type\": \"exposure\", \"local-ts\": %llu, "
				"\"hmd-ts\": %u, \"exposure-ts\": %u, \"count\": %u, \"device-ts\": %llu, "
				"\"delay-slot\": %d	}",
				(unsigned long long) now,
				hmd_ts, exposure_hmd_ts, exposure_count,
				(unsigned long long) dev_info->device_time_ns, dev_info->fusion_slot);
		ohmd_unlock_mutex (dev->device_lock);
	}
	/* Clear the info for non-existent devices */
	for (; i < RIFT_MAX_TRACKED_DEVICES; i++) {
		rift_tracked_device_exposure_info *dev_info = info->devices + i;
		dev_info->fusion_slot = -1;
	}

done:
	ohmd_unlock_mutex (ctx->tracker_lock);
}

/* Called from a sensor device when a video frame has been captured.
 * Iterate the exposure history and find the exposure that matches
 * the arrival time of the frame within +/- 10ms
 */
bool
rift_tracker_frame_captured (rift_tracker_ctx *ctx, uint64_t local_ts, uint64_t frame_start_local_ts, rift_tracker_exposure_info *out_info, const char *source)
{
	int i;
	ohmd_lock_mutex (ctx->tracker_lock);

	/* Find and populate the exposure info and return true if found */
	bool have_exposure_info = false;

	for (i = 0; i < ctx->exposure_history_size; i++) {
		rift_tracker_exposure_info *info = ctx->exposure_history + i;
		int64_t time_diff_ns = frame_start_local_ts - info->local_ts;
		if (time_diff_ns > -10000000 && time_diff_ns < 10000000) {
			have_exposure_info = true;
			*out_info = *info;
		}
	}

	if (!have_exposure_info)
		goto done;

	for (i = 0; i < ctx->n_devices; i++) {
		rift_tracked_device_priv *dev = ctx->devices + i;

		ohmd_lock_mutex (dev->device_lock);

		if (i < out_info->n_devices) {
			rift_tracked_device_exposure_info *dev_info = out_info->devices + i;

#if LOGLEVEL == 0
			LOGD("Frame capture - ts %llu, delay slot %d for dev %d",
				(unsigned long long) dev_info->device_time_ns, dev_info->fusion_slot, dev->base.id);
#endif
			dev_info->fusion_slot = rift_tracked_device_exposure_claim(dev, dev_info);
		}

		rift_tracked_device_send_imu_debug(dev);

		if (dev->debug_file != NULL) {
			fprintf(dev->debug_file, ",\n{ \"type\": \"frame-captured\", \"local-ts\": %llu, "
				"\"frame-start-local-ts\": %llu, \"source\": \"%s\" }",
				(unsigned long long) local_ts, (unsigned long long) frame_start_local_ts, source);
		}
		ohmd_unlock_mutex (dev->device_lock);
	}

done:
	ohmd_unlock_mutex (ctx->tracker_lock);

	return have_exposure_info;
}

void
rift_tracker_frame_release (rift_tracker_ctx *ctx, uint64_t local_ts, uint64_t frame_local_ts, rift_tracker_exposure_info *info, const char *source)
{
	int i;
	ohmd_lock_mutex (ctx->tracker_lock);
	for (i = 0; i < ctx->n_devices; i++) {
		rift_tracked_device_priv *dev = ctx->devices + i;

		ohmd_lock_mutex (dev->device_lock);

		/* This device might not have exposure info for this frame if it
		 * recently came online */
		if (info && i < info->n_devices) {
			rift_tracked_device_exposure_info *dev_info = info->devices + i;
			rift_tracked_device_exposure_release_locked(dev, dev_info);
		}

		rift_tracked_device_send_imu_debug(dev);

		if (dev->debug_file != NULL) {
			fprintf(dev->debug_file, ",\n{ \"type\": \"frame-release\", \"local-ts\": %llu, "
				"\"frame-local-ts\": %llu, \"source\": \"%s\" }",
				(unsigned long long) local_ts, (unsigned long long) frame_local_ts, source);
		}
		ohmd_unlock_mutex (dev->device_lock);
	}
	ohmd_unlock_mutex (ctx->tracker_lock);
}

void rift_tracker_retry_video(rift_tracker_ctx *ctx, uint64_t now_ns)
{
	if (ctx)
		for (size_t i = 0; i < ctx->n_sensors; i++)
			rift_sensor_retry(ctx->sensors[i], now_ns);
}

void
rift_tracker_free (rift_tracker_ctx *tracker_ctx)
{
	int i;

	if (!tracker_ctx)
		return;

	/* cv1touch: first the hot-plug worker, then the sensors it started or was to free */
	stop_hotplug_worker(tracker_ctx);
	struct sensor_hotplug *h = &tracker_ctx->hotplug;

	/* Signal every stream before the first blocking drain. Sensor workers keep
	 * their frame storage until each sensor's normal stop/join/free completes. */
	for (size_t s = 0; s < h->n_done; s++)
		if (h->done[s].sensor)
			rift_sensor_request_stop(h->done[s].sensor);
	for (size_t s = 0; s < h->n_frees; s++)
		rift_sensor_request_stop(h->frees[s]);
	if (h->adopting)
		rift_sensor_request_stop(h->adopting);
	for (i = 0; i < tracker_ctx->n_sensors; i++)
		rift_sensor_request_stop(tracker_ctx->sensors[i]);

	for (size_t s = 0; s < h->n_done; s++) {
		if (h->done[s].sensor)
			rift_sensor_free(h->done[s].sensor);
		rift_sensor_close_attachment_pin(h->done[s].pin_fd);
	}
	for (size_t s = 0; s < h->n_frees; s++)
		rift_sensor_free(h->frees[s]);
	if (h->adopting) {
		rift_sensor_free(h->adopting);
		h->adopting = NULL;
	}
	for (i = 0; i < tracker_ctx->n_sensors; i++) {
		rift_sensor_ctx *sensor_ctx = tracker_ctx->sensors[i];
		rift_sensor_free (sensor_ctx);
	}
	free(h->requests);
	free(h->done);
	free(h->frees);
	if (h->cond) ohmd_destroy_cond(h->cond);
	if (h->lock) ohmd_destroy_mutex(h->lock);

	for (i = 0; i < RIFT_MAX_TRACKED_DEVICES; i++) {
		rift_tracked_device_priv *dev = tracker_ctx->devices + i;
		if (dev->base.led_search)
			led_search_model_free (dev->base.led_search);
		if (dev->debug_metadata != NULL)
			ohmd_pw_debug_stream_free (dev->debug_metadata);

		rift_kalman_6dof_clear(&dev->ukf_fusion);
		if (dev->device_lock) ohmd_destroy_mutex (dev->device_lock);
	}

	/* Stop USB event thread */
	tracker_ctx->usb_completed = true;
	if (tracker_ctx->usb_thread) ohmd_destroy_thread (tracker_ctx->usb_thread);

	if (tracker_ctx->usb_ctx)
		libusb_exit (tracker_ctx->usb_ctx);

	if (tracker_ctx->tracker_lock) ohmd_destroy_mutex (tracker_ctx->tracker_lock);
	if (tracker_ctx->roster_lock) ohmd_destroy_mutex (tracker_ctx->roster_lock);
	free(tracker_ctx->sensors);
    for (size_t s = 0; s < tracker_ctx->n_sensor_streams; s++)
        rift_sensor_close_attachment_pin(tracker_ctx->sensor_streams[s].pin_fd);
	free(tracker_ctx->sensor_streams);
	free (tracker_ctx);
}

/* Called with only device_lock held. Camera copies carry slot index + timestamp,
 * not references to UKF storage. Invalidate that identity before reusing slots so
 * late get/update/claim/release callbacks cannot affect this new epoch. */
static void rift_tracked_device_reset_continuity(rift_tracked_device_priv *dev)
{
	posef held_pose;
	rift_kalman_6dof_get_pose_at(&dev->ukf_fusion, dev->device_time_ns, &held_pose,
		NULL, NULL, NULL, NULL, NULL);
	rift_kalman_6dof_reset(&dev->ukf_fusion, &held_pose);
	for (int i = 0; i < dev->n_delay_slots; i++) {
		dev->delay_slots[i].valid = false;
		dev->delay_slots[i].use_count = 0;
		dev->delay_slots[i].had_used_report = false;
	}
	dev->delay_slot_index = 0;
	dev->last_no_free_delay_slot = 0;
	dev->last_observed_pose_ts = dev->last_observed_orient_ts = 0;
	dev->last_acquired_pose_lock_ts = dev->last_reported_pose = 0;
	dev->pose_output_filter.pos_filter.have_prev_y = false;
	dev->pose_output_filter.orient_filter.have_prev_y = false;
	dev->reported_lin_vel = dev->reported_lin_accel = dev->reported_ang_vel = (vec3f){{0}};
}

void rift_tracked_device_imu_update(rift_tracked_device *dev_base, uint64_t local_ts, uint32_t device_ts, float dt, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);
	rift_tracked_device_imu_observation *obs;

	ohmd_lock_mutex (dev->device_lock);

	/* Handle device_ts wrap by extending to 64-bit and working in nanoseconds */
	uint64_t next_time_ns = rift_device_time_next(dev->device_time_ns, dev->last_device_ts, device_ts);
	/* Use sample stamps, not dt: Touch deliberately stamps the previous report. */
	if (dev->device_time_ns != 0 && next_time_ns - dev->device_time_ns > MAX_IMU_GAP_NS)
		rift_tracked_device_reset_continuity(dev);
	dev->device_time_ns = next_time_ns;
	dev->last_device_ts = device_ts;

	rift_kalman_6dof_imu_update (&dev->ukf_fusion, dev->device_time_ns, ang_vel, accel, mag_field);

	obs = dev->pending_imu_observations + dev->num_pending_imu_observations;
	obs->local_ts = local_ts;
	obs->device_ts = dev->device_time_ns;
	obs->dt = dt;
	obs->ang_vel = *ang_vel;
	obs->accel = *accel;
	obs->mag = *mag_field;

	dev->num_pending_imu_observations++;

	if (dev->num_pending_imu_observations == RIFT_MAX_PENDING_IMU_OBSERVATIONS) {
		/* No camera observations for a while - send our observations from here instead */
		rift_tracked_device_send_imu_debug(dev);
	}

	ohmd_unlock_mutex (dev->device_lock);
}

void rift_tracked_device_get_view_pose(rift_tracked_device *dev_base, posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);

	ohmd_lock_mutex (dev->device_lock);

	if (dev->device_time_ns > dev->last_reported_pose) {
		posef device_pose;
		posef imu_global_pose;
		vec3f imu_ang_vel = { 0, };
		vec3f imu_vel = { 0, }, imu_accel = { 0, };

		rift_kalman_6dof_get_pose_at(&dev->ukf_fusion, dev->device_time_ns, &imu_global_pose, &imu_vel, &imu_accel, &imu_ang_vel, NULL, NULL);

		/* Take our fusion / IMU global pose back to device pose by
		 * computing the IMU->device pose and applying the
		 * IMU->world pose to get device->world pose */
		oposef_apply(&dev->device_from_fusion, &imu_global_pose, &device_pose);

		dev->reported_pose.orient = device_pose.orient;
		if (dev->last_observed_pose_ts == 0 || dev->device_time_ns - dev->last_observed_pose_ts >= (POSE_LOST_THRESHOLD * 1000000UL)) {
			/* Don't let the device move unless there's a recent observation of actual position */
			device_pose.pos = dev->reported_pose.pos;
			imu_vel.x = imu_vel.y = imu_vel.z = 0.0;
			imu_accel.x = imu_accel.y = imu_accel.z = 0.0;
		}

		exp_filter_pose_run(&dev->pose_output_filter, dev->device_time_ns, &device_pose, &dev->reported_pose);
		dev->last_reported_pose = dev->device_time_ns;

		/* Angular Velocity and acceleration need rotating into the device space.
		 * Linear velocity should also acquire a component from angular velocity */
		oquatf_get_rotated(&dev->device_from_fusion.orient, &imu_ang_vel, &dev->reported_ang_vel);
		oquatf_get_rotated(&dev->device_from_fusion.orient, &imu_accel, &dev->reported_lin_accel);

		/* Linear velocity generated by the angular velocity at the IMU offset
		 * is the cross product of the (rotated) position and the angular
		 * velocity */
		vec3f rotated_imu_pos, extra_lin_vel;
		oquatf_get_rotated(&dev->device_from_fusion.orient, &dev->device_from_fusion.pos, &rotated_imu_pos);
		ovec3f_cross(&dev->reported_ang_vel, &rotated_imu_pos, &extra_lin_vel);

		oquatf_get_rotated(&dev->device_from_fusion.orient, &imu_vel, &dev->reported_lin_vel);
		ovec3f_add(&dev->reported_lin_vel, &dev->reported_lin_vel, &extra_lin_vel);
	}

	if (pose)
		*pose = dev->reported_pose;
	if (ang_vel)
		*ang_vel = dev->reported_ang_vel;
	if (accel)
		*accel = dev->reported_lin_accel;
	if (vel)
		*vel = dev->reported_lin_vel;

	ohmd_unlock_mutex (dev->device_lock);
}

static rift_tracker_pose_delay_slot *get_matching_delay_slot(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info);

/* Retrieve the latest model pose estimate from a delay slot into the exposure info.
 * Because we can receive pose updates and new IMU data between frame capture and
 * when we go to do a visual search, and those can improve the estimate of the
 * pose estimate we had when the exposure happened */
bool rift_tracked_device_get_latest_exposure_info_pose (rift_tracked_device *dev_base, rift_tracked_device_exposure_info *dev_info)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);
	rift_tracker_pose_delay_slot *slot = NULL;
	bool res = false;

	if (dev_info->fusion_slot == -1)
		return false;

	ohmd_lock_mutex (dev->device_lock);

	slot = get_matching_delay_slot(dev, dev_info);
	if (slot != NULL) {
		posef imu_global_pose;
		vec3f global_pos_error, global_rot_error;

		rift_kalman_6dof_get_delay_slot_pose_at(&dev->ukf_fusion, dev_info->device_time_ns, slot->slot_id, &imu_global_pose,
						NULL, NULL, NULL, &global_pos_error, &global_rot_error);

		oposef_apply(&dev->model_from_fusion, &imu_global_pose, &dev_info->capture_pose);

		int i;
		for (i = 0; i < 3; i++) {
			if (global_rot_error.arr[i] < MIN_ROT_ERROR)
				global_rot_error.arr[i] = MIN_ROT_ERROR;
			if (global_pos_error.arr[i] < MIN_POS_ERROR)
				global_pos_error.arr[i] = MIN_POS_ERROR;
		}

		oquatf_get_rotated_abs(&dev->model_from_fusion.orient, &global_pos_error, &dev_info->pos_error);
		oquatf_get_rotated_abs(&dev->model_from_fusion.orient, &global_rot_error, &dev_info->rot_error);
		res = true;
	}
	else {
		/* If we failed to get the pose, it means the delay slot was overridden,
		 * so clear it in the device info */
		dev_info->fusion_slot = -1;
	}

	ohmd_unlock_mutex (dev->device_lock);

	return res;
}

bool rift_tracked_device_model_pose_update(rift_tracked_device *dev_base, uint64_t local_ts, uint64_t frame_start_local_ts, rift_tracker_exposure_info *exposure_info,
    rift_pose_metrics *score, posef *model_pose, const char *source)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);
	uint64_t frame_device_time_ns = 0;
	rift_tracker_pose_delay_slot *slot = NULL;
	int frame_fusion_slot = -1;
	bool update_position = false;
	bool update_orientation = false;
	posef imu_pose;

	ohmd_lock_mutex (dev->device_lock);

	/* Apply the fusion->model pose on top of the passed model->global pose,
	 * to get the global IMU pose */
	oposef_apply(&dev->fusion_from_model, model_pose, &imu_pose);

	rift_tracked_device_send_imu_debug(dev);

	if (dev->index >= exposure_info->n_devices) {
		ohmd_unlock_mutex (dev->device_lock);
		return true; /* Returning true means the sensor will stop searching any further */
	}

	/* This device existed when the exposure was taken and therefore has info */
	rift_tracked_device_exposure_info *dev_info = exposure_info->devices + dev->index;
	frame_device_time_ns = dev_info->device_time_ns;

	slot = get_matching_delay_slot(dev, dev_info);
	if (slot != NULL) {
		quatf orient_diff;
		vec3f pos_error, rot_error;

		ovec3f_subtract(&model_pose->pos, &dev_info->capture_pose.pos, &pos_error);

		oquatf_diff(&model_pose->orient, &dev_info->capture_pose.orient, &orient_diff);
		oquatf_normalize_me(&orient_diff);
		oquatf_to_rotation(&orient_diff, &rot_error);

		LOGD ("Got pose update for delay slot %d for dev %d, ts %llu (delay %f) orient %f %f %f %f diff %f %f %f pos %f %f %f diff %f %f %f from %s\n",
			slot->slot_id, dev->base.id,
			(unsigned long long) frame_device_time_ns, (double) (dev->device_time_ns - frame_device_time_ns) / 1000000000.0,
			model_pose->orient.x, model_pose->orient.y, model_pose->orient.z, model_pose->orient.w,
			rot_error.x, rot_error.y, rot_error.z,
			model_pose->pos.x, model_pose->pos.y, model_pose->pos.z,
			pos_error.x, pos_error.y, pos_error.z,
			source);

		/* If this observation was based on a prior, but position didn't match and we already received a newer observation,
		 * ignore it. */
		if (dev_info->had_pose_lock && !POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_POSITION) && dev->last_observed_pose_ts > frame_device_time_ns) {
			update_position = false;
			LOGV("Ignoring position observation with error %f %f %f (prior stddev was %f %f %f)\n",
				pos_error.x, pos_error.y, pos_error.z,
				dev_info->pos_error.x, dev_info->pos_error.y, dev_info->pos_error.z);
		}
		else {
			update_position = true;
		}

		/* If we have a strong match, update both position and orientation */
		if (POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_ORIENT)) {
			update_orientation = true;
			if ((dev->device_time_ns - dev->last_observed_orient_ts) > (POSE_LOST_ORIENT_THRESHOLD * 1000000UL)) {
				LOGV("Matched orientation after %f sec", (dev->device_time_ns - dev->last_observed_pose_ts) / 1000000000.0);
			}
			/* Only update the time if we're actually going to apply this matched orientation below */
			if (update_position)
				dev->last_observed_orient_ts = dev->device_time_ns;
		}
		else if ((dev->device_time_ns - dev->last_observed_orient_ts) > (POSE_LOST_ORIENT_THRESHOLD * 1000000UL)) {
			LOGV("Forcing orientation observation");
			update_orientation = true;
			/* Don't update the orientation match time here - only do that on an actual match */
		}
		else {
			/* FIXME: If roll and pitch are acceptable (the gravity vector matched), but yaw is out of spec, we could perhaps do a
			 * yaw-only update for this device and see if that brings it into matching orientation */
		}

		if (update_position) {
			if (dev->last_acquired_pose_lock_ts == 0)
				dev->last_acquired_pose_lock_ts = dev->last_acquired_pose_lock_ts;

			if (update_position) {
				if (update_orientation) {
					rift_kalman_6dof_pose_update(&dev->ukf_fusion, dev->device_time_ns, &imu_pose, slot->slot_id);
				} else {
					rift_kalman_6dof_position_update(&dev->ukf_fusion, dev->device_time_ns, &imu_pose.pos, slot->slot_id);
				}
      }
			dev->last_observed_pose_ts = dev->device_time_ns;
			dev->last_observed_pose = imu_pose;
		}

		frame_fusion_slot = slot->slot_id;

		slot->had_used_report |= update_position;
	}

	rift_tracked_device_send_debug_printf(dev, local_ts, "{ \"type\": \"pose\", \"local-ts\": %llu, "
		"\"device-ts\": %u, \"frame-start-local-ts\": %llu, "
		"\"frame-local-ts\": %llu, \"frame-hmd-ts\": %u, "
		"\"frame-exposure-count\": %u, \"frame-device-ts\": %llu, \"frame-fusion-slot\": %d, "
		"\"source\": \"%s\", "
		"\"score-flags\": %d, \"update-position\": %d, \"update-orient\": %d, "
		"\"pos\" : [ %f, %f, %f ], "
		"\"orient\" : [ %f, %f, %f, %f ], "
		"\"capture-pos\" : [ %f, %f, %f ], "
		"\"capture-orient\" : [ %f, %f, %f, %f ], "
		"\"rot-std-dev\" : [ %f, %f, %f ], "
		"\"pos-std-dev\" : [ %f, %f, %f ] "
		"}",
		(unsigned long long) local_ts, dev->device_time_ns,
		(unsigned long long) frame_start_local_ts,
		(unsigned long long) exposure_info->local_ts, exposure_info->hmd_ts,
		exposure_info->count,
		(unsigned long long) frame_device_time_ns, frame_fusion_slot,
		source, score->match_flags, update_position, update_orientation,
		model_pose->pos.x, model_pose->pos.y, model_pose->pos.z,
		model_pose->orient.x, model_pose->orient.y, model_pose->orient.z, model_pose->orient.w,
		dev_info->capture_pose.pos.x, dev_info->capture_pose.pos.y,
		dev_info->capture_pose.pos.z, dev_info->capture_pose.orient.x,
		dev_info->capture_pose.orient.y, dev_info->capture_pose.orient.z,
		dev_info->capture_pose.orient.w,
		dev_info->rot_error.x, dev_info->rot_error.y, dev_info->rot_error.z,
		dev_info->pos_error.x, dev_info->pos_error.y, dev_info->pos_error.z
	);
	ohmd_unlock_mutex (dev->device_lock);

	return update_position || update_orientation;
}

/* Called with the device lock held */
void rift_tracked_device_get_model_pose_locked(rift_tracked_device_priv *dev, uint32_t device_ts, posef *pose, vec3f *pos_error, vec3f *rot_error)
{
	posef imu_global_pose, model_pose;
	vec3f global_pos_error, global_rot_error;

	rift_kalman_6dof_get_pose_at(&dev->ukf_fusion, dev->device_time_ns, &imu_global_pose, NULL, NULL, NULL, &global_pos_error, &global_rot_error);

	/* Apply the pose conversion from IMU->model */
	oposef_apply(&dev->model_from_fusion, &imu_global_pose, &model_pose);

	if (pos_error) {
		int i;

		for (i = 0; i < 3; i++) {
			if (global_pos_error.arr[i] < MIN_POS_ERROR)
				global_pos_error.arr[i] = MIN_POS_ERROR;
		}

		oquatf_get_rotated_abs(&dev->model_from_fusion.orient, &global_pos_error, pos_error);
	}
	if (rot_error) {
		int i;

		for (i = 0; i < 3; i++) {
			if (global_rot_error.arr[i] < MIN_ROT_ERROR)
				global_rot_error.arr[i] = MIN_ROT_ERROR;
		}

		oquatf_get_rotated_abs(&dev->model_from_fusion.orient, &global_rot_error, rot_error);
	}

	dev->model_pose.orient = model_pose.orient;
	if (dev->last_observed_pose_ts != 0 && dev->device_time_ns - dev->last_observed_pose_ts < (POSE_LOST_THRESHOLD * 1000000UL)) {
		/* Don't let the device move unless there's a recent observation of actual position */
		dev->model_pose.pos = model_pose.pos;
	}
	*pose = dev->model_pose;

	LOGD ("Reporting pose for dev %d, orient %f %f %f %f pos %f %f %f",
		dev->base.id,
		pose->orient.x, pose->orient.y, pose->orient.z, pose->orient.w,
		pose->pos.x, pose->pos.y, pose->pos.z);
}

/* Called with the device lock held */
static void
rift_tracked_device_send_imu_debug(rift_tracked_device_priv *dev)
{
	int i;

	if (dev->num_pending_imu_observations == 0)
		return;

	if (dev->debug_metadata && ohmd_pw_debug_stream_connected(dev->debug_metadata)) {
		char debug_str[1024];

		for (i = 0; i < dev->num_pending_imu_observations; i++) {
			rift_tracked_device_imu_observation *obs = dev->pending_imu_observations + i;

			snprintf (debug_str, 1024, ",\n{ \"type\": \"imu\", \"local-ts\": %llu, "
				 "\"device-ts\": %llu, \"dt\": %f, "
				 "\"ang_vel\": [ %f, %f, %f ], \"accel\": [ %f, %f, %f ], "
				 "\"mag\": [ %f, %f, %f ] }",
				(unsigned long long) obs->local_ts,
				(unsigned long long) obs->device_ts,
				obs->dt,
				obs->ang_vel.x, obs->ang_vel.y, obs->ang_vel.z,
				obs->accel.x, obs->accel.y, obs->accel.z,
				obs->mag.x, obs->mag.y, obs->mag.z);

			debug_str[1023] = '\0';

			ohmd_pw_debug_stream_push (dev->debug_metadata, obs->local_ts, debug_str);
		}
	}

	dev->num_pending_imu_observations = 0;
}

static void
rift_tracked_device_send_debug_printf(rift_tracked_device_priv *dev, uint64_t local_ts, const char *fmt, ...)
{
	if (dev->debug_metadata && ohmd_pw_debug_stream_connected(dev->debug_metadata)) {
		char debug_str[1024];
		va_list args;

		/* Send any pending IMU debug first */
		rift_tracked_device_send_imu_debug(dev);

		/* Print output string and send */
		va_start(args, fmt);
		vsnprintf(debug_str, 1024, fmt, args);
		va_end(args);

		debug_str[1023] = '\0';

		ohmd_pw_debug_stream_push (dev->debug_metadata, local_ts, debug_str);
	}
}

static rift_tracker_pose_delay_slot *
find_free_delay_slot(rift_tracked_device_priv *dev)
{
	/* Pose observation delay slots */
	for (int i = 0; i < dev->n_delay_slots; i++) {
		int slot_no = dev->delay_slot_index;
		rift_tracker_pose_delay_slot *slot = dev->delay_slots + slot_no;

		/* Cycle through the free delay slots */
		dev->delay_slot_index = (slot_no+1) % dev->n_delay_slots;

		if (slot->use_count == 0)
			return slot;
	}

	/* Failed to find a free slot */
	return NULL;
}

static rift_tracker_pose_delay_slot *
reclaim_delay_slot(rift_tracked_device_priv *dev)
{
	/* Pose observation delay slots */
	for (int i = 0; i < dev->n_delay_slots; i++) {
		rift_tracker_pose_delay_slot *slot = dev->delay_slots + i;

		/* If a slot already received a pose observation, use that one */
		/* FIXME: Check that the poses were integrated, and integrate them as-needed if not */
		if (slot->valid && slot->had_used_report)
			return slot;
	}

	/* Failed to find a free slot */
	return NULL;
}


static rift_tracker_pose_delay_slot *
get_matching_delay_slot(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info)
{
	rift_tracker_pose_delay_slot *slot = NULL;
	int slot_no = dev_info->fusion_slot;

	if (slot_no >= 0 && slot_no < dev->n_delay_slots) {
		slot = dev->delay_slots + slot_no;
	}

	if (slot && slot->valid && slot->device_time_ns == dev_info->device_time_ns)
		return slot;

	return NULL;
}

/* Called with the device lock held. Allocate a delay slot and populate the device exposure info */
static void
rift_tracked_device_on_new_exposure(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info) {
	rift_tracker_pose_delay_slot *slot = find_free_delay_slot(dev);

	dev_info->device_time_ns = dev->device_time_ns;

	if (slot == NULL) {
		/* We might reclaim a busy delay slot if some frame search is being slow and we already got an observation from another camera */
		slot = reclaim_delay_slot(dev);
		if (slot) {
			LOGV ("Reclaimed delay slot %d for dev %d, ts %llu (delay %f)", slot->slot_id, dev->base.id, (unsigned long long) dev->device_time_ns,
				(double) (dev->device_time_ns - slot->device_time_ns) / 1000000000.0);
		}
	}

	if (dev->last_observed_pose_ts != 0 && dev->device_time_ns - dev->last_observed_pose_ts < (POSE_LOST_THRESHOLD * 1000000UL))
		dev_info->had_pose_lock = true;
	else {
		dev_info->had_pose_lock = false;
		if (dev->last_acquired_pose_lock_ts != 0) {
			LOGV("Device %d Lost pose_lock at TS %" PRIu64 " after %" PRIu64 "\n",
			    dev->base.id, dev->device_time_ns, dev->device_time_ns - dev->last_acquired_pose_lock_ts);
			dev->last_acquired_pose_lock_ts = 0;
		}
	}
	dev_info->last_acquired_pose_lock_ts = dev->last_acquired_pose_lock_ts;

	rift_tracked_device_get_model_pose_locked(dev, dev->device_time_ns, &dev_info->capture_pose, &dev_info->pos_error, &dev_info->rot_error);

	if (slot) {
		slot->device_time_ns = dev_info->device_time_ns;
		slot->valid = true;
		slot->use_count = 0;
		slot->had_used_report = false;

		LOGD ("Assigning free delay slot %d for dev %d, ts %llu", slot->slot_id, dev->base.id, (unsigned long long) dev->device_time_ns);
		dev_info->fusion_slot = slot->slot_id;

		/* Tell the kalman filter to prepare the delay slot */
		rift_kalman_6dof_prepare_delay_slot(&dev->ukf_fusion, dev_info->device_time_ns, slot->slot_id);

		/* Clear the last no-free-delay-slot tracking to avoid logging noise */
		dev->last_no_free_delay_slot = 0;
	}
	else {
		if (dev->last_no_free_delay_slot != 0 && (dev->device_time_ns - dev->last_no_free_delay_slot > (NO_FREE_DELAY_SLOT_THRESHOLD * 1000000UL))) {
			LOGV("No free delay slot for dev %d @ ts %llu (for %ums now)", dev->base.id,
				  (unsigned long long) dev->device_time_ns, NO_FREE_DELAY_SLOT_THRESHOLD);
		}
		dev->last_no_free_delay_slot = dev->device_time_ns;
		dev_info->fusion_slot = -1;
	}
}

static int
rift_tracked_device_exposure_claim(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info)
{
	rift_tracker_pose_delay_slot *slot = get_matching_delay_slot(dev, dev_info);

	/* There is a delay slot for this frame, claim it */
	if (slot) {
		slot->use_count++;

		LOGD ("Claimed delay slot %d for dev %d, ts %llu. use_count now %d",
			slot->slot_id, dev->base.id, (unsigned long long) dev_info->device_time_ns, slot->use_count);

		return slot->slot_id;
	}
	else {
		/* The slot was not allocated (we missed the exposure event), or it
		 * was overridden by a later exposure because there's not enough slots */
		if (dev_info->fusion_slot != -1) {
#if LOGLEVEL == 0
			rift_tracker_pose_delay_slot *slot = dev->delay_slots + dev_info->fusion_slot;

			LOGD ("Lost delay slot %d for dev %d, ts %llu (slot valid %d ts %llu)",
				dev_info->fusion_slot, dev->base.id, (unsigned long long) dev_info->device_time_ns,
				slot->valid, (unsigned long long) slot->device_time_ns);
#endif
		}
	}

	return -1;
}

static void
rift_tracked_device_exposure_release_locked(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info)
{
	rift_tracker_pose_delay_slot *slot = get_matching_delay_slot(dev, dev_info);

	/* There is a delay slot for this frame, release it */
	if (slot) {
		if (slot->use_count > 0) {
			slot->use_count--;
			LOGD ("Released delay slot %d for dev %d, ts %llu. use_count now %d",
				dev_info->fusion_slot, dev->base.id, (unsigned long long) dev_info->device_time_ns,
				slot->use_count);
			/* Clear the fusion slot in the dev info now that it's released */
			dev_info->fusion_slot = -1;
		}
	}
}

void rift_tracked_device_frame_release(rift_tracked_device *dev_base, rift_tracker_exposure_info *exposure_info)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);

	ohmd_lock_mutex (dev->device_lock);
	if (dev->index < exposure_info->n_devices) {
		/* This device existed when the exposure was taken and therefore has info */
		rift_tracked_device_exposure_info *dev_info = exposure_info->devices + dev->index;
		/* Check that the exposure_info matches the device we expect */
		OHMD_ASSERT(dev_info->device_index == dev->index);
		rift_tracked_device_exposure_release_locked(dev, dev_info);
	}
	ohmd_unlock_mutex (dev->device_lock);
}

void rift_tracked_device_get_export(rift_tracked_device *dev_base, rift_tracked_device_export *out)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);
	posef imu_global_pose;

	/* Takes and releases device_lock itself; keeps upstream's hold/filter semantics for the view pose */
	rift_tracked_device_get_view_pose(dev_base, &out->view_pose, &out->view_lin_vel, &out->view_lin_accel, &out->view_ang_vel);

	ohmd_lock_mutex (dev->device_lock);
	rift_kalman_6dof_get_pose_at(&dev->ukf_fusion, dev->device_time_ns, &imu_global_pose, NULL, NULL,
		&out->gyro_raw, &out->pos_error, &out->rot_error);
	oposef_apply(&dev->device_from_fusion, &imu_global_pose, &out->raw_pose);
	rift_kalman_6dof_get_gyro_bias(&dev->ukf_fusion, &out->gyro_bias);
	out->device_time_ns = dev->device_time_ns;
	out->last_observed_pose_ts = dev->last_observed_pose_ts;
	out->last_observed_orient_ts = dev->last_observed_orient_ts;
	ohmd_unlock_mutex (dev->device_lock);
}

void rift_tracker_set_images_wanted(rift_tracker_ctx *ctx,
		int (*wanted)(const char *serial, uint64_t lifetime, void *data), void *data)
{
	ohmd_lock_mutex(ctx->roster_lock);
	for (size_t i = 0; i < ctx->n_sensors; i++) {
		rift_sensor_ctx *sensor = ctx->sensors[i];
		rift_sensor_set_images_wanted(sensor, !wanted || wanted(rift_sensor_serial_no(sensor), rift_sensor_lifetime(sensor), data));
	}
	ohmd_unlock_mutex(ctx->roster_lock);
}

void rift_tracker_set_search_skip(rift_tracker_ctx *ctx, uint32_t device_mask)
{
	__atomic_store_n(&ctx->cv1touch_search_skip, device_mask, __ATOMIC_RELAXED);
}

uint32_t rift_tracker_get_search_skip(rift_tracker_ctx *ctx)
{
	return __atomic_load_n(&ctx->cv1touch_search_skip, __ATOMIC_RELAXED);
}

int rift_tracker_get_sensor_count(rift_tracker_ctx *ctx)
{
	return (int)ctx->n_sensors;
}

int rift_tracker_get_sensor_stream(rift_tracker_ctx *ctx, int index, ohmd_cv1touch_sensor_stream *out)
{
	if (index < 0 || (size_t)index >= ctx->n_sensor_streams || out == NULL)
		return -1;
	*out = ctx->sensor_streams[index].stream;
	return 0;
}

/* Consumes pin_fd on success and failure. Private records own exactly one pin. */
static bool record_sensor_stream(rift_tracker_ctx *ctx,
    const ohmd_cv1touch_sensor_stream *stream, int pin_fd)
{
    struct sensor_stream_record record = {.stream = *stream, .pin_fd = pin_fd};
    if (pin_fd < 0 && record.stream.attachment.kind == OHMD_CV1TOUCH_ATTACHMENT_SYSFS)
        record.stream.attachment = (ohmd_cv1touch_sensor_attachment){0};
    for (size_t i = 0; i < ctx->n_sensor_streams; i++) {
        if (cv1touch_sensor_serial_equal(ctx->sensor_streams[i].stream.serial, stream->serial)) {
            rift_sensor_close_attachment_pin(ctx->sensor_streams[i].pin_fd);
            ctx->sensor_streams[i] = record;
            return true;
        }
    }
    if (ctx->n_sensor_streams >= INT_MAX) goto fail;
    if (ctx->n_sensor_streams == ctx->sensor_stream_capacity) {
        size_t capacity = ctx->sensor_stream_capacity ? ctx->sensor_stream_capacity * 2 : 4;
        if (capacity > INT_MAX) capacity = INT_MAX;
        if (capacity > SIZE_MAX / sizeof(*ctx->sensor_streams)) goto fail;
        void *next = realloc(ctx->sensor_streams, capacity * sizeof(*ctx->sensor_streams));
        if (!next) goto fail;
        ctx->sensor_streams = next;
        ctx->sensor_stream_capacity = capacity;
    }
    ctx->sensor_streams[ctx->n_sensor_streams++] = record;
    return true;
fail:
    rift_sensor_close_attachment_pin(pin_fd);
    return false;
}

bool rift_tracker_record_sensor_stream(rift_tracker_ctx *ctx, const ohmd_cv1touch_sensor_stream *stream)
{
    return record_sensor_stream(ctx, stream, -1);
}

void rift_tracker_forget_sensor_stream(rift_tracker_ctx *ctx, const char *serial)
{
    for (size_t i = 0; i < ctx->n_sensor_streams; i++) {
        if (cv1touch_sensor_serial_equal(ctx->sensor_streams[i].stream.serial, serial)) {
            rift_sensor_close_attachment_pin(ctx->sensor_streams[i].pin_fd);
            ctx->n_sensor_streams--;
            memmove(ctx->sensor_streams + i, ctx->sensor_streams + i + 1,
                (ctx->n_sensor_streams - i) * sizeof(*ctx->sensor_streams));
            return;
        }
    }
}

int rift_tracker_get_sensor_video(rift_tracker_ctx *ctx, int index, ohmd_cv1touch_sensor_video *out)
{
	if (index < 0 || (size_t)index >= ctx->n_sensors || out == NULL)
		return -1;
	rift_sensor_get_video_info(ctx->sensors[index], out);
	return 0;
}

int rift_tracker_get_sensor_video_quality(rift_tracker_ctx *ctx, int index, ohmd_cv1touch_sensor_video_quality *out)
{
	if (index < 0 || (size_t)index >= ctx->n_sensors || out == NULL)
		return -1;
	rift_sensor_get_video_quality(ctx->sensors[index], out);
	return 0;
}

int rift_tracker_get_sensor_info(rift_tracker_ctx *ctx, int index, ohmd_cv1touch_sensor *out)
{
	if (index < 0 || index >= ctx->n_sensors)
		return -1;
	rift_sensor_get_cv1touch_info(ctx->sensors[index], out);
	return 0;
}

int rift_tracker_set_sensor_pose(rift_tracker_ctx *ctx, const ohmd_cv1touch_sensor_pose *pose)
{
	for (int i = 0; i < ctx->n_sensors; i++) {
		if (rift_sensor_set_cv1touch_pose(ctx->sensors[i], pose))
			return 0;
	}
	return -1;
}

int rift_tracker_clear_sensor_pose(rift_tracker_ctx *ctx, const char *serial, uint64_t timeout_ns, uint32_t inspection_mask)
{
	for (int i = 0; i < ctx->n_sensors; i++) {
		if (rift_sensor_clear_cv1touch_pose(ctx->sensors[i], serial, timeout_ns, inspection_mask))
			return 0;
	}
	return -1;
}

int rift_tracker_sensor_pose_applied(rift_tracker_ctx *ctx, const char *serial)
{
	for (int i = 0; i < ctx->n_sensors; i++) {
		int applied = rift_sensor_cv1touch_pose_applied(ctx->sensors[i], serial);
		if (applied >= 0)
			return applied;
	}
	return -1;
}

/* Any thread. roster_lock is held through rift_sensor_retire: the engine thread takes a sensor
 * out of the roster under it before the sensor can be freed. */
int rift_tracker_retire_sensor(rift_tracker_ctx *ctx, const char *serial)
{
	ohmd_lock_mutex(ctx->roster_lock);
	for (size_t i = 0; i < ctx->n_sensors; i++) {
		if (cv1touch_sensor_serial_equal(rift_sensor_serial_no(ctx->sensors[i]), serial))
			rift_sensor_retire(ctx->sensors[i]);
	}
	ohmd_unlock_mutex(ctx->roster_lock);
	return 0;
}
