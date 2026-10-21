/* cv1touch extensions to OpenHMD's Rift CV1 driver. Not part of upstream OpenHMD.
 * All host timestamps are CLOCK_MONOTONIC nanoseconds. */
#ifndef OPENHMD_CV1TOUCH_H
#define OPENHMD_CV1TOUCH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "openhmd.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A calibrated sensor pose: camera -> world, OpenCV camera axes (x right, y down, z forward). */
typedef struct ohmd_cv1touch_sensor_pose {
	char serial[32];                   /* NUL-terminated; empty = unused slot */
	float orient[4];                   /* x y z w */
	float pos[3];                      /* metres */
} ohmd_cv1touch_sensor_pose;

/* Zero-initialised by ohmd_ctx_create(); every field's zero value is the cv1touch default. */
typedef struct ohmd_cv1touch_config {
	/* > 0: each update blocks up to this many ms for the next controller radio report, so a
	 * dedicated update thread wakes as soon as controller data lands. 0: upstream polling. */
	int radio_wait_ms;
	/* Register the headset with the optical tracker. false: headset is radio + exposure sync only. */
	bool hmd_tracking;
	/* Do not light the headset's tracking LEDs. */
	bool hmd_leds_off;
	/* Rift sensor serials to leave closed (NUL-terminated; empty entries unused). */
	char ignore_sensor_serials[8][32];
	/* true: never place sensors at runtime (no provisional placement). Sensors listed in
	 * fixed_poses track with that pose; any other sensor only observes (calibration capture). */
	bool fixed_poses_only;
	ohmd_cv1touch_sensor_pose fixed_poses[8];
} ohmd_cv1touch_config;

typedef struct ohmd_cv1touch_controller {
	bool calibrated;                   /* factory calibration read and tracked device created */
	uint64_t device_time_ns;           /* controller clock (unwrapped) of the newest IMU sample */
	uint64_t received_host_ns;         /* host time the radio report carrying that sample was read */
	uint64_t filter_ns;                /* time that sample spent in the engine's IMU update, lock wait included */
	uint64_t last_pose_fix_device_ns;  /* newest accepted optical position fix, device clock; 0 = never */
	uint64_t last_orient_fix_device_ns;
	float view_orient[4];              /* x y z w: engine output pose (position hold + output filter) */
	float view_pos[3];
	float view_lin_vel[3];
	float view_ang_vel[3];
	float view_lin_acc[3];
	float raw_orient[4];               /* unfiltered UKF state mapped to the device frame */
	float raw_pos[3];
	float pos_stddev[3];               /* sqrt(diag(P)), fusion frame */
	float rot_stddev[3];
	float gyro_raw[3];                 /* rad/s, IMU frame, bias not removed */
	float gyro_bias[3];                /* UKF gyro bias estimate, IMU frame */
	uint8_t buttons;                   /* 0x1 A/X, 0x2 B/Y, 0x4 Oculus (right) / Menu (left), 0x8 stick */
	float trigger;
	float grip;
	float stick[2];
	float cap[5];                      /* stick, b_y, trigger, a_x, thumbrest; 1.0 = calibrated touch level, unclamped */
	int32_t device_id;                 /* OpenHMD device id: 1 right, 2 left */
	float imu_accel[3];                /* newest IMU sample, factory-calibrated, IMU frame: m/s^2 */
	float imu_gyro[3];                 /* same sample: rad/s */
} ohmd_cv1touch_controller;

#define OHMD_CV1TOUCH_STREAM_ERROR_NONE 0u
#define OHMD_CV1TOUCH_STREAM_ERROR_LIKELY_USB_BANDWIDTH 1u
#define OHMD_CV1TOUCH_STREAM_ERROR_OTHER 2u

/* Last USB video start attempt for each serial, separate from the sensor sample
 * ABI. A successful attempt clears the failure and records the setting it used.
 * Records can outlive unplugging: callers must forget on detach; engine close
 * frees all records. Current presence/video requires OS enumeration and counters.
 * Engine thread only. */
typedef struct ohmd_cv1touch_sensor_stream {
	char serial[32];
	uint32_t error_kind;
	int32_t error_code;                /* raw libusb/start return code, zero on success */
	int32_t os_error;                  /* Linux caller errno, possibly clobbered; zero if unavailable */
	int32_t alt_setting;               /* setting used by that attempt; zero if it failed */
	uint32_t reserved[4];              /* zero; room for additive diagnostics */
} ohmd_cv1touch_sensor_stream;

/* Separate acquisition diagnostics: frames delivered before any analysis lock or
 * queue. Serial-keyed and 64-bit; the existing released-buffer sample is unchanged. */
typedef struct ohmd_cv1touch_sensor_video {
	char serial[32];
	uint64_t frames_received;
} ohmd_cv1touch_sensor_video;

#define OHMD_CV1TOUCH_VIDEO_UNKNOWN 0
#define OHMD_CV1TOUCH_VIDEO_RAW 1
#define OHMD_CV1TOUCH_VIDEO_JPEG 2

/* Independent quality diagnostics; does not extend the 40-byte acquisition record.
 * USB3 uses raw video; USB2 uses JPEG in normal tracking and calibration too.
 * Decoded counts JPEG images accepted as packed GRAY8, not raw images. Dropped
 * counts rejected image candidates, excluding queue eviction/buffer starvation.
 * Counters are cumulative for this native device's lifetime, not logging windows. */
typedef struct ohmd_cv1touch_sensor_video_quality {
	char serial[32];
	uint64_t frames_decoded;
	uint64_t frames_dropped;
	uint32_t video_format;             /* OHMD_CV1TOUCH_VIDEO_* */
	uint32_t reserved;                 /* zero */
} ohmd_cv1touch_sensor_video_quality;

typedef struct ohmd_cv1touch_sensor {
	char serial[32];
	bool have_camera_pose;
	float camera_orient[4];            /* camera -> world */
	float camera_pos[3];
	uint32_t frames;                   /* cumulative frames released */
	uint32_t pose_fixes[3];            /* cumulative accepted optical fixes per OpenHMD device id: 0 headset, 1 right, 2 left */
	/* factory intrinsics, OpenCV fisheye model */
	int32_t width, height;
	float fx, fy, cx, cy;
	float k[4];
} ohmd_cv1touch_sensor;

typedef struct ohmd_cv1touch_blob {
	float x, y;
	uint16_t width, height;
	uint32_t area;
	int32_t led_id;                    /* (OpenHMD device id << 8) | LED index, or negative if unlabelled */
} ohmd_cv1touch_blob;

typedef struct ohmd_cv1touch_led {
	float pos[3];                      /* metres, controller model frame */
	float dir[3];                      /* unit LED normal */
} ohmd_cv1touch_led;                   /* 24 bytes */

typedef struct ohmd_cv1touch_frame_device {
	int32_t device_id;                 /* OpenHMD device id: 0 headset, 1 right, 2 left */
	uint32_t match_flags;              /* RIFT_POSE_MATCH_* of the final evaluation: 0x1 good, 0x2 strong */
	uint64_t device_time_ns;           /* device clock at this exposure */
	int32_t matched_blobs;
	int32_t visible_leds;
	uint32_t pose_valid;               /* 1: cam_orient/cam_pos hold this frame's pose */
	uint32_t reserved0;
	float cam_orient[4];               /* object -> camera (OpenCV camera axes), x y z w */
	float cam_pos[3];
	float reserved1;
} ohmd_cv1touch_frame_device;          /* 64 bytes */

typedef struct ohmd_cv1touch_frame_info {
	char serial[32];
	uint64_t frame_start_host_ns;
	uint32_t exposure_valid;
	uint32_t exposure_count;           /* HMD exposure counter (16-bit, wraps), shared by synced sensors */
	uint32_t exposure_hmd_ts;
	uint32_t n_devices;
	ohmd_cv1touch_frame_device devices[3];
} ohmd_cv1touch_frame_info;            /* 248 bytes */

#define OHMD_CV1TOUCH_REPORT_HMD 0
#define OHMD_CV1TOUCH_REPORT_RADIO 1

/* Called from sensor analysis threads with the sensor lock held: copy and return quickly. */
typedef void (*ohmd_cv1touch_frame_cb)(void *user, const ohmd_cv1touch_frame_info *info,
                                       int n_blobs, const ohmd_cv1touch_blob *blobs);
/* Called from sensor analysis threads, just before the frame callback for the same frame, with its
 * GRAY8 pixels (valid only during the call): copy what is needed and return quickly. */
typedef void (*ohmd_cv1touch_image_cb)(void *user, const ohmd_cv1touch_frame_info *info, int width, int height,
                                       int stride, const uint8_t *pixels);
/* Called from the update thread for every HID report read from the headset or its radio. */
typedef void (*ohmd_cv1touch_report_cb)(void *user, int source, uint64_t host_ns, const uint8_t *data, int len);

OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_cv1touch_configure(ohmd_context *ctx, const ohmd_cv1touch_config *config);
/* Complete lists, copied transactionally into the context. Call before opening devices.
 * Overrides the legacy inline eight-entry lists. Serials must be unique within each list,
 * 1..31 ASCII alphanumeric characters; poses must be finite with unit quaternions.
 * Returns -1 on invalid input/allocation or if a device is already open. */
OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_configure_sensors(ohmd_context *ctx,
        const char (*ignored)[32], size_t ignored_count,
        const ohmd_cv1touch_sensor_pose *poses, size_t pose_count);
OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_cv1touch_set_frame_callback(ohmd_context *ctx, ohmd_cv1touch_frame_cb cb, void *user);
OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_cv1touch_set_image_callback(ohmd_context *ctx, ohmd_cv1touch_image_cb cb, void *user);
OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_cv1touch_set_report_callback(ohmd_context *ctx, ohmd_cv1touch_report_cb cb, void *user);
OHMD_APIENTRYDLL uint32_t OHMD_APIENTRY ohmd_cv1touch_get_io_errors(ohmd_device *device);
/* Process-wide log receiver, installed before any engine worker starts and not changed while
 * workers run. Each callback borrows one complete line, at most 2048 bytes, with its original
 * prefix. Overlong lines are marked as truncated. The receiver must not block. With NULL,
 * ordinary diagnostics keep their original stdout/stderr streams; assertions still abort
 * without synchronous output. Does not change descriptor flags. */
typedef void (*ohmd_cv1touch_log_cb)(const char *bytes, size_t len);
OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_cv1touch_set_log_callback(ohmd_cv1touch_log_cb callback);
/* Same installation/lifetime rules. A native assertion enqueues its diagnostic, calls this
 * best-effort bounded drain (which must return even with a blocked writer), then aborts.
 * NULL skips the drain. It never calls ordinary stdio or joins native workers. */
typedef void (*ohmd_cv1touch_log_fatal_cb)(void);
OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_cv1touch_set_log_fatal_callback(ohmd_cv1touch_log_fatal_cb callback);
/* C-boundary panic: drain once and abort silently. Recursive receivers/fatal handlers skip
 * callbacks and the drain. Ordinary thread context only; not async-signal-safe. */
#ifdef _MSC_VER
__declspec(noreturn)
#else
__attribute__((noreturn))
#endif
OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_cv1touch_abort(void);

/* The engine logs to its C runtime's stdout, fully buffered when that is a pipe or a file. This
 * turns the buffering off, so each line goes out as it is written and none is lost when the
 * process ends abruptly. For Windows, where the library carries its own C runtime: elsewhere the
 * process's stdout is the engine's too. */
OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_cv1touch_unbuffer_log(void);

/* A decoder for radio reports read through a second handle on the radio's hidraw node (each reader
 * gets every report), for a thread of the daemon's own: it decodes on its own copy of each
 * controller's state and never waits on the engine. Only factory calibration comes from the
 * engine, once fetched; until then a controller's messages are skipped. Its device clocks are its
 * own (they start at its first sample), so its samples' device times only compare among
 * themselves. `device`: any device of the headset; the decoder must be freed before it closes. */
typedef struct ohmd_cv1touch_radio_decoder ohmd_cv1touch_radio_decoder;
OHMD_APIENTRYDLL ohmd_cv1touch_radio_decoder *OHMD_APIENTRY ohmd_cv1touch_radio_decoder_new(ohmd_device *device);
OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_cv1touch_radio_decoder_free(ohmd_cv1touch_radio_decoder *dec);
/* One radio report received at host time `received_ns`: fills `out[0..n)` (room for 2) with the
 * IMU samples and inputs of its calibrated, non-empty Touch messages and returns n. */
OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_radio_decode(ohmd_cv1touch_radio_decoder *dec, const unsigned char *buf, int size,
		uint64_t received_ns, ohmd_cv1touch_controller *out);
/* The controllers (bit 1 << OpenHMD device id: 1 right, 2 left) the engine's brute-force pose search
 * leaves out while the decoder lives, because its owner tracks them itself or they are asleep. The
 * engine still matches them against its own priors, and a sensor with no pose yet still searches
 * for them. Freeing the decoder searches for all again. */
OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_cv1touch_radio_decoder_set_search_skip(ohmd_cv1touch_radio_decoder *dec,
		uint32_t device_mask);
/* Stable factory-calibration identity, 32 lowercase hex bytes + NUL. Returns -1 until
 * calibrated or on invalid arguments. Engine thread only. Does not change sample ABI. */
OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_get_controller_identity(ohmd_device *device, char *out, int size);
OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_get_controller(ohmd_device *device, ohmd_cv1touch_controller *out);
/* The controller's factory LED model: number of LEDs written (0 until read over the radio), -1 on error. */
OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_get_led_model(ohmd_device *device, ohmd_cv1touch_led *out, int max);
OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_get_sensor_count(ohmd_device *device);
OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_get_sensor(ohmd_device *device, int index, ohmd_cv1touch_sensor *out);
/* Current sensor acquisition counters; indices can change after discovery.
 * Returns -1 past the last record. Engine thread only; counters are atomic. */
OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_get_sensor_video(ohmd_device *device, int index, ohmd_cv1touch_sensor_video *out);
/* Current quality counters and negotiated input format. Same enumeration and
 * engine-thread-only rules as acquisition; no locks or device I/O. */
OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_get_sensor_video_quality(ohmd_device *device, int index, ohmd_cv1touch_sensor_video_quality *out);
/* Last start-attempt results, keyed by serial, including failed starts. Indices
 * may change after device discovery; returns -1 past the last record. Engine thread only. */
OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_get_sensor_stream(ohmd_device *device, int index, ohmd_cv1touch_sensor_stream *out);
/* Forget a detached serial without freeing its native sensor object. Idempotent;
 * returns 0, or -1 on invalid arguments/context. Engine thread only. */
OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_forget_sensor_stream(ohmd_device *device, const char *serial);
/* Gives the sensor pose->serial names the camera -> world pose it tracks with from now on, as its
 * fixed pose at start did; applied by the sensor's own analysis thread before its next frame.
 * Returns 0, or -1 when no such sensor is open. */
OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_set_sensor_pose(ohmd_device *device, const ohmd_cv1touch_sensor_pose *pose);
/* Takes the open sensor serial's camera -> world pose away: it then labels the blobs of every
 * device in every frame (the long search, as a sensor with no pose yet does) and tracks with
 * nothing, until ohmd_cv1touch_set_sensor_pose gives it a pose again. Returns 0, or -1 when no
 * such sensor is open. */
OHMD_APIENTRYDLL int OHMD_APIENTRY ohmd_cv1touch_clear_sensor_pose(ohmd_device *device, const char *serial);

#ifdef __cplusplus
}
#endif

#endif
