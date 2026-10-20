#ifndef __UVC_H__
#define __UVC_H__

#include <libusb.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdint.h>
#include "openhmd_cv1touch.h"
#include "../ohmd-video.h"
#include "../rift-sensor-usb.h"

typedef struct rift_sensor_uvc_stream rift_sensor_uvc_stream;
typedef void (*rift_sensor_uvc_stream_frame_cb)(struct rift_sensor_uvc_stream *stream, ohmd_video_frame *frame, void *cb_data);

/* Why the assembler discarded an image, in the order the log summary lists them */
enum rift_sensor_uvc_discard {
	UVC_DISCARD_INCOMPLETE,
	UVC_DISCARD_TOO_LONG,
	UVC_DISCARD_HEADER,
	UVC_DISCARD_CAMERA_ERROR,
	UVC_DISCARD_TRANSFER,
	UVC_DISCARD_SEQUENCE,
	UVC_DISCARD_REASONS
};

struct rift_sensor_uvc_stream {
	ohmd_context* ohmd_ctx;
	/* Sensor number in log lines */
	int sensor_id;

	ohmd_video_frame_format format;
	int stride;
	int width;
	int height;

	ohmd_mutex *frames_lock;
	int n_free_frames;
	ohmd_video_frame **free_frames;

	/* Frame data destination */
	ohmd_video_frame *cur_frame;

	/* Total size of a full frame in bytes */
	int frame_size;
	/* Current frame ID */
	int frame_id;
	/* Current PTS being accumulated */
	uint32_t cur_pts;
	/* Number of bytes collected from the current frame */
	int frame_collected;
	/* true if we're skipping the current frame */
	bool skip_frame;
	/* An image starts only at an observed FID/PTS boundary. */
	bool have_frame_id;
	bool frame_delivered;
	bool reject_next_frame;
	/* Inspect segment lengths only until the camera's JPEG scan begins. */
	size_t jpeg_header_offset;
	bool jpeg_scan_started;

	/* Time at which we started skipping frames */
	uint64_t skip_frame_start;

	/* Images discarded since discard_window_start, logged at most every 10 s */
	uint32_t discards[UVC_DISCARD_REASONS];
	uint32_t n_discards;
	/* Damaged packets seen while no new image could be identified. */
	uint32_t bad_packets;
	uint64_t discard_window_start;

	/* USB streaming alt_setting */
	int alt_setting;
	/* Most recent failed start; copied before cleanup can change errno. */
	ohmd_cv1touch_sensor_stream start_status;

	int num_transfers;
	struct libusb_transfer **transfer;
	int active_transfers;
	/* Failed submissions, retried by the engine even if every transfer is parked */
	struct libusb_transfer **parked;
	int n_parked;
	uint64_t retry_after_ns; /* engine thread only */

	rift_sensor_uvc_stream_frame_cb frame_cb;
	void *frame_cb_data;

	libusb_context *usb_ctx;
	libusb_device_handle *devh;

	/* Stop must be visible to callbacks before the stopping thread takes libusb locks. */
	atomic_bool video_running;
	ohmd_video_frame **alloced_frames;
	int n_alloced_frames;
};

int rift_sensor_uvc_set_cur(libusb_device_handle *devh, uint8_t interface, uint8_t entity,
		uint8_t selector, void *data, uint16_t wLength);
int rift_sensor_uvc_get_cur(libusb_device_handle *devh, uint8_t interface, uint8_t entity,
		uint8_t selector, void *data, uint16_t wLength);

int rift_sensor_uvc_stream_setup (ohmd_context* ohmd_ctx, libusb_context *usb_ctx, libusb_device_handle *devh,
		struct libusb_device_descriptor *desc,
		rift_sensor_uvc_stream *stream);
int rift_sensor_uvc_stream_clear (rift_sensor_uvc_stream *stream);

int rift_sensor_uvc_stream_start(rift_sensor_uvc_stream *stream, uint8_t min_frames,
	rift_sensor_uvc_stream_frame_cb frame_cb, void *frame_cb_data);

void rift_sensor_uvc_stream_request_stop(rift_sensor_uvc_stream *stream);
void rift_sensor_uvc_stream_retry(rift_sensor_uvc_stream *stream, uint64_t now_ns);
int rift_sensor_uvc_stream_stop(rift_sensor_uvc_stream *stream);

/* This can only be called safely either before the stream is started
 * or from one of the callbacks */
bool rift_sensor_uvc_stream_set_frame(rift_sensor_uvc_stream *stream, ohmd_video_frame *frame);

#endif /* __UVC_H__ */
