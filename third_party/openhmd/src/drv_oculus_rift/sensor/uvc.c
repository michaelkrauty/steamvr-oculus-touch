#include "../../log-output.h"
#include "../../ohmd-assert.h"
#include <errno.h>
#include <libusb.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../openhmdi.h"

#include "esp570.h"
#include "esp770u.h"
#include "uvc.h"
#include "uvc-layout.h"

#ifdef __linux__
#include <asm/byteorder.h>
#else
/* FIXME: Assumption of little endianness, which is pretty valid on non-LINUX */
#define __le32 uint32_t
#define __le16 uint16_t
#define __u8 uint8_t
#define __cpu_to_le16
#define __le16_to_cpu
#define __cpu_to_le32
#define __le32_to_cpu
#endif

/* cv1touch: clang-cl defines _MSC_VER and takes the GNU attribute; the pragma pair closed
 * before the struct body and packed nothing */
#if defined(__GNUC__) || defined(__clang__)
#define PACKED_STRUCT(name) struct __attribute__((packed)) name
#elif defined(_MSC_VER)
#define PACKED_STRUCT(name) \
    __pragma(pack(push, 1)) struct name __pragma(pack(pop))
#endif


#define SET_CUR 0x01
#define GET_CUR 0x81
#define TIMEOUT 1000

#define CONTROL_IFACE   0

#define VS_PROBE_CONTROL	1
#define VS_COMMIT_CONTROL	2

#define RIFT_SENSOR_CLOCK_FREQ	(40000000)
#define RIFT_SENSOR_WIDTH       1280
#define RIFT_SENSOR_HEIGHT      960
#define RIFT_SENSOR_FRAME_SIZE  (RIFT_SENSOR_WIDTH * RIFT_SENSOR_HEIGHT)

#define VERBOSE_DEBUG 0

PACKED_STRUCT(uvc_probe_commit_control) {
	__le16 bmHint;
	__u8 bFormatIndex;
	__u8 bFrameIndex;
	__le32 dwFrameInterval;
	__le16 wKeyFrameRate;
	__le16 wPFrameRate;
	__le16 wCompQuality;
	__le16 wCompWindowSize;
	__le16 wDelay;
	__le32 dwMaxVideoFrameSize;
	__le32 dwMaxPayloadTransferSize;
	__le32 dwClockFrequency;
	__u8 bmFramingInfo;
};

PACKED_STRUCT(uvc_payload_header) {
	__u8 bHeaderLength;
	__u8 bmHeaderInfo;
	__le32 dwPresentationTime;
	__le16 wSofCounter;
	__le32 scrSourceClock;
};

static void
rift_sensor_uvc_stream_release_frame(ohmd_video_frame *frame, rift_sensor_uvc_stream *stream);

int rift_sensor_uvc_set_cur(libusb_device_handle *dev, uint8_t interface, uint8_t entity,
		uint8_t selector, void *data, uint16_t wLength)
{
	uint8_t bmRequestType = LIBUSB_REQUEST_TYPE_CLASS |
		LIBUSB_RECIPIENT_INTERFACE;
	uint8_t bRequest = SET_CUR;
	uint16_t wValue = selector << 8;
	uint16_t wIndex = entity << 8 | interface;
	unsigned int timeout = TIMEOUT;
	int ret;

	ret = libusb_control_transfer(dev, bmRequestType, bRequest, wValue,
			wIndex, data, wLength, timeout);
	if (ret < 0) {
		ohmd_log_fprintf(stderr, "failed to transfer SET CUR %u %u: %d %d %m\n",
			entity, selector, ret, errno);
	}
	return ret;
}

int rift_sensor_uvc_get_cur(libusb_device_handle *dev, uint8_t interface, uint8_t entity,
		uint8_t selector, void *data, uint16_t wLength)
{
	uint8_t bmRequestType = 0x80 | LIBUSB_REQUEST_TYPE_CLASS |
		LIBUSB_RECIPIENT_INTERFACE;
	uint8_t bRequest = GET_CUR;
	uint16_t wValue = selector << 8;
	uint16_t wIndex = entity << 8 | interface;
	unsigned int timeout = TIMEOUT;
	int ret;

	ret = libusb_control_transfer(dev, bmRequestType, bRequest, wValue,
			wIndex, data, wLength, timeout);
	if (ret < 0) {
		ohmd_log_fprintf(stderr, "failed to transfer GET CUR %u %u: %d %d %m\n",
			entity, selector, ret, errno);
	}
	return ret;
}

#define UVC_DISCARD_REPORT_NS 10000000000ULL

/* Plain names for the log summary: "2 incomplete, 1 with a damaged header" */
static const char *const uvc_discard_names[UVC_DISCARD_REASONS] = {
	[UVC_DISCARD_INCOMPLETE] = "incomplete",
	[UVC_DISCARD_TOO_LONG] = "too long",
	[UVC_DISCARD_HEADER] = "with a damaged header",
	[UVC_DISCARD_CAMERA_ERROR] = "marked faulty by the camera",
	[UVC_DISCARD_TRANSFER] = "hit by a USB transfer error",
	[UVC_DISCARD_SEQUENCE] = "out of sequence",
};

static uint64_t uvc_now(struct rift_sensor_uvc_stream *stream)
{
#ifdef _WIN32
	/* cv1touch: the platform layer's nanosecond clock, which is the daemon's clock there */
	return ohmd_monotonic_get(stream->ohmd_ctx);
#else
	struct timespec now;
	(void)stream;
	clock_gettime(CLOCK_MONOTONIC, &now);
	return (uint64_t)now.tv_sec * 1000000000 + now.tv_nsec;
#endif
}

static void report_discards(struct rift_sensor_uvc_stream *stream, uint64_t now)
{
	char reasons[256] = "";
	size_t used = 0;

	for (int i = 0; i < UVC_DISCARD_REASONS; i++) {
		if (stream->discards[i] == 0)
			continue;
		int n = snprintf(reasons + used, sizeof reasons - used, "%s%u %s", used ? ", " : "",
				stream->discards[i], uvc_discard_names[i]);
		if (n > 0)
			used = OHMD_MIN(used + (size_t)n, sizeof reasons - 1);
		stream->discards[i] = 0;
	}
	unsigned secs = (unsigned)((now - stream->discard_window_start + 500000000) / 1000000000);
	if (stream->n_discards != 0) {
		char packets[80] = "";
		if (stream->bad_packets != 0)
			snprintf(packets, sizeof packets, "; %u more damaged packets while skipping images", stream->bad_packets);
		LOGW("Sensor %d video: discarded %u damaged image%s in the last %u s (%s)%s", stream->sensor_id,
				stream->n_discards, stream->n_discards == 1 ? "" : "s", secs, reasons, packets);
	} else {
		LOGW("Sensor %d video: received %u damaged packets in the last %u s while waiting for an image boundary",
				stream->sensor_id, stream->bad_packets, secs);
	}
	stream->n_discards = 0;
	stream->bad_packets = 0;
}

/* Bad traffic advances the reporting window even when an image was already discarded.
 * Keep that packet count separate: an unframed packet is not another rejected image. */
static void count_discard(struct rift_sensor_uvc_stream *stream, enum rift_sensor_uvc_discard reason,
		bool image)
{
	uint64_t now = uvc_now(stream);

	if (stream->n_discards == 0 && stream->bad_packets == 0)
		stream->discard_window_start = now;
	if (image) {
		stream->n_discards++;
		stream->discards[reason]++;
	} else {
		stream->bad_packets++;
	}
	if (now >= stream->discard_window_start + UVC_DISCARD_REPORT_NS)
		report_discards(stream, now);
}

/* Ignore the image's remaining packets until a valid boundary. Keep our buffer: it is
 * still owned by the assembler and can be reused directly. */
static void drop_frame(struct rift_sensor_uvc_stream *stream)
{
	stream->skip_frame = true;
	stream->frame_collected = 0;
	if (stream->cur_frame)
		stream->cur_frame->data_size = 0;
}

/* Discard the image being assembled and count it once. Damage between a delivered image
 * and the next boundary costs nothing: that next image still has to start at a boundary. */
static void reject_frame(struct rift_sensor_uvc_stream *stream, enum rift_sensor_uvc_discard reason)
{
	bool image = !stream->frame_delivered && !stream->skip_frame;
	if (image)
		drop_frame(stream);
	count_discard(stream, reason, image);
}

static void deliver_frame(struct rift_sensor_uvc_stream *stream)
{
	uint64_t start_ts = stream->cur_frame->start_ts;

	stream->cur_frame->data_size = stream->frame_collected;
	if (stream->frame_cb) {
		stream->frame_cb(stream, stream->cur_frame, stream->frame_cb_data);
		stream->cur_frame = NULL;
	}
	stream->frame_collected = 0;
	stream->frame_delivered = true;

	/* Also report the end of an episode, timed by the stamp this image already has. */
	if ((stream->n_discards != 0 || stream->bad_packets != 0) &&
			start_ts >= stream->discard_window_start + UVC_DISCARD_REPORT_NS)
		report_discards(stream, start_ts);
}

/* An APP segment can contain a thumbnail's EOI. Skip marker payloads by their
 * lengths until SOS; the camera's baseline entropy stream then escapes FF bytes.
 * Header state spans USB packets, without scanning the image's pixel payload. */
static bool jpeg_scan_started(struct rift_sensor_uvc_stream *stream)
{
	if (stream->jpeg_scan_started || stream->jpeg_header_offset == SIZE_MAX)
		return stream->jpeg_scan_started;
	size_t size = stream->frame_collected;
	if (size < 2)
		return false;
	const unsigned char *data = stream->cur_frame->data;
	size_t offset = stream->jpeg_header_offset;
	if (offset == 0) {
		if (data[0] != 0xff || data[1] != 0xd8)
			goto invalid;
		offset = 2;
	}
	while (offset < size) {
		if (data[offset] != 0xff)
			goto invalid;
		while (offset + 1 < size && data[offset + 1] == 0xff)
			offset++;
		if (size - offset < 4)
			break;
		unsigned marker = data[offset + 1];
		if (marker == 0 || marker == 0xd8 || marker == 0xd9)
			goto invalid;
		if (marker == 1 || (marker >= 0xd0 && marker <= 0xd7)) {
			offset += 2;
			continue;
		}
		size_t length = ((unsigned)data[offset + 2] << 8) | data[offset + 3];
		if (length < 2)
			goto invalid;
		if (length > size - offset - 2)
			break;
		offset += 2 + length;
		if (marker == 0xda) {
			stream->jpeg_scan_started = true;
			break;
		}
	}
	stream->jpeg_header_offset = offset;
	return stream->jpeg_scan_started;
invalid:
	/* Still deliver at UVC EOF: the decoder supplies the image validation. */
	stream->jpeg_header_offset = SIZE_MAX;
	return false;
}

void process_payload(struct rift_sensor_uvc_stream *stream, unsigned char *payload,
		     size_t len)
{
	/* Empty service intervals are normal during sensor blanking. Nonempty
	 * packets must contain the entire header before any field is inspected. */
	if (len == 0)
		return;
	if (len < sizeof(struct uvc_payload_header)) {
		reject_frame(stream, UVC_DISCARD_HEADER);
		return;
	}
	struct uvc_payload_header *h = (struct uvc_payload_header *)payload;
	if (h->bHeaderLength != sizeof *h || !(h->bmHeaderInfo & 0x80)) {
		reject_frame(stream, UVC_DISCARD_HEADER);
		return;
	}
	int frame_id = h->bmHeaderInfo & 1;
	bool is_eof = (h->bmHeaderInfo & 2) != 0;
	/* A header without a PTS continues the current exposure. */
	uint32_t pts = (h->bmHeaderInfo & 4) ? __le32_to_cpu(h->dwPresentationTime) : stream->cur_pts;
	payload += sizeof *h;
	len -= sizeof *h;

	if (!stream->have_frame_id) {
		/* Opening the interface can land inside an image. Observe its identity
		 * but wait for the next real boundary before accepting any pixels. */
		stream->have_frame_id = true;
		stream->frame_id = frame_id;
		stream->cur_pts = pts;
		stream->skip_frame = true;
		return;
	}

	/* A new image needs a new PTS. A lone FID toggle can be a damaged header in the middle
	 * of the old image. After a delivery a new PTS alone also starts one: FID repeats when
	 * an even number of whole images was lost. */
	if (pts != stream->cur_pts && (frame_id != stream->frame_id || stream->frame_delivered)) {
		if (!stream->frame_delivered && !stream->skip_frame)
			reject_frame(stream, UVC_DISCARD_INCOMPLETE);
		bool skip = stream->reject_next_frame;
		stream->reject_next_frame = false;
		stream->frame_id = frame_id;
		stream->cur_pts = pts;
		stream->frame_collected = 0;
		stream->skip_frame = skip;
		stream->frame_delivered = false;
		stream->jpeg_header_offset = 0;
		stream->jpeg_scan_started = false;
	} else if (frame_id != stream->frame_id || pts != stream->cur_pts) {
		/* Never join pixels from different exposures just because FID was lost. */
		reject_frame(stream, UVC_DISCARD_SEQUENCE);
		return;
	}

	/* The camera's error flag belongs to this header's image, including its opening
	 * header. Adopt the boundary before deciding whether that candidate was rejected. */
	if (h->bmHeaderInfo & 0x40) {
		reject_frame(stream, UVC_DISCARD_CAMERA_ERROR);
		return;
	}

	if (stream->frame_delivered) {
		/* Extra raw data means the image size or boundary was wrong, so the next boundary
		 * is untrustworthy: discard that image too. libjpeg stops at EOI, so bytes after a
		 * delivered JPEG cannot change it. */
		if (len != 0 && stream->format != OHMD_VIDEO_FRAME_FORMAT_JPEG) {
			bool image = !stream->reject_next_frame;
			stream->reject_next_frame = true;
			count_discard(stream, UVC_DISCARD_TOO_LONG, image);
		}
		return;
	}
	if (stream->skip_frame)
		return;

	if (len != 0) {
		if (stream->cur_frame == NULL) {
			ohmd_lock_mutex(stream->frames_lock);
			if (stream->n_free_frames > 0)
				stream->cur_frame = stream->free_frames[--stream->n_free_frames];
			ohmd_unlock_mutex(stream->frames_lock);
		}
		if (stream->cur_frame == NULL) {
			if (stream->skip_frame_start == 0) {
				LOGW("No frame provided for pixel data. Skipping frames");
				stream->skip_frame_start = uvc_now(stream);
			}
			drop_frame(stream);
			return;
		}
		if (stream->frame_collected == 0) {
			/* Keep the first pixel packet's host time, regardless of later drops
			 * or header-only packets. Camera exposure matching uses this stamp. */
			stream->cur_frame->start_ts = uvc_now(stream);
			stream->cur_frame->pts = stream->cur_pts;
			stream->cur_frame->stride = stream->stride;
			stream->cur_frame->width = stream->width;
			stream->cur_frame->height = stream->height;
			stream->cur_frame->format = stream->format;
			stream->cur_frame->data_size = 0;

			if (stream->skip_frame_start != 0) {
				LOGW("Got capture frame after %f sec", (stream->cur_frame->start_ts - stream->skip_frame_start) / 1000000000.0);
				stream->skip_frame_start = 0;
			}
		}
		if (len > (size_t)(stream->frame_size - stream->frame_collected)) {
			reject_frame(stream, UVC_DISCARD_TOO_LONG);
			return;
		}
		memcpy(stream->cur_frame->data + stream->frame_collected, payload, len);
		stream->frame_collected += (int)len;
	}

	if (stream->format == OHMD_VIDEO_FRAME_FORMAT_JPEG) {
		/* Early EOI delivery is safe only after SOS, outside any APP/thumbnail data.
		 * EOF can also deliver padded or truncated input for the decoder to check. */
		if (stream->frame_collected >= 2 && (is_eof ||
				(jpeg_scan_started(stream) && stream->cur_frame->data[stream->frame_collected - 2] == 0xff &&
				 stream->cur_frame->data[stream->frame_collected - 1] == 0xd9))) {
			deliver_frame(stream);
			return;
		}
	} else if (stream->frame_collected == stream->frame_size) {
		/* No EOF/boundary wait: hand off on the packet that completes the image. */
		deliver_frame(stream);
		return;
	}
	if (is_eof)
		reject_frame(stream, UVC_DISCARD_INCOMPLETE);
}

/* Puts transfers parked after a failed submit back in flight, stopping at the first that fails again */
static void resubmit_parked(struct rift_sensor_uvc_stream *stream)
{
	while (stream->n_parked > 0 && stream->video_running) {
		int ret = libusb_submit_transfer(stream->parked[stream->n_parked - 1]);
		if (ret == LIBUSB_ERROR_NO_DEVICE)
			stream->n_parked = 0;
		if (ret < 0)
			return;
		stream->n_parked--;
		stream->active_transfers++;
	}
}

static void iso_transfer_cb(struct libusb_transfer *transfer)
{
	struct rift_sensor_uvc_stream *stream = transfer->user_data;
	int ret;
	int i;

	/* Handle error conditions */
	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		if (stream->video_running && transfer->status != LIBUSB_TRANSFER_CANCELLED)
			reject_frame(stream, UVC_DISCARD_TRANSFER);
		if (transfer->status != LIBUSB_TRANSFER_CANCELLED)
			LOGE("USB transfer error: %u", transfer->status);
		stream->active_transfers--;
		return;
	}

	if (!stream->video_running) {
		/* Not resubmitting. Reduce transfer count */
		stream->active_transfers--;
		return;
	}

	/* Handle contained isochronous packets */
	for (i = 0; i < transfer->num_iso_packets; i++) {
		unsigned char *payload;
		size_t payload_len;

		payload = libusb_get_iso_packet_buffer_simple(transfer, i);
		payload_len = transfer->iso_packet_desc[i].actual_length;
		if (transfer->iso_packet_desc[i].status != LIBUSB_TRANSFER_COMPLETED) {
			reject_frame(stream, UVC_DISCARD_TRANSFER);
			continue;
		}
		process_payload(stream, payload, payload_len);
	}

	/* Never wait inside a completion callback: libusb holds its context-wide
	 * event-waiter lock, so retrying here can block every camera and teardown.
	 * The engine update retries parked transfers even when none remain in flight. */
	ret = libusb_submit_transfer(transfer);
	if (ret >= 0)
		return;
	if (ret != LIBUSB_ERROR_NO_DEVICE) {
		LOGW("Parked USB xfer: %s", libusb_error_name(ret));
		stream->parked[stream->n_parked++] = transfer;
	}
	stream->active_transfers--;
}

int rift_sensor_uvc_stream_setup (ohmd_context* ohmd_ctx,
		libusb_context *usb_ctx, libusb_device_handle *devh,
		struct libusb_device_descriptor *desc,
		struct rift_sensor_uvc_stream *stream)
{
	int ret;
	int alt_setting;
	int num_packets;
	int packet_size;

	stream->frames_lock = ohmd_create_mutex(ohmd_ctx);
	if (!stream->frames_lock) return -ENOMEM;
	stream->ohmd_ctx = ohmd_ctx;
	stream->usb_ctx = usb_ctx;
	stream->devh = devh;
	stream->video_running = false;

	/* Skip the first frame */
	stream->skip_frame = true;
	stream->skip_frame_start = 0;

	ret = libusb_set_auto_detach_kernel_driver(devh, 1);
	/* cv1touch: there is no kernel driver to detach on Windows: libusb says not supported */
	if (ret < 0 && ret != LIBUSB_ERROR_NOT_SUPPORTED) {
		ohmd_log_printf("could not detach uvcvideo driver\n");
		return ret;
	}

	ret = libusb_claim_interface(devh, 0);
	if (ret < 0) {
		ohmd_log_printf("could not claim control interface\n");
		return ret;
	}

	ret = libusb_claim_interface(devh, 1);
	if (ret < 0) {
		ohmd_log_printf("could not claim UVC data interface\n");
		return ret;
	}

	bool is_usb2 = (desc->bcdUSB < 0x300);

	struct uvc_probe_commit_control control = {
		.bFormatIndex = 1,
		.bFrameIndex = 1,
	};

	switch (desc->idProduct) {
	case DK2_PID:
		control.dwFrameInterval = __cpu_to_le32(166666);
		control.dwMaxVideoFrameSize = __cpu_to_le32(752 * 480);
		control.dwMaxPayloadTransferSize = __cpu_to_le16(3000);

		stream->stride = 752;
		stream->width = 752;
		stream->height = 480;
		stream->format = OHMD_VIDEO_FRAME_FORMAT_GRAY8;

		num_packets = 32;
		packet_size = 3060;
		alt_setting = 7;

		esp570_setup_unknown_3(devh);
		break;
	case CV1_PID:
		if (is_usb2) {
			/* JPEG mode for USB 2 connection */
			control.bFormatIndex = 2;
			control.bFrameIndex = 2;
			packet_size = 2048;
			alt_setting = 2;
			stream->format = OHMD_VIDEO_FRAME_FORMAT_JPEG;
		} else {
			control.bFrameIndex = 4;
			packet_size = 16384;
			alt_setting = 2;
			stream->format = OHMD_VIDEO_FRAME_FORMAT_GRAY8;
		}

		control.dwFrameInterval = __cpu_to_le32(192000);
		control.dwMaxVideoFrameSize = __cpu_to_le32(RIFT_SENSOR_FRAME_SIZE);
		control.dwMaxPayloadTransferSize = __cpu_to_le16(packet_size);
		control.dwClockFrequency = __cpu_to_le32(RIFT_SENSOR_CLOCK_FREQ);

		stream->stride = RIFT_SENSOR_WIDTH;
		stream->width = RIFT_SENSOR_WIDTH;
		stream->height = RIFT_SENSOR_HEIGHT;

		if (rift_sensor_esp770u_init_regs(devh) < 0) {
			LOGE("Failed to init CV1 sensor");
			return -1;
		}
		break;
	default:
		ohmd_log_printf("Unknown / unhandled USB device VID/PID 0x%04x / 0x%04x\n",
			desc->idVendor, desc->idProduct);
		return -1;
	}

	ret = rift_sensor_uvc_set_cur(devh, 1, 0, VS_PROBE_CONTROL, &control,
			sizeof control);
	if (ret < 0)
		return ret;

	ret = rift_sensor_uvc_get_cur(devh, 1, 0, VS_PROBE_CONTROL, &control,
			sizeof control);
	if (ret < 0) {
		ohmd_log_printf("failed to get PROBE\n");
		return ret;
	}

	ret = rift_sensor_uvc_set_cur(devh, 1, 0, VS_COMMIT_CONTROL, &control,
			sizeof control);
	if (ret < 0) {
		ohmd_log_printf("failed to set COMMIT\n");
		return ret;
	}

	/* In JPEG mode, we have some extra init packets to send */
	if (desc->idProduct == CV1_PID && is_usb2) {
		ret = rift_sensor_esp770u_init_jpeg(devh);
		if (ret < 0) {
			LOGE("Failed to init CV1 sensor in JPEG mode");
			return ret;
		}
	}

	stream->alt_setting = alt_setting;
	stream->frame_size = stream->stride * stream->height;

	uvc_transfer_layout(stream->frame_size, packet_size, UVC_WHOLE_MS_TRANSFERS, &stream->num_transfers, &num_packets);

	stream->transfer = ohmd_alloc(ohmd_ctx, stream->num_transfers * sizeof(*stream->transfer));
	if (!stream->transfer)
		return -ENOMEM;
	stream->parked = ohmd_alloc(ohmd_ctx, stream->num_transfers * sizeof(*stream->parked));
	if (!stream->parked)
		return -ENOMEM;

	int n_dev_mem = 0;
	for (int i = 0; i < stream->num_transfers; i++) {
		stream->transfer[i] = libusb_alloc_transfer(num_packets);
		if (!stream->transfer[i]) {
			ohmd_log_fprintf(stderr, "failed to allocate isochronous transfer\n");
			return -ENOMEM;
		}

		uint8_t bEndpointAddress = 0x81;
		int transfer_size = num_packets * packet_size;
		/* Memory mapped from the device node, when usbfs gives some: the controller then writes
		 * into it directly, where a heap buffer costs a kmalloc'd bounce buffer (order 7 here, which
		 * fragmentation can fail) plus a clear and a copy on every transfer */
		void *buf = libusb_dev_mem_alloc(devh, transfer_size);
		if (buf != NULL)
			n_dev_mem++;
		else {
			buf = malloc(transfer_size);
			stream->transfer[i]->flags |= LIBUSB_TRANSFER_FREE_BUFFER;
		}

        if (!buf) return -ENOMEM;
		libusb_fill_iso_transfer(stream->transfer[i], devh,
					 bEndpointAddress, buf, transfer_size,
					 num_packets, iso_transfer_cb, stream,
					 TIMEOUT);
		libusb_set_iso_packet_lengths(stream->transfer[i], packet_size);
	}
	ohmd_log_printf("UVC: %d of %d transfers read into device memory\n", n_dev_mem, stream->num_transfers);

	return 0;
}

static int stream_start_failed(rift_sensor_uvc_stream *stream, int code, int os_error)
{
#ifndef __linux__
	os_error = 0; /* libusb's portable error code is available on every platform. */
#endif
	stream->start_status = (ohmd_cv1touch_sensor_stream){
		.error_kind = OHMD_CV1TOUCH_STREAM_ERROR_OTHER,
		.error_code = code,
		.os_error = os_error,
	};
#ifdef __linux__
	/* libusb v1.0.30's Linux SETINTERFACE and SUBMITURB paths log
	 * ENOSPC at ERROR before returning OTHER or IO. We never set a libusb
	 * log level, but LIBUSB_DEBUG can enable it. ENABLE_DEBUG_LOGGING
	 * builds force logging and ignore context options/callbacks;
	 * USE_SYSTEM_LOGGING_FACILITY can bypass global callbacks too.
	 * There is no effective-log-level getter, so a system library cannot
	 * be proven quiet. Logging may replace ioctl's errno even though we
	 * capture it immediately after return: ENOSPC is only a likely
	 * bandwidth cause, never proof (core.c/linux_usbfs.c).
	 * A fallback can use failure at full bandwidth followed by success at
	 * a lower setting as a measured basis for a stronger diagnosis. */
	if (os_error == ENOSPC)
		stream->start_status.error_kind = OHMD_CV1TOUCH_STREAM_ERROR_LIKELY_USB_BANDWIDTH;
#endif
	return code;
}

int rift_sensor_uvc_stream_start(rift_sensor_uvc_stream *stream, uint8_t min_frames,
	rift_sensor_uvc_stream_frame_cb frame_cb, void *frame_cb_data)
{
	int ret, i;

	/* A stopped stream retains buffers until its consumers have joined and clear runs. */
	if (stream->video_running || stream->alloced_frames || stream->free_frames)
		return stream_start_failed(stream, -EBUSY, EBUSY);

	errno = 0;
	ret = libusb_set_interface_alt_setting(stream->devh, 1, stream->alt_setting);
	const int start_errno = errno;
	if (ret) {
		stream_start_failed(stream, ret, start_errno);
		LOGE("Failed to set USB interface 1 alt setting %d: %s (%d)",
			stream->alt_setting, libusb_error_name(ret), ret);
		LOGE("Check kernel USB logs; if bandwidth is exhausted, move this sensor to a different USB host controller");
		return ret;
	}

	stream->video_running = true;
	stream->cur_frame = NULL;
	stream->have_frame_id = false;
	stream->skip_frame = true;
	stream->frame_delivered = stream->reject_next_frame = false;
	stream->frame_collected = 0;
	stream->jpeg_header_offset = 0;
	stream->jpeg_scan_started = false;

	stream->frame_cb = frame_cb;
	stream->frame_cb_data = frame_cb_data;

	/* Allocate frames and put on the free list */
	stream->alloced_frames = ohmd_alloc(stream->ohmd_ctx, min_frames * sizeof (ohmd_video_frame *));
	stream->free_frames = ohmd_alloc(stream->ohmd_ctx, min_frames * sizeof (ohmd_video_frame *));
    stream->n_alloced_frames = stream->n_free_frames = 0;
    if (!stream->alloced_frames || !stream->free_frames) goto allocation_failed;

	for (i = 0; i < min_frames; i++) {
		ohmd_video_frame *frame = ohmd_alloc(stream->ohmd_ctx, sizeof (ohmd_video_frame));
        if (!frame) goto allocation_failed;
		frame->data = ohmd_alloc(stream->ohmd_ctx, stream->frame_size);
        if (!frame->data) { free(frame); goto allocation_failed; }
		frame->data_block_size = stream->frame_size;
		frame->data_size = 0;

		frame->releasefn = (ohmd_video_frame_release_func) rift_sensor_uvc_stream_release_frame;
		frame->owner = stream;
		stream->alloced_frames[i] = stream->free_frames[i] = frame;
        stream->n_alloced_frames++;
	}
	stream->n_free_frames = stream->n_alloced_frames = min_frames;

	/* Submit transfers; any parked when the stream last stopped go in with the rest */
	stream->n_parked = 0;
    /* Completion callbacks share this lock: do not expose a partial active count. */
    libusb_lock_event_waiters(stream->usb_ctx);
    stream->active_transfers = 0;
	for (int i = 0; i < stream->num_transfers; i++) {
		errno = 0;
		ret = libusb_submit_transfer(stream->transfer[i]);
		const int submit_errno = errno;
		if (ret < 0) {
			stream_start_failed(stream, ret, submit_errno);
			LOGE("failed to submit iso transfer %d. Error %d errno %d", i, ret, errno);
            libusb_unlock_event_waiters(stream->usb_ctx);
			rift_sensor_uvc_stream_stop(stream);
			return ret;
		}
        stream->active_transfers++;
	}
    libusb_unlock_event_waiters(stream->usb_ctx);
	return 0;
allocation_failed:
	stream_start_failed(stream, -ENOMEM, ENOMEM);
	rift_sensor_uvc_stream_stop(stream);
	return -ENOMEM;
}

/* The engine thread owns stream lifetime; callbacks and maintenance serialize
 * parked/active counts under libusb's event-waiter lock. No sensor/tracker lock
 * is held by the caller, so callbacks may finish acquiring those locks. */
void rift_sensor_uvc_stream_retry(rift_sensor_uvc_stream *stream, uint64_t now_ns)
{
	if (now_ns < stream->retry_after_ns)
		return;
	/* At most one retry batch per 10 ms (less than a 52 Hz camera period).
	 * Failed batches stop at the first failure; no sleeps or unbounded retries. */
	stream->retry_after_ns = now_ns + 10000000;
	libusb_lock_event_waiters(stream->usb_ctx);
	resubmit_parked(stream);
	libusb_unlock_event_waiters(stream->usb_ctx);
}

void rift_sensor_uvc_stream_request_stop(rift_sensor_uvc_stream *stream)
{
	stream->video_running = false;
}

int rift_sensor_uvc_stream_stop(struct rift_sensor_uvc_stream *stream)
{
	int ret;

	/* Nonblocking stop requests can be broadcast before any stream drains. */
	rift_sensor_uvc_stream_request_stop(stream);

	ret = libusb_set_interface_alt_setting(stream->devh, 1, 0);
	if (ret) {
		LOGW("Failed to clear USB alt setting to 0 for sensor errno %d (%s)", errno, strerror(errno));
	}

	libusb_lock_event_waiters(stream->usb_ctx);

	/* Wait for active transfers to finish */
	while (stream->active_transfers > 0) {
		ret = libusb_wait_for_event(stream->usb_ctx, NULL);
		if (ret)
			break;
	}
	libusb_unlock_event_waiters(stream->usb_ctx);


	return 0;
}

int rift_sensor_uvc_stream_clear (struct rift_sensor_uvc_stream *stream)
{
	int i;
	OHMD_ASSERT(!stream->video_running);

	/* Analysis workers can own frames after USB stops. The device is cleared only
	 * after its owner joins those workers, so reclaim the frame storage here. */
	/* Free frames */
	for (i = 0; i < stream->n_alloced_frames; i++) {
		ohmd_video_frame *frame = stream->alloced_frames[i];
		free(frame->data);
		free(frame);
	}
	free(stream->alloced_frames);
	free(stream->free_frames);
	stream->alloced_frames = stream->free_frames = NULL;
	stream->cur_frame = NULL;
	stream->n_alloced_frames = stream->n_free_frames = 0;


	if (stream->transfer != NULL) {
		for (i = 0; i < stream->num_transfers; i++) {
			if (stream->transfer[i] != NULL) {
				struct libusb_transfer *t = stream->transfer[i];
				if (!(t->flags & LIBUSB_TRANSFER_FREE_BUFFER) && t->buffer != NULL)
					libusb_dev_mem_free(stream->devh, t->buffer, t->length);
				libusb_free_transfer(stream->transfer[i]);
				stream->transfer[i] = NULL;
			}
		}
		free(stream->transfer);
		stream->transfer = NULL;
		free(stream->parked);
		stream->parked = NULL;
	}

	if (stream->frames_lock != NULL) {
		ohmd_destroy_mutex(stream->frames_lock);
		stream->frames_lock = NULL;
	}

	return 0;
}

static void
rift_sensor_uvc_stream_release_frame(ohmd_video_frame *frame, rift_sensor_uvc_stream *stream)
{
	OHMD_ASSERT(frame->owner == stream);
	OHMD_ASSERT(stream->n_free_frames < stream->n_alloced_frames);
	OHMD_ASSERT(frame->data_block_size == stream->frame_size);

	frame->data_size = 0;

	/* Put the frame back on the free queue */
	ohmd_lock_mutex(stream->frames_lock);
	stream->free_frames[stream->n_free_frames++] = frame;
	ohmd_unlock_mutex(stream->frames_lock);
}
