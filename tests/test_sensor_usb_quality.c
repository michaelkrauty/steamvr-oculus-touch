// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* Real USB constructor, assembler and quality getter. Only device operations and
 * UVC negotiation are substituted; the constructor must install the quality wiring. */
#define frame_captured_cb usb_frame_captured_cb
#define rift_sensor_uvc_stream_setup fake_uvc_setup
#include "rift-sensor-usb.c"
#undef rift_sensor_uvc_stream_setup
#undef frame_captured_cb
#include "sensor/uvc.c"
#include "rift-sensor.c"
#include "test_util.h"

static uint16_t usb_version;
static int closes, callbacks, locks_destroyed;

void *ohmd_allocfn(ohmd_context *ctx, const char *message, size_t size)
{ (void)ctx; (void)message; return calloc(1, size); }
void ohmd_lock_mutex(ohmd_mutex *mutex) { CHECK(mutex == (void *)1); }
void ohmd_unlock_mutex(ohmd_mutex *mutex) { CHECK(mutex == (void *)1); }
void ohmd_destroy_mutex(ohmd_mutex *mutex)
{ CHECK(mutex == (void *)1); locks_destroyed++; }

libusb_device *LIBUSB_CALL libusb_get_device(libusb_device_handle *handle)
{ CHECK(handle == (void *)2); return (void *)3; }
int LIBUSB_CALL libusb_get_device_descriptor(libusb_device *device, struct libusb_device_descriptor *desc)
{
    CHECK(device == (void *)3);
    *desc = (struct libusb_device_descriptor){.idVendor = 0x2833, .idProduct = CV1_PID, .bcdUSB = usb_version};
    return 0;
}
void LIBUSB_CALL libusb_close(libusb_device_handle *handle)
{ CHECK(handle == (void *)2); closes++; }
int LIBUSB_CALL libusb_set_interface_alt_setting(libusb_device_handle *handle, int interface, int alt)
{ CHECK(handle == (void *)2 && interface == 1 && (alt == 0 || alt == 2)); return 0; }
int LIBUSB_CALL libusb_submit_transfer(struct libusb_transfer *transfer)
{ (void)transfer; CHECK(false); return -1; }
void LIBUSB_CALL libusb_lock_event_waiters(libusb_context *ctx) { CHECK(ctx == (void *)4); }
void LIBUSB_CALL libusb_unlock_event_waiters(libusb_context *ctx) { CHECK(ctx == (void *)4); }
int LIBUSB_CALL libusb_wait_for_event(libusb_context *ctx, struct timeval *timeout)
{ (void)ctx; (void)timeout; CHECK(false); return -1; }

int fake_uvc_setup(ohmd_context *ctx, libusb_context *usb_ctx, libusb_device_handle *handle,
        struct libusb_device_descriptor *desc, rift_sensor_uvc_stream *stream)
{
    CHECK(handle == (void *)2 && usb_ctx == (void *)4);
    stream->ohmd_ctx = ctx;
    stream->usb_ctx = usb_ctx;
    stream->devh = handle;
    /* A negotiated input format, not the public format or counter pointer. */
    stream->format = desc->bcdUSB < 0x300 ? OHMD_VIDEO_FRAME_FORMAT_JPEG : OHMD_VIDEO_FRAME_FORMAT_GRAY8;
    stream->width = stream->height = stream->stride = 128;
    stream->frame_size = 128 * 128;
    stream->frames_lock = (void *)1;
    stream->alt_setting = 2;
    return 0;
}
int rift_sensor_esp770u_flash_read(libusb_device_handle *handle, uint32_t address, uint8_t *data, uint16_t size)
{
    CHECK(handle == (void *)2 && address == 0x1d000 && size == 128);
    memset(data, 0, size);
    float focal = 100, center = 64;
    memcpy(data + 0x30, &focal, sizeof focal);
    memcpy(data + 0x34, &center, sizeof center);
    memcpy(data + 0x38, &center, sizeof center);
    return size;
}
int esp570_eeprom_read(libusb_device_handle *handle, uint16_t address, uint8_t *data, uint8_t size)
{ (void)handle; (void)address; (void)data; (void)size; CHECK(false); return -1; }
int rift_sensor_ar0134_init(libusb_device_handle *handle, bool usb2)
{ CHECK(handle == (void *)2 && usb2 == (usb_version < 0x300)); return 0; }
int rift_sensor_esp770u_setup_radio(libusb_device_handle *handle, const uint8_t radio[5])
{ CHECK(handle == (void *)2 && radio != NULL); return 0; }
int mt9v034_setup(libusb_device_handle *handle) { (void)handle; CHECK(false); return -1; }
int mt9v034_set_sync(libusb_device_handle *handle, bool enabled)
{ (void)handle; (void)enabled; CHECK(false); return -1; }

static void captured(rift_sensor_device *device, ohmd_video_frame *frame, void *data)
{ (void)device; (void)frame; (void)data; callbacks++; CHECK(false); }

static void feed(rift_sensor_uvc_stream *stream, int fid, uint32_t pts, bool error)
{
    struct uvc_payload_header header = {
        .bHeaderLength = sizeof header, .bmHeaderInfo = 0x84 | fid | (error ? 0x40 : 0),
        .dwPresentationTime = __cpu_to_le32(pts),
    };
    process_payload(stream, (unsigned char *)&header, sizeof header);
}

static void constructor_quality(bool jpeg)
{
    usb_version = jpeg ? 0x200 : 0x300;
    ohmd_context ctx = {0};
    const uint8_t radio[5] = {0};
    rift_sensor_device *device = rift_sensor_usb_new(&ctx, 0, "CAMERA1", (void *)4, (void *)2, radio);
#if !HAVE_LIBJPEG
    if (jpeg) {
        CHECK(device == NULL && closes == 1 && locks_destroyed == 1);
        return; /* Unsupported JPEG construction must unwind its device state. */
    }
#endif
    CHECK(device != NULL);
    rift_sensor_ctx sensor = {.dev = device};
    snprintf(sensor.serial_no, sizeof sensor.serial_no, "%s", "CAMERA1");
    ohmd_cv1touch_sensor_video_quality quality;
    rift_sensor_get_video_quality(&sensor, &quality);
    CHECK(quality.video_format == (jpeg ? OHMD_CV1TOUCH_VIDEO_JPEG : OHMD_CV1TOUCH_VIDEO_RAW));
    CHECK(quality.frames_dropped == 0 && quality.frames_decoded == 0);
    CHECK(device->calib.width == 128 && device->calib.height == 128);
    CHECK(rift_sensor_device_start_video(device, 1, captured, NULL));
    rift_sensor_uvc_stream *stream = &((rift_sensor_usb_device *)device)->uvc_stream;
    feed(stream, 0, 0, false); /* Observe the initial frame boundary. */
    feed(stream, 1, 1000, true); /* The camera rejects this identified image. */
    for (int i = 0; i < 10; i++) feed(stream, 1, 1000, true);
    rift_sensor_get_video_quality(&sensor, &quality);
    CHECK(!strcmp(quality.serial, "CAMERA1") && quality.reserved == 0);
    CHECK(quality.frames_dropped == 1 && quality.frames_decoded == 0);
    CHECK(device->cv1touch_frames_dropped == 1 && callbacks == 0);
    rift_sensor_device_free(device);
    CHECK(closes == 1 && locks_destroyed == 1);
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (!strcmp(argv[1], "raw")) constructor_quality(false);
    else if (!strcmp(argv[1], "jpeg")) constructor_quality(true);
    else CHECK(false);
    puts("test_sensor_usb_quality ok");
    return 0;
}
