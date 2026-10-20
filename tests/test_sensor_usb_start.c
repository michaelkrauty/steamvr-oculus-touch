// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* Run the real USB start and exported sensor diagnostic getter with USB and
 * camera setup stubbed. No handles, transfers or worker threads are created. */
#include <errno.h>

#define rift_sensor_uvc_stream_start fake_uvc_start
#define frame_captured_cb usb_frame_captured_cb
#include "rift-sensor-usb.c"
#undef frame_captured_cb
#undef rift_sensor_uvc_stream_start
#include "rift-sensor.c"
#include "test_util.h"

static int video_result, setup_result, video_starts, radio_setups;
static ohmd_cv1touch_sensor_stream start_failure;

int fake_uvc_start(rift_sensor_uvc_stream *stream, uint8_t min_frames,
        rift_sensor_uvc_stream_frame_cb cb, void *data)
{
    CHECK(min_frames == 3 && cb != NULL && data != NULL);
    video_starts++;
    if (video_result < 0)
        stream->start_status = start_failure;
    /* Leave an old UVC failure in place on success: the USB wrapper must clear
     * its own published record and use the setting this attempt selected. */
    return video_result;
}

int rift_sensor_ar0134_init(libusb_device_handle *handle, bool usb2_mode)
{
    CHECK(handle == NULL && !usb2_mode);
    return setup_result;
}

int rift_sensor_esp770u_setup_radio(libusb_device_handle *handle, const uint8_t radio_id[5])
{
    CHECK(handle == NULL && radio_id != NULL);
    radio_setups++;
    return 0;
}

int mt9v034_setup(libusb_device_handle *handle) { (void)handle; return 0; }
int mt9v034_set_sync(libusb_device_handle *handle, bool enabled)
{ (void)handle; (void)enabled; return 0; }

static ohmd_cv1touch_sensor_stream exported_record(rift_sensor_usb_device *device)
{
    rift_sensor_ctx sensor = {.dev = &device->base};
    snprintf(sensor.serial_no, sizeof sensor.serial_no, "%s", "CAMERA1");
    ohmd_cv1touch_sensor_stream out;
    rift_sensor_get_stream_info(&sensor, &out);
    CHECK(strcmp(out.serial, "CAMERA1") == 0);
    return out;
}

int main(void)
{
    rift_sensor_usb_device device = {.usb_pid = CV1_PID};
    start_failure = (ohmd_cv1touch_sensor_stream){
        .error_kind = OHMD_CV1TOUCH_STREAM_ERROR_LIKELY_USB_BANDWIDTH,
        .error_code = LIBUSB_ERROR_OTHER, .os_error = ENOSPC,
    };
    video_result = LIBUSB_ERROR_OTHER;
    CHECK(!rift_sensor_usb_start(&device.base, 3, NULL, NULL));
    ohmd_cv1touch_sensor_stream out = exported_record(&device);
    CHECK(out.error_kind == start_failure.error_kind);
    CHECK(out.error_code == LIBUSB_ERROR_OTHER && out.os_error == ENOSPC);
    CHECK(out.alt_setting == 0 && radio_setups == 0);

    video_result = 0;
    device.uvc_stream.alt_setting = 1;
    CHECK(rift_sensor_usb_start(&device.base, 3, NULL, NULL));
    out = exported_record(&device);
    CHECK(out.error_kind == OHMD_CV1TOUCH_STREAM_ERROR_NONE);
    CHECK(out.error_code == 0 && out.os_error == 0 && out.alt_setting == 1);
    CHECK(video_starts == 2 && radio_setups == 1);

    setup_result = LIBUSB_ERROR_ACCESS;
    CHECK(!rift_sensor_usb_start(&device.base, 3, NULL, NULL));
    out = exported_record(&device);
    CHECK(out.error_kind == OHMD_CV1TOUCH_STREAM_ERROR_OTHER);
    CHECK(out.error_code == LIBUSB_ERROR_ACCESS && out.os_error == 0);
    CHECK(out.alt_setting == 0 && radio_setups == 1);
    puts("test_sensor_usb_start ok");
    return 0;
}
