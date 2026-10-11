// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
/* Exercise the real headset open/cleanup path when tracker construction fails.
 * All HID operations and tracker creation are synthetic; no devices are opened. */
#define rift_tracker_new fake_tracker_new
#define rift_hmd_radio_get_address fake_radio_address
#include "rift.c"
#include "test_util.h"
#include "omath.c"
#include "packet.c"
#include "openhmd.c"

static int handles, tracker_calls, radio_clears, calibration_clears;
double ohmd_get_tick(void) { return 0; }
void rift_hmd_radio_init(rift_hmd_radio_state *radio, hid_device *handle) { radio->handle = handle; }
void rift_hmd_radio_clear(rift_hmd_radio_state *radio) { CHECK(radio->handle); radio_clears++; }
void rift_touch_clear_calibration(rift_touch_calibration *calibration) { (void)calibration; calibration_clears++; }
void rift_tracker_free(rift_tracker_ctx *tracker) { (void)tracker; CHECK(false); }
rift_tracked_device *rift_tracker_add_device(rift_tracker_ctx *tracker, int id,
    posef *imu, posef *model, rift_leds *leds, rift_tracked_device_imu_calibration *calibration) {
    (void)tracker; (void)id; (void)imu; (void)model; (void)leds; (void)calibration;
    CHECK(false); return NULL;
}
static struct hid_device_info radio_info = {
    .path = "TEST_RADIO", .interface_number = 1,
};

hid_device *hid_open_path(const char *path) {
    CHECK(!strcmp(path, "TEST_HMD") || !strcmp(path, "TEST_RADIO"));
    handles++;
    return (hid_device *)malloc(1);
}
void hid_close(hid_device *handle) {
    CHECK(handle && handles > 0);
    handles--;
    free(handle);
}
struct hid_device_info *hid_enumerate(unsigned short vendor, unsigned short product) {
    CHECK(vendor == OCULUS_VR_INC_ID && product == RIFT_CV1_PID);
    return &radio_info;
}
void hid_free_enumeration(struct hid_device_info *info) { CHECK(info == &radio_info); }
int hid_set_nonblocking(hid_device *handle, int nonblock) {
    CHECK(handle && nonblock == 1);
    return 0;
}
int hid_send_feature_report(hid_device *handle, const unsigned char *data, size_t length) {
    CHECK(handle && data);
    return (int)length;
}
int hid_get_feature_report(hid_device *handle, unsigned char *data, size_t length) {
    CHECK(handle && length == FEATURE_BUFFER_SIZE);
    switch (data[0]) {
    case RIFT_CMD_RANGE: return 8;
    case RIFT_CMD_IMU_CALIBRATION: return 69;
    case RIFT_CMD_DISPLAY_INFO: return 56;
    case RIFT_CMD_SENSOR_CONFIG: return 7;
    /* An empty synthetic LED model is enough to reach tracker creation. */
    case RIFT_CMD_POSITION_INFO:
    case RIFT_CMD_PATTERN_INFO: return -1;
    default: CHECK(false); return -1;
    }
}
bool fake_radio_address(hid_device *handle, uint8_t address[5]) {
    CHECK(handle);
    memset(address, 0, 5);
    return true;
}
rift_tracker_ctx *fake_tracker_new(ohmd_context *ctx, const uint8_t address[5]) {
    CHECK(ctx && address && handles == 2);
    tracker_calls++;
    return NULL;
}

int main(void) {
    ohmd_context ctx = {0};
    ohmd_driver driver = {.ctx = &ctx};
    ohmd_device_desc desc = {.revision = REV_CV1};
    snprintf(desc.path, sizeof desc.path, "TEST_HMD");
    for (int tracking = 0; tracking < 2; tracking++) {
        ctx.cv1touch_config.hmd_tracking = tracking;
        CHECK(open_hmd(&driver, &desc) == NULL);
        CHECK(handles == 0);
        CHECK(strstr(ctx.error_msg, "tracker") != NULL);
    }
    CHECK(tracker_calls == 2);
    CHECK(radio_clears == 2 && calibration_clears == 4);
    puts("test_hmd_tracker_failure: ok");
    return 0;
}
