// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* Actual capture delivery, queued-buffer release and video getter, with only
 * clock/tracker/lock operations substituted. No USB handles or threads. */
#include "rift-sensor.c"
#include "ohmd-video.c"
#include "test_util.h"

static rift_sensor_ctx *watched;
static uint64_t expected_received;
static int releases;

uint64_t ohmd_monotonic_get(ohmd_context *ctx) { (void)ctx; return 1000000000; }
void ohmd_lock_mutex(ohmd_mutex *mutex)
{
    CHECK(mutex == (void *)1);
    ohmd_cv1touch_sensor_video video;
    rift_sensor_get_video_info(watched, &video);
    /* Acquiring the analysis lock must not delay counting a delivered frame. */
    CHECK(video.frames_received == expected_received);
    CHECK(watched->fast_analysis_q.head == watched->fast_analysis_q.tail);
    CHECK(watched->capture_frame_q.head != watched->capture_frame_q.tail);
}
void ohmd_unlock_mutex(ohmd_mutex *mutex) { CHECK(mutex == (void *)1); }
void ohmd_cond_signal(ohmd_cond *cond) { (void)cond; }

bool rift_tracker_frame_captured(rift_tracker_ctx *ctx, uint64_t now,
        uint64_t start, rift_tracker_exposure_info *info, const char *source)
{
    (void)ctx; (void)now; (void)start; (void)info;
    CHECK(strcmp(source, "CAMERA1") == 0);
    return false;
}
void rift_tracker_frame_release(rift_tracker_ctx *ctx, uint64_t now,
        uint64_t start, rift_tracker_exposure_info *info, const char *source)
{ (void)ctx; (void)now; (void)start; (void)info; (void)source; }

void blobwatch_release_observation(blobwatch *bw, blobservation *ob)
{ (void)bw; (void)ob; CHECK(false); /* this fixture queues no blob analysis */ }

static void released(ohmd_video_frame *frame, void *owner)
{ (void)frame; (void)owner; releases++; }

int main(void)
{
    CHECK(sizeof(ohmd_cv1touch_sensor_video) == 40);
    ohmd_context ctx = {0};
    rift_sensor_device device = {0};
    device.calib.width = device.calib.height = 128;
    rift_sensor_ctx sensor = {.ohmd_ctx = &ctx, .dev = &device, .sensor_lock = (void *)1};
    snprintf(sensor.serial_no, sizeof sensor.serial_no, "%s", "CAMERA1");
    watched = &sensor;
    rift_sensor_analysis_frame analysis = {0};
    PUSH_QUEUE(&sensor.capture_frame_q, &analysis);
    unsigned char pixels[128 * 128] = {0};
    ohmd_video_frame frame = {
        .format = OHMD_VIDEO_FRAME_FORMAT_GRAY8,
        .width = 128, .height = 128, .stride = 128, .data = pixels,
        .data_size = sizeof pixels, .data_block_size = sizeof pixels,
        .releasefn = released,
    };
    expected_received = 1;
    frame_captured_cb(&device, &frame, &sensor);
    ohmd_cv1touch_sensor_video video;
    rift_sensor_get_video_info(&sensor, &video);
    CHECK(strcmp(video.serial, "CAMERA1") == 0 && video.frames_received == 1);
    CHECK(POP_QUEUE(&sensor.fast_analysis_q) == &analysis);
    /* Finishing old work after capture stops advances only released buffers. */
    release_capture_frame(&sensor, &analysis);
    rift_sensor_get_video_info(&sensor, &video);
    CHECK(video.frames_received == 1 && releases == 1);
    ohmd_cv1touch_sensor sample;
    rift_sensor_get_cv1touch_info(&sensor, &sample);
    CHECK(sample.frames == 1);

    __atomic_store_n(&sensor.cv1touch_frames_received, UINT64_C(4294967296), __ATOMIC_RELAXED);
    expected_received = UINT64_C(4294967297);
    frame_captured_cb(&device, &frame, &sensor);
    rift_sensor_get_video_info(&sensor, &video);
    CHECK(video.frames_received == UINT64_C(4294967297));
    CHECK(POP_QUEUE(&sensor.fast_analysis_q) == &analysis);
    release_capture_frame(&sensor, &analysis);
    rift_sensor_get_video_info(&sensor, &video);
    CHECK(video.frames_received == UINT64_C(4294967297) && releases == 2);
    /* Invalid raw input must be rejected before acquisition progress or any lock. */
    frame.data_size--;
    frame_captured_cb(&device, &frame, &sensor);
    rift_sensor_get_video_info(&sensor, &video);
    CHECK(video.frames_received == UINT64_C(4294967297) && releases == 3);
    CHECK(POP_QUEUE(&sensor.fast_analysis_q) == NULL);
    /* JPEG acquisition still precedes decoding and the analysis lock. */
    frame.format = OHMD_VIDEO_FRAME_FORMAT_JPEG;
    expected_received = UINT64_C(4294967298);
    frame_captured_cb(&device, &frame, &sensor);
    rift_sensor_get_video_info(&sensor, &video);
    CHECK(video.frames_received == expected_received && releases == 3);
    CHECK(POP_QUEUE(&sensor.fast_analysis_q) == &analysis);
    release_capture_frame(&sensor, &analysis);
    CHECK(releases == 4);
    ohmd_cv1touch_sensor_video_quality quality;
    device.cv1touch_video_format = OHMD_CV1TOUCH_VIDEO_JPEG;
    __atomic_store_n(&device.cv1touch_frames_decoded, UINT64_C(4294967296), __ATOMIC_RELAXED);
    __atomic_store_n(&device.cv1touch_frames_dropped, UINT64_C(4294967303), __ATOMIC_RELAXED);
    memset(&quality, 0xff, sizeof quality);
    rift_sensor_get_video_quality(&sensor, &quality);
    CHECK(!strcmp(quality.serial, "CAMERA1") && quality.reserved == 0);
    CHECK(quality.video_format == OHMD_CV1TOUCH_VIDEO_JPEG);
    CHECK(quality.frames_decoded == UINT64_C(4294967296));
    CHECK(quality.frames_dropped == UINT64_C(4294967303));
    rift_sensor_device other_device = {0};
    rift_sensor_ctx other = {.dev = &other_device};
    snprintf(other.serial_no, sizeof other.serial_no, "%s", "CAMERA2");
    rift_sensor_get_video_quality(&other, &quality);
    CHECK(!strcmp(quality.serial, "CAMERA2") && quality.video_format == OHMD_CV1TOUCH_VIDEO_UNKNOWN);
    CHECK(quality.frames_decoded == 0 && quality.frames_dropped == 0);
    device.cv1touch_video_format = OHMD_CV1TOUCH_VIDEO_RAW;
    rift_sensor_get_video_quality(&sensor, &quality);
    CHECK(quality.video_format == OHMD_CV1TOUCH_VIDEO_RAW);
    CHECK(quality.frames_dropped == UINT64_C(4294967303));
    puts("test_sensor_video ok");
    return 0;
}
