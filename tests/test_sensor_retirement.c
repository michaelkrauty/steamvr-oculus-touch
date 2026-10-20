// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
/* Analysis can already own a frame when its USB attachment is retired. Exercise
 * the actual commit boundary without USB devices or pose-fitting timing. */
#define ohmd_monotonic_get test_now
#define rift_tracked_device_model_pose_update test_pose_update
#define rift_tracked_device_frame_release test_frame_release
#define rift_clear_blob_labels test_clear_labels
#include "rift-sensor.c"
#include "test_util.h"
#include <pthread.h>
#include <errno.h>
static unsigned poses, releases;
static bool check_commit_lock;
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
uint64_t test_now(ohmd_context *ctx) { (void)ctx; return 1; }
void ohmd_lock_mutex(ohmd_mutex *m) { CHECK(pthread_mutex_lock((void *)m) == 0); }
void ohmd_unlock_mutex(ohmd_mutex *m) { CHECK(pthread_mutex_unlock((void *)m) == 0); }
bool test_pose_update(rift_tracked_device *d, uint64_t t, uint64_t f,
    rift_tracker_exposure_info *e, rift_pose_metrics *s, posef *p, const char *src) {
  (void)d; (void)t; (void)f; (void)e; (void)s; (void)p; (void)src;
  if (check_commit_lock) CHECK(pthread_mutex_trylock(&lock) == EBUSY);
  poses++;
  return false;
}
void test_frame_release(rift_tracked_device *d, rift_tracker_exposure_info *e) {
  (void)d; (void)e; releases++;
}
void test_clear_labels(struct blob *b, int n, int id) { (void)b; (void)n; (void)id; }
/* No device is inspected here: the inspection paths are never reached. */
uint64_t ohmd_monotonic_per_sec(ohmd_context *ctx) { (void)ctx; abort(); }
uint64_t ohmd_monotonic_conv(uint64_t t, uint64_t src, uint64_t dst) {
  (void)t; (void)src; (void)dst; abort();
}
rift_correspondence_verdict rift_pose_finder_check_inspection_blobs(rift_pose_finder *pf,
    int id, const rift_sensor_analysis_frame *frame) { (void)pf; (void)id; (void)frame; abort(); }
rift_identity_verdict rift_pose_finder_identify_inspection(rift_pose_finder *pf, int id,
    uint64_t t, const quatf *own, const quatf *other, const quatf *camera, bool tracked,
    bool *judgeable) {
  (void)pf; (void)id; (void)t; (void)own; (void)other; (void)camera; (void)tracked;
  (void)judgeable; abort();
}
void rift_pose_finder_set_inspection_pose(rift_pose_finder *pf, int id, const posef *world,
    const posef *cam) { (void)pf; (void)id; (void)world; (void)cam; abort(); }
void blobwatch_update_labels(blobwatch *bw, blobservation *ob, int id) {
  (void)bw; (void)ob; (void)id; abort();
}
static pthread_mutex_t gate = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t condition = PTHREAD_COND_INITIALIZER;
static bool analysing, submit;
struct analysis { rift_sensor_ctx *sensor; rift_tracked_device *dev;
  rift_sensor_analysis_frame *frame; };
static void *analyse(void *data) {
  struct analysis *a = data;
  CHECK(pthread_mutex_lock(&gate) == 0);
  analysing = true;
  CHECK(pthread_cond_signal(&condition) == 0);
  while (!submit) CHECK(pthread_cond_wait(&condition, &gate) == 0);
  CHECK(pthread_mutex_unlock(&gate) == 0);
  posef pose = {0};
  rift_pose_metrics score = {0};
  handle_found_pose(a->sensor, a->dev, a->frame, &pose, &score);
  return NULL;
}
int main(void) {
  rift_sensor_device usb = {0};
  rift_sensor_ctx sensor = {.dev = &usb, .sensor_lock = (void *)&lock};
  ohmd_cv1touch_sensor_attachment discovered = {.kind = OHMD_CV1TOUCH_ATTACHMENT_USB,
    .data = {1 | (7 << 8) | (2 << 16), 2 | (3 << 8), 0}};
  rift_sensor_set_attachment(&sensor, &discovered, -1);
  /* Stream start replaces its diagnostics, but cannot replace discovery identity. */
  usb.stream_status = (ohmd_cv1touch_sensor_stream){.error_kind = OHMD_CV1TOUCH_STREAM_ERROR_OTHER,
    .error_code = LIBUSB_ERROR_OTHER};
  ohmd_cv1touch_sensor_stream attempt;
  rift_sensor_get_stream_info(&sensor, &attempt);
  CHECK(attempt.error_code == LIBUSB_ERROR_OTHER);
  CHECK(memcmp(&attempt.attachment, &discovered, sizeof discovered) == 0);
  rift_tracked_device dev = {.id = 1};
  ohmd_video_frame video = {0};
  blobservation obs = {0};
  rift_sensor_analysis_frame frame = {.vframe = &video, .bwobs = &obs};
  posef pose = {0};
  rift_pose_metrics score = {0};
  check_commit_lock = true;
  handle_found_pose(&sensor, &dev, &frame, &pose, &score);
  CHECK(poses == 1 && releases == 1);
  /* A completed USB transfer reported removal while analysis was running. */
  __atomic_store_n(&usb.usb_gone, 1, __ATOMIC_RELAXED);
  handle_found_pose(&sensor, &dev, &frame, &pose, &score);
  CHECK(poses == 1 && releases == 2);
  __atomic_store_n(&usb.usb_gone, 0, __ATOMIC_RELAXED);
  struct analysis a = {&sensor, &dev, &frame};
  pthread_t worker;
  CHECK(pthread_create(&worker, NULL, analyse, &a) == 0);
  CHECK(pthread_mutex_lock(&gate) == 0);
  while (!analysing) CHECK(pthread_cond_wait(&condition, &gate) == 0);
  CHECK(pthread_mutex_unlock(&gate) == 0);
  rift_sensor_retire(&sensor);
  CHECK(pthread_mutex_lock(&gate) == 0);
  submit = true;
  CHECK(pthread_cond_signal(&condition) == 0);
  CHECK(pthread_mutex_unlock(&gate) == 0);
  CHECK(pthread_join(worker, NULL) == 0);
  CHECK(poses == 1 && releases == 3);
  puts("test_sensor_retirement: ok");
}
