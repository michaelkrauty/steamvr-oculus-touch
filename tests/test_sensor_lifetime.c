// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
/* Exercise actual analyser construction/destruction with failure at every owned
 * allocation. */
#include "ohmd-pipewire.h"
#include "rift-sensor-pose-search.h"
#include "rift-sensor.h"
#include <stddef.h>
static ohmd_thread *test_thread(ohmd_context *, void *);
static void test_pose_init(rift_pose_finder *, rift_sensor_camera_params *);
#define ohmd_create_thread(ctx, routine, arg) test_thread(ctx, arg)
#define rift_pose_finder_init(pf, calib, cb, data) test_pose_init(pf, calib)
#include "omath.c"
#include "rift-sensor.c"
#include "test_util.h"
static int calls, fail_at, owned, closed;
static void *allocate(size_t n) {
  if (++calls == fail_at)
    return NULL;
  void *p = calloc(1, n);
  CHECK(p);
  owned++;
  return p;
}
static void release(void *p) {
  if (p) {
    owned--;
    free(p);
  }
}
void *ohmd_allocfn(ohmd_context *ctx, const char *message, size_t n) {
  (void)ctx;
  (void)message;
  return allocate(n);
}
ohmd_mutex *ohmd_create_mutex(ohmd_context *ctx) {
  (void)ctx;
  return allocate(1);
}
void ohmd_destroy_mutex(ohmd_mutex *p) { release(p); }
void ohmd_lock_mutex(ohmd_mutex *p) { CHECK(p); }
void ohmd_unlock_mutex(ohmd_mutex *p) { CHECK(p); }
ohmd_cond *ohmd_create_cond(ohmd_context *ctx) {
  (void)ctx;
  return allocate(1);
}
void ohmd_destroy_cond(ohmd_cond *p) { release(p); }
void ohmd_cond_broadcast(ohmd_cond *p) { CHECK(p); }
static ohmd_thread *test_thread(ohmd_context *ctx, void *arg) {
  (void)ctx;
  (void)arg;
  return allocate(1);
}
void ohmd_destroy_thread(ohmd_thread *p) { release(p); }
blobwatch *blobwatch_new(uint8_t threshold) {
  (void)threshold;
  return allocate(1);
}
void blobwatch_free(blobwatch *p) { release(p); }
static void test_pose_init(rift_pose_finder *pf,
                           rift_sensor_camera_params *calib) {
  (void)calib;
  pf->cs = allocate(1);
}
void rift_pose_finder_clear(rift_pose_finder *pf) { release(pf->cs); }
void rift_pose_finder_set_camera_pose(rift_pose_finder *pf, const posef *pose) {
  (void)pf;
  (void)pose;
}
#ifdef HAVE_PIPEWIRE
ohmd_pw_video_stream *ohmd_pw_video_stream_new(const char *a, const char *b,
                                               ohmd_pw_video_format f,
                                               uint16_t w, uint16_t h,
                                               uint16_t n, uint16_t d) {
  (void)a;
  (void)b;
  (void)f;
  (void)w;
  (void)h;
  (void)n;
  (void)d;
  return NULL;
}
void ohmd_pw_video_stream_free(ohmd_pw_video_stream *p) { CHECK(!p); }
#endif
#if HAVE_LIBJPEG
void ohmd_jpeg_decoder_free(ohmd_jpeg_decoder *p) { CHECK(!p); }
#endif
static void stop(rift_sensor_device *d) { CHECK(d); }
static void close_device(rift_sensor_device *d) {
  closed++;
  free(d);
}
int main(void) {
  ohmd_context ctx = {0};
  rift_sensor_device_funcs funcs = {.stop = stop, .free = close_device};
  /* Buffers and outer analyser are freed directly, so subtract their successful
   * allocations after destruction; all lock/cond/thread/search allocations use
   * counted destructors. */
  for (int fail = 1; fail <= 11; fail++) {
    calls = owned = closed = 0;
    fail_at = fail;
    rift_sensor_device *d = calloc(1, sizeof *d);
    d->funcs = &funcs;
    d->calib.width = 1280;
    d->calib.height = 960;
    rift_sensor_ctx *s = rift_sensor_new(&ctx, 64, "TEST000064", d, NULL);
    if (s)
      rift_sensor_free(s);
    CHECK(closed == 1);
    int direct = (fail > 1 ? 1 : 0) + (fail > 5 ? 1 : 0);
    CHECK(owned == direct);
  }
  puts("test_sensor_lifetime: ok");
  return 0;
}
