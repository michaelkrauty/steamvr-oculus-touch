// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
/* Exercise actual analyser construction/destruction with failure at every owned
 * allocation. */
#define _GNU_SOURCE
#include <fcntl.h>
#include <errno.h>
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
/* No worker runs in this allocation-failure harness. */
uint64_t ohmd_monotonic_get(ohmd_context *ctx) { (void)ctx; return 0; }
uint64_t ohmd_monotonic_per_sec(ohmd_context *ctx) { (void)ctx; return 1000000000; }
uint64_t ohmd_monotonic_conv(uint64_t t, uint64_t src, uint64_t dst) {
  return t / src * dst + t % src * dst / src;
}
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
void ohmd_cond_signal(ohmd_cond *p) { CHECK(p); }
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
  pf->cs = allocate(sizeof *pf->cs);
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
static void serial_case_matches(void) {
  rift_sensor_ctx sensor = {.sensor_lock = (void *)1};
  strcpy(sensor.serial_no, "CAMERA1");
  ohmd_cv1touch_sensor_pose pose = {
      .serial = "cAmErA1", .orient = {0, 0, 0, 1}, .pos = {1, 2, 3}};
  CHECK(rift_sensor_set_cv1touch_pose(&sensor, &pose));
  CHECK(sensor.have_pending_pose && !sensor.pending_clear);
  CHECK(sensor.pending_pose.pos.x == 1);
  CHECK(rift_sensor_cv1touch_pose_applied(&sensor, "camera1") == 0);
  CHECK(rift_sensor_cv1touch_pose_applied(&sensor, "camera10") == -1);
  CHECK(!rift_sensor_clear_cv1touch_pose(&sensor, "camera10", 0, 0));
  CHECK(rift_sensor_clear_cv1touch_pose(&sensor, "camera1", 0, 0));
  CHECK(sensor.pending_clear && !sensor.have_pending_pose);
  sensor.pf.cv1touch_allow_placement = true;
  CHECK(!rift_sensor_clear_cv1touch_pose(&sensor, "camera1", 0, 0));
}

static void gone_diagnostic_survives_publication(void) {
  rift_sensor_device dev = {0};
  rift_sensor_ctx sensor = {0};
  sensor.dev = &dev;
  sensor.sensor_lock = (void *)1;
  ohmd_cv1touch_sensor out;
  CHECK(sizeof(out) == 120 && offsetof(ohmd_cv1touch_sensor, camera_orient) == 36);
  rift_sensor_get_cv1touch_info(&sensor, &out);
  CHECK(!out.usb_gone);
  __atomic_store_n(&dev.usb_gone, 1, __ATOMIC_RELAXED);
  for (int i = 0; i < 2; i++) {
    rift_sensor_get_cv1touch_info(&sensor, &out);
    CHECK(out.usb_gone);
  }
}

/* The actual sensor owns a pending pin until the retained start record takes it.
 * No device nodes: a metadata directory exercises real close-on-free ownership. */
static void pin_ownership_survives_sensor_free(bool transfer) {
  calls = owned = closed = fail_at = 0;
  ohmd_context ctx = {0};
  rift_sensor_device_funcs funcs = {.stop = stop, .free = close_device};
  rift_sensor_device *device = calloc(1, sizeof *device);
  CHECK(device);
  device->funcs = &funcs;
  device->calib.width = 1280;
  device->calib.height = 960;
  rift_sensor_ctx *sensor = rift_sensor_new(&ctx, 0, "TEST000001", device, NULL);
  CHECK(sensor && sensor->attachment_pin == -1);
  int pin = open(".", O_PATH | O_DIRECTORY | O_CLOEXEC);
  CHECK(pin >= 0);
  ohmd_cv1touch_sensor_attachment key = {.kind = OHMD_CV1TOUCH_ATTACHMENT_SYSFS};
  rift_sensor_set_attachment(sensor, &key, pin);
  if (transfer) {
    CHECK(rift_sensor_take_attachment_pin(sensor) == pin);
    CHECK(sensor->attachment_pin == -1);
  }
  rift_sensor_free(sensor);
  CHECK(closed == 1 && owned == 2); /* directly freed sensor and frame array */
  if (transfer) {
    int flags = fcntl(pin, F_GETFD);
    CHECK(flags >= 0 && (flags & FD_CLOEXEC));
    rift_sensor_close_attachment_pin(pin);
  }
  CHECK(fcntl(pin, F_GETFD) == -1 && errno == EBADF);
}

int main(void) {
  pin_ownership_survives_sensor_free(false);
  pin_ownership_survives_sensor_free(true);
  serial_case_matches();
  gone_diagnostic_survives_publication();
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
