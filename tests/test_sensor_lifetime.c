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
static void *viewer;
#undef ohmd_pw_video_stream_connected
#define ohmd_pw_video_stream_connected(v) ((v) && (void *)(v) == viewer)
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
/* The lifetime an analysis thread could read when it starts. */
static uint64_t thread_lifetime;
static ohmd_thread *test_thread(ohmd_context *ctx, void *arg) {
  (void)ctx;
  thread_lifetime = ((rift_sensor_ctx *)arg)->lifetime;
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
/* A freed sensor gives back what its queued frames hold. */
static unsigned slot_releases, video_releases, callbacks;
void rift_tracker_frame_release(rift_tracker_ctx *t, uint64_t now, uint64_t start,
                                rift_tracker_exposure_info *info, const char *source) {
  (void)t; (void)now; (void)start; (void)source;
  CHECK(info != NULL);
  slot_releases++;
}
void blobwatch_release_observation(blobwatch *bw, blobservation *ob) { (void)bw; (void)ob; }
void ohmd_video_frame_release(ohmd_video_frame *f) { CHECK(f); video_releases++; }
static void count_frame(void *user, uint64_t lifetime, const ohmd_cv1touch_frame_info *info, int n,
                        const ohmd_cv1touch_blob *blobs) {
  (void)user; (void)lifetime; (void)info; (void)n; (void)blobs;
  callbacks++;
}
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
  rift_sensor_ctx *sensor = rift_sensor_new(&ctx, 0, "TEST000001", device, NULL, NULL);
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

/* Each frame left queued for analysis holds a delay slot of every device it was captured for:
 * a sensor freed while the engine runs gives them back, and no frame reaches the callback. */
static void free_releases_frames_queued_for_analysis(void) {
  calls = owned = closed = fail_at = 0;
  slot_releases = video_releases = callbacks = 0;
  ohmd_context ctx = {0};
  ctx.cv1touch_frame_cb = count_frame;
  rift_sensor_device_funcs funcs = {.stop = stop, .free = close_device};
  rift_sensor_device *device = calloc(1, sizeof *device);
  CHECK(device);
  device->funcs = &funcs;
  device->calib.width = 1280;
  device->calib.height = 960;
  rift_sensor_ctx *sensor = rift_sensor_new(&ctx, 0, "TEST000001", device, NULL, NULL);
  CHECK(sensor);
  ohmd_video_frame video[3] = {0};
  blobservation obs = {0};
  for (int i = 0; i < 3; i++) {
    rift_sensor_analysis_frame *frame = POP_QUEUE(&sensor->capture_frame_q);
    CHECK(frame);
    frame->vframe = &video[i];
    frame->bwobs = &obs;
    frame->exposure_info_valid = true;
    PUSH_QUEUE(i < 2 ? &sensor->fast_analysis_q : &sensor->long_analysis_q, frame);
  }
  rift_sensor_free(sensor);
  CHECK(slot_releases == 3 && video_releases == 3 && callbacks == 0);
  CHECK(closed == 1);
}

/* Every native object of a serial has its own lifetime, never zero, numbered upward across
 * objects, and fixed before its analysis threads can read it. */
static void native_lifetimes_are_unique_and_set_before_threads(void) {
  calls = owned = closed = fail_at = 0;
  ohmd_context ctx = {0};
  rift_sensor_device_funcs funcs = {.stop = stop, .free = close_device};
  uint64_t seen[3] = {0};
  for (int i = 0; i < 3; i++) {
    rift_sensor_device *device = calloc(1, sizeof *device);
    CHECK(device);
    device->funcs = &funcs;
    device->calib.width = 1280;
    device->calib.height = 960;
    thread_lifetime = 0;
    rift_sensor_ctx *sensor = rift_sensor_new(&ctx, 0, "TEST000001", device, NULL, NULL);
    CHECK(sensor);
    seen[i] = sensor->lifetime;
    CHECK(seen[i] != 0);
    CHECK(thread_lifetime == seen[i]);
    CHECK(i == 0 || seen[i] > seen[i - 1]);
    rift_sensor_free(sensor);
  }
  CHECK(closed == 3);
}

static void sensor_demand(void)
{
  rift_sensor_ctx sensor = {.sensor_lock = (void *)1};
  CHECK(sensor_images_wanted(&sensor));
  rift_sensor_set_images_wanted(&sensor, false);
  CHECK(!sensor_images_wanted(&sensor));
#define DEMAND(field) do { \
  sensor.field = 1; \
  CHECK(sensor_images_wanted(&sensor)); \
  sensor.field = 0; \
  CHECK(!sensor_images_wanted(&sensor)); \
} while (0)
  DEMAND(have_pending_pose);
  DEMAND(pending_clear);
  DEMAND(pending_candidate);
  DEMAND(pending_label_updates);
  DEMAND(pending_label_revoke);
  DEMAND(pf.inspection_mask);
  DEMAND(long_analysis_busy);
  DEMAND(accept_from_ns);
#undef DEMAND
  /* a sensor viewer on either stream */
  sensor.debug_vid_raw = (void *)&viewer;
  sensor.debug_vid = (void *)&sensor;
  viewer = sensor.debug_vid_raw;
  CHECK(sensor_images_wanted(&sensor));
  viewer = sensor.debug_vid;
  CHECK(sensor_images_wanted(&sensor));
  viewer = NULL;
  CHECK(!sensor_images_wanted(&sensor));
  sensor.debug_vid_raw = sensor.debug_vid = NULL;
  rift_sensor_set_images_wanted(&sensor, true);
  CHECK(sensor_images_wanted(&sensor));
}

int main(void) {
  native_lifetimes_are_unique_and_set_before_threads();
  sensor_demand();
  free_releases_frames_queued_for_analysis();
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
    rift_sensor_ctx *s = rift_sensor_new(&ctx, 64, "TEST000064", d, NULL, NULL);
    if (s)
      rift_sensor_free(s);
    CHECK(closed == 1);
    int direct = (fail > 1 ? 1 : 0) + (fail > 5 ? 1 : 0);
    CHECK(owned == direct);
  }
  puts("test_sensor_lifetime: ok");
  return 0;
}
