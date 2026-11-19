// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
/* Analysis can already own a frame when its USB attachment is retired. Exercise
 * the actual commit boundary without USB devices or pose-fitting timing. A sensor
 * added to a running engine holds its frames back until it is adopted; the actual
 * analysis threads run inline here, one frame at a time. */
#define ohmd_monotonic_get test_now
#define rift_tracked_device_model_pose_update test_pose_update
#define rift_tracked_device_frame_release test_frame_release
#define rift_clear_blob_labels test_clear_labels
#define correspondence_search_set_model test_set_model
#define rift_pose_finder_exp_info_to_dev_state test_exp_info
#define rift_pose_finder_process_blobs_fast test_search_fast
#define rift_pose_finder_process_blobs_long test_search_long
#define rift_pose_finder_set_camera_pose test_set_camera_pose
#define blobwatch_process test_blobwatch_process
#define blobwatch_release_observation test_release_observation
#define rift_tracker_frame_release test_tracker_frame_release
#define rift_tracker_get_search_skip test_search_skip
#define ohmd_video_frame_release test_video_frame_release
#define ohmd_set_thread_name test_thread_name
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
void blobwatch_clear_labels(blobwatch *bw, int id) { (void)bw; (void)id; abort(); }
void rift_pose_finder_set_inspection_candidate(rift_pose_finder *pf, int id,
    const blobservation *ob, uint64_t epoch, const uint64_t *tracks, const posef *world,
    const posef *cam) {
  (void)pf; (void)id; (void)ob; (void)epoch; (void)tracks; (void)world; (void)cam; abort();
}
bool rift_pose_finder_filter_inspection_labels(rift_pose_finder *pf, int id, blobservation *ob,
    const uint64_t *tracks) { (void)pf; (void)id; (void)ob; (void)tracks; abort(); }

/* The analysis threads' surroundings: each call is counted, nothing blocks. */
static unsigned models_set, searches, long_searches, slot_releases, video_releases, callbacks;
static rift_sensor_ctx *inline_sensor; /* the fast thread's frame ends its loop */
static blobservation observed = {.num_blobs = 1};
bool test_set_model(correspondence_search_t *cs, int id, led_search_model_t *model) {
  (void)cs; (void)id; (void)model; models_set++; return true;
}
void test_exp_info(rift_pose_finder *pf, const rift_tracked_device_exposure_info *e,
    rift_sensor_frame_device_state *s) { (void)pf; (void)e; (void)s; }
void test_search_fast(rift_pose_finder *pf, rift_sensor_analysis_frame *f, rift_tracked_device **d,
    uint32_t skip) { (void)pf; (void)f; (void)d; (void)skip; searches++; }
void test_search_long(rift_pose_finder *pf, rift_sensor_analysis_frame *f, rift_tracked_device **d,
    uint32_t skip) { (void)pf; (void)f; (void)d; (void)skip; long_searches++; }
void test_set_camera_pose(rift_pose_finder *pf, const posef *pose) { (void)pf; (void)pose; }
void test_blobwatch_process(blobwatch *bw, uint8_t *frame, int w, int h, uint8_t phase,
    rift_led *leds, uint8_t n, blobservation **out) {
  (void)bw; (void)frame; (void)w; (void)h; (void)phase; (void)leds; (void)n;
  *out = &observed;
  inline_sensor->shutdown = true;
}
void test_release_observation(blobwatch *bw, blobservation *ob) { (void)bw; (void)ob; }
void test_tracker_frame_release(rift_tracker_ctx *t, uint64_t now, uint64_t start,
    rift_tracker_exposure_info *info, const char *source) {
  (void)t; (void)now; (void)start; (void)info; (void)source; slot_releases++;
}
uint32_t test_search_skip(rift_tracker_ctx *t) { (void)t; return 0; }
void test_video_frame_release(ohmd_video_frame *f) { (void)f; video_releases++; }
void test_thread_name(const char *name) { (void)name; }
void ohmd_cond_signal(ohmd_cond *c) { (void)c; }
void ohmd_cond_wait(ohmd_cond *c, ohmd_mutex *m) { (void)c; (void)m; abort(); }
static void count_frame(void *user, uint64_t lifetime, const ohmd_cv1touch_frame_info *info, int n,
    const ohmd_cv1touch_blob *blobs) {
  (void)user; (void)lifetime; (void)info; (void)n; (void)blobs; callbacks++;
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

/* A sensor with a lock, queues, frames and a 4x2 raw image layout; no threads, no USB. */
static ohmd_context analysis_ctx;
static rift_sensor_device analysis_usb;
static rift_sensor_analysis_frame analysis_frames[NUM_CAPTURE_BUFFERS];
static uint8_t pixels[8];
static ohmd_video_frame videos[NUM_CAPTURE_BUFFERS];
static void analysis_sensor(rift_sensor_ctx *sensor) {
  analysis_ctx = (ohmd_context){0};
  analysis_ctx.cv1touch_frame_cb = count_frame;
  analysis_usb = (rift_sensor_device){.calib = {.width = 4, .height = 2}};
  *sensor = (rift_sensor_ctx){.ohmd_ctx = &analysis_ctx, .dev = &analysis_usb,
    .sensor_lock = (void *)&lock, .frames = analysis_frames};
  INIT_QUEUE(&sensor->capture_frame_q);
  INIT_QUEUE(&sensor->fast_analysis_q);
  INIT_QUEUE(&sensor->long_analysis_q);
  for (int i = 0; i < NUM_CAPTURE_BUFFERS; i++) {
    analysis_frames[i] = (rift_sensor_analysis_frame){.id = (uint8_t)i};
    PUSH_QUEUE(&sensor->capture_frame_q, &analysis_frames[i]);
  }
  models_set = searches = long_searches = slot_releases = video_releases = callbacks = 0;
  inline_sensor = sensor;
}
/* A captured frame that started at start_ns, queued as the capture callback queues it. */
static rift_sensor_analysis_frame *captured(rift_sensor_ctx *sensor, rift_sensor_frame_queue *q,
    uint64_t start_ns, uint8_t exposed_devices) {
  rift_sensor_analysis_frame *frame = POP_QUEUE(&sensor->capture_frame_q);
  CHECK(frame);
  ohmd_video_frame *video = &videos[frame->id];
  *video = (ohmd_video_frame){.format = OHMD_VIDEO_FRAME_FORMAT_GRAY8, .data = pixels,
    .width = 4, .height = 2, .stride = 4, .data_size = 8, .data_block_size = 8,
    .start_ts = start_ns};
  frame->vframe = video;
  frame->bwobs = NULL;
  frame->exposure_info_valid = true;
  frame->exposure_info.n_devices = exposed_devices;
  PUSH_QUEUE(q, frame);
  return frame;
}

/* A device the tracker adds after the sensor took a frame: the frame carries its exposure
 * info, the sensor has no pointer for it yet, so the frame's search leaves it out. */
static void a_frame_searches_only_the_devices_the_sensor_has(void) {
  rift_sensor_ctx sensor;
  analysis_sensor(&sensor);
  rift_tracked_device right = {.id = 1};
  sensor.devices[0] = &right;
  sensor.n_devices = 1;
  rift_sensor_analysis_frame *frame = captured(&sensor, &sensor.fast_analysis_q, 10, 2);
  CHECK(init_frame_analyse(&sensor, frame, sensor.n_devices));
  CHECK(frame->n_devices == 1);
  CHECK(init_frame_analyse(&sensor, frame, 3));
  CHECK(frame->n_devices == 2);
}

/* A sensor a running engine adopts is given every device the tracker has, some of which it
 * may have: each is attached once, in tracker order. */
static void a_device_is_attached_once(void) {
  rift_sensor_ctx sensor;
  analysis_sensor(&sensor);
  rift_tracked_device right = {.id = 1}, left = {.id = 2};
  CHECK(rift_sensor_add_device(&sensor, &right));
  CHECK(rift_sensor_add_device(&sensor, &right));
  CHECK(sensor.n_devices == 1 && models_set == 1);
  CHECK(rift_sensor_add_device(&sensor, &left));
  CHECK(rift_sensor_add_device(&sensor, &left));
  CHECK(sensor.n_devices == 2 && models_set == 2);
  CHECK(sensor.devices[0] == &right && sensor.devices[1] == &left);
}

/* The actual fast analysis thread, inline: it takes the one queued frame, and the blob
 * extraction ends its loop. */
static void analyse_one_frame(uint64_t accept_from_ns, uint64_t start_ns) {
  rift_sensor_ctx sensor;
  analysis_sensor(&sensor);
  rift_tracked_device right = {.id = 1};
  sensor.devices[0] = &right;
  sensor.n_devices = 1;
  sensor.accept_from_ns = accept_from_ns;
  captured(&sensor, &sensor.fast_analysis_q, start_ns, 1);
  fast_analysis_thread(&sensor);
  CHECK(slot_releases == 1 && video_releases == 1);
  bool accepted = start_ns >= accept_from_ns;
  CHECK(searches == (accepted ? 1u : 0u));
  CHECK(callbacks == (accepted ? 1u : 0u));
}

/* A sensor added to a running engine: its frames before its start returned were exposed
 * before its radio followed the headset. They are tracked as blobs, but searched for no
 * pose (so they cannot place the sensor either), reach no frame callback and move nothing. */
static void frames_before_acceptance_are_held_back(void) {
  analyse_one_frame(0, 10);              /* a sensor opened with the engine */
  analyse_one_frame(UINT64_MAX, 10);     /* added, not yet adopted */
  analyse_one_frame(100, 99);            /* adopted: a frame from before its start returned */
  analyse_one_frame(100, 100);           /* the first frame after it */

  rift_sensor_ctx sensor;
  analysis_sensor(&sensor);
  rift_tracked_device dev = {.id = 1};
  ohmd_video_frame video = {.start_ts = 99};
  rift_sensor_analysis_frame frame = {.vframe = &video, .bwobs = &observed};
  posef pose = {0};
  rift_pose_metrics score = {0};
  unsigned before = poses;
  sensor.accept_from_ns = 100;
  handle_found_pose(&sensor, &dev, &frame, &pose, &score);
  CHECK(poses == before);
  video.start_ts = 100;
  handle_found_pose(&sensor, &dev, &frame, &pose, &score);
  CHECK(poses == before + 1);
}

/* A frame an analysis thread took just as shutdown began goes back with the queued ones: it
 * holds a delay slot of each device it was captured for. */
static void a_frame_taken_at_shutdown_is_released(void) {
  for (int thread = 0; thread < 2; thread++) {
    rift_sensor_ctx sensor;
    analysis_sensor(&sensor);
    sensor.shutdown = true;
    sensor.retired = true;
    captured(&sensor, thread ? &sensor.long_analysis_q : &sensor.fast_analysis_q, 10, 1);
    if (thread)
      long_analysis_thread(&sensor);
    else
      fast_analysis_thread(&sensor);
    CHECK(slot_releases == 1 && video_releases == 1 && callbacks == 0);
    CHECK(searches == 0 && long_searches == 0);
  }
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
  CHECK(rift_sensor_retired(&sensor));
  check_commit_lock = false;
  a_frame_searches_only_the_devices_the_sensor_has();
  a_device_is_attached_once();
  frames_before_acceptance_are_held_back();
  a_frame_taken_at_shutdown_is_released();
  puts("test_sensor_retirement: ok");
}
