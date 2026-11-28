// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
/* The live sensor lifecycle of the actual tracker: adds, releases and its hot-plug worker,
 * which runs on a real thread here. USB, the sensor objects and the filters are substituted;
 * the sysfs pins are real directories. Every fake that stands for a USB start, stop or retry
 * checks the thread calling it holds no roster, tracker or hot-plug lock. */
#define _GNU_SOURCE
#include <stdlib.h>
#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <sys/stat.h>
#include <dirent.h>
#include <time.h>
static char sysfs_root[PATH_MAX];
#define CV1TOUCH_SYSFS_USB_ROOT sysfs_root
#define libusb_init fake_usb_init
#define libusb_exit fake_usb_exit
#define libusb_get_device_list fake_device_list
#define libusb_free_device_list fake_free_list
#define libusb_get_device_descriptor fake_descriptor
#define libusb_get_bus_number fake_bus
#define libusb_get_device_address fake_address
#define libusb_get_port_numbers fake_ports
#define libusb_open fake_open
#define libusb_close fake_close
#define libusb_get_string_descriptor_ascii fake_serial
#define ohmd_create_thread fake_thread
#define ohmd_destroy_thread fake_join
#define ohmd_set_thread_name fake_thread_name
#define ohmd_monotonic_get fake_now
#define ohmd_monotonic_per_sec fake_per_sec
#define ohmd_monotonic_conv fake_conv
#define rift_sensor_usb_new fake_sensor_device
#define rift_sensor_new fake_sensor_new
#define rift_sensor_start fake_sensor_start
#define rift_sensor_free fake_sensor_free
#define rift_sensor_request_stop fake_sensor_request_stop
#define rift_sensor_retry fake_sensor_retry
#define rift_sensor_get_stream_info fake_sensor_stream_info
#define rift_sensor_set_attachment fake_sensor_set_attachment
#define rift_sensor_take_attachment_pin fake_sensor_take_attachment_pin
#define rift_sensor_serial_no fake_sensor_serial
#define rift_sensor_retire fake_sensor_retire
#define rift_sensor_retired fake_sensor_retired
#define rift_sensor_lifetime fake_sensor_lifetime
#define rift_sensor_accept_frames_from fake_sensor_accept
#define rift_sensor_get_usb_device fake_sensor_usb_device
#define rift_sensor_add_device fake_sensor_add_device
#define rift_sensor_pause_for_model_swap fake_sensor_pause_for_model_swap
#define rift_sensor_update_device_model_parked fake_sensor_update_device_model_parked
#define rift_sensor_resume_after_model_swap fake_sensor_resume_after_model_swap
#define rift_kalman_6dof_init fake_kalman_init
#include "omath.c"
#include "rift-tracker.c"
#include "test_util.h"
void fake_sensor_pause_for_model_swap(rift_sensor_ctx *s, int id) { (void)s; (void)id; }
void fake_sensor_update_device_model_parked(rift_sensor_ctx *s, rift_tracked_device *d) { (void)s; (void)d; }
void fake_sensor_resume_after_model_swap(rift_sensor_ctx *s) { (void)s; }

/* ohmd platform locks and threads on pthreads, with each lock's owner kept for the checks */
struct ohmd_mutex {
  pthread_mutex_t mutex;
  pthread_t owner;
  bool held;
};
struct ohmd_cond {
  pthread_cond_t cond;
};
struct ohmd_thread {
  pthread_t thread;
  unsigned int (*routine)(void *);
  void *arg;
  bool running;
};
ohmd_mutex *ohmd_create_mutex(ohmd_context *ctx) {
  (void)ctx;
  ohmd_mutex *m = calloc(1, sizeof *m);
  CHECK(m && pthread_mutex_init(&m->mutex, NULL) == 0);
  return m;
}
void ohmd_destroy_mutex(ohmd_mutex *m) {
  CHECK(!__atomic_load_n(&m->held, __ATOMIC_ACQUIRE) && pthread_mutex_destroy(&m->mutex) == 0);
  free(m);
}
/* Other threads read owner/held to tell whether they hold the lock themselves: atomics. */
static void owned_by_me(ohmd_mutex *m) {
  __atomic_store_n(&m->owner, pthread_self(), __ATOMIC_RELAXED);
  __atomic_store_n(&m->held, true, __ATOMIC_RELEASE);
}
static bool holds(ohmd_mutex *m) {
  return m && __atomic_load_n(&m->held, __ATOMIC_ACQUIRE) &&
         pthread_equal(__atomic_load_n(&m->owner, __ATOMIC_RELAXED), pthread_self());
}
void ohmd_lock_mutex(ohmd_mutex *m) {
  CHECK(pthread_mutex_lock(&m->mutex) == 0);
  owned_by_me(m);
}
void ohmd_unlock_mutex(ohmd_mutex *m) {
  CHECK(holds(m));
  __atomic_store_n(&m->held, false, __ATOMIC_RELEASE);
  CHECK(pthread_mutex_unlock(&m->mutex) == 0);
}
ohmd_cond *ohmd_create_cond(ohmd_context *ctx) {
  (void)ctx;
  ohmd_cond *c = calloc(1, sizeof *c);
  CHECK(c && pthread_cond_init(&c->cond, NULL) == 0);
  return c;
}
void ohmd_destroy_cond(ohmd_cond *c) {
  CHECK(pthread_cond_destroy(&c->cond) == 0);
  free(c);
}
void ohmd_cond_wait(ohmd_cond *c, ohmd_mutex *m) {
  CHECK(holds(m));
  __atomic_store_n(&m->held, false, __ATOMIC_RELEASE);
  CHECK(pthread_cond_wait(&c->cond, &m->mutex) == 0);
  owned_by_me(m);
}
void ohmd_cond_signal(ohmd_cond *c) { CHECK(pthread_cond_signal(&c->cond) == 0); }
void ohmd_cond_broadcast(ohmd_cond *c) { CHECK(pthread_cond_broadcast(&c->cond) == 0); }
static void *run_thread(void *arg) {
  ohmd_thread *t = arg;
  t->routine(t->arg);
  return NULL;
}
static int workers_started;
ohmd_thread *fake_thread(ohmd_context *ctx, unsigned int (*routine)(void *), void *arg) {
  (void)ctx;
  ohmd_thread *t = calloc(1, sizeof *t);
  CHECK(t);
  t->routine = routine;
  t->arg = arg;
  /* The USB event thread would need a real libusb context: it is not run. */
  if (routine == sensor_hotplug_thread) {
    t->running = true;
    workers_started++;
    CHECK(pthread_create(&t->thread, NULL, run_thread, t) == 0);
  }
  return t;
}
void fake_join(ohmd_thread *t) {
  if (t->running)
    CHECK(pthread_join(t->thread, NULL) == 0);
  free(t);
}
static pthread_t worker_thread;
static bool worker_known;
void fake_thread_name(const char *name) {
  CHECK(strcmp(name, "cv1t-sensors") == 0);
  worker_thread = pthread_self();
  __atomic_store_n(&worker_known, true, __ATOMIC_RELEASE);
}
static bool on_worker(void) {
  return __atomic_load_n(&worker_known, __ATOMIC_ACQUIRE) && pthread_equal(worker_thread, pthread_self());
}
uint64_t fake_now(ohmd_context *ctx) {
  (void)ctx;
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (uint64_t)now.tv_sec * 1000000000u + (uint64_t)now.tv_nsec;
}
uint64_t fake_per_sec(ohmd_context *ctx) { (void)ctx; return 1000000000u; }
uint64_t fake_conv(uint64_t ticks, uint64_t from, uint64_t to) { return ticks / (from / to); }

/* Fake state is shared by the test (the engine thread), the worker and a retirement thread. */
static pthread_mutex_t fake = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t fake_changed = PTHREAD_COND_INITIALIZER;
static rift_tracker_ctx *tracker;
#define MAX_FAKES 64
static int plugged[MAX_FAKES], n_plugged;          /* device ids in the USB list */
static char serial_at[MAX_FAKES][32];              /* the serial each device reports */
static int opens[MAX_FAKES], open_error[MAX_FAKES], handles, sequence;
static int last_open_seq[MAX_FAKES];

/* The engine thread pauses inside adoption, before the roster join, while the worker is in
 * the Windows search. The worker records whether that search opened the device being adopted. */
static pthread_mutex_t gate = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t gate_cond = PTHREAD_COND_INITIALIZER;
static int worker_lists, stall_list, adopt_device, opens_at_gap;
static bool release_list, stalled_in_list, stall_adopt, in_adopt_gap, adopt_gap_seen, gap_opened_held;

/* Caller holds `gate`. */
static bool wait_gate(bool *flag) {
  struct timespec until;
  clock_gettime(CLOCK_REALTIME, &until);
  until.tv_sec += 5;
  while (!*flag) {
    int rc = pthread_cond_timedwait(&gate_cond, &gate, &until);
    if (rc == ETIMEDOUT)
      return false;
    if (rc == EINTR)
      continue;
    CHECK(rc == 0);
  }
  return true;
}

static void no_engine_locks(rift_tracker_ctx *t) {
  CHECK(!holds(t->roster_lock));
  CHECK(!holds(t->tracker_lock));
  CHECK(!holds(t->hotplug.lock));
}

int fake_usb_init(libusb_context **ctx) { *ctx = (void *)1; return 0; }
void fake_usb_exit(libusb_context *ctx) { (void)ctx; }
static libusb_device *listed[MAX_FAKES + 1];
ssize_t fake_device_list(libusb_context *ctx, libusb_device ***out) {
  (void)ctx;
  CHECK(on_worker() || tracker == NULL);
  if (on_worker() && stall_list) {
    pthread_mutex_lock(&gate);
    int n = ++worker_lists;
    if (n == stall_list) {
      stalled_in_list = true;
      pthread_cond_broadcast(&gate_cond);
      CHECK(wait_gate(&release_list));
    }
    pthread_mutex_unlock(&gate);
  }
  pthread_mutex_lock(&fake);
  for (int i = 0; i < n_plugged; i++)
    listed[i] = (void *)(uintptr_t)plugged[i];
  listed[n_plugged] = NULL;
  int n = n_plugged;
  pthread_mutex_unlock(&fake);
  *out = listed;
  return n;
}
void fake_free_list(libusb_device **list, int unref) { (void)list; (void)unref; }
int fake_descriptor(libusb_device *dev, struct libusb_device_descriptor *desc) {
  (void)dev;
  memset(desc, 0, sizeof *desc);
  desc->idVendor = 0x2833;
  desc->idProduct = CV1_PID;
  desc->iSerialNumber = 1;
  return 0;
}
uint8_t fake_bus(libusb_device *dev) { (void)dev; return 1; }
uint8_t fake_address(libusb_device *dev) { return (uint8_t)(uintptr_t)dev; }
int fake_ports(libusb_device *dev, uint8_t *ports, int size) {
  CHECK(size >= 2); ports[0] = 2; ports[1] = (uint8_t)(uintptr_t)dev; return 2;
}
int fake_open(libusb_device *dev, libusb_device_handle **handle) {
  int id = (int)(uintptr_t)dev;
  pthread_mutex_lock(&fake);
  opens[id]++;
  last_open_seq[id] = ++sequence;
  int error = open_error[id];
  if (!error)
    handles++;
  pthread_mutex_unlock(&fake);
  if (error)
    return error;
  *handle = (void *)dev;
  return 0;
}
void fake_close(libusb_device_handle *handle) {
  (void)handle;
  pthread_mutex_lock(&fake);
  handles--;
  CHECK(handles >= 0);
  pthread_mutex_unlock(&fake);
}
int fake_serial(libusb_device_handle *handle, uint8_t index, unsigned char *data, int len) {
  (void)index;
  int id = (int)(uintptr_t)handle;
  pthread_mutex_lock(&fake);
  int n = snprintf((char *)data, len, "%s", serial_at[id]);
  pthread_mutex_unlock(&fake);
  return n;
}

struct fake_sensor {
  rift_tracker_ctx *tracker;
  char serial[32];
  int device; /* also its handle */
  ohmd_cv1touch_sensor_attachment attachment;
  int pin_fd;
  bool stopping, retired, freed, retired_when_freed;
  uint64_t accept_from;
  rift_tracked_device *devices[8];
  int n_devices;
  int freed_seq;
  uint64_t lifetime;
  bool have_pose;
  ohmd_cv1touch_sensor_pose pose;
};
static struct fake_sensor *made[256];
static int n_made, live;
static int fail_setup, setup_error, fail_start_device;
rift_sensor_device *fake_sensor_device(ohmd_context *ctx, int id, const char *serial,
    libusb_context *usb, libusb_device_handle *handle, const uint8_t radio[5], int *error_code) {
  (void)ctx; (void)id; (void)serial; (void)usb;
  CHECK(memcmp(radio, "\1\2\3\4\5", 5) == 0);
  if ((int)(uintptr_t)handle == fail_setup) {
    fake_close(handle);
    *error_code = setup_error;
    return NULL;
  }
  return (void *)handle;
}
rift_sensor_ctx *fake_sensor_new(ohmd_context *ctx, int id, const char *serial,
    rift_sensor_device *device, rift_tracker_ctx *t, const ohmd_cv1touch_sensor_pose *pose) {
  (void)ctx; (void)id;
  struct fake_sensor *s = calloc(1, sizeof *s);
  CHECK(s);
  s->tracker = t;
  snprintf(s->serial, sizeof s->serial, "%s", serial);
  s->device = (int)(uintptr_t)device;
  s->pin_fd = -1;
  if (pose) {
    s->have_pose = true;
    s->pose = *pose;
  }
  pthread_mutex_lock(&fake);
  CHECK(n_made < 256);
  made[n_made++] = s;
  s->lifetime = (uint64_t)n_made;
  live++;
  pthread_mutex_unlock(&fake);
  if (on_worker()) {
    pthread_mutex_lock(&gate);
    if (in_adopt_gap && !adopt_gap_seen) {
      pthread_mutex_lock(&fake);
      gap_opened_held = opens[adopt_device] != opens_at_gap;
      pthread_mutex_unlock(&fake);
      adopt_gap_seen = true;
      pthread_cond_broadcast(&gate_cond);
    }
    pthread_mutex_unlock(&gate);
  }
  return (void *)s;
}

/* A start can be held until the test lets it go (or the engine closes). */
static bool hold_starts;
static int starts_entered;
bool fake_sensor_start(rift_sensor_ctx *sensor) {
  struct fake_sensor *s = (void *)sensor;
  no_engine_locks(s->tracker);
  /* a sensor added later is started with its frames held back; one opened with the engine not */
  CHECK(on_worker() ? s->accept_from == UINT64_MAX : s->accept_from == 0);
  pthread_mutex_lock(&fake);
  starts_entered++;
  pthread_cond_broadcast(&fake_changed);
  while (hold_starts && !__atomic_load_n(&s->tracker->hotplug.quit, __ATOMIC_ACQUIRE)) {
    struct timespec until;
    clock_gettime(CLOCK_REALTIME, &until);
    until.tv_nsec += 10000000;
    if (until.tv_nsec >= 1000000000) { until.tv_sec++; until.tv_nsec -= 1000000000; }
    pthread_cond_timedwait(&fake_changed, &fake, &until);
  }
  pthread_mutex_unlock(&fake);
  return s->device != fail_start_device;
}
void fake_sensor_stream_info(rift_sensor_ctx *sensor, ohmd_cv1touch_sensor_stream *out) {
  struct fake_sensor *s = (void *)sensor;
  memset(out, 0, sizeof *out);
  snprintf(out->serial, sizeof out->serial, "%s", s->serial);
  out->attachment = s->attachment;
  if (s->device == fail_start_device) {
    out->error_kind = OHMD_CV1TOUCH_STREAM_ERROR_LIKELY_USB_BANDWIDTH;
    out->error_code = LIBUSB_ERROR_IO;
    out->os_error = ENOSPC;
  } else out->alt_setting = 2;
}
void fake_sensor_set_attachment(rift_sensor_ctx *sensor, const ohmd_cv1touch_sensor_attachment *a, int pin) {
  struct fake_sensor *s = (void *)sensor;
  s->attachment = *a;
  s->pin_fd = pin;
}
int fake_sensor_take_attachment_pin(rift_sensor_ctx *sensor) {
  struct fake_sensor *s = (void *)sensor;
  int pin = s->pin_fd;
  s->pin_fd = -1;
  return pin;
}
/* A retirement thread can be stopped inside the roster walk, holding a sensor's pointer. */
static __thread bool retiring;
static struct fake_sensor *pause_at;
static bool paused;
const char *fake_sensor_serial(rift_sensor_ctx *sensor) {
  struct fake_sensor *s = (void *)sensor;
  if (retiring && s == __atomic_load_n(&pause_at, __ATOMIC_ACQUIRE)) {
    pthread_mutex_lock(&fake);
    paused = true;
    pthread_cond_broadcast(&fake_changed);
    /* wait to be freed under us; a held roster lock makes this time out */
    struct timespec until;
    clock_gettime(CLOCK_REALTIME, &until);
    until.tv_nsec += 300000000;
    if (until.tv_nsec >= 1000000000) { until.tv_sec++; until.tv_nsec -= 1000000000; }
    while (!s->freed && pthread_cond_timedwait(&fake_changed, &fake, &until) == 0) {}
    pthread_mutex_unlock(&fake);
  }
  return s->serial;
}
void fake_sensor_retire(rift_sensor_ctx *sensor) {
  struct fake_sensor *s = (void *)sensor;
  pthread_mutex_lock(&fake);
  CHECK(!s->freed);
  s->retired = true;
  pthread_mutex_unlock(&fake);
}
uint64_t fake_sensor_lifetime(rift_sensor_ctx *sensor) {
  return ((struct fake_sensor *)(void *)sensor)->lifetime;
}
bool fake_sensor_retired(rift_sensor_ctx *sensor) {
  struct fake_sensor *s = (void *)sensor;
  pthread_mutex_lock(&fake);
  bool retired = s->retired;
  pthread_mutex_unlock(&fake);
  return retired;
}
void fake_sensor_accept(rift_sensor_ctx *sensor, uint64_t ns) {
  ((struct fake_sensor *)sensor)->accept_from = ns;
}
const struct libusb_device *fake_sensor_usb_device(rift_sensor_ctx *sensor) {
  return (const void *)(uintptr_t)((struct fake_sensor *)sensor)->device;
}
static struct fake_sensor *in_roster(const char *serial);
bool fake_sensor_add_device(rift_sensor_ctx *sensor, rift_tracked_device *device) {
  struct fake_sensor *s = (void *)sensor;
  CHECK(!holds(s->tracker->tracker_lock) && !holds(s->tracker->hotplug.lock));
  CHECK(s->n_devices < 8);
  /* Adoption attaches devices before the roster join. Hold that gap open until the worker's
   * Windows search has run, and check the started sensor is still held without being listed. */
  if (stall_adopt && s->device == adopt_device) {
    CHECK(s->tracker->hotplug.adopting == sensor);
    CHECK(s->tracker->hotplug.n_done == 0);
    CHECK(in_roster(s->serial) == NULL);
    pthread_mutex_lock(&gate);
    in_adopt_gap = true;
    pthread_mutex_lock(&fake);
    opens_at_gap = opens[adopt_device];
    pthread_mutex_unlock(&fake);
    release_list = true;
    pthread_cond_broadcast(&gate_cond);
    CHECK(wait_gate(&adopt_gap_seen));
    CHECK(!gap_opened_held);
    stall_adopt = false;
    pthread_mutex_unlock(&gate);
  }
  s->devices[s->n_devices++] = device; /* no deduplication: a repeat shows */
  return true;
}
void fake_sensor_request_stop(rift_sensor_ctx *sensor) {
  struct fake_sensor *s = (void *)sensor;
  no_engine_locks(s->tracker);
  s->stopping = true;
}
void fake_sensor_retry(rift_sensor_ctx *sensor, uint64_t now) {
  (void)now;
  no_engine_locks(((struct fake_sensor *)sensor)->tracker);
}
void fake_sensor_free(rift_sensor_ctx *sensor) {
  struct fake_sensor *s = (void *)sensor;
  no_engine_locks(s->tracker);
  CHECK(!s->freed);
  fake_close((void *)(uintptr_t)s->device);
  rift_sensor_close_attachment_pin(s->pin_fd);
  s->pin_fd = -1;
  pthread_mutex_lock(&fake);
  s->freed = true;
  s->retired_when_freed = s->retired;
  s->freed_seq = ++sequence;
  live--;
  pthread_cond_broadcast(&fake_changed);
  pthread_mutex_unlock(&fake);
}
static int delay_slots = -1;
void fake_kalman_init(rift_kalman_6dof_filter *state, posef *pose, int num_delay_slots) {
  (void)state; (void)pose;
  delay_slots = num_delay_slots;
}
void exp_filter_pose_init(exp_filter_pose *f) { (void)f; }
led_search_model_t *led_search_model_new(rift_leds *leds) { (void)leds; return (void *)1; }
void led_search_model_free(led_search_model_t *model) { (void)model; }
void rift_kalman_6dof_clear(rift_kalman_6dof_filter *state) { (void)state; }
void rift_pose_finder_cv1touch_reset(void) {}
void *ohmd_allocfn(ohmd_context *ctx, const char *message, size_t size) {
  (void)ctx; (void)message;
  return calloc(1, size);
}

static int fd_count(void) {
  DIR *dir = opendir("/proc/self/fd"); CHECK(dir); int count = 0;
  for (struct dirent *e; (e = readdir(dir));) if (e->d_name[0] != '.') count++;
  CHECK(closedir(dir) == 0); return count;
}
static void camera_path(char *out, size_t size, int id) {
  CHECK(snprintf(out, size, "%s/1-2.%d", sysfs_root, id) < (int)size);
}
static void fixture_camera(int id, bool create) {
  char path[PATH_MAX], field[PATH_MAX]; camera_path(path, sizeof path, id);
  if (create) CHECK(mkdir(path, 0700) == 0);
  for (int f = 0; f < 2; f++) {
    CHECK(snprintf(field, sizeof field, "%s/%s", path, f ? "devnum" : "busnum") < (int)sizeof field);
    if (create) {
      FILE *out = fopen(field, "w"); CHECK(out); CHECK(fprintf(out, "%d\n", f ? id : 1) > 0); CHECK(fclose(out) == 0);
    } else CHECK(unlink(field) == 0);
  }
  if (!create) CHECK(rmdir(path) == 0);
}
/* The key the watcher reads for device id now. */
static ohmd_cv1touch_sensor_attachment sysfs_key(int id) {
  char path[PATH_MAX]; camera_path(path, sizeof path, id); struct stat st; CHECK(stat(path, &st) == 0);
  uint64_t inode = (uint64_t)st.st_ino;
  return (ohmd_cv1touch_sensor_attachment){.kind = OHMD_CV1TOUCH_ATTACHMENT_SYSFS,
    .data = {1u | ((uint32_t)id << 8), (uint32_t)inode, (uint32_t)(inode >> 32)}};
}
static void plug(int id, const char *serial) {
  pthread_mutex_lock(&fake);
  plugged[n_plugged++] = id;
  snprintf(serial_at[id], sizeof serial_at[id], "%s", serial);
  pthread_mutex_unlock(&fake);
}
static void unplug(int id) {
  pthread_mutex_lock(&fake);
  for (int i = 0; i < n_plugged; i++)
    if (plugged[i] == id) {
      memmove(plugged + i, plugged + i + 1, (size_t)(n_plugged - i - 1) * sizeof *plugged);
      n_plugged--;
      break;
    }
  pthread_mutex_unlock(&fake);
}
/* The engine thread's poll, until the worker has finished something. */
static ohmd_cv1touch_sensor_added finished(void) {
  ohmd_cv1touch_sensor_added out;
  for (int i = 0; i < 5000; i++) {
    int n = rift_tracker_poll_sensor_adds(tracker, &out, 1);
    CHECK(n >= 0 && n <= 1);
    if (n == 1)
      return out;
    usleep(1000);
  }
  CHECK(!"the worker finished nothing in 5 s");
  return out;
}
static void wait_freed(struct fake_sensor *s) {
  pthread_mutex_lock(&fake);
  for (int i = 0; i < 500 && !s->freed; i++) {
    pthread_mutex_unlock(&fake);
    usleep(10000);
    pthread_mutex_lock(&fake);
  }
  CHECK(s->freed);
  pthread_mutex_unlock(&fake);
}
static void wait_start_entered(int count) {
  pthread_mutex_lock(&fake);
  while (starts_entered < count)
    CHECK(pthread_cond_wait(&fake_changed, &fake) == 0);
  pthread_mutex_unlock(&fake);
}
static void let_starts_go(void) {
  pthread_mutex_lock(&fake);
  hold_starts = false;
  pthread_cond_broadcast(&fake_changed);
  pthread_mutex_unlock(&fake);
}
static struct fake_sensor *in_roster(const char *serial) {
  for (size_t i = 0; i < tracker->n_sensors; i++) {
    struct fake_sensor *s = (void *)tracker->sensors[i];
    if (!strcmp(s->serial, serial))
      return s;
  }
  return NULL;
}
static rift_tracked_device_imu_calibration no_calibration;
static rift_tracked_device *add_controller(int id) {
  posef pose = {.orient = {{0, 0, 0, 1}}};
  return rift_tracker_add_device(tracker, id, &pose, &pose, NULL, &no_calibration);
}
static const uint8_t radio[5] = {1, 2, 3, 4, 5};
static ohmd_context *ctx;
static void open_tracker(void) {
  starts_entered = 0;
  tracker = NULL;
  tracker = rift_tracker_new(ctx, radio);
  CHECK(tracker);
}
static void close_tracker(int baseline_fds) {
  rift_tracker_free(tracker);
  tracker = NULL;
  CHECK(live == 0 && handles == 0);
  CHECK(fd_count() == baseline_fds);
  for (int i = 0; i < n_made; i++)
    free(made[i]);
  n_made = 0;
  n_plugged = 0;
  memset(opens, 0, sizeof opens);
  __atomic_store_n(&worker_known, false, __ATOMIC_RELEASE);
}

/* An engine opened with no sensors: its controllers still get delay slots, so a sensor plugged
 * in later can feed their fusion. */
static void controllers_of_a_sensorless_engine_get_delay_slots(int fds) {
  open_tracker();
  CHECK(tracker->n_sensors == 0);
  add_controller(1);
  CHECK(delay_slots == NUM_POSE_DELAY_SLOTS);
  close_tracker(fds);
}

/* A plugged-in sensor joins the running engine: every controller attached once, in tracker
 * order (including one that came online during its start), its pin kept with its record, its
 * pose given with the request, and frames accepted only from the end of its start. */
static void a_plugged_in_sensor_joins_the_running_engine(int fds) {
  plug(1, "TEST000001");
  open_tracker();
  CHECK(tracker->n_sensors == 1 && tracker->next_sensor_id == 1);
  rift_tracked_device *right = add_controller(1);
  fixture_camera(2, true);
  plug(2, "TEST000002");
  hold_starts = true;
  ohmd_cv1touch_sensor_attachment key = sysfs_key(2);
  ohmd_cv1touch_sensor_pose pose = {.serial = "test000002", .orient = {0, 0, 0, 1}, .pos = {1, 2, 3}};
  CHECK(rift_tracker_add_sensor(tracker, "TEST000002", &key, &pose) == 0);
  CHECK(rift_tracker_add_sensor(tracker, "test000002", &key, NULL) == 1); /* queued already */
  CHECK(rift_tracker_add_sensor(tracker, "TEST000001", &key, NULL) == 1); /* open already */
  wait_start_entered(2);
  uint64_t before_start_done = fake_now(NULL);
  rift_tracked_device *left = add_controller(2);   /* online while it starts */
  ohmd_cv1touch_sensor_added none;
  CHECK(rift_tracker_poll_sensor_adds(tracker, &none, 1) == 0);
  let_starts_go();
  ohmd_cv1touch_sensor_added added = finished();
  CHECK(added.outcome == OHMD_CV1TOUCH_SENSOR_ADD_STARTED && added.error_code == 0);
  CHECK(!strcmp(added.stream.serial, "TEST000002") && added.stream.alt_setting == 2);
  CHECK(!memcmp(&added.stream.attachment, &key, sizeof key));
  struct fake_sensor *s = in_roster("TEST000002");
  CHECK(s && tracker->n_sensors == 2 && tracker->next_sensor_id == 2);
  CHECK(s->n_devices == 2 && s->devices[0] == right && s->devices[1] == left);
  CHECK(s->accept_from >= before_start_done && s->accept_from != UINT64_MAX);
  CHECK(s->have_pose && !strcmp(s->pose.serial, "test000002") && s->pose.pos[2] == 3);
  /* its callbacks' lifetime, as the engine reports it, and as the accessor does until retired */
  uint64_t lifetime = 0;
  CHECK(added.lifetime != 0 && added.lifetime == s->lifetime);
  CHECK(rift_tracker_get_sensor_lifetime(tracker, "test000002", &lifetime) == 0 && lifetime == s->lifetime);
  ohmd_cv1touch_sensor_stream stream;
  CHECK(rift_tracker_get_sensor_stream(tracker, 1, &stream) == 0 && !strcmp(stream.serial, "TEST000002"));
  int pin = tracker->sensor_streams[1].pin_fd;
  CHECK(pin >= 0 && (fcntl(pin, F_GETFD) & FD_CLOEXEC) && (fcntl(pin, F_GETFL) & O_PATH));
  CHECK(fd_count() == fds + 2);
  /* a later controller is attached by the tracker itself, as for any sensor */
  CHECK(rift_tracker_add_sensor(tracker, "TEST000002", &key, NULL) == 1);
  CHECK(rift_tracker_retire_sensor(tracker, "TEST000002") == 0);
  CHECK(rift_tracker_get_sensor_lifetime(tracker, "TEST000002", &lifetime) == -1);
  CHECK(rift_tracker_get_sensor_lifetime(tracker, "TEST000003", &lifetime) == -1);
  close_tracker(fds);
  fixture_camera(2, false);
}

/* Replugging: the old object is retired, leaves the roster and is freed before the worker
 * opens anything else, even when the release and the new add wait together. */
static void a_replugged_sensor_frees_its_old_object_first(int fds) {
  fixture_camera(2, true);
  fixture_camera(3, true);
  plug(2, "TEST000002");
  plug(3, "TEST000003");
  open_tracker();
  CHECK(tracker->n_sensors == 2);
  struct fake_sensor *old = in_roster("TEST000002");
  hold_starts = true;
  /* the worker is busy starting another sensor */
  fixture_camera(4, true);
  plug(4, "TEST000004");
  ohmd_cv1touch_sensor_attachment other = sysfs_key(4);
  CHECK(rift_tracker_add_sensor(tracker, "TEST000004", &other, NULL) == 0);
  wait_start_entered(3);
  /* meanwhile sensor 2 was unplugged and plugged back in */
  CHECK(rift_tracker_release_sensor(tracker, "test000002") == 0);
  CHECK(old->retired && !in_roster("TEST000002") && tracker->n_sensors == 1);
  CHECK(rift_tracker_release_sensor(tracker, "TEST000002") == 0); /* idempotent */
  unplug(2);
  fixture_camera(2, false);
  fixture_camera(2, true);
  plug(2, "TEST000002");
  ohmd_cv1touch_sensor_attachment key = sysfs_key(2);
  CHECK(rift_tracker_add_sensor(tracker, "TEST000002", &key, NULL) == 0);
  let_starts_go();
  CHECK(finished().outcome == OHMD_CV1TOUCH_SENSOR_ADD_STARTED);
  ohmd_cv1touch_sensor_added added = finished();
  CHECK(added.outcome == OHMD_CV1TOUCH_SENSOR_ADD_STARTED && !strcmp(added.stream.serial, "TEST000002"));
  CHECK(old->freed && old->retired_when_freed && old->freed_seq < last_open_seq[2]);
  CHECK(in_roster("TEST000002") != old && tracker->n_sensors == 3);
  close_tracker(fds);
  for (int id = 2; id <= 4; id++) fixture_camera(id, false);
}

/* The incarnation is checked before USB is opened: a different connection at the same place,
 * or no device at all, never opens anything. Transient open failures keep libusb's code. */
static void a_stale_or_missing_attachment_is_never_opened(int fds) {
  fixture_camera(5, true);
  open_tracker();
  plug(5, "TEST000005");
  ohmd_cv1touch_sensor_attachment key = sysfs_key(5);
  char path[PATH_MAX];
  camera_path(path, sizeof path, 5);
  int old_node = open(path, O_PATH | O_DIRECTORY | O_CLOEXEC); /* its inode stays taken */
  CHECK(old_node >= 0);
  fixture_camera(5, false);
  fixture_camera(5, true); /* replugged since the watcher's scan */
  CHECK(rift_tracker_add_sensor(tracker, "TEST000005", &key, NULL) == 0);
  ohmd_cv1touch_sensor_added added = finished();
  CHECK(added.outcome == OHMD_CV1TOUCH_SENSOR_ADD_STALE && opens[5] == 0);
  CHECK(!memcmp(&added.stream.attachment, &key, sizeof key) && !strcmp(added.stream.serial, "TEST000005"));
  unplug(5);
  key = sysfs_key(5);
  CHECK(rift_tracker_add_sensor(tracker, "TEST000005", &key, NULL) == 0);
  CHECK(finished().outcome == OHMD_CV1TOUCH_SENSOR_ADD_NOT_FOUND && opens[5] == 0);
  plug(5, "TEST000005");
  open_error[5] = LIBUSB_ERROR_BUSY;
  CHECK(rift_tracker_add_sensor(tracker, "TEST000005", &key, NULL) == 0);
  added = finished();
  CHECK(added.outcome == OHMD_CV1TOUCH_SENSOR_ADD_OPEN_FAILED && added.error_code == LIBUSB_ERROR_BUSY);
  open_error[5] = 0;
  fail_setup = 5;
  setup_error = LIBUSB_ERROR_NO_DEVICE;
  CHECK(rift_tracker_add_sensor(tracker, "TEST000005", &key, NULL) == 0);
  added = finished();
  CHECK(added.outcome == OHMD_CV1TOUCH_SENSOR_ADD_OPEN_FAILED && added.error_code == LIBUSB_ERROR_NO_DEVICE);
  fail_setup = 0;
  CHECK(tracker->n_sensor_streams == 0 && tracker->n_sensors == 0);
  CHECK(close(old_node) == 0);
  CHECK(fd_count() == fds);
  /* invalid requests are refused before anything is queued */
  ohmd_cv1touch_sensor_attachment unknown = {0}, no_inode = {.kind = OHMD_CV1TOUCH_ATTACHMENT_SYSFS, .data = {1 | (5 << 8)}};
  ohmd_cv1touch_sensor_pose other_pose = {.serial = "TEST000006", .orient = {0, 0, 0, 1}};
  CHECK(rift_tracker_add_sensor(tracker, "TEST000005", &unknown, NULL) == -1);
  CHECK(rift_tracker_add_sensor(tracker, "TEST000005", &no_inode, NULL) == -1);
  CHECK(rift_tracker_add_sensor(tracker, "", &key, NULL) == -1);
  CHECK(rift_tracker_add_sensor(tracker, "TEST/00005", &key, NULL) == -1);
  CHECK(rift_tracker_add_sensor(tracker, "TEST000005", &key, &other_pose) == -1);
  close_tracker(fds);
  fixture_camera(5, false);
}

/* An ignored sensor, or another serial than the one asked for, is closed again and rejected. */
static void an_ignored_or_other_sensor_is_rejected(int fds) {
  char ignored[1][32] = {"test000006"};
  CHECK(ohmd_cv1touch_configure_sensors(ctx, ignored, 1, NULL, 0) == 0);
  fixture_camera(6, true);
  fixture_camera(7, true);
  open_tracker();
  plug(6, "TEST000006");
  plug(7, "TEST000077");
  ohmd_cv1touch_sensor_attachment key = sysfs_key(6);
  CHECK(rift_tracker_add_sensor(tracker, "TEST000006", &key, NULL) == 0);
  CHECK(finished().outcome == OHMD_CV1TOUCH_SENSOR_ADD_REJECTED);
  key = sysfs_key(7);
  CHECK(rift_tracker_add_sensor(tracker, "TEST000007", &key, NULL) == 0);
  CHECK(finished().outcome == OHMD_CV1TOUCH_SENSOR_ADD_REJECTED);
  CHECK(tracker->n_sensors == 0 && handles == 0 && fd_count() == fds);
  close_tracker(fds);
  CHECK(ohmd_cv1touch_configure_sensors(ctx, NULL, 0, NULL, 0) == 0);
  fixture_camera(6, false);
  fixture_camera(7, false);
}

/* A start that fails for bandwidth is recorded once, for doctor, with its pin; forgetting the
 * record closes the pin. The sensor itself is freed. */
static void a_failed_start_is_recorded_once(int fds) {
  fixture_camera(8, true);
  open_tracker();
  plug(8, "TEST000008");
  fail_start_device = 8;
  ohmd_cv1touch_sensor_attachment key = sysfs_key(8);
  CHECK(rift_tracker_add_sensor(tracker, "TEST000008", &key, NULL) == 0);
  ohmd_cv1touch_sensor_added added = finished();
  CHECK(added.outcome == OHMD_CV1TOUCH_SENSOR_ADD_START_FAILED && added.error_code == LIBUSB_ERROR_IO);
  CHECK(added.lifetime == 0); /* only a sensor that joined has callbacks */
  CHECK(added.stream.error_kind == OHMD_CV1TOUCH_STREAM_ERROR_LIKELY_USB_BANDWIDTH && added.stream.os_error == ENOSPC);
  CHECK(tracker->n_sensors == 0 && live == 0 && tracker->n_sensor_streams == 1);
  ohmd_cv1touch_sensor_stream stream;
  CHECK(rift_tracker_get_sensor_stream(tracker, 0, &stream) == 0);
  CHECK(stream.error_kind == OHMD_CV1TOUCH_STREAM_ERROR_LIKELY_USB_BANDWIDTH);
  CHECK(!memcmp(&stream.attachment, &key, sizeof key));
  CHECK(fd_count() == fds + 1);
  rift_tracker_forget_sensor_stream(tracker, "TEST000008");
  CHECK(fd_count() == fds);
  fail_start_device = 0;
  close_tracker(fds);
  fixture_camera(8, false);
}

/* Closing the engine during a start, with a release still queued, leaks nothing. */
static void closing_during_an_add_leaks_nothing(int fds) {
  fixture_camera(9, true);
  fixture_camera(10, true);
  plug(9, "TEST000009");
  open_tracker();
  plug(10, "TEST000010");
  hold_starts = true;
  ohmd_cv1touch_sensor_attachment key = sysfs_key(10);
  CHECK(rift_tracker_add_sensor(tracker, "TEST000010", &key, NULL) == 0);
  wait_start_entered(2);
  CHECK(rift_tracker_release_sensor(tracker, "TEST000009") == 0);
  close_tracker(fds); /* the held start ends when the engine closes */
  hold_starts = false;
  fixture_camera(9, false);
  fixture_camera(10, false);
}

/* Retirement from another thread holds the roster lock through the sensor's retirement: the
 * engine thread cannot take that sensor out of the roster, and the worker cannot free it,
 * while retirement still holds its pointer. */
struct retirement { const char *serial; };
static void *retire_from_watcher(void *arg) {
  struct retirement *r = arg;
  retiring = true;
  CHECK(rift_tracker_retire_sensor(tracker, r->serial) == 0);
  return NULL;
}
static void retirement_never_reaches_a_freed_sensor(int fds) {
  fixture_camera(11, true);
  plug(11, "TEST000011");
  open_tracker();
  struct fake_sensor *s = in_roster("TEST000011");
  CHECK(s);
  /* the worker runs, as it would once anything was added or released */
  CHECK(start_hotplug_worker(tracker));
  struct retirement r = {"TEST000011"};
  pthread_t watcher;
  paused = false;
  __atomic_store_n(&pause_at, s, __ATOMIC_RELEASE);
  CHECK(pthread_create(&watcher, NULL, retire_from_watcher, &r) == 0);
  pthread_mutex_lock(&fake);
  while (!paused)
    CHECK(pthread_cond_wait(&fake_changed, &fake) == 0);
  pthread_mutex_unlock(&fake);
  CHECK(rift_tracker_release_sensor(tracker, "TEST000011") == 0);
  wait_freed(s);
  CHECK(s->retired_when_freed);
  CHECK(pthread_join(watcher, NULL) == 0);
  __atomic_store_n(&pause_at, NULL, __ATOMIC_RELEASE);
  close_tracker(fds);
  fixture_camera(11, false);
}

/* A Windows key cannot be verified natively: the worker opens unheld Rift Sensors until one
 * reports the serial. A device a sensor of this engine holds is never opened again. (Matching
 * logic only; Windows hardware was not exercised.) */
static void a_windows_key_matches_an_unheld_sensor_by_serial(int fds) {
  plug(12, "TEST000012");
  open_tracker();
  struct fake_sensor *held = in_roster("TEST000012");
  CHECK(held && opens[12] == 1);
  plug(13, "TEST000099");
  plug(14, "TEST000014");
  ohmd_cv1touch_sensor_attachment arrival = {.kind = OHMD_CV1TOUCH_ATTACHMENT_WINDOWS,
    .data = {0x9abcdef0, 0x12345678, 0}};
  CHECK(rift_tracker_add_sensor(tracker, "TEST000014", &arrival, NULL) == 0);
  ohmd_cv1touch_sensor_added added = finished();
  CHECK(added.outcome == OHMD_CV1TOUCH_SENSOR_ADD_STARTED);
  CHECK(!memcmp(&added.stream.attachment, &arrival, sizeof arrival));
  CHECK(opens[12] == 1 && opens[13] == 1 && opens[14] == 2);
  CHECK(in_roster("TEST000014") && tracker->n_sensors == 2);
  /* none unheld reports it */
  CHECK(rift_tracker_add_sensor(tracker, "TEST000015", &arrival, NULL) == 0);
  CHECK(finished().outcome == OHMD_CV1TOUCH_SENSOR_ADD_NOT_FOUND);
  CHECK(opens[12] == 1 && opens[14] == 2);
  /* one could not be opened: it may well have been the one */
  open_error[13] = LIBUSB_ERROR_ACCESS;
  CHECK(rift_tracker_add_sensor(tracker, "TEST000015", &arrival, NULL) == 0);
  added = finished();
  CHECK(added.outcome == OHMD_CV1TOUCH_SENSOR_ADD_OPEN_FAILED && added.error_code == LIBUSB_ERROR_ACCESS);
  open_error[13] = 0;
  close_tracker(fds);
}

/* While the engine thread adopts a started sensor, before that sensor joins the roster, the
 * worker's Windows search still treats its device as held. It used to be in neither done nor
 * the roster for that window, so the search opened it. (Matching logic only; Windows hardware
 * was not exercised.) */
static void a_sensor_being_adopted_stays_held(int fds) {
  open_tracker();
  CHECK(add_controller(1));
  plug(16, "TEST000016");
  plug(17, "TEST000017");
  worker_lists = 0;
  stall_list = 2;
  release_list = false;
  stalled_in_list = false;
  stall_adopt = true;
  in_adopt_gap = false;
  adopt_gap_seen = false;
  gap_opened_held = false;
  adopt_device = 16;
  ohmd_cv1touch_sensor_attachment key = {.kind = OHMD_CV1TOUCH_ATTACHMENT_WINDOWS,
    .data = {0x11111111, 0x22222222, 0}};
  CHECK(rift_tracker_add_sensor(tracker, "TEST000016", &key, NULL) == 0);
  CHECK(rift_tracker_add_sensor(tracker, "TEST000017", &key, NULL) == 0);
  pthread_mutex_lock(&gate);
  CHECK(wait_gate(&stalled_in_list));
  pthread_mutex_unlock(&gate);
  CHECK(tracker->hotplug.n_done == 1);
  ohmd_cv1touch_sensor_added added;
  CHECK(rift_tracker_poll_sensor_adds(tracker, &added, 1) == 1);
  CHECK(added.outcome == OHMD_CV1TOUCH_SENSOR_ADD_STARTED);
  CHECK(!strcmp(added.stream.serial, "TEST000016"));
  CHECK(!gap_opened_held);
  CHECK(in_roster("TEST000016") && tracker->hotplug.adopting == NULL);
  CHECK(opens[16] == opens_at_gap);
  stall_list = 0;
  ohmd_cv1touch_sensor_added second = finished();
  CHECK(second.outcome == OHMD_CV1TOUCH_SENSOR_ADD_STARTED);
  CHECK(!strcmp(second.stream.serial, "TEST000017"));
  CHECK(in_roster("TEST000017") && tracker->n_sensors == 2);
  CHECK(opens[16] == opens_at_gap && opens[17] == 2);
  close_tracker(fds);
}

int main(void) {
  const char *tmp = getenv("TMPDIR");
  CHECK(snprintf(sysfs_root, sizeof sysfs_root, "%s/cv1t-hotplug-XXXXXX", tmp ? tmp : ".") < (int)sizeof sysfs_root);
  CHECK(mkdtemp(sysfs_root));
  fixture_camera(1, true);
  fixture_camera(12, true);
  int fds = fd_count();
  ctx = ohmd_ctx_create();
  CHECK(ctx);
  fds = fd_count();
  controllers_of_a_sensorless_engine_get_delay_slots(fds);
  a_plugged_in_sensor_joins_the_running_engine(fds);
  a_replugged_sensor_frees_its_old_object_first(fds);
  a_stale_or_missing_attachment_is_never_opened(fds);
  an_ignored_or_other_sensor_is_rejected(fds);
  a_failed_start_is_recorded_once(fds);
  closing_during_an_add_leaks_nothing(fds);
  retirement_never_reaches_a_freed_sensor(fds);
  a_windows_key_matches_an_unheld_sensor_by_serial(fds);
  a_sensor_being_adopted_stays_held(fds);
  CHECK(workers_started > 0);
  ohmd_ctx_destroy(ctx);
  fixture_camera(1, false);
  fixture_camera(12, false);
  CHECK(rmdir(sysfs_root) == 0);
  puts("test_sensor_hotplug: ok");
  return 0;
}
