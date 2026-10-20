// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
#define _GNU_SOURCE
#include <stdlib.h>
#include <errno.h>
#include <limits.h>
#include <sys/stat.h>
#include <dirent.h>
static char sysfs_root[PATH_MAX];
#define CV1TOUCH_SYSFS_USB_ROOT sysfs_root
static int realloc_calls, fail_realloc;
static void *test_realloc(void *p, size_t n) {
  if (++realloc_calls == fail_realloc)
    return NULL;
  return realloc(p, n);
}
#define realloc test_realloc
/* Actual tracker discovery/lifetime with hardware and worker creation
 * substituted. */
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
#define rift_sensor_usb_new fake_sensor_device
#define rift_sensor_new fake_sensor_new
#define rift_sensor_start fake_sensor_start
#define rift_sensor_free fake_sensor_free
#define rift_sensor_request_stop fake_sensor_request_stop
#define rift_sensor_retry fake_sensor_retry
#define rift_sensor_get_cv1touch_info fake_sensor_info
#define rift_sensor_get_stream_info fake_sensor_stream_info
#define rift_sensor_get_video_quality fake_sensor_quality
#define rift_sensor_set_attachment fake_sensor_set_attachment
#define rift_sensor_take_attachment_pin fake_sensor_take_attachment_pin
#define rift_sensor_serial_no fake_sensor_serial
#define rift_sensor_retire fake_sensor_retire
#include "rift-tracker.c"
#include "test_util.h"

static int requested, live, starts, released, handles;
static int expected_stops, stop_requests, retries;
static int fail_descriptor, fail_serial, duplicate_serial, fail_open,
    fail_device, fail_analyser, fail_start, fail_alloc, alloc_calls, fail_mutex,
    mutex_calls, fail_thread;
static libusb_device *devices[66];
struct fake_sensor {
  int id;
  char serial[32];
  libusb_device_handle *handle;
  bool stopping, retired;
  ohmd_cv1touch_sensor_attachment attachment;
  int pin_fd;
};
int fake_usb_init(libusb_context **ctx) {
  *ctx = (void *)1;
  return 0;
}
void fake_usb_exit(libusb_context *ctx) { (void)ctx; }
ssize_t fake_device_list(libusb_context *ctx, libusb_device ***out) {
  (void)ctx;
  for (int i = 0; i < requested; i++)
    devices[i] = (void *)(uintptr_t)(i + 1);
  devices[requested] = NULL;
  *out = devices;
  return requested;
}
void fake_free_list(libusb_device **list, int unref) {
  (void)list;
  (void)unref;
}
int fake_descriptor(libusb_device *dev, struct libusb_device_descriptor *desc) {
  if ((int)(uintptr_t)dev == fail_descriptor)
    return -1;
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
void fake_sensor_set_attachment(rift_sensor_ctx *sensor, const ohmd_cv1touch_sensor_attachment *attachment, int pin_fd) {
  ((struct fake_sensor *)sensor)->attachment = *attachment;
  ((struct fake_sensor *)sensor)->pin_fd = pin_fd;
}
int fake_sensor_take_attachment_pin(rift_sensor_ctx *sensor) {
  struct fake_sensor *s = (void *)sensor; int fd = s->pin_fd; s->pin_fd = -1; return fd;
}
static uint64_t generation;
static void snapshot_attachment(void *user, const char *serial, ohmd_cv1touch_sensor_attachment *out) {
  CHECK(user == &generation && strcmp(serial, "TEST000001") == 0);
  *out = (ohmd_cv1touch_sensor_attachment){.kind = OHMD_CV1TOUCH_ATTACHMENT_WINDOWS,
    .data = {(uint32_t)generation, (uint32_t)(generation >> 32), 0}};
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
static uint64_t fixture_inode(int id) {
  char path[PATH_MAX]; camera_path(path, sizeof path, id); struct stat st; CHECK(stat(path, &st) == 0); return (uint64_t)st.st_ino;
}
static bool replace_during_open;
int fake_open(libusb_device *dev, libusb_device_handle **handle) {
  if ((int)(uintptr_t)dev == fail_open)
    return -1;
  if (replace_during_open && (uintptr_t)dev == 1) {
    fixture_camera(1, false); fixture_camera(1, true); replace_during_open = false;
  }
  *handle = (void *)dev;
  handles++;
  return 0;
}
void fake_close(libusb_device_handle *handle) {
  (void)handle;
  handles--;
  CHECK(handles >= 0);
}
int fake_serial(libusb_device_handle *handle, uint8_t index,
                unsigned char *data, int len) {
  (void)index;
  if ((int)(uintptr_t)handle == fail_serial)
    return -1;
  if ((int)(uintptr_t)handle == duplicate_serial)
    return snprintf((char *)data, len, "test%06d", 1);
  return snprintf((char *)data, len, "TEST%06zu", (size_t)(uintptr_t)handle);
}
ohmd_thread *fake_thread(ohmd_context *ctx, unsigned int (*routine)(void *),
                         void *arg) {
  (void)ctx;
  (void)routine;
  (void)arg;
  return fail_thread ? NULL : (void *)1;
}
void fake_join(ohmd_thread *thread) { (void)thread; }
rift_sensor_device *fake_sensor_device(ohmd_context *ctx, int id,
                                       const char *serial, libusb_context *usb,
                                       libusb_device_handle *handle,
                                       const uint8_t radio[5]) {
  (void)ctx;
  (void)id;
  (void)serial;
  (void)usb;
  (void)radio;
  if ((int)(uintptr_t)handle == fail_device) {
    fake_close(handle);
    return NULL;
  }
  return (void *)handle;
}
rift_sensor_ctx *fake_sensor_new(ohmd_context *ctx, int id, const char *serial,
                                 rift_sensor_device *device,
                                 rift_tracker_ctx *tracker) {
  (void)ctx;
  (void)tracker;
  if ((int)(uintptr_t)device == fail_analyser) {
    fake_close((void *)device);
    return NULL;
  }
  struct fake_sensor *s = calloc(1, sizeof *s);
  CHECK(s);
  s->id = id;
  s->pin_fd = -1;
  s->handle = (void *)device;
  snprintf(s->serial, sizeof s->serial, "%s", serial);
  live++;
  return (void *)s;
}
bool fake_sensor_start(rift_sensor_ctx *sensor) {
  starts++;
  if (generation) generation++; /* A is replaced by B during its failed start. */
  return ((struct fake_sensor *)sensor)->id != fail_start - 1;
}
void fake_sensor_retire(rift_sensor_ctx *sensor) {
  ((struct fake_sensor *)sensor)->retired = true;
}
void fake_sensor_request_stop(rift_sensor_ctx *sensor) {
  struct fake_sensor *s = (void *)sensor;
  CHECK(!s->stopping);
  s->stopping = true;
  stop_requests++;
}
void fake_sensor_retry(rift_sensor_ctx *sensor, uint64_t now_ns) {
  CHECK(sensor && now_ns == 123);
  retries++;
}
void fake_sensor_free(rift_sensor_ctx *sensor) {
  CHECK(sensor);
  if (expected_stops) {
    CHECK(stop_requests == expected_stops);
    CHECK(((struct fake_sensor *)sensor)->stopping);
  }
  live--;
  released++;
  fake_close(((struct fake_sensor *)sensor)->handle);
  rift_sensor_close_attachment_pin(((struct fake_sensor *)sensor)->pin_fd);
  free(sensor);
}
const char *fake_sensor_serial(rift_sensor_ctx *sensor) {
  return ((struct fake_sensor *)sensor)->serial;
}
void fake_sensor_info(rift_sensor_ctx *sensor, ohmd_cv1touch_sensor *out) {
  memset(out, 0, sizeof *out);
  snprintf(out->serial, sizeof out->serial, "%s", fake_sensor_serial(sensor));
}
void fake_sensor_stream_info(rift_sensor_ctx *sensor, ohmd_cv1touch_sensor_stream *out) {
  memset(out, 0, sizeof *out);
  snprintf(out->serial, sizeof out->serial, "%s", fake_sensor_serial(sensor));
  out->attachment = ((struct fake_sensor *)sensor)->attachment;
  if (((struct fake_sensor *)sensor)->id == fail_start - 1) {
    out->error_kind = OHMD_CV1TOUCH_STREAM_ERROR_LIKELY_USB_BANDWIDTH;
    out->error_code = LIBUSB_ERROR_OTHER;
    out->os_error = ENOSPC;
  } else out->alt_setting = 2;
}
void *ohmd_allocfn(ohmd_context *ctx, const char *message, size_t size) {
  (void)ctx;
  (void)message;
  if (++alloc_calls == fail_alloc)
    return NULL;
  return calloc(1, size);
}
ohmd_mutex *ohmd_create_mutex(ohmd_context *ctx) {
  (void)ctx;
  return ++mutex_calls == fail_mutex ? NULL : (void *)1;
}
void ohmd_destroy_mutex(ohmd_mutex *mutex) { (void)mutex; }
void led_search_model_free(led_search_model_t *model) { (void)model; }
void rift_kalman_6dof_clear(rift_kalman_6dof_filter *state) { (void)state; }
void rift_pose_finder_cv1touch_reset(void) {}
void fake_sensor_quality(rift_sensor_ctx *sensor, ohmd_cv1touch_sensor_video_quality *out) {
  struct fake_sensor *fake = (struct fake_sensor *)sensor;
  memset(out, 0, sizeof *out);
  snprintf(out->serial, sizeof out->serial, "%s", fake->serial);
  out->video_format = OHMD_CV1TOUCH_VIDEO_RAW;
  out->frames_dropped = UINT64_C(4294967296) + fake->id;
}
int main(void) {
  const char *tmp = getenv("TMPDIR");
  CHECK(snprintf(sysfs_root, sizeof sysfs_root, "%s/cv1t-pins-XXXXXX", tmp ? tmp : ".") < (int)sizeof sysfs_root);
  CHECK(mkdtemp(sysfs_root));
  for (int id = 1; id <= 65; id++) fixture_camera(id, true);
  int baseline_fds = fd_count();
  ohmd_context *ctx = ohmd_ctx_create();
  CHECK(ctx);
  uint8_t radio[5] = {0};
  int counts[] = {0, 1, 4, 5, 9, 65};
  for (size_t k = 0; k < sizeof counts / sizeof counts[0]; k++) {
    requested = counts[k];
    starts = released = 0;
    rift_tracker_ctx *tr = rift_tracker_new(ctx, radio);
    CHECK(tr);
    CHECK(rift_tracker_get_sensor_count(tr) == requested);
    CHECK(starts == requested && live == requested);
    ohmd_cv1touch_sensor_video_quality quality;
    CHECK(rift_tracker_get_sensor_video_quality(tr, -1, &quality) == -1);
    CHECK(rift_tracker_get_sensor_video_quality(tr, requested, &quality) == -1);
    CHECK(rift_tracker_get_sensor_video_quality(tr, 0, NULL) == -1);
    for (int i = 0; i < requested; i++) {
      ohmd_cv1touch_sensor info;
      CHECK(rift_tracker_get_sensor_info(tr, i, &info) == 0);
      CHECK(info.serial[0]);
      CHECK(rift_tracker_get_sensor_video_quality(tr, i, &quality) == 0);
      CHECK(!strcmp(info.serial, quality.serial));
      CHECK(quality.video_format == OHMD_CV1TOUCH_VIDEO_RAW);
      CHECK(quality.frames_dropped == UINT64_C(4294967296) + i);
      int pin = tr->sensor_streams[i].pin_fd;
      CHECK(pin >= 0 && (fcntl(pin, F_GETFD) & FD_CLOEXEC) && (fcntl(pin, F_GETFL) & O_PATH));
    }
    if (requested) {
      CHECK(rift_tracker_retire_sensor(tr, "test000001") == 0);
      CHECK(((struct fake_sensor *)tr->sensors[0])->retired);
    }
    retries = 0;
    rift_tracker_retry_video(tr, 123);
    CHECK(retries == requested);
    expected_stops = requested;
    stop_requests = 0;
    rift_tracker_free(tr);
    CHECK(fd_count() == baseline_fds);
    CHECK(stop_requests == requested);
    expected_stops = 0;
    CHECK(live == 0 && released == requested && handles == 0);
  }
  /* Full context configuration reaches discovery beyond the legacy eighth
   * entry. */
  char ignored[1][32] = {"test000065"};
  CHECK(ohmd_cv1touch_configure_sensors(ctx, ignored, 1, NULL, 0) == 0);
  requested = 65;
  rift_tracker_ctx *filtered = rift_tracker_new(ctx, radio);
  CHECK(filtered && rift_tracker_get_sensor_count(filtered) == 64);
  rift_tracker_free(filtered);
  CHECK(fd_count() == baseline_fds);
  CHECK(live == 0 && handles == 0);
  CHECK(ohmd_cv1touch_configure_sensors(ctx, NULL, 0, NULL, 0) == 0);
  /* Every admission stage may fail: keep later devices and release each owned
   * handle. */
  int *failures[] = {&fail_descriptor, &fail_serial, &duplicate_serial,
                     &fail_open,       &fail_device, &fail_analyser,
                     &fail_start};
  for (size_t k = 0; k < sizeof failures / sizeof failures[0]; k++) {
    requested = 9;
    *failures[k] = 5;
    starts = released = 0;
    rift_tracker_ctx *tr = rift_tracker_new(ctx, radio);
    CHECK(tr);
    CHECK(rift_tracker_get_sensor_count(tr) == 8);
    if (failures[k] == &fail_start) {
      ohmd_cv1touch_sensor_stream stream;
      CHECK(rift_tracker_get_sensor_stream(tr, 4, &stream) == 0);
      CHECK(strcmp(stream.serial, "TEST000005") == 0);
      CHECK(stream.error_kind == OHMD_CV1TOUCH_STREAM_ERROR_LIKELY_USB_BANDWIDTH);
      CHECK(stream.error_code == LIBUSB_ERROR_OTHER && stream.os_error == ENOSPC);
      CHECK(stream.attachment.kind == OHMD_CV1TOUCH_ATTACHMENT_SYSFS);
      CHECK(stream.attachment.data[0] == (1u | (5u << 8)));
      CHECK(((uint64_t)stream.attachment.data[1] | ((uint64_t)stream.attachment.data[2] << 32)) == fixture_inode(5));
      int failed_pin = tr->sensor_streams[4].pin_fd;
      CHECK(failed_pin >= 0 && (fcntl(failed_pin, F_GETFD) & FD_CLOEXEC));
      CHECK(rift_tracker_get_sensor_stream(tr, 9, &stream) == -1);
      /* A successful reconnect/fallback replaces this serial's failure; it
       * does not append another record or leave a stale bandwidth diagnosis. */
      memset(&stream, 0, sizeof stream);
      snprintf(stream.serial, sizeof stream.serial, "test000005");
      stream.alt_setting = 1;
      CHECK(rift_tracker_record_sensor_stream(tr, &stream));
      CHECK(rift_tracker_get_sensor_stream(tr, 4, &stream) == 0);
      CHECK(stream.error_kind == OHMD_CV1TOUCH_STREAM_ERROR_NONE && stream.alt_setting == 1);
      CHECK(rift_tracker_get_sensor_stream(tr, 9, &stream) == -1);
      rift_tracker_forget_sensor_stream(tr, "TeSt000005");
      rift_tracker_forget_sensor_stream(tr, "TEST000005");
      CHECK(rift_tracker_get_sensor_stream(tr, 8, &stream) == -1);
      CHECK(rift_tracker_record_sensor_stream(tr, &(ohmd_cv1touch_sensor_stream){
        .serial = "RECONNECTED", .alt_setting = 1}));
      CHECK(rift_tracker_get_sensor_stream(tr, 8, &stream) == 0);
      CHECK(strcmp(stream.serial, "RECONNECTED") == 0 && stream.alt_setting == 1);
    }
    rift_tracker_free(tr);
    CHECK(fd_count() == baseline_fds);
    CHECK(live == 0 && handles == 0);
    *failures[k] = 0;
  }
  /* The native retained failure carries A even though the object was freed
   * and the OS snapshot used by the next open now describes B. */
  generation = UINT64_C(0x123456789abcdef0);
  ohmd_cv1touch_set_sensor_attachment_callback(ctx, snapshot_attachment, &generation);
  requested = fail_start = 1;
  rift_tracker_ctx *old_attachment = rift_tracker_new(ctx, radio);
  CHECK(old_attachment && rift_tracker_get_sensor_count(old_attachment) == 0 && live == 0);
  ohmd_cv1touch_sensor_stream old_start;
  CHECK(rift_tracker_get_sensor_stream(old_attachment, 0, &old_start) == 0);
  CHECK(old_start.attachment.kind == OHMD_CV1TOUCH_ATTACHMENT_WINDOWS);
  CHECK(old_start.attachment.data[0] == 0x9abcdef0 && old_start.attachment.data[1] == 0x12345678);
  CHECK(generation == UINT64_C(0x123456789abcdef1));
  rift_tracker_free(old_attachment);
  CHECK(fd_count() == baseline_fds);
  generation = fail_start = 0;
  ohmd_cv1touch_set_sensor_attachment_callback(ctx, NULL, NULL);
  /* Native capture precedes USB open: replacement at that boundary must not
   * turn A's failed start into an attempt on B, even with the same address. */
  uint64_t before_replug = fixture_inode(1);
  requested = fail_start = 1;
  replace_during_open = true;
  rift_tracker_ctx *pinned = rift_tracker_new(ctx, radio);
  CHECK(pinned && live == 0 && pinned->n_sensor_streams == 1);
  CHECK(rift_tracker_get_sensor_stream(pinned, 0, &old_start) == 0);
  uint64_t recorded_inode = (uint64_t)old_start.attachment.data[1] | ((uint64_t)old_start.attachment.data[2] << 32);
  CHECK(old_start.attachment.kind == OHMD_CV1TOUCH_ATTACHMENT_SYSFS && recorded_inode == before_replug);
  CHECK(recorded_inode != fixture_inode(1));
  int original_pin = pinned->sensor_streams[0].pin_fd;
  CHECK(original_pin >= 0 && (fcntl(original_pin, F_GETFD) & FD_CLOEXEC));
  CHECK(fcntl(original_pin, F_GETFL) & O_PATH);
  CHECK(fd_count() == baseline_fds + 1);
  /* Repeated replacement/upsert/forget leaves exactly one or zero pins. The
   * incoming pin is acquired before the old one closes, so inode reuse cannot
   * make two physical incarnations equal. */
  for (int cycle = 0; cycle < 100; cycle++) {
    fixture_camera(1, false); fixture_camera(1, true);
    ohmd_cv1touch_sensor_attachment next;
    int pin = pin_sensor_attachment((void *)1, &next);
    CHECK(pin >= 0 && (fcntl(pin, F_GETFD) & FD_CLOEXEC) && (fcntl(pin, F_GETFL) & O_PATH));
    old_start.attachment = next;
    int old_pin = pinned->sensor_streams[0].pin_fd;
    CHECK(record_sensor_stream(pinned, &old_start, pin));
    CHECK(fcntl(old_pin, F_GETFD) == -1 && errno == EBADF);
    CHECK(fd_count() == baseline_fds + 1);
    rift_tracker_forget_sensor_stream(pinned, old_start.serial);
    CHECK(fcntl(pin, F_GETFD) == -1 && errno == EBADF && fd_count() == baseline_fds);
    pin = pin_sensor_attachment((void *)1, &next); CHECK(pin >= 0);
    old_start.attachment = next;
    CHECK(record_sensor_stream(pinned, &old_start, pin));
    CHECK(fd_count() == baseline_fds + 1);
  }
  int last_pin = pinned->sensor_streams[0].pin_fd;
  rift_tracker_free(pinned);
  CHECK(fcntl(last_pin, F_GETFD) == -1 && errno == EBADF && fd_count() == baseline_fds);
  fail_start = 0;
  /* Unknown metadata cannot retain an unpinned identity. */
  char missing[PATH_MAX]; camera_path(missing, sizeof missing, 1);
  char number[PATH_MAX]; CHECK(snprintf(number, sizeof number, "%s/devnum", missing) < (int)sizeof number);
  CHECK(unlink(number) == 0);
  rift_tracker_ctx *unknown = rift_tracker_new(ctx, radio);
  CHECK(unknown && rift_tracker_get_sensor_stream(unknown, 0, &old_start) == 0);
  CHECK(old_start.attachment.kind == OHMD_CV1TOUCH_ATTACHMENT_UNKNOWN);
  CHECK(unknown->sensor_streams[0].pin_fd == -1);
  rift_tracker_free(unknown); CHECK(fd_count() == baseline_fds);
  FILE *restore = fopen(number, "w"); CHECK(restore); CHECK(fputs("1\n", restore) >= 0); CHECK(fclose(restore) == 0);
  /* Resource failures unwind the entire half-built tracker, with no lost old
   * realloc pointer. */
  requested = 9;
  realloc_calls = 0;
  fail_realloc = 2;
  CHECK(rift_tracker_new(ctx, radio) == NULL);
  CHECK(live == 0 && handles == 0 && fd_count() == baseline_fds);
  fail_realloc = 0;
  /* The stream table grows after the three sensor-pointer allocations. An
   * allocation failure must unwind started and not-yet-started sensors too. */
  for (int failure = 4; failure <= 6; failure++) {
    realloc_calls = 0;
    fail_realloc = failure;
    CHECK(rift_tracker_new(ctx, radio) == NULL);
    CHECK(live == 0 && handles == 0 && fd_count() == baseline_fds);
  }
  fail_realloc = 0;
  alloc_calls = 0;
  fail_alloc = 1;
  CHECK(rift_tracker_new(ctx, radio) == NULL);
  fail_alloc = 0;
  for (int k = 1; k <= 4; k++) {
    mutex_calls = 0;
    fail_mutex = k;
    CHECK(rift_tracker_new(ctx, radio) == NULL);
    CHECK(live == 0 && handles == 0);
  }
  fail_mutex = 0;
  fail_thread = 1;
  CHECK(rift_tracker_new(ctx, radio) == NULL);
  fail_thread = 0;
  /* Empty and last valid indexes remain checked for any admitted count. */
  requested = 65;
  rift_tracker_ctx *tr = rift_tracker_new(ctx, radio);
  ohmd_cv1touch_sensor info;
  CHECK(rift_tracker_get_sensor_info(tr, -1, &info) == -1);
  CHECK(rift_tracker_get_sensor_info(tr, 65, &info) == -1);
  /* A used report from the final camera makes a delayed state reclaimable. */
  rift_tracked_device_priv dev = {0};
  dev.n_delay_slots = 1;
  dev.delay_slots[0].valid = true;
  CHECK(reclaim_delay_slot(&dev) == NULL);
  for (int i = 0; i < 65; i++)
    dev.delay_slots[0].had_used_report |= i == 64;
  CHECK(reclaim_delay_slot(&dev) == &dev.delay_slots[0]);
  rift_tracker_free(tr);
  CHECK(fd_count() == baseline_fds);
  CHECK(live == 0 && handles == 0);
  ohmd_ctx_destroy(ctx);
  CHECK(fd_count() == baseline_fds);
  for (int id = 1; id <= 65; id++) fixture_camera(id, false);
  CHECK(rmdir(sysfs_root) == 0);
  puts("test_sensor_discovery: ok");
  return 0;
}
