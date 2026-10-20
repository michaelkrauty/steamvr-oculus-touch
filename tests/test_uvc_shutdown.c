// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* Exercise real callbacks, retry maintenance and stop with libusb's callback
 * lock contract. No USB devices are opened. Every failed completion must return,
 * even when no sibling transfer remains, so a peer can stop independently. */
#include <pthread.h>
#include <stdatomic.h>
#include <time.h>
#include <unistd.h>

#include "sensor/uvc.c"
#include "test_util.h"

static pthread_mutex_t waiters = PTHREAD_MUTEX_INITIALIZER;
static atomic_int submissions;
static atomic_bool submit_ok;
static int alt_result, alt_errno, submit_errno;
static bool alt_logging;
static struct rift_sensor_uvc_stream stream;
static struct libusb_transfer transfers[6];

int LIBUSB_CALL libusb_submit_transfer(struct libusb_transfer *transfer)
{
	(void)transfer;
	atomic_fetch_add(&submissions, 1);
	if (!submit_ok) {
		errno = submit_errno;
		return LIBUSB_ERROR_IO;
	}
	return 0;
}

int LIBUSB_CALL libusb_set_interface_alt_setting(libusb_device_handle *dev, int interface, int alt)
{
	(void)dev;
	CHECK(interface == 1 && (alt == 0 || alt == 1));
	if (alt != 0) {
		if (alt_errno >= 0) errno = alt_errno;
		/* Model libusb's log output replacing an ioctl error before returning. */
		if (alt_logging) errno = ENOSPC;
		return alt_result;
	}
	return 0;
}

void LIBUSB_CALL libusb_lock_event_waiters(libusb_context *ctx)
{
	(void)ctx;
	struct timespec deadline;
	CHECK(clock_gettime(CLOCK_REALTIME, &deadline) == 0);
	deadline.tv_sec += 2;
	/* A callback may not hold this lock indefinitely waiting for its own stop. */
	CHECK(pthread_mutex_timedlock(&waiters, &deadline) == 0);
}

void LIBUSB_CALL libusb_unlock_event_waiters(libusb_context *ctx)
{
	(void)ctx;
	CHECK(pthread_mutex_unlock(&waiters) == 0);
}

int LIBUSB_CALL libusb_wait_for_event(libusb_context *ctx, struct timeval *tv)
{
	(void)ctx;
	(void)tv;
	CHECK(false); /* the callback must retire its last transfer before unlocking */
	return 0;
}

void ohmd_lock_mutex(ohmd_mutex *mutex) { CHECK(mutex == (void *)1); }
void ohmd_unlock_mutex(ohmd_mutex *mutex) { CHECK(mutex == (void *)1); }
void ohmd_destroy_mutex(ohmd_mutex *mutex) { CHECK(mutex == (void *)1); }

void ohmd_sleep(double seconds)
{
	(void)seconds;
	struct timespec delay = {0, 500000};
	nanosleep(&delay, NULL);
}

static void *complete_transfers(void *unused)
{
	(void)unused;
	for (int i = 0; i < 6; i++) {
		CHECK(pthread_mutex_lock(&waiters) == 0);
		iso_transfer_cb(&transfers[i]);
		CHECK(pthread_mutex_unlock(&waiters) == 0);
	}
	return NULL;
}

static int allocation_count, fail_allocation;
void *ohmd_allocfn(ohmd_context *ctx, const char *message, size_t n)
{
    (void)ctx; (void)message;
    if (++allocation_count == fail_allocation) return NULL;
    return calloc(1, n);
}
static void outstanding_frame_lifetime(void)
{
    struct rift_sensor_uvc_stream s = {0};
    s.alt_setting = 1; s.frame_size = 16;
    allocation_count = fail_allocation = 0;
    CHECK(rift_sensor_uvc_stream_start(&s, 5, NULL, NULL) == 0);
    CHECK(rift_sensor_uvc_stream_stop(&s) == 0);
    CHECK(s.alloced_frames != NULL && s.n_alloced_frames == 5);
    /* An analyser may still access and return this frame after transfers stop. */
    s.frames_lock = (void *)1;
    ohmd_video_frame *frame = s.free_frames[--s.n_free_frames];
    frame->data[0] = 42;
    rift_sensor_uvc_stream_release_frame(frame, &s);
    CHECK(s.n_free_frames == 5);
    CHECK(rift_sensor_uvc_stream_clear(&s) == 0);
    CHECK(s.alloced_frames == NULL && s.n_alloced_frames == 0);
}

static void startup_allocation_failures(void)
{
    for (int fail = 1; fail <= 12; fail++) {
        struct rift_sensor_uvc_stream s = {0};
        s.alt_setting = 1; s.frame_size = 16;
        allocation_count = 0; fail_allocation = fail;
        CHECK(rift_sensor_uvc_stream_start(&s, 5, NULL, NULL) == -ENOMEM);
        CHECK(s.start_status.error_kind == OHMD_CV1TOUCH_STREAM_ERROR_OTHER);
        CHECK(s.start_status.error_code == -ENOMEM && s.start_status.os_error == ENOMEM);
        CHECK(!s.video_running && s.active_transfers == 0);
        CHECK(rift_sensor_uvc_stream_clear(&s) == 0);
        CHECK(!s.alloced_frames && !s.free_frames);
        CHECK(s.n_alloced_frames == 0 && s.active_transfers == 0);
    }
}

static void startup_error_diagnostics(void)
{
	struct rift_sensor_uvc_stream s = {0};
	s.alt_setting = 1;
	alt_result = LIBUSB_ERROR_OTHER;
	/* Quiet and logging-enabled system libraries get the same qualified
	 * result. Even EIO followed by logging's ENOSPC must never claim proof. */
	for (int logging = 0; logging < 2; logging++) {
		alt_logging = logging;
		alt_errno = ENOSPC;
		CHECK(rift_sensor_uvc_stream_start(&s, 5, NULL, NULL) == LIBUSB_ERROR_OTHER);
		CHECK(s.start_status.error_kind == OHMD_CV1TOUCH_STREAM_ERROR_LIKELY_USB_BANDWIDTH);
		CHECK(s.start_status.error_code == LIBUSB_ERROR_OTHER && s.start_status.os_error == ENOSPC);
		CHECK(s.start_status.alt_setting == 0);
		alt_errno = EIO;
		CHECK(rift_sensor_uvc_stream_start(&s, 5, NULL, NULL) == LIBUSB_ERROR_OTHER);
		CHECK(s.start_status.error_kind == (logging ? OHMD_CV1TOUCH_STREAM_ERROR_LIKELY_USB_BANDWIDTH : OHMD_CV1TOUCH_STREAM_ERROR_OTHER));
		CHECK(s.start_status.os_error == (logging ? ENOSPC : EIO));
	}
	alt_logging = false;
	alt_errno = EACCES;
	CHECK(rift_sensor_uvc_stream_start(&s, 5, NULL, NULL) == LIBUSB_ERROR_OTHER);
	CHECK(s.start_status.error_kind == OHMD_CV1TOUCH_STREAM_ERROR_OTHER);
	CHECK(s.start_status.os_error == EACCES);
	/* An error which supplies no errno must not inherit a preceding ENOSPC. */
	errno = ENOSPC;
	alt_errno = -1;
	CHECK(rift_sensor_uvc_stream_start(&s, 5, NULL, NULL) == LIBUSB_ERROR_OTHER);
	CHECK(s.start_status.error_kind == OHMD_CV1TOUCH_STREAM_ERROR_OTHER);
	CHECK(s.start_status.os_error == 0);
	alt_result = alt_errno = 0;
}

static void startup_submission_failures(void)
{
	/* usbfs can reject the first iso URB for bandwidth after the alternate
	 * setting succeeded. Exercise the real start/stop path, not the helper. */
	for (int attempt = 0; attempt < 3; attempt++) {
		const int errors[] = {ENOSPC, EACCES, 0};
		struct rift_sensor_uvc_stream s = {0};
		s.alt_setting = 1; s.frame_size = 16;
		s.num_transfers = 1;
		s.transfer = calloc(1, sizeof *s.transfer);
		CHECK(s.transfer != NULL);
		s.transfer[0] = libusb_alloc_transfer(0);
		CHECK(s.transfer[0] != NULL);
		allocation_count = fail_allocation = 0;
		submit_ok = false;
		submit_errno = errors[attempt];
		CHECK(rift_sensor_uvc_stream_start(&s, 5, NULL, NULL) == LIBUSB_ERROR_IO);
		CHECK(s.start_status.error_kind == (attempt == 0
			? OHMD_CV1TOUCH_STREAM_ERROR_LIKELY_USB_BANDWIDTH
			: OHMD_CV1TOUCH_STREAM_ERROR_OTHER));
		CHECK(s.start_status.error_code == LIBUSB_ERROR_IO);
		CHECK(s.start_status.os_error == errors[attempt] && s.start_status.alt_setting == 0);
		CHECK(!s.video_running && s.active_transfers == 0);
		CHECK(rift_sensor_uvc_stream_clear(&s) == 0);
		CHECK(!s.alloced_frames && !s.free_frames && !s.transfer);
	}
	submit_errno = 0;
	submit_ok = true;
}

int main(void)
{
	alarm(5);
	struct libusb_transfer *parked[6];
	stream.video_running = true;
	stream.active_transfers = 6;
	stream.parked = parked;
	for (int i = 0; i < 6; i++) {
		transfers[i].user_data = &stream;
		transfers[i].status = LIBUSB_TRANSFER_COMPLETED;
	}
	pthread_t worker;
	CHECK(pthread_create(&worker, NULL, complete_transfers, NULL) == 0);
	while (atomic_load(&submissions) < 6)
		ohmd_sleep(0.0005);
	/* An earlier sensor is torn down while its peer owns the callback lock.
	 * This also models cleanup after an individual sensor fails startup. */
	struct rift_sensor_uvc_stream earlier = {0};
	earlier.video_running = true;
	CHECK(rift_sensor_uvc_stream_stop(&earlier) == 0);
	CHECK(pthread_join(worker, NULL) == 0);
	CHECK(stream.video_running && stream.active_transfers == 0);
	CHECK(stream.n_parked == 6);

	/* Persistent failure attempts only once per maintenance interval. */
	rift_sensor_uvc_stream_retry(&stream, 1);
	CHECK(atomic_load(&submissions) == 7);
	rift_sensor_uvc_stream_retry(&stream, 2);
	CHECK(atomic_load(&submissions) == 7);
	CHECK(stream.n_parked == 6 && stream.active_transfers == 0);
	/* Once the transport recovers, all parked transfers resume without a
	 * callback, a new frame, or a daemon/device reopen. */
	submit_ok = true;
	rift_sensor_uvc_stream_retry(&stream, 10000001);
	CHECK(atomic_load(&submissions) == 13);
	CHECK(stream.n_parked == 0 && stream.active_transfers == 6);

	/* Two-phase stop first publishes flags without waiting on USB locks. */
	CHECK(pthread_mutex_lock(&waiters) == 0);
	rift_sensor_uvc_stream_request_stop(&earlier);
	rift_sensor_uvc_stream_request_stop(&stream);
	CHECK(!earlier.video_running && !stream.video_running);
	for (int i = 0; i < 6; i++)
		iso_transfer_cb(&transfers[i]);
	CHECK(pthread_mutex_unlock(&waiters) == 0);
	CHECK(rift_sensor_uvc_stream_stop(&stream) == 0);
	rift_sensor_uvc_stream_retry(&stream, 20000001);
	CHECK(atomic_load(&submissions) == 13);
	CHECK(stream.active_transfers == 0);
	outstanding_frame_lifetime();
	startup_allocation_failures();
	startup_error_diagnostics();
	startup_submission_failures();
	alarm(0);
	puts("test_uvc_shutdown: ok");
	return 0;
}
