// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

/* Exercise the real callback/stop functions with libusb's callback lock contract.
 * No USB devices are opened. Failed resubmits park five transfers; the sixth retries
 * while holding the event-waiter lock, exactly as a libusb completion callback does. */
#include <pthread.h>
#include <stdatomic.h>
#include <time.h>
#include <unistd.h>

#include "sensor/uvc.c"
#include "test_util.h"

static pthread_mutex_t waiters = PTHREAD_MUTEX_INITIALIZER;
static atomic_int submissions;
static struct rift_sensor_uvc_stream stream;
static struct libusb_transfer transfers[6];

int LIBUSB_CALL libusb_submit_transfer(struct libusb_transfer *transfer)
{
	(void)transfer;
	atomic_fetch_add(&submissions, 1);
	return LIBUSB_ERROR_IO;
}

int LIBUSB_CALL libusb_set_interface_alt_setting(libusb_device_handle *dev, int interface, int alt)
{
	(void)dev;
	CHECK(interface == 1 && (alt == 0 || alt == 1));
	return 0;
}

void LIBUSB_CALL libusb_lock_event_waiters(libusb_context *ctx)
{
	(void)ctx;
	struct timespec deadline;
	CHECK(clock_gettime(CLOCK_REALTIME, &deadline) == 0);
	deadline.tv_sec += 2;
	/* Baseline deadlocks here: stop has not cleared video_running, and the
	 * retrying callback cannot release this lock until that flag is cleared. */
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
        CHECK(!s.video_running && s.active_transfers == 0);
        CHECK(rift_sensor_uvc_stream_clear(&s) == 0);
        CHECK(!s.alloced_frames && !s.free_frames);
        CHECK(s.n_alloced_frames == 0 && s.active_transfers == 0);
    }
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
	CHECK(rift_sensor_uvc_stream_stop(&stream) == 0);
	CHECK(pthread_join(worker, NULL) == 0);
	CHECK(!stream.video_running && stream.active_transfers == 0);
	CHECK(stream.n_parked == 5);
	outstanding_frame_lifetime();
	startup_allocation_failures();
	alarm(0);
	puts("test_uvc_shutdown: ok");
	return 0;
}
