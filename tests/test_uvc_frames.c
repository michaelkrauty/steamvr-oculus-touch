// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* Real UVC assembly, pooled-buffer reuse and blob extraction; no device is opened. */
#include <fcntl.h>
#include <unistd.h>
#include "sensor/uvc.c"
#include "rift-sensor-blobwatch.c"
#include "rift-sensor-flicker.c"
#include "test_util.h"

#define WIDTH 128
#define HEIGHT 128
#define FRAME_BYTES (WIDTH * HEIGHT)
static rift_sensor_uvc_stream stream;
static unsigned char image[FRAME_BYTES];
static int callbacks, last_bytes, last_blobs, packet_number, delivered_packet;
static uint64_t submitted_ns, delivered_ns, first_pixel_ns;
static ohmd_video_frame *held_frame;
static bool retain_frame;

void *ohmd_allocfn(ohmd_context *ctx, const char *message, size_t size)
{ (void)ctx; (void)message; return calloc(1, size); }
void ohmd_lock_mutex(ohmd_mutex *mutex) { CHECK(mutex == (void *)1); }
void ohmd_unlock_mutex(ohmd_mutex *mutex) { CHECK(mutex == (void *)1); }
void ohmd_destroy_mutex(ohmd_mutex *mutex) { CHECK(mutex == (void *)1); }
int LIBUSB_CALL libusb_set_interface_alt_setting(libusb_device_handle *dev, int iface, int alt)
{ (void)dev; CHECK(iface == 1 && (alt == 0 || alt == 2)); return 0; }
int LIBUSB_CALL libusb_submit_transfer(struct libusb_transfer *transfer)
{ CHECK(transfer != NULL); return 0; }
void LIBUSB_CALL libusb_lock_event_waiters(libusb_context *ctx) { (void)ctx; }
void LIBUSB_CALL libusb_unlock_event_waiters(libusb_context *ctx) { (void)ctx; }
int LIBUSB_CALL libusb_wait_for_event(libusb_context *ctx, struct timeval *timeout)
{ (void)ctx; (void)timeout; CHECK(false); return -1; }

/* The engine's log goes to stdout in this unity build. A pipe holds these few lines. */
static int log_pipe[2] = {-1, -1}, saved_stdout = -1;
static char log_text[8192];

static void capture_log(void)
{
    fflush(stdout);
    CHECK(pipe(log_pipe) == 0);
    CHECK(fcntl(log_pipe[0], F_SETFL, O_NONBLOCK) == 0);
    saved_stdout = dup(1);
    CHECK(saved_stdout >= 0 && dup2(log_pipe[1], 1) == 1);
}

/* Returns everything logged since capture_log, and keeps capturing. */
static const char *captured_log(void)
{
    fflush(stdout);
    size_t used = strlen(log_text);
    ssize_t n;
    while ((n = read(log_pipe[0], log_text + used, sizeof log_text - 1 - used)) > 0)
        used += (size_t)n;
    log_text[used] = 0;
    return log_text;
}

static void end_capture(void)
{
    captured_log();
    CHECK(dup2(saved_stdout, 1) == 1);
    close(saved_stdout);
    close(log_pipe[0]);
    close(log_pipe[1]);
    fputs(log_text, stdout);
    log_text[0] = 0;
}

static int count_lines(const char *text, const char *needle)
{
    int n = 0;
    for (const char *at = text; (at = strstr(at, needle)) != NULL; at += strlen(needle))
        n++;
    return n;
}

static uint64_t clock_ns(void)
{
    struct timespec now;
    CHECK(clock_gettime(CLOCK_MONOTONIC, &now) == 0);
    return (uint64_t)now.tv_sec * 1000000000 + now.tv_nsec;
}

static int count_blobs(unsigned char *data)
{
    blobwatch *bw = blobwatch_new(BLOB_THRESHOLD_CV1);
    CHECK(bw != NULL);
    blobservation *ob = NULL;
    blobwatch_process(bw, data, WIDTH, HEIGHT, 0, NULL, 0, &ob);
    CHECK(ob != NULL);
    int count = ob->num_blobs;
    blobwatch_release_observation(bw, ob);
    blobwatch_free(bw);
    return count;
}

static void captured(rift_sensor_uvc_stream *s, ohmd_video_frame *frame, void *user)
{
    (void)user;
    CHECK(s == &stream && frame->owner == s);
    delivered_ns = clock_ns();
    delivered_packet = packet_number;
    callbacks++;
    last_bytes = frame->data_size;
    last_blobs = frame->format == OHMD_VIDEO_FRAME_FORMAT_GRAY8 ? count_blobs(frame->data) : 0;
    if (retain_frame) held_frame = frame;
    else rift_sensor_uvc_stream_release_frame(frame, s);
}

static void feed(unsigned fid, uint32_t pts, unsigned flags, const unsigned char *data, size_t size)
{
    unsigned char packet[FRAME_BYTES + 12] = {12, 0};
    CHECK(size <= FRAME_BYTES);
    packet[1] = 0x8c | fid | flags;
    for (int i = 0; i < 4; i++) packet[2 + i] = pts >> (8 * i);
    if (size) memcpy(packet + 12, data, size);
    packet_number++;
    submitted_ns = clock_ns();
    process_payload(&stream, packet, size + 12);
}

static void full(unsigned fid, uint32_t pts)
{
    int before = callbacks;
    for (int i = 0; i < FRAME_BYTES; i += 4096) {
        feed(fid, pts, 0, image + i, 4096);
        CHECK(callbacks == before + (i + 4096 == FRAME_BYTES));
    }
    CHECK(delivered_packet == packet_number);
    CHECK(last_bytes == FRAME_BYTES);
    CHECK(stream.n_free_frames == 1 && stream.cur_frame == NULL);
}

/* A complete raw image, then the separate 12-byte end header the sensor sends.
 * Exact size delivers on the completing payload packet. The end header adds nothing. */
static void full_eof(unsigned fid, uint32_t pts)
{
    int before = callbacks;
    for (int i = 0; i < FRAME_BYTES; i += 4096) {
        feed(fid, pts, 0, image + i, 4096);
        CHECK(callbacks == before + (i + 4096 == FRAME_BYTES));
    }
    CHECK(delivered_packet == packet_number);
    CHECK(last_bytes == FRAME_BYTES);
    CHECK(stream.n_free_frames == 1 && stream.cur_frame == NULL);
    uint32_t bad = stream.bad_packets, discards = stream.n_discards;
    feed(fid, pts, 2, NULL, 0);
    CHECK(callbacks == before + 1 && delivered_packet != packet_number);
    CHECK(stream.bad_packets == bad && stream.n_discards == discards);
}

static void begin(void)
{
    memset(&stream, 0, sizeof stream);
    memset(image, 0, sizeof image);
    callbacks = packet_number = delivered_packet = 0;
    held_frame = NULL; retain_frame = false;
    stream.frame_size = FRAME_BYTES;
    stream.stride = stream.width = WIDTH;
    stream.height = HEIGHT;
    stream.format = OHMD_VIDEO_FRAME_FORMAT_GRAY8;
    stream.alt_setting = 2;
    stream.frames_lock = (void *)1;
    stream.skip_frame = true;
    CHECK(rift_sensor_uvc_stream_start(&stream, 1, captured, NULL) == 0);
}

static void start(void)
{
    begin();
    /* Observe an initial identity before the first trustworthy FID transition. */
    feed(0, 0, 0, NULL, 0);
}

static void finish(void)
{
    CHECK(stream.n_free_frames + (stream.cur_frame != NULL) == stream.n_alloced_frames);
    CHECK(rift_sensor_uvc_stream_stop(&stream) == 0);
    CHECK(rift_sensor_uvc_stream_clear(&stream) == 0);
    CHECK(!stream.free_frames && !stream.alloced_frames);
}

static void stale_tail(void)
{
    start();
    for (int y = 100; y < 106; y++) memset(image + y * WIDTH + 20, 200, 6);
    full(1, 1000);
    CHECK(last_blobs == 1);
    memset(image, 0, sizeof image);
    feed(0, 2000, 2, image, FRAME_BYTES / 2);
    int retained = count_blobs(stream.alloced_frames[0]->data);
    printf("stale tail: old spot still produces %d blob; short callbacks=%d, bytes=%d\n",
        retained, callbacks - 1, last_bytes);
    CHECK(retained == 1); /* Returning the real pooled buffer does not clear pixels. */
    CHECK(callbacks == 1); /* The short image must never be analysed. */
    full(1, 3000);
    CHECK(last_blobs == 0 && callbacks == 2);
    finish();
}

static void nearly_complete(void)
{
    start();
    feed(1, 1000, 2, image, FRAME_BYTES - 1);
    CHECK(callbacks == 0);
    full(0, 2000);
    finish();
}

static void header_only(void)
{
    start();
    feed(1, 1000, 0, NULL, 0);
    /* No pixels in the first frame: its header still advances the identity. */
    full(0, 2000);
    feed(0, 2000, 2, NULL, 0); /* A trailing header-only EOF is harmless. */
    CHECK(callbacks == 1);
    full(1, 3000);
    finish();
}

static void fid_changed_without_pts(void)
{
    start();
    feed(1, 1000, 0, image, 1024);
    feed(0, 1000, 0, image, FRAME_BYTES / 2);
    feed(0, 1000, 0, image, FRAME_BYTES / 2);
    CHECK(callbacks == 0);
    full(0, 2000);
    finish();
}

static void pts_changed_without_fid(void)
{
    start();
    feed(1, 1000, 0, image, FRAME_BYTES / 2);
    feed(1, 2000, 0, image, FRAME_BYTES / 2);
    CHECK(callbacks == 0);
    full(0, 3000);
    finish();
}

static void malformed_header(void)
{
    start();
    feed(1, 1000, 0, image, FRAME_BYTES / 2);
    unsigned char bad[12] = {3};
    process_payload(&stream, bad, sizeof bad);
    feed(1, 1000, 0, image, FRAME_BYTES / 2);
    CHECK(callbacks == 0);
    full(0, 2000);
    finish();
}

static void error_flag(void)
{
    start();
    feed(1, 1000, 0, image, FRAME_BYTES / 2);
    feed(1, 1000, 0x40, NULL, 0);
    feed(1, 1000, 0, image, FRAME_BYTES / 2);
    CHECK(callbacks == 0);
    full(0, 2000);
    finish();
}

static void opening_error(bool jpeg)
{
    start();
    full(1, 1000);
    if (jpeg) stream.format = OHMD_VIDEO_FRAME_FORMAT_JPEG;
    unsigned char encoded[] = {0xff, 0xd8, 0xff, 0xda, 0, 2, 0xff, 0xd9};
    feed(0, 2000, 0x40, NULL, 0); /* ERR belongs to the new image, not the delivered one. */
    feed(0, 2000, 2, jpeg ? encoded : image, jpeg ? sizeof encoded : FRAME_BYTES);
    CHECK(callbacks == 1 && stream.discards[UVC_DISCARD_CAMERA_ERROR] == 1);
    if (jpeg) {
        feed(1, 3000, 0, encoded, sizeof encoded);
        CHECK(callbacks == 2 && last_bytes == sizeof encoded);
    } else full(1, 3000);
    finish();
}

static void bad_traffic(bool after_rejection)
{
    start();
    stream.sensor_id = 3;
    capture_log();
    if (after_rejection) feed(1, 1000, 2, image, 100);
    unsigned char bad[12] = {3};
    for (int i = 0; i < 100; i++) process_payload(&stream, bad, sizeof bad);
    CHECK(stream.n_discards == (unsigned)after_rejection);
    CHECK(count_lines(captured_log(), "Sensor 3 video:") == 0);
    stream.discard_window_start = uvc_now(&stream) - UVC_DISCARD_REPORT_NS;
    process_payload(&stream, bad, sizeof bad);
    CHECK(count_lines(captured_log(), "Sensor 3 video:") == 1);
    CHECK(stream.n_discards == 0);
    if (after_rejection) CHECK(strstr(captured_log(), "discarded 1 damaged image"));
    for (int i = 0; i < 100; i++) process_payload(&stream, bad, sizeof bad);
    CHECK(count_lines(captured_log(), "Sensor 3 video:") == 1);
    stream.discard_window_start = uvc_now(&stream) - UVC_DISCARD_REPORT_NS;
    process_payload(&stream, bad, sizeof bad);
    CHECK(count_lines(captured_log(), "Sensor 3 video:") == 2 && stream.n_discards == 0);
    end_capture();
    finish();
}

/* Repeated raw overruns still report when no next boundary ever arrives. */
static void raw_overrun_report(void)
{
    start();
    stream.sensor_id = 3;
    capture_log();
    full(1, 1000);
    feed(1, 1000, 0, image, 1);
    CHECK(stream.reject_next_frame && stream.n_discards == 1);
    for (int i = 0; i < 100; i++) feed(1, 1000, 0, image, 1);
    CHECK(stream.n_discards == 1 && stream.discards[UVC_DISCARD_TOO_LONG] == 1);
    CHECK(count_lines(captured_log(), "Sensor 3 video:") == 0);
    stream.discard_window_start = uvc_now(&stream) - UVC_DISCARD_REPORT_NS;
    feed(1, 1000, 0, image, 1);
    CHECK(count_lines(captured_log(), "Sensor 3 video:") == 1);
    CHECK(strstr(captured_log(), "discarded 1 damaged image in the last 10 s (1 too long)"));
    CHECK(strstr(captured_log(), "101 more damaged packets while skipping images"));
    CHECK(stream.n_discards == 0 && stream.bad_packets == 0);
    for (int i = 0; i < 100; i++) feed(1, 1000, 0, image, 1);
    CHECK(count_lines(captured_log(), "Sensor 3 video:") == 1);
    CHECK(stream.n_discards == 0 && stream.bad_packets == 100 && callbacks == 1);
    feed(0, 2000, 0, image, FRAME_BYTES); /* Only the following image is tainted. */
    CHECK(callbacks == 1);
    full(1, 3000);
    CHECK(callbacks == 2 && count_lines(captured_log(), "Sensor 3 video:") == 1);
    end_capture();
    finish();
}

static void quality_counts(void)
{
    uint64_t rejected = UINT64_C(4294967295), other_rejected = 0;
    start();
    stream.frames_dropped = &rejected;
    feed(1, 1000, 2, image, 100);
    CHECK(rejected == UINT64_C(4294967296));
    for (int i = 0; i < 5; i++) feed(1, 1000, 0x40, NULL, 0);
    CHECK(rejected == UINT64_C(4294967296));
    full(0, 2000);
    for (int i = 0; i < 100; i++) feed(0, 2000, 0, image, 1);
    CHECK(rejected == UINT64_C(4294967297));
    stream.discard_window_start = uvc_now(&stream) - UVC_DISCARD_REPORT_NS;
    feed(0, 2000, 0, image, 1); /* Reporting clears only its window counters. */
    CHECK(rejected == UINT64_C(4294967297) && stream.n_discards == 0);
    feed(1, 3000, 0, image, FRAME_BYTES);
    CHECK(callbacks == 1 && rejected == UINT64_C(4294967297));
    full(0, 4000);
    struct rift_sensor_uvc_stream other = { .frames_dropped = &other_rejected };
    count_discard(&other, UVC_DISCARD_INCOMPLETE, true);
    CHECK(other_rejected == 1 && rejected == UINT64_C(4294967297));
    CHECK(rift_sensor_uvc_stream_stop(&stream) == 0);
    CHECK(rift_sensor_uvc_stream_clear(&stream) == 0);
    stream.frames_lock = (void *)1; /* Recreate the fixture lock after clear. */
    CHECK(rift_sensor_uvc_stream_start(&stream, 1, captured, NULL) == 0);
    CHECK(stream.frames_dropped == &rejected && rejected == UINT64_C(4294967297));
    finish();
}

static void truncated_header(void)
{
    const unsigned char header[12] = {12, 0x8d, 0xe8, 0x03};
    for (size_t n = 1; n < sizeof header; n++) {
        start();
        feed(1, 1000, 0, image, FRAME_BYTES / 2);
        unsigned char *bad = malloc(n);
        CHECK(bad != NULL);
        memcpy(bad, header, n); /* A plausible prefix, missing only its final bytes. */
        process_payload(&stream, bad, n);
        free(bad);
        CHECK(stream.discards[UVC_DISCARD_HEADER] == 1 && stream.n_discards == 1);
        feed(1, 1000, 0, image, FRAME_BYTES / 2);
        CHECK(callbacks == 0);
        full(0, 2000);
        finish();
    }
}


static void initial_boundary(void)
{
    start();
    /* The first observed image might already be missing its opening packets. */
    feed(0, 0, 0, image, FRAME_BYTES);
    CHECK(callbacks == 0);
    full(1, 1000);
    finish();
}

/* The first packet is left over from an earlier stream (frame ID 0, PTS A). The first
 * image reuses frame ID 0 with PTS B and is delivered; the next image, frame ID 1 and
 * PTS C, follows. Each image ends with a separate header-only end packet. */
static void startup_same_fid(void)
{
    begin();
    uint64_t dropped = 0;
    stream.frames_dropped = &dropped;
    feed(0, 1000, 0, image, FRAME_BYTES);
    CHECK(callbacks == 0 && stream.bad_packets == 0 && stream.n_discards == 0 && dropped == 0);
    for (int y = 100; y < 106; y++) memset(image + y * WIDTH + 20, 200, 6);
    full_eof(0, 2000);
    CHECK(callbacks == 1 && last_blobs == 1);
    CHECK(stream.bad_packets == 0 && stream.n_discards == 0 && dropped == 0);
    memset(image, 0, sizeof image);
    full_eof(1, 3000);
    CHECK(callbacks == 2 && last_blobs == 0 && last_bytes == FRAME_BYTES);
    CHECK(stream.bad_packets == 0 && stream.n_discards == 0 && dropped == 0);

    /* The same struct starts again with the relaxed rule armed, not left closed. */
    CHECK(rift_sensor_uvc_stream_stop(&stream) == 0);
    CHECK(rift_sensor_uvc_stream_clear(&stream) == 0);
    callbacks = packet_number = delivered_packet = 0;
    stream.frames_lock = (void *)1;
    CHECK(rift_sensor_uvc_stream_start(&stream, 1, captured, NULL) == 0);
    CHECK(stream.frames_dropped == &dropped && !stream.have_boundary);
    feed(0, 4000, 0, image, FRAME_BYTES);
    full_eof(0, 5000);
    CHECK(callbacks == 1 && stream.bad_packets == 0 && stream.n_discards == 0 && dropped == 0);
    finish();
}

/* Opening frame ID 1 and a first image with frame ID 0: both fields change, which
 * already started an image. That image is still delivered and nothing is counted. */
static void startup_new_fid(void)
{
    begin();
    uint64_t dropped = 0;
    stream.frames_dropped = &dropped;
    feed(1, 1000, 0, image, FRAME_BYTES);
    CHECK(callbacks == 0 && dropped == 0);
    for (int y = 100; y < 106; y++) memset(image + y * WIDTH + 20, 200, 6);
    full_eof(0, 2000);
    CHECK(callbacks == 1 && last_blobs == 1);
    CHECK(stream.bad_packets == 0 && stream.n_discards == 0 && dropped == 0);
    memset(image, 0, sizeof image);
    full_eof(1, 3000);
    CHECK(callbacks == 2 && last_blobs == 0 && last_bytes == FRAME_BYTES);
    CHECK(stream.bad_packets == 0 && stream.n_discards == 0 && dropped == 0);
    finish();
}

/* Packets that repeat the opening frame ID and timestamp are the leftover. A short
 * EOF and a payload longer than one image are still not delivered and not counted. */
static void startup_matching_tail(void)
{
    begin();
    uint64_t dropped = 0;
    stream.frames_dropped = &dropped;
    feed(0, 1000, 0, image, 4096);
    feed(0, 1000, 2, image, FRAME_BYTES / 2);
    CHECK(callbacks == 0 && stream.bad_packets == 0 && stream.n_discards == 0 && dropped == 0);
    unsigned char *packet = calloc(1, FRAME_BYTES + 1 + 12);
    CHECK(packet != NULL);
    packet[0] = 12;
    packet[1] = 0x8c;
    for (int i = 0; i < 4; i++) packet[2 + i] = (unsigned char)(1000 >> (8 * i));
    packet_number++;
    process_payload(&stream, packet, FRAME_BYTES + 1 + 12);
    free(packet);
    CHECK(callbacks == 0 && stream.bad_packets == 0 && stream.n_discards == 0 && dropped == 0);
    CHECK(stream.discards[UVC_DISCARD_TOO_LONG] == 0);
    CHECK(stream.discards[UVC_DISCARD_INCOMPLETE] == 0);
    full_eof(1, 2000);
    CHECK(callbacks == 1 && stream.bad_packets == 0 && stream.n_discards == 0 && dropped == 0);
    finish();
}

/* Before the first boundary a frame-ID change with the latched PTS is one damaged
 * packet, not a new image. The image that then arrives with a new PTS is kept. */
static void startup_fid_only(void)
{
    begin();
    uint64_t dropped = 0;
    stream.frames_dropped = &dropped;
    feed(0, 1000, 0, image, 4096);
    feed(1, 1000, 0, image, 4096);
    CHECK(callbacks == 0 && stream.bad_packets == 1 && stream.n_discards == 0 && dropped == 0);
    CHECK(stream.discards[UVC_DISCARD_SEQUENCE] == 0);
    full_eof(0, 2000);
    CHECK(callbacks == 1 && stream.bad_packets == 1 && stream.n_discards == 0 && dropped == 0);
    full_eof(1, 3000);
    CHECK(callbacks == 2 && stream.bad_packets == 1 && stream.n_discards == 0 && dropped == 0);
    finish();
}

/* The opening header carries no presentation time, so a same frame ID with a new
 * timestamp is not an image start. Each of its packets, including the end header,
 * counts as a damaged packet and nothing is delivered or dropped. The next frame
 * ID, with its own timestamp, is kept. Removing the have_pts gate delivers the
 * same-frame-ID image instead. */
static void startup_no_pts(void)
{
    begin();
    uint64_t dropped = 0;
    stream.frames_dropped = &dropped;
    unsigned char opening[12 + 4096] = {12, 0x88};
    packet_number++;
    process_payload(&stream, opening, sizeof opening);
    CHECK(callbacks == 0 && stream.bad_packets == 0 && stream.n_discards == 0 && dropped == 0);
    CHECK(stream.have_frame_id && !stream.have_pts && !stream.have_boundary && stream.skip_frame);

    uint32_t packets = 0;
    for (int i = 0; i < FRAME_BYTES; i += 4096) {
        feed(0, 1000, 0, image + i, 4096);
        packets++;
        CHECK(callbacks == 0);
    }
    feed(0, 1000, 2, NULL, 0);
    packets++;
    CHECK(callbacks == 0 && stream.bad_packets == packets && stream.n_discards == 0 && dropped == 0);
    CHECK(stream.discards[UVC_DISCARD_SEQUENCE] == 0);
    CHECK(stream.discards[UVC_DISCARD_INCOMPLETE] == 0);

    full_eof(1, 2000);
    CHECK(callbacks == 1 && last_bytes == FRAME_BYTES);
    CHECK(stream.bad_packets == packets && stream.n_discards == 0 && dropped == 0);
    finish();
}

static void overflow(void)
{
    start();
    feed(1, 1000, 0, image, FRAME_BYTES - 1);
    feed(1, 1000, 2, image, 2);
    CHECK(callbacks == 0 && stream.cur_frame != NULL);
    CHECK(stream.cur_frame->data_size == 0 && stream.n_free_frames == 0);
    full(0, 2000);
    finish();
}

static void trailing_data(void)
{
    start();
    full(1, 1000);
    feed(1, 1000, 2, image, 1);
    CHECK(callbacks == 1);
    feed(0, 2000, 2, image, FRAME_BYTES);
    CHECK(callbacks == 1); /* The boundary following an overrun is tainted too. */
    full(1, 3000);
    finish();
}

static void buffer_retention(void)
{
    start();
    retain_frame = true;
    feed(1, 1000, 0, image, FRAME_BYTES);
    CHECK(callbacks == 1 && held_frame != NULL && stream.n_free_frames == 0);
    CHECK(stream.cur_frame == NULL);
    feed(0, 2000, 0, image, 4096); /* No buffer: discard this whole candidate. */
    rift_sensor_uvc_stream_release_frame(held_frame, &stream);
    held_frame = NULL;
    retain_frame = false;
    feed(0, 2000, 2, image, FRAME_BYTES - 4096);
    CHECK(callbacks == 1 && stream.n_free_frames == 1);
    full(1, 3000);
    feed(0, 4000, 0, image, 4096);
    for (int i = 0; i < 20; i++) feed(0, 4000, 0x40, NULL, 0);
    CHECK(stream.cur_frame != NULL && stream.n_free_frames == 0);
    full(1, 5000);
    feed(0, 6000, 2, image, 100);
    CHECK(stream.cur_frame != NULL && stream.n_free_frames == 0);
    finish(); /* Clear also reclaims the assembler-owned buffer on a drop. */
}

static void packet_error(void)
{
    start();
    feed(1, 1000, 0, image, FRAME_BYTES / 2);
    struct libusb_transfer *transfer = libusb_alloc_transfer(1);
    CHECK(transfer != NULL);
    transfer->user_data = &stream;
    transfer->status = LIBUSB_TRANSFER_COMPLETED;
    transfer->num_iso_packets = 1;
    transfer->buffer = image;
    transfer->iso_packet_desc[0].status = LIBUSB_TRANSFER_ERROR;
    transfer->iso_packet_desc[0].actual_length = 0;
    stream.active_transfers = 1;
    iso_transfer_cb(transfer);
    feed(1, 1000, 0, image, FRAME_BYTES / 2);
    CHECK(callbacks == 0);
    transfer->status = LIBUSB_TRANSFER_CANCELLED;
    iso_transfer_cb(transfer);
    libusb_free_transfer(transfer);
    CHECK(stream.active_transfers == 0);
    full(0, 2000);
    finish();
}

static void timestamp(void)
{
    start();
    feed(1, UINT32_MAX, 0, NULL, 0);
    CHECK(stream.cur_frame == NULL);
    feed(1, UINT32_MAX, 0, image, FRAME_BYTES / 2);
    first_pixel_ns = stream.cur_frame->start_ts;
    CHECK(first_pixel_ns >= submitted_ns && first_pixel_ns <= clock_ns());
    /* Header-only continuation and a continuation without PTS keep the stamp. */
    feed(1, UINT32_MAX, 0, NULL, 0);
    unsigned char packet[FRAME_BYTES / 2 + 12] = {12, 0x89};
    process_payload(&stream, packet, sizeof packet);
    CHECK(callbacks == 1 && stream.alloced_frames[0]->start_ts == first_pixel_ns);
    full(0, 0); /* Timestamp wrap is a different PTS, not an error. */
    finish();
}

static void jpeg_assembly(void)
{
    start();
    stream.format = OHMD_VIDEO_FRAME_FORMAT_JPEG;
    unsigned char jpeg[] = {0xff, 0xd8, 0xff, 0xda, 0, 2, 0xff, 0xd9};
    unsigned char padded[] = {0xff, 0xd8, 0xff, 0xda, 0, 2, 0xff, 0xd9, 0, 0, 0, 0};
    feed(1, 1000, 2, jpeg, 4); /* EOF delivers; the decoder checks whether data ran out. */
    CHECK(callbacks == 1 && last_bytes == 4);
    feed(0, 2000, 0, jpeg, sizeof jpeg - 1);
    CHECK(callbacks == 1);
    feed(0, 2000, 0, jpeg + sizeof jpeg - 1, 1); /* Split EOI, immediate delivery without EOF. */
    CHECK(callbacks == 2 && last_bytes == sizeof jpeg);
    feed(0, 2000, 2, padded + sizeof jpeg, 4); /* Padding after a delivered JPEG cannot change it. */
    feed(1, 3000, 2, padded, sizeof padded); /* EOI need not end the payload. */
    CHECK(callbacks == 3 && last_bytes == sizeof padded);
    feed(0, 4000, 0, image, FRAME_BYTES - 1);
    feed(0, 4000, 2, jpeg, 2); /* The buffer still bounds a JPEG. */
    CHECK(callbacks == 3 && stream.discards[UVC_DISCARD_TOO_LONG] == 1);
    feed(1, 5000, 2, jpeg, sizeof jpeg);
    CHECK(callbacks == 4);
    finish();
}

static void missing_pts(void)
{
    start();
    unsigned char packet[1024 + 12] = {12, 0x89};
    process_payload(&stream, packet, sizeof packet); /* FID transition without PTS. */
    feed(1, 1000, 2, image, FRAME_BYTES - 1024);
    CHECK(callbacks == 0);
    full(0, 2000);
    finish();
}

static void missing_header_end(void)
{
    start();
    feed(1, 1000, 0, image, FRAME_BYTES / 2);
    unsigned char packet[12] = {12, 0x0d, 0xe8, 0x03};
    process_payload(&stream, packet, sizeof packet); /* EOH must be set. */
    feed(1, 1000, 2, image, FRAME_BYTES / 2);
    CHECK(callbacks == 0);
    full(0, 2000);
    finish();
}

/* A stall loses an even number of whole images: the next has the delivered image's FID
 * but a new PTS. It and the image after it both arrive; a partial one still does not. */
static void same_fid_after_stall(void)
{
    start();
    full(1, 1000);
    full(1, 4000);
    full(0, 5000);
    CHECK(callbacks == 3 && stream.n_discards == 0);
    feed(0, 8000, 0, image, FRAME_BYTES / 2); /* the rest of this image was lost */
    full(1, 9000);
    CHECK(callbacks == 4 && stream.discards[UVC_DISCARD_INCOMPLETE] == 1);
    finish();
}

/* Damage in the idle gap after a delivered image never costs the next complete image. */
static void gap_damage(void)
{
    start();
    full(1, 1000);
    struct libusb_transfer *transfer = libusb_alloc_transfer(1);
    CHECK(transfer != NULL);
    transfer->user_data = &stream;
    transfer->status = LIBUSB_TRANSFER_COMPLETED;
    transfer->num_iso_packets = 1;
    transfer->buffer = image;
    transfer->iso_packet_desc[0].status = LIBUSB_TRANSFER_ERROR;
    stream.active_transfers = 1;
    iso_transfer_cb(transfer);
    full(0, 2000);

    transfer->status = LIBUSB_TRANSFER_ERROR;
    iso_transfer_cb(transfer);
    CHECK(stream.active_transfers == 0);
    full(1, 3000);

    unsigned char bad[12] = {3};
    process_payload(&stream, bad, sizeof bad);
    process_payload(&stream, bad, 5);
    feed(0, 3000, 0x40, NULL, 0);
    feed(0, 3000, 0, NULL, 0); /* A lone FID toggle in the gap */
    full(0, 4000);
    CHECK(callbacks == 4 && stream.n_discards == 0);
    libusb_free_transfer(transfer);
    finish();
}

/* Each discarded image counts once, by reason; the log summary is rate-limited. */
static void discard_summary(void)
{
    start();
    stream.sensor_id = 3;
    capture_log();
    feed(1, 1000, 2, image, 100); /* incomplete */
    feed(0, 2000, 0, image, 100);
    for (int i = 0; i < 5; i++) /* one image, five bad packets */
        feed(0, 2000, 0x40, NULL, 0);
    feed(1, 3000, 0, image, 100);
    process_payload(&stream, image, 3);
    feed(0, 4000, 0, image, FRAME_BYTES - 1);
    feed(0, 4000, 0, image, 2);
    feed(1, 5000, 0, image, 100);
    feed(1, 6000, 0, image, 100); /* new PTS without FID toggle mid-image */
    feed(0, 7000, 0, image, 100);
    stream.active_transfers = 1;
    struct libusb_transfer *transfer = libusb_alloc_transfer(0);
    CHECK(transfer != NULL);
    transfer->user_data = &stream;
    transfer->status = LIBUSB_TRANSFER_ERROR;
    iso_transfer_cb(transfer);
    libusb_free_transfer(transfer);
    full(1, 8000);
    feed(1, 8000, 0, image, 1); /* taints the next image */
    CHECK(stream.discards[UVC_DISCARD_INCOMPLETE] == 1);
    CHECK(stream.discards[UVC_DISCARD_CAMERA_ERROR] == 1);
    CHECK(stream.discards[UVC_DISCARD_HEADER] == 1);
    CHECK(stream.discards[UVC_DISCARD_TOO_LONG] == 2);
    CHECK(stream.discards[UVC_DISCARD_SEQUENCE] == 1);
    CHECK(stream.discards[UVC_DISCARD_TRANSFER] == 1);
    CHECK(stream.n_discards == 7);
    CHECK(count_lines(captured_log(), "video: discarded") == 0);

    /* Ten seconds later the next discard logs everything once. */
    stream.discard_window_start -= UVC_DISCARD_REPORT_NS;
    feed(0, 9000, 0, image, FRAME_BYTES); /* the tainted image */
    feed(1, 10000, 2, image, 100);
    CHECK(count_lines(captured_log(), "Sensor 3 video: discarded 8 damaged images in the last 10 s "
        "(2 incomplete, 2 too long, 1 with a damaged header, 1 marked faulty by the camera, "
        "1 hit by a USB transfer error, 1 out of sequence)") == 1);
    CHECK(stream.n_discards == 0 && stream.discards[UVC_DISCARD_INCOMPLETE] == 0);

    for (int i = 0; i < 50; i++)
        feed(i & 1, 11000 + i * 1000, 2, image, 100);
    CHECK(stream.n_discards == 50);
    CHECK(count_lines(captured_log(), "video: discarded") == 1);

    /* An episode that ended is reported by the next delivered image. */
    full(0, 70000);
    CHECK(count_lines(captured_log(), "video: discarded") == 1);
    stream.discard_window_start -= UVC_DISCARD_REPORT_NS;
    full(1, 71000);
    CHECK(count_lines(captured_log(), "Sensor 3 video: discarded 50 damaged images in the last ") == 1);
    CHECK(count_lines(captured_log(), "video: discarded") == 2 && stream.n_discards == 0);
    full(0, 72000);
    CHECK(count_lines(captured_log(), "video: discarded") == 2);
    end_capture();
    finish();
}

/* No free buffer: one warning when skipping starts and one when a buffer is back. */
static void starvation_log(void)
{
    start();
    capture_log();
    retain_frame = true;
    feed(1, 1000, 0, image, FRAME_BYTES);
    CHECK(callbacks == 1 && held_frame != NULL);
    for (int i = 0; i < 3; i++)
        feed(i & 1, 2000 + i * 1000, 0, image, FRAME_BYTES);
    CHECK(callbacks == 1 && stream.n_discards == 0);
    CHECK(count_lines(captured_log(), "No frame provided for pixel data. Skipping frames") == 1);
    CHECK(count_lines(captured_log(), "Got capture frame after") == 0);
    retain_frame = false;
    rift_sensor_uvc_stream_release_frame(held_frame, &stream);
    full(1, 5000);
    CHECK(count_lines(captured_log(), "Got capture frame after") == 1);
    full(0, 6000);
    CHECK(count_lines(captured_log(), "No frame provided for pixel data") == 1);
    CHECK(count_lines(captured_log(), "Got capture frame after") == 1);
    end_capture();
    finish();
}

static int compare_u64(const void *a, const void *b)
{
    uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
    return (x > y) - (x < y);
}

static void latency(void)
{
    start();
    uint64_t times[2000];
    for (unsigned i = 0; i < 2000; i++) {
        full((i + 1) & 1, 1000 + i * 1000);
        times[i] = delivered_ns - submitted_ns;
    }
    qsort(times, 2000, sizeof *times, compare_u64);
    printf("delivery_ns n=2000 p50=%llu p95=%llu p99=%llu max=%llu; callback on completing packet\n",
        (unsigned long long)times[999], (unsigned long long)times[1899],
        (unsigned long long)times[1979], (unsigned long long)times[1999]);
    finish();
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (!strcmp(argv[1], "stale-tail")) stale_tail();
    else if (!strcmp(argv[1], "nearly-complete")) nearly_complete();
    else if (!strcmp(argv[1], "header-only")) header_only();
    else if (!strcmp(argv[1], "fid-sequence")) fid_changed_without_pts();
    else if (!strcmp(argv[1], "pts-sequence")) pts_changed_without_fid();
    else if (!strcmp(argv[1], "malformed-header")) malformed_header();
    else if (!strcmp(argv[1], "error-flag")) error_flag();
    else if (!strcmp(argv[1], "truncated-header")) truncated_header();
    else if (!strcmp(argv[1], "initial-boundary")) initial_boundary();
    else if (!strcmp(argv[1], "startup-same-fid")) startup_same_fid();
    else if (!strcmp(argv[1], "startup-new-fid")) startup_new_fid();
    else if (!strcmp(argv[1], "startup-matching-tail")) startup_matching_tail();
    else if (!strcmp(argv[1], "startup-fid-only")) startup_fid_only();
    else if (!strcmp(argv[1], "startup-no-pts")) startup_no_pts();
    else if (!strcmp(argv[1], "overflow")) overflow();
    else if (!strcmp(argv[1], "trailing-data")) trailing_data();
    else if (!strcmp(argv[1], "buffer-retention")) buffer_retention();
    else if (!strcmp(argv[1], "packet-error")) packet_error();
    else if (!strcmp(argv[1], "timestamp")) timestamp();
    else if (!strcmp(argv[1], "jpeg")) jpeg_assembly();
    else if (!strcmp(argv[1], "missing-pts")) missing_pts();
    else if (!strcmp(argv[1], "missing-header-end")) missing_header_end();
    else if (!strcmp(argv[1], "same-fid-after-stall")) same_fid_after_stall();
    else if (!strcmp(argv[1], "gap-damage")) gap_damage();
    else if (!strcmp(argv[1], "discard-summary")) discard_summary();
    else if (!strcmp(argv[1], "starvation-log")) starvation_log();
    else if (!strcmp(argv[1], "opening-error-raw")) opening_error(false);
    else if (!strcmp(argv[1], "opening-error-jpeg")) opening_error(true);
    else if (!strcmp(argv[1], "bad-traffic-start")) bad_traffic(false);
    else if (!strcmp(argv[1], "bad-traffic-rejected")) bad_traffic(true);
    else if (!strcmp(argv[1], "raw-overrun-report")) raw_overrun_report();
    else if (!strcmp(argv[1], "quality-counts")) quality_counts();
    else if (!strcmp(argv[1], "latency")) latency();
    else CHECK(false);
    return 0;
}
