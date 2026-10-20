// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* The actual analysis preparation must reject incomplete decoded/raw images. */
#include "platform-posix.c"
#include "rift-sensor.c"
#include "sensor/uvc.c"
#if HAVE_LIBJPEG
#include "ohmd-jpeg.c"
#endif
#include "ohmd-video.c"
#include "test_util.h"
#include <fcntl.h>
#include <unistd.h>

void *ohmd_allocfn(ohmd_context *ctx, const char *message, size_t size)
{ (void)ctx; (void)message; return calloc(1, size); }

void rift_pose_finder_exp_info_to_dev_state(rift_pose_finder *pf,
	const rift_tracked_device_exposure_info *info, rift_sensor_frame_device_state *state)
{ (void)pf; (void)info; (void)state; CHECK(false); }

uint64_t ohmd_monotonic_conv(uint64_t ticks, uint64_t src_per_sec, uint64_t dst_per_sec)
{ return ticks / src_per_sec * dst_per_sec + ticks % src_per_sec * dst_per_sec / src_per_sec; }

/* The capture callback's queue-reclaim path; the fixtures never empty the capture queue. */
void rift_tracker_frame_release(rift_tracker_ctx *ctx, uint64_t local_ts, uint64_t frame_local_ts,
	rift_tracker_exposure_info *info, const char *source)
{ (void)ctx; (void)local_ts; (void)frame_local_ts; (void)info; (void)source; CHECK(false); }
void blobwatch_release_observation(blobwatch *bw, blobservation *ob)
{ (void)bw; (void)ob; CHECK(false); }

static int tracker_frames, releases;

bool rift_tracker_frame_captured(rift_tracker_ctx *ctx, uint64_t local_ts, uint64_t frame_start_local_ts,
	rift_tracker_exposure_info *info, const char *source)
{
	(void)ctx; (void)local_ts; (void)frame_start_local_ts; (void)source;
	memset(info, 0, sizeof *info);
	tracker_frames++;
	return true;
}

static void count_release(ohmd_video_frame *frame, void *owner)
{ (void)frame; (void)owner; releases++; }

static void raw_layout(void)
{
	ohmd_context ctx = {0};
	ohmd_monotonic_init(&ctx);
	rift_sensor_device dev = {0};
	dev.calib.width = dev.calib.height = 128;
	rift_sensor_ctx sensor = { .ohmd_ctx = &ctx, .dev = &dev };
	unsigned char pixels[128 * 128] = {0};
	ohmd_video_frame video = { .format = OHMD_VIDEO_FRAME_FORMAT_GRAY8,
		.width = 128, .height = 128, .stride = 128, .data = pixels,
		.data_size = sizeof pixels, .data_block_size = sizeof pixels };
	rift_sensor_analysis_frame frame = { .vframe = &video };
	CHECK(init_frame_analyse(&sensor, &frame));
	video.data_size--;
	CHECK(!init_frame_analyse(&sensor, &frame));
	video.data_size = sizeof pixels;
	video.data_block_size--;
	CHECK(!init_frame_analyse(&sensor, &frame));
	video.data_block_size = sizeof pixels;
	video.stride++;
	CHECK(!init_frame_analyse(&sensor, &frame));
	/* Packed and complete for its own size, but not the camera's. */
	video.stride = video.width = 127;
	video.data_size = 127 * 128;
	CHECK(!init_frame_analyse(&sensor, &frame));
	video.stride = video.width = 128;
	video.height = 127;
	video.data_size = 128 * 127;
	CHECK(!init_frame_analyse(&sensor, &frame));
	video.height = 128;
	video.data_size = sizeof pixels;
	CHECK(init_frame_analyse(&sensor, &frame));
}

/* A short raw image is released before the capture callback queues it for analysis. */
static void capture_guard(void)
{
	ohmd_context ctx = {0};
	ohmd_monotonic_init(&ctx);
	rift_sensor_device dev = {0};
	dev.calib.width = dev.calib.height = 128;
	rift_sensor_ctx sensor = { .ohmd_ctx = &ctx, .dev = &dev };
	sensor.sensor_lock = ohmd_create_mutex(&ctx);
	sensor.fast_analysis_q_cond = ohmd_create_cond(&ctx);
	CHECK(sensor.sensor_lock && sensor.fast_analysis_q_cond);
	INIT_QUEUE(&sensor.capture_frame_q);
	INIT_QUEUE(&sensor.fast_analysis_q);
	INIT_QUEUE(&sensor.long_analysis_q);
	rift_sensor_analysis_frame slot = {0};
	PUSH_QUEUE(&sensor.capture_frame_q, &slot);
	unsigned char pixels[128 * 128] = {0};
	ohmd_video_frame video = { .format = OHMD_VIDEO_FRAME_FORMAT_GRAY8,
		.width = 128, .height = 128, .stride = 128, .data = pixels,
		.data_size = sizeof pixels - 1, .data_block_size = sizeof pixels,
		.releasefn = count_release };
	frame_captured_cb(&dev, &video, &sensor);
	CHECK(releases == 1 && tracker_frames == 0);
	CHECK(POP_QUEUE(&sensor.fast_analysis_q) == NULL);
	video.data_size = sizeof pixels;
	frame_captured_cb(&dev, &video, &sensor);
	CHECK(releases == 1 && tracker_frames == 1);
	CHECK(POP_QUEUE(&sensor.fast_analysis_q) == &slot && slot.vframe == &video);
	ohmd_destroy_cond(sensor.fast_analysis_q_cond);
	ohmd_destroy_mutex(sensor.sensor_lock);
}

#if HAVE_LIBJPEG
static unsigned char *encode_image_mode(unsigned long *size, unsigned restart_interval, bool progressive, int width, int height)
{
	struct jpeg_compress_struct encoder = {0};
	struct jpeg_error_mgr errors;
	encoder.err = jpeg_std_error(&errors);
	jpeg_create_compress(&encoder);
	unsigned char *data = NULL;
	jpeg_mem_dest(&encoder, &data, size);
	encoder.image_width = width;
	encoder.image_height = height;
	encoder.input_components = 1;
	encoder.in_color_space = JCS_GRAYSCALE;
	jpeg_set_defaults(&encoder);
	jpeg_set_quality(&encoder, 90, TRUE);
	if (progressive) jpeg_simple_progression(&encoder);
	encoder.restart_interval = restart_interval;
	jpeg_start_compress(&encoder, TRUE);
	unsigned char row[1280];
	CHECK(width <= (int)sizeof row);
	while (encoder.next_scanline < encoder.image_height) {
		for (int x = 0; x < width; x++)
			row[x] = (x * 73 + encoder.next_scanline * 31) & 255;
		JSAMPROW rows[] = {row};
		CHECK(jpeg_write_scanlines(&encoder, rows, 1) == 1);
	}
	jpeg_finish_compress(&encoder);
	jpeg_destroy_compress(&encoder);
	return data;
}

static unsigned char *encode_image(unsigned long *size, unsigned restart_interval)
{
	return encode_image_mode(size, restart_interval, false, 128, 128);
}

static void jpeg_quality(bool corrupt)
{
	ohmd_context ctx = {0};
	ohmd_monotonic_init(&ctx);
	rift_sensor_device dev = {0};
	dev.calib.width = dev.calib.height = 128;
	rift_sensor_ctx sensor = { .ohmd_ctx = &ctx, .dev = &dev };
	unsigned long size = 0;
	unsigned char *data = encode_image(&size, 0);
	ohmd_video_frame input = { .format = OHMD_VIDEO_FRAME_FORMAT_JPEG,
		.width = 128, .height = 128, .stride = 128, .data = data,
		.data_size = size, .data_block_size = size, .pts = 123, .start_ts = 456 };
	ohmd_jpeg_decoder *decoder = ohmd_jpeg_decoder_new(&ctx);
	CHECK(decoder != NULL);
	if (corrupt) {
		/* Retain a valid JPEG header and EOI, but remove half the entropy data.
		 * libjpeg can conceal this with invented rows and still return success. */
		input.data_size = size / 2;
		data[input.data_size - 2] = 0xff;
		data[input.data_size - 1] = 0xd9;
	}
	for (int attempt = 0; attempt < 5; attempt++) {
		ohmd_video_frame *output = NULL;
		bool decoded = ohmd_jpeg_decoder_decode(decoder, &input, &output);
		CHECK(decoded == !corrupt);
		if (decoded) {
			CHECK(output != NULL && output->pts == 123 && output->start_ts == 456);
			rift_sensor_analysis_frame frame = { .vframe = output };
			CHECK(init_frame_analyse(&sensor, &frame));
			output->data_size--;
			CHECK(!init_frame_analyse(&sensor, &frame));
			output->data_size++;
			ohmd_video_frame_release(output);
		} else {
			CHECK(output == NULL);
		}
		CHECK(decoder->n_free_frames == decoder->n_alloced_frames);
	}
	/* A failed decode must neither leak a pooled output nor stall the decoder. */
	free(data);
	data = encode_image(&size, 0);
	input.data = data;
	input.data_size = input.data_block_size = size;
	ohmd_video_frame *output = NULL;
	CHECK(ohmd_jpeg_decoder_decode(decoder, &input, &output));
	CHECK(valid_raw_frame(&dev, output));
	ohmd_video_frame_release(output);
	CHECK(decoder->n_alloced_frames == 1 && decoder->n_free_frames == 1);
	input.width--;
	CHECK(!ohmd_jpeg_decoder_decode(decoder, &input, &output) && output == NULL);
	CHECK(decoder->n_free_frames == 1);
	ohmd_jpeg_decoder_free(decoder);
	free(data);
}

static int jpeg_deliveries;
static ohmd_video_frame *assembled_jpeg;

static void jpeg_assembled(rift_sensor_uvc_stream *stream, ohmd_video_frame *frame, void *data)
{
	(void)stream; (void)data;
	jpeg_deliveries++;
	assembled_jpeg = frame;
}

static void jpeg_packet(rift_sensor_uvc_stream *stream, unsigned identity, const unsigned char *data, size_t size)
{
	unsigned char *packet = calloc(1, size + 12);
	CHECK(packet != NULL);
	packet[0] = 12; packet[1] = 0x8c | (identity & 1); packet[2] = identity;
	memcpy(packet + 12, data, size);
	process_payload(stream, packet, size + 12);
	free(packet);
}

/* A valid APP1 contains thumbnail-like EOI bytes exactly at a USB packet boundary. */
static void jpeg_embedded_eoi(void)
{
	ohmd_context ctx = {0};
	ohmd_monotonic_init(&ctx);
	unsigned long size = 0;
	unsigned char *data = encode_image(&size, 0);
	const unsigned char app[] = {0xff, 0xe1, 0, 10, 0xff, 0xd8, 1, 2, 0xff, 0xd9, 3, 4};
	unsigned char *encoded = malloc(size + sizeof app);
	unsigned char *pixels = malloc(size + sizeof app);
	CHECK(encoded && pixels);
	memcpy(encoded, data, 2);
	memcpy(encoded + 2, app, sizeof app);
	memcpy(encoded + 2 + sizeof app, data + 2, size - 2);
	ohmd_video_frame buffer = {.data = pixels, .data_block_size = size + sizeof app};
	ohmd_video_frame *free_frames[] = {&buffer};
	rift_sensor_uvc_stream stream = {.ohmd_ctx = &ctx, .format = OHMD_VIDEO_FRAME_FORMAT_JPEG,
		.width = 128, .height = 128, .stride = 128, .frame_size = size + sizeof app,
		.have_frame_id = true, .frame_cb = jpeg_assembled, .frames_lock = ohmd_create_mutex(&ctx),
		.free_frames = free_frames, .n_free_frames = 1, .n_alloced_frames = 1};
	buffer.owner = &stream;
	CHECK(stream.frames_lock != NULL);
	ohmd_jpeg_decoder *decoder = ohmd_jpeg_decoder_new(&ctx);
	for (unsigned identity = 1; identity <= 3; identity++) {
		/* Split the segment length as well as the embedded EOI. */
		jpeg_packet(&stream, identity, encoded, 5);
		jpeg_packet(&stream, identity, encoded + 5, 7); /* ends at the thumbnail's FF D9 */
		CHECK(jpeg_deliveries == (int)identity - 1 && stream.cur_frame == &buffer);
		jpeg_packet(&stream, identity, encoded + 12, size + sizeof app - 13);
		CHECK(jpeg_deliveries == (int)identity - 1);
		jpeg_packet(&stream, identity, encoded + size + sizeof app - 1, 1); /* outer EOI */
		CHECK(jpeg_deliveries == (int)identity && assembled_jpeg == &buffer && stream.cur_frame == NULL);
		CHECK(buffer.data_size == size + sizeof app && !memcmp(buffer.data, encoded, buffer.data_size));
		ohmd_video_frame *decoded = NULL;
		CHECK(ohmd_jpeg_decoder_decode(decoder, &buffer, &decoded));
		CHECK(decoded->data_size == 128 * 128);
		ohmd_video_frame_release(decoded);
		rift_sensor_uvc_stream_release_frame(&buffer, &stream);
		CHECK(stream.n_free_frames == 1);
	}
	ohmd_jpeg_decoder_free(decoder);
	ohmd_destroy_mutex(stream.frames_lock);
	free(pixels); free(encoded); free(data);
}

enum jpeg_edit { PADDING, EXTRANEOUS, EXTRANEOUS_HEADER, TRUNCATED, MISSING_EOI, BAD_CODE, RESYNC, PROGRESSION };

/* Decodes an edited libjpeg image; the edits are what a USB 2 sensor's MJPEG may contain. */
static bool decode_edited(enum jpeg_edit edit, int *warnings, char *message, size_t message_size)
{
	ohmd_context ctx = {0};
	ohmd_monotonic_init(&ctx);
	unsigned long size = 0;
	unsigned char *data = encode_image_mode(&size, edit == RESYNC ? 1 : 0, edit == PROGRESSION, 128, 128);
	unsigned char *edited = calloc(1, size + 64);
	CHECK(edited != NULL && size > 1024 && data[size - 2] == 0xff && data[size - 1] == 0xd9);
	memcpy(edited, data, size);
	size_t edited_size = size;
	switch (edit) {
	case PADDING: /* zeros after EOI */
		edited_size += 32;
		break;
	case EXTRANEOUS: /* encoder padding between the last entropy byte and EOI */
		memset(edited + size - 2, 0x55, 16);
		memcpy(edited + size - 2 + 16, data + size - 2, 2);
		edited_size += 16;
		break;
	case EXTRANEOUS_HEADER: /* stray bytes between SOI and the next marker */
		memset(edited + 2, 0x12, 4);
		memcpy(edited + 6, data + 2, size - 2);
		edited_size += 4;
		break;
	case TRUNCATED: /* the last packets never arrived, EOF delivered the rest */
		edited_size = size * 6 / 10;
		break;
	case MISSING_EOI:
		edited_size = size - 2;
		break;
	case BAD_CODE: /* all-ones bits are no Huffman code; libjpeg-turbo warns near the end */
		for (int i = 0; i < 4; i++) {
			edited[size - 200 + 2 * i] = 0xff;
			edited[size - 200 + 2 * i + 1] = 0;
		}
		break;
	case PROGRESSION: { /* Remove the first (DC) scan, retain the valid later AC scans. */
		size_t begin = 2;
		while (begin + 4 < size && data[begin + 1] != 0xda)
			begin += 2 + ((unsigned)data[begin + 2] << 8) + data[begin + 3];
		CHECK(begin + 4 < size && data[begin] == 0xff && data[begin + 1] == 0xda);
		size_t end = begin + 2 + ((unsigned)data[begin + 2] << 8) + data[begin + 3];
		while (end + 1 < size) {
			if (data[end] == 0xff && data[end + 1] != 0 && data[end + 1] != 0xff &&
					(data[end + 1] < 0xd0 || data[end + 1] > 0xd7)) break;
			end++;
		}
		CHECK(end + 1 < size);
		memmove(edited + begin, data + end, size - end);
		edited_size -= end - begin;
		break;
	}
	case RESYNC: { /* a restart marker carries the wrong number */
		int seen = 0;
		for (size_t i = 400; i + 1 < size; i++)
			if (data[i] == 0xff && (data[i + 1] & 0xf8) == 0xd0 && ++seen == 3) {
				edited[i + 1] = 0xd0 | ((data[i + 1] + 4) & 7);
				break;
			}
		CHECK(seen == 3);
		break;
	}
	}
	ohmd_jpeg_decoder *decoder = ohmd_jpeg_decoder_new(&ctx);
	CHECK(decoder != NULL);
	ohmd_video_frame input = { .format = OHMD_VIDEO_FRAME_FORMAT_JPEG,
		.width = 128, .height = 128, .stride = 128, .data = edited,
		.data_size = edited_size, .data_block_size = size + 64 };
	ohmd_video_frame *output = NULL;
	bool decoded = ohmd_jpeg_decoder_decode(decoder, &input, &output);
	CHECK(decoded == (output != NULL));
	*warnings = ohmd_jpeg_decoder_warnings(decoder);
	snprintf(message, message_size, "%s", ohmd_jpeg_decoder_message(decoder));
	if (decoded) {
		rift_sensor_device dev = {0};
		dev.calib.width = dev.calib.height = 128;
		CHECK(valid_raw_frame(&dev, output));
		ohmd_video_frame_release(output);
	}
	CHECK(decoder->n_free_frames == decoder->n_alloced_frames);
	ohmd_jpeg_decoder_free(decoder);
	free(edited);
	free(data);
	return decoded;
}

static void jpeg_edit(enum jpeg_edit edit)
{
	int warnings = -1;
	char message[JMSG_LENGTH_MAX];
	bool decoded = decode_edited(edit, &warnings, message, sizeof message);
	printf("decoded=%d warnings=%d message=\"%s\"\n", decoded, warnings, message);
	switch (edit) {
	case PADDING:
		CHECK(decoded && warnings == 0);
		break;
	case EXTRANEOUS:
		CHECK(decoded && warnings == 1 && strstr(message, "extraneous bytes before marker 0xd9"));
		break;
	case EXTRANEOUS_HEADER:
		CHECK(!decoded && strstr(message, "extraneous bytes before marker 0xe0"));
		break;
	case TRUNCATED:
	case MISSING_EOI:
		CHECK(!decoded && strstr(message, "Premature end of JPEG file"));
		break;
	case BAD_CODE:
		CHECK(!decoded && strstr(message, "bad Huffman code"));
		break;
	case PROGRESSION:
		CHECK(!decoded && strstr(message, "progression"));
		break;
	case RESYNC:
		CHECK(!decoded && strstr(message, "instead of RST"));
		break;
	}
}

/* Only warnings that mean libjpeg filled in damaged data reject the image. */
static void jpeg_warning_codes(void)
{
	static const int concealing[] = { JWRN_JPEG_EOF, JWRN_HIT_MARKER, JWRN_HUFF_BAD_CODE, JWRN_MUST_RESYNC, JWRN_BOGUS_PROGRESSION,
#if JPEG_LIB_VERSION >= 70 || defined(C_ARITH_CODING_SUPPORTED) || defined(D_ARITH_CODING_SUPPORTED)
		JWRN_ARITH_BAD_CODE,
#endif
	};
	static const int harmless[] = { JWRN_JFIF_MAJOR, JWRN_ADOBE_XFORM, JWRN_BOGUS_ICC };
	struct jpeg_error_mgr err = {0};
	for (size_t i = 0; i < sizeof concealing / sizeof *concealing; i++) {
		err.msg_code = concealing[i];
		CHECK(jpeg_warning_conceals_damage(&err));
	}
	for (size_t i = 0; i < sizeof harmless / sizeof *harmless; i++) {
		err.msg_code = harmless[i];
		CHECK(!jpeg_warning_conceals_damage(&err));
	}
	err.msg_code = JWRN_EXTRANEOUS_DATA;
	err.msg_parm.i[1] = JPEG_EOI;
	CHECK(!jpeg_warning_conceals_damage(&err));
	err.msg_parm.i[1] = 0xc4; /* before a Huffman table */
	CHECK(jpeg_warning_conceals_damage(&err));
}

static void keep_input(ohmd_video_frame *frame, void *owner)
{ (void)frame; (void)owner; }

/* Decode failures and accepted warnings: at most one line per sensor every 10 s. */
static void jpeg_summary(void)
{
	ohmd_context ctx = {0};
	ohmd_monotonic_init(&ctx);
	rift_sensor_device dev = {0};
	dev.calib.width = dev.calib.height = 128;
	rift_sensor_ctx sensor = { .ohmd_ctx = &ctx, .dev = &dev, .id = 3 };
	unsigned long size = 0;
	unsigned char *good = encode_image(&size, 0);
	unsigned char *padded = calloc(1, size + 16);
	CHECK(padded != NULL);
	memcpy(padded, good, size - 2);
	memset(padded + size - 2, 0x55, 16);
	memcpy(padded + size + 14, good + size - 2, 2);
	ohmd_video_frame input = { .format = OHMD_VIDEO_FRAME_FORMAT_JPEG,
		.width = 128, .height = 128, .stride = 128, .data = good,
		.data_block_size = size + 16, .releasefn = keep_input };

	/* The engine's log goes to stdout in this unity build; a pipe holds these few lines. */
	int pipe_fds[2];
	CHECK(pipe(pipe_fds) == 0 && fcntl(pipe_fds[0], F_SETFL, O_NONBLOCK) == 0);
	fflush(stdout);
	int saved_stdout = dup(1);
	CHECK(saved_stdout >= 0 && dup2(pipe_fds[1], 1) == 1);

	uint64_t second = 1000000000;
	rift_sensor_analysis_frame frame = {0};
#define DECODE(image, bytes, ts, ok) do { \
		input.data = (image); \
		input.data_size = (bytes); \
		input.start_ts = (ts); \
		frame.vframe = &input; \
		CHECK(decode_frame(&sensor, &frame) == (ok)); \
		if (ok) \
			ohmd_video_frame_release(frame.vframe); \
	} while (0)
	for (int i = 0; i < 5; i++)
		DECODE(good, size / 2, 100 * second + i * second / 50, false);
	DECODE(padded, size + 16, 101 * second, true);
	CHECK(sensor.jpeg_failed == 5 && sensor.jpeg_warned == 1);
	DECODE(good, size, 109 * second, true);
	CHECK(sensor.jpeg_failed == 5 && sensor.jpeg_warned == 1);
	DECODE(good, size, 110 * second, true);
	CHECK(sensor.jpeg_failed == 0 && sensor.jpeg_warned == 0);

	for (int i = 0; i < 20; i++)
		DECODE(good, size / 2, 111 * second + i * second / 50, false);
	DECODE(good, size, 121 * second, true);
	DECODE(padded, size + 16, 122 * second, true);
	DECODE(padded, size + 16, 125 * second, true);
	DECODE(good, size, 132 * second, true);
#undef DECODE

	fflush(stdout);
	CHECK(dup2(saved_stdout, 1) == 1);
	close(saved_stdout);
	static char log[8192];
	ssize_t n = read(pipe_fds[0], log, sizeof log - 1);
	log[n > 0 ? n : 0] = 0;
	close(pipe_fds[0]);
	close(pipe_fds[1]);
	fputs(log, stdout);
	CHECK(strstr(log, "Sensor 3 video: discarded 5 damaged images in the last 10 s (5 could not be decoded), "
		"and kept 1 despite decoder warnings; latest decoder message: Corrupt JPEG data: "));
	CHECK(strstr(log, "Sensor 3 video: discarded 20 damaged images in the last 10 s (20 could not be decoded); "
		"latest decoder message: Premature end of JPEG file"));
	CHECK(strstr(log, "Sensor 3 video: kept 2 images despite decoder warnings in the last 10 s; latest decoder message: Corrupt JPEG data: "));
	int lines = 0;
	for (const char *at = log; (at = strstr(at, "Sensor 3 video:")) != NULL; at++)
		lines++;
	CHECK(lines == 3);
	ohmd_jpeg_decoder_free(sensor.jpeg_decoder);
	free(padded);
	free(good);
}
#endif

int main(int argc, char **argv)
{
	CHECK(argc == 2);
	if (!strcmp(argv[1], "layout")) raw_layout();
	else if (!strcmp(argv[1], "capture-guard")) capture_guard();
#if HAVE_LIBJPEG
	else if (!strcmp(argv[1], "jpeg")) jpeg_quality(false);
	else if (!strcmp(argv[1], "jpeg-corrupt")) jpeg_quality(true);
	else if (!strcmp(argv[1], "jpeg-padding")) jpeg_edit(PADDING);
	else if (!strcmp(argv[1], "jpeg-extraneous")) jpeg_edit(EXTRANEOUS);
	else if (!strcmp(argv[1], "jpeg-extraneous-header")) jpeg_edit(EXTRANEOUS_HEADER);
	else if (!strcmp(argv[1], "jpeg-truncated")) jpeg_edit(TRUNCATED);
	else if (!strcmp(argv[1], "jpeg-missing-eoi")) jpeg_edit(MISSING_EOI);
	else if (!strcmp(argv[1], "jpeg-bad-code")) jpeg_edit(BAD_CODE);
	else if (!strcmp(argv[1], "jpeg-resync")) jpeg_edit(RESYNC);
	else if (!strcmp(argv[1], "jpeg-warning-codes")) jpeg_warning_codes();
	else if (!strcmp(argv[1], "jpeg-summary")) jpeg_summary();
	else if (!strcmp(argv[1], "jpeg-embedded-eoi")) jpeg_embedded_eoi();
	else if (!strcmp(argv[1], "jpeg-progression")) jpeg_edit(PROGRESSION);
#endif
	else CHECK(false);
	puts("test_sensor_frame_quality: ok");
	return 0;
}
