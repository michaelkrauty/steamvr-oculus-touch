/*
 * OpenHMD JPEG video frame decoding
 * Copyright 2021 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */
#include "../log-output.h"
#include "../ohmd-assert.h"
#include <stddef.h>
#include <stdlib.h>
#include <setjmp.h>
#include <errno.h>

#include "ohmd-jpeg.h"

#include "jpeglib.h"
#include "jerror.h"

#define MAX_QUEUE_FRAMES 4

typedef struct ohmd_jpeg_decoder_err ohmd_jpeg_decoder_err;

struct ohmd_jpeg_decoder_err {
	struct jpeg_error_mgr base;

	/* The latest warning or failure, kept for the sensor's log summary */
	char error_msg[JMSG_LENGTH_MAX];
	/* Warnings accepted while decoding the current image */
	int warnings;
	jmp_buf setjmp_buffer;
};

struct ohmd_jpeg_decoder {
	ohmd_context *ohmd_ctx;

	struct jpeg_decompress_struct cinfo;
	ohmd_jpeg_decoder_err err_mgr;

	ohmd_mutex *frames_lock;

	ohmd_video_frame **free_frames;
	int n_free_frames;

	ohmd_video_frame **alloced_frames;
	int n_alloced_frames;
};

/* Error handling */
METHODDEF (void) handle_jpeg_error(j_common_ptr cinfo)
{
	ohmd_jpeg_decoder_err *err_mgr = (ohmd_jpeg_decoder_err *) cinfo->err;

	( *(cinfo->err->format_message) ) (cinfo, err_mgr->error_msg);

	/* Jump to the setjmp point */
	longjmp(err_mgr->setjmp_buffer, 1);
}

METHODDEF (void) handle_output_message(j_common_ptr cinfo)
{
	return;
}

/* libjpeg reports damaged input as a warning and keeps decoding, filling in what is missing.
 * Invented pixels are not camera observations. Other warnings leave the pixels intact. */
static bool jpeg_warning_conceals_damage(const struct jpeg_error_mgr *err)
{
	switch (err->msg_code) {
	case JWRN_JPEG_EOF: /* the data ran out: the memory source substitutes an EOI */
	case JWRN_HIT_MARKER:
	case JWRN_HUFF_BAD_CODE:
	case JWRN_MUST_RESYNC:
	case JWRN_BOGUS_PROGRESSION:
#if JPEG_LIB_VERSION >= 70 || defined(C_ARITH_CODING_SUPPORTED) || defined(D_ARITH_CODING_SUPPORTED)
	case JWRN_ARITH_BAD_CODE:
#endif
		return true;
	case JWRN_EXTRANEOUS_DATA:
		/* Padding before EOI is skipped after the whole image was decoded. */
		return err->msg_parm.i[1] != JPEG_EOI;
	default:
		return false;
	}
}

METHODDEF (void) handle_emit_message (j_common_ptr cinfo, int msg_level)
{
	ohmd_jpeg_decoder_err *err_mgr = (ohmd_jpeg_decoder_err *) cinfo->err;

	/* Non-negative levels are trace messages */
	if (msg_level >= 0)
		return;
	if (jpeg_warning_conceals_damage(cinfo->err))
		handle_jpeg_error(cinfo);
	err_mgr->warnings++;
	( *(cinfo->err->format_message) ) (cinfo, err_mgr->error_msg);
}

static void
ohmd_jpeg_release_frame(ohmd_video_frame *frame, ohmd_jpeg_decoder *jd);

ohmd_jpeg_decoder *ohmd_jpeg_decoder_new(ohmd_context *ohmd_ctx)
{
	ohmd_jpeg_decoder *jd = ohmd_alloc(ohmd_ctx, sizeof(ohmd_jpeg_decoder));

	jd->ohmd_ctx = ohmd_ctx;
	jd->cinfo.err = jpeg_std_error(&jd->err_mgr.base);
	jd->err_mgr.base.output_message = handle_output_message;
	jd->err_mgr.base.emit_message = handle_emit_message;
	jd->err_mgr.base.error_exit = handle_jpeg_error;
	jpeg_create_decompress(&jd->cinfo);

	jd->frames_lock = ohmd_create_mutex(ohmd_ctx);

	/* Allocate frame store. They will be allocated as needed */
	jd->alloced_frames = ohmd_alloc(ohmd_ctx, MAX_QUEUE_FRAMES * sizeof (ohmd_video_frame *));
	jd->free_frames = ohmd_alloc(ohmd_ctx, MAX_QUEUE_FRAMES * sizeof (ohmd_video_frame *));

	jd->n_free_frames = jd->n_alloced_frames = 0;

	return jd;
}

void ohmd_jpeg_decoder_free(ohmd_jpeg_decoder *jd)
{
	int i;

	jpeg_destroy_decompress(&jd->cinfo);

	/* Free frames */
	for (i = 0; i < jd->n_alloced_frames; i++) {
		ohmd_video_frame *frame = jd->alloced_frames[i];
		free(frame->data);
		free(frame);
	}
	free(jd->alloced_frames);
	free(jd->free_frames);

	if (jd->frames_lock)
		ohmd_destroy_mutex(jd->frames_lock);
	free(jd);
}

static bool get_output_frame(ohmd_jpeg_decoder *jd, ohmd_video_frame *in, ohmd_video_frame **out)
{
	ohmd_lock_mutex(jd->frames_lock);
	while (jd->n_free_frames > 0) {
		jd->n_free_frames--;
		ohmd_video_frame *cur_frame = jd->free_frames[jd->n_free_frames];

		/* Frame format must match or something is very wrong */
		OHMD_ASSERT(in->width == cur_frame->width && in->height == cur_frame->height);
		*out = cur_frame;
		goto done;
	}

	/* No free frame available, allocate one. assert if there's no room
	 * because the limit is too small */
	OHMD_ASSERT(jd->n_alloced_frames < MAX_QUEUE_FRAMES);

	ohmd_video_frame *frame = ohmd_alloc(jd->ohmd_ctx, sizeof (ohmd_video_frame));
	size_t data_size = in->width * in->height;

	frame->format = OHMD_VIDEO_FRAME_FORMAT_GRAY8;
	frame->data = ohmd_alloc(jd->ohmd_ctx, data_size);
	frame->data_block_size = frame->data_size = data_size;

	frame->releasefn = (ohmd_video_frame_release_func) ohmd_jpeg_release_frame;
	frame->owner = jd;
	jd->alloced_frames[jd->n_alloced_frames++] = frame;

	*out = frame;

done:
	ohmd_unlock_mutex(jd->frames_lock);
	return true;
}

const char *ohmd_jpeg_decoder_message(const ohmd_jpeg_decoder *jd)
{
	return jd->err_mgr.error_msg;
}

int ohmd_jpeg_decoder_warnings(const ohmd_jpeg_decoder *jd)
{
	return jd->err_mgr.warnings;
}

#define DECODE_FAIL(...) do { \
		snprintf(jd->err_mgr.error_msg, sizeof jd->err_mgr.error_msg, __VA_ARGS__); \
		goto fail; \
	} while (0)

bool ohmd_jpeg_decoder_decode(ohmd_jpeg_decoder *jd, ohmd_video_frame *in, ohmd_video_frame **out)
{
	*out = NULL;
	jd->err_mgr.warnings = 0;

	if (setjmp(jd->err_mgr.setjmp_buffer)) {
		/* If an error occurs, it will longjmp back here, indicating failure */
		goto fail;
	}

	if (in->data == NULL || in->data_size > in->data_block_size ||
			in->width <= 0 || in->height <= 0 || in->stride != in->width ||
			(size_t)in->height > SIZE_MAX / in->width)
		DECODE_FAIL("Invalid JPEG input frame");
	/* Bytes after EOI are ignored, and the decoder reports data that ends early. */
	if (in->data_size < 2 || in->data[0] != 0xff || in->data[1] != 0xd8)
		DECODE_FAIL("Image data does not start like a JPEG");

	jpeg_mem_src(&jd->cinfo, in->data, in->data_size);

	int ret = jpeg_read_header(&jd->cinfo, TRUE);
	if (ret != JPEG_HEADER_OK)
		DECODE_FAIL("Corrupted JPEG frame");

	if (jd->cinfo.jpeg_color_space != JCS_YCbCr &&
	    jd->cinfo.jpeg_color_space != JCS_GRAYSCALE &&
	    jd->cinfo.jpeg_color_space != JCS_RGB)
		DECODE_FAIL("Unsupported JPEG frame colorspace %u", (unsigned)jd->cinfo.jpeg_color_space);

	jd->cinfo.out_color_space = JCS_GRAYSCALE;

	if (!jpeg_start_decompress(&jd->cinfo))
		DECODE_FAIL("Failed to initialise JPEG decompression");

	/* Check the frame is of the expected dimension */
	if (in->width != jd->cinfo.output_width || in->height != jd->cinfo.output_height ||
			jd->cinfo.output_components != 1)
		DECODE_FAIL("Unexpected size / format JPEG frame (width %d vs %u, height %d vs %u, %d components)",
				in->width, jd->cinfo.output_width, in->height, jd->cinfo.output_height, jd->cinfo.output_components);

	if (!get_output_frame(jd, in, out))
		DECODE_FAIL("No JPEG output frame");

	/* Copy the details. Only the format is changing */
	(*out)->start_ts = in->start_ts;
	(*out)->pts = in->pts;
	(*out)->stride = in->stride;
	(*out)->width = in->width;
	(*out)->height = in->height;

	while (jd->cinfo.output_scanline < jd->cinfo.output_height) {
		unsigned char *buffer_array[1];
		buffer_array[0] = (*out)->data + (jd->cinfo.output_scanline * (*out)->width);

		if (jpeg_read_scanlines(&jd->cinfo, buffer_array, 1) != 1)
			DECODE_FAIL("JPEG decoding stopped at row %u", jd->cinfo.output_scanline);
	}

	if (!jpeg_finish_decompress(&jd->cinfo))
		DECODE_FAIL("JPEG decoding did not finish");

#if 0
	/* Debug code to dump frames to files */
	char fname[128];
	static int file_index = 0;
	sprintf(fname, "/tmp/openhmd-jpeg/frame-%05d.jpg", file_index++);
	FILE *f = fopen(fname, "wb");
	if (f == NULL) {
		ohmd_log_fprintf(stderr, "Decode fail: err %d %m", errno);
		return false;
	}
	fwrite(in->data, in->data_size, 1, f);
	fclose(f);
#endif

	LOGD("Success JPEG decode of %zu bytes", in->data_size);
	return true;

fail:
	LOGD("Failed JPEG decode of %zu bytes", in->data_size);
	if (*out != NULL) {
		ohmd_video_frame_release(*out);
		*out = NULL;
	}
	jpeg_abort_decompress (&jd->cinfo);
	return false;
}
#undef DECODE_FAIL

static void
ohmd_jpeg_release_frame(ohmd_video_frame *frame, ohmd_jpeg_decoder *jd)
{
	OHMD_ASSERT(frame->owner == jd);

	/* Put the frame back on the free queue */
	ohmd_lock_mutex(jd->frames_lock);
	OHMD_ASSERT(jd->n_free_frames < jd->n_alloced_frames);
	jd->free_frames[jd->n_free_frames++] = frame;
	ohmd_unlock_mutex(jd->frames_lock);
}
