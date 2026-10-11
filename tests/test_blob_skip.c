// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* The dark-pixel skip (16 pixels at a time) must find exactly the blobs the pixel-by-pixel scan
 * finds: random frames with blobs, noise up to and around the threshold, and widths that are not
 * multiples of 16, run through both. */
#include <string.h>

/* Unity build: compile the engine sources under test into this binary. */
#include "rift-sensor-blobwatch.c"
#include "rift-sensor-flicker.c"
#include "test_util.h"

static uint32_t rng_state = 12345;
static uint32_t rnd(void)
{
	rng_state = rng_state * 1664525u + 1013904223u;
	return rng_state >> 8;
}

static void make_frame(uint8_t *f, int w, int h)
{
	for (int i = 0; i < w * h; i++) {
		uint32_t r = rnd() % 100;
		/* mostly dark, some speckle near the threshold */
		f[i] = r < 90 ? (uint8_t)(rnd() % 12) : (uint8_t)(BLOB_THRESHOLD_CV1 - 3 + rnd() % 7);
	}
	int n = 5 + rnd() % 25;
	for (int b = 0; b < n; b++) {
		int bw = 1 + rnd() % 9, bh = 1 + rnd() % 9, x0 = rnd() % (w - bw), y0 = rnd() % (h - bh);
		for (int y = y0; y < y0 + bh; y++)
			for (int x = x0; x < x0 + bw; x++)
				f[y * w + x] = (uint8_t)(60 + rnd() % 190);
	}
}

static void blobs(uint8_t *frame, int w, int h, bool simd, blobservation *out)
{
	skip_simd = simd;
	blobwatch *bw = blobwatch_new(BLOB_THRESHOLD_CV1);
	blobservation *ob = NULL;
	blobwatch_process(bw, frame, w, h, 0, NULL, 0, &ob);
	CHECK(ob != NULL);
	*out = *ob;
	blobwatch_release_observation(bw, ob);
	blobwatch_free(bw);
}

int main(void)
{
	/* the skip itself, against the plain scan, at every start and width */
	static uint8_t line[100];
	for (int trial = 0; trial < 2000; trial++) {
		for (int i = 0; i < 100; i++)
			line[i] = (rnd() % 40 == 0) ? (uint8_t)(BLOB_THRESHOLD_CV1 + 1 + rnd() % 200) : (uint8_t)(rnd() % (BLOB_THRESHOLD_CV1 + 1));
		int width = 1 + rnd() % 100, x = rnd() % width;
		int expect = x;
		while (expect < width && line[expect] <= BLOB_THRESHOLD_CV1)
			expect++;
		skip_simd = true;
		CHECK(skip_dark(line, x, width, BLOB_THRESHOLD_CV1) == expect);
	}

	/* whole frames, both ways */
	static uint8_t frame[1283 * 97];
	for (int trial = 0; trial < 200; trial++) {
		int w = 64 + rnd() % 1219, h = 16 + rnd() % 81;
		make_frame(frame, w, h);
		blobservation a, b;
		blobs(frame, w, h, false, &a);
		blobs(frame, w, h, true, &b);
		CHECK(a.num_blobs == b.num_blobs);
		for (int i = 0; i < a.num_blobs; i++)
			CHECK(memcmp(&a.blobs[i], &b.blobs[i], sizeof a.blobs[i]) == 0);
	}
	return 0;
}
