// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krautkramer

/* Inclusive endpoints through the actual full-frame detector, not fabricated extents. */
#include <string.h>
#include "rift-sensor-blobwatch.c"
#include "rift-sensor-flicker.c"
#include "test_util.h"
#define W 1280
#define H 960
static uint8_t frame[W * H];

static void rect(int x, int y, int w, int h, int value)
{
	for (int j = y; j < y + h; j++)
		memset(frame + j * W + x, value, (size_t)w);
}

static blobservation detect(int threshold)
{
	blobwatch *bw = blobwatch_new(threshold);
	blobservation *ob = NULL;
	blobwatch_process(bw, frame, W, H, 0, NULL, 0, &ob);
	CHECK(ob != NULL);
	blobservation copy = *ob;
	blobwatch_release_observation(bw, ob);
	blobwatch_free(bw);
	return copy;
}

static void box(const struct blob *b, int x, int y, int w, int h, int area)
{
	CHECK(b->left == x && b->top == y);
	CHECK(b->width == w && b->height == h && b->area == area);
	CHECK_NEAR(b->x, x + w / 2.0, 1e-4);
	CHECK_NEAR(b->y, y + h / 2.0, 1e-4);
}

static void gaussians(int threshold)
{
	const double sigma = threshold == BLOB_THRESHOLD_CV1 ? 0.8 : 1.2;
	const int peak = threshold == BLOB_THRESHOLD_CV1 ? 200 : 255;
	const int expected_area = threshold == BLOB_THRESHOLD_CV1 ? 23 : 19;
	for (int rotate = 0; rotate < 2; rotate++) {
		for (int bottom = 0; bottom < 2; bottom++) {
			int cy = bottom ? H - (rotate ? 2 : 5) : 100;
			int w = rotate ? 9 : 3, h = rotate ? 3 : 9;
			memset(frame, 0, sizeof(frame));
			for (int dy = -12; dy <= 12; dy++) {
				for (int dx = -12; dx <= 12; dx++) {
					int x = 100 + dx, y = cy + dy;
					double a = rotate ? dy : dx, b = rotate ? dx : dy;
					if (y >= 0 && y < H)
						frame[y * W + x] = (uint8_t)lround(peak * exp(-0.5 *
							(a * a / (sigma * sigma) + b * b / (9 * sigma * sigma))));
				}
			}
			/* Verify threshold membership independently of the detector. */
			int area = 0, left = W, right = 0, top = H, last = 0;
			for (int y = 0; y < H; y++) {
				for (int x = 0; x < W; x++) {
					if (frame[y * W + x] <= threshold)
						continue;
					area++;
					left = min(left, x); right = max(right, x);
					top = min(top, y); last = max(last, y);
				}
			}
			CHECK(area == expected_area && right - left + 1 == w && last - top + 1 == h);
			blobservation ob = detect(threshold);
			CHECK(ob.num_blobs == 1);
			box(&ob.blobs[0], 100 - w / 2, cy - h / 2, w, h, expected_area);
		}
	}
}

static void endpoints(int threshold)
{
	/* A right-hand extent on the terminating row triggers the in-scan flush;
	 * otherwise the trailing flush runs. The threshold-valued row is not a
	 * member and must affect neither centroid nor height, above or below. */
	for (int early = 0; early < 2; early++) {
		for (int below = 0; below < 2; below++) {
			memset(frame, 0, sizeof(frame));
			rect(100, 100, 4, 4, 200);
			rect(100, below ? 104 : 99, 4, 1, threshold);
			if (early)
				rect(200, 104, 4, 4, 200);
			blobservation ob = detect(threshold);
			CHECK(ob.num_blobs == 1 + early);
			box(&ob.blobs[0], 100, 100, 4, 4, 16);
			if (early)
				box(&ob.blobs[1], 200, 104, 4, 4, 16);
		}
	}
	/* Separated neighbors and both previous/current extents at the final row. */
	memset(frame, 0, sizeof(frame));
	rect(100, 100, 6, 6, 200);
	rect(100, 107, 6, 6, 200);
	rect(300, H - 7, 6, 6, 200);
	rect(400, H - 6, 6, 6, 200);
	blobservation ob = detect(threshold);
	CHECK(ob.num_blobs == 4);
	box(&ob.blobs[0], 100, 100, 6, 6, 36);
	box(&ob.blobs[1], 100, 107, 6, 6, 36);
	box(&ob.blobs[2], 300, H - 7, 6, 6, 36);
	box(&ob.blobs[3], 400, H - 6, 6, 6, 36);
}

static void policy(int threshold)
{
	const int sizes[][3] = {
		{4, 12, 1}, {12, 4, 1}, /* inclusive 3:1 boundary */
		{4, 13, 0}, {13, 4, 0}, {24, 3, 0}, {3, 24, 0},
		{36, 36, 1}, {37, 36, 0}, {36, 37, 0}, /* existing size cap */
		{1, 1, 0}
	};
	for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
		memset(frame, 0, sizeof(frame));
		int w = sizes[i][0], h = sizes[i][1];
		rect(100, 100, w, h, 200);
		blobservation ob = detect(threshold);
		CHECK(ob.num_blobs == sizes[i][2]);
		if (ob.num_blobs)
			box(&ob.blobs[0], 100, 100, w, h, w * h);
	}
	memset(frame, 0, sizeof(frame));
	rect(100, 100, 4, 4, BLOB_THRESHOLD_MIN - 1);
	CHECK(detect(threshold).num_blobs == 0);
	memset(frame, 0, sizeof(frame));
	rect(100, H - 1, 1, 1, 200);
	CHECK(detect(threshold).num_blobs == 0);
}

int main(void)
{
	for (int simd = 0; simd < 2; simd++) {
		skip_simd = simd;
		gaussians(BLOB_THRESHOLD_CV1);
		gaussians(BLOB_THRESHOLD_DK2);
		endpoints(BLOB_THRESHOLD_CV1);
		endpoints(BLOB_THRESHOLD_DK2);
		policy(BLOB_THRESHOLD_CV1);
		policy(BLOB_THRESHOLD_DK2);
	}
	return 0;
}
