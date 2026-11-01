// SPDX-License-Identifier: Apache-2.0

/* Full-frame production regression: moving a peak within a scanline run must
 * not change acceptance. Height accounting is intentionally not tested here. */
#include "rift-sensor-blobwatch.c"
#include "rift-sensor-flicker.c"
#include "test_util.h"

#define W 1280
#define H 960
static uint8_t frame[W * H];
static int failures;

static void patch(int threshold, int y0, int w, int h, int faint,
                  int peak, int px, int py, int expected)
{
	memset(frame, 0, sizeof(frame));
	for (int y = 0; y < h; y++)
		memset(frame + (y0 + y) * W + 100, faint, (size_t)w);
	frame[(y0 + py) * W + 100 + px] = peak;
	blobwatch *bw = blobwatch_new(threshold);
	CHECK(bw != NULL);
	blobservation *ob = NULL;
	blobwatch_process(bw, frame, W, H, 0, NULL, 0, &ob);
	CHECK(ob != NULL);
	printf("simd=%d threshold=%d y=%d patch=%dx%d faint=%d peak=%d at=%d,%d count=%d expected=%d\n",
	       skip_simd, threshold, y0, w, h, faint, peak, px, py, ob->num_blobs, expected);
	if (ob->num_blobs != expected)
		failures++;
	if (ob->num_blobs == 1 && expected == 1) {
		CHECK(ob->blobs[0].area == (unsigned)(w * h));
		CHECK(ob->blobs[0].width == w);
		CHECK(ob->blobs[0].left == 100 && ob->blobs[0].top == y0);
	}
	blobwatch_release_observation(bw, ob);
	blobwatch_free(bw);
}

int main(void)
{
	for (int simd = 0; simd <= 1; simd++) {
		skip_simd = simd;
		for (int peak = 63; peak <= 64; peak++)
			for (int py = 0; py < 4; py++)
				for (int px = 0; px < 4; px++)
					patch(BLOB_THRESHOLD_CV1, 100, 4, 4, 40, peak, px, py, peak >= 64);
		const int thresholds[] = {BLOB_THRESHOLD_CV1, BLOB_THRESHOLD_DK2};
		for (unsigned t = 0; t < sizeof(thresholds) / sizeof(thresholds[0]); t++) {
			int threshold = thresholds[t];
			for (int px = 0; px < 2; px++)
				patch(threshold, 100, 4, 4, threshold + 1, 200, px, 0, 1);
			patch(threshold, 100, 4, 4, threshold, threshold, 0, 0, 0);
			patch(threshold, 100, 1, 1, 200, 200, 0, 0, 0);
			patch(threshold, H - 1, 1, 1, 200, 200, 0, 0, 0);
			/* Narrow multi-row components are not singleton noise. */
			patch(threshold, 100, 1, 2, 200, 200, 0, 0, 1);
			patch(threshold, H - 2, 1, 2, 200, 200, 0, 0, 1);
		}
		patch(BLOB_THRESHOLD_CV1, 100, 4, 4, 40, 40, 0, 0, 0);
		patch(BLOB_THRESHOLD_CV1, 100, 40, 4, 200, 200, 0, 0, 0);
		patch(BLOB_THRESHOLD_CV1, 100, 24, 3, 200, 200, 0, 0, 0);
	}
	printf("brightness mismatches=%d\n", failures);
	return failures != 0;
}
