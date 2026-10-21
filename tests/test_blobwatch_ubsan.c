// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* The first processed scanline has no previous extents. UBSan must reject a range formed from
 * that null previous line, even when a dark first line would skip the overlap loops. */
#include "rift-sensor-blobwatch.c"
#include "rift-sensor-flicker.c"
#include "test_util.h"

#define W 64
#define H 16

static void check_frames(bool simd, bool lit_first_line)
{
	uint8_t frame[W * H] = {0};
	const int first_y = lit_first_line ? 2 : 6;
	for (int y = first_y; y < first_y + 3; y++)
		memset(frame + y * W + 4, 200, 4);
	for (int y = H - 2; y < H; y++)
		memset(frame + y * W + 40, 200, 4);

	skip_simd = simd;
	blobwatch *bw = blobwatch_new(BLOB_THRESHOLD_CV1);
	CHECK(bw != NULL);
	for (int frame_index = 0; frame_index < 4; frame_index++) {
		blobservation *ob = NULL;
		blobwatch_process(bw, frame, W, H, 0, NULL, 0, &ob);
		CHECK(ob != NULL);
		CHECK(ob->num_blobs == 2);
		CHECK(ob->blobs[0].left == 4 && ob->blobs[0].top == first_y);
		CHECK(ob->blobs[0].width == 4 && ob->blobs[0].area == 12);
		CHECK_NEAR(ob->blobs[0].x, 6.0, 0.000001);
		CHECK_NEAR(ob->blobs[0].y, lit_first_line ? 3.5 : 7.5, 0.000001);
		CHECK(ob->blobs[1].left == 40 && ob->blobs[1].top == H - 2);
		CHECK(ob->blobs[1].width == 4 && ob->blobs[1].area == 8);
		CHECK_NEAR(ob->blobs[1].x, 42.0, 0.000001);
		CHECK_NEAR(ob->blobs[1].y, 15.0, 0.000001);
		CHECK(ob->blobs[0].age == (uint32_t)frame_index);
		CHECK(ob->blobs[1].age == (uint32_t)frame_index);
		blobwatch_release_observation(bw, ob);
	}
	blobwatch_free(bw);
}

int main(void)
{
	check_frames(false, false);
	check_frames(false, true);
	check_frames(true, false);
	check_frames(true, true);
	return 0;
}
