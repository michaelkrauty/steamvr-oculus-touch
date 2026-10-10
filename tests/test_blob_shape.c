// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

/* Laser-sweep streaks must not become LED candidates: blobwatch keeps round LED-like blobs and
 * drops long thin ones (seen on a sensor with a strong view of Lighthouse sweeps). */
#include <string.h>

/* Unity build: compile the engine sources under test into this binary. */
#include "rift-sensor-blobwatch.c"
#include "rift-sensor-flicker.c"
#include "test_util.h"

#define W 128
#define H 128

static void fill(uint8_t *frame, int x0, int y0, int w, int h)
{
	for (int y = y0; y < y0 + h; y++)
		memset(frame + y * W + x0, 200, (size_t)w);
}

int main(void)
{
	static uint8_t frame[W * H];
	fill(frame, 20, 20, 6, 6);   /* LED-like */
	fill(frame, 60, 80, 24, 3);  /* horizontal laser streak */
	fill(frame, 100, 30, 3, 20); /* vertical laser streak */
	fill(frame, 40, 60, 9, 4);   /* oblong but short of the cut-off ratio: kept */

	blobwatch *bw = blobwatch_new(BLOB_THRESHOLD_CV1);
	blobservation *ob = NULL;
	blobwatch_process(bw, frame, W, H, 0, NULL, 0, &ob);
	CHECK(ob != NULL);
	CHECK(ob->num_blobs == 2);
	/* Identify by width: blobwatch records height one row larger than the pixel extent. */
	int round = 0, oblong = 0;
	for (int i = 0; i < ob->num_blobs; i++) {
		if (ob->blobs[i].width == 6)
			round++;
		if (ob->blobs[i].width == 9)
			oblong++;
	}
	CHECK(round == 1 && oblong == 1);
	blobwatch_release_observation(bw, ob);
	blobwatch_free(bw);
	return 0;
}
