// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* A sensor stream's isochronous transfers cover two frames' worth of packets: the USB thread can
 * stall that long (125 us per packet) before the controller runs out of queued transfers and every
 * sensor loses video at once. At most 32 packets a transfer, as upstream laid them out. */
#include "uvc-layout.h"
#include "test_util.h"

int main(void)
{
	int n, per;
	/* CV1 sensor, raw 1280x960 over USB 3: 75 packets a frame, 3 transfers of 25 a frame */
	uvc_transfer_layout(1280 * 960, 16384, &n, &per);
	CHECK(per == 25 && n == 6);
	/* CV1 sensor over USB 2 (JPEG, 2 KB packets): 600 packets a frame */
	uvc_transfer_layout(1280 * 960, 2048, &n, &per);
	CHECK(per <= 32 && n * per >= 2 * 600 - 2 * 19);
	/* DK2 sensor 752x480: 118 packets a frame */
	uvc_transfer_layout(752 * 480, 3060, &n, &per);
	CHECK(per <= 32 && n == 8 && per == 29);
	puts("test_uvc_layout: ok");
	return 0;
}
