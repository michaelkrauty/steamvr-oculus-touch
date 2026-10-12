/* cv1touch: how a sensor stream's isochronous transfers are laid out. */
#ifndef UVC_LAYOUT_H
#define UVC_LAYOUT_H

#include <stdbool.h>

/* Frames' worth of packets the queued transfers cover. A transfer goes back in flight only when the
 * USB event thread, which serves every sensor, gets to its completion: with one frame's worth
 * (9.4 ms for a CV1 sensor at 125 us a packet) a ~6 ms stall of that thread (preempted, or waiting
 * on the tracker's locks) ran every sensor's queue dry and lost video on all of them at once. */
#define UVC_QUEUED_FRAMES 2

/* libusb's WinUSB backend rejects an isochronous transfer that is not whole 1 ms bus frames: with a
 * packet every 125 us microframe, a multiple of 8 packets. */
#ifdef _WIN32
#define UVC_WHOLE_MS_TRANSFERS true
#else
#define UVC_WHOLE_MS_TRANSFERS false
#endif

/* At most 32 packets a transfer, as many transfers as a frame needs that way, times
 * UVC_QUEUED_FRAMES. With whole_ms, 32-packet transfers (4 ms) covering UVC_QUEUED_FRAMES frames. */
static inline void uvc_transfer_layout(int frame_size, int packet_size, bool whole_ms, int *num_transfers, int *packets_per_transfer)
{
	int packets = (frame_size + packet_size - 1) / packet_size;
	if (whole_ms) {
		*packets_per_transfer = 32;
		*num_transfers = (packets * UVC_QUEUED_FRAMES + 31) / 32;
		return;
	}
	int per_frame = (packets + 31) / 32;
	*packets_per_transfer = packets / per_frame;
	*num_transfers = per_frame * UVC_QUEUED_FRAMES;
}

#endif
