/* cv1touch: how a sensor stream's isochronous transfers are laid out. */
#ifndef UVC_LAYOUT_H
#define UVC_LAYOUT_H

/* Frames' worth of packets the queued transfers cover. A transfer goes back in flight only when the
 * USB event thread, which serves every sensor, gets to its completion: with one frame's worth
 * (9.4 ms for a CV1 sensor at 125 us a packet) a ~6 ms stall of that thread (preempted, or waiting
 * on the tracker's locks) ran every sensor's queue dry and lost video on all of them at once. */
#define UVC_QUEUED_FRAMES 2

/* At most 32 packets a transfer, as many transfers as a frame needs that way, times
 * UVC_QUEUED_FRAMES. */
static inline void uvc_transfer_layout(int frame_size, int packet_size, int *num_transfers, int *packets_per_transfer)
{
	int packets = (frame_size + packet_size - 1) / packet_size;
	int per_frame = (packets + 31) / 32;
	*packets_per_transfer = packets / per_frame;
	*num_transfers = per_frame * UVC_QUEUED_FRAMES;
}

#endif
