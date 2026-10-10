/* cv1touch: the controller clock a tracked device keeps. Its IMU timestamps are 32-bit
 * microseconds; the engine extends them to 64-bit nanoseconds as each sample reaches its filter.
 * Shared so the radio poll can stamp a sample before the filter sees it, with the same value
 * the filter will give it. */
#ifndef RIFT_CV1TOUCH_CLOCK_H
#define RIFT_CV1TOUCH_CLOCK_H

#include <stdint.h>

/* The device time of `device_ts`, after the newest sample at `device_time_ns` (0: none yet),
 * stamped `last_device_ts`. The arithmetic is upstream's, 32-bit products included. */
static inline uint64_t rift_device_time_next(uint64_t device_time_ns, uint32_t last_device_ts, uint32_t device_ts)
{
	if (device_time_ns == 0)
		return device_ts * 1000;
	return device_time_ns + ((uint32_t)(device_ts - last_device_ts)) * 1000;
}

#endif
