// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

/* The controller clock the radio poll and the engine's filter share. */
#include <assert.h>
#include <stdio.h>
#include "rift-cv1touch-clock.h"

int main(void)
{
	/* the first sample: its timestamp in ns (upstream's 32-bit product: 5 ms is exact) */
	uint64_t t = rift_device_time_next(0, 0, 5000);
	assert(t == 5000000);
	/* the next, 2.5 ms on */
	assert(rift_device_time_next(t, 5000, 7500) == 7500000);
	/* the 32-bit microsecond counter wraps (every ~71.6 min): time still runs forward */
	assert(rift_device_time_next(9000000000ull, 0xfffff000u, 0x00000800u) == 9000000000ull + 0x1800ull * 1000);
	/* the same state and timestamp give the same time: the poll's stamp is the filter's */
	uint64_t a = rift_device_time_next(123456789000ull, 4000000u, 4002500u);
	uint64_t b = rift_device_time_next(123456789000ull, 4000000u, 4002500u);
	assert(a == b && a == 123456789000ull + 2500000);
	puts("test_device_clock: ok");
	return 0;
}
