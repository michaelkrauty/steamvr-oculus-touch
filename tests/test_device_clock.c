// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* The controller clock the radio poll and the engine's filter share. */
#include <assert.h>
#include <stdio.h>
#include "rift-cv1touch-clock.h"

static void elapsed_time_keeps_the_full_modular_delta(void)
{
	const uint64_t start = 9000000000ull;
	const uint32_t previous[] = {1000000u, 0xfffff000u};
	/* Last exact 32-bit ns product, first overflowing product, 5 s, and long gaps.
	 * Subtraction must remain modular even when the microsecond counter wraps. */
	const uint32_t gaps[] = {2000u, 4294967u, 4294968u, 5000000u, 3000000000u, UINT32_MAX};
	for (unsigned i = 0; i < sizeof previous / sizeof previous[0]; i++) {
		for (unsigned j = 0; j < sizeof gaps / sizeof gaps[0]; j++) {
			uint32_t next = previous[i] + gaps[j];
			assert(rift_device_time_next(start, previous[i], next) == start + (uint64_t)gaps[j] * 1000);
		}
	}
}

static void initial_epoch_keeps_upstream_truncation(void)
{
	uint64_t t = rift_device_time_next(0, 0, 5000000u);
	assert(t == 705032704ull);
	assert(rift_device_time_next(t, 5000000u, 5002000u) == t + 2000000ull);
}

int main(void)
{
	initial_epoch_keeps_upstream_truncation();
	elapsed_time_keeps_the_full_modular_delta();
	/* the first sample: its timestamp in ns (upstream's 32-bit product: 5 ms is exact) */
	uint64_t t = rift_device_time_next(0, 0, 5000);
	assert(t == 5000000);
	/* the next, 2.5 ms on */
	assert(rift_device_time_next(t, 5000, 7500) == 7500000);
	/* the 32-bit microsecond counter wraps (every ~71.6 min): time still runs forward */
	assert(rift_device_time_next(9000000000ull, 0xfffff000u, 0x00000800u) == 9000000000ull + 0x1800ull * 1000);
	/* A counter restart is also a modular forward delta, not a backward extended epoch. */
	assert(rift_device_time_next(1000000000ull, 1000000u, 2000u) == 4294969296000ull);
	/* the same state and timestamp give the same time: the poll's stamp is the filter's */
	uint64_t a = rift_device_time_next(123456789000ull, 4000000u, 4002500u);
	uint64_t b = rift_device_time_next(123456789000ull, 4000000u, 4002500u);
	assert(a == b && a == 123456789000ull + 2500000);
	puts("test_device_clock: ok");
	return 0;
}
