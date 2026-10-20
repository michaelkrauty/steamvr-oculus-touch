// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Internal Interface for Platform Specific Functions */


#ifndef PLATFORM_H
#define PLATFORM_H

#include "openhmd.h"

#ifdef _WIN32
#include <stdlib.h>
/* cv1touch: glibc's qsort_r, on MSVC's qsort_s, which passes the context first */
typedef struct {
	int (*compare)(const void *, const void *, void *);
	void *arg;
} ohmd_qsort_r_context;

static inline int ohmd_qsort_r_compare(void *context, const void *a, const void *b)
{
	ohmd_qsort_r_context *c = (ohmd_qsort_r_context *)context;
	return c->compare(a, b, c->arg);
}

static inline void qsort_r(void *base, size_t n, size_t size, int (*compare)(const void *, const void *, void *), void *arg)
{
	ohmd_qsort_r_context c = { compare, arg };
	qsort_s(base, n, size, ohmd_qsort_r_compare, &c);
}
#endif

double ohmd_get_tick();
void ohmd_toggle_ovr_service(int state);

typedef struct ohmd_thread ohmd_thread;
typedef struct ohmd_mutex ohmd_mutex;
typedef struct ohmd_cond ohmd_cond;

ohmd_mutex* ohmd_create_mutex(ohmd_context* ctx);
void ohmd_destroy_mutex(ohmd_mutex* mutex);

void ohmd_lock_mutex(ohmd_mutex* mutex);
void ohmd_unlock_mutex(ohmd_mutex* mutex);

ohmd_cond* ohmd_create_cond(ohmd_context* ctx);
void ohmd_destroy_cond(ohmd_cond* cond);

void ohmd_cond_wait(ohmd_cond* cond, ohmd_mutex* mutex);
void ohmd_cond_signal(ohmd_cond* cond);
void ohmd_cond_broadcast(ohmd_cond* cond);

ohmd_thread* ohmd_create_thread(ohmd_context* ctx, unsigned int (*routine)(void* arg), void* arg);
void ohmd_destroy_thread(ohmd_thread* thread);

/* cv1touch: names the calling thread (Linux: 15 characters), so the daemon and profilers find it */
void ohmd_set_thread_name(const char* name);

/* Directory creation and writes return zero on success, a negative errno on failure.
 * Writes publish a sibling temporary file by rename only after it is written and closed, so a
 * reader sees the old file or the complete new one. Nothing is synced to disk: callers on the
 * engine thread must not wait for it, and a cache torn by a power cut fails to parse. */
int ohmd_ensure_path(const char* pathname);
int ohmd_write_file(const char* filename, char *buf, unsigned long buf_len);
/* Reads return zero on success, nonzero on failure. */
int ohmd_read_file(const char* filename, char **out_buf, unsigned long *out_len);

/* String functions */

int findEndPoint(char* path, int endpoint);

#endif
