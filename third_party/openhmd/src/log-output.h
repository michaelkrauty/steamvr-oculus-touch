// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
#ifndef OHMD_LOG_OUTPUT_H
#define OHMD_LOG_OUTPUT_H
#include <stdio.h>

/* Unity tests without the engine keep their ordinary stdio diagnostics. */
#ifdef OHMD_NONBLOCK_LOG
#ifdef __cplusplus
extern "C" {
#endif
int ohmd_log_printf(const char *format, ...);
int ohmd_log_fprintf(FILE *stream, const char *format, ...);
int ohmd_log_puts(const char *text);
#ifdef __cplusplus
}
#endif
#else
#define ohmd_log_printf printf
#define ohmd_log_fprintf fprintf
#define ohmd_log_puts puts
#endif
#endif
