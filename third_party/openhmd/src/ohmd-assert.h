// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
#ifndef OHMD_ASSERT_H
#define OHMD_ASSERT_H

#include <assert.h>

#ifdef _MSC_VER
#define OHMD_NORETURN __declspec(noreturn)
#else
#define OHMD_NORETURN __attribute__((noreturn))
#endif

#if defined(OHMD_NONBLOCK_LOG) || defined(OHMD_NONBLOCK_ASSERT)
#include "openhmd.h"
#ifdef __cplusplus
extern "C" {
#endif
OHMD_NORETURN OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_assert_fail(const char *expression,
        const char *file, int line, const char *function);
#ifdef __cplusplus
}
#endif
#endif

/* A project macro survives later assert.h/cassert includes in C++, headers and unity tests.
 * Keep the build's existing NDEBUG policy; standalone unity builds keep the CRT assertion. */
#if defined(NDEBUG)
#define OHMD_ASSERT(expression) ((void)0)
#elif defined(OHMD_NONBLOCK_LOG) || defined(OHMD_NONBLOCK_ASSERT)
#define OHMD_ASSERT(expression) ((expression) ? (void)0 : ohmd_assert_fail(#expression, __FILE__, __LINE__, __func__))
#else
#define OHMD_ASSERT(expression) assert(expression)
#endif

#endif
