// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

#include "log.h"

#include <atomic>
#include <cstdarg>
#include <cstdio>

#ifndef CV1T_DRIVER_TEST
#include <openvr_driver.h>
#endif

static std::atomic<bool> g_ready{false};

void cv1t_log_ready() { g_ready = true; }

void cv1t_log(const char *fmt, ...)
{
	char buf[512];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof buf, fmt, ap);
	va_end(ap);
#ifndef CV1T_DRIVER_TEST
	if (g_ready && vr::VRDriverLog()) {
		vr::VRDriverLog()->Log(buf);
		return;
	}
#endif
	fprintf(stderr, "%s\n", buf);
}
