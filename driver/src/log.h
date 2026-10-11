// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#pragma once
/* Into the vrserver log once the driver context exists (cv1t_log_ready), stderr before that and
 * in tests. Never blocks on anything but the log call itself. */
void cv1t_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void cv1t_log_ready();
