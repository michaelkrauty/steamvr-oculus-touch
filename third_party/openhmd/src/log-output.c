// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb.h>
#include "openhmd_cv1touch.h"
#include "log-output.h"
#include "ohmd-assert.h"

#define LOG_BYTES 2048
static ohmd_cv1touch_log_cb log_callback;
static ohmd_cv1touch_log_fatal_cb log_fatal_callback;
static _Thread_local char log_line[LOG_BYTES];
static _Thread_local size_t log_len;
static _Thread_local bool log_truncated;
static _Thread_local bool in_receiver;
static _Thread_local bool in_fatal;

static OHMD_NORETURN void abort_silently(void)
{
#ifdef _WIN32
    /* UCRT abort() can otherwise write another message or open a crash-report dialog. */
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
    abort();
}

static void check_entry(void)
{
    if (in_receiver || in_fatal) abort_silently();
}

static void receive_line(const char *bytes, size_t len)
{
    if (in_receiver) abort_silently();
    in_receiver = true;
    log_callback(bytes, len);
    in_receiver = false;
}

static OHMD_NORETURN void finish_fatal(void)
{
    if (log_fatal_callback) log_fatal_callback();
    abort_silently();
}

static void finish_line(void)
{
    static const char marker[] = " [truncated]";
    if (log_truncated) {
        memcpy(log_line + log_len, marker, sizeof marker - 1);
        log_len += sizeof marker - 1;
    }
    log_line[log_len++] = '\n';
    size_t len = log_len;
    log_len = 0;
    log_truncated = false;
    receive_line(log_line, len);
}

static void log_bytes(const char *bytes, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if (bytes[i] == '\n') {
            finish_line();
        } else if (log_len < LOG_BYTES - sizeof(" [truncated]")) {
            log_line[log_len++] = bytes[i];
        } else {
            log_truncated = true;
        }
    }
}

static int log_vfprintf(FILE *stream, const char *format, va_list args)
{
    check_entry();
    if (!log_callback) return vfprintf(stream, format, args);
    char bytes[LOG_BYTES];
    int n = vsnprintf(bytes, sizeof bytes, format, args);
    if (n < 0) return n;
    size_t len = (size_t)n < sizeof bytes ? (size_t)n : sizeof bytes - 1;
    log_bytes(bytes, len);
    if ((size_t)n >= sizeof bytes) {
        log_truncated = true;
        finish_line();
    }
    return n;
}

int ohmd_log_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int n = log_vfprintf(stdout, format, args);
    va_end(args);
    return n;
}

int ohmd_log_fprintf(FILE *stream, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int n = log_vfprintf(stream, format, args);
    va_end(args);
    return n;
}

int ohmd_log_puts(const char *text)
{
    check_entry();
    if (!log_callback) return puts(text);
    log_bytes(text, strlen(text));
    log_bytes("\n", 1);
    return 0;
}

static void LIBUSB_CALL usb_log(libusb_context *ctx, enum libusb_log_level level, const char *text)
{
    check_entry();
    (void)ctx;
    (void)level;
    log_bytes(text, strlen(text));
}

OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_cv1touch_set_log_callback(ohmd_cv1touch_log_cb callback)
{
    /* Installed before workers start, and immutable while they run. TLS joins printf fragments
     * into complete lines, so different threads cannot split a prefix from its message. */
    log_callback = callback;
    /* The global handler replaces libusb's default stderr writer, including during init.
     * libusb builds with ENABLE_DEBUG_LOGGING or USE_SYSTEM_LOGGING_FACILITY can bypass this
     * callback and still write synchronously. No process-wide fd flags are changed for them. */
    libusb_set_log_cb(NULL, callback ? usb_log : NULL, LIBUSB_LOG_CB_GLOBAL);
}

OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_cv1touch_set_log_fatal_callback(ohmd_cv1touch_log_fatal_cb callback)
{
    log_fatal_callback = callback;
}

OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_assert_fail(const char *expression,
        const char *file, int line, const char *function)
{
    check_entry();
    in_fatal = true;
    /* The CRT's assertion printer can block before abort(), even with our normal log callback.
     * Format locally and call only the bounded receiver/drain. With no receiver, still abort. */
    if (log_callback) {
        /* Never append to TLS: a receiver may have failed with a full truncated line there. */
        char bytes[LOG_BYTES];
        int n = snprintf(bytes, sizeof bytes, "%s:%d: %s: Assertion `%s' failed.\n",
                file, line, function, expression);
        if (n > 0) {
            size_t len = (size_t)n;
            if (len >= sizeof bytes) {
                static const char marker[] = " [truncated]\n";
                memcpy(bytes + sizeof bytes - (sizeof marker - 1), marker, sizeof marker - 1);
                len = sizeof bytes;
            }
            receive_line(bytes, len);
        }
    }
    finish_fatal();
}

OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_cv1touch_abort(void)
{
    check_entry();
    in_fatal = true;
    finish_fatal();
}
