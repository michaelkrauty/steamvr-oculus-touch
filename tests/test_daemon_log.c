// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* The actual native stdio adapter, including a normal libusb log callback, without USB access. */
#include <string.h>
#include "../third_party/openhmd/src/openhmdi.h"
#include "../third_party/openhmd/src/log-output.c"
#include "test_util.h"

static char received[8192];
static size_t used;
static unsigned calls;

static void collect(const char *bytes, size_t len)
{
    CHECK(used + len < sizeof received);
    memcpy(received + used, bytes, len);
    used += len;
    received[used] = '\0';
    calls++;
}

int main(void)
{
    ohmd_cv1touch_set_log_callback(collect);
    LOGI("first %d", 1);
    ohmd_log_printf("a");
    ohmd_log_fprintf(stderr, "b\n");
    usb_log(NULL, LIBUSB_LOG_LEVEL_WARNING, "USB warning\n");
    CHECK(strcmp(received, "[II] first 1\nab\nUSB warning\n") == 0);
    CHECK(calls == 3);
    char long_line[8000];
    memset(long_line, 'x', sizeof long_line - 1);
    long_line[sizeof long_line - 1] = '\0';
    ohmd_log_printf("%s\n", long_line);
    CHECK(calls == 4);
    CHECK(strstr(received, " [truncated]\n") != NULL);
    CHECK(used < 4096);
    ohmd_log_printf("after\n");
    CHECK(calls == 5);
    CHECK(strcmp(received + used - 6, "after\n") == 0);
    ohmd_cv1touch_set_log_callback(NULL);
    return 0;
}
