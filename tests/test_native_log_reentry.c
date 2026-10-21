// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include <string.h>
#include "support/expected_abort.h"
#include "../third_party/openhmd/src/log-output.c"

static unsigned calls;
static bool partial;

static void asserting_receiver(const char *bytes, size_t len)
{
    (void)bytes;
    (void)len;
    if (++calls > 1) _Exit(77);
    OHMD_ASSERT(0 && "receiver assertion");
}

static void collect(const char *bytes, size_t len)
{
    if (partial && (len < 10 || memcmp(bytes, "unfinished", 10) == 0)) _Exit(79);
}

static void recursive_fatal(void)
{
    if (++calls > 1) _Exit(78);
    OHMD_ASSERT(0 && "fatal callback assertion");
}

int main(int argc, char **argv)
{
    if (argc != 2) return 1;
    if (strcmp(argv[1], "fatal") == 0) {
        ohmd_cv1touch_set_log_callback(collect);
        ohmd_cv1touch_set_log_fatal_callback(recursive_fatal);
        prepare_expected_abort();
        OHMD_ASSERT(0 && "outer assertion");
    } else if (strcmp(argv[1], "partial") == 0) {
        partial = true;
        ohmd_cv1touch_set_log_callback(collect);
        ohmd_log_printf("unfinished fragment");
        prepare_expected_abort();
        OHMD_ASSERT(0 && "independent fatal record");
    } else {
        ohmd_cv1touch_set_log_callback(asserting_receiver);
        if (strcmp(argv[1], "truncated") == 0) {
            char text[4096];
            memset(text, 'x', sizeof text - 1);
            text[sizeof text - 1] = '\0';
            prepare_expected_abort();
            ohmd_log_printf("%s\n", text);
        } else {
            prepare_expected_abort();
            ohmd_log_printf("short record\n");
        }
    }
    return 1;
}
