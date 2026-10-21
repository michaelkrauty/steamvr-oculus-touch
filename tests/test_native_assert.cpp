// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

// Internal headers must keep the project assertion even if C++ includes cassert afterwards.
#include "../third_party/openhmd/src/omath.h"
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include "support/expected_abort.h"
#ifdef OHMD_NONBLOCK_ASSERT
#include "openhmd_cv1touch.h"

static void collect(const char *bytes, size_t len)
{
    std::fwrite(bytes, 1, len, stderr);
}

static void drain()
{
    std::fputs("fatal drain called\n", stderr);
}
#endif

int main(int argc, char **)
{
    int evaluated = 0;
    OHMD_ASSERT(++evaluated == 1);
    if (evaluated != 1) return 1;
    if (argc == 1) return 0;
#ifdef _WIN32
    // Standalone CRT assertions use stderr instead of opening a test-runner dialog.
    _set_error_mode(_OUT_TO_STDERR);
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
#ifdef OHMD_NONBLOCK_ASSERT
    ohmd_cv1touch_set_log_callback(collect);
    ohmd_cv1touch_set_log_fatal_callback(drain);
#endif
    prepare_expected_abort();
    OHMD_ASSERT(argc == 0);
    return 1;
}
