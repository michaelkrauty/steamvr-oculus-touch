// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#ifndef CV1TOUCH_TEST_EXPECTED_ABORT_H
#define CV1TOUCH_TEST_EXPECTED_ABORT_H

#ifndef _WIN32
#include <stdio.h>
#include <stdlib.h>
#include <sys/resource.h>
#ifdef __linux__
#include <sys/prctl.h>
#include <unistd.h>
#endif
#endif

/* Only in a child whose remaining job is to abort. Never use this in a socket fixture's
 * owner: non-dumpable processes lose same-user access through /proc/<pid>/fd. Setup
 * failures exit with 80 instead of aborting with core dumps still enabled. */
static void prepare_expected_abort(void)
{
#ifndef _WIN32
    const struct rlimit disabled = {0, 0};
    struct rlimit actual;
    if (setrlimit(RLIMIT_CORE, &disabled) != 0 ||
        getrlimit(RLIMIT_CORE, &actual) != 0 || actual.rlim_cur != 0 || actual.rlim_max != 0)
        _Exit(80);
#ifdef __linux__
    /* A zero RLIMIT_CORE alone still invokes a piped core_pattern handler. */
    if (prctl(PR_SET_DUMPABLE, 0L) != 0) _Exit(80);
    const int dumpable = prctl(PR_GET_DUMPABLE);
    const char *directory = getenv("CV1TOUCH_TEST_ABORT_SNAPSHOTS");
    if (directory) {
        char path[4096];
        int len = snprintf(path, sizeof path, "%s/%ld.limits", directory, (long)getpid());
        if (len < 0 || (size_t)len >= sizeof path) _Exit(80);
        FILE *limits = fopen("/proc/self/limits", "r");
        FILE *snapshot = fopen(path, "wx");
        if (!limits || !snapshot) _Exit(80);
        if (fprintf(snapshot, "dumpable: %d\n", dumpable) < 0) _Exit(80);
        char line[256];
        while (fgets(line, sizeof line, limits))
            if (fputs(line, snapshot) == EOF) _Exit(80);
        if (ferror(limits) || fclose(limits) != 0 || fclose(snapshot) != 0) _Exit(80);
    }
    if (dumpable != 0) _Exit(80);
#endif
#endif
}

#endif
