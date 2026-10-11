// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty
/* Real optional-debug ownership paths, with no PipeWire server or worker
 * threads. */
#include "test_util.h"
#include <pipewire/pipewire.h>
#include <stdlib.h>
static int calls, fail_at, live;
static void *acquire(size_t size) {
  if (++calls == fail_at)
    return NULL;
  void *p = calloc(1, size);
  CHECK(p);
  live++;
  return p;
}
static void release(void *p) {
  if (p) {
    live--;
    free(p);
  }
}
static void *stream_new(struct pw_properties *p) {
  pw_properties_free(p);
  return acquire(1);
}
static int maybe_fail(void) { return ++calls == fail_at ? -1 : 0; }
#define calloc(n, size) acquire((n) * (size))
#define free(p) release(p)
#define pw_init(a, b) ((void)0)
#define pw_loop_new(props) ((struct pw_loop *)acquire(1))
#define pw_thread_loop_new_full(loop, name, props)                             \
  ((struct pw_thread_loop *)acquire(1))
#define pw_context_new(loop, props, size) ((struct pw_context *)acquire(1))
#define pw_context_connect(ctx, props, size) ((struct pw_core *)acquire(1))
#define pw_thread_loop_start(thread) maybe_fail()
#define pw_thread_loop_stop(thread) ((void)0)
#define pw_thread_loop_lock(thread) ((void)0)
#define pw_thread_loop_unlock(thread) ((void)0)
#define pw_core_disconnect(core) release(core)
#define pw_context_destroy(ctx) release(ctx)
#define pw_thread_loop_destroy(thread) release(thread)
#define pw_loop_destroy(loop) release(loop)
#define pw_stream_new(core, name, props) ((struct pw_stream *)stream_new(props))
#define pw_stream_add_listener(stream, listener, events, data) ((void)0)
#define pw_stream_connect(...) maybe_fail()
#define pw_stream_disconnect(stream) ((void)0)
#define pw_stream_destroy(stream) release(stream)
#include "ohmd-pipewire.c"
#undef calloc
#undef free
int main(void) {
  for (int fail = 1; fail <= 6; fail++) {
    calls = live = 0;
    fail_at = fail;
    CHECK(ohmd_pipewire_global_init() == NULL);
    CHECK(live == 0 && ohmd_pw_global_data_ptr == NULL);
  }
  for (int kind = 0; kind < 2; kind++) {
    for (int fail = 1; fail <= 10; fail++) {
      calls = live = 0;
      fail_at = fail;
      ohmd_pw_base_stream *s =
          kind ? (ohmd_pw_base_stream *)ohmd_pw_debug_stream_new("TEST", "TEST")
               : (ohmd_pw_base_stream *)ohmd_pw_video_stream_new(
                     "TEST", "TEST", OHMD_PW_VIDEO_FORMAT_RGB, 64, 64, 60, 1);
      if (s)
        ohmd_pw_stream_free_common(s);
      CHECK(live == 0 && ohmd_pw_global_data_ptr == NULL);
    }
  }
  calls = live = 0;
  fail_at = 0;
  CHECK(ohmd_pipewire_global_init());
  CHECK(ohmd_pipewire_global_init());
  ohmd_pipewire_global_deinit();
  CHECK(live > 0);
  ohmd_pipewire_global_deinit();
  CHECK(live == 0);
  CHECK(ohmd_pw_video_stream_new("TEST", "TEST", (ohmd_pw_video_format)99, 64,
                                 64, 60, 1) == NULL);
  CHECK(live == 0 && ohmd_pw_global_data_ptr == NULL);
  puts("test_pipewire_lifetime: ok");
  return 0;
}
