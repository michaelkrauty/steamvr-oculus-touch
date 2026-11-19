// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
/* Real optional-debug ownership paths, with no PipeWire server or worker
 * threads. */
#include "test_util.h"
#include <pipewire/pipewire.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
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
/* The shared connection's start can be held while another thread asks for it. */
static pthread_mutex_t gate = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t gate_changed = PTHREAD_COND_INITIALIZER;
static bool hold_start, starting;
static int start_loop(void) {
  CHECK(pthread_mutex_lock(&gate) == 0);
  starting = true;
  CHECK(pthread_cond_broadcast(&gate_changed) == 0);
  struct timespec until;
  CHECK(clock_gettime(CLOCK_REALTIME, &until) == 0);
  until.tv_nsec += 300000000;
  if (until.tv_nsec >= 1000000000) {
    until.tv_sec++;
    until.tv_nsec -= 1000000000;
  }
  while (hold_start && pthread_cond_timedwait(&gate_changed, &gate, &until) == 0) {
  }
  starting = false;
  CHECK(pthread_mutex_unlock(&gate) == 0);
  return maybe_fail();
}
#define calloc(n, size) acquire((n) * (size))
#define free(p) release(p)
#define pw_init(a, b) ((void)0)
#define pw_loop_new(props) ((struct pw_loop *)acquire(1))
#define pw_thread_loop_new_full(loop, name, props)                             \
  ((struct pw_thread_loop *)acquire(1))
#define pw_context_new(loop, props, size) ((struct pw_context *)acquire(1))
#define pw_context_connect(ctx, props, size) ((struct pw_core *)acquire(1))
#define pw_thread_loop_start(thread) start_loop()
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
static void *first_user(void *arg) {
  (void)arg;
  CHECK(ohmd_pipewire_global_init());
  return NULL;
}

/* A sensor's streams are made on the engine's live-sensor worker while the engine thread can
 * make a controller's: a second user never gets the shared connection before it has started. */
static void a_second_user_waits_for_the_connection(void) {
  calls = live = 0;
  fail_at = 0;
  hold_start = true;
  pthread_t first;
  CHECK(pthread_create(&first, NULL, first_user, NULL) == 0);
  CHECK(pthread_mutex_lock(&gate) == 0);
  while (!starting)
    CHECK(pthread_cond_wait(&gate_changed, &gate) == 0);
  CHECK(pthread_mutex_unlock(&gate) == 0);
  ohmd_pw_global_data *second = ohmd_pipewire_global_init();
  CHECK(pthread_mutex_lock(&gate) == 0);
  bool still_starting = starting;
  CHECK(pthread_mutex_unlock(&gate) == 0);
  CHECK(second && !still_starting);
  CHECK(pthread_join(first, NULL) == 0);
  hold_start = false;
  CHECK(ohmd_pw_global_data_ptr == second && second->use_count == 2);
  ohmd_pipewire_global_deinit();
  ohmd_pipewire_global_deinit();
  CHECK(live == 0 && ohmd_pw_global_data_ptr == NULL);
}

/* State callbacks and USB boundary reads run on different threads. */
static void *toggle_connection(void *arg) {
  ohmd_pw_video_stream *v = arg;
  for (int i = 0; i < 10000; i++) {
    on_stream_state_changed(&v->base, PW_STREAM_STATE_PAUSED, PW_STREAM_STATE_STREAMING, NULL);
    on_stream_state_changed(&v->base, PW_STREAM_STATE_STREAMING, PW_STREAM_STATE_PAUSED, NULL);
  }
  return NULL;
}

static void connection_is_safe_to_read_concurrently(void) {
  ohmd_pw_video_stream v = {0};
  pthread_t thread;
  CHECK(pthread_create(&thread, NULL, toggle_connection, &v) == 0);
  for (int i = 0; i < 10000; i++)
    (void)ohmd_pw_video_stream_connected(&v);
  CHECK(pthread_join(thread, NULL) == 0);
  CHECK(!ohmd_pw_video_stream_connected(&v));
  on_stream_state_changed(&v.base, PW_STREAM_STATE_PAUSED, PW_STREAM_STATE_STREAMING, NULL);
  CHECK(ohmd_pw_video_stream_connected(&v));
}

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
  a_second_user_waits_for_the_connection();
  connection_is_safe_to_read_concurrently();
  puts("test_pipewire_lifetime: ok");
  return 0;
}
