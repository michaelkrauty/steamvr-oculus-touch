// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* cv1touch shared-memory ABI between cv1touchd (single writer) and its readers (SteamVR driver,
 * tools). All host timestamps are monotonic ns: CLOCK_MONOTONIC on Linux, QueryPerformanceCounter
 * on Windows (see cv1t_now_ns). This header is the single definition:
 * Rust gets it through bindgen (src/ffi.rs); C/C++ readers include it and may use the inline
 * helpers below. Bump CV1T_SHM_VERSION on any layout change. */
#ifndef CV1TOUCH_SHM_H
#define CV1TOUCH_SHM_H

#include <limits.h>
#include <stddef.h>
#include <stdint.h>
/* Rust's bindings need only the types: CV1T_SHM_TYPES_ONLY leaves out the platform headers and the
 * inline helpers, so they generate for any target. */
#ifndef CV1T_SHM_TYPES_ONLY
#include <string.h>
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <linux/futex.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#endif
#endif

/* Linux: the POSIX shared-memory object CV1T_SHM_NAME; readers wait on update_futex.
 * Windows: the file mapping Local\cv1touch-shm-<name> (<name> = CV1T_SHM_NAME without its "/"),
 * owned through the mutex Local\cv1touch-owner-<name>. Publish n (the new update_futex value) sets
 * the manual-reset event Local\cv1touch-update<n % 2>-<name> and resets the other, so a reader that
 * saw n waits on update<(n + 1) % 2> (cv1t_shm_wait_events). */
#define CV1T_SHM_NAME "/cv1touch"
#define CV1T_SHM_MAGIC 0x54315643u /* "CV1T" little-endian */
#define CV1T_SHM_VERSION 4u
/* Legacy sensor diagnostics prefix; complete diagnostics are available via control JSON.
 * This is not a tracking/acquisition limit. Keep the controller/alignment ABI fixed. */
#define CV1T_MAX_SENSORS 8

enum { CV1T_HAND_LEFT = 0, CV1T_HAND_RIGHT = 1, CV1T_HAND_COUNT = 2 };

/* cv1t_controller_state.flags */
#define CV1T_FLAG_CALIBRATED (1u << 0) /* factory calibration read: controller paired and seen */
#define CV1T_FLAG_CONNECTED (1u << 1)  /* IMU samples arriving, or held through a gap (below) */
#define CV1T_FLAG_POSITION (1u << 2)   /* optical position fix within the hold window */
/* With CONNECTED: no IMU sample for over 100 ms, for at most 10 s (a radio gap). The state is the
 * controller as it was at its last sample, inputs held, not moving, without a position: keep it
 * in the application as it was. Readers that do not know this flag see an old sample. */
#define CV1T_FLAG_RADIO_GAP (1u << 3)
/* A gap that began still and received sleep beacons. Informational only: RADIO_GAP and CONNECTED
 * keep their meaning, including the full input hold. Older readers ignore this additive flag. */
#define CV1T_FLAG_ASLEEP (1u << 4)

#define CV1T_BTN_A (1u << 0)
#define CV1T_BTN_B (1u << 1)
#define CV1T_BTN_X (1u << 2)
#define CV1T_BTN_Y (1u << 3)
#define CV1T_BTN_SYSTEM (1u << 4) /* right: Oculus button, left: menu button */
#define CV1T_BTN_STICK (1u << 5)

#define CV1T_TOUCH_A (1u << 0)
#define CV1T_TOUCH_B (1u << 1)
#define CV1T_TOUCH_X (1u << 2)
#define CV1T_TOUCH_Y (1u << 3)
#define CV1T_TOUCH_TRIGGER (1u << 4)
#define CV1T_TOUCH_STICK (1u << 5)
#define CV1T_TOUCH_THUMBREST (1u << 6)

enum {
	CV1T_CAP_STICK = 0,
	CV1T_CAP_BY = 1,
	CV1T_CAP_TRIGGER = 2,
	CV1T_CAP_AX = 3,
	CV1T_CAP_THUMBREST = 4,
	CV1T_CAP_COUNT = 5,
};

typedef struct cv1t_controller_state {
	uint64_t sample_host_ns;            /* host-clock estimate of the newest IMU sample */
	uint64_t received_host_ns;          /* when its radio report was read */
	uint64_t publish_host_ns;           /* when the daemon published this snapshot */
	uint64_t last_position_fix_host_ns; /* newest optical position fix, 0 = never */
	uint64_t device_time_ns;            /* controller clock of the newest IMU sample */
	float orient[4];                    /* x y z w, engine output pose */
	float pos[3];                       /* metres */
	float raw_orient[4];                /* unfiltered UKF pose */
	float raw_pos[3];
	float lin_vel[3];                   /* m/s world, as tracked (and shaped for SteamVR's prediction) */
	float ang_vel[3];                   /* the world rate w as body (q^-1 w) + gyro_bias, so that
	                                     * ang_vel - gyro_bias is the body rate SteamVR integrates and
	                                     * q (ang_vel - gyro_bias) = w; as tracked (and shaped) */
	float lin_acc[3];
	float gyro_raw[3];                  /* rad/s IMU frame, bias not removed */
	float gyro_bias[3];
	float imu_accel[3];                 /* newest IMU sample, factory-calibrated, IMU frame: m/s^2 */
	float imu_gyro[3];                  /* same sample: rad/s */
	float pos_stddev[3];
	float rot_stddev[3];
	float trigger;                      /* 0..1 */
	float grip;                         /* 0..1 */
	float stick[2];                     /* -1..1 */
	float cap[CV1T_CAP_COUNT];          /* raw normalized capacitive values */
	uint32_t flags;
	uint32_t buttons;
	uint32_t touches;
	int32_t battery_percent;            /* 0..100 from the radio's battery channel; -1: not known yet */
} cv1t_controller_state;

typedef struct cv1t_slot {
	uint32_t seq; /* seqlock: odd while the writer is mid-update */
	uint32_t reserved;
	cv1t_controller_state state;
} cv1t_slot;

typedef struct cv1t_sensor_status {
	char serial[32];
	uint32_t placed;
	uint32_t frames;                      /* cumulative */
	uint32_t pose_fixes[CV1T_HAND_COUNT]; /* cumulative accepted optical fixes, indexed by CV1T_HAND_* */
	float camera_orient[4];               /* camera -> world */
	float camera_pos[3];
	uint32_t reserved;
} cv1t_sensor_status;

/* cv1t_alignment.valid */
#define CV1T_ALIGN_LIGHTHOUSE (1u << 0)   /* lighthouse_orient/pos are set */
#define CV1T_ALIGN_RENDER_LEFT (1u << 1)  /* render_*[CV1T_HAND_LEFT] are set */
#define CV1T_ALIGN_RENDER_RIGHT (1u << 2) /* render_*[CV1T_HAND_RIGHT] are set */
#define CV1T_ALIGN_PREDICTION (1u << 3)   /* prediction_* are set; otherwise SteamVR gets all of it */

/* How the SteamVR driver places a controller: lighthouse_from_world * world_from_model (the
 * published orient/pos) * model_from_render[hand]. The daemon publishes it from
 * registration.json and render-offsets.json, under cv1t_shm.alignment_seq. */
typedef struct cv1t_alignment {
	uint64_t universe_id;                    /* SteamVR universe of the registration, 0 = unknown */
	uint32_t valid;                          /* CV1T_ALIGN_* */
	uint32_t prediction_position;            /* percent of lin_vel SteamVR extrapolates with, 0..100 */
	float lighthouse_orient[4];              /* Lighthouse raw universe <- cv1touch world, x y z w */
	float lighthouse_pos[3];
	float render_orient[CV1T_HAND_COUNT][4]; /* LED model frame <- render model frame, x y z w */
	float render_pos[CV1T_HAND_COUNT][3];
	uint32_t prediction_rotation;            /* percent of the angular rate, 0..100 */
} cv1t_alignment;

typedef struct cv1t_shm {
	uint32_t magic;          /* written last on create, zeroed on daemon exit */
	uint32_t version;
	uint32_t size;
	uint32_t update_futex;   /* incremented after each publish batch; futex word */
	uint64_t generation;     /* random per daemon start */
	uint64_t heartbeat_host_ns;
	uint32_t daemon_pid;
	uint32_t sensor_count;   /* compatibility prefix count, protected by sensor_seq */
	uint32_t sensor_seq;
	uint32_t reserved;
	cv1t_slot hands[CV1T_HAND_COUNT];
	cv1t_sensor_status sensors[CV1T_MAX_SENSORS];
	uint32_t alignment_seq;
	uint32_t reserved2;
	cv1t_alignment alignment;
} cv1t_shm;

#ifdef __cplusplus
#define CV1T_STATIC_ASSERT(c, m) static_assert(c, m)
#else
#define CV1T_STATIC_ASSERT(c, m) _Static_assert(c, m)
#endif
CV1T_STATIC_ASSERT(sizeof(cv1t_controller_state) == 256, "cv1t_controller_state ABI");
CV1T_STATIC_ASSERT(sizeof(cv1t_slot) == 264, "cv1t_slot ABI");
CV1T_STATIC_ASSERT(sizeof(cv1t_sensor_status) == 80, "cv1t_sensor_status ABI");
CV1T_STATIC_ASSERT(sizeof(cv1t_alignment) == 104, "cv1t_alignment ABI");
CV1T_STATIC_ASSERT(sizeof(cv1t_shm) == 1328, "cv1t_shm ABI");

/* ---- Reader rules ----
 * - fstat() the segment and require st_size >= sizeof(cv1t_shm) before mmap(): the daemon creates the
 *   object, then sizes it, then publishes magic; mapping an unsized object SIGBUSes on first access.
 *   On Windows the mapping is created at full size; check the view (VirtualQuery) all the same.
 * - Trust nothing until magic == CV1T_SHM_MAGIC (acquire load) and version/size match.
 * - A daemon exit zeroes magic; a crash leaves magic set, so heartbeat_host_ns age is the crash signal.
 *   A restarted daemon creates a fresh segment: re-open by name when the heartbeat goes stale.
 * - Gate all controller inputs and pose use on CV1T_FLAG_CONNECTED (and position on CV1T_FLAG_POSITION).
 */

#ifndef CV1T_SHM_TYPES_ONLY
/* ---- Inline reader helpers for C/C++ consumers (the Rust side implements its own in src/shm.rs) ---- */

static inline uint64_t cv1t_now_ns(void)
{
#ifdef _WIN32
	LARGE_INTEGER f, t;
	QueryPerformanceFrequency(&f);
	QueryPerformanceCounter(&t);
	const uint64_t freq = f.QuadPart > 0 ? (uint64_t)f.QuadPart : 1, ticks = (uint64_t)t.QuadPart;
	/* exactly ticks * 1e9 / freq, as the daemon computes it in 128 bits (freq < 1.8e10) */
	return ticks / freq * 1000000000ull + ticks % freq * 1000000000ull / freq;
#else
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
#endif
}

/* Consistent copy of a seqlock-protected block: 0, or -1 if the writer kept it busy for max_tries. */
static inline int cv1t_seq_read(const uint32_t *seq, void *dst, const void *src, size_t n, int max_tries)
{
	for (int i = 0; i < max_tries; i++) {
		uint32_t s1 = __atomic_load_n(seq, __ATOMIC_ACQUIRE);
		if (s1 & 1u)
			continue;
		memcpy(dst, src, n);
		__atomic_thread_fence(__ATOMIC_SEQ_CST);
		if (__atomic_load_n(seq, __ATOMIC_RELAXED) == s1)
			return 0;
	}
	return -1;
}

static inline int cv1t_shm_read_hand(const cv1t_shm *shm, int hand, cv1t_controller_state *out)
{
	return cv1t_seq_read(&shm->hands[hand].seq, out, &shm->hands[hand].state, sizeof *out, 1000);
}

static inline int cv1t_shm_read_alignment(const cv1t_shm *shm, cv1t_alignment *out)
{
	return cv1t_seq_read(&shm->alignment_seq, out, &shm->alignment, sizeof *out, 1000);
}

#ifdef _WIN32
/* Block until update_futex differs from last_seen, a publish wakes the reader, or timeout_ns passes;
 * returns the current value. events: the opened update0 and update1 (either may be NULL: then it
 * sleeps out the timeout). */
static inline uint32_t cv1t_shm_wait_events(const cv1t_shm *shm, HANDLE const events[2], uint32_t last_seen, int64_t timeout_ns)
{
	if (__atomic_load_n(&shm->update_futex, __ATOMIC_ACQUIRE) == last_seen) {
		int64_t ms = timeout_ns > 0 ? (timeout_ns + 999999) / 1000000 : 0;
		if (ms > 0x7fffffff)
			ms = 0x7fffffff;
		HANDLE e = events[(last_seen + 1u) % 2u];
		if (e)
			WaitForSingleObject(e, (DWORD)ms);
		else
			Sleep((DWORD)ms);
	}
	return __atomic_load_n(&shm->update_futex, __ATOMIC_ACQUIRE);
}
#else
/* Block until update_futex differs from last_seen or timeout_ns passes; returns the current value. */
static inline uint32_t cv1t_shm_wait(const cv1t_shm *shm, uint32_t last_seen, int64_t timeout_ns)
{
	struct timespec ts = { (time_t)(timeout_ns / 1000000000), (long)(timeout_ns % 1000000000) };
	syscall(SYS_futex, (uint32_t *)&shm->update_futex, FUTEX_WAIT, last_seen, &ts, NULL, 0);
	return __atomic_load_n(&shm->update_futex, __ATOMIC_ACQUIRE);
}
#endif
#endif /* CV1T_SHM_TYPES_ONLY */

#endif
