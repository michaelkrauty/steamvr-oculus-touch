/* SPDX-License-Identifier: BSL-1.0
 * Owned sensor configuration for the cv1touch acquisition extension. */
#ifndef CV1TOUCH_SENSOR_CONFIG_H
#define CV1TOUCH_SENSOR_CONFIG_H
#include "openhmd_cv1touch.h"
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#ifndef CV1TOUCH_CONFIG_ALLOC
#define CV1TOUCH_CONFIG_ALLOC malloc
#endif
typedef struct {
  bool configured;
  size_t ignored_count, pose_count;
  char (*ignored)[32];
  ohmd_cv1touch_sensor_pose *poses;
} cv1touch_sensor_config;
static inline bool cv1touch_sensor_serial_valid(const char serial[32]) {
  size_t n = 0;
  while (n < 32 && serial[n])
    n++;
  if (n == 0 || n == 32)
    return false;
  for (size_t i = 0; i < n; i++) {
    unsigned char c = serial[i];
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9')))
      return false;
  }
  return true;
}
static inline void cv1touch_sensor_config_clear(cv1touch_sensor_config *c) {
  free(c->ignored);
  free(c->poses);
  memset(c, 0, sizeof *c);
}
/* Transactional: reject invalid inputs/allocation failure without changing the
 * old copy. */
static inline int
cv1touch_sensor_config_set(cv1touch_sensor_config *c, const char (*ignored)[32],
                           size_t ni, const ohmd_cv1touch_sensor_pose *poses,
                           size_t np) {
  if (ni > INT_MAX || np > INT_MAX || ni > SIZE_MAX / 32 ||
      np > SIZE_MAX / sizeof *poses || (ni && !ignored) || (np && !poses))
    return -1;
  for (size_t i = 0; i < ni; i++) {
    if (!cv1touch_sensor_serial_valid(ignored[i]))
      return -1;
    for (size_t j = 0; j < i; j++)
      if (!strcmp(ignored[i], ignored[j]))
        return -1;
  }
  for (size_t i = 0; i < np; i++) {
    if (!cv1touch_sensor_serial_valid(poses[i].serial))
      return -1;
    for (size_t j = 0; j < i; j++)
      if (!strcmp(poses[i].serial, poses[j].serial))
        return -1;
    double norm = 0;
    for (int k = 0; k < 4; k++) {
      if (!isfinite(poses[i].orient[k]))
        return -1;
      norm += (double)poses[i].orient[k] * poses[i].orient[k];
    }
    if (fabs(norm - 1) > 0.002)
      return -1;
    for (int k = 0; k < 3; k++)
      if (!isfinite(poses[i].pos[k]))
        return -1;
  }
  cv1touch_sensor_config next = {0};
  next.configured = true;
  next.ignored_count = ni;
  next.pose_count = np;
  if (ni) {
    next.ignored = (char (*)[32])CV1TOUCH_CONFIG_ALLOC(ni * 32);
    if (!next.ignored)
      goto fail;
    memcpy(next.ignored, ignored, ni * 32);
  }
  if (np) {
    next.poses =
        (ohmd_cv1touch_sensor_pose *)CV1TOUCH_CONFIG_ALLOC(np * sizeof *poses);
    if (!next.poses)
      goto fail;
    memcpy(next.poses, poses, np * sizeof *poses);
  }
  cv1touch_sensor_config_clear(c);
  *c = next;
  return 0;
fail:
  cv1touch_sensor_config_clear(&next);
  return -1;
}
static inline bool cv1touch_sensor_ignored(const cv1touch_sensor_config *c,
                                           const ohmd_cv1touch_config *legacy,
                                           const char *serial) {
  size_t n = c->configured ? c->ignored_count : 8;
  for (size_t i = 0; i < n; i++)
    if (!strcmp(c->configured ? c->ignored[i]
                              : legacy->ignore_sensor_serials[i],
                serial))
      return true;
  return false;
}
#endif
