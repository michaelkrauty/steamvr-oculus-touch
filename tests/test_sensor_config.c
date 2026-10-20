// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
#include <stdlib.h>
static int fail_alloc, calls;
static void *config_alloc(size_t n) {
  if (++calls == fail_alloc)
    return NULL;
  return malloc(n);
}
#define CV1TOUCH_CONFIG_ALLOC config_alloc
#include "cv1touch-sensor-config.h"
#include "omath.c"
#include "rift-cv1touch-poses.h"
#include "test_util.h"
static void serial_case_matches(void) {
  cv1touch_sensor_config c = {0};
  ohmd_cv1touch_config legacy = {0};
  char ignored[1][32] = {"camera1"};
  ohmd_cv1touch_sensor_pose poses[2] = {
      {.serial = "camera2", .orient = {0, 0, 0, 1}, .pos = {1, 2, 3}},
      {.serial = "CAMERA2", .orient = {0, 0, 0, 1}}};
  posef found;
  CHECK(cv1touch_sensor_config_set(&c, ignored, 1, poses, 1) == 0);
  CHECK(cv1touch_sensor_ignored(&c, &legacy, "CaMeRa1"));
  CHECK(!cv1touch_sensor_ignored(&c, &legacy, "CAMERA"));
  CHECK(!cv1touch_sensor_ignored(&c, &legacy, "CAMERA10"));
  CHECK(cv1touch_fixed_pose_lookup_list(c.poses, c.pose_count, "CaMeRa2", &found));
  CHECK(found.pos.x == 1 && found.pos.y == 2 && found.pos.z == 3);
  char duplicates[2][32] = {"camera1", "CAMERA1"};
  CHECK(cv1touch_sensor_config_set(&c, duplicates, 2, poses, 1) == -1);
  CHECK(cv1touch_sensor_config_set(&c, ignored, 1, poses, 2) == -1);
  CHECK(c.ignored_count == 1 && c.pose_count == 1);
  cv1touch_sensor_config_clear(&c);
  strcpy(legacy.ignore_sensor_serials[0], "cAmErA1");
  legacy.fixed_poses[0] = poses[0];
  CHECK(cv1touch_sensor_ignored(&c, &legacy, "CAMERA1"));
  CHECK(cv1touch_sensor_ignored(&c, &legacy, "camera1"));
  CHECK(!cv1touch_sensor_ignored(&c, &legacy, "CAMERA10"));
  CHECK(cv1touch_fixed_pose_lookup(&legacy, "CAMERA2", &found));
}

int main(void) {
  serial_case_matches();
  char names[65][32] = {0};
  ohmd_cv1touch_sensor_pose poses[65] = {0};
  for (int i = 0; i < 65; i++) {
    snprintf(names[i], 32, "TEST%06d", i);
    snprintf(poses[i].serial, 32, "TEST%06d", i);
    poses[i].orient[3] = 1;
    poses[i].pos[0] = i;
  }
  cv1touch_sensor_config c = {0};
  ohmd_cv1touch_config legacy = {0};
  posef found;
  size_t counts[] = {0, 1, 4, 5, 9, 65};
  for (size_t i = 0; i < sizeof counts / sizeof counts[0]; i++) {
    size_t n = counts[i];
    CHECK(cv1touch_sensor_config_set(&c, names, n, poses, n) == 0);
    CHECK(c.ignored_count == n && c.pose_count == n);
    if (n) {
      CHECK(cv1touch_sensor_ignored(&c, &legacy, names[n - 1]));
      CHECK(cv1touch_fixed_pose_lookup_list(c.poses, c.pose_count, names[n - 1],
                                            &found));
      CHECK(found.pos.x == n - 1);
    }
  }
  names[64][0] = 'X';
  poses[64].pos[0] = -1;
  CHECK(!strcmp(c.ignored[64], "TEST000064") && c.poses[64].pos[0] == 64);
  void *old = c.poses;
  for (int n = 1; n <= 2; n++) {
    calls = 0;
    fail_alloc = n;
    CHECK(cv1touch_sensor_config_set(&c, names, 65, poses, 65) == -1);
    CHECK(c.poses == old && c.pose_count == 65);
  }
  fail_alloc = 0;
  CHECK(cv1touch_sensor_config_set(&c, NULL, 1, poses, 65) == -1);
  CHECK(cv1touch_sensor_config_set(&c, names, (size_t)INT_MAX + 1, poses, 65) ==
        -1);
  strcpy(names[0], "bad/name");
  CHECK(cv1touch_sensor_config_set(&c, names, 65, poses, 65) == -1);
  CHECK(c.poses == old);
  strcpy(names[0], "TEST000000");
  poses[0].orient[3] = NAN;
  CHECK(cv1touch_sensor_config_set(&c, names, 65, poses, 65) == -1);
  poses[0].orient[3] = 2;
  CHECK(cv1touch_sensor_config_set(&c, names, 65, poses, 65) == -1);
  poses[0].orient[3] = 1;
  poses[0].pos[2] = INFINITY;
  CHECK(cv1touch_sensor_config_set(&c, names, 65, poses, 65) == -1);
  poses[0].pos[2] = 0;
  strcpy(names[1], names[0]);
  CHECK(cv1touch_sensor_config_set(&c, names, 65, poses, 65) == -1);
  CHECK(c.poses == old && c.pose_count == 65);
  cv1touch_sensor_config_clear(&c);
  CHECK(!c.poses && !c.ignored && !c.configured);
  puts("test_sensor_config: ok");
  return 0;
}
