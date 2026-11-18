// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* Exercise the actual admission and cleanup path without cameras or controller data. */
#include "omath.c"
#include "rift-sensor-pose-search.c"
#include "test_util.h"

correspondence_search_t *correspondence_search_new(rift_sensor_camera_params *calib)
{
    (void)calib;
    return NULL;
}

void correspondence_search_free(correspondence_search_t *cs)
{
    (void)cs;
}

int main(void)
{
    rift_pose_finder a = {0}, b = {0}, c = {0}, fixed = {0}, unplaced = {0};
    rift_tracked_device_exposure_info unlocked = {0}, locked = {.had_pose_lock = true};
    posef pose = {.orient = {.w = 1}};
    rift_pose_finder_cv1touch_reset();
    rift_pose_finder_init(&a, NULL, NULL, NULL);
    rift_pose_finder_init(&b, NULL, NULL, NULL);
    rift_pose_finder_init(&c, NULL, NULL, NULL);
    rift_pose_finder_init(&fixed, NULL, NULL, NULL);
    rift_pose_finder_init(&unplaced, NULL, NULL, NULL);
    rift_pose_finder_set_camera_pose(&fixed, &pose);
    CHECK(cv1touch_may_place(&a, &unlocked));
    /* A pending pose clear can allow the same finder to place again. */
    rift_pose_finder_set_camera_pose(&a, &pose);
    a.have_camera_pose = false;
    CHECK(!cv1touch_may_place(&a, &unlocked));
    CHECK(cv1touch_may_place(&a, &locked));
    CHECK(cv1touch_may_place(&a, &locked));
    CHECK(!cv1touch_may_place(&b, &unlocked));
    rift_pose_finder_clear(&unplaced);
    rift_pose_finder_clear(&fixed);
    CHECK(!cv1touch_may_place(&b, &unlocked));
    rift_pose_finder_clear(&a);
    rift_pose_finder_clear(&a);
    CHECK(cv1touch_may_place(&b, &unlocked));
    CHECK(cv1touch_may_place(&c, &locked));
    rift_pose_finder_clear(&b);
    CHECK(!cv1touch_may_place(&a, &unlocked));
    rift_pose_finder_clear(&c);
    CHECK(cv1touch_may_place(&a, &unlocked));
    rift_pose_finder_clear(&a);
    puts("placement lifecycle ok");
    return 0;
}
