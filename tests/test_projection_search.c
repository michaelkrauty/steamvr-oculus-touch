// SPDX-License-Identifier: Apache-2.0
/* Synthetic, hardware-free complete set_blobs + find_one_pose regression. */
#define _GNU_SOURCE
#include "correspondence_search.c"
#include "led_search.c"
#include "omath.c"
#include "rift-sensor-maths.c"
#include "rift-sensor-pose-helper.c"
#include "test_util.h"

extern void count_begin(void);
extern void count_end(uint64_t *);
extern int projection_fast_path(void);
extern int projection_heap_counters(void);
extern void projection_reference(int);
extern void projection_contracts(void);

struct result {
    posef pose;
    rift_pose_metrics score;
    bool accepted;
    unsigned trials, checks;
    int labels[64];
    uint64_t allocations[6]; /* Mat payload, C++ new, glibc heap: overlapping, never summed. */
};

static quatf turn(float deg) {
    vec3f axis = {{.3f, .8f, .4f}};
    ovec3f_normalize_me(&axis);
    quatf q;
    oquatf_init_axis(&q, &axis, DEG_TO_RAD(deg));
    return q;
}

static struct result run(rift_leds *model, rift_sensor_camera_params *cal,
                         const struct blob *observations, int count, int deep, int legacy) {
    struct result r = {0};
    struct blob blobs[64];
    memcpy(blobs, observations, count * sizeof(*blobs));
    projection_reference(legacy);
    led_search_model_t *lm = led_search_model_new(model);
    correspondence_search_t *cs = correspondence_search_new(cal);
    correspondence_search_set_model(cs, 1, lm);
    count_begin();
    correspondence_search_set_blobs(cs, blobs, count);
    r.accepted = correspondence_search_find_one_pose(
        cs, 1,
        CS_FLAG_SHALLOW_SEARCH | (deep ? CS_FLAG_DEEP_SEARCH : 0) | CS_FLAG_STOP_FOR_STRONG_MATCH,
        &r.pose, NULL, NULL, NULL, 0, &r.score);
    count_end(r.allocations);
    r.trials = cs->num_trials;
    r.checks = cs->num_pose_checks;
    for (int i = 0; i < count; i++)
        r.labels[i] = blobs[i].led_id;
    /* Label assignment downstream of the full operation must also agree. */
    if (r.accepted)
        rift_mark_matching_blobs(&r.pose, blobs, count, 1, model, cal);
    for (int i = 0; i < count; i++)
        r.labels[32 + i] = blobs[i].led_id;
    correspondence_search_free(cs);
    led_search_model_free(lm);
    return r;
}

int main(void) {
    const int sizes[] = {7, 12, 16, 24, 44, 64};
    unsigned passed = 0, accepted = 0, rejected = 0;
    projection_contracts();
    for (unsigned sz = 0; sz < sizeof(sizes) / sizeof(sizes[0]); sz++) {
        int n = sizes[sz];
        for (int geo = 0; geo < 3; geo++)
            for (int fish = 0; fish < 2; fish++) {
                rift_led points[64];
                for (int i = 0; i < n; i++) {
                    points[i] = (rift_led){.id = i,
                                           .pos = {{.08f * cosf(i * 2.4f), .07f * sinf(i * 1.7f),
                                                    .012f * sinf(i * .9f)}},
                                           .dir = {{0, 0, -1}}};
                    if (geo == 1)
                        points[i].pos.z = 0;
                    if (geo == 2) {
                        points[i].pos.y *= .02f;
                        points[i].pos.z *= .02f;
                    }
                }
                rift_leds model = {.points = points, .num_points = n, .radius_mm = .004};
                for (int scenario = 0; scenario < 3; scenario++) {
                    rift_sensor_camera_params cal = {
                        .width = 1280, .height = 960, .dist_fisheye = fish};
                    cal.camera_matrix.m[0] = cal.camera_matrix.m[4] = 600;
                    cal.camera_matrix.m[2] = 640;
                    cal.camera_matrix.m[5] = 480;
                    cal.camera_matrix.m[8] = 1;
                    cal.dist_coeffs[0] = fish ? -.08 : -.15;
                    cal.dist_coeffs[1] = .012;
                    int count = scenario == 2 ? 6 : (n < 12 ? n : 12);
                    float noise = scenario == 0 ? 0 : scenario == 1 ? .25f : .6f;
                    posef truth = {.orient = turn(scenario == 0 ? 0 : 20),
                                   .pos = {{scenario == 2 ? .7f : .05f, -.1f, 1.6f}}};
                    vec3f px[64];
                    projection_reference(1);
                    rift_project_points(points, n, &cal, &truth, px);
                    struct blob blobs[64];
                    for (int i = 0; i < count; i++)
                        blobs[i] = (struct blob){.blob_id = i + 1,
                                                 .led_id = -1,
                                                 .x = px[i].x + noise * sinf(i * 3.1f),
                                                 .y = px[i].y + noise * cosf(i * 1.3f),
                                                 .width = 3,
                                                 .height = 3,
                                                 .area = 9};
                    struct result a = run(&model, &cal, blobs, count, scenario == 2, 1);
                    struct result b = run(&model, &cal, blobs, count, scenario == 2, 0);
                    CHECK(a.accepted == b.accepted && a.trials == b.trials && a.checks == b.checks);
                    CHECK(memcmp(&a.pose, &b.pose, sizeof(a.pose)) == 0);
                    CHECK(memcmp(&a.score, &b.score, sizeof(a.score)) == 0);
                    CHECK(memcmp(a.labels, b.labels, sizeof(a.labels)) == 0);
                    for (int layer = 0; layer < 6; layer++) {
                        if (projection_fast_path() && a.checks &&
                            (layer < 4 || projection_heap_counters()))
                            CHECK(b.allocations[layer] < a.allocations[layer]);
                        else
                            CHECK(b.allocations[layer] == a.allocations[layer]);
                    }
                    printf("FULL n=%d geo=%d fish=%d scenario=%d ok=%d trials=%u checks=%u "
                           "mat=%lu/%lu mat_bytes=%lu/%lu new=%lu/%lu new_bytes=%lu/%lu "
                           "heap=%lu/%lu heap_bytes=%lu/%lu\n",
                           n, geo, fish, scenario, b.accepted, b.trials, b.checks, a.allocations[0],
                           b.allocations[0], a.allocations[1], b.allocations[1], a.allocations[2],
                           b.allocations[2], a.allocations[3], b.allocations[3], a.allocations[4],
                           b.allocations[4], a.allocations[5], b.allocations[5]);
                    passed++;
                    accepted += b.accepted;
                    rejected += !b.accepted;
                }
            }
    }
    CHECK(accepted && rejected);
    printf("projection search: %u complete-operation exact comparisons; %u accepted, %u rejected\n",
           passed, accepted, rejected);
    return 0;
}
