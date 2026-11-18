// SPDX-License-Identifier: Apache-2.0
/* Synthetic, hardware-free complete set_blobs + find_one_pose regression. */
#define _GNU_SOURCE
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

/* Count every heap request the search sources make; the camera they call is
 * compiled under allocation poisons in test_projection_camera.c. */
static int counting;
static unsigned long requests, request_bytes;
static void hit(size_t n) { if (counting) { requests++; request_bytes += n; } }
static void *count_malloc(size_t n) { hit(n); return malloc(n); }
static void *count_calloc(size_t n, size_t size) { hit(n * size); return calloc(n, size); }
#define malloc(n) count_malloc(n)
#define calloc(n, size) count_calloc(n, size)
#define realloc(...) ((void)sizeof(char[-1]))
#include "correspondence_search.c"
#include "led_search.c"
#include "omath.c"
#include "rift-sensor-maths.c"
#include "rift-sensor-pose-helper.c"
#include "test_util.h"

extern void camera_contracts(void);

struct search_case {
    int n, geo, fish, scenario, count;
    float blobs[12][2];
    int accepted;
    unsigned trials, checks;
    float pose[7]; /* Orientation x, y, z, w, then position. */
    int match_flags, matched_blobs, unmatched_blobs, visible_leds;
    double reprojection_error;
    int labels[12], marked[12];
    float orient_error[3], pos_error[3];
};
#include "support/projection_search_cases.inc"

struct result {
    posef pose;
    rift_pose_metrics score;
    bool accepted;
    unsigned trials, checks;
    int labels[64];
    unsigned long requests, request_bytes;
};

static void compare_score(const struct search_case *e, const rift_pose_metrics *b) {
    CHECK(e->match_flags == (int)b->match_flags);
    CHECK(e->matched_blobs == b->matched_blobs);
    CHECK(e->unmatched_blobs == b->unmatched_blobs);
    CHECK(e->visible_leds == b->visible_leds);
    for (int i = 0; i < 3; i++) {
        CHECK(e->orient_error[i] == b->orient_error.arr[i]);
        CHECK(e->pos_error[i] == b->pos_error.arr[i]);
    }
    /* Projection permits 0.001 px per coordinate. For n matched points the
     * metric is S = sum(dx*dx + dy*dy), so Cauchy-Schwarz gives
     * |S' - S| <= 2*epsilon*sqrt(2*n*S) + 2*n*epsilon*epsilon.
     * The old float-matrix/Rodrigues round trip need not preserve score bits. */
    const double camera_pixel_tolerance = 0.001;
    double bound = 2 * camera_pixel_tolerance * sqrt(2 * e->matched_blobs * e->reprojection_error)
                 + 2 * e->matched_blobs * camera_pixel_tolerance * camera_pixel_tolerance;
    CHECK_NEAR(e->reprojection_error, b->reprojection_error, bound);
}

static struct result run(rift_leds *model, rift_sensor_camera_params *cal,
                         const struct blob *observations, int count, int deep) {
    struct result r = {0};
    struct blob blobs[64];
    memcpy(blobs, observations, count * sizeof(*blobs));
    led_search_model_t *lm = led_search_model_new(model);
    correspondence_search_t *cs = correspondence_search_new(cal);
    correspondence_search_set_model(cs, 1, lm);
    requests = request_bytes = 0;
    counting = 1;
    correspondence_search_set_blobs(cs, blobs, count);
    r.accepted = correspondence_search_find_one_pose(
        cs, 1,
        CS_FLAG_SHALLOW_SEARCH | (deep ? CS_FLAG_DEEP_SEARCH : 0) | CS_FLAG_STOP_FOR_STRONG_MATCH,
        &r.pose, NULL, NULL, NULL, 0, &r.score);
    counting = 0;
    r.requests = requests;
    r.request_bytes = request_bytes;
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

static void *contracts(void *unused) {
    (void)unused;
    camera_contracts();
    return NULL;
}

int main(void) {
    const int sizes[] = {7, 12, 16, 24, 44, 64};
    unsigned accepted = 0, rejected = 0;
    pthread_t a, b;
    camera_contracts();
    CHECK(pthread_create(&a, NULL, contracts, NULL) == 0 && pthread_create(&b, NULL, contracts, NULL) == 0);
    CHECK(pthread_join(a, NULL) == 0 && pthread_join(b, NULL) == 0);
    for (unsigned c = 0; c < sizeof(search_cases) / sizeof(search_cases[0]); c++) {
        const struct search_case *e = &search_cases[c];
        unsigned sz = 0;
        while (sizes[sz] != e->n)
            sz++;
        rift_led points[64];
        for (int i = 0; i < e->n; i++)
            points[i] = (rift_led){.id = i,
                                   .pos = {{search_models[sz * 3 + e->geo][i][0],
                                            search_models[sz * 3 + e->geo][i][1],
                                            search_models[sz * 3 + e->geo][i][2]}},
                                   .dir = {{0, 0, -1}}};
        rift_leds model = {.points = points, .num_points = e->n, .radius_mm = .004};
        rift_sensor_camera_params cal = {.width = 1280, .height = 960, .dist_fisheye = e->fish};
        cal.camera_matrix.m[0] = cal.camera_matrix.m[4] = 600;
        cal.camera_matrix.m[2] = 640;
        cal.camera_matrix.m[5] = 480;
        cal.camera_matrix.m[8] = 1;
        cal.dist_coeffs[0] = e->fish ? -.08 : -.15;
        cal.dist_coeffs[1] = .012;
        struct blob blobs[64];
        for (int i = 0; i < e->count; i++)
            blobs[i] = (struct blob){.blob_id = i + 1,
                                     .led_id = -1,
                                     .x = e->blobs[i][0],
                                     .y = e->blobs[i][1],
                                     .width = 3,
                                     .height = 3,
                                     .area = 9};
        struct result r = run(&model, &cal, blobs, e->count, e->scenario == 2);
        CHECK(r.accepted == e->accepted && r.trials == e->trials && r.checks == e->checks);
        CHECK(memcmp(r.pose.orient.arr, e->pose, 4 * sizeof(float)) == 0);
        CHECK(memcmp(r.pose.pos.arr, e->pose + 4, 3 * sizeof(float)) == 0);
        compare_score(e, &r.score);
        for (int i = 0; i < e->count; i++) {
            CHECK(r.labels[i] == e->labels[i]);
            CHECK(r.labels[32 + i] == e->marked[i]);
        }
        /* The measured operation's only heap request is set_blobs' point array. */
        CHECK(r.requests == 1);
        CHECK(r.request_bytes == e->count * sizeof(cs_image_point_t));
        printf("FULL n=%d geo=%d fish=%d scenario=%d ok=%d trials=%u checks=%u requests=%lu bytes=%lu\n",
               e->n, e->geo, e->fish, e->scenario, r.accepted, r.trials, r.checks, r.requests,
               r.request_bytes);
        accepted += r.accepted;
        rejected += !r.accepted;
    }
    CHECK(accepted == 54 && rejected == 54);
    printf("projection search: %zu frozen complete-operation scenarios; %u accepted, %u rejected\n",
           sizeof(search_cases) / sizeof(search_cases[0]), accepted, rejected);
    return 0;
}
