// SPDX-License-Identifier: Apache-2.0
/* Fictional fixtures, independently generated with two pinned OpenCV versions. */
#include "test_util.h"
#include "rift-sensor-camera.h"
#include <float.h>
#include <string.h>

/* The camera translation unit must not call a heap allocator, on any platform.
 * Compile its actual source under allocation poisons, including the 64-point path. */
#define malloc(...) ((void)sizeof(char[-1]))
#define calloc(...) ((void)sizeof(char[-1]))
#define realloc(...) ((void)sizeof(char[-1]))
#define aligned_alloc(...) ((void)sizeof(char[-1]))
#define free(...) ((void)sizeof(char[-1]))
#include "rift-sensor-camera.c"

struct camera_case { int fish; double k[5]; float pixel[2], ray[2]; };
struct projection_case { int fish; float q[4], t[3], point[3], pixel[2]; };
#define camera_cases camera_4_12
#define projection_cases projection_4_12
#define tiny_camera_cases tiny_camera_4_12
#include "support/camera_4_12.inc"
#undef camera_cases
#undef projection_cases
#undef tiny_camera_cases
#define camera_cases camera_5_1
#define projection_cases projection_5_1
#define tiny_camera_cases tiny_camera_5_1
#include "support/camera_5_1.inc"
#undef camera_cases
#undef projection_cases
#undef tiny_camera_cases

static rift_sensor_camera_params camera(int fish)
{
    rift_sensor_camera_params c = {0};
    c.dist_fisheye = fish;
    c.camera_matrix.m[0] = 613; c.camera_matrix.m[4] = 587;
    c.camera_matrix.m[2] = 641; c.camera_matrix.m[5] = 479;
    c.camera_matrix.m[8] = 1;
    return c;
}

static void check_rays(const struct camera_case *cases, size_t count)
{
    unsigned exact = 0, sentinels = 0;
    for (size_t i = 0; i < count; i++) {
        rift_sensor_camera_params c = camera(cases[i].fish);
        memcpy(c.dist_coeffs, cases[i].k, sizeof(cases[i].k));
        struct blob b = {.x = cases[i].pixel[0], .y = cases[i].pixel[1]};
        vec3f out;
        undistort_points(&b, 1, &out, &c);
        CHECK(out.z == 1);
        CHECK((out.x == -1000000 && out.y == -1000000) ==
              (cases[i].ray[0] == -1000000 && cases[i].ray[1] == -1000000));
        sentinels += out.x == -1000000;
        for (int j = 0; j < 2; j++) {
            float ref = cases[i].ray[j], magnitude = fmaxf(fabsf(ref), 1);
            /* Criterion A: two float ULPs at max(abs(reference), 1), not an
             * arbitrary normalised-coordinate epsilon. */
            double tolerance = 2.0 * (nextafterf(magnitude, INFINITY) - magnitude);
            CHECK_NEAR(out.arr[j], ref, tolerance);
            exact += memcmp(&out.arr[j], &ref, sizeof(ref)) == 0;
        }
    }
    CHECK(sentinels > 0);
    CHECK(exact * 100 >= count * 2 * 99);
    printf("camera undistortion: %u/%zu exact components, %u sentinels\n", exact, count*2, sentinels);
}

static void check_tiny(const struct camera_case *cases)
{
    rift_sensor_camera_params c = camera(1);
    c.camera_matrix.m[0] = c.camera_matrix.m[4] = 0x1.ffffffcbcb8p-1;
    c.camera_matrix.m[2] = c.camera_matrix.m[5] = 0;
    for (int i = 0; i < 3; i++) {
        struct blob b = {.x = cases[i].pixel[0], .y = cases[i].pixel[1]};
        vec3f out;
        double radius = b.x / c.camera_matrix.m[0];
        CHECK(i == 0 ? radius < 1e-8 : i == 1 ? radius == 1e-8 : radius > 1e-8);
        undistort_points(&b, 1, &out, &c);
        CHECK(out.x == cases[i].ray[0] && out.y == cases[i].ray[1]);
        CHECK((out.x == 0) == (i < 2));
    }
}

static void check_projection(const struct projection_case *cases, size_t count)
{
    unsigned close = 0;
    for (size_t i = 0; i < count; i++) {
        rift_sensor_camera_params c = camera(cases[i].fish);
        const double k[5] = {-.08, .012, .001, -.0005, .0002};
        memcpy(c.dist_coeffs, k, sizeof(k));
        posef p;
        memcpy(p.orient.arr, cases[i].q, sizeof(cases[i].q));
        memcpy(p.pos.arr, cases[i].t, sizeof(cases[i].t));
        rift_led led = {0};
        memcpy(led.pos.arr, cases[i].point, sizeof(cases[i].point));
        vec3f out;
        rift_project_points(&led, 1, &c, &p, &out);
        CHECK(out.z == 0);
        double qx = p.orient.x, qy = p.orient.y, qz = p.orient.z, qw = p.orient.w;
        double n2 = qx*qx + qy*qy + qz*qz + qw*qw;
        double depth = ((2*qx*qz - 2*qw*qy)*led.pos.x +
                        (2*qy*qz + 2*qw*qx)*led.pos.y +
                        (n2 - 2*qx*qx - 2*qy*qy)*led.pos.z)/n2 + p.pos.z;
        for (int j = 0; j < 2; j++) {
            float ref = cases[i].pixel[j];
            CHECK(isfinite(out.arr[j]));
            /* Criterion A uses an absolute pixel bound at depth >= 0.05 m;
             * only near/behind the camera plane is relative 1e-6 allowed. */
            double tolerance = depth >= .05 ? .001 : fmax(.001, fabs(ref)*1e-6);
            CHECK_NEAR(out.arr[j], ref, tolerance);
            float magnitude = fmaxf(fabsf(ref), FLT_MIN);
            close += fabs((double)out.arr[j] - ref) <= nextafterf(magnitude, INFINITY) - magnitude;
        }
    }
    CHECK(close * 100 >= count * 2 * 99);
    printf("camera projection: %u/%zu components within one float ULP\n", close, count*2);
}

/* OpenCV is not the reference near a half turn: its matrix-to-vector Rodrigues
 * conversion treats a rotation whose sine is below 1e-5 as exactly pi, which
 * moves this point by more than 0.001 px. The fictional fixture comes from
 * tests/generate_camera_exact.py, which rotates by the normalised quaternion
 * in double precision without OpenCV. */
#include "support/camera_exact.inc"

static void check_half_turn(void)
{
    rift_sensor_camera_params c = camera(1);
    const double k[5] = {-.08, .012, .001, -.0005, .0002};
    memcpy(c.dist_coeffs, k, sizeof(k));
    posef p;
    memcpy(p.orient.arr, exact_half_turn.q, sizeof(exact_half_turn.q));
    memcpy(p.pos.arr, exact_half_turn.t, sizeof(exact_half_turn.t));
    rift_led led = {0};
    memcpy(led.pos.arr, exact_half_turn.point, sizeof(exact_half_turn.point));
    vec3f out;
    rift_project_points(&led, 1, &c, &p, &out);
    for (int j = 0; j < 2; j++) CHECK_NEAR(out.arr[j], exact_half_turn.pixel[j], .001);
}

static void round_trip_and_bounds(void)
{
    for (int fish = 0; fish < 2; fish++) {
        rift_sensor_camera_params c = camera(fish);
        c.dist_coeffs[0] = -.03;
        c.dist_coeffs[4] = .0002;
        posef p = {.orient = {{0, 0, 0, 1}}};
        rift_led leds[64] = {0};
        struct blob blobs[64] = {0};
        vec3f out[66], rays[66];
        for (int i = 0; i < 66; i++) out[i] = rays[i] = (vec3f){{123,456,789}};
        for (int i = 0; i < 64; i++)
            leds[i].pos = (vec3f){{(i%8 - 3.5f)*.04f, (i/8 - 3.5f)*.04f, 1}};
        rift_project_points(leds, 64, &c, &p, out + 1);
        for (int i = 0; i < 64; i++) { blobs[i].x = out[i+1].x; blobs[i].y = out[i+1].y; }
        undistort_points(blobs, 64, rays + 1, &c);
        CHECK(out[0].x == 123 && out[65].x == 123);
        CHECK(rays[0].x == 123 && rays[65].x == 123);
        for (int i = 0; i < 64; i++) {
            /* Float pixel rounding divided by focal length dominates this bound. */
            CHECK_NEAR(rays[i+1].x, leds[i].pos.x, 1e-7);
            CHECK_NEAR(rays[i+1].y, leds[i].pos.y, 1e-7);
        }
        /* Zero quaternions use identity; non-unit quaternions are normalised. */
        vec3f zero[64], scaled[64];
        p.orient.w = 0;
        rift_project_points(leds, 64, &c, &p, zero);
        p.orient.w = 2;
        rift_project_points(leds, 64, &c, &p, scaled);
        CHECK(memcmp(out+1, zero, sizeof(zero)) == 0);
        CHECK(memcmp(zero, scaled, sizeof(zero)) == 0);
        rift_project_points(NULL, 0, &c, &p, NULL);
        undistort_points(NULL, 0, NULL, &c);
    }
}

static void count_boundaries(void)
{
    const int counts[] = {-1, 0, 1, 4, 7, 12, 16, 24, 44, 63, 64, 65, 255, 1024};
    rift_led leds[1024] = {0};
    struct blob blobs[1024] = {0};
    vec3f pixels[1026], rays[1026];
    posef p = {.orient = {{0,0,0,1}}, .pos = {{0,0,1}}};
    for (int fish = 0; fish < 2; fish++) {
        rift_sensor_camera_params c = camera(fish);
        for (size_t n = 0; n < sizeof(counts)/sizeof(*counts); n++) {
            int count = counts[n], end = count > 0 ? count : 0;
            for (int i = 0; i < 1026; i++) pixels[i] = rays[i] = (vec3f){{123,456,789}};
            rift_project_points(leds, count, &c, &p, pixels+1);
            undistort_points(blobs, count, rays+1, &c);
            CHECK(pixels[0].x == 123 && pixels[end+1].x == 123);
            CHECK(rays[0].x == 123 && rays[end+1].x == 123);
            for (int i = 0; i < end; i++) {
                CHECK(pixels[i+1].x == 641 && pixels[i+1].y == 479 && pixels[i+1].z == 0);
                vec3f single;
                undistort_points(blobs+i, 1, &single, &c);
                CHECK(memcmp(&single, rays+i+1, sizeof(single)) == 0);
            }
        }
    }
}

void camera_contracts(void)
{
    check_rays(camera_4_12, sizeof(camera_4_12)/sizeof(*camera_4_12));
    check_rays(camera_5_1, sizeof(camera_5_1)/sizeof(*camera_5_1));
    check_projection(projection_4_12, sizeof(projection_4_12)/sizeof(*projection_4_12));
    check_projection(projection_5_1, sizeof(projection_5_1)/sizeof(*projection_5_1));
    check_half_turn();
    round_trip_and_bounds();
    count_boundaries();
}

#ifndef CAMERA_CONTRACTS_ONLY
int main(void)
{
    check_tiny(tiny_camera_4_12);
    check_tiny(tiny_camera_5_1);
    camera_contracts();
    return 0;
}
#endif
