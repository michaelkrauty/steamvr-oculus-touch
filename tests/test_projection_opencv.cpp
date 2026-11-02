// SPDX-License-Identifier: Apache-2.0
/* Exercise the actual bridge and its frozen vector oracle in the same process. */
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/calib3d.hpp>
#else
#include <opencv2/calib3d/calib3d.hpp>
#endif

/* Fault-inject replacement after the real library call. This forces the bridge
 * itself to consume the returned row/column/strided Mat, not its stale scratch. */
static thread_local int replace_output;
static void replace_pixels(cv::OutputArray dst) {
    if (!replace_output)
        return;
    cv::Mat original = dst.getMat();
    int n = (int)original.total();
    cv::Mat storage(n, replace_output == 3 ? 2 : 1, CV_32FC2);
    cv::Mat replacement = storage.col(0);
    for (int i = 0; i < n; i++)
        replacement.at<cv::Point2f>(i, 0) =
            original.at<cv::Point2f>(i / original.cols, i % original.cols);
    if (replace_output == 1)
        replacement = replacement.reshape(2, 1);
    // Poison the abandoned view so reading the original scratch cannot pass.
    original.setTo(cv::Scalar(-123456, 654321));
    dst.getMatRef() = replacement;
}
namespace cv {
static void test_projectPoints(InputArray objects, InputArray r, InputArray t, InputArray K,
                               InputArray D, OutputArray pixels) {
    projectPoints(objects, r, t, K, D, pixels);
    replace_pixels(pixels);
}
namespace fisheye {
static void test_projectPoints(InputArray objects, OutputArray pixels, InputArray r, InputArray t,
                               InputArray K, InputArray D) {
    projectPoints(objects, pixels, r, t, K, D);
    replace_pixels(pixels);
}
} // namespace fisheye
} // namespace cv
#define projectPoints test_projectPoints
#define rift_project_points optimized_project_points
#include "rift-sensor-opencv.cpp"
#undef rift_project_points
#undef projectPoints
#include "support/projection_reference.inc"
#include "test_util.h"
#include <cstring>
#include <thread>

static bool reference;
extern "C" void projection_reference(int enabled) { reference = enabled; }
extern "C" void rift_project_points(rift_led *leds, int n, rift_sensor_camera_params *calib,
                                    posef *pose, vec3f *out) {
    if (reference)
        reference_project_points(leds, n, calib, pose, out);
    else
        optimized_project_points(leds, n, calib, pose, out);
}

static int invoke(bool legacy, rift_led *leds, int n, rift_sensor_camera_params *c, posef *p,
                  vec3f *out) {
    try {
        if (legacy)
            reference_project_points(leds, n, c, p, out);
        else
            optimized_project_points(leds, n, c, p, out);
        return 0;
    } catch (const cv::Exception &e) {
        return e.code;
    } catch (const std::length_error &) {
        return 1;
    }
}

static void direct_contracts() {
    const int sizes[] = {-1, 0, 1, 4, 7, 12, 16, 24, 44, 63, 64, 65, 255, 1024};
    for (int fish = 0; fish < 2; fish++)
        for (int n : sizes)
            for (int g = 0; g < 6; g++) {
                rift_sensor_camera_params c = {};
                c.width = 1280;
                c.height = 960;
                c.dist_fisheye = fish;
                c.camera_matrix.m[0] = c.camera_matrix.m[4] = 600;
                c.camera_matrix.m[2] = 640;
                c.camera_matrix.m[5] = 480;
                c.camera_matrix.m[8] = 1;
                c.dist_coeffs[0] = -.1;
                c.dist_coeffs[1] = .012;
                c.dist_coeffs[2] = .001;
                c.dist_coeffs[3] = -.0005;
                c.dist_coeffs[4] = .0002;
                posef p = {};
                p.orient.w = 1;
                p.pos.z = 2;
                if (g == 3) {
                    p.orient.w = 0;
                    p.orient.y = 1;
                }
                if (g == 4)
                    p.pos.z = .00001;
                if (g == 5) {
                    p.pos.z = -2;
                    p.pos.x = 1;
                }
                int count = std::max(n, 0);
                std::vector<rift_led> leds(count);
                std::vector<vec3f> expected(count + 2), actual(count + 2);
                memset(expected.data(), 0x5a, expected.size() * sizeof(vec3f));
                actual = expected;
                for (int i = 0; i < count; i++) {
                    leds[i].pos.x = .08f * cosf(i * 2.4f);
                    leds[i].pos.y = g == 2 ? 0 : .07f * sinf(i * 1.7f);
                    leds[i].pos.z = g == 0 ? .012f * sinf(i * .9f) : 0;
                }
                int a = invoke(true, leds.data(), n, &c, &p, expected.data() + 1);
                int b = invoke(false, leds.data(), n, &c, &p, actual.data() + 1);
                CHECK(a == b);
                CHECK(memcmp(expected.data(), actual.data(), expected.size() * sizeof(vec3f)) == 0);
#if CV_VERSION_MAJOR == 5
                if (n > 0 && n <= 64) {
                    for (int layout = 1; layout <= 3; layout++) {
                        replace_output = layout;
                        CHECK(invoke(false, leds.data(), n, &c, &p, actual.data() + 1) == a);
                        CHECK(memcmp(expected.data(), actual.data(),
                                     expected.size() * sizeof(vec3f)) == 0);
                    }
                    replace_output = 0;
                }
#endif
            }
}

extern "C" void projection_contracts(void) {
    direct_contracts();
    /* Concurrent independent calls must not share scratch. No dispatch writes here. */
    std::thread a(direct_contracts), b(direct_contracts);
    a.join();
    b.join();

    puts("projection contracts: 168 boundary/geometry/model cases, repeated concurrently; returned "
         "Mat layouts passed");
}
