// SPDX-License-Identifier: Apache-2.0
/* Exercise the actual bridge and its frozen vector oracle in the same process. */
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/calib3d.hpp>
#else
#include <opencv2/calib3d/calib3d.hpp>
#endif

#include "rift-sensor-opencv.cpp"
#include "support/projection_reference.inc"
#include "test_util.h"
#include <cstring>
#include <thread>

extern "C" void candidate_project_points(rift_led *, int, rift_sensor_camera_params *, posef *, vec3f *);
extern "C" void candidate_camera_contracts(void);
static bool reference;
extern "C" void projection_reference(int enabled) { reference = enabled; }
extern "C" void rift_project_points(rift_led *leds, int n, rift_sensor_camera_params *calib,
                                    posef *pose, vec3f *out) {
    if (reference)
        reference_project_points(leds, n, calib, pose, out);
    else
        candidate_project_points(leds, n, calib, pose, out);
}

extern "C" void projection_contracts(void) {
    candidate_camera_contracts();
    std::thread a(candidate_camera_contracts), b(candidate_camera_contracts);
    a.join();
    b.join();
}
