// SPDX-License-Identifier: LGPL-2.0-or-later OR BSL-1.0
// Copyright 2015 Philipp Zabel
/* Frozen vector projection oracle from the OpenHMD bridge. */
static void reference_project_points(rift_led *leds, int num_led_pos,
                                     rift_sensor_camera_params *calib, posef *pose,
                                     vec3f *out_points) {
    cv::Mat cameraK = cv::Mat(3, 3, CV_64FC1, calib->camera_matrix.m);
    cv::Mat distCoeffs;

    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);
    int i;

    for (i = 0; i < 3; i++)
        tvec.at<double>(i) = pose->pos.arr[i];

    quatf_to_3x3(R, &pose->orient);
    cv::Rodrigues(R, rvec);

    std::vector<cv::Point3f> led_points3d(num_led_pos);
    std::vector<cv::Point2f> projected_points2d(num_led_pos);

    for (int i = 0; i < num_led_pos; i++) {
        led_points3d[i].x = leds[i].pos.x;
        led_points3d[i].y = leds[i].pos.y;
        led_points3d[i].z = leds[i].pos.z;
    }

    if (calib->dist_fisheye) {
        distCoeffs = cv::Mat(4, 1, CV_64FC1, calib->dist_coeffs);
        cv::fisheye::projectPoints(led_points3d, projected_points2d, rvec, tvec, cameraK,
                                   distCoeffs);
    } else {
        distCoeffs = cv::Mat(5, 1, CV_64FC1, calib->dist_coeffs);
        // Pinhole camera undistort
        cv::projectPoints(led_points3d, rvec, tvec, cameraK, distCoeffs, projected_points2d);
    }

    for (int i = 0; i < num_led_pos; i++) {
        out_points[i].x = projected_points2d[i].x;
        out_points[i].y = projected_points2d[i].y;
        out_points[i].z = 0;
    }
}
