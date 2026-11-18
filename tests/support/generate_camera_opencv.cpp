// SPDX-License-Identifier: Apache-2.0
/* Optional native backend for generate_camera_reference.py. Build with:
 * c++ generate_camera_opencv.cpp $(pkg-config --cflags --libs opencv5) -o generator
 * Then pass that executable to the Python script. Not linked into any test. */
#include <opencv2/calib3d.hpp>
#include <opencv2/core/version.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
int main(int argc, char **argv) {
    if (argc == 1) { std::cout << CV_VERSION << '\n'; return 0; }
    bool fish = argv[2][0] == '1', project = argv[1][0] == 'p';
    double intrinsics[] = {613,0,641,0,587,479,0,0,1};
    if (argc > 3) { intrinsics[0] = intrinsics[4] = 0x1.ffffffcbcb8p-1; intrinsics[2] = intrinsics[5] = 0; }
    cv::Mat K(3,3,CV_64F,intrinsics);
    cv::Mat d(fish ? 4 : 5, 1, CV_64F), r(3,1,CV_64F), t(3,1,CV_64F);
    for (int i = 0; i < d.rows; i++) std::cin >> d.at<double>(i);
    std::vector<cv::Point2f> out;
    if (project) {
        for (int i = 0; i < 3; i++) std::cin >> r.at<double>(i);
        for (int i = 0; i < 3; i++) std::cin >> t.at<double>(i);
        std::vector<cv::Point3f> in;
        float x, y, z;
        while (std::cin >> x >> y >> z) in.emplace_back(x,y,z);
        if (fish) cv::fisheye::projectPoints(in,out,r,t,K,d);
        else cv::projectPoints(in,r,t,K,d,out);
    } else {
        std::vector<cv::Point2f> in;
        float x,y;
        while (std::cin >> x >> y) in.emplace_back(x,y);
        if (fish) cv::fisheye::undistortPoints(in,out,K,d);
        else cv::undistortPoints(in,out,K,d);
    }
    std::cout << std::setprecision(17);
    for (auto p : out) std::cout << p.x << ' ' << p.y << '\n';
}
