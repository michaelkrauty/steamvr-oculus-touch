// SPDX-License-Identifier: Apache-2.0
// Offline generator helper, never linked into tests or the tracking engine.
#include <opencv2/core/version.hpp>
#include <opencv2/calib3d.hpp>
#include <iomanip>
#include <iostream>
#include <vector>

int main()
{
    std::cout << "OpenCV " << CV_VERSION << '\n' << std::setprecision(17);
    int n, lens, guess;
    while (std::cin >> n >> lens >> guess) {
        double k[9], d[5], q[4], t[3];
        for (double &x : k) std::cin >> x;
        for (double &x : d) std::cin >> x;
        for (double &x : q) std::cin >> x;
        for (double &x : t) std::cin >> x;
        std::vector<cv::Point3f> objects(n);
        std::vector<cv::Point2f> pixels(n), rays;
        for (auto &p : objects) std::cin >> p.x >> p.y >> p.z;
        for (auto &p : pixels) std::cin >> p.x >> p.y;
        cv::Mat K(3,3,CV_64F,k), D(lens ? 4 : 5,1,CV_64F,d);
        if (lens) cv::fisheye::undistortPoints(pixels,rays,K,D);
        else cv::undistortPoints(pixels,rays,K,D);
        double norm = 0;
        for (double x : q) norm += x*x;
        norm = std::sqrt(norm);
        for (double &x : q) x /= norm;
        double x=q[0], y=q[1], z=q[2], w=q[3];
        cv::Matx33d R(1-2*y*y-2*z*z,2*x*y-2*w*z,2*x*z+2*w*y,
            2*x*y+2*w*z,1-2*x*x-2*z*z,2*y*z-2*w*x,
            2*x*z-2*w*y,2*y*z+2*w*x,1-2*x*x-2*y*y);
        cv::Mat r, translation(3,1,CV_64F,t), inliers;
        cv::Rodrigues(R,r);
        bool ok=cv::solvePnPRansac(objects,rays,cv::Mat::eye(3,3,CV_64F),cv::Mat::zeros(4,1,CV_64F),
            r,translation,guess,100,(float)(3/k[0]),(float).99,inliers,cv::SOLVEPNP_ITERATIVE);
        std::cout << "{\"baseline\":[" << ok << ',' << inliers.rows << "],\"baseline_pose\":[";
        for (int j=0;j<6;j++) {
            if (j) std::cout << ',';
            std::cout << (ok ? (j<3 ? r.at<double>(j) : translation.at<double>(j-3)) : 0);
        }
        std::cout << "]}\n";
    }
}
