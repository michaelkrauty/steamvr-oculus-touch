/* SPDX-License-Identifier: Apache-2.0
 * Camera formulas adapted from OpenCV 5.1.0, cross-checked against 4.12.0:
 * modules/geometry/src/{fisheye.cpp,pinhole.cpp,calibration_base.cpp}.
 * Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
 * Copyright (C) 2009, Willow Garage Inc., all rights reserved.
 * Third party copyrights are property of their respective owners.
 * See third_party/openhmd/LICENSE-OpenCV for the Apache 2.0 license.
 */
#include "rift-sensor-camera.h"
#include <float.h>
#include <math.h>

void rift_project_points(rift_led *leds, int count, rift_sensor_camera_params *calib,
                         posef *pose, vec3f *out)
{
    double qx = pose->orient.x, qy = pose->orient.y;
    double qz = pose->orient.z, qw = pose->orient.w;
    double norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (norm == 0) {
        qw = 1;
    } else {
        qx /= norm; qy /= norm; qz /= norm; qw /= norm;
    }
    double R[9] = {
        1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy,
        2*qx*qy + 2*qw*qz, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qw*qx,
        2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, 1 - 2*qx*qx - 2*qy*qy
    };
    const double *k = calib->dist_coeffs, *K = calib->camera_matrix.m;
    for (int i = 0; i < count; i++) {
        double X = leds[i].pos.x, Y = leds[i].pos.y, Z = leds[i].pos.z;
        double x = R[0]*X + R[1]*Y + R[2]*Z + pose->pos.x;
        double y = R[3]*X + R[4]*Y + R[5]*Z + pose->pos.y;
        double z = R[6]*X + R[7]*Y + R[8]*Z + pose->pos.z;
        double xd, yd;
        if (calib->dist_fisheye) {
            z = fabs(z) < DBL_MIN ? 1 : z;
            x /= z; y /= z;
            double r2 = x*x + y*y, r = sqrt(r2), theta = atan(r);
            double t2 = theta*theta, t3 = t2*theta, t4 = t2*t2, t5 = t4*theta;
            double t6 = t3*t3, t7 = t6*theta, t8 = t4*t4, t9 = t8*theta;
            double td = theta + k[0]*t3 + k[1]*t5 + k[2]*t7 + k[3]*t9;
            double scale = r > 1e-8 ? td*(1/r) : 1;
            xd = x*scale; yd = y*scale;
        } else {
            z = z ? 1/z : 1;
            x *= z; y *= z;
            double r2 = x*x + y*y, r4 = r2*r2, r6 = r4*r2;
            double radial = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6;
            double a1 = 2*x*y, a2 = r2 + 2*x*x, a3 = r2 + 2*y*y;
            xd = x*radial + k[2]*a1 + k[3]*a2;
            yd = y*radial + k[2]*a3 + k[3]*a1;
        }
        out[i].x = xd*K[0] + K[2];
        out[i].y = yd*K[4] + K[5];
        out[i].z = 0;
    }
}

void undistort_points(struct blob *blobs, int count, vec3f *out,
                      rift_sensor_camera_params *calib)
{
    const double *k = calib->dist_coeffs, *K = calib->camera_matrix.m;
    for (int i = 0; i < count; i++) {
        double x, y;
        if (calib->dist_fisheye) {
            double px = (blobs[i].x - K[2])/K[0];
            double py = (blobs[i].y - K[5])/K[4];
            double td = fmin(sqrt(px*px + py*py), 1.57079632679489661923);
            double theta = td, scale = 0;
            bool converged = td <= 1e-8;
            if (!converged) {
                for (int j = 0; j < 10; j++) {
                    double t2 = theta*theta, t4 = t2*t2, t6 = t4*t2, t8 = t6*t2;
                    double a = k[0]*t2, b = k[1]*t4, c = k[2]*t6, d = k[3]*t8;
                    double fix = (theta*(1 + a + b + c + d) - td) /
                                 (1 + 3*a + 5*b + 7*c + 9*d);
                    theta -= fix;
                    if (fabs(fix) < 1e-8) {
                        converged = true;
                        break;
                    }
                }
                scale = tan(theta)/td;
            }
            if (converged && !(td > 0 && theta < 0)) {
                x = px*scale; y = py*scale;
            } else {
                x = y = -1000000;
            }
        } else {
            double x0 = (blobs[i].x - K[2])*(1/K[0]);
            double y0 = (blobs[i].y - K[5])*(1/K[4]);
            x = x0; y = y0;
            for (int j = 0; j < 5; j++) {
                double r2 = x*x + y*y;
                double inv = 1/(1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2);
                if (inv < 0) {
                    x = x0; y = y0;
                    break;
                }
                double dx = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
                double dy = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;
                x = (x0 - dx)*inv;
                y = (y0 - dy)*inv;
            }
        }
        out[i].x = x; out[i].y = y; out[i].z = 1;
    }
}
