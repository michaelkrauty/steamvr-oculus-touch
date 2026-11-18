/* SPDX-License-Identifier: Apache-2.0
 * Adapted from OpenCV 4.12: solvePnPRansac and its consensus loop
 * (solvepnp.cpp, ptsetreg.cpp), EPnP (epnp.cpp), P3P (p3p.cpp,
 * polynom_solver.cpp), the Jacobi singular value
 * decomposition, back substitution and transposed product it uses
 * (lapack.cpp, matmul.simd.hpp), Rodrigues conversions (calibration_base.cpp)
 * and the Levenberg-Marquardt schedule (compat_ptsetreg.cpp).
 * Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
 * Copyright (C) 2009, Willow Garage Inc., all rights reserved.
 * Copyright (C) 2013, OpenCV Foundation, all rights reserved.
 * Third party copyrights are property of their respective owners.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * The adapted source files also retain this original license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the copyright holders may not be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL CORPORATION OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <float.h>
#include <math.h>
#include <string.h>

#include "rift-sensor-pose.h"
#include "rift-sensor-camera.h"

typedef struct {
    double object[3];
    double image[3];
} pose_point;

/* Row-major Rodrigues matrix and its derivatives with respect to rvec. */
static void rotation(const double *r, double *R, double dR[3][9])
{
    static const double I[9] = {1,0,0,0,1,0,0,0,1};
    static const double dK[3][9] = {
        {0,0,0,0,0,-1,0,1,0}, {0,0,1,0,0,0,-1,0,0},
        {0,-1,0,1,0,0,0,0,0}
    };
    double theta = sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
    if (theta < DBL_EPSILON) {
        memcpy(R, I, sizeof(I));
        if (dR) memcpy(dR, dK, sizeof(dK));
        return;
    }
    double it = 1 / theta, c = cos(theta), s = sin(theta), c1 = 1-c;
    double u[3] = {r[0]*it, r[1]*it, r[2]*it};
    double K[9] = {0,-u[2],u[1],u[2],0,-u[0],-u[1],u[0],0};
    for (int k = 0; k < 9; k++) {
        int a = k/3, b = k%3;
        double uu = u[a]*u[b];
        R[k] = c*I[k] + c1*uu + s*K[k];
        if (dR) for (int j = 0; j < 3; j++) {
            double duu = (a == j ? u[b] : 0) + (b == j ? u[a] : 0);
            dR[j][k] = -s*u[j]*I[k] + (s-2*c1*it)*u[j]*uu +
                c1*it*duu + (c-s*it)*u[j]*K[k] + s*it*dK[j][k];
        }
    }
}

/* OpenCV's RNG (operations.hpp). */
static uint32_t random_next(uint64_t *state)
{
    *state = (uint64_t)(uint32_t)*state*UINT32_C(4164903690) + (uint32_t)(*state >> 32);
    return (uint32_t)*state;
}

static unsigned random_index(uint64_t *state, unsigned n)
{
    return random_next(state) % n;
}

/* The arithmetic from here to estimate_initial_pose follows OpenCV operation
 * for operation. With five points EPnP's 12 x 12 system has a two-dimensional
 * null space whose basis the decomposition orients by rounding, so these
 * results are reproducible only with OpenCV's order of operations, its own
 * hypot, and no fused multiply-add contraction (excluded in the build files). */
static double svd_hypot(double a, double b)
{
    a = fabs(a); b = fabs(b);
    if (a > b) { b /= a; return a*sqrt(1 + b*b); }
    if (b > 0) { a /= b; return b*sqrt(1 + a*a); }
    return 0;
}

/* JacobiSVDImpl_ for double with both bases requested. At holds the n columns
 * of an m x n matrix (m >= n, n <= 12) as rows; on return they are the left
 * singular vectors, W holds the singular values in descending order and Vt
 * the right singular vectors as rows. */
static void jacobi_svd(double *At, int m, int n, double *W, double *Vt)
{
    const double eps = DBL_EPSILON*10;
    int max_iter = m > 30 ? m : 30;
    for (int i = 0; i < n; i++) {
        double sd = 0;
        for (int k = 0; k < m; k++) sd += At[i*m+k]*At[i*m+k];
        W[i] = sd;
        for (int k = 0; k < n; k++) Vt[i*n+k] = 0;
        Vt[i*n+i] = 1;
    }
    for (int iter = 0; iter < max_iter; iter++) {
        bool changed = false;
        for (int i = 0; i < n-1; i++) for (int j = i+1; j < n; j++) {
            double *Ai = At+i*m, *Aj = At+j*m, a = W[i], p = 0, b = W[j], c, s;
            for (int k = 0; k < m; k++) p += Ai[k]*Aj[k];
            if (fabs(p) <= eps*sqrt(a*b)) continue;
            p *= 2;
            double beta = a-b, gamma = svd_hypot(p, beta);
            if (beta < 0) {
                double delta = (gamma-beta)*0.5;
                s = sqrt(delta/gamma);
                c = p/(gamma*s*2);
            } else {
                c = sqrt((gamma+beta)/(gamma*2));
                s = p/(gamma*c*2);
            }
            a = b = 0;
            for (int k = 0; k < m; k++) {
                double t0 = c*Ai[k] + s*Aj[k], t1 = -s*Ai[k] + c*Aj[k];
                Ai[k] = t0; Aj[k] = t1;
                a += t0*t0; b += t1*t1;
            }
            W[i] = a; W[j] = b;
            changed = true;
            double *Vi = Vt+i*n, *Vj = Vt+j*n;
            for (int k = 0; k < n; k++) {
                double t0 = c*Vi[k] + s*Vj[k], t1 = -s*Vi[k] + c*Vj[k];
                Vi[k] = t0; Vj[k] = t1;
            }
        }
        if (!changed) break;
    }
    for (int i = 0; i < n; i++) {
        double sd = 0;
        for (int k = 0; k < m; k++) sd += At[i*m+k]*At[i*m+k];
        W[i] = sqrt(sd);
    }
    for (int i = 0; i < n-1; i++) {
        int j = i;
        for (int k = i+1; k < n; k++) if (W[j] < W[k]) j = k;
        if (i == j) continue;
        double t = W[i]; W[i] = W[j]; W[j] = t;
        for (int k = 0; k < m; k++) { t = At[i*m+k]; At[i*m+k] = At[j*m+k]; At[j*m+k] = t; }
        for (int k = 0; k < n; k++) { t = Vt[i*n+k]; Vt[i*n+k] = Vt[j*n+k]; Vt[j*n+k] = t; }
    }
    /* A zero singular value gets an orthogonal left vector from a fixed
     * random sign pattern. */
    uint64_t state = 0x12345678;
    for (int i = 0; i < n; i++) {
        double *Ai = At+i*m, sd = W[i];
        for (int attempt = 0; attempt < 100 && sd <= DBL_MIN; attempt++) {
            const double val0 = 1./m;
            for (int k = 0; k < m; k++) Ai[k] = (random_next(&state) & 256) ? val0 : -val0;
            for (int pass = 0; pass < 2; pass++) for (int j = 0; j < i; j++) {
                double asum = 0;
                sd = 0;
                for (int k = 0; k < m; k++) sd += Ai[k]*At[j*m+k];
                for (int k = 0; k < m; k++) {
                    double t = Ai[k] - sd*At[j*m+k];
                    Ai[k] = t; asum += fabs(t);
                }
                asum = asum > eps*100 ? 1/asum : 0;
                for (int k = 0; k < m; k++) Ai[k] *= asum;
            }
            sd = 0;
            for (int k = 0; k < m; k++) sd += Ai[k]*Ai[k];
            sd = sqrt(sd);
        }
        double s = sd > DBL_MIN ? 1/sd : 0.;
        for (int k = 0; k < m; k++) Ai[k] *= s;
    }
}

/* cv::SVD::compute of a row-major m x n matrix, m >= n: Ut (n x m) holds
 * the left singular vectors as rows. */
static void svd(const double *A, int m, int n, double *Ut, double *W, double *Vt)
{
    for (int i = 0; i < n; i++) for (int k = 0; k < m; k++) Ut[i*m+k] = A[k*n+i];
    jacobi_svd(Ut, m, n, W, Vt);
}

/* cv::solve with DECOMP_SVD for one right-hand side, m >= n, n <= 5. */
static void svd_solve(const double *A, int m, int n, const double *b, double *x)
{
    double Ut[5*6], W[5], Vt[5*5], threshold = 0;
    svd(A, m, n, Ut, W, Vt);
    for (int i = 0; i < n; i++) { x[i] = 0; threshold += W[i]; }
    threshold *= DBL_EPSILON*2;
    for (int i = 0; i < n; i++) {
        double wi = W[i], s = 0;
        if (fabs(wi) <= threshold) continue;
        wi = 1/wi;
        for (int j = 0; j < m; j++) s += Ut[i*m+j]*b[j];
        s *= wi;
        for (int j = 0; j < n; j++) x[j] = x[j] + s*Vt[i*n+j];
    }
}

/* cv::invert with DECOMP_SVD of a 3 x 3 matrix. */
static void svd_invert3(const double *A, double *X)
{
    double Ut[9], W[3], Vt[9], threshold = 0, buffer[3];
    svd(A, 3, 3, Ut, W, Vt);
    for (int i = 0; i < 9; i++) X[i] = 0;
    for (int i = 0; i < 3; i++) threshold += W[i];
    threshold *= DBL_EPSILON*2;
    for (int i = 0; i < 3; i++) {
        double wi = W[i];
        if (fabs(wi) <= threshold) continue;
        wi = 1/wi;
        for (int j = 0; j < 3; j++) buffer[j] = Ut[i*3+j]*wi;
        for (int r = 0; r < 3; r++) {
            double s = Vt[i*3+r];
            for (int j = 0; j < 3; j++) X[r*3+j] = X[r*3+j] + s*buffer[j];
        }
    }
}

/* cv::mulTransposed(A, D, true): D = A^T A, upper triangle summed by row of A,
 * lower triangle copied. */
static void mul_transposed(const double *A, int rows, int cols, double *D)
{
    for (int i = 0; i < cols; i++) for (int j = i; j < cols; j++) {
        double s = 0;
        for (int k = 0; k < rows; k++) s += A[k*cols+i]*A[k*cols+j];
        D[i*cols+j] = s;
    }
    for (int i = 1; i < cols; i++) for (int j = 0; j < i; j++) D[i*cols+j] = D[j*cols+i];
}

/* cv::Rodrigues from a matrix, which first replaces R by the nearest rotation
 * U Vt. Out-of-range or non-finite input gives the zero vector. */
static void matrix_to_vector(const double *R0, double *r)
{
    double Ut[9], W[3], Vt[9], R[9], s, c, theta;
    for (int k = 0; k < 9; k++) if (!(R0[k] >= -100 && R0[k] < 100)) { r[0] = r[1] = r[2] = 0; return; }
    svd(R0, 3, 3, Ut, W, Vt);
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        double sum = 0;
        for (int k = 0; k < 3; k++) sum += Ut[k*3+i]*Vt[k*3+j];
        R[i*3+j] = sum;
    }
    r[0] = R[7]-R[5]; r[1] = R[2]-R[6]; r[2] = R[3]-R[1];
    s = sqrt((r[0]*r[0] + r[1]*r[1] + r[2]*r[2])*0.25);
    c = (R[0] + R[4] + R[8] - 1)*0.5;
    c = c > 1. ? 1. : c < -1. ? -1. : c;
    theta = acos(c);
    if (s < 1e-5) {
        if (c > 0) { r[0] = r[1] = r[2] = 0; return; }
        double t = (R[0]+1)*0.5;
        r[0] = sqrt(t < 0. ? 0. : t);
        t = (R[4]+1)*0.5;
        r[1] = sqrt(t < 0. ? 0. : t)*(R[1] < 0 ? -1. : 1.);
        t = (R[8]+1)*0.5;
        r[2] = sqrt(t < 0. ? 0. : t)*(R[2] < 0 ? -1. : 1.);
        if (fabs(r[0]) < fabs(r[1]) && fabs(r[0]) < fabs(r[2]) && (R[5] > 0) != (r[1]*r[2] > 0))
            r[2] = -r[2];
        theta /= sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
        for (int i = 0; i < 3; i++) r[i] *= theta;
    } else {
        double vth = 1/(2*s);
        vth *= theta;
        for (int i = 0; i < 3; i++) r[i] *= vth;
    }
}

/* EPnP for exactly five points (epnp.cpp) with the identity camera matrix
 * solvePnPRansac passes for normalised image points. */
typedef struct {
    double pws[15], us[10], alphas[20], pcs[15], cws[4][3], ccs[4][3];
} epnp;

static double epnp_dot(const double *a, const double *b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static double epnp_dist2(const double *a, const double *b)
{
    return (a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]);
}

static void epnp_control_points(epnp *e)
{
    double pw0[15], pw0tpw0[9], dc[3], uct[9], v[9];
    for (int j = 0; j < 3; j++) e->cws[0][j] = 0;
    for (int i = 0; i < 5; i++) for (int j = 0; j < 3; j++) e->cws[0][j] += e->pws[3*i+j];
    for (int j = 0; j < 3; j++) e->cws[0][j] /= 5;
    for (int i = 0; i < 5; i++) for (int j = 0; j < 3; j++) pw0[3*i+j] = e->pws[3*i+j] - e->cws[0][j];
    mul_transposed(pw0, 5, 3, pw0tpw0);
    svd(pw0tpw0, 3, 3, uct, dc, v);
    for (int i = 1; i < 4; i++) {
        double k = sqrt(dc[i-1]/5);
        for (int j = 0; j < 3; j++) e->cws[i][j] = e->cws[0][j] + k*uct[3*(i-1)+j];
    }
}

static void epnp_barycentric(epnp *e)
{
    double cc[9], ci[9];
    for (int i = 0; i < 3; i++) for (int j = 1; j < 4; j++) cc[3*i+j-1] = e->cws[j][i] - e->cws[0][i];
    svd_invert3(cc, ci);
    for (int i = 0; i < 5; i++) {
        const double *pi = e->pws+3*i;
        double *a = e->alphas+4*i;
        for (int j = 0; j < 3; j++)
            a[1+j] = ci[3*j]*(pi[0]-e->cws[0][0]) + ci[3*j+1]*(pi[1]-e->cws[0][1]) +
                ci[3*j+2]*(pi[2]-e->cws[0][2]);
        a[0] = 1.0 - a[1] - a[2] - a[3];
    }
}

/* Focal lengths 1 and principal point 0, written as OpenCV evaluates them. */
static void epnp_fill_M(double *M, const double *as, double u, double v)
{
    double *M1 = M, *M2 = M+12;
    for (int i = 0; i < 4; i++) {
        M1[3*i] = as[i]; M1[3*i+1] = 0.0; M1[3*i+2] = as[i]*(0 - u);
        M2[3*i] = 0.0; M2[3*i+1] = as[i]; M2[3*i+2] = as[i]*(0 - v);
    }
}

static void epnp_L_6x10(const double *ut, double *l)
{
    const double *v[4] = {ut+12*11, ut+12*10, ut+12*9, ut+12*8};
    double dv[4][6][3];
    for (int i = 0; i < 4; i++) {
        int a = 0, b = 1;
        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 3; k++) dv[i][j][k] = v[i][3*a+k] - v[i][3*b+k];
            if (++b > 3) { a++; b = a+1; }
        }
    }
    for (int i = 0; i < 6; i++) {
        double *row = l+10*i;
        row[0] = epnp_dot(dv[0][i], dv[0][i]);
        row[1] = 2.0*epnp_dot(dv[0][i], dv[1][i]);
        row[2] = epnp_dot(dv[1][i], dv[1][i]);
        row[3] = 2.0*epnp_dot(dv[0][i], dv[2][i]);
        row[4] = 2.0*epnp_dot(dv[1][i], dv[2][i]);
        row[5] = epnp_dot(dv[2][i], dv[2][i]);
        row[6] = 2.0*epnp_dot(dv[0][i], dv[3][i]);
        row[7] = 2.0*epnp_dot(dv[1][i], dv[3][i]);
        row[8] = 2.0*epnp_dot(dv[2][i], dv[3][i]);
        row[9] = epnp_dot(dv[3][i], dv[3][i]);
    }
}

/* The three approximations of betas from columns of L. */
static void epnp_betas(const double *l, const double *rho, int approximation, double *betas)
{
    static const int columns[3][5] = {{0,1,3,6}, {0,1,2}, {0,1,2,3,4}};
    static const int widths[3] = {4, 3, 5};
    int w = widths[approximation-1];
    double L[6*5], b[5];
    for (int i = 0; i < 6; i++) for (int j = 0; j < w; j++) L[i*w+j] = l[10*i+columns[approximation-1][j]];
    svd_solve(L, 6, w, rho, b);
    if (approximation == 1) {
        if (b[0] < 0) {
            betas[0] = sqrt(-b[0]);
            betas[1] = -b[1]/betas[0]; betas[2] = -b[2]/betas[0]; betas[3] = -b[3]/betas[0];
        } else {
            betas[0] = sqrt(b[0]);
            betas[1] = b[1]/betas[0]; betas[2] = b[2]/betas[0]; betas[3] = b[3]/betas[0];
        }
        return;
    }
    if (b[0] < 0) {
        betas[0] = sqrt(-b[0]);
        betas[1] = (b[2] < 0) ? sqrt(-b[2]) : 0.0;
    } else {
        betas[0] = sqrt(b[0]);
        betas[1] = (b[2] > 0) ? sqrt(b[2]) : 0.0;
    }
    if (b[1] < 0) betas[0] = -betas[0];
    betas[2] = approximation == 3 ? b[3]/betas[0] : 0.0;
    betas[3] = 0.0;
}

/* Householder QR solve of the 6 x 4 Gauss-Newton system, including its
 * pivot search over rows k to 4 and its early return, which leaves x as it
 * was. */
static void epnp_qr_solve(double A[6][4], double *b, double *x)
{
    double A1[4], A2[4];
    for (int k = 0; k < 4; k++) {
        double eta = fabs(A[k][k]);
        for (int i = k+1; i < 6; i++) {
            double elt = fabs(A[i-1][k]);
            if (eta < elt) eta = elt;
        }
        if (eta == 0) return;
        double sum2 = 0, inv_eta = 1./eta;
        for (int i = k; i < 6; i++) { A[i][k] *= inv_eta; sum2 += A[i][k]*A[i][k]; }
        double sigma = sqrt(sum2);
        if (A[k][k] < 0) sigma = -sigma;
        A[k][k] += sigma;
        A1[k] = sigma*A[k][k];
        A2[k] = -eta*sigma;
        for (int j = k+1; j < 4; j++) {
            double sum = 0;
            for (int i = k; i < 6; i++) sum += A[i][k]*A[i][j];
            double tau = sum/A1[k];
            for (int i = k; i < 6; i++) A[i][j] -= tau*A[i][k];
        }
    }
    for (int j = 0; j < 4; j++) {
        double tau = 0;
        for (int i = j; i < 6; i++) tau += A[i][j]*b[i];
        tau /= A1[j];
        for (int i = j; i < 6; i++) b[i] -= tau*A[i][j];
    }
    x[3] = b[3]/A2[3];
    for (int i = 2; i >= 0; i--) {
        double sum = 0;
        for (int j = i+1; j < 4; j++) sum += A[i][j]*x[j];
        x[i] = (b[i] - sum)/A2[i];
    }
}

static void epnp_gauss_newton(const double *l, const double *rho, double *betas)
{
    double x[4] = {0};
    for (int iter = 0; iter < 5; iter++) {
        double A[6][4], b[6];
        for (int i = 0; i < 6; i++) {
            const double *L = l+10*i;
            A[i][0] = 2*L[0]*betas[0] + L[1]*betas[1] + L[3]*betas[2] + L[6]*betas[3];
            A[i][1] = L[1]*betas[0] + 2*L[2]*betas[1] + L[4]*betas[2] + L[7]*betas[3];
            A[i][2] = L[3]*betas[0] + L[4]*betas[1] + 2*L[5]*betas[2] + L[8]*betas[3];
            A[i][3] = L[6]*betas[0] + L[7]*betas[1] + L[8]*betas[2] + 2*L[9]*betas[3];
            b[i] = rho[i] - (L[0]*betas[0]*betas[0] + L[1]*betas[0]*betas[1] +
                L[2]*betas[1]*betas[1] + L[3]*betas[0]*betas[2] + L[4]*betas[1]*betas[2] +
                L[5]*betas[2]*betas[2] + L[6]*betas[0]*betas[3] + L[7]*betas[1]*betas[3] +
                L[8]*betas[2]*betas[3] + L[9]*betas[3]*betas[3]);
        }
        epnp_qr_solve(A, b, x);
        for (int i = 0; i < 4; i++) betas[i] += x[i];
    }
}

static double epnp_R_and_t(epnp *e, const double *ut, const double *betas, double R[3][3], double *t)
{
    for (int i = 0; i < 4; i++) e->ccs[i][0] = e->ccs[i][1] = e->ccs[i][2] = 0.0;
    for (int i = 0; i < 4; i++) {
        const double *v = ut+12*(11-i);
        for (int j = 0; j < 4; j++) for (int k = 0; k < 3; k++) e->ccs[j][k] += betas[i]*v[3*j+k];
    }
    for (int i = 0; i < 5; i++) {
        const double *a = e->alphas+4*i;
        for (int j = 0; j < 3; j++)
            e->pcs[3*i+j] = a[0]*e->ccs[0][j] + a[1]*e->ccs[1][j] + a[2]*e->ccs[2][j] + a[3]*e->ccs[3][j];
    }
    if (e->pcs[2] < 0.0) {
        for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) e->ccs[i][j] = -e->ccs[i][j];
        for (int i = 0; i < 15; i++) e->pcs[i] = -e->pcs[i];
    }
    double pc0[3] = {0}, pw0[3] = {0}, abt[9] = {0}, Ut[9], W[3], Vt[9];
    for (int i = 0; i < 5; i++) for (int j = 0; j < 3; j++) {
        pc0[j] += e->pcs[3*i+j]; pw0[j] += e->pws[3*i+j];
    }
    for (int j = 0; j < 3; j++) { pc0[j] /= 5; pw0[j] /= 5; }
    for (int i = 0; i < 5; i++) {
        const double *pc = e->pcs+3*i, *pw = e->pws+3*i;
        for (int j = 0; j < 3; j++) {
            abt[3*j] += (pc[j]-pc0[j])*(pw[0]-pw0[0]);
            abt[3*j+1] += (pc[j]-pc0[j])*(pw[1]-pw0[1]);
            abt[3*j+2] += (pc[j]-pc0[j])*(pw[2]-pw0[2]);
        }
    }
    svd(abt, 3, 3, Ut, W, Vt);
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
        R[i][j] = Ut[i]*Vt[j] + Ut[3+i]*Vt[3+j] + Ut[6+i]*Vt[6+j];
    const double det = R[0][0]*R[1][1]*R[2][2] + R[0][1]*R[1][2]*R[2][0] + R[0][2]*R[1][0]*R[2][1] -
        R[0][2]*R[1][1]*R[2][0] - R[0][1]*R[1][0]*R[2][2] - R[0][0]*R[1][2]*R[2][1];
    if (det < 0) for (int j = 0; j < 3; j++) R[2][j] = -R[2][j];
    for (int i = 0; i < 3; i++) t[i] = pc0[i] - epnp_dot(R[i], pw0);
    double sum2 = 0.0;
    for (int i = 0; i < 5; i++) {
        const double *pw = e->pws+3*i;
        double Xc = epnp_dot(R[0], pw) + t[0], Yc = epnp_dot(R[1], pw) + t[1];
        double inv_Zc = 1.0/(epnp_dot(R[2], pw) + t[2]);
        double ue = Xc*inv_Zc, ve = Yc*inv_Zc, u = e->us[2*i], v = e->us[2*i+1];
        sum2 += sqrt((u-ue)*(u-ue) + (v-ve)*(v-ve));
    }
    return sum2/5;
}

/* epnp::compute_pose; R is row-major. */
static void epnp_pose(epnp *e, double *R, double *t)
{
    double M[10*12], mtm[144], d[12], ut[144], v[144], l[60], rho[6];
    double betas[4][4] = {{0}}, errors[4] = {0}, Rs[4][3][3], ts[4][3];
    epnp_control_points(e);
    epnp_barycentric(e);
    for (int i = 0; i < 5; i++) epnp_fill_M(M+24*i, e->alphas+4*i, e->us[2*i], e->us[2*i+1]);
    mul_transposed(M, 10, 12, mtm);
    svd(mtm, 12, 12, ut, d, v);
    epnp_L_6x10(ut, l);
    rho[0] = epnp_dist2(e->cws[0], e->cws[1]); rho[1] = epnp_dist2(e->cws[0], e->cws[2]);
    rho[2] = epnp_dist2(e->cws[0], e->cws[3]); rho[3] = epnp_dist2(e->cws[1], e->cws[2]);
    rho[4] = epnp_dist2(e->cws[1], e->cws[3]); rho[5] = epnp_dist2(e->cws[2], e->cws[3]);
    for (int k = 1; k < 4; k++) {
        epnp_betas(l, rho, k, betas[k]);
        epnp_gauss_newton(l, rho, betas[k]);
        errors[k] = epnp_R_and_t(e, ut, betas[k], Rs[k], ts[k]);
    }
    int N = 1;
    if (errors[2] < errors[1]) N = 2;
    if (errors[3] < errors[N]) N = 3;
    memcpy(R, Rs[N], 9*sizeof(double));
    memcpy(t, ts[N], 3*sizeof(double));
}

static bool prior_parameters(const posef *pose, double *p)
{
    double q[4] = {pose->orient.x, pose->orient.y, pose->orient.z, pose->orient.w};
    double norm = 0;
    for (int i = 0; i < 4; i++) {
        if (!isfinite(q[i])) return false;
        norm += q[i]*q[i];
    }
    if (norm == 0) { q[3] = 1; norm = 1; }
    double scale = 1/sqrt(norm);
    for (int i = 0; i < 4; i++) q[i] *= scale;
    double x=q[0], y=q[1], z=q[2], w=q[3];
    double R[9] = {1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y,
        2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x,
        2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x*x-2*y*y};
    matrix_to_vector(R, p);
    for (int i = 0; i < 3; i++) {
        p[i+3] = pose->pos.arr[i];
        if (!isfinite(p[i+3])) return false;
    }
    return true;
}

/* Symmetric Jacobi pseudoinverse. The six-dimensional damped normal matrix
 * is positive semidefinite; this also handles rank deficiency without heap
 * storage. The cutoff is OpenCV's SVD back-substitution cutoff. */
static void normal_step(double A[6][6], const double *b, double *x)
{
    double V[6][6] = {{0}};
    for (int i = 0; i < 6; i++) V[i][i] = 1;
    for (int sweep = 0; sweep < 100; sweep++) {
        bool changed = false;
        for (int p = 0; p < 5; p++) for (int q = p+1; q < 6; q++) {
            double a = A[p][q];
            if (fabs(a) <= DBL_EPSILON*sqrt(fabs(A[p][p]*A[q][q]))) continue;
            changed = true;
            double tau = (A[q][q]-A[p][p])/(2*a);
            double t = copysign(1.0, tau)/(fabs(tau)+hypot(1, tau));
            double c = 1/sqrt(1+t*t), s = t*c;
            A[p][p] -= t*a; A[q][q] += t*a;
            A[p][q] = A[q][p] = 0;
            for (int k = 0; k < 6; k++) {
                if (k != p && k != q) {
                    double ap=A[k][p], aq=A[k][q];
                    A[k][p] = A[p][k] = c*ap-s*aq;
                    A[k][q] = A[q][k] = s*ap+c*aq;
                }
                double vp=V[k][p], vq=V[k][q];
                V[k][p]=c*vp-s*vq; V[k][q]=s*vp+c*vq;
            }
        }
        if (!changed) break;
    }
    double cutoff = 0;
    for (int i = 0; i < 6; i++) { cutoff += fabs(A[i][i]); x[i] = 0; }
    cutoff *= 2*DBL_EPSILON;
    for (int j = 0; j < 6; j++) if (fabs(A[j][j]) > cutoff) {
        double d = 0;
        for (int k = 0; k < 6; k++) d += V[k][j]*b[k];
        d /= A[j][j];
        for (int k = 0; k < 6; k++) x[k] += V[k][j]*d;
    }
}

/* Residual is projection minus observation, as in solvePnP ITERATIVE. */
static double residual(const pose_point *points, int n, uint64_t mask,
    const double *p, double H[6][6], double *g)
{
    double R[9], dR[3][9], error = 0;
    rotation(p, R, H ? dR : NULL);
    if (H) { memset(H, 0, 36*sizeof(double)); memset(g, 0, 6*sizeof(double)); }
    for (int i = 0; i < n; i++) if (mask & (UINT64_C(1) << i)) {
        const double *v = points[i].object;
        double xyz[3];
        for (int k = 0; k < 3; k++) xyz[k] = R[3*k]*v[0]+R[3*k+1]*v[1]+R[3*k+2]*v[2]+p[k+3];
        double iz = xyz[2] ? 1/xyz[2] : 1;
        double xy[2] = {xyz[0]*iz, xyz[1]*iz};
        for (int k = 0; k < 2; k++) {
            double e = xy[k]-points[i].image[k];
            error += e*e;
            if (H) {
                double J[6] = {0};
                for (int j = 0; j < 3; j++) {
                    double dz = dR[j][6]*v[0]+dR[j][7]*v[1]+dR[j][8]*v[2];
                    double dx = dR[j][3*k]*v[0]+dR[j][3*k+1]*v[1]+dR[j][3*k+2]*v[2];
                    J[j] = iz*(dx-xy[k]*dz);
                }
                J[k+3] = iz; J[5] = -xy[k]*iz;
                for (int a = 0; a < 6; a++) {
                    g[a] += J[a]*e;
                    for (int b = 0; b < 6; b++) H[a][b] += J[a]*J[b];
                }
            }
        }
    }
    return error;
}

static bool refine(const pose_point *points, int n, uint64_t mask, double *p)
{
    int exponent = -3;
    for (int iter = 0; iter < 20; iter++) {
        double H[6][6], g[6], trial[6], error = residual(points,n,mask,p,H,g), next;
        if (!isfinite(error)) return false;
        for (;;) {
            double A[6][6], step[6];
            memcpy(A, H, sizeof(A));
            double lambda = exp(exponent*log(10.0));
            for (int k = 0; k < 6; k++) A[k][k] *= 1+lambda;
            normal_step(A, g, step);
            for (int k = 0; k < 6; k++) trial[k] = p[k]-step[k];
            next = residual(points,n,mask,trial,NULL,NULL);
            if (!isfinite(next)) return false;
            if (next <= error || ++exponent > 16) break;
        }
        if (exponent > -16) exponent--;
        double diff = 0, norm = 0;
        for (int k = 0; k < 6; k++) {
            double d = trial[k]-p[k]; diff += d*d; norm += p[k]*p[k];
            p[k] = trial[k];
        }
        if (sqrt(diff)/(sqrt(norm)+DBL_EPSILON) < FLT_EPSILON) break;
    }
    for (int k = 0; k < 6; k++) if (!isfinite(p[k])) return false;
    return true;
}

/* solvePnPRansac's inlier test: OpenCV projects in double with the model's
 * rotation vector, rounds to float and compares the float squared error. */
static int consensus(const pose_point *points, int n, const double *r, const double *t,
    float threshold2, uint64_t *mask)
{
    double R[9];
    int good = 0;
    rotation(r, R, NULL);
    *mask = 0;
    for (int i = 0; i < n; i++) {
        const double *v = points[i].object;
        double x = R[0]*v[0] + R[1]*v[1] + R[2]*v[2] + t[0];
        double y = R[3]*v[0] + R[4]*v[1] + R[5]*v[2] + t[1];
        double z = R[6]*v[0] + R[7]*v[1] + R[8]*v[2] + t[2];
        z = z ? 1./z : 1;
        x *= z; y *= z;
        float dx = (float)points[i].image[0] - (float)x, dy = (float)points[i].image[1] - (float)y;
        float error = 0;
        error += dx*dx;
        error += dy*dy;
        if (error <= threshold2) { *mask |= UINT64_C(1) << i; good++; }
    }
    return good;
}

/* solvePnPRansac keeps a strictly larger consensus, of at least five points. */
static bool larger_consensus(int good, int best)
{
    return good > (best > 4 ? best : 4);
}

/* RANSACUpdateNumIters for five-point samples. */
static int ransac_iterations(double p, double ep, int max_iters)
{
    double num = 1. - p, denom = 1. - pow(1. - ep, 5);
    if (num < DBL_MIN) num = DBL_MIN;
    if (denom < DBL_MIN) return 0;
    num = log(num); denom = log(denom);
    return denom >= 0 || -num >= max_iters*(-denom) ? max_iters : (int)lrint(num/denom);
}

/* OpenCV 4.12's P3P (p3p.cpp, polynom_solver.cpp): Gao's solution, main
 * branch only. */
#define CV_PI 3.1415926535897932384626433832795

static int solve_deg2(double a, double b, double c, double *x1, double *x2)
{
    double delta = b*b - 4*a*c;
    if (delta < 0) return 0;
    double inv_2a = 0.5/a;
    if (delta == 0) { *x1 = -b*inv_2a; *x2 = *x1; return 1; }
    double sqrt_delta = sqrt(delta);
    *x1 = (-b + sqrt_delta)*inv_2a;
    *x2 = (-b - sqrt_delta)*inv_2a;
    return 2;
}

static int solve_deg3(double a, double b, double c, double d, double *x0, double *x1, double *x2)
{
    if (a == 0) {
        if (b == 0) {
            if (c == 0) return 0;
            *x0 = -d/c;
            return 1;
        }
        *x2 = 0;
        return solve_deg2(b, c, d, x0, x1);
    }
    double inv_a = 1./a;
    double b_a = inv_a*b, b_a2 = b_a*b_a;
    double c_a = inv_a*c;
    double d_a = inv_a*d;
    double Q = (3*c_a - b_a2)/9;
    double R = (9*b_a*c_a - 27*d_a - 2*b_a*b_a2)/54;
    double Q3 = Q*Q*Q;
    double D = Q3 + R*R;
    double b_a_3 = (1./3.)*b_a;
    if (Q == 0) {
        if (R == 0) { *x0 = *x1 = *x2 = -b_a_3; return 3; }
        *x0 = cbrt(2*R) - b_a_3;
        return 1;
    }
    if (D <= 0) {
        double theta = acos(R/sqrt(-Q3));
        double sqrt_Q = sqrt(-Q);
        *x0 = 2*sqrt_Q*cos(theta/3.0) - b_a_3;
        *x1 = 2*sqrt_Q*cos((theta + 2*CV_PI)/3.0) - b_a_3;
        *x2 = 2*sqrt_Q*cos((theta + 4*CV_PI)/3.0) - b_a_3;
        return 3;
    }
    double AD = 0., BD = 0., R_abs = fabs(R);
    if (R_abs > DBL_EPSILON) {
        AD = cbrt(R_abs + sqrt(D));
        AD = (R >= 0) ? AD : -AD;
        BD = -Q/AD;
    }
    *x0 = AD + BD - b_a_3;
    return 1;
}

static int solve_deg4(double a, double b, double c, double d, double e, double *x)
{
    if (a == 0) { x[3] = 0; return solve_deg3(b, c, d, e, x, x+1, x+2); }
    double inv_a = 1./a;
    b *= inv_a; c *= inv_a; d *= inv_a; e *= inv_a;
    double b2 = b*b, bc = b*c, b3 = b2*b;
    double r0, r1, r2;
    int n = solve_deg3(1, -c, d*b - 4*e, 4*c*e - d*d - b2*e, &r0, &r1, &r2);
    if (n == 0) return 0;
    double R2 = 0.25*b2 - c + r0, R;
    if (R2 < 0) return 0;
    R = sqrt(R2);
    double inv_R = 1./R;
    int nb_real_roots = 0;
    double D2, E2;
    if (R < 10E-12) {
        double temp = r0*r0 - 4*e;
        if (temp < 0) D2 = E2 = -1;
        else {
            double sqrt_temp = sqrt(temp);
            D2 = 0.75*b2 - 2*c + 2*sqrt_temp;
            E2 = D2 - 4*sqrt_temp;
        }
    } else {
        double u = 0.75*b2 - 2*c - R2, v = 0.25*inv_R*(4*bc - 8*d - b3);
        D2 = u + v;
        E2 = u - v;
    }
    double b_4 = 0.25*b, R_2 = 0.5*R;
    if (D2 >= 0) {
        double D = sqrt(D2);
        nb_real_roots = 2;
        double D_2 = 0.5*D;
        x[0] = R_2 + D_2 - b_4;
        x[1] = x[0] - D;
    }
    if (E2 >= 0) {
        double E = sqrt(E2), E_2 = 0.5*E;
        if (nb_real_roots == 0) {
            x[0] = -R_2 + E_2 - b_4;
            x[1] = x[0] - E;
            nb_real_roots = 2;
        } else {
            x[2] = -R_2 + E_2 - b_4;
            x[3] = x[2] - E;
            nb_real_roots = 4;
        }
    }
    return nb_real_roots;
}

static int p3p_lengths(double lengths[4][3], const double *distances, const double *cosines)
{
    double p = cosines[0]*2, q = cosines[1]*2, r = cosines[2]*2;
    double inv_d22 = 1./(distances[2]*distances[2]);
    double a = inv_d22*(distances[0]*distances[0]);
    double b = inv_d22*(distances[1]*distances[1]);
    double a2 = a*a, b2 = b*b, p2 = p*p, q2 = q*q, r2 = r*r;
    double pr = p*r, pqr = q*pr;
    if (p2 + q2 + r2 - pqr - 1 == 0) return 0;
    double ab = a*b, a_2 = 2*a;
    double A = -2*b + b2 + a2 + 1 + ab*(2 - r2) - a_2;
    if (A == 0) return 0;
    double a_4 = 4*a;
    double B = q*(-2*(ab + a2 + 1 - b) + r2*ab + a_4) + pr*(b - b2 + ab);
    double C = q2 + b2*(r2 + p2 - 2) - b*(p2 + pqr) - ab*(r2 + pqr) + (a2 - a_2)*(2 + q2) + 2;
    double D = pr*(ab-b2+b) + q*((p2-2)*b + 2*(ab - a2) + a_4 - 2);
    double E = 1 + 2*(b - a - ab) + b2 - b*p2 + a2;
    double temp = (p2*(a-1+b) + r2*(a-1-b) + pqr - a*pqr);
    double b0 = b*temp*temp;
    if (b0 == 0) return 0;
    double real_roots[4];
    int n = solve_deg4(A, B, C, D, E, real_roots);
    if (n == 0) return 0;
    int nb_solutions = 0;
    double r3 = r2*r, pr2 = p*r2, r3q = r3*q;
    double inv_b0 = 1./b0;
    for (int i = 0; i < n; i++) {
        double x = real_roots[i];
        if (x <= 0) continue;
        double x2 = x*x;
        double b1 =
            ((1-a-b)*x2 + (q*a-q)*x + 1 - a + b) *
            (((r3*(a2 + ab*(2 - r2) - a_2 + b2 - 2*b + 1))*x +
            (r3q*(2*(b-a2) + a_4 + ab*(r2 - 2) - 2) + pr2*(1 + a2 + 2*(ab-a-b) + r2*(b - b2) + b2)))*x2 +
            (r3*(q2*(1-2*a+a2) + r2*(b2-ab) - a_4 + 2*(a2 - b2) + 2) + r*p2*(b2 + 2*(ab - b - a) + 1 + a2) + pr2*q*(a_4 + 2*(b - ab - a2) - 2 - r2*b))*x +
            2*r3q*(a_2 - b - a2 + ab - 1) + pr2*(q2 - a_4 + 2*(a2 - b2) + r2*b + q2*(a2 - a_2) + 2) +
            p2*(p*(2*(ab - a - b) + a2 + b2 + 1) + 2*q*r*(b + a_2 - a2 - ab - 1)));
        if (b1 <= 0) continue;
        double y = inv_b0*b1;
        double v = x2 + y*y - x*y*r;
        if (v <= 0) continue;
        double Z = distances[2]/sqrt(v);
        lengths[nb_solutions][0] = x*Z;
        lengths[nb_solutions][1] = y*Z;
        lengths[nb_solutions][2] = Z;
        nb_solutions++;
    }
    return nb_solutions;
}

static void p3p_jacobi_4x4(double *A, double *D, double *U)
{
    double B[4], Z[4] = {0};
    for (int i = 0; i < 16; i++) U[i] = i%5 ? 0. : 1.;
    B[0] = A[0]; B[1] = A[5]; B[2] = A[10]; B[3] = A[15];
    memcpy(D, B, sizeof(B));
    for (int iter = 0; iter < 50; iter++) {
        double sum = fabs(A[1]) + fabs(A[2]) + fabs(A[3]) + fabs(A[6]) + fabs(A[7]) + fabs(A[11]);
        if (sum == 0.0) return;
        double tresh = (iter < 3) ? 0.2*sum/16. : 0.0;
        for (int i = 0; i < 3; i++) {
            double *pAij = A + 5*i + 1;
            for (int j = i+1; j < 4; j++) {
                double Aij = *pAij, eps_machine = 100.0*fabs(Aij);
                if (iter > 3 && fabs(D[i]) + eps_machine == fabs(D[i]) && fabs(D[j]) + eps_machine == fabs(D[j]))
                    *pAij = 0.0;
                else if (fabs(Aij) > tresh) {
                    double hh = D[j] - D[i], t;
                    if (fabs(hh) + eps_machine == fabs(hh)) t = Aij/hh;
                    else {
                        double theta = 0.5*hh/Aij;
                        t = 1.0/(fabs(theta) + sqrt(1.0 + theta*theta));
                        if (theta < 0.0) t = -t;
                    }
                    hh = t*Aij;
                    Z[i] -= hh; Z[j] += hh; D[i] -= hh; D[j] += hh;
                    *pAij = 0.0;
                    double c = 1.0/sqrt(1 + t*t), s = t*c, tau = s/(1.0 + c);
                    for (int k = 0; k <= i-1; k++) {
                        double g = A[k*4+i], h = A[k*4+j];
                        A[k*4+i] = g - s*(h + g*tau); A[k*4+j] = h + s*(g - h*tau);
                    }
                    for (int k = i+1; k <= j-1; k++) {
                        double g = A[i*4+k], h = A[k*4+j];
                        A[i*4+k] = g - s*(h + g*tau); A[k*4+j] = h + s*(g - h*tau);
                    }
                    for (int k = j+1; k < 4; k++) {
                        double g = A[i*4+k], h = A[j*4+k];
                        A[i*4+k] = g - s*(h + g*tau); A[j*4+k] = h + s*(g - h*tau);
                    }
                    for (int k = 0; k < 4; k++) {
                        double g = U[k*4+i], h = U[k*4+j];
                        U[k*4+i] = g - s*(h + g*tau); U[k*4+j] = h + s*(g - h*tau);
                    }
                }
                pAij++;
            }
        }
        for (int i = 0; i < 4; i++) B[i] += Z[i];
        memcpy(D, B, sizeof(B));
        memset(Z, 0, sizeof(Z));
    }
}

/* p3p::align: X holds the three object points; R is row-major. */
static void p3p_align(double M_end[3][3], const double X[3][3], double *R, double *T)
{
    double C_start[3], C_end[3], s[9], Qs[16] = {0}, evs[4], U[16], q[4];
    for (int i = 0; i < 3; i++) C_end[i] = (M_end[0][i] + M_end[1][i] + M_end[2][i])/3;
    for (int i = 0; i < 3; i++) C_start[i] = (X[0][i] + X[1][i] + X[2][i])/3;
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
        s[i*3+j] = (X[0][i]*M_end[0][j] + X[1][i]*M_end[1][j] + X[2][i]*M_end[2][j])/3 - C_end[j]*C_start[i];
    Qs[0] = s[0] + s[4] + s[8];
    Qs[5] = s[0] - s[4] - s[8];
    Qs[10] = s[4] - s[8] - s[0];
    Qs[15] = s[8] - s[0] - s[4];
    Qs[4] = Qs[1] = s[5] - s[7];
    Qs[8] = Qs[2] = s[6] - s[2];
    Qs[12] = Qs[3] = s[1] - s[3];
    Qs[9] = Qs[6] = s[3] + s[1];
    Qs[13] = Qs[7] = s[6] + s[2];
    Qs[14] = Qs[11] = s[7] + s[5];
    p3p_jacobi_4x4(Qs, evs, U);
    int i_ev = 0;
    double ev_max = evs[0];
    for (int i = 1; i < 4; i++) if (evs[i] > ev_max) ev_max = evs[i_ev = i];
    for (int i = 0; i < 4; i++) q[i] = U[i*4+i_ev];
    double q02 = q[0]*q[0], q12 = q[1]*q[1], q22 = q[2]*q[2], q32 = q[3]*q[3];
    double q0_1 = q[0]*q[1], q0_2 = q[0]*q[2], q0_3 = q[0]*q[3];
    double q1_2 = q[1]*q[2], q1_3 = q[1]*q[3], q2_3 = q[2]*q[3];
    R[0] = q02 + q12 - q22 - q32; R[1] = 2.*(q1_2 - q0_3); R[2] = 2.*(q1_3 + q0_2);
    R[3] = 2.*(q1_2 + q0_3); R[4] = q02 + q22 - q12 - q32; R[5] = 2.*(q2_3 - q0_1);
    R[6] = 2.*(q1_3 - q0_2); R[7] = 2.*(q2_3 + q0_1); R[8] = q02 + q32 - q12 - q22;
    for (int i = 0; i < 3; i++) T[i] = C_end[i] - (R[3*i]*C_start[0] + R[3*i+1]*C_start[1] + R[3*i+2]*C_start[2]);
}

/* solvePnP with SOLVEPNP_P3P for four points: of the solutions for the first
 * three, the one with the least squared reprojection error over all four,
 * ties going to the smaller error of the fourth. */
static bool four_points(const pose_point *points, double *R, double *t)
{
    double mu[4], mv[4], mk[3], X[3][3], distances[3], cosines[3], lengths[4][3];
    double best = INFINITY, best_fourth = INFINITY, Rs[9], ts[3];
    for (int i = 0; i < 4; i++) { mu[i] = points[i].image[0]; mv[i] = points[i].image[1]; }
    for (int i = 0; i < 3; i++) {
        double norm = sqrt(mu[i]*mu[i] + mv[i]*mv[i] + 1);
        mk[i] = 1./norm; mu[i] *= mk[i]; mv[i] *= mk[i];
        memcpy(X[i], points[i].object, sizeof(X[i]));
    }
    distances[0] = sqrt((X[1][0]-X[2][0])*(X[1][0]-X[2][0]) + (X[1][1]-X[2][1])*(X[1][1]-X[2][1]) + (X[1][2]-X[2][2])*(X[1][2]-X[2][2]));
    distances[1] = sqrt((X[0][0]-X[2][0])*(X[0][0]-X[2][0]) + (X[0][1]-X[2][1])*(X[0][1]-X[2][1]) + (X[0][2]-X[2][2])*(X[0][2]-X[2][2]));
    distances[2] = sqrt((X[0][0]-X[1][0])*(X[0][0]-X[1][0]) + (X[0][1]-X[1][1])*(X[0][1]-X[1][1]) + (X[0][2]-X[1][2])*(X[0][2]-X[1][2]));
    cosines[0] = mu[1]*mu[2] + mv[1]*mv[2] + mk[1]*mk[2];
    cosines[1] = mu[0]*mu[2] + mv[0]*mv[2] + mk[0]*mk[2];
    cosines[2] = mu[0]*mu[1] + mv[0]*mv[1] + mk[0]*mk[1];
    int n = p3p_lengths(lengths, distances, cosines);
    bool found = false;
    for (int h = 0; h < n; h++) {
        double M[3][3], fourth = 0, total = 0;
        for (int i = 0; i < 3; i++) {
            M[i][0] = lengths[h][i]*mu[i]; M[i][1] = lengths[h][i]*mv[i]; M[i][2] = lengths[h][i]*mk[i];
        }
        p3p_align(M, X, Rs, ts);
        const double *v = points[3].object;
        double X3p = Rs[0]*v[0] + Rs[1]*v[1] + Rs[2]*v[2] + ts[0];
        double Y3p = Rs[3]*v[0] + Rs[4]*v[1] + Rs[5]*v[2] + ts[1];
        double Z3p = Rs[6]*v[0] + Rs[7]*v[1] + Rs[8]*v[2] + ts[2];
        double mu3p = X3p/Z3p, mv3p = Y3p/Z3p;
        fourth = (mu3p - mu[3])*(mu3p - mu[3]) + (mv3p - mv[3])*(mv3p - mv[3]);
        for (int i = 0; i < 4; i++) {
            const double *w = points[i].object;
            double x = Rs[0]*w[0] + Rs[1]*w[1] + Rs[2]*w[2] + ts[0];
            double y = Rs[3]*w[0] + Rs[4]*w[1] + Rs[5]*w[2] + ts[1];
            double z = Rs[6]*w[0] + Rs[7]*w[1] + Rs[8]*w[2] + ts[2];
            z = z ? 1./z : 1;
            double dx = points[i].image[0] - x*z, dy = points[i].image[1] - y*z;
            total += dx*dx;
            total += dy*dy;
        }
        if (total < best || (total == best && fourth < best_fourth)) {
            best = total; best_fourth = fourth; found = true;
            memcpy(R, Rs, sizeof(Rs)); memcpy(t, ts, sizeof(ts));
        }
    }
    return found;
}

static void epnp_points(epnp *e, const pose_point *points, const int *idx)
{
    for (int k = 0; k < 5; k++) {
        memcpy(e->pws+3*k, points[idx[k]].object, 3*sizeof(double));
        memcpy(e->us+2*k, points[idx[k]].image, 2*sizeof(double));
    }
}

/* The baseline's float conversion (oquatf_init_axis), except that a zero
 * rotation gives the identity instead of NaN and a result that does not fit
 * in float is refused: finite parameters can still overflow the squared norm
 * or the float angle. */
static bool parameters_pose(const double *p, posef *result)
{
    double angle = sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
    if (angle == 0) {
        result->orient.x = result->orient.y = result->orient.z = 0; result->orient.w = 1;
    } else {
        double inorm = 1.0f/angle;
        float v[3] = {(float)(p[0]*inorm), (float)(p[1]*inorm), (float)(p[2]*inorm)};
        if (v[0] != 0 || v[1] != 0 || v[2] != 0) {
            float length = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
            for (int k = 0; k < 3; k++) v[k] /= length;
        }
        float half = (float)angle/2.0f;
        result->orient.x = v[0]*sinf(half); result->orient.y = v[1]*sinf(half);
        result->orient.z = v[2]*sinf(half); result->orient.w = cosf(half);
    }
    for (int k = 0; k < 4; k++) if (!isfinite(result->orient.arr[k])) return false;
    for (int k = 0; k < 3; k++) {
        result->pos.arr[k] = (float)p[k+3];
        if (!isfinite(result->pos.arr[k])) return false;
    }
    return true;
}

/* cv::solvePnPRansac as the baseline called it: four points take the P3P
 * solution and five the EPnP solution, without an inlier test or refinement;
 * more take the largest consensus of EPnP on five-point samples, refined over
 * its inliers from the caller's pose. */
bool estimate_initial_pose(struct blob *blobs, int num_blobs, int device_id,
    rift_led *leds, int num_led_pos, rift_sensor_camera_params *calib,
    posef *pose, int *num_leds_out, int *num_inliers, bool use_extrinsic_guess)
{
    pose_point points[64];
    struct blob selected[64];
    vec3f image[64];
    uint64_t taken = 0, mask = 0;
    int n = 0, best = 0;
    double R[9], t[3], p[6];
    if (num_inliers) *num_inliers = 0;
    for (int i = 0; i < num_blobs; i++) {
        int id = blobs[i].led_id;
        if (LED_OBJECT_ID(id) != device_id) continue;
        id = LED_LOCAL_ID(id);
        if (id < 0 || id >= num_led_pos || id >= 64 || (taken & (UINT64_C(1)<<id))) continue;
        taken |= UINT64_C(1)<<id;
        selected[n] = blobs[i];
        for (int k = 0; k < 3; k++) points[n].object[k] = leds[id].pos.arr[k];
        n++;
    }
    if (num_leds_out) *num_leds_out = n;
    if (n < 4 || !isfinite(calib->camera_matrix.m[0]) || calib->camera_matrix.m[0] <= 0) return false;
    undistort_points(selected, n, image, calib);
    for (int i = 0; i < n; i++) {
        points[i].image[0] = image[i].x; points[i].image[1] = image[i].y; points[i].image[2] = 1;
        for (int k = 0; k < 3; k++) if (!isfinite(points[i].object[k]) || !isfinite(points[i].image[k])) return false;
    }
    if (n <= 5) {
        static const int first[5] = {0, 1, 2, 3, 4};
        epnp e;
        if (n == 4) {
            if (!four_points(points, R, t)) return false;
        } else {
            epnp_points(&e, points, first);
            epnp_pose(&e, R, t);
        }
        for (int k = 0; k < 9; k++) if (!isfinite(R[k])) return false;
        matrix_to_vector(R, p);
        memcpy(p+3, t, sizeof(t));
        best = n;
    } else {
        float threshold = (float)(3.0/calib->camera_matrix.m[0]);
        float threshold2 = (float)((double)threshold*threshold);
        uint64_t state = UINT64_MAX;
        double model[6];
        for (int iter = 0, niters = 100; iter < niters; iter++) {
            int idx[5];
            for (int k = 0; k < 5; k++) {
                bool duplicate;
                do {
                    idx[k] = (int)random_index(&state, (unsigned)n); duplicate = false;
                    for (int j = 0; j < k; j++) if (idx[k] == idx[j]) duplicate = true;
                } while (duplicate);
            }
            epnp e;
            double r[3];
            uint64_t inliers;
            epnp_points(&e, points, idx);
            epnp_pose(&e, R, t);
            matrix_to_vector(R, r);
            int good = consensus(points, n, r, t, threshold2, &inliers);
            if (larger_consensus(good, best)) {
                best = good; mask = inliers;
                memcpy(model, r, sizeof(r)); memcpy(model+3, t, sizeof(t));
                niters = ransac_iterations((float)0.99, (double)(n-good)/n, niters);
            }
        }
        if (!best) return false;
        if (use_extrinsic_guess) {
            if (!prior_parameters(pose,p)) return false;
        } else memcpy(p, model, sizeof(model));
        if (!refine(points,n,mask,p)) return false;
    }
    for (int k = 0; k < 6; k++) if (!isfinite(p[k])) return false;
    posef result = *pose;
    if (!parameters_pose(p, &result)) return false;
    *pose=result;
    if (num_inliers) *num_inliers=best;
    return true;
}
