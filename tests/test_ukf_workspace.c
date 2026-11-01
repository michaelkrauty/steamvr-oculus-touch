// SPDX-License-Identifier: Apache-2.0
/* Synthetic native filter regression. Linker wrappers count whole operations,
 * not just the covariance product. Reference mode uses the allocating helper;
 * production mode exercises the owned workspace. No hardware or timing. */
#include "rift-kalman-6dof.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#undef printf
#undef fprintf
#define REQUIRE(x)                                                                                 \
    do {                                                                                           \
        if (!(x)) {                                                                                \
            fprintf(stderr, "failed %s line %d\n", #x, __LINE__);                                  \
            exit(2);                                                                               \
        }                                                                                          \
    } while (0)
static int counting, reference;
static unsigned long live;
static uint64_t cross_hash = UINT64_C(14695981039346656037);
static unsigned long allocs, frees, bytes, updates, failures, logs;
void *__real_malloc(size_t);
void *__real_calloc(size_t, size_t);
void __real_free(void *);
void *__wrap_malloc(size_t n) {
    if (counting) {
        allocs++;
        bytes += n;
    }
    void *p = __real_malloc(n);
    if (p)
        live++;
    return p;
}
void *__wrap_calloc(size_t n, size_t s) {
    if (counting) {
        allocs++;
        bytes += n * s;
    }
    void *p = __real_calloc(n, s);
    if (p)
        live++;
    return p;
}
void *__wrap_realloc(void *p, size_t n) {
    (void)p;
    if (counting) {
        allocs++;
        bytes += n;
    } /* Native math must not resize storage. */
    REQUIRE(false && "native math must not resize storage");
    return NULL;
}
void __wrap_free(void *p) {
    if (counting && p)
        frees++;
    if (p) {
        REQUIRE(live > 0);
        live--;
    }
    __real_free(p);
}
bool __real_ukf_base_update(ukf_base *, ukf_measurement *);
static void hash_matrix(uint64_t *hash, const matrix2d *m) {
    for (int j = 0; j < m->cols; j++)
        for (int i = 0; i < m->rows; i++) {
            double v = MATRIX2D_XY(m, i, j);
            REQUIRE(isfinite(v));
            unsigned char b[sizeof v];
            memcpy(b, &v, sizeof v);
            for (unsigned z = 0; z < sizeof v; z++) {
                *hash ^= b[z];
                *hash *= UINT64_C(1099511628211);
            }
        }
}
bool __wrap_ukf_base_update(ukf_base *u, ukf_measurement *m) {
    bool ok = __real_ukf_base_update(u, m);
    updates++;
    failures += !ok;
    if (ok)
        hash_matrix(&cross_hash, m->Pxz);
    return ok;
}
matrix_result __real_matrix2d_multiplyXYXt_workspace(matrix2d *, const matrix2d *, const matrix2d *,
                                                     matrix2d *);
matrix_result __wrap_matrix2d_multiplyXYXt_workspace(matrix2d *d, const matrix2d *x,
                                                     const matrix2d *y, matrix2d *tmp) {
    if (reference)
        return matrix2d_multiplyXYXt(d, x, y);
    return __real_matrix2d_multiplyXYXt_workspace(d, x, y, tmp);
}
int ohmd_log_printf(const char *fmt, ...) {
    (void)fmt;
    logs++;
    return 0;
}
int ohmd_log_fprintf(FILE *f, const char *fmt, ...) {
    (void)f;
    (void)fmt;
    logs++;
    return 0;
}
int ohmd_log_puts(const char *s) {
    (void)s;
    logs++;
    return 0;
}
void ohmd_assert_fail(const char *a, const char *b, int c, const char *d) {
    fprintf(stderr, "assertion %s %s:%d %s\n", a, b, c, d);
    exit(3);
}
static void begin(void) {
    allocs = frees = bytes = updates = failures = logs = 0;
    counting = 1;
}
static void end(const char *s, unsigned n) {
    counting = 0;
    REQUIRE(updates == n && !failures && !logs);
    REQUIRE(allocs == (reference ? n : 0) && frees == allocs);
    printf("COUNT %s allocations=%lu frees=%lu bytes=%lu successful_updates=%lu\n", s, allocs,
           frees, bytes, updates);
}
static uint64_t hash = UINT64_C(14695981039346656037);
static void snapshot(rift_kalman_6dof_filter *f, unsigned step) {
    hash_matrix(&hash, f->ukf.x_prior);
    hash_matrix(&hash, f->ukf.P_prior);
    printf("STATE step=%u cumulative_hash=%016llx\n", step, (unsigned long long)hash);
    printf("CROSS step=%u cumulative_hash=%016llx\n", step, (unsigned long long)cross_hash);
}
static void dump(rift_kalman_6dof_filter *f) {
    matrix2d *mat[] = {f->ukf.x_prior, f->ukf.P_prior};
    for (int k = 0; k < 2; k++)
        for (int j = 0; j < mat[k]->cols; j++)
            for (int i = 0; i < mat[k]->rows; i++)
                printf("FINAL %d %d %d %a\n", k, i, j, MATRIX2D_XY(mat[k], i, j));
}
static void dump_cross(ukf_measurement **measurements, int mixed) {
    for (int k = 0; k < 3; k++)
        if (k == 0 || mixed) {
            matrix2d *m = measurements[k]->Pxz;
            for (int j = 0; j < m->cols; j++)
                for (int i = 0; i < m->rows; i++)
                    printf("FINAL_CROSS %d %d %d %a\n", k, i, j, MATRIX2D_XY(m, i, j));
        }
}
/* Rectangular, strided views must preserve padding and reject bad shapes before
 * writing either output. The legacy helper is also kept API-compatible. */
static void helper_regression(void) {
    matrix2d *owners[5];
    matrix2d x, y, d, tmp, expected;
    matrix2d *views[] = {&x, &y, &d, &tmp, &expected};
    int rows[] = {3, 2, 3, 3, 3}, cols[] = {2, 2, 3, 2, 3};
    for (int k = 0; k < 5; k++) {
        owners[k] = matrix2d_alloc(rows[k] + 2, cols[k] + 2);
        matrix2d_fill(owners[k], 99);
        REQUIRE(matrix2d_submatrix_ref(owners[k], 1, 1, rows[k], cols[k], views[k]) ==
                MATRIX_RESULT_OK);
    }
    for (int j = 0; j < x.cols; j++)
        for (int i = 0; i < x.rows; i++)
            MATRIX2D_XY(&x, i, j) = .25 + i - j;
    matrix2d_fill_identity(&y);
    REQUIRE(matrix2d_multiplyXYXt(&expected, &x, &y) == MATRIX_RESULT_OK);
    begin();
    REQUIRE(__real_matrix2d_multiplyXYXt_workspace(&d, &x, &y, &tmp) == MATRIX_RESULT_OK);
    end("strided_helper", 0);
    for (int j = 0; j < d.cols; j++)
        for (int i = 0; i < d.rows; i++)
            REQUIRE(MATRIX2D_XY(&d, i, j) == MATRIX2D_XY(&expected, i, j));
#if MATRIX_CHECKS
    matrix2d bad = tmp;
    bad.cols--;
    matrix2d_fill(&d, 99);
    matrix2d_fill(&tmp, 99);
    begin();
    REQUIRE(__real_matrix2d_multiplyXYXt_workspace(&d, &x, &y, &bad) == MATRIX_RESULT_INVALID);
    matrix2d bad_y = y;
    bad_y.rows--;
    REQUIRE(__real_matrix2d_multiplyXYXt_workspace(&d, &x, &bad_y, &tmp) == MATRIX_RESULT_INVALID);
    REQUIRE(matrix2d_multiplyXYXt(&d, &x, &bad_y) == MATRIX_RESULT_INVALID);
    end("invalid_helper", 0);
    for (int j = 0; j < d.cols; j++)
        for (int i = 0; i < d.rows; i++)
            REQUIRE(MATRIX2D_XY(&d, i, j) == 99);
    for (int j = 0; j < tmp.cols; j++)
        for (int i = 0; i < tmp.rows; i++)
            REQUIRE(MATRIX2D_XY(&tmp, i, j) == 99);
#endif
    for (int k = 0; k < 5; k++) {
        for (int j = 0; j < owners[k]->cols; j++)
            for (int i = 0; i < owners[k]->rows; i++)
                if (i == 0 || i > rows[k] || j == 0 || j > cols[k])
                    REQUIRE(MATRIX2D_XY(owners[k], i, j) == 99);
        matrix2d_free(owners[k]);
    }
    REQUIRE(live == 0);
}
static bool reject_measurement(const ukf_base *u, const ukf_measurement *m, const matrix2d *x,
                               matrix2d *z) {
    (void)u;
    (void)m;
    (void)x;
    (void)z;
    return false;
}
int main(int argc, char **argv) {
    REQUIRE(argc == 4);
    int slots = atoi(argv[1]), mixed = atoi(argv[2]);
    reference = atoi(argv[3]);
    REQUIRE(slots >= 0 && slots <= MAX_DELAY_SLOTS);
    helper_regression();
    rift_kalman_6dof_filter f = {0};
    posef p = {.orient = {.w = 1}};
    vec3f gyro = {{0}}, accel = {{0, 9.80665f, 0}}, mag = {{0}};
    rift_kalman_6dof_init(&f, &p, slots);
    uint64_t t = 1000000000;
    ukf_measurement *measurements[] = {&f.m1, &f.m2, &f.m_position};
    matrix2d *scratch[3];
    unsigned scratch_bytes = 0;
    for (int i = 0; i < 3; i++) {
        ukf_measurement *m = measurements[i];
        scratch[i] = m->KPz_tmp;
        REQUIRE(scratch[i]->rows == f.ukf.N_cov && scratch[i]->cols == m->N_cov);
        REQUIRE(scratch[i] != m->K && scratch[i] != m->Pxz && scratch[i] != m->Pz);
        scratch_bytes += sizeof(matrix2d) + sizeof(double) * scratch[i]->rows * scratch[i]->cols;
    }
    begin();
    for (int j = 0; j < slots; j++)
        rift_kalman_6dof_prepare_delay_slot(&f, t, j);
    end("prepare", 0);
    unsigned optical = 0;
    for (unsigned i = 0; i < 4000; i++) {
        gyro.x = .017f * sinf(i * .013f);
        gyro.y = .025f;
        gyro.z = .008f * cosf(i * .021f);
        accel.x = .12f * sinf(i * .019f);
        accel.z = .03f * cosf(i * .011f);
        t += 1000000;
        begin();
        rift_kalman_6dof_imu_update(&f, t, &gyro, &accel, &mag);
        end("imu", 1);
        snapshot(&f, i);
        if (mixed && i % 20 == 19) {
            int slot = slots ? optical % slots : -1;
            rift_kalman_6dof_get_delay_slot_pose_at(&f, t, slot, &p, NULL, NULL, NULL, NULL, NULL);
            p.pos.x += .001f;
            p.pos.z -= .0005f;
            begin();
            if (optical % 2)
                rift_kalman_6dof_position_update(&f, t, &p.pos, slot);
            else
                rift_kalman_6dof_pose_update(&f, t, &p, slot);
            end(optical % 2 ? "position" : "pose", 1);
            snapshot(&f, 4000 + optical);
            if (slots) {
                begin();
                rift_kalman_6dof_release_delay_slot(&f, slot);
                REQUIRE(!f.slot_inuse[slot]);
                rift_kalman_6dof_prepare_delay_slot(&f, t, slot);
                REQUIRE(f.slot_inuse[slot]);
                end("reuse", 0);
                snapshot(&f, 5000 + optical);
            }
            optical++;
        }
    }
    dump(&f);
    dump_cross(measurements, mixed);
    MATRIX2D_XY(f.ukf.P_prior, 0, 0) = -1e8;
    begin();
    rift_kalman_6dof_imu_update(&f, t + 1000000, &gyro, &accel, &mag);
    counting = 0;
    REQUIRE(logs > 0 && updates == 0 && allocs == 0 && frees == 0);
    puts("CONTROL prediction_failure allocations=0 updates=0");
    p = (posef){.orient = {.w = 1}};
    begin();
    rift_kalman_6dof_reset(&f, &p);
    end("reset", 0);
    for (int j = 0; j < slots; j++)
        REQUIRE(!f.slot_inuse[j]);
    begin();
    rift_kalman_6dof_imu_update(&f, t + 2000000, &gyro, &accel, &mag);
    end("post_reset_imu", 1);
    snapshot(&f, 6000);
    dump(&f);
    dump_cross(measurements, mixed);
    for (int i = 0; i < 3; i++)
        REQUIRE(measurements[i]->KPz_tmp == scratch[i]);
    ukf_measurement_fn saved = f.m1.measurement_fn;
    f.m1.measurement_fn = reject_measurement;
    begin();
    REQUIRE(!__wrap_ukf_base_update(&f.ukf, &f.m1));
    counting = 0;
    REQUIRE(updates == 1 && failures == 1 && !logs && !allocs && !frees);
    f.m1.measurement_fn = saved;
    puts("CONTROL measurement_failure allocations=0 updates=1 failures=1");
    snapshot(&f, 6001);
    rift_kalman_6dof_clear(&f);
    REQUIRE(live == 0);
    printf("LIFECYCLE scratch_matrices=3 scratch_bytes=%u retained_allocations=%lu\n",
           scratch_bytes, live);
    puts("PASS");
    return 0;
}
