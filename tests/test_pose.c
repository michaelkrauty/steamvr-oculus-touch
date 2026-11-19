/* SPDX-License-Identifier: Apache-2.0 */
#include "rift-sensor-pose.h"
#include "rift-sensor-camera.h"
#include "test_util.h"
#include <float.h>
#include <string.h>
/* Unity-build the actual C implementations, exposing the private analytic
 * Jacobian without a production API. Allocation poisons work on Windows too. */
#define malloc(...) ((void)sizeof(char[-1]))
#define calloc(...) ((void)sizeof(char[-1]))
#define realloc(...) ((void)sizeof(char[-1]))
#define aligned_alloc(...) ((void)sizeof(char[-1]))
#define free(...) ((void)sizeof(char[-1]))
#include "rift-sensor-pose.c"
#include "rift-sensor-camera.c"

struct pose_fixture {
    const char *name;
    int n, lens, guess;
    double K[9], D[5];
    float q[4], t[3], object[64][3], pixels[64][2];
    int baseline[2][2];
    double baseline_pose[2][6];
};
struct svd_fixture {
    int m, n;
    double A[24], W[4], Ut[24], Vt[16];
};
#include "support/pose_fixtures.inc"

/* A link-time allocation trap covers the camera and solver together, including
 * their first call. Enable with -DPOSE_ALLOCATION_TRAP and GNU ld --wrap flags.
 * The ordinary test remains portable to Windows without linker interposition. */
#ifdef POSE_ALLOCATION_TRAP
static int solving;
void *__real_malloc(size_t);
void *__real_calloc(size_t, size_t);
void *__real_realloc(void *, size_t);
void __real_free(void *);
void *__wrap_malloc(size_t n) { CHECK(!solving); return __real_malloc(n); }
void *__wrap_calloc(size_t n, size_t s) { CHECK(!solving); return __real_calloc(n,s); }
void *__wrap_realloc(void *p, size_t n) { CHECK(!solving); return __real_realloc(p,n); }
void __wrap_free(void *p) { CHECK(!solving); __real_free(p); }
#endif

static bool solve(struct blob *blobs, int n, rift_led *leds, int model_size,
    rift_sensor_camera_params *camera, posef *pose, int *count, int *inliers, bool guess)
{
#ifdef POSE_ALLOCATION_TRAP
    solving = 1;
#endif
    bool ok = estimate_initial_pose(blobs,n,2,leds,model_size,camera,pose,count,inliers,guess);
#ifdef POSE_ALLOCATION_TRAP
    solving = 0;
#endif
    return ok;
}

static void setup(const struct pose_fixture *f, struct blob *blobs, rift_led *leds,
    rift_sensor_camera_params *camera, posef *prior)
{
    memset(camera,0,sizeof(*camera)); memset(prior,0,sizeof(*prior));
    memset(leds,0,64*sizeof(*leds)); memset(blobs,0,100*sizeof(*blobs));
    memcpy(camera->camera_matrix.m,f->K,sizeof(f->K));
    memcpy(camera->dist_coeffs,f->D,sizeof(f->D)); camera->dist_fisheye=f->lens;
    prior->orient.x=f->q[0]; prior->orient.y=f->q[1];
    prior->orient.z=f->q[2]; prior->orient.w=f->q[3];
    memcpy(prior->pos.arr,f->t,sizeof(f->t));
    for (int i=0;i<f->n;i++) {
        memcpy(leds[i].pos.arr,f->object[i],3*sizeof(float));
        blobs[i].x=f->pixels[i][0]; blobs[i].y=f->pixels[i][1];
        blobs[i].led_id=LED_MAKE_ID(2,i);
    }
}

/* Compare unit quaternions using the difference norm, not acos(dot), which
 * loses precision below 1e-6 radians. Quaternion signs are interchangeable. */
static double rotation_error(const posef *pose, const double *r)
{
    double angle=sqrt(r[0]*r[0]+r[1]*r[1]+r[2]*r[2]);
    double scale=angle ? sin(angle/2)/angle : 0.5;
    double ref[4]={r[0]*scale,r[1]*scale,r[2]*scale,cos(angle/2)};
    double q[4]={pose->orient.x,pose->orient.y,pose->orient.z,pose->orient.w};
    double norm=0, dot=0, delta=0;
    for (int k=0;k<4;k++) norm+=q[k]*q[k];
    norm=sqrt(norm);
    for (int k=0;k<4;k++) { q[k]/=norm; dot+=q[k]*ref[k]; }
    for (int k=0;k<4;k++) { double d=q[k]-(dot<0 ? -ref[k] : ref[k]); delta+=d*d; }
    return 4*asin(fmin(1,sqrt(delta)/2));
}

static double translation_error(const posef *pose, const double *r)
{
    double dt=0;
    for (int k=0;k<3;k++) { double d=pose->pos.arr[k]-r[k+3]; dt+=d*d; }
    return sqrt(dt);
}

static void fixtures(void)
{
    double max_t=0, max_r=0;
    int failures=0, successes=0;
    for (size_t c=0;c<sizeof(pose_fixtures)/sizeof(pose_fixtures[0]);c++) {
        const struct pose_fixture *f=&pose_fixtures[c];
        struct blob blobs[100]; rift_led leds[64]; rift_sensor_camera_params camera;
        posef prior, result; int n=-1, inliers=-1;
        setup(f,blobs,leds,&camera,&prior); result=prior;
        bool ok=solve(blobs,f->n,leds,f->n,&camera,&result,&n,&inliers,f->guess);
        CHECK(n==f->n); CHECK(ok==f->baseline[0][0]);
        if (!ok) { CHECK(inliers==0); CHECK(memcmp(&result,&prior,sizeof(result))==0); failures++; }
        else {
            /* The expected pose is OpenCV 4.12's solvePnPRansac, which the
             * solver follows operation for operation. On these well-conditioned
             * cases 1 micrometre and 0.0001 degree leave room only for the float
             * output and the libm differences between compilers. */
            double dt=translation_error(&result,f->baseline_pose[0]);
            double dr=rotation_error(&result,f->baseline_pose[0]);
            CHECK(inliers==f->baseline[0][1]); CHECK(dt<=1e-6); CHECK(dr<=1.7453292519943296e-6);
            max_t=fmax(max_t,dt); max_r=fmax(max_r,dr); successes++;
        }
        /* 5.1 differs in its four-point solver and refinement; reported only. */
        printf("%s ok=%d inliers=%d 4.12=%d/%d 5.1=%d/%d",f->name,ok,inliers,
            f->baseline[0][0],f->baseline[0][1],f->baseline[1][0],f->baseline[1][1]);
        if (ok && f->baseline[1][0])
            printf(" 5.1 dt=%.9g dr=%.9g",translation_error(&result,f->baseline_pose[1]),
                rotation_error(&result,f->baseline_pose[1]));
        putchar('\n');
        for (int repeat=0;repeat<5;repeat++) {
            posef again=prior; int count=-1, consensus=-1;
            CHECK(solve(blobs,f->n,leds,f->n,&camera,&again,&count,&consensus,f->guess)==ok);
            CHECK(count==n); CHECK(consensus==inliers); CHECK(memcmp(&again,&result,sizeof(result))==0);
        }
    }
    printf("pose fixtures: %d results, %d rejected; max 4.12 dt=%.9g m dr=%.9g rad\n",
        successes,failures,max_t,max_r);
}

/* Exactly representable matrices, so 1e-12 absorbs the few roundings while
 * a changed order, sign or random completion of the basis fails. */
static void decompositions(void)
{
    for (size_t c=0;c<sizeof(svd_fixtures)/sizeof(svd_fixtures[0]);c++) {
        const struct svd_fixture *f=&svd_fixtures[c];
        double Ut[24], W[4], Vt[16];
        svd(f->A,f->m,f->n,Ut,W,Vt);
        for (int i=0;i<f->n;i++) CHECK_NEAR(W[i],f->W[i],1e-12);
        for (int i=0;i<f->m*f->n;i++) CHECK_NEAR(Ut[i],f->Ut[i],1e-12);
        for (int i=0;i<f->n*f->n;i++) CHECK_NEAR(Vt[i],f->Vt[i],1e-12);
    }
    puts("pose decomposition checks passed");
}

static void boundaries(void)
{
    const struct pose_fixture *f=NULL;
    for (size_t i=0;i<sizeof(pose_fixtures)/sizeof(pose_fixtures[0]);i++)
        if (!strcmp(pose_fixtures[i].name,"many-lens0-guess0")) f=&pose_fixtures[i];
    CHECK(f);
    struct blob blobs[100]; rift_led leds[64]; rift_sensor_camera_params camera;
    posef prior, result, clean; int n, inliers;
    setup(f,blobs,leds,&camera,&prior); clean=prior;
    CHECK(solve(blobs,f->n,leds,f->n,&camera,&clean,&n,&inliers,false));
    /* Duplicate and foreign labels, negative labels, and labels outside both
     * the model and the 64-bit set must not alter the selected points. */
    blobs[f->n]=blobs[0]; blobs[f->n].x+=100;
    blobs[f->n+1].led_id=LED_MAKE_ID(1,1);
    blobs[f->n+2].led_id=-1; blobs[f->n+3].led_id=LED_MAKE_ID(2,63);
    blobs[f->n+4].led_id=LED_MAKE_ID(2,64); blobs[f->n+5].led_id=LED_MAKE_ID(2,255);
    result=prior;
    CHECK(solve(blobs,f->n+6,leds,f->n,&camera,&result,&n,&inliers,false));
    CHECK(n==f->n); CHECK(memcmp(&result,&clean,sizeof(clean))==0);
    for (int count=0;count<4;count++) {
        result=prior; n=-1; inliers=-1;
        CHECK(!solve(blobs,count,leds,f->n,&camera,&result,&n,&inliers,true));
        CHECK(n==count); CHECK(inliers==0); CHECK(memcmp(&prior,&result,sizeof(prior))==0);
    }
    /* A nonfinite prior must not be silently replaced by the consensus pose. */
    result=prior; result.orient.x=NAN; prior=result;
    CHECK(!solve(blobs,f->n,leds,f->n,&camera,&result,&n,&inliers,true));
    CHECK(inliers==0); CHECK(memcmp(&prior,&result,sizeof(prior))==0);
    setup(f,blobs,leds,&camera,&prior);
    result=prior; camera.camera_matrix.m[0]=0;
    CHECK(!solve(blobs,f->n,leds,f->n,&camera,&result,&n,&inliers,false));
    CHECK(inliers==0); CHECK(memcmp(&prior,&result,sizeof(prior))==0);
    setup(f,blobs,leds,&camera,&prior);
    result=prior; blobs[0].x=INFINITY;
    CHECK(!solve(blobs,f->n,leds,f->n,&camera,&result,&n,&inliers,false));
    CHECK(inliers==0); CHECK(memcmp(&prior,&result,sizeof(prior))==0);
    setup(f,blobs,leds,&camera,&prior);
    result=prior; leds[0].pos.z=NAN;
    CHECK(!solve(blobs,f->n,leds,f->n,&camera,&result,&n,&inliers,false));
    CHECK(inliers==0); CHECK(memcmp(&prior,&result,sizeof(prior))==0);
    setup(f,blobs,leds,&camera,&prior);
    result=prior;
    CHECK(solve(blobs,f->n,leds,f->n,&camera,&result,NULL,NULL,false));
    CHECK(memcmp(&result,&clean,sizeof(clean))==0);
    setup(f,blobs,leds,&camera,&prior);
    /* Four and five points take the direct solution of all of them, as the
     * baseline does: no inlier test, so a wrong label is kept, and neither
     * the caller's pose nor a refinement is used. */
    blobs[3].x+=100; blobs[3].y-=100;
    for (int count=4;count<=5;count++) {
        posef distant=prior; distant.pos.z+=1; distant.orient.x+=.3f;
        result=prior;
        CHECK(solve(blobs,count,leds,f->n,&camera,&result,&n,&inliers,true));
        CHECK(n==count); CHECK(inliers==count); CHECK(memcmp(&prior,&result,sizeof(prior))!=0);
        CHECK(solve(blobs,count,leds,f->n,&camera,&distant,&n,&inliers,false));
        CHECK(memcmp(&distant,&result,sizeof(result))==0);
    }
    /* Count 64 was exercised above; a 65th distinct label must be skipped. */
    for (size_t i=0;i<sizeof(pose_fixtures)/sizeof(pose_fixtures[0]);i++) {
        if (pose_fixtures[i].n!=64) continue;
        setup(&pose_fixtures[i],blobs,leds,&camera,&prior);
        blobs[64].led_id=LED_MAKE_ID(2,64); result=prior;
        CHECK(solve(blobs,65,leds,64,&camera,&result,&n,&inliers,false));
        CHECK(n==64); CHECK(inliers==64);
        /* The hard 64-label limit also applies to models larger than 64. */
        result=prior;
        CHECK(solve(blobs,65,leds,65,&camera,&result,&n,&inliers,false));
        CHECK(n==64); CHECK(inliers==64); break;
    }
    puts("pose boundary and failure checks passed");
}

/* Orthogonal pair rotations give an independent eigensystem and hence a
 * known minimum-norm solution. Weak diagonal modes stay separate so their
 * rank decisions do not depend on cancellation in a rotated matrix. */
static void normal_rank_cases(void)
{
    double A[6][6] = {{0}}, b[6] = {0}, x[6];
    normal_step(A, b, x);
    for (int i = 0; i < 6; i++) CHECK(x[i] == 0);

    /* Eigenvectors (1,1)/sqrt(2), (1,-1)/sqrt(2), plus axes.
     * The sum is approximately 1, so the cutoff is 2*DBL_EPSILON.
     * Weak values are separated by 4x, making a changed cutoff visible. */
    memset(A, 0, sizeof(A));
    A[0][0] = A[0][1] = A[1][0] = A[1][1] = .5;
    A[2][2] = DBL_EPSILON * .5;
    A[3][3] = DBL_EPSILON * 8;
    A[4][4] = 1e-9;
    A[5][5] = 1e-5;
    b[0] = b[1] = 1;
    b[2] = A[2][2] * 3;
    b[3] = A[3][3] * 4;
    b[4] = A[4][4] * -2;
    b[5] = A[5][5] * .25;
    double original[6][6]; memcpy(original, A, sizeof(A));
    normal_step(A, b, x);
    const double expected[6] = {1, 1, 0, 4, -2, .25};
    /* Exact axes and 45-degree rotation need only a few rounding operations;
     * 1e-12 covers those without obscuring any rank or nullspace error. */
    for (int i = 0; i < 6; i++) {
        CHECK(isfinite(x[i])); CHECK_NEAR(x[i], expected[i], 1e-12);
        double residual = -b[i];
        for (int j = 0; j < 6; j++) residual += original[i][j]*x[j];
        CHECK_NEAR(residual, i == 2 ? -b[2] : 0, 1e-14);
    }
    CHECK_NEAR(x[0]-x[1], 0, 1e-14); /* exact nullspace component */
    /* A normal matrix with correlated columns and a 1e10 diagonal range.
     * Construct b directly from the known solution, then independently check
     * both solution and original-system residual. The weak-column solution
     * tolerance accounts for cancellation when subtracting correlated terms. */
    memset(A, 0, sizeof(A));
    const double diagonal[6] = {1e-5, 1e-2, 1, 1e2, 1e3, 1e5};
    const double truth[6] = {.5, -1, 2, -.25, .125, -.75};
    for (int i = 0; i < 6; i++) A[i][i] = diagonal[i];
    A[0][1] = A[1][0] = .25*sqrt(diagonal[0]*diagonal[1]);
    A[2][3] = A[3][2] = .25*sqrt(diagonal[2]*diagonal[3]);
    memcpy(original, A, sizeof(A));
    for (int i = 0; i < 6; i++) {
        b[i] = 0;
        for (int j = 0; j < 6; j++) b[i] += A[i][j]*truth[j];
    }
    normal_step(A, b, x);
    for (int i = 0; i < 6; i++) {
        CHECK(isfinite(x[i])); CHECK_NEAR(x[i], truth[i], 1e-9);
        double residual = -b[i];
        for (int j = 0; j < 6; j++) residual += original[i][j]*x[j];
        CHECK_NEAR(residual, 0, 1e-10);
    }
}

static void mathematics(void)
{
    normal_rank_cases();
    /* Frozen first twelve outputs of RNG(UINT64_MAX), as used by OpenCV's
     * ptsetreg.cpp; modulo UINT32_MAX leaves these particular draws unchanged. */
    static const uint32_t draws[]={130063605,3133359004U,2578348940U,925327173,
        1080261831,2946015512U,94037301,2298661280U,300167573,43921110,776868985,1162377994};
    uint64_t state=UINT64_MAX;
    for (size_t i=0;i<sizeof(draws)/sizeof(draws[0]);i++) CHECK(random_index(&state,UINT32_MAX)==draws[i]);
    /* Central differences are independent of the analytic derivative. The
     * 2e-8 absolute tolerance covers h=1e-6 truncation and double rounding. */
    const double vectors[][3]={{0,0,0},{.17,-.24,.09},{3.141592653589793,0,0},{1e-8,-2e-8,3e-8}};
    for (size_t v=0;v<sizeof(vectors)/sizeof(vectors[0]);v++) {
        double R[9], dR[3][9]; rotation(vectors[v],R,dR);
        for (int j=0;j<3;j++) {
            double plus[3],minus[3],A[9],B[9];
            memcpy(plus,vectors[v],sizeof(plus)); memcpy(minus,plus,sizeof(minus));
            plus[j]+=1e-6; minus[j]-=1e-6;
            rotation(plus,A,NULL); rotation(minus,B,NULL);
            for (int k=0;k<9;k++) CHECK_NEAR(dR[j][k],(A[k]-B[k])/2e-6,2e-8);
        }
    }
    /* Independently constructed SPD system, with disparate diagonal scales. */
    double H[6][6], b[6]={0}, truth[6]={.2,-.7,.003,2,-3,.1}, x[6];
    for (int i=0;i<6;i++) for (int j=0;j<6;j++) {
        H[i][j]=(i==j ? (i+1)*100.0 : 0)+(i+1)*(j+1);
        b[i]+=H[i][j]*truth[j];
    }
    normal_step(H,b,x);
    for (int i=0;i<6;i++) CHECK_NEAR(x[i],truth[i],1e-12);
    /* An exact zero residual must take the zero-angle quaternion limit. */
    struct blob blobs[8]={{0}}; rift_led leds[64];
    rift_sensor_camera_params camera={0}; posef prior={0}, result;
    memset(leds,0,sizeof(leds));
    camera.camera_matrix.m[0]=camera.camera_matrix.m[4]=512;
    camera.camera_matrix.m[8]=1;
    prior.pos.z=1; prior.orient.w=1;
    for (int i=0;i<8;i++) {
        leds[i].pos.x=(i&1) ? .125f : -.125f;
        leds[i].pos.y=(i&2) ? .0625f : -.0625f;
        leds[i].pos.z=(i&4) ? 1 : 0;
        blobs[i].led_id=LED_MAKE_ID(2,i);
        blobs[i].x=512*leds[i].pos.x/(1+leds[i].pos.z);
        blobs[i].y=512*leds[i].pos.y/(1+leds[i].pos.z);
    }
    for (int zero_quaternion=0;zero_quaternion<2;zero_quaternion++) {
        result=prior; result.orient.w=zero_quaternion ? 0 : 1;
        int n,inliers;
        CHECK(solve(blobs,8,leds,8,&camera,&result,&n,&inliers,true));
        CHECK(n==8); CHECK(inliers==8);
        CHECK(result.orient.x==0); CHECK(result.orient.y==0); CHECK(result.orient.z==0);
        CHECK(result.orient.w==1); CHECK(result.pos.z==1);
    }
    puts("pose analytic Jacobian, normal solve and exact-zero checks passed");
}

/* Decisions the end-to-end fixtures cannot pin. */
/* The same three rays admit multiple P3P poses. Project a fourth point from
 * each candidate in turn: it must select that candidate, not simply the
 * first solution returned by P3P. All coordinates are synthetic. */
static void four_point_selection(void)
{
    double X[3][3]={{-.12,-.10,-.08},{.12,-.08,.08},{-.10,.10,.08}};
    pose_point points[4]={0};
    double rays[3][3], distances[3], cosines[3], lengths[4][3];
    for (int i=0;i<3;i++) {
        memcpy(points[i].object,X[i],sizeof(X[i]));
        double z=X[i][2]+.85, norm;
        points[i].image[0]=(X[i][0]+.035)/z;
        points[i].image[1]=(X[i][1]-.02)/z;
        norm=sqrt(points[i].image[0]*points[i].image[0]+points[i].image[1]*points[i].image[1]+1);
        rays[i][0]=points[i].image[0]/norm; rays[i][1]=points[i].image[1]/norm; rays[i][2]=1/norm;
    }
    for (int i=0;i<3;i++) {
        int a=(i+1)%3, b=(i+2)%3;
        distances[i]=0; cosines[i]=0;
        for (int k=0;k<3;k++) {
            double d=X[a][k]-X[b][k];
            distances[i]+=d*d; cosines[i]+=rays[a][k]*rays[b][k];
        }
        distances[i]=sqrt(distances[i]);
    }
    int n=p3p_lengths(lengths,distances,cosines);
    CHECK(n>=2);
    points[3].object[0]=.12; points[3].object[1]=.10; points[3].object[2]=-.08;
    for (int h=0;h<n;h++) {
        double M[3][3], expected[9], translation[3], actual[9], t[3], projected[3];
        for (int i=0;i<3;i++) for (int k=0;k<3;k++) M[i][k]=lengths[h][i]*rays[i][k];
        p3p_align(M,X,expected,translation);
        for (int i=0;i<3;i++) {
            projected[i]=translation[i];
            for (int k=0;k<3;k++) projected[i]+=expected[3*i+k]*points[3].object[k];
        }
        CHECK(fabs(projected[2])>1e-6);
        points[3].image[0]=projected[0]/projected[2];
        points[3].image[1]=projected[1]/projected[2];
        CHECK(four_points(points,actual,t));
        for (int i=0;i<9;i++) CHECK_NEAR(actual[i],expected[i],1e-8);
        for (int i=0;i<3;i++) CHECK_NEAR(t[i],translation[i],1e-8);
    }
}

static void decisions(void)
{
    /* Finite parameters whose squared norm, or whose angle in float, overflows
     * give a non-finite quaternion, which must be refused. */
    const double huge[2][6] = {{1e200,1e200,1e200,0,0,1}, {1e39,0,0,0,0,1}}, quarter[6] = {0,0,1.5707963267948966,0,0,1};
    posef pose;
    for (int i=0;i<2;i++) CHECK(!parameters_pose(huge[i],&pose));
    CHECK(parameters_pose(quarter,&pose));
    CHECK_NEAR(pose.orient.z,sqrt(.5),1e-6); CHECK_NEAR(pose.orient.w,sqrt(.5),1e-6); CHECK(pose.pos.z==1);
    /* A sample replaces the best only with a strictly larger consensus of at
     * least five points. */
    CHECK(!larger_consensus(4,0)); CHECK(larger_consensus(5,0));
    CHECK(!larger_consensus(5,5)); CHECK(larger_consensus(6,5));
    /* The inlier test rounds to float as OpenCV does. With the identity pose
     * at unit depth this point is outside the threshold in exact arithmetic
     * and inside it in float, by a relative 2e-6 either way. */
    const float threshold = (float)(3.0/613), threshold2 = (float)((double)threshold*threshold);
    const pose_point point = {{0x1.0502ecbf8p-2,0,1}, {.25,0,1}};
    const double zero[3] = {0};
    uint64_t mask = 0;
    CHECK((.25-point.object[0])*(.25-point.object[0]) > threshold2);
    CHECK(consensus(&point,1,zero,zero,threshold2,&mask)==1); CHECK(mask==1);
}

int main(void)
{
    mathematics(); decompositions(); fixtures(); boundaries(); four_point_selection(); decisions();
    puts("pose checks passed");
    return 0;
}
