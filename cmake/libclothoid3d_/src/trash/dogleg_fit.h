#ifndef DOGLEG_FIT_H
#define DOGLEG_FIT_H

#include "dogleg.h"
#include "eq14.h"
#include "eq45.h"
#include "eq44.h"
#include "string.h"

// Function to calculate the Euclidean distance between two points in 3D space
double distance_6(double x0, double y0, double z0, double x1, double y1, double z1) {
    return sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2) + pow(z1 - z0, 2));
}

// Solves compile error by not finding this equation...
void eq44_G123(const double thetai0, const double thetai4,
               const double kappai0, const double kappai4,
               const double sharpnessi0, const double sharpnessi4,
               double yi1, // Initial guess value, unknown var.
               double s1,  // Initial guess value, unknown var.

               double *sharpnessi1, double *sharpnessi2, double *sharpnessi3,
               double *kappai1, double *kappai2, double *kappai3,
               double *thetai1, double *thetai2, double *thetai3,
               double *yi2, double *yi3, double *yi4);

// Residual function for dense Jacobian
void residual_function(const double *p, double *x, double* J, void* cookie) {
    // Casting undefined to defined double.
    double* params = (double*)cookie;

    double y11 = p[0];
    double y21 = p[1];
    double s1  = p[2];

    printf("y11: %f \n", y11);
    printf("y21: %f \n", y21);
    printf("s1: %f \n", s1);

    double theta10 = params[0];
    double theta14 = params[1];
    double kappa10 = params[2];
    double kappa14 = params[3];
    double sharpness10 = params[4];
    double sharpness14 = params[5];

    double theta20 = params[6];
    double theta24 = params[7];
    double kappa20 = params[8];
    double kappa24 = params[9];
    double sharpness20 = params[10];
    double sharpness24 = params[11];

    double xs = params[12];
    double ys = params[13];
    double zs = params[14];

    double xe = params[15];
    double ye = params[16];
    double ze = params[17];

    double y12, y13, y14;
    double y22, y23, y24;

    printf("xs: %f \n",xs);
    printf("ys: %f \n",ys);
    printf("zs: %f \n",zs);
    printf("xe: %f \n",xe);
    printf("ye: %f \n",ye);
    printf("ze: %f \n",ze);

    double theta11, theta12, theta13;
    double theta21, theta22, theta23;

    double kappa11, kappa12, kappa13;
    double kappa21, kappa22, kappa23;

    double sharpness11, sharpness12, sharpness13;
    double sharpness21, sharpness22, sharpness23;

    // 1. Calculate constraints for the clothoid compound using initial guess values.

    // eq44, first step, solve G3 continuity for i=1.
    eq44_G123(theta10, theta14, kappa10, kappa14, sharpness10, sharpness14, y11, s1,
              &sharpness11, &sharpness12, &sharpness13,
              &kappa11, &kappa12, &kappa13,
              &theta11, &theta12, &theta13,
              &y12, &y13, &y14);

    // eq44, second step, solve G3 continuity for i=2.
    eq44_G123(theta20, theta24, kappa20, kappa24, sharpness20, sharpness24, y21, s1,
              &sharpness21, &sharpness22, &sharpness23,
              &kappa21, &kappa22, &kappa23,
              &theta21, &theta22, &theta23,
              &y22, &y23, &y24);

    double x1, y1, z1; // Endpoint clothoid 1.
    double x2, y2, z2; // Endpoint clothoid 2.
    double x3, y3, z3; // Endpoint clothoid 3.
    double x4, y4, z4; // Endpoint clothoid 4.
    double l1, l2, l3, l4;

    // 2. Calculate clothoid compound endpoint using the initial guess values for y11, y21, s1.
    eq14_gauss(s1, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, y11, y21, &x1, &y1, &z1, &l1);
    eq14_gauss(s1, x1, y1, z1, theta11, theta21, kappa11, kappa21, sharpness11, sharpness21, y12, y22, &x2, &y2, &z2, &l2);
    eq14_gauss(s1, x2, y2, z2, theta12, theta22, kappa12, kappa22, sharpness12, sharpness22, y13, y23, &x3, &y3, &z3, &l3);
    eq14_gauss(s1, x3, y3, z3, theta13, theta23, kappa13, kappa23, sharpness13, sharpness23, y14, y24, &x4, &y4, &z4, &l4);

    printf("x4: %f \n",x4);
    printf("y4: %f \n",y4);
    printf("z4: %f \n",z4);

    x[0] = xe - x4;
    x[1] = ye - y4;
    x[2] = ze - z4;

    // Jacobian (gradient of the residual)
    if (J) {
        // Numerical differentiation to approximate the Jacobian
        double h = 0.01; // Small step size
        double p_perturbed[3];
        double x_perturbed[3];

        for (int j = 0; j < 3; j++) {
            memcpy(p_perturbed, p, sizeof(double) * 3);
            p_perturbed[j] += h;
            residual_function(p_perturbed, x_perturbed, NULL, cookie);
            for (int k = 0; k < 3; k++) {
                J[k * 3 + j] = (x_perturbed[k] - x[k]) / h; // d(x[k])/d(p[j])
            }
        }
    }
}

int dogleg_fit(const double theta10, const double theta14,
               const double kappa10, const double kappa14,
               const double sharpness10, const double sharpness14,

               const double theta20, const double theta24,
               const double kappa20, const double kappa24,
               const double sharpness20, const double sharpness24,

               const double xs, const double ys, const double zs,
               const double xe, const double ye, const double ze,

               double *y11, double *y21, double *s1) {

    // Initial guess for y11, y21, s1.
    double p[3] = {*y11, *y21, *s1};

    // Right-hand side result
    double user_data[20];
    user_data[0] = theta10;
    user_data[1] = theta14;
    user_data[2] = kappa10;
    user_data[3] = kappa14;
    user_data[4] = sharpness10;
    user_data[5] = sharpness14;
    user_data[6] = theta20;
    user_data[7] = theta24;
    user_data[8] = kappa20;
    user_data[9] = kappa24;
    user_data[10] = sharpness20;
    user_data[11] = sharpness24;
    user_data[12] = xs;
    user_data[13] = ys;
    user_data[14] = zs;
    user_data[15] = xe;
    user_data[16] = ye;
    user_data[17] = ze;

    // Solver parameters
    dogleg_parameters2_t parameters;
    dogleg_getDefaultParameters(&parameters);

    // Set termination thresholds
    parameters.Jt_x_threshold = 1e-6;
    parameters.update_threshold = 1e-6;
    parameters.trustregion_threshold = 1e-6;
    parameters.max_iterations = 2000;

    // Solver context
    dogleg_solverContext_t* ctx = NULL;

    // Call the dense optimizer
    double residual_norm = dogleg_optimize_dense2(
                p,                // Initial guess and output solution
                3,                // Number of state variables (y11, y21, s1)
                3,                // Number of measurements (residuals)
                residual_function, // Residual function
                user_data,         // Cookie (user data)
                &parameters,      // Solver parameters
                &ctx              // Solver context (optional)
                );

    // Check if the solver succeeded
    if (residual_norm >= 0) {
        printf("\nDogleg solver: \n");
        printf("Solution found: y11 = %f, y21 = %f, s1 = %f\n", p[0], p[1], p[2]);
        printf("Residual norm: %f\n", residual_norm);

        // Update the output variables
        *y11 = p[0];
        *y21 = p[1];
        *s1 = p[2];
    } else {
        printf("Solver failed.\n");
    }

    // Free the solver context if it was created
    if (ctx) {
        dogleg_freeContext(&ctx);
    }

    return 0;
}

#endif // DOGLEG_FIT_H
