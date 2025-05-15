/********************************************************************
*   Written by Grotius, alias Skynet.
*   michelwijnja@gmail.com
*
* Author: Michel Wijnja
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2024 All rights reserved.
********************************************************************/
#ifndef SINGLE_CLOTHOID_FIT_H
#define SINGLE_CLOTHOID_FIT_H

#include "../eq14.h"
#include "../eq18.h"
#include "../eq19.h"
#include "../eq20.h"
#include "../eq30.h"
#include "dogleg.h"

// Residual function for dogleg optimizer
void residual_function_clothoid_dogleg(const double *p, double *x, double* J, void* cookie) {
    // Casting cookie to access the parameters
    double* params = (double*)cookie;

    // Extract the current guess for gamma20 and s1
    double gamma10 = p[0];
    double gamma20 = p[1];
    double s1 = p[2];

    // Extract the fixed parameters from the cookie
    double theta10 = params[0];
    double kappa10 = params[1];
    double sharpness10 = params[2];

    double theta20 = params[3];
    double kappa20 = params[4];
    double sharpness20 = params[5];

    double xs = params[6];
    double ys = params[7];
    double zs = params[8];

    double xe = params[9];
    double ye = params[10];
    double ze = params[11];

    // Variables for the computed point
    double xi, yi, zi, ltot;

    // Call the clothoid equation function to compute the point (xi, yi, zi)
    eq14_gauss(s1, xs, ys, zs,
               theta10, theta20,
               kappa10, kappa20,
               sharpness10, sharpness20,
               gamma10, gamma20,
               &xi, &yi, &zi, &ltot);

      printf("xi: %f, y: %f, zi: %f \n", xi, yi, zi);

    // Compute the residuals (errors)
    double dx = xe - xi;
    double dy = ye - yi;
    double dz = ze - zi;

    // Calculate the Euclidean error
    double error = sqrt(dx * dx + dy * dy + dz * dz);

    // Optionally print the error for debugging
    // printf("error: %f \n", error);

    // Return the residuals in x[] array (to be minimized by the optimizer)
    x[0] = dx;
    x[1] = dy;
    x[2] = dz;  // Include the third component (dz) for the residual

    // Compute the Jacobian (partial derivatives)
    double delta = 1e-6;  // Small perturbation for numerical differentiation

    // Derivatives of residuals with respect to each parameter
    // dx/dgamma10
    double p1 = p[0] + delta;
    eq14_gauss(s1, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, p1, gamma20, &xi, &yi, &zi, &ltot);
    double dx_dgamma10 = (xe - xi) - dx;

    // dx/dgamma20
    double p2 = p[1] + delta;
    eq14_gauss(s1, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, gamma10, p2, &xi, &yi, &zi, &ltot);
    double dx_dgamma20 = (xe - xi) - dx;

    // dx/ds1
    double p3 = p[2] + delta;
    eq14_gauss(p3, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, gamma10, gamma20, &xi, &yi, &zi, &ltot);
    double dx_ds1 = (xe - xi) - dx;

    // Assign the Jacobian values
    J[0] = dx_dgamma10 / delta;  // d(dx)/d(gamma10)
    J[1] = dx_dgamma20 / delta;  // d(dx)/d(gamma20)
    J[2] = dx_ds1 / delta;       // d(dx)/d(s1)

    printf("J0 %f \n",J[0]);
    printf("J1 %f \n",J[1]);
    printf("J2 %f \n",J[2]);

    // Repeat the same process for dy and dz
    // dy/dgamma10
    eq14_gauss(s1, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, p1, gamma20, &xi, &yi, &zi, &ltot);
    double dy_dgamma10 = (ye - yi) - dy;

    // dy/dgamma20
    eq14_gauss(s1, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, gamma10, p2, &xi, &yi, &zi, &ltot);
    double dy_dgamma20 = (ye - yi) - dy;

    // dy/ds1
    eq14_gauss(p3, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, gamma10, gamma20, &xi, &yi, &zi, &ltot);
    double dy_ds1 = (ye - yi) - dy;

    J[3] = dy_dgamma10 / delta;
    J[4] = dy_dgamma20 / delta;
    J[5] = dy_ds1 / delta;

    printf("J3 %f \n",J[3]);
    printf("J4 %f \n",J[4]);
    printf("J5 %f \n",J[5]);

    // dz/dgamma10
    eq14_gauss(s1, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, p1, gamma20, &xi, &yi, &zi, &ltot);
    double dz_dgamma10 = (ze - zi) - dz;

    // dz/dgamma20
    eq14_gauss(s1, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, gamma10, p2, &xi, &yi, &zi, &ltot);
    double dz_dgamma20 = (ze - zi) - dz;

    // dz/ds1
    eq14_gauss(p3, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, gamma10, gamma20, &xi, &yi, &zi, &ltot);
    double dz_ds1 = (ze - zi) - dz;

    J[6] = dz_dgamma10 / delta;
    J[7] = dz_dgamma20 / delta;
    J[8] = dz_ds1 / delta;

    printf("J6 %f \n",J[6]);
    printf("J7 %f \n",J[7]);
    printf("J8 %f \n",J[8]);
}

int single_clothoid_fit_dogleg(const double theta10,
                                const double kappa10,
                                const double sharpness10,
                                const double theta20,
                                const double kappa20,
                                const double sharpness20,
                                const double xs, const double ys, const double zs,
                                const double xe, const double ye, const double ze,
                                double *gamma10, double *gamma20, double *s1) {

    // Initial guess
    double p[3] = {*gamma10, *gamma20, *s1};

    // User data (cookie)
    double user_data[12] = {
        theta10, kappa10, sharpness10,
        theta20, kappa20, sharpness20,
        xs, ys, zs,
        xe, ye, ze
    };

    // Solver parameters
    dogleg_parameters2_t parameters;
    dogleg_getDefaultParameters(&parameters);

    // Set termination thresholds
    parameters.Jt_x_threshold = 1e-6;
    parameters.update_threshold = 1e-6;
    parameters.trustregion_threshold = 1e-6;
    parameters.max_iterations = 20;

    // Solver context
    dogleg_solverContext_t* ctx = NULL;

    // Call the dense optimizer
    double residual_norm = dogleg_optimize_dense2(
                p,                // Initial guess and output solution
                3,                // Number of state variables (y11, y21, s1)
                3,                // Number of measurements (residuals)
                residual_function_clothoid_dogleg, // Residual function
                user_data,         // Cookie (user data)
                &parameters,      // Solver parameters
                &ctx              // Solver context (optional)
                );

    int r=0;

    // Check if the solver succeeded
    if (residual_norm < 1e-6) {
        printf("\nPowel Example: \n");
        printf("Solution found: gamma10: %f, gamma20: %f, s1: %f \n", p[0], p[1], p[2]);
        printf("Residual norm: %f\n", residual_norm);
    } else {
        r=-1;
    }

    // Free the solver context if it was created
    if (ctx) {
        dogleg_freeContext(&ctx);
    }

    printf("results anyway: gamma10: %f, gamma20: %f, s1: %f \n", p[0], p[1], p[2]);
    *gamma10=p[0];
    *gamma20=p[1];
    *s1=p[2];

    return r;
}

// Plot a single clothoid
void run_single_clothoid_fit_dogleg(){

    // Print to file for visualization (if needed)
    FILE *data_file = fopen("clothoid_point.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    // Start point.
    double xs = 0.0;
    double ys = 0.0;
    double zs = 0.0;

    // Target point.
    double xe = 11.00;
    double ye = 5.00;
    double ze = 2.2;

    // Guess.
    double s1 = 0.001;
    double gamma10 = 0.0001; // Z
    double gamma20 = 0.0001; // Xy.

    double degrees_xy_start = 0.0;
    double degrees_z_start = -20.0;

    // Example for z component:
    // Initial values at spline start, end point for z.
    double theta10 = degrees_z_start * (M_PI / 180.0); // To radians.
    double kappa10 = 0;
    double sharpness10 = 0;

    // Example for xy component:
    // Initial values at spline start, end point for xy.
    double theta20 = degrees_xy_start * (M_PI / 180.0);
    double kappa20 = 0;
    double sharpness20 = 0;

    // Solve xy. use same z.
    int r= single_clothoid_fit_dogleg(theta10, kappa10, sharpness10, theta20, kappa20, sharpness20,
                               xs, ys, zs, xe, ye, zs, &gamma10, &gamma20, &s1);

    printf("r: %d \n",r);

    if(r!=0){
        return;
    }

    // Solve xyz.
    ze=2;
    r= single_clothoid_fit_dogleg(theta10, kappa10, sharpness10, theta20, kappa20, sharpness20,
                               xs, ys, zs, xe, ye, ze, &gamma10, &gamma20, &s1);

    printf("r: %d \n",r);

    if(r!=0){
        return;
    }

    double ltot;
    double xi,yi,zi;
    double segments=100;

    // Clothoid 1. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double s = progress * s1;


        eq14_gauss( s, xs, ys, zs,
                    theta10, theta20,
                    kappa10,  kappa20,
                    sharpness10, sharpness20,
                    gamma10,  gamma20,
                    &xi, &yi, &zi, &ltot);
        fprintf(data_file, "%lf %lf %lf\n", xi, yi, zi);
    }
    printf("endpoint x: %f y: %f z: %f \n",xi,yi,zi);
    printf("ltot: %f \n",ltot);

    // Derivates at end of curve. Z.
    double theta_z, kappa_z, sharpness_z;
    eq18_single_clothoid(ltot,sharpness10,gamma10,&sharpness_z);
    eq19_single_clothoid(ltot,sharpness10,kappa10,gamma10,&kappa_z);
    eq20_single_clothoid(ltot,sharpness10,kappa10,theta10,gamma10,&theta_z);
    printf("sharpness_z: %f \n",sharpness_z);
    printf("kappa_z: %f \n",kappa_z);
    printf("theta_z_degrees: %f \n",theta_z * (180.0 / M_PI));

    // Derivates at end of curve. XY.
    double theta_xy, kappa_xy, sharpness_xy;
    eq18_single_clothoid(ltot,sharpness20,gamma20,&sharpness_xy);
    eq19_single_clothoid(ltot,sharpness20,kappa20,gamma20,&kappa_xy);
    eq20_single_clothoid(ltot,sharpness20,kappa20,theta20,gamma20,&theta_xy);
    printf("sharpness_xy: %f \n",sharpness_xy);
    printf("kappa_xy: %f \n",kappa_xy);
    printf("theta_xy_degrees: %f \n",theta_xy * (180.0 / M_PI));


    fclose(data_file);

    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up 3D plotting
        fprintf(gnuplot, "set title 'Clothoid 3D Trajectory'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");

        // Set fixed ranges for X, Y, and Z axes
        fprintf(gnuplot, "set xrange [-5:20]\n");   // Set the fixed range for X
        fprintf(gnuplot, "set yrange [-5:20]\n");   // Set the fixed range for Y
        fprintf(gnuplot, "set zrange [-5:20]\n");   // Set the fixed range for Z

        // Set view angle for better 3D visualization
        fprintf(gnuplot, "set view 70, 20\n");  // You can adjust the angle here

        // Plot the data in 3D (use correct filename)
        fprintf(gnuplot, "splot 'clothoid_point.dat' using 1:2:3 with lines\n");

        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

#endif // SINGLE_CLOTHOID_FIT_H
