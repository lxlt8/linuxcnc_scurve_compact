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
#ifndef SINGLE_CLOTHOID_FIT_GSL_H
#define SINGLE_CLOTHOID_FIT_GSL_H

#include <stdio.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multimin.h>

/* Residual function. Calculates fit error based on distance between
 * curpos -> tarpos.
 */
void residual_function_clothoid_gsl(const double *p, double *x, void* cookie) {
    double* params = (double*)cookie;

    double gamma10 = p[0];
    double gamma20 = p[1];
    double s1  = p[2];

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

    double xi, yi, zi, ltot;

    eq14_gauss(s1, xs, ys, zs,
               theta10, theta20,
               kappa10, kappa20,
               sharpness10, sharpness20,
               gamma10, gamma20,
               &xi, &yi, &zi, &ltot);

    x[0] = xe - xi;
    x[1] = ye - yi;
    x[2] = ze - zi;
}

/* Objective function for GSL
 * This is the iteration function that repeats until treshold
 * is within limits, ie result < 1e-6
 *
 */
double objective_function_gsl(const gsl_vector *v, void *cookie) {
    double p[3];
    p[0] = gsl_vector_get(v, 0); // gamma10
    p[1] = gsl_vector_get(v, 1); // gamma20
    p[2] = gsl_vector_get(v, 2); // s1

    double x[3];
    residual_function_clothoid_gsl(p, x, cookie);

    // Compute the Euclidean norm of the residuals
    double residual_norm = 0.0;
    for (int i = 0; i < 3; i++) {
        residual_norm += x[i] * x[i];
    }
    return sqrt(residual_norm);
}

/* Setup gsl solver and run the objective function.
 *
 */
void single_clothoid_fit_gsl(const double theta10,
                             const double kappa10,
                             const double sharpness10,
                             const double theta20,
                             const double kappa20,
                             const double sharpness20,
                             const double xs, const double ys, const double zs,
                             const double xe, const double ye, const double ze,
                             double *gamma10, double *gamma20, double *s1) {

    // Initial guess
    gsl_vector *x = gsl_vector_alloc(3);
    gsl_vector_set(x, 0, *gamma10);
    gsl_vector_set(x, 1, *gamma20);
    gsl_vector_set(x, 2, *s1);

    // User data (cookie)
    double user_data[12] = {
        theta10, kappa10, sharpness10,
        theta20, kappa20, sharpness20,
        xs, ys, zs,
        xe, ye, ze
    };

    // Set up the Nelder-Mead solver
    const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex2;
    gsl_multimin_fminimizer *s = gsl_multimin_fminimizer_alloc(T, 3);

    gsl_multimin_function F;
    F.n = 3;
    F.f = objective_function_gsl;
    F.params = user_data;

    // Set initial step sizes
    gsl_vector *step_sizes = gsl_vector_alloc(3);
    gsl_vector_set_all(step_sizes, 0.1); // Initial step sizes for each parameter

    // Initialize the solver
    gsl_multimin_fminimizer_set(s, &F, x, step_sizes);

    // Optimization parameters
    double tolerance = 1e-6;
    int max_iterations = 1000;
    int status;
    int iter = 0;

    // Run the solver
    do {
        iter++;
        status = gsl_multimin_fminimizer_iterate(s);

        if (status != 0) {
            printf("Warning: solver failed to converge (status = %d)\n", status);
            break;
        }

        // Check for convergence
        double size = gsl_multimin_fminimizer_size(s);
        status = gsl_multimin_test_size(size, tolerance);

        if (status == GSL_SUCCESS) {
            printf("Converged after %d iterations\n", iter);
        }

    } while (status == GSL_CONTINUE && iter < max_iterations);

    // Update the output variables
    *gamma10 = gsl_vector_get(s->x, 0);
    *gamma20 = gsl_vector_get(s->x, 1);
    *s1 = gsl_vector_get(s->x, 2);

    printf("Solution found: gamma10 = %f, gamma20 = %f, s1 = %f\n", *gamma10, *gamma20, *s1);

    // Free memory
    gsl_vector_free(x);
    gsl_vector_free(step_sizes);
    gsl_multimin_fminimizer_free(s);
}

/* Example. Fit a single clothoid given a target position.
 * Plot the result in GNU plot.
 *
 * Tested ok.
 */
int run_single_clothoid_fit_gsl() {

    // Print to file for visualization (if needed)
    FILE *data_file = fopen("clothoid_point.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return 0;
    }

    // Start point.
    double xs = 0.0;
    double ys = 0.0;
    double zs = 0.0;

    // Target point.
    double xe = 520.05;
    double ye = 266.50;
    double ze = 125.50;

    double l= sqrt(xe * xe + ye * ye + ze * ze);

    // Guess.
    double s1 = 0.00;
    double gamma10 = 0.000; // Z
    double gamma20 = 0.000; // Xy.

    double degrees_xy_start = 0.0;
    double degrees_z_start = 0.0;

    // Example for z component:
    // Initial values at spline start.
    double theta10 = degrees_z_start * (M_PI / 180.0); // To radians.
    double kappa10 = 0;
    double sharpness10 = 0;

    // Example for xy component:
    // Initial values at spline start.
    double theta20 = degrees_xy_start * (M_PI / 180.0);
    double kappa20 = 0;
    double sharpness20 = 0;

    single_clothoid_fit_gsl(
                theta10, kappa10, sharpness10, theta20, kappa20, sharpness20,
                xs, ys, zs, xe, ye, ze,
                &gamma10, &gamma20, &s1
                );

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
        fprintf(gnuplot, "set xrange [-5:500]\n");   // Set the fixed range for X
        fprintf(gnuplot, "set yrange [-5:500]\n");   // Set the fixed range for Y
        fprintf(gnuplot, "set zrange [-5:300]\n");   // Set the fixed range for Z

        // Set view angle for better 3D visualization
        fprintf(gnuplot, "set view 70, 20\n");  // You can adjust the angle here

        // Plot the data in 3D (use correct filename)
        fprintf(gnuplot, "splot 'clothoid_point.dat' using 1:2:3 with lines\n");

        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
    return 0;
}

#endif // SINGLE_CLOTHOID_FIT_GSL_H
