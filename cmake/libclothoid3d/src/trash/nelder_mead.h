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
#ifndef NELDER_MEAD_H
#define NELDER_MEAD_H

#include "residual_clothoid_fit.h"
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multimin.h>

/* Objective function for GSL. Cast's void into gsl_vector.
 * This is the iteration function that repeats until treshold
 * is within limits, ie result < 1e-6
 *
 */
static double nelder_mead_objective_funtion(const gsl_vector *v, void *cookie) {

    double p[3];
    p[0] = gsl_vector_get(v, 0); // y11
    p[1] = gsl_vector_get(v, 1); // y21
    p[2] = gsl_vector_get(v, 2); // s1

    // The residu is a xyz vector.
    // x = xyz[0]
    // y = xyz[1]
    // z = xyz[2]
    double xyz_vector[3];
    residual_function(p, xyz_vector, cookie);

    // Compute the Euclidean norm of the residuals
    double residual_norm = 0.0;
    for (int i = 0; i < 3; i++) {
        residual_norm += xyz_vector[i] * xyz_vector[i];
    }
    return sqrt(residual_norm);
}

/* Solver method : Nelder-Mead simplex method
 * Setup gsl solver and run the objective function.
 */
static void init_nelder_mead(const double theta10, const double theta14,
                     const double kappa10, const double kappa14,
                     const double sharpness10, const double sharpness14,
                     const double theta20, const double theta24,
                     const double kappa20, const double kappa24,
                     const double sharpness20, const double sharpness24,
                     const double xs, const double ys, const double zs,
                     const double xe, const double ye, const double ze,
                     double *y11, double *y21, double *s1) {

    // Initial guess, the 3 unknown's. y11, y21, s1.
    gsl_vector *x = gsl_vector_alloc(3);
    gsl_vector_set(x, 0, *y11);
    gsl_vector_set(x, 1, *y21);
    gsl_vector_set(x, 2, *s1);

    // User data (cookie)
    double user_data[18] = {
        theta10, theta14, kappa10, kappa14, sharpness10, sharpness14,
        theta20, theta24, kappa20, kappa24, sharpness20, sharpness24,
        xs, ys, zs,
        xe, ye, ze
    };

    // Set up the Nelder-Mead solver
    const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex2;
    gsl_multimin_fminimizer *s = gsl_multimin_fminimizer_alloc(T, 3);

    gsl_multimin_function F;
    F.n = 3;
    F.f = nelder_mead_objective_funtion;
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
    *y11 = gsl_vector_get(s->x, 0);
    *y21 = gsl_vector_get(s->x, 1);
    *s1 = gsl_vector_get(s->x, 2);

    printf("Solution found: gamma10 = %f, gamma20 = %f, s1 = %f\n", *y11, *y21, *s1);

    // Free memory
    gsl_vector_free(x);
    gsl_vector_free(step_sizes);
    gsl_multimin_fminimizer_free(s);
}

#endif // NELDER_MEAD_H
