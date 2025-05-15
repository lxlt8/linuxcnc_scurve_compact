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
#ifndef CERES_CAPI_H
#define CERES_CAPI_H

#include "residual_function_jacobian.h"
#include "ceres/c_api.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Solver method : CERES_LEVENBERG_MARQUARDT
 * Setup Ceres solver and run the objective function.
 */
static void init_ceres_capi(const double theta10, const double theta14,
                     const double kappa10, const double kappa14,
                     const double sharpness10, const double sharpness14,
                     const double theta20, const double theta24,
                     const double kappa20, const double kappa24,
                     const double sharpness20, const double sharpness24,
                     const double xs, const double ys, const double zs,
                     const double xe, const double ye, const double ze,
                     double *y11, double *y21, double *s1) {

    // User data (cookie)
    double user_data[18] = {
        theta10, theta14, kappa10, kappa14, sharpness10, sharpness14,
        theta20, theta24, kappa20, kappa24, sharpness20, sharpness24,
        xs, ys, zs,
        xe, ye, ze
    };

    double *parameter_pointers[] = { y11, y21, s1 };
    int parameter_sizes[] = { 1, 1, 1 };
    int num_observations = 1000;

    ceres_problem_t* problem;

    /* Create the Ceres problem */
    problem = ceres_create_problem();
    if (problem == NULL) {
        fprintf(stderr, "Failed to create Ceres problem.\n");
        return;
    }

    /* Add all the residuals. */
    for (int i = 0; i < num_observations; ++i) {
        ceres_problem_add_residual_block(
                    problem,
                    residual_function_jacobian,  // Cost function
                    &user_data,            // Points to the user data
                    NULL,                  // No loss function
                    NULL,                  // No loss function user data
                    3,                     // Number of residuals
                    3,                     // Number of parameter blocks
                    parameter_sizes,
                    parameter_pointers);
    }

    /* Solve the problem */
    ceres_solve(problem);

    /* Free the problem */
    ceres_free_problem(problem);
    problem = NULL;  // Prevent double-free
}

#endif // CERES_CAPI_H
