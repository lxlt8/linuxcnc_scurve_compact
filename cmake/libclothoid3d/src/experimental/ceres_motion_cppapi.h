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
#ifndef CERES_MOTION_CPPAPI_H
#define CERES_MOTION_CPPAPI_H

#include "ceres_motion_intf.h"

#ifdef __cplusplus
extern "C" {
#endif

ceres_motion_intf* init_cpp_motion_ptr();
void ceres_cpp_motion_init(ceres_motion_intf *ptr, void* xyz_vector, const void* user_data);
void ceres_cpp_motion_solve(ceres_motion_intf *ptr, double *kappa, double *y21, double *s1, int solver_type, double s1_lower_limit, double s1_upper_limit);
void ceres_cpp_motion_result(ceres_motion_intf *ptr, double *result);
void ceres_cpp_motion_free_ptr(ceres_motion_intf *ptr);

#ifdef __cplusplus
}
#endif

/* Solver method : Ceres.
 */
static void init_ceres_motion_cpp(const double theta20,
                                  const double kappa20,
                                  const double sharpness20,
                                  const double xs, const double ys,
                                  const double xe, const double ye,
                                  double *kappa,
                                  double *y21,
                                  double *s1,
                                  int solver_type,
                                  double s1_lower_limit, double s1_upper_limit) {

    // User data (cookie)
    double user_data[7] = {
        theta20, kappa20, sharpness20,
        xs, ys, xe, ye
    };

    double parameters[3]={ *kappa, *y21, *s1 };

    ceres_motion_intf *ptr;
    ptr = init_cpp_motion_ptr();
    ceres_cpp_motion_init(ptr, parameters, user_data);
    ceres_cpp_motion_solve(ptr,kappa, y21,s1,solver_type,s1_lower_limit,s1_upper_limit);

    double result[3];
    ceres_cpp_motion_result(ptr, result);
    ceres_cpp_motion_free_ptr(ptr);
}

#endif // CERES_MOTION_CPPAPI_H
