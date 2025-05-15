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
#ifndef CERES_CPPAPI_H
#define CERES_CPPAPI_H

#include "ceres_intf.h"

#ifdef __cplusplus
extern "C" {
#endif

ceres_intf* init_cpp_ptr();
void ceres_cpp_init(ceres_intf *ptr, void* xyz_vector, const void* user_data);
void ceres_cpp_solve(ceres_intf *ptr, double *y11, double *y21, double *s1, int solver_type, double s1_lower_limit, double s1_upper_limit);
void ceres_cpp_result(ceres_intf *ptr, double *result);
void ceres_cpp_solve_algorithm1(ceres_intf *ptr, double *Ptraj, const struct segment data, double eps, double *s);
void ceres_cpp_fit(ceres_intf *ptr,
                   struct segment *seg0,
                   struct segment *seg1,
                   struct segment *seg2,
                   double *deviation,
                   double *s);
void ceres_cpp_free_ptr(ceres_intf *ptr);

#ifdef __cplusplus
}
#endif

/* Solver method : Ceres.
 */
static void init_ceres_cpp(const double theta10, const double theta14,
                           const double kappa10, const double kappa14,
                           const double sharpness10, const double sharpness14,
                           const double theta20, const double theta24,
                           const double kappa20, const double kappa24,
                           const double sharpness20, const double sharpness24,
                           const double xs, const double ys, const double zs,
                           const double xe, const double ye, const double ze,
                           double *y11, double *y21, double *s1, int solver_type,
                           double s1_lower_limit, double s1_upper_limit) {

    // User data (cookie)
    double user_data[18] = {
        theta10, theta14, kappa10, kappa14, sharpness10, sharpness14,
        theta20, theta24, kappa20, kappa24, sharpness20, sharpness24,
        xs, ys, zs,
        xe, ye, ze
    };

    double parameters[3]={ *y11, *y21, *s1 };

    ceres_intf *ptr;
    ptr = init_cpp_ptr();
    ceres_cpp_init(ptr, parameters, user_data);
    ceres_cpp_solve(ptr, y11,y21,s1,solver_type,s1_lower_limit,s1_upper_limit);

    double result[3];
    ceres_cpp_result(ptr, result);
    ceres_cpp_free_ptr(ptr);
}

static void init_ceres_cpp_algoritmh1(double *Ptraj, struct segment data, double eps, double *s){

    ceres_intf *ptr;
    ptr = init_cpp_ptr();
    ceres_cpp_solve_algorithm1(ptr, Ptraj, data, eps, s);
    ceres_cpp_free_ptr(ptr);
}

static void init_ceres_cpp_fit(struct segment *seg0,
                               struct segment *seg1,
                               struct segment *seg2,
                               double *deviation,
                               double *s) {

    ceres_intf *ptr;
    ptr = init_cpp_ptr();
    ceres_cpp_fit(ptr,seg0,seg1,seg2,deviation,s);
    ceres_cpp_free_ptr(ptr);
}

#endif // CERES_CPPAPI_H
