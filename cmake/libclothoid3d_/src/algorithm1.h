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
#ifndef ALGORITHM1_H
#define ALGORITHM1_H

#include "segment.h"
#include "solver/ceres_cppapi.h"
#include <math.h>
#include <stdio.h>

/* Algorithm1, page 9.
 *
 * Find closest point Pi on the clothoid spline, given a traject point Ps.
 *
 * Ptraj = Point in space.
 * Pi = Closest point to Ps on clothoid spline.
 * s = Distance closest point on clothoid spline.
 * eps = Tolerance (e.g., 1e-7).
 * data = Clothoid data.
 * dist_Ps_Pi = Distance between Ptraj and Pi, this is the real deviation.
 */
static void algorithm1(double Ptraj[3], const struct segment seg, double eps, double Pi[3], double *s, double *dist_Ps_Pi) {

    // Initialize Ceres solver for Algorithm 1.
    // Ceres finds the closest point to the clothoid.
    init_ceres_cpp_algoritmh1(Ptraj, seg, eps, s);

    // Print the computed closest distance on the clothoid
    // printf("Closest distance on clothoid s: %f\n", *s);

    // Interpolate s on the clothoid to find Pi
    eq14_interpolate_clothoid(&seg, *s, Pi);

    // Print the coordinates of Pi
    // printf("Closest point Pi on clothoid: (%f, %f, %f)\n", Pi[0], Pi[1], Pi[2]);

    // Calculate the Euclidean distance between Ps and Pi
    double dx = Ptraj[0] - Pi[0];
    double dy = Ptraj[1] - Pi[1];
    double dz = Ptraj[2] - Pi[2];
    *dist_Ps_Pi = sqrt(dx * dx + dy * dy + dz * dz);

    // Print the distance between Ps and Pi
    // printf("Distance between Ps and Pi: %f\n", *dist_Ps_Pi);
}

#endif // ALGORITHM1_H
