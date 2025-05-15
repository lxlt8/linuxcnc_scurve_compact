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
#ifndef ALGORITHM2_H
#define ALGORITHM2_H

#include "clothoid_common.h"
#include <math.h>
#include <stdio.h>

// Example function for Distance_PointToClothoid (placeholder)
double Distance_PointToClothoid(double s, double (*Ptraj)(double), double (*P)(double)) {
    // This function should compute the squared distance between Ptraj(s) and the closest point on P(s).
    // For now, it returns a dummy value.
    return 0.0;  // Replace with actual implementation
}

// Example trajectory function (placeholder)
double Ptraj(double s) {
    // This function should return the value of the trajectory at parameter s.
    return s;  // Replace with actual implementation
}

// Example clothoid spline function (placeholder)
double P(double s) {
    // This function should return the value of the clothoid spline at parameter s.
    return s;  // Replace with actual implementation
}

/* Algorithm2, page 10.
 *
 * Function to compute the maximum distance between the trajectory and the clothoid spline
 *
 *
 */
double algorithm2(double s0, double se, double (*Ptraj)(double), double (*P)(double)) {
    double epsilon = 1e-6;  // Tolerance for convergence
    double rb = 0.4;        // Ratio for initial midpoint calculation
    double straj_l = s0;    // Start of the trajectory segment
    double straj_r = se;    // End of the trajectory segment
    double straj_m = (1 - rb) * straj_l + rb * straj_r;  // Initial midpoint
    double gl = 0;          // Distance at the start point
    double gr = 0;          // Distance at the end point
    double gm = -Distance_PointToClothoid(straj_m, Ptraj, P);  // Distance at the midpoint

    while (1) {
        // Calculate coefficients for the next point
        double alpha1 = (gl - gr) / (straj_l - straj_r);
        double alpha2 = ((gl - gm) / (straj_l - straj_m) - alpha1) / (straj_m - straj_r);

        // Calculate the new point to evaluate
        double straj_new = 0.5 * (straj_l + straj_r - alpha1 / alpha2);

        // Calculate the distance at the new point
        double g_new = -Distance_PointToClothoid(straj_new, Ptraj, P);

        // Check for convergence
        if (fabs(g_new - gm) < epsilon) {
            return sqrt(-g_new);  // Return the maximum distance
        }

        // Update the search interval
        if (straj_new < straj_m) {
            if (g_new < gm) {
                straj_r = straj_m;
                gr = gm;
                straj_m = straj_new;
                gm = g_new;
            } else {
                straj_l = straj_new;
                gl = g_new;
            }
        } else {
            if (g_new < gm) {
                straj_l = straj_m;
                gl = gm;
                straj_m = straj_new;
                gm = g_new;
            } else {
                straj_r = straj_new;
                gr = g_new;
            }
        }
    }
}




#endif // ALGORITHM2_H
