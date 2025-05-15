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
#ifndef EQ14_H
#define EQ14_H

#include <stdio.h>
#include <math.h>
#include "eq17.h"
#include "eq18.h"
#include "eq19.h"
#include "eq20.h"
#include "segment.h"
#include "gauss_legendre.h"

/* eq14, page 4.
 *
 * Function to compute clothoid curve. (ARCHIVED)
 *
 * This way off computing the clothoid is not very accurate,
 * as it has the property to drift off.
 *
 * This is not really a problem as the ceres fit algo just fits the
 * clothoid to desired position.
 *
 * This function still shows some acceleration spikes at the clothoid junctions
 * in a 1ms update cycle.
 *
 * Incrementing values, including nummerical errors have to property to drift off
 * end position.
 *
 * This function is left here to show the problem.
 *
 * ltot = clothoid length up to s. And is left here for info.
 */
static void eq14_drift(double s, double x0, double y0, double z0,
                       double theta10, double theta20,
                       double kappa10, double kappa20,
                       double sharpness10, double sharpness20,
                       double gamma1, double gamma2,
                       double *xi, double *yi, double *zi, double *ltot) {

    int N = 500; // Number of integration steps
    *xi = x0, *yi = y0, *zi = z0;
    double ds = s / N;

    *ltot=0;

    for (int i = 0; i <= N; i++) {
        double s = i * ds;

        // Compute theta1(s) and theta2(s)
        double theta1 = theta10 + kappa10 * s + (sharpness10 / 2.0) * s * s + (gamma1 / 6.0) * s * s * s;
        double theta2 = theta20 + kappa20 * s + (sharpness20 / 2.0) * s * s + (gamma2 / 6.0) * s * s * s;

        // Compute new coordinates using small-step integration
        double dx = cos(theta1) * cos(theta2) * ds;
        double dy = cos(theta1) * sin(theta2) * ds;
        double dz = sin(theta1) * ds;

        *xi += dx;
        *yi += dy;
        *zi += dz;

        *ltot+=sqrt(dx*dx + dy*dy + dz*dz);

    }

    // printf("ltot eq14_drift: %f \n",*ltot);
}

/* eq14, page 4
 *
 * Most efficient and precise solution to compute the clothoid curve lenght.
 * Using gauss legendre, up to 20 points. Use a 30 point legendre to get even
 * more accuracy.
 *
 * This calculation is more accurate then the above eq14_drift function.
 *
 * s = lenght on clothoid to retrieve the xyz point, xi,yi,zi.
 * x0 y0 z0 = Clothoid start point.
 * ltot = Clothoid length up to s.
 *
 * gamma1, gamma2, clothoid's rigid body factor. Synonims for y11, y22.
 *
 */
static void eq14_gauss(double s, double x0, double y0, double z0,
                       double theta10, double theta20,
                       double kappa10, double kappa20,
                       double sharpness10, double sharpness20,
                       double gamma1, double gamma2,
                       double *xi, double *yi, double *zi, double *ltot) {

    // Initialize output variables
    *xi = x0;
    *yi = y0;
    *zi = z0;
    *ltot = 0;

    // Use 64-point Gauss-Legendre quadrature
    for (int i = 0; i <16; i++) {
        // Scale node to interval [0, s]
        double si = 0.5 * s * (gauss_nodes_16[i] + 1);

        // Scale weight to interval [0, s]
        double weight = 0.5 * s * gauss_weights_16[i];

        // Compute theta1(si) and theta2(si)
        double theta1 = theta10 + kappa10 * si + (sharpness10 / 2.0) * si * si + (gamma1 / 6.0) * si * si * si;
        double theta2 = theta20 + kappa20 * si + (sharpness20 / 2.0) * si * si + (gamma2 / 6.0) * si * si * si;

        // Compute new coordinates using Gauss quadrature integration
        double dx = cos(theta1) * cos(theta2) * weight;
        double dy = cos(theta1) * sin(theta2) * weight;
        double dz = sin(theta1) * weight;

        // Update coordinates
        *xi += dx;
        *yi += dy;
        *zi += dz;

        // printf("xi: %f \n",*xi);
        // printf("yi: %f \n",*yi);
        // printf("zi: %f \n",*zi);

        // Update total length (integral of the curve)
        *ltot += sqrt(dx * dx + dy * dy + dz * dz);

        // printf("ltot: %f \n",*ltot);
    }
}

/* Returns the point pi for x,y,z.
*
*/
static void eq14_interpolate_clothoid(const struct segment *data, double s, double pi[3]) {

    // Calculate segment boundaries
    double s1 = data->s1;
    double s2 = s1 + data->s1;
    double s3 = s2 + data->s1;
    double s4 = s3 + data->s1;

    double l1 = 0, l2 = 0, l3 = 0, l4 = 0;

    // Initialize output point
    pi[0] = 0;
    pi[1] = 0;
    pi[2] = 0;

    s = fmin(s4, s); // Upper limit.
    s = fmax(0, s);  // Lower limit.

    if (s <= s1) {
        eq14_gauss(s, data->p0[0], data->p0[1], data->p0[2],
                data->theta10, data->theta20, data->kappa10, data->kappa20, data->sharpness10, data->sharpness20,
                data->y11, data->y21,
                &pi[0], &pi[1], &pi[2], &l1);

    } else if (s > s1 && s <= s2) {
        eq14_gauss(s - s1, data->x1, data->y1, data->z1,
                   data->theta11, data->theta21, data->kappa11, data->kappa21, data->sharpness11, data->sharpness21,
                   data->y12, data->y22,
                   &pi[0], &pi[1], &pi[2], &l2);

    } else if (s > s2 && s <= s3) {
        eq14_gauss(s - s2, data->x2, data->y2, data->z2,
                   data->theta12, data->theta22, data->kappa12, data->kappa22, data->sharpness12, data->sharpness22,
                   data->y13, data->y23,
                   &pi[0], &pi[1], &pi[2], &l3);

    } else if (s > s3 && s <= s4) {
        eq14_gauss(s - s3, data->x3, data->y3, data->z3,
                   data->theta13, data->theta23, data->kappa13, data->kappa23, data->sharpness13, data->sharpness23,
                   data->y14, data->y24,
                   &pi[0], &pi[1], &pi[2], &l4);

    } else {
        // Handle the case where s is out of bounds (optional)
        printf("error: interpolate. \n");
    }
}

/* Returns the point pi for x,y,z.
*
*/
static void eq14_interpolate_single_2d_clothoid(double s,
                                                double y21,
                                                double p0[3],
                                                double theta20,
                                                double kappa20,
                                                double sharpness20,
                                                double pi[3]) {

    double l1 = 0;

    // Initialize output point
    pi[0] = 0;
    pi[1] = 0;
    pi[2] = 0;

    eq14_gauss(s, p0[0], p0[1], p0[2],
            0, theta20, 0, kappa20, 0, sharpness20,
            0, y21,
            &pi[0], &pi[1], &pi[2], &l1);
}

/* Total clothoid compound lenght.
*/
static double clothoid_compound_length(const struct segment *data){
    return data->s1*4;
}

#endif // EQ14_H























