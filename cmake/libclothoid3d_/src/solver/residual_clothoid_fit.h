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
#ifndef RESIDUAL_CLOTHOID_FIT_H
#define RESIDUAL_CLOTHOID_FIT_H

#include "../eq44_G123.h"
#include "clothoid3d.h"

/* Residual function. Calculates fit error based on distance between
 * curpos -> tarpos.
 */

static void residual_function(const double *p, double *xyz_vector, void* cookie) {
    double* params = (double*)cookie;

    double y11 = p[0]; // Gamma value, clothoid rigid for z.
    double y21 = p[1]; // Gamma value, clothoid rigid for xy.
    double s1  = p[2]; // Length of individual clothoid segment.

    double s2, s3, s4;
    s4 = s3 = s2 = s1;

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

    double theta11, theta12, theta13;
    double theta21, theta22, theta23;

    double kappa11, kappa12, kappa13;
    double kappa21, kappa22, kappa23;

    double sharpness11, sharpness12, sharpness13;
    double sharpness21, sharpness22, sharpness23;

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

    double x1, y1, z1;      // Endpoint clothoid 1.
    double x2, y2, z2;      // Endpoint clothoid 2.
    double x3, y3, z3;      // Endpoint clothoid 3.
    double x4, y4, z4;      // Endpoint clothoid 4.
    double l1, l2, l3, l4;  // Individual length's.

    // eq14, Calculate clothoid compound endpoint using the initial guess values for y11, y21, s1.
    eq14_gauss(s1, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, y11, y21, &x1, &y1, &z1, &l1);
    eq14_gauss(s2, x1, y1, z1, theta11, theta21, kappa11, kappa21, sharpness11, sharpness21, y12, y22, &x2, &y2, &z2, &l2);
    eq14_gauss(s3, x2, y2, z2, theta12, theta22, kappa12, kappa22, sharpness12, sharpness22, y13, y23, &x3, &y3, &z3, &l3);
    eq14_gauss(s4, x3, y3, z3, theta13, theta23, kappa13, kappa23, sharpness13, sharpness23, y14, y24, &x4, &y4, &z4, &l4);

    xyz_vector[0] = xe - x4;
    xyz_vector[1] = ye - y4;
    xyz_vector[2] = ze - z4;
}

#endif // RESIDUAL_CLOTHOID_FIT_H
