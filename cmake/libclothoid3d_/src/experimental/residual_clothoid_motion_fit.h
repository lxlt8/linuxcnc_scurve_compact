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
#ifndef RESIDUAL_CLOTHOID_MOTION_FIT_H
#define RESIDUAL_CLOTHOID_MOTION_FIT_H

#include "velocity.h"
#include "eq14.h"

/* Residual function. Calculates fit error based on distance between
 * curpos -> tarpos.
 */

static void residual_motion_function(const double *p, double *residuals, void* cookie) {
    double* params = (double*)cookie;

    double y21 = p[0]; // Gamma value, clothoid rigid for xy.
    double s1  = p[1]; // Length of individual clothoid segment.

    double theta20 = params[0];
    double kappa20 = params[1];
    double sharpness20 = params[2];

    double x0 = params[3];
    double y0 = params[4];
    double x2 = params[5];
    double y2 = params[6];

    double z0=0,x1=0,y1=0,z1=0,z2=0,xi=0,yi=0,zi=0;
    double l1,l2;

    // First clothoid.
    eq14_gauss(s1,x0,y0,z0,0,theta20,0,kappa20,0,sharpness20,0,y21,&x1,&y1,&z1,&l1);

    // End values first clothoid.
    double sharpness21 = sharpness20 + y21 * s1;
    double kappa21 = kappa20 + sharpness20 * s1 + 0.5 * y21 * (s1 * s1);
    double theta21 = theta20 + kappa20 * s1 + (0.5*sharpness20) * (s1 * s1) + (1.0 / 6.0) * y21 * (s1 * s1 * s1);

    y21 = -fabs(y21);

    // Second clothoid, negative change of rate : -y21.
    eq14_gauss(s1,x1,y1,z1,0,theta21,0,kappa21,0,sharpness21,0,y21,&xi,&yi,&zi,&l2);

    // End values second clothoid.
    double sharpness22 = sharpness21 + y21 * s1;
    double kappa22 = kappa21 + sharpness21 * s1 + 0.5 * y21 * (s1 * s1);
    double theta22 = theta21 + kappa21 * s1 + (0.5*sharpness21) * (s1 * s1) + (1.0 / 6.0) * y21 * (s1 * s1 * s1);

    // Residual for ceres solver.
    residuals[0] = xi - x2;
    residuals[1] = yi - y2;
    residuals[2] = 0; //theta22; // Solver can't get this to zero.


}

#endif // RESIDUAL_CLOTHOID_MOTION_FIT_H
