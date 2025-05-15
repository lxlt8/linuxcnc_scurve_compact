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
#ifndef EQ42_H
#define EQ42_H

#include "eq43.h" // Needed for M0, N0, M4, N4.

/* Eq42, page 7.
 *
 * Calculate sharpness c, for the clothoid compound.
 *
 * c10 = sharpness for z at clothoid start point.
 * c14 = sharpness for z at clothoid end point.
 *
 * c20 = sharpness for xy at clothoid start point.
 * c24 = sharpness for xy at clothoid end point.
 *
 * Looks like eq41 is not needed as we have already eq30_sharpness.
 * or we can use eq30 in a modified version for it.
 *
 * C1.0 = c, sharpness.
 *
 */

static void eq42(  double sharpness0_[3], double sharpness1_[3], // Third derivate sharpness c at start, end of spline.
            const double theta10, const double theta20, const double theta14, const double theta24, // Retrieved from Eq.39
            const double kappa10, const double kappa20, const double kappa14, const double kappa24, // Retrieved from Eq.40
            double *c10, double *c20, double *c14, double *c24
){
    double M0,N0,M4,N4;

    c10 = ( sharpness0_[2] + sin( theta10*(kappa10*kappa10) )) / cos(theta10);

    if(sin(theta20)!=0){
        eq43_M0(theta10, theta20, kappa10, kappa20, c10, M0);
        c20 = - ( sharpness0_[0] + M0 ) / cos(theta10)*sin(theta20);
    } else {
        eq43_N0(theta10, theta20, kappa10, kappa20, c10, N0);
        c20 = ( sharpness0_[1] + N0 ) / cos(theta10)*cos(theta20);
    }

    c14 = (sharpness1_[2] + sin(theta14) * (kappa14*kappa14)) / cos(theta14);

    if(sin(theta24)!=0){
        eq43_M4(theta14, theta24, kappa14, kappa24, c10, M4);
        c24 = -(( sharpness1_[0] + M4 ) / ( cos(theta14)*sin(theta24) ));
    } else {
        eq43_N4(theta14, theta24, kappa14, kappa24, c10, N4);
        c24 = ( sharpness1_[1] + N4 ) / ( cos(theta14)*cos(theta24) );
    }

    printf("Sharpness 1.0: %lf\n", c10);
    printf("Sharpness 2.0: %lf\n", c20);
    printf("Sharpness 1.4: %lf\n", c14);
    printf("Sharpness 2.4: %lf\n", c24);
}

#endif // EQ42_H
