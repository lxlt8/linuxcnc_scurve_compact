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
#ifndef EQ40_H
#define EQ40_H

#include <stdio.h>
#include <math.h>

/*  Eq40, page 7.

    kappa0_ = Second derivate K, kappa, wich is curvature. Transformed "_", uses xyz. First segment.
    kappa1_ = Second segment.

    theta10,20,14,24 = Result eq39.

    k10 = kappa, first segment, curvature in z
    k20 = kappa, first segment, curvature in xy

    k14 = kappa, second segment, curvature in z
    k24 = kappa, second segment, curvature in xy
*/

static void eq40(  double kappa0_[3], double kappa1_[3],
            const double theta10, const double theta20, const double theta14, const double theta24,
            double *k10, double *k20, double *k14, double *k24){

    k10 = kappa0_[2] / cos( theta10 );

    if(sin(theta20)!=0){
        k20 = ( kappa0_[0] + sin(theta10)*cos(theta20)*k10 ) / ( cos(theta10)*sin(theta20) );
    } else {
        k20 = ( kappa0_[0] + sin(theta10)*sin(theta20)*k10 ) / ( cos(theta10)*cos(theta20) );
    }

    k14 = kappa1_[2] / cos(theta14);

    if(sin(theta24)!=0){
        k24 = ( kappa1_[0] + sin(theta14)*cos(theta24)*k14 ) / ( cos(theta14)*sin(theta24) );
    } else {
        k24 = ( kappa1_[0] + sin(theta14)*sin(theta24)*k14 ) / ( cos(theta14)*cos(theta24) );
    }

    printf("Kappa 1.0: %lf\n", k10);
    printf("Kappa 2.0: %lf\n", k20);
    printf("Kappa 1.4: %lf\n", k14);
    printf("Kappa 2.4: %lf\n", k24);

}

#endif // EQ40_H
