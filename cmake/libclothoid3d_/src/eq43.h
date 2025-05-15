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
#ifndef EQ43_H
#define EQ43_H

#include "math.h"

/* Eq43, page 7.
 *
 * Calculate M,N
 *
 */

static void eq43_M0(double theta10, double theta20, double kappa10, double kappa20, double c10, double *M0){

    *M0 = cos(theta10)*cos(theta20)*(kappa10*kappa10)
            - sin(theta10)*sin(theta20)*kappa10*kappa20
            + sin(theta10)*cos(theta20)*c10
            - sin(theta10)*sin(theta20)*kappa10*kappa20
            + cos(theta10)*cos(theta20*(kappa20*kappa20));
}

static void eq43_N0(double theta10, double theta20, double kappa10, double kappa20, double c10, double *N0){

    *N0 = cos(theta10)*sin(theta20)*(kappa10*kappa10)
            + sin(theta10)*cos(theta20)*kappa10*kappa20
            + sin(theta10)*sin(theta20)*c10
            + sin(theta10)*cos(theta20)*kappa10*kappa20
            + cos(theta10)*sin(theta20*(kappa20*kappa20));
}

static void eq43_M4(double theta14, double theta24, double kappa14, double kappa24, double c10, double *M4){

    *M4 = cos(theta14)*cos(theta24)*(kappa14*kappa14)
            - sin(theta14)*sin(theta24)*kappa14*kappa24
            + sin(theta14)*cos(theta24)*c10
            - sin(theta14)*sin(theta24)*kappa14*kappa24
            + cos(theta14)*cos(theta24*(kappa24*kappa24));

}

static void eq43_N4(double theta14, double theta24, double kappa14, double kappa24, double c10, double *N4){

    *N4 = cos(theta14)*sin(theta24)*(kappa14*kappa14)
            + sin(theta14)*cos(theta24)*kappa14*kappa24
            + sin(theta14)*sin(theta24)*c10
            + sin(theta14)*cos(theta24)*kappa14*kappa24
            + cos(theta14)*sin(theta24*(kappa24*kappa24));
}


#endif // EQ43_H
