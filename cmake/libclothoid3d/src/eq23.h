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
#ifndef EQ23_H
#define EQ23_H

/* eq23, page 4. Four-clothoid.
 *
 * Calculate inbetween theta's of the spline at the junctions.
 *
 * yi1 to yi4 are gamma values, used in eq44, calculated by eq 45 with
 * yi1 as the initial guess value for the dogleg solver.
 *
 * thetai0 and thetai4 are already known before this funtion. The thetai4 is returned as check value.
 *
 */
static void eq23(const double yi1, const double yi2, const double yi3, const double yi4,
          const double s1, const double s2, const  double s3, const double s4,
          const double sharpnessi0, const double sharpnessi1, const double sharpnessi2, const double sharpnessi3,
          const double kappai0, const double kappai1, const double kappai2, const double kappai3,
          const  double thetai0, double *thetai1, double *thetai2, double *thetai3, double *thetai4){

    *thetai1 =    thetai0 + kappai0*s1 + 0.5*sharpnessi0*(s1*s1) + (1.0/6.0)*yi1*(s1*s1*s1);
    *thetai2 = (*thetai1) + kappai1*s2 + 0.5*sharpnessi1*(s2*s2) + (1.0/6.0)*yi2*(s2*s2*s2);
    *thetai3 = (*thetai2) + kappai2*s3 + 0.5*sharpnessi2*(s3*s3) + (1.0/6.0)*yi3*(s3*s3*s3);
    *thetai4 = (*thetai3) + kappai3*s4 + 0.5*sharpnessi3*(s4*s4) + (1.0/6.0)*yi4*(s4*s4*s4);
}

#endif // EQ23_H
