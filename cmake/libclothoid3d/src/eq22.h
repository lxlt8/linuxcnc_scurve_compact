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
#ifndef EQ22_H
#define EQ22_H

/* eq22, page 4. Four-clothoid.
 *
 * Calculate inbetween kappa at spline junctions.
 *
 * kappai0 and kappai4 are already known before this funtion.
 * kappai4 is returned as check value.
 *
 */
static void eq22(const double yi1, const double yi2, const double yi3, const double yi4,
          const double s1, const double s2, const double s3, const double s4,
          const double sharpnessi0, const double sharpnessi1, const double sharpnessi2, const double sharpnessi3,
          const double kappai0, double *kappai1, double *kappai2, double *kappai3, double *kappai4){

    *kappai1 =    kappai0 + sharpnessi0*s1 + 0.5*yi1*(s1*s1);
    *kappai2 = (*kappai1) + sharpnessi1*s2 + 0.5*yi2*(s2*s2);
    *kappai3 = (*kappai2) + sharpnessi2*s3 + 0.5*yi3*(s3*s3);
    *kappai4 = (*kappai3) + sharpnessi3*s4 + 0.5*yi4*(s4*s4);
}

#endif // EQ22_H
