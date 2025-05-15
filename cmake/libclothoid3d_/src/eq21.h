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
#ifndef EQ21_H
#define EQ21_H

/* eq21, page 4. Four-clothoid.
 *
 * Calculate inbetween spline sharpness c at juntions.
 *
 * yi1 to yi4 are gamma values, used in eq44, calculated by eq 45 with
 * yi1 as the initial guess value for the dogleg solver.
 *
 * ci0 and ci4 are already known before this funtion. The ci4 is returned as check value.
 *
 */
static void eq21(const double yi1, const double yi2, const double yi3, const double yi4,
          const double s1, const double s2, const  double s3, const double s4,
          const double sharpnessi0, double *sharpnessi1, double *sharpnessi2, double *sharpnessi3, double *sharpnessi4){

    *sharpnessi1 = sharpnessi0 + yi1*s1;
    *sharpnessi2 = (*sharpnessi1) + yi2*s2;
    *sharpnessi3 = (*sharpnessi2) + yi3*s3;
    *sharpnessi4 = (*sharpnessi3) + yi4*s4;
}

#endif // EQ21_H
