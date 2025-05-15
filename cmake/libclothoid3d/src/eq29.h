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
#ifndef EQ29_H
#define EQ29_H

#include "eq30.h"

/* Note : no helix.
 * Interpolate arc traj point "ptraj" given s "circumfence lenght from arc start to ptraj".
 * To simplify the input, we give 3 arc points and the distance s to interpolate the point ptraj.
 *
 * ptraj = interpolated point on arc, given lenght from arc startpoin.
 * s     = circumfence arc lenght from arc start to point ptraj.
 * pa0   = arc start xyz.
 * pae   = arc end xyz.
 * pc    = arc center xyz.
 * na    = arc normal.
 */
static void eq29(double pa0[3], double pc[3], double na[3], double s, double ptraj[3]) {

    double va1[3]; // Unit vector va1.
    double va2[3]; // Unit vector va2.

    va1_va2(pa0, pc, na, va1, va2); // Calculate the unit vectors for the arc. In total 3 unit vectors.

    // Arc radius.
    double r = distance_3d(pa0, pc);

    // Step 1: Calculate cos(s/r) and sin(s/r)
    double cos_term = cos(s / r);
    double sin_term = sin(s / r);

    // Step 2: Compute the resulting point
    ptraj[0] = pc[0] + r * cos_term * va1[0] + r * sin_term * va2[0];  // x-component
    ptraj[1] = pc[1] + r * cos_term * va1[1] + r * sin_term * va2[1];  // y-component
    ptraj[2] = pc[2] + r * cos_term * va1[2] + r * sin_term * va2[2];  // z-component
}

// Function to calculate the circumference of a circle
static double circle_length(double r) {
    return 2 * M_PI * r;  // M_PI is defined in math.h
}

// Function to proof eq29. Page 5.
static void eq29_proof(){

    double pa0[3] = {0,0,0};    // Arc start point.
    // double pae[3] = {100,0,0};  // Arc end point.
    double pc[3]  = {50,0,0};   // Arc center point.
    double na[3]  = {0,0,1};    // Arc normal.

    double ptraj[3];            // Interpolated point on arc circumfence, given the arc lenght s.

    double r = distance_3d(pa0,pc);
    double ltot = circle_length(r);

    double s = ltot/4; // Take 50% of circlelenght as input.

    eq29(pa0,pc,na,s,ptraj);

    printf("eq29 proof, interpolate point on arc Ptraj(s): \n");
    printf("full circle lenght: %f \n", ltot);
    printf("s: %f \n", s);
    printf("ptraj x: %lf\n", ptraj[0]);
    printf("ptraj y: %lf\n", ptraj[1]);
    printf("ptraj z: %lf\n", ptraj[2]);
    printf("\n");

    /* Terminal output :
        eq29 proof:
        circle lenght: 314.159265
        s: 78.539816
        ptraj x: 50.000000
        ptraj y: 50.000000
        ptraj z: 0.000000
    */
}

#endif // EQ29_H
