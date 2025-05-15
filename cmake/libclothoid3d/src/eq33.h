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
#ifndef EQ33_H
#define EQ33_H

#include "eq30.h"
#include "eq31.h" // Used for arc length.
#include "eq32.h" // Calculates the normal n to be used in the transformation matrix.

// Compute the transformation matrix T
static void eq33_matrix(double n[3], double T[3][3]) {
    double nx = n[0], ny = n[1], nz = n[2];
    double w = sqrt(ny * ny + nz * nz);

    if (w == 0) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
    } else {
        T[0][0] = 0; T[0][1] = 0; T[0][2] = (nx > 0) ? 1 : -1;
        T[1][0] = 0; T[1][1] = 1; T[1][2] = 0;
        T[2][0] = (nx > 0) ? 1 : -1; T[2][1] = 0; T[2][2] = 0;
    }
}

// Function to transform points. Updates P[3].
static void fwd_trans(double P[3], double T[3][3]) {
    double result[3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i] += T[i][j] * P[j];
        }
    }
    for (int i = 0; i < 3; i++) {
        P[i] = result[i];
    }
}

// Function to apply inverse transformation. Updates P[3].
static void inv_trans(double P[3], double T[3][3]) {
    double T_inv[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            T_inv[i][j] = T[j][i]; // Transpose of T
        }
    }
    fwd_trans(P, T_inv);
}

/* Calculate the rotation matrix T3x3
 *
 * pa0  = start point first segment.
 * pae0 = end point first segment.
 * pc0  = center first segment.
 * na0  = normal first segment.
 *
 * pa1  = start point second segment.
 * pae1 = end point second segment.
 * pc1  = center second segment.
 * na1  = normal second segment.
 *
 * T    = transformation matrix.
 */
static void eq33(  double pa0[3], double pae0[3], double pc0[3], double na0[3],
            double pa1[3], double pae1[3], double pc1[3], double na1[3],
            double T[3][3]) {

    // Compute tangent θ "theta" vectors, see page 6. Fig. 6
    // We use eq32 to calculate the normal n.
    double v1[3];
    double v2[3];
    double n[3];
    double s=0;     // Used for length.

    // Get the lenght of first segment.
    arc_length(pa0,pae0,pc0,na0,&s);

    // Use end point for s. Calculate first derivate θ theta at point s.
    eq30_theta(pa0,pc0,na0,s,v1);

    // Use startpoint for s, Calculate first derivate θ theta at point s.
    s=0;
    eq30_theta(pa1,pc1,na1,s,v2); // End point s, first derivate θ theta.

    // Find n, using eq.32, if v1 x v2 fails it reverts to using arc normals.
    eq32(v1,v2,na0,na1,n);

    // Compute transformation matrix.
    eq33_matrix(n, T);
}

#endif // EQ33_H
