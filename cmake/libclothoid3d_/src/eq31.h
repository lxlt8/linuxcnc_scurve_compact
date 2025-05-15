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
#ifndef EQ31_H
#define EQ31_H

#include "eq30.h"

// Function to calculate dot product of two vectors
static  double dot_product(double v1[3], double v2[3]) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

/*
* pa0 = arc start xyz
* pae = arc end xyz
* pc  = arc center xyz
* va1 = unit vector arc center to arc start.
* na  = unit vector arc center perpendicular to arc plane.
* va2 = cross product va1, na. A 90 degrees unit vector va1, na.
*/
static void arc_length(double pa0[3], double pae[3], double pc[3], double na[3], double *arc_length){

    double va[3];
    double vb[3];

    unit_vector(pc,pa0,va);
    unit_vector(pc,pae,vb);

    // Arc radius.
    double r = distance_3d(pa0, pc);

    // Prevent division by zero if radius is zero
    if (r == 0.0) {
        *arc_length = 0.0;
        return;
    }

    // Compute the dot product of va and vb
    double dot = dot_product(va, vb);
    // printf("dot product: %f \n", dot);

    // Clamp dot product to avoid numerical issues
    if (dot > 1.0) dot = 1.0;
    if (dot < -1.0) dot = -1.0;

    // Compute angle in radians (theta)
    double theta = acos(dot);

    // Compute arc length
    *arc_length = r * theta;
}

/* Tangent vectors in the corner of 2 segments. In 3d space. Not plane aligned yet.
 * Page 6 for illustration. The picture 6 -> top left.
 *
 * v1 = tangent vector first segment.
 * v2 = tangent vector second segment.
 *
 * for v1 the s is at endpoint.
 * for v2 the s is at startpoint.
 *
 */
static void eq31_v1(double pa0[3], double pae[3], double pc[3], double na[3], double v1[3]){

    double ptraj_prime[3];
    double sc; // Need s here as the curve endpoint for v1, called sc in document.

    // Extract s at the endpos for the arc.
    arc_length(pa0,pae,pc,na,&sc);
    printf("arc length: %f \n", sc);

    // From eq30 get the P` first derivate θ theta.
    eq30_theta(pa0,pc,na,sc,ptraj_prime);

    // Copy first derivate ptraj_prime result at position sc into v1.
    v1[0]=ptraj_prime[0];
    v1[1]=ptraj_prime[1];
    v1[2]=ptraj_prime[2];
}

static void eq31_v1_proof(){

    double pa0[3] = {0,0,0};    // Arc start point.
    double pae[3] = {100,0,0};  // Arc end point.
    double pc[3]  = {50,0,0};   // Arc center point.
    double na[3]  = {0,0,1};    // Arc normal.

    double v1[3];
    eq31_v1(pa0,pae,pc,na,v1);

    printf("eq31_v1_proof, theta at end of segment: \n");
    printf("v1 x: %lf\n", v1[0]);
    printf("v1 y: %lf\n", v1[1]);
    printf("v1 z: %lf\n", v1[2]);
    printf("\n");

    /* Terminal output:
        v1 x: 0.000000
        v1 y: -1.000000
        v1 z: 0.000000
    */
}

static void eq31_v2(double pa0[3], double pae[3], double pc[3], double na[3], double v2[3]){

    double ptraj_prime[3];
    double sc=0; // Need s here at the arc start point.

    // From eq30 get the P` first derivate θ theta.
    eq30_theta(pa0,pc,na,sc,ptraj_prime);

    // Copy first derivate ptraj_prime result at position sc into v1.
    v2[0]=ptraj_prime[0];
    v2[1]=ptraj_prime[1];
    v2[2]=ptraj_prime[2];
}

static void eq31_v2_proof(){

    double pa0[3] = {0,0,0};    // Arc start point.
    double pae[3] = {100,0,0};  // Arc end point.
    double pc[3]  = {50,0,0};   // Arc center point.
    double na[3]  = {0,0,1};    // Arc normal.

    double v2[3];
    eq31_v2(pa0,pae,pc,na,v2);

    printf("eq31_v2_proof, theta at start of segment: \n");
    printf("v2 x: %lf\n", v2[0]);
    printf("v2 y: %lf\n", v2[1]);
    printf("v2 z: %lf\n", v2[2]);
    printf("\n");

    /* Terminal output:
        v2 x: 0.000000
        v2 y: 1.000000
        v2 z: -0.000000
    */
}

#endif // EQ31_H











