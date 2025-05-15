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
#ifndef EQ45_H
#define EQ45_H

#include <stdio.h>
#include <math.h>
#include <segment.h>

#define toDegrees (180.0 / M_PI)

/* Condition for extra torsion turn.
 */
static int eq45_extra_torsion_turns(struct segment *seg){
    if(seg->theta10!=0 && seg->theta20==0 && seg->theta14==0 && seg->theta24!=0){
        return 1;
    }
    return 0;
}

static void theta_to_xy(double theta, double *x, double *y) {
    *x = cos(theta);
    *y = sin(theta);
}

/* This angle is always positive.
 */
static double angle_between_vectors(double ax, double ay, double bx, double by) {
    double dot_product = ax * bx + ay * by;
    double magnitude_a = sqrt(ax * ax + ay * ay);
    double magnitude_b = sqrt(bx * bx + by * by);

    if (magnitude_a == 0 || magnitude_b == 0) {
        printf("Error: One of the vectors has zero magnitude.\n");
        return 0.0; // Avoid division by zero
    }

    double cos_theta = dot_product / (magnitude_a * magnitude_b);

    // Clamp value to [-1,1] to avoid numerical errors in acos
    if (cos_theta > 1.0) cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;

    return acos(cos_theta); // Returns angle in radians
}

/* This angle is negative and positive. This we need for eq45.
 */
static double signed_angle_between_vectors(double ax, double ay, double bx, double by) {
    double dot_product = ax * bx + ay * by;
    double magnitude_a = sqrt(ax * ax + ay * ay);
    double magnitude_b = sqrt(bx * bx + by * by);

    if (magnitude_a == 0 || magnitude_b == 0) {
        printf("Error: One of the vectors has zero magnitude.\n");
        return 0.0; // Avoid division by zero
    }

    double cos_theta = dot_product / (magnitude_a * magnitude_b);

    // Clamp value to [-1,1] to avoid numerical errors in acos
    if (cos_theta > 1.0) cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;

    double angle = acos(cos_theta); // Angle in radians

    // Compute cross product to determine sign
    double cross_product = ax * by - ay * bx;
    if (cross_product < 0) {
        angle = -angle; // Make angle negative for clockwise rotation
    }

    return angle;
}

/* eq45, page 7. Four-clothoid.
 *
 * This function calculates the constrains for a 4 clothoid compound.
 *
 * For z  axis constrains : y11, y12, y13, y14.
 * For xy axis constrains : y21, y22, y23, y24.
 *
 * Unknown variables are : y11, y21, s1.
 * For i = 1 , 2.
 *
 * The value y11 changes the endpos off the z of the clothoid compound.
 * The value y21 changes the endpos off the xy of the clothoid compound.
 *
 * y11, y21 can be seen as rigid factors for the clothoid. How smaller the factor,
 * how straigher, more rigid the clothoid's appear.
 *
 * S1 is the indiviual clothoid length.
 * S1 is not the length of the compount but clothoid
 * compound lenght / 4.
 *
 * As an example a solver like dogleg will try values for y11, y21, s1
 * to iterate the clothoid compound to it's target position.
 * At each iteration the error value (distance curpos to tarpos) is getting
 * smaller, until the treshold: 1e-6 for example is fullfilled.
 *
 */
static void eq45(const double thetai0, const double thetai4,
                 const double kappai0, const double kappai4,
                 const double sharpnessi0, const double sharpnessi4,
                 const double s1,
                 double yi1, double *yi2, double *yi3, double *yi4) {

    if (s1 == 0) {
        printf("Error: s1 cannot be zero.\n");
        return;
    }

    // Convert thetai0 back to vector.
    // We do this because theta's are in the range [0,-M_PI], [0,M_PI],
    // wich makes angle faults in 3rth quadrant when simply doing : thetai4 - thetai0.
    // This could be the reason why in the abstact the forward & inverse transformation
    // is done to avoid this problem. However the solution seems simple.
    double x0, y0;
    theta_to_xy(thetai0, &x0, &y0);

    // Convert thetai4 back to vector.
    double x4, y4;
    theta_to_xy(thetai4, &x4, &y4);

    // Calculate angle between 2 vectors. Get signed angle wich is negative or positive.
    // Input "dt" is influenced by negative or positive input.
    double signed_angle =  signed_angle_between_vectors(x0,y0,x4,y4);

    double dt = signed_angle;              // thetai4 - thetai0;
    double dk = kappai4 - kappai0;         // Delta kappa.
    double dc = sharpnessi4 - sharpnessi0; // Delta sharpness.

    // Calculate yi2
    *yi2 = ((3 * dt - 3 * (dk + 4 * kappai0) * s1 + (dc - 12 * sharpnessi0) * (s1 * s1)) / (3 * (s1 * s1 * s1))) - 3 * yi1;

    // Calculate yi3
    *yi3 = ((2 * dk - (dc + 8 * sharpnessi0) * s1) / (2 * (s1 * s1))) - (3*yi1) - (2*(*yi2));

    // Calculate yi4
    *yi4 = (dc / s1) - yi1 - (*yi2) - (*yi3);
}

#endif // EQ45_H




























