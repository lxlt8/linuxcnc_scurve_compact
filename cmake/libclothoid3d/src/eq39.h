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
#ifndef EQ39_H
#define EQ39_H

#include <stdio.h>
#include <math.h>

/*  Eq 39, page 7.

    Spline start, end posture.

    First segment, at end of segment, wich is :
        θ1.0 = theta1, inclination angle. Measures how much the vector tilts from the XY plane.
        It is zero if the vector is completely in the XY plane.
        It is positive if the vector is pointing upward (Z > 0).
        It is negative if the vector is pointing downward (Z < 0).

        θ2.0 = theta2, Azimuth angle. Is the direction of the projection of the vector onto the XY plane.
        It is the angle measured counterclockwise from the positive X-axis.

    Second segment:
        θ1.4 =
        θ2.4 =

    theta0_ = unit vector (θ "theta", first derivate) of first transformed segment.
    theta1_ = unit vector (θ "theta", first derivate) of second transformed segment.

    t10 = theta θ1.0
    t20 = theta θ2.0
    t14 = theta θ1.4
    t24 = theta θ2.4

    The eq39 states s0 & se.
    Must we trhead s0 & se as interpolated points on the original segments to get a real start, end posture for the spline?
    Using endpoints is in my opinion not a valid start posture for a clothoid spline.

*/
static void eq39(  double theta0_[3], double theta1_[3],
            double *t10, double *t20, double *t14, double *t24) {

    t10 = asin(theta0_[2]);

    if(theta0_[1]>=0){
        t20 = acos(theta0_[0] / cos(th10));
    } else {
        t20 = 2*M_PI - acos( theta0_[0] / cos(t10) );
    }

    t14 = asin(theta1_[2]);

    if(theta1_[1]>=0){
         t24 = acos( theta1_[0]/cos(t14) );
    } else {
         t24 = 2*M_PI - acos( theta1_[0] / cos(t14));
    }

    printf("Theta 1.0: %lf\n", t10);
    printf("Theta 2.0: %lf\n", t20);
    printf("Theta 1.4: %lf\n", t14);
    printf("Theta 2.4: %lf\n", t24);
}

#endif // EQ39_H
