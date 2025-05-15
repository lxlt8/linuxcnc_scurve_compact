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
#ifndef EQ15_H
#define EQ15_H

#include <stdio.h>
#include <math.h>

/* Function to calculate the second derivative of the 3D general clothoid spline
 * The second derivate P'' is the c "sharpness, rate of change"

    theta1 = θ1(s) determines angle in a local plane.
    theta2 = θ2(s) determines angle into 3D.
    kappa1 = θ1′(s) is in-plane curvature κ1(s).
    kappa2 = θ2′(s) is torsion κ2(s), controlling how much the curve leaves the original plane.
*/
static void eq15(double theta1, double theta2, double kappa1, double kappa2,
          double p_double_prime[3]) {


    p_double_prime[0] = -sin(theta1) * cos(theta2) * kappa1 - cos(theta1) * sin(theta2) * kappa2;
    p_double_prime[1] = -sin(theta1) * sin(theta2) * kappa1 + cos(theta1) * cos(theta2) * kappa2;
    p_double_prime[2] = cos(theta1) * kappa1;
}

/* Test eq15.
 */
static void eq15_test() {
    // Example usage
    double theta1 = 0.5; // Angle theta1 at curve length s (in radians)
    double theta2 = 0.3; // Angle theta2 at curve length s (in radians)
    double kappa1 = 0.2; // Curvature kappa1 at curve length s (theta1'(s))
    double kappa2 = 0.1; // Curvature kappa2 at curve length s (theta2'(s))
    double p_double_prime[3];

    // Calculate the second derivative
    calculateSecondDerivative(theta1, theta2, kappa1, kappa2, p_double_prime);

    // Print the result
    printf("P''(s) = (%f, %f, %f)\n", p_double_prime[0], p_double_prime[1], p_double_prime[2]);
}

#endif // EQ15_H
