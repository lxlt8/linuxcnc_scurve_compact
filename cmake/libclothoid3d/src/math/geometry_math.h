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
#ifndef GEOMETRY_MATH_H
#define GEOMETRY_MATH_H

#include "vector_math.h"

#define toDegrees (180.0 / M_PI)

// Function to calculate the distance between two points in 3D space
static double distance_3d(const double p0[3], const double p1[3]) {
    double dx = p1[0] - p0[0];
    double dy = p1[1] - p0[1];
    double dz = p1[2] - p0[2];

    // Calculate and return the Euclidean distance
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Function to find the closest point on the line defined by line_start and line_end given a 3D point point
static void get_closest_point_to_line(const double point[3], const double line_start[3], const double line_end[3], double intersection[3]) {
    double U;
    double lineMag = distance_3d(line_start, line_end);

    U = ( ((point[0] - line_start[0]) * (line_end[0] - line_start[0])) +
          ((point[1] - line_start[1]) * (line_end[1] - line_start[1])) +
          ((point[2] - line_start[2]) * (line_end[2] - line_start[2]))) /
         (lineMag * lineMag);

    // Calculate the closest point (intersection)
    intersection[0] = line_start[0] + U * (line_end[0] - line_start[0]);
    intersection[1] = line_start[1] + U * (line_end[1] - line_start[1]);
    intersection[2] = line_start[2] + U * (line_end[2] - line_start[2]);
}

// Function to rotate a point around a line defined by two points (theLineP1 and theLineP2) by an angle theta.
static void rotate_point_around_line(const double thePointToRotate[3], const double theta, const double theLineP1[3], const double theLineP2[3], double result[3]) {
    double r[3];           // Direction vector of the line
    double costheta, sintheta;
    double q[3] = {0.0, 0.0, 0.0};  // Rotated point coordinates

    // Compute the direction vector r from P1 to P2
    r[0] = theLineP2[0] - theLineP1[0];
    r[1] = theLineP2[1] - theLineP1[1];
    r[2] = theLineP2[2] - theLineP1[2];

    // Normalize the direction vector
    normalize(r);

    // Translate the point to rotate to the origin relative to P1
    double translatedPoint[3] = {
        thePointToRotate[0] - theLineP1[0],
        thePointToRotate[1] - theLineP1[1],
        thePointToRotate[2] - theLineP1[2]
    };

    // Calculate the cosine and sine of the angle
    costheta = cos(theta);
    sintheta = sin(theta);

    // Apply the rotation matrix to the translated point
    q[0] += (costheta + (1 - costheta) * r[0] * r[0]) * translatedPoint[0];
    q[0] += ((1 - costheta) * r[0] * r[1] - r[2] * sintheta) * translatedPoint[1];
    q[0] += ((1 - costheta) * r[0] * r[2] + r[1] * sintheta) * translatedPoint[2];

    q[1] += ((1 - costheta) * r[0] * r[1] + r[2] * sintheta) * translatedPoint[0];
    q[1] += (costheta + (1 - costheta) * r[1] * r[1]) * translatedPoint[1];
    q[1] += ((1 - costheta) * r[1] * r[2] - r[0] * sintheta) * translatedPoint[2];

    q[2] += ((1 - costheta) * r[0] * r[2] - r[1] * sintheta) * translatedPoint[0];
    q[2] += ((1 - costheta) * r[1] * r[2] + r[0] * sintheta) * translatedPoint[1];
    q[2] += (costheta + (1 - costheta) * r[2] * r[2]) * translatedPoint[2];

    // Translate the point back to its original position
    result[0] = q[0] + theLineP1[0];
    result[1] = q[1] + theLineP1[1];
    result[2] = q[2] + theLineP1[2];
}

/* Project point on plane.
 *
 * p1 = point in space.
 * pc = point on plane.
 * pn = plane normal axis.
 * pi = projected point on plane.
 *
 */
static void project_point_on_plane(const double p1[3], const double pc[3], const double pn[3], double pi[3]) {

    // Extend p1 to be a line in the direction of the plane normal pn. Line p1-p11.
    double p11[3];
    add_vectors(p1,pn,p11);
    // Find closest point from pc to the line p1,p11. This is the plane intersection.
    get_closest_point_to_line(pc,p1,p11,pi);
}

/* Check intersection line p1,p2 to line p3,pe.
 */
static int intersect(double p1[3], double p2[3], double p3[3], double p4[3])
{
    double d = (p4[1] - p3[1]) * (p2[0] - p1[0]) -
               (p4[0] - p3[0]) * (p2[1] - p1[1]);

    if (d == 0.0)
        return 0;

    double n_a = (p4[0] - p3[0]) * (p1[1] - p3[1]) -
                 (p4[1] - p3[1]) * (p1[0] - p3[0]);

    double n_b = (p2[0] - p1[0]) * (p1[1] - p3[1]) -
                 (p2[1] - p1[1]) * (p1[0] - p3[0]);

    double ua = n_a / d;
    double ub = n_b / d;

    if (ua >= 0.0 && ua <= 1.0 && ub >= 0.0 && ub <= 1.0)
    {
        // double hitp[3]; // Optional retrieve intersection point.
        // hitp[0] = p1[0] + ua * (p2[0] - p1[0]);
        // hitp[1] = p1[1] + ua * (p2[1] - p1[1]);
        // hitp[2] = p1[2] + ua * (p2[2] - p1[2]);
        return 1;
    }
    return 0;
}

#endif // GEOMETRY_MATH_H
