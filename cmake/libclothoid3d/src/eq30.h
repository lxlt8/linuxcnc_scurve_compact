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
#ifndef EQ30_H
#define EQ30_H

#include <stdio.h>
#include "math/vector_math.h"
#include "math/geometry_math.h"

/* Funtion to get 3 unit vectors from an arc. Given arc start, arc center, arc end point. Page 6 for illustration.
 * Needed for eq.30
 *
 * pa0 = arc start xyz
 * pae = arc end xyz
 * pc  = arc center xyz
 * na  = arc normal
 *
 * va1 = unit vector arc center to arc start. Mathematicly called : normal N.
 * va2 = cross product va1, na. A 90 degrees unit vector va1, na. Mathematicly called : tangent T.
 *
 * The question is, here we thread G2, G3 the same. So unit vector's may differ along the way.
 */
static void va1_va2(double pa0[3], double pc[3], double na[3], double va1[3], double va2[3]){

    // va1.
    unit_vector(pc,pa0,va1);

    // va2.
    cross_product_normalized(va1, na, va2);
}

/* Funtion to get the unit vector from a line. Page 6 for illustration.
 *
 * pl0 = line start xyz.
 * ple = line end xyz.
 * vl  = unit vector line from start to end.
 */
static void vl(double pl0[3], double ple[3], double vl[3]){
    unit_vector(pl0,ple,vl);
}

/* Calcuate first derivate θ "theta" of the point P, into P`traj(s), given s = arc circumfence lenght
 * from arc startpoint.
 */
static void eq30_theta(double pa0[3], double pc[3], double na[3], double s, double ptraj_prime[3]){

    // Calculate the unit vectors for the arc.
    double va1[3]; // Normal N.
    double va2[3]; // Tangent T.
    va1_va2(pa0, pc, na, va1, va2);

    // Arc radius.
    double r = distance_3d(pa0,pc);

    // Step 1: Calculate sin(s/r) and cos(s/r)
    double sin_term = sin(s/r);
    double cos_term = cos(s/r);

    // Step 2: Compute ptraj_prime components
    ptraj_prime[0] = -sin_term * va1[0] + cos_term * va2[0];  // x-component
    ptraj_prime[1] = -sin_term * va1[1] + cos_term * va2[1];  // y-component
    ptraj_prime[2] = -sin_term * va1[2] + cos_term * va2[2];  // z-component
}

/* Calcuate second derivate K "kappa" of the point P, into P`traj(s), given s = arc circumfence lenght
 * from arc startpoint.
 */
static void eq30_kappa(double pa0[3], double pc[3], double na[3], double s, double ptraj_prime_double[3]){

    // Calculate the unit vectors for the arc.
    double va1[3];
    double va2[3];
    va1_va2(pa0, pc, na, va1, va2);

    // Arc radius.
    double r = distance_3d(pa0,pc);

    // Step 1: Calculate sin(s/r) and cos(s/r)
    double cos_term = (1/r) * cos(s/r);
    double sin_term = (1/r) * sin(s/r);

    // Step 2: Compute ptraj_prime components
    ptraj_prime_double[0] = -cos_term * va1[0] - sin_term * va2[0];  // x-component
    ptraj_prime_double[1] = -cos_term * va1[1] - sin_term * va2[1];  // y-component
    ptraj_prime_double[2] = -cos_term * va1[2] - sin_term * va2[2];  // z-component
}

/* Calcuate third derivate c "sharpness" of the point P, into P`traj(s), given s = arc circumfence lenght
 * from arc startpoint.
 */
static void eq30_sharpness(double pa0[3], double pc[3], double na[3], double s, double ptraj_prime_tri[3]){

    // Calculate the unit vectors for the arc.
    double va1[3];
    double va2[3];
    va1_va2(pa0, pc, na, va1, va2);

    // Arc radius.
    double r = distance_3d(pa0,pc);

    // Step 1: Calculate sin(s/r) and cos(s/r)
    double sin_term = (1/(r*r)) * sin(s/r);
    double cos_term = (1/(r*r)) * cos(s/r);

    // Step 2: Compute ptraj_prime components
    ptraj_prime_tri[0] = sin_term * va1[0] - cos_term * va2[0];  // x-component
    ptraj_prime_tri[1] = sin_term * va1[1] - cos_term * va2[1];  // y-component
    ptraj_prime_tri[2] = sin_term * va1[2] - cos_term * va2[2];  // z-component
}

/* Calculate the curvature kappa of a helix
 *
 * Curvature κ = radius r / (radius squared r2 + pitch squared p2)
 *
 * r = radius.
 * pitch = pitch per turn.
 */
static void eq30_kappa_radius_pitch(const double r, const double pitch, double *kappa) {

    // p = pitch per unit arc length. Is calculated as the vertical distance per turn (pitch) divided by 2π,
    double p = pitch / (2 * M_PI);  // Corrected parentheses
    *kappa = r / (r * r + p * p);
}

/* Interpolate arc, helix given distance s.
 * Alternative to function ~/gcode/arc.h -> interpolate_arc
 */
static void eq30_interpolate_helix(const double p0[3], const double p1[3], const double pc[3], const double pn[3],
                            const int cw, const double pitch, const double progress, const double arc_angle, double ptraj[3]) {

    double radius = distance_3d(p0, pc);
    if (radius == 0.0) return;  // Avoid division by zero

    double N[3];  // Normal vector
    subtract_vectors(p0, pc, N);  // Normal vector, p0 to pc
    normalize(N);

    double T[3];  // Tangent vector
    cross_product(N, pn, T);  // Tangent vector: cross product of N and pn (helix axis)
    normalize(T);

    if (cw == 0) {
        // For counterclockwise (CCW), reverse the direction of T
        negate_vector(T);
    }

    double theta = progress * arc_angle;  // progress should be in [0, 1]

    // Compute sine and cosine terms
    double sin_term = sin(theta);
    double cos_term = cos(theta);

    // Compute the vertical displacement corresponding to arc length s
    double vertical_displacement = (pitch * theta) / (2 * M_PI);

    // Decide if the helix is moving off or toward the plane p0, pc, with normal axis pn.
    // If dir is positive, it moves off the plane.
    double pcp1[3];
    subtract_vectors(pc,p1,pcp1);
    double dir = dot_product(pcp1,pn);
    // Flip the sign of the vertical component if CW (downward motion)
    if (dir > 0) {
        vertical_displacement = -vertical_displacement;
    }

    // Compute the interpolated point on the helix
    ptraj[0] = pc[0] + radius * (cos_term * N[0] + sin_term * T[0]) + vertical_displacement * pn[0];  // x-component
    ptraj[1] = pc[1] + radius * (cos_term * N[1] + sin_term * T[1]) + vertical_displacement * pn[1];  // y-component
    ptraj[2] = pc[2] + radius * (cos_term * N[2] + sin_term * T[2]) + vertical_displacement * pn[2];  // z-component
}

/* Calculate first derivative θ "theta" of the point P for a helix,
 * into P`traj(s), given s = arc circumference length from arc startpoint.
 * The function calculates the tangent vector at the specified arc length s.
 *
 * p0 = arc start point.
 * pc = arc center point.
 * pn = arc central axis direction.
 * pitch = pitch per turn (vertical distance per turn).
 * s = arc length along the helix.
 */
static void eq30_theta_helix(const double p0[3], const double p1[3], const double pc[3], const double pn[3],
                             const int cw, const double pitch, const double progress, const double arc_angle, double ptraj_prime[3]) {
    double radius = distance_3d(p0, pc);
    if (radius == 0.0) return;  // Avoid division by zero

    double N[3];  // Normal vector
    subtract_vectors(p0, pc, N);  // Normal vector, p0 to pc
    normalize(N);

    double T[3];  // Tangent vector
    cross_product(N, pn, T);  // Tangent vector: cross product of N and pn (helix axis)
    normalize(T);

    if (cw == 0) {
        // For counterclockwise (CCW), reverse the direction of T
        negate_vector(T);
    }

    double theta = progress * arc_angle;  // progress should be in [0, 1]

    // Compute sine and cosine terms
    double sin_term = sin(theta);
    double cos_term = cos(theta);

    // Compute the vertical motion per unit arc length (helical rise)
    double vertical_term = (pitch / (2 * M_PI * radius));  // Equivalent to torsion

    // Decide if the helix is moving off or toward the plane p0, pc, with normal axis pn.
    // If dir is positive, it moves off the plane.
    double pcp1[3];
    subtract_vectors(pc, p1, pcp1);
    double dir = dot_product(pcp1, pn);
    // Flip the sign of the vertical component if CW (downward motion)
    if (dir > 0) {
        vertical_term = -vertical_term;
    }

    // Compute the first derivative (tangent vector) of the helix
    ptraj_prime[0] = -sin_term * N[0] + cos_term * T[0] + vertical_term * pn[0];  // x-component
    ptraj_prime[1] = -sin_term * N[1] + cos_term * T[1] + vertical_term * pn[1];  // y-component
    ptraj_prime[2] = -sin_term * N[2] + cos_term * T[2] + vertical_term * pn[2];  // z-component
}

/* Calculate the second derivative kappa(s) for a helix
 *
 * p0 = arc start point.
 * p1 = arc end point.
 * pc = arc center point.
 * pn = arc central axis trough pc.
 * cw = arc direction flag.
 * turns = helix turns.
 *
 * s = distance at helix circumfence.
 * ptraj_prime_double = kappa vector at s.
 *
 */
static void eq30_kappa_helix(   const double p0[3], const double p1[3], const double pc[3], const double pn[3],
                                const int cw, const double pitch, const double progress, const double arc_angle,
                                double ptraj_prime_double[3]) {

    double radius = distance_3d(p0, pc);
    if (radius == 0.0) return;  // Avoid division by zero

    double N[3];  // Normal vector
    subtract_vectors(p0, pc, N);  // Normal vector, p0 to pc
    normalize(N);

    double T[3];  // Tangent vector
    cross_product(N, pn, T);  // Tangent vector: cross product of N and pn (helix axis)
    normalize(T);

    if (cw == 0) {
        // For counterclockwise (CCW), reverse the direction of T
        negate_vector(T);
    }

    double theta = progress * arc_angle;

    // Compute the second derivative terms for kappa (curvature)
    double cos_term = cos(theta);
    double sin_term = sin(theta);

    // Compute torsion component (helical rise effect)
    double vertical_term = pitch / (2 * M_PI * radius * 2 * M_PI * radius);  // Torsion

    // Decide if the helix is moving off or toward the plane p0, pc, with normal axis pn.
    // If dir is positive, it moves off the plane.
    double pcp1[3];
    subtract_vectors(pc,p1,pcp1);
    double dir = dot_product(pcp1,pn);
    // Flip the sign of the vertical component if CW (downward motion)
    if (dir > 0) {
        vertical_term = -vertical_term;
    }

    // Compute the second derivative (curvature vector)
    ptraj_prime_double[0] = (-cos_term * N[0] - sin_term * T[0]) / radius + vertical_term * pn[0];
    ptraj_prime_double[1] = (-cos_term * N[1] - sin_term * T[1]) / radius + vertical_term * pn[1];
    ptraj_prime_double[2] = (-cos_term * N[2] - sin_term * T[2]) / radius + vertical_term * pn[2];
}


/* Calculate third derivative (sharpness) of the helix
 *
 * p0 = arc start point.
 * pc = arc center point.
 * pn = arc central axis direction.
 * pitch = pitch per turn (vertical distance per turn).
 * s = arc length along the helix.
 * cw = arc direction flag (clockwise or counter-clockwise).
 * ptraj_prime_tri = sharpness vector at s.
 */
static void eq30_sharpness_helix(  const double p0[3], const double p1[3], const double pc[3], const double pn[3],
                            const int cw, const double pitch, const double progress, const double arc_angle, double ptraj_prime_tri[3]) {

    double radius = distance_3d(p0, pc);
    if (radius == 0.0) return;  // Avoid division by zero

    double N[3];  // Normal vector
    subtract_vectors(p0, pc, N);  // Normal vector, p0 to pc
    normalize(N);

    double T[3];  // Tangent vector
    cross_product(N, pn, T);  // Tangent vector: cross product of N and pn (helix axis)
    normalize(T);

    if (cw == 0) {
        // For counterclockwise (CCW), reverse the direction of T
        negate_vector(T);
    }

    double theta = progress * arc_angle;  // progress should be in [0, 1]

    // Compute sine and cosine terms
    double sin_term = sin(theta);
    double cos_term = cos(theta);

    // Compute the vertical motion per unit arc length (helical rise)
    double vertical_term = (pitch / (2 * M_PI * radius));  // Equivalent to torsion

    // Decide if the helix is moving off or toward the plane p0, pc, with normal axis pn.
    double pcp1[3];
    subtract_vectors(pc, p1, pcp1);
    double dir = dot_product(pcp1, pn);
    if (dir > 0) {
        vertical_term = -vertical_term;  // Adjust for downward motion
    }

    // Compute the sharpness (third derivative) by differentiating the curvature
    // The sharpness involves the rate of change of the curvature (second derivative)

    ptraj_prime_tri[0] = (cos_term * N[0] + sin_term * T[0]) / (radius * radius) + vertical_term * pn[0];
    ptraj_prime_tri[1] = (cos_term * N[1] + sin_term * T[1]) / (radius * radius) + vertical_term * pn[1];
    ptraj_prime_tri[2] = (cos_term * N[2] + sin_term * T[2]) / (radius * radius) + vertical_term * pn[2];
}

/* Decompose vector.
 * ptraj_prime[3] = theta vector at s (distance), calculated by: eq30_theta_helix();
 * theta1i = z tilt angle from top xy plane.
 * theta2i = xy angle from top xy plane.
 */
static void eq30_decompose_theta_components(const double ptraj_prime[3], double *theta1i, double *theta2i) {

    *theta1i = atan2(ptraj_prime[2], magnitude_xy(ptraj_prime));  // Vertical angle
    *theta2i = atan2(ptraj_prime[1], ptraj_prime[0]);  // Horizontal angle in the XY-plane.
}

/* Decompose vector.
 * ptraj_prime_double[3] = kappa vector at s (distance), calculated by: eq30_kappa_helix();
 * kappa1i is the curvature along the z-direction (vertical component).
 * kappa2i is the magnitude of the curvature vector in the xy-plane,
 */
static void eq30_decompose_kappa_components(const double ptraj_prime_double[3], double *kappa1i, double *kappa2i) {

    // Calculate the magnitude of the xy-plane component
    double xy_magnitude = magnitude_xy(ptraj_prime_double); // Horizontal magnitude

    // kappa1i is the curvature in the z-direction (z-component of the curvature vector)
    *kappa1i = ptraj_prime_double[2];  // Vertical curvature (z-component)

    // kappa2i is the curvature in the xy-plane (horizontal curvature)
    *kappa2i = xy_magnitude;  // Horizontal curvature (xy-component)
}

/* Decompose sharpness vector.
 * ptraj_prime_tri[3] = sharpness vector at s (distance), calculated by: eq30_sharpness_helix();
 * sharpness1i is the sharpness along the z-direction (vertical component).
 * sharpness2i is the magnitude of the sharpness vector in the xy-plane,
 */
static void eq30_decompose_sharpness_components(const double ptraj_prime_tri[3], double *sharpness1i, double *sharpness2i) {
    // Calculate the magnitude of the xy-plane component of the sharpness vector
    double xy_magnitude = magnitude_xy(ptraj_prime_tri); // Horizontal magnitude

    // sharpness1i is the sharpness in the z-direction (z-component of the sharpness vector)
    *sharpness1i = ptraj_prime_tri[2];  // Vertical sharpness (z-component)

    // sharpness2i is the sharpness in the xy-plane (horizontal sharpness)
    *sharpness2i = xy_magnitude;  // Horizontal sharpness (xy-component)
}

// Funtion to proof eq30.
static void eq30_proof(){

    // An arc, given 3 points.
    double pa0[3] = {0,0,0};    // Arc start point.
    // double pae[3] = {100,0,0};  // Arc end point.
    double pc[3]  = {50,0,0};   // Arc center point.
    double na[3]  = {0,0,1};    // Arc normal.

    double s=0; // If s=0, it means we are on the arc at the startpoint.
    // Use arclenght s to interpolate over the arc circumfence.

    printf("eq30_proof, calculate derivates off the segment. \n");
    printf("distance on segment s: %lf\n", s);

    // Calculate the first derivate θ theta at s=0, wich means arc startpoint.
    double ptraj_prime[3];      // Result first derivate θ theta.
    eq30_theta(pa0,pc,na,s,ptraj_prime);
    printf("ptraj_prime (theta) x: %lf\n", ptraj_prime[0]);
    printf("ptraj_prime (theta) y: %lf\n", ptraj_prime[1]);
    printf("ptraj_prime (theta) z: %lf\n", ptraj_prime[2]);

    // Calculate the second derivate K kappa at s=0, wich means arc startpoint.
    double ptraj_prime_double[3];
    eq30_kappa(pa0,pc,na,s,ptraj_prime_double);
    printf("ptraj_prime_double (kappa) x: %lf\n", ptraj_prime_double[0]);
    printf("ptraj_prime_double (kappa) y: %lf\n", ptraj_prime_double[1]);
    printf("ptraj_prime_double (kappa) z: %lf\n", ptraj_prime_double[2]);

    // Calculate the third derivate c sharpness at s=0, wich means arc startpoint.
    double ptraj_prime_tri[3];
    eq30_sharpness(pa0,pc,na,s,ptraj_prime_tri);
    printf("ptraj_prime_tri (sharpness) x: %lf\n", ptraj_prime_tri[0]);
    printf("ptraj_prime_tri (sharpness) y: %lf\n", ptraj_prime_tri[1]);
    printf("ptraj_prime_tri (sharpness) z: %lf\n", ptraj_prime_tri[2]);

    printf("\n");

    /* Terminal output:
        eq30_proof, calculate derivates off the segment.
        distance on segment s: 0.000000
        ptraj_prime (theta) x: 0.000000
        ptraj_prime (theta) y: 1.000000
        ptraj_prime (theta) z: -0.000000
        ptraj_prime_double (kappa) x: 0.020000
        ptraj_prime_double (kappa) y: -0.000000
        ptraj_prime_double (kappa) z: 0.000000
        ptraj_prime_tri (sharpness) x: -0.000000
        ptraj_prime_tri (sharpness) y: -0.000400
        ptraj_prime_tri (sharpness) z: 0.000000
     */
}

#endif // EQ30_H


















