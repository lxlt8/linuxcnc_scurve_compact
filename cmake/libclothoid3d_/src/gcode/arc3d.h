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
#ifndef ARC3D_H
#define ARC3D_H

#include <stdio.h>
#include "../segment.h"
#include "../math/vector_math.h"
#include "../math/geometry_math.h"
#include "../eq30.h"

/* [ARCHIVED] Calculate a arc half way point.
 * Valid for : non helix arc.
 */
static void arc_waypoint(const double p0[3], const double pc[3], const double pn[3], const double arc_angle, const int cw, double pw[3]){

    double pc1[3] = {pc[0]+pn[0], pc[1]+pn[1], pc[2]+pn[2]};
    // Calculate arc waypoint pw.
    if(cw){
        rotate_point_around_line(p0,-0.5*arc_angle,pc,pc1,pw);
    } else {
        rotate_point_around_line(p0,0.5*arc_angle,pc,pc1,pw);
    }
    // printf("pw: (%.2f, %.2f, %.2f)\n", pw[0], pw[1], pw[2]);
}

/* [ARCHIVED] Function to calculate the angle of the 3D arc defined by points p0, pw, p1
 * Valid for : non helix arc.
 *
 * p0 = arc start.
 * pw = arc way point.
 * p1 = arc end point.
 * arc_angle = calculated arc angle in radians.
 * pc = calculated arc center point.
 */
static int init_2p_wp_arc(const double p0[3], const double pw[3], const double p1[3], double *arc_angle, double pc[3]) {
    // Check if p0 and p1 are the same point
    if (p0[0] == p1[0] && p0[1] == p1[1] && p0[2] == p1[2]) {
        printf("Error, points p0 and p1 are identical.\n");
        return 0;
    }

    // Calculate vectors from p0 to pw and p0 to p1
    double v1[3], v2[3];
    subtract_vectors(pw, p0, v1);  // v1 = pw - p0
    subtract_vectors(p1, p0, v2);  // v2 = p1 - p0

    // Calculate dot products
    double v1v1 = dot_product(v1, v1);
    double v2v2 = dot_product(v2, v2);
    double v1v2 = dot_product(v1, v2);

    // Base calculation for coefficients
    double denom = v1v1 * v2v2 - v1v2 * v1v2;
    if (fabs(denom) < 1e-20) { // Check for small denominator
        printf("Error, arc points are too close or collinear.\n");
        return 0;
    }

    double base = 0.5 / denom;
    double k1 = base * v2v2 * (v1v1 - v1v2);
    double k2 = base * v1v1 * (v2v2 - v1v2);

    // Calculate the center of the arc
    pc[0] = p0[0] + v1[0] * k1 + v2[0] * k2;
    pc[1] = p0[1] + v1[1] * k1 + v2[1] * k2;
    pc[2] = p0[2] + v1[2] * k1 + v2[2] * k2;

    double va[3];
    double vb[3];
    // Vectors p0 to pc, p1 to pc.
    subtract_vectors(p0,pc,va);
    subtract_vectors(p1,pc,vb);
    normalize(va);
    normalize(vb);

    // Calculate cross product for arc plane normal
    double n[3];
    cross_product(v1, v2, n);
    double nl = sqrt(dot_product(n, n));
    if (nl < 1e-12) { // Check for small norm
        printf("Error, arc vectors are collinear.\n");
        return 0;
    }
    normalize(n);

    // Calculate arc angle using dot product, clamp the dot product to [-1, 1]
    double dot = dot_product(va, vb);
    dot = fmax(-1.0, fmin(1.0, dot));
    *arc_angle = acos(dot);

    // Check the sign of the angle
    double vab[3];
    cross_product(va, vb, vab);
    if (dot_product(vab, n) < 0) {
        *arc_angle = 2 * M_PI - *arc_angle;
    }

    return 1;
}

// Function to calculate the length of a helix given the arc radius, helix depth, and arc angle
static void arc_length_calc(double arc_radius, double helix_depth, double arc_angle, double *arc_length) {
    // Calculate the number of turns from the arc angle (arc_angle in radians)
    double turns = arc_angle / (2 * M_PI);

    // Circumference of the arc at each turn
    double circumference = 2 * M_PI * arc_radius;

    // Vertical distance per turn (divide the helix depth by the number of turns)
    double vertical_distance_per_turn = helix_depth / turns;

    // Length of one full turn (hypotenuse of the triangle formed by vertical distance and arc circumference)
    double turn_length = sqrt(pow(circumference, 2) + pow(vertical_distance_per_turn, 2));

    // Return the length for the given arc angle (using the calculated turns)
    *arc_length = turn_length * turns;
}

/* Function to calculate the length of a helix */
static void helix_length(double *radius, double *pitch, double *theta, double *length) {
    // Calculate the helix length using the formula
    *length = sqrt(pow(*radius, 2) + pow(*pitch / (2 * M_PI), 2)) * (*theta);
}

/* Function to calculate the arc angle from gcode info. Helix valid.
 *
 * p0 = arc start.
 * p1 = arc end.
 * pc = arc center.
 * pn = arc normal.
 * cw = clockwise flag = 1. counter clockwise ccw = 0;
 * turns = helix full turns. 0=no extra turns.
 * arc_angle = calculated arc angle in radians.
 */
static void arc_angle_length_radius(const double p0[3], const double p1[3], const double pc[3], const double pn[3], const int cw, const double turns,
double *arc_angle, double *arc_length, double *arc_radius, double *arc_pitch) {

    double fabs_turns = fabs(turns);

    // For helixes the point p1 could be out off plane, because of the pitch value wich ends at p1.
    // The task: project p1 onto the plane p0, pc, where pn is the plane normal axis.
    // p1_ is point p1 projected on plane.
    double p1_[3];
    copy_vector(p1,p1_);
    project_point_on_plane(p1,pc,pn,p1_);
    double helix_depth = distance_3d(p1,p1_);

    double v0[3];
    double v1[3];

    // Calculate vectors from center to p0 and p1
    subtract_vectors(p0,pc,v0);
    subtract_vectors(p1_,pc,v1);

    // Normalize the vectors
    normalize(v0);
    normalize(v1);

    // Calculate the dot product of the two normalized vectors
    double dot = dot_product(v0, v1);
    dot = fmax(-1.0, fmin(1.0, dot));  // Clamp the value to avoid NaN from floating point errors

    // Calculate the angle (in radians). This angle is from 0-180 degrees.
    double angle = acos(dot);

    // Use the cross product to determine direction
    double cross[3];
    cross_product(v0, v1, cross);

    // Dot product of the cross product with the normal vector (pn)
    double direction = dot_product(cross, pn);

    // Adjust the angle based on the direction.
    if (direction < 0 && !cw) {
        angle = 2 * M_PI - angle;
        // printf("0-180. \n");
    }
    if (direction > 0 && cw) {
        angle = 2 * M_PI - angle;
        // printf("180-360. \n");
    }
    if (direction == 0) {
        // printf("180. \n");
    }

    // Exception, if startpoint = endpoint.
    if(angle==0){
        fabs_turns+=1;
    }

    *arc_angle = angle + ((fabs_turns) * (2*M_PI));
    *arc_radius = distance_3d(p0,pc);

    double real_turns = *arc_angle / (2*M_PI);
    *arc_pitch = helix_depth / real_turns;

    arc_length_calc(*arc_radius, helix_depth, *arc_angle, arc_length);
    // printf("arc lenght original: %f \n",*arc_length);

    // printf("arc angle: %f \n",*arc_angle);
    // Extra check.
    // double ltot=0;
    // helix_length(arc_radius,arc_pitch,arc_angle,&ltot);
    // printf("ltot: %f \n",ltot);
}

/* Interpolate arc given length s.
 *
 * p0 = arc start.
 * pc = arc center.
 * pn = arc normal. Axis trough center.
 * arc_angle = total arc angle including helix turns.
 * cw = clockwise flag = 1. Counter clockwise ccw = 0;
 * theta = total angle up to interpolated point.
 */
static void interpolate_arc3d(const struct segment *seg, const double progress, double pi[3], double *theta){

    // For helixes the point p1 could be out off plane, because of the pitch value wich ends at p1.
    // The task: project p1 onto the plane p0, pc, where pn is the plane normal axis.
    // p1_ is point p1 projected on plane.
    double p1_[3];
    copy_vector(seg->p1,p1_);                               // Temponairy end point p1_.
    project_point_on_plane(seg->p1,seg->pc,seg->pn,p1_);    // p1_ is projected on helix zero plane.
    double helix_span = distance_3d(seg->p1,p1_);           // Get the helix span p1 to p1_
    // printf("helix span: %f \n",helix_span);

    // Decide if the helix is moving off or toward the plane p0, pc, with normal axis pn.
    // If dir is positive, it moves off the plane.
    double pcp1[3];
    subtract_vectors(seg->pc,seg->p1,pcp1);
    double dir = dot_product(pcp1,seg->pn);

    // Arc axis points pc, pc1.
    double pc1[3] = {seg->pc[0]+seg->pn[0], seg->pc[1]+seg->pn[1], seg->pc[2]+seg->pn[2]}; // Helix axis points: pc, pc1.
    *theta = progress * seg->angle;

    // Arc interpolation on plane.
    double p1i[3];
    if(seg->cw){
        rotate_point_around_line(seg->p0,-(*theta),seg->pc,pc1,p1i);
    } else {
        rotate_point_around_line(seg->p0,*theta,seg->pc,pc1,p1i);
    }

    // Helix depth based on current progress.
    double helix_dist = progress * helix_span;  // Helix distance in the direction of the helix axis.
    // Direction of cut.
    if(dir>0){
        helix_dist*=-1;
    }

    // Arc interpolation + helix current depth.
    double helix_v[3];
    double pnn[3];
    copy_vector(seg->pn,pnn); // Because pn is const.
    normalize(pnn);
    scale_vector(pnn, helix_dist, helix_v);
    add_vectors(helix_v, p1i, pi);
}

/* If a helix is trimmed, the center point must be updated.
 *
 * 1. We need the travel on the helix axis.
 * 2. We need the helix axis points to get the axis direction.
 * 3. Check the new turns.
 *
 * theta = total angle up to interpolated point.
 */
static void interpolate_arc3d_update_center(struct segment *seg, const double progress, double pi[3], double *theta){

    // For helixes the point p1 could be out off plane, because of the pitch value wich ends at p1.
    // The task: project p1 onto the plane p0, pc, where pn is the plane normal axis.
    // p1_ is point p1 projected on plane.
    double p1_[3];
    copy_vector(seg->p1,p1_);                               // Temponairy end point p1_.
    project_point_on_plane(seg->p1,seg->pc,seg->pn,p1_);    // p1_ is projected on helix zero plane.
    double helix_span = distance_3d(seg->p1,p1_);           // Get the helix span p1 to p1_
    // printf("helix span: %f \n",helix_span);

    // Decide if the helix is moving off or toward the plane p0, pc, with normal axis pn.
    // If dir is positive, it moves off the plane.
    double pcp1[3];
    subtract_vectors(seg->pc,seg->p1,pcp1);
    double dir = dot_product(pcp1,seg->pn);

    // Arc axis points pc, pc1.
    double pc1[3] = {seg->pc[0]+seg->pn[0], seg->pc[1]+seg->pn[1], seg->pc[2]+seg->pn[2]}; // Helix axis points: pc, pc1.
    *theta = progress * seg->angle;

    // Arc interpolation on plane.
    double p1i[3];
    if(seg->cw){
        rotate_point_around_line(seg->p0,-(*theta),seg->pc,pc1,p1i);
    } else {
        rotate_point_around_line(seg->p0,*theta,seg->pc,pc1,p1i);
    }

    // Helix depth based on current progress.
    double helix_dist = progress * helix_span;  // Helix distance in the direction of the helix axis.
    // Direction of cut.
    if(dir>0){
        helix_dist*=-1;
    }

    // Arc interpolation + helix current depth.
    double helix_v[3];
    double pnn[3];
    copy_vector(seg->pn,pnn);   // Temp axis normal.
    normalize(pnn);             // Normalized helix axis direction vector.
    scale_vector(pnn, helix_dist, helix_v);
    add_vectors(helix_v, p1i, pi);

    // Use scale_vector to compute the translation vector
    add_vectors(helix_v,seg->pc,seg->pc);

    // print_vector("arc-helix new center:",seg->pc);
}


/* Arc, helix properties to fit on a 3d clothoid.
 *
 * p0 = arc start point.
 * pc = arc center point.
 * pn = arc normal axis trough arc center.
 * cw = arc clocwise flag = 1. Counter clockwise ccw = 0
 * pitch = helix forward distance each turn.
 *
 * theta1i = z tilt angle related to top view xy plane.
 * theta2i = xy angle at top view xy plane.
 * kappa1i = z kappa.
 * kappa2i = xy kappa.
 * sharpness1i = z sharpness c.
 * sharpness2i = xy sharpness c.
 */
static void arc_tangents_kappas_sharpnesses_z_xy(  const double p0[3], const double p1[3], const double pc[3], const double pn[3], const int cw,
const double pitch, const double progress, const double arc_angle,
double *theta1i, double *theta2i,
double *kappa1i, double *kappa2i,
double *sharpness1i, double *sharpness2i){

    double theta_v[3]; // Theta vector at s.
    eq30_theta_helix(p0,p1,pc,pn,cw,pitch,progress,arc_angle,theta_v);

    // Decompose into z and xy.
    eq30_decompose_theta_components(theta_v,theta1i,theta2i);

    double kappa_v[3]; // Theta vector at s.
    eq30_kappa_helix(p0,p1,pc,pn,cw,pitch,progress,arc_angle,kappa_v);
    // Decompose into z and xy.
    eq30_decompose_kappa_components(kappa_v,kappa1i,kappa2i);

    double sharpness_v[3]; // Theta vector at s.
    eq30_sharpness_helix(p0,p1,pc,pn,cw,pitch,progress,arc_angle,sharpness_v);
    // Decompose into z and xy.
    eq30_decompose_sharpness_components(sharpness_v,sharpness1i,sharpness2i);

    // It seems for a G2 arc, the kappa is always negative.
    if(cw){
        *kappa1i=-fabs(*kappa1i);
        *kappa2i=-fabs(*kappa2i);
    }
}

/* Plot to file.
 */
static void plot_arc_file(struct segment *seg, FILE *data_file){

    double segments=100;
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double pi[3]={};

        // Alternative interpolation function:
        // interpolate_arc(seg,progress,pi);

        eq30_interpolate_helix(seg->p0,seg->p1,seg->pc,seg->pn,seg->cw,seg->pitch,progress,seg->angle,pi);

        fprintf(data_file, "%lf %lf %lf\n", pi[0], pi[1], pi[2]);
        // print_vector("arc pi:",pi);
    }
    fprintf(data_file, "\n\n");  // Add two blank lines to separate datasets
}

/* Plot arc helix.
 */
static void plot_arc(struct segment *seg){

    // Draw result.
    // Print to file for visualization (if needed)
    FILE *data_file = fopen("arc_helix.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    double segments=100;
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double pi[3]={};

        printf("progress plot: %f \n", progress);
        // Alternative interpolation function:
        // interpolate_arc(seg,progress,pi);

        eq30_interpolate_helix(seg->p0,seg->p1,seg->pc,seg->pn,seg->cw,seg->pitch,progress,seg->angle,pi);

        fprintf(data_file, "%lf %lf %lf\n", pi[0], pi[1], pi[2]);
        // printf("pi: (%f, %f, %f)\n", pi[0], pi[1], pi[2]);
    }

    fflush(data_file);
    fclose(data_file);

    double min = -100;
    double max = 100;

    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up 3D plotting
        fprintf(gnuplot, "set title 'Arc helix test'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");

        fprintf(gnuplot, "set xrange [%f:%f]\n", min, max); // Use %f to format the double value
        fprintf(gnuplot, "set yrange [%f:%f]\n", min, max); // Use %f to format the double value
        fprintf(gnuplot, "set zrange [%f:%f]\n", min, max); // Use %f to format the double value

        // Set view angle for better 3D visualization
        fprintf(gnuplot, "set view 70, 20\n");  // You can adjust the angle here

        // Plot the data in 3D (use correct filename)
        fprintf(gnuplot, "splot 'arc_helix.dat' using 1:2:3 with lines\n");

        fflush(gnuplot);
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Initialize arc helix.
 */
static void init_arc(   const double p0[3],
const double p1[3],
const double pc[3],
const double pn[3],
const int cw,
const int turns,
struct segment *seg,
        int update_original_lenght
        ){

    seg->segment_type=ARC;

    seg->theta10=0;
    seg->theta14=0;
    seg->theta20=0;
    seg->theta24=0;

    seg->kappa10=0;
    seg->kappa14=0;
    seg->kappa20=0;
    seg->kappa24=0;

    seg->sharpness10=0;
    seg->sharpness14=0;
    seg->sharpness20=0;
    seg->sharpness24=0;

    copy_vector(p0,seg->p0);
    copy_vector(p1,seg->p1);
    copy_vector(pc,seg->pc);
    copy_vector(pn,seg->pn);

    seg->cw=cw;
    seg->turns=turns;

    arc_angle_length_radius(p0,p1,pc,pn,cw,seg->turns,
                            &seg->angle,&seg->length,&seg->radius,&seg->pitch);

    // Used for trimming.
    if(update_original_lenght){
        seg->original_length=seg->length;
    }

    double progress;

    progress=0.0;
    arc_tangents_kappas_sharpnesses_z_xy(seg->p0,seg->p1,seg->pc,seg->pn,seg->cw,seg->pitch,progress,seg->angle,
                                         &seg->theta10,&seg->theta20,
                                         &seg->kappa10,&seg->kappa20,
                                         &seg->sharpness10,&seg->sharpness20);
    progress=1.0;
    arc_tangents_kappas_sharpnesses_z_xy(seg->p0,seg->p1,seg->pc,seg->pn,seg->cw,seg->pitch,progress,seg->angle,
                                         &seg->theta14,&seg->theta24,
                                         &seg->kappa14,&seg->kappa24,
                                         &seg->sharpness14,&seg->sharpness24);
}

/* Test & plot arc helix.
                 */
static void test_arc_yz_plane(){

    // Arc in yz plane. x is normal.
    struct segment seg;
    double p0[3] = {0,0,0};
    double pc[3] = {0,50,0};
    double p1[3] = {0,100,0};
    double pn[3] = {1,0,0};
    double cw=1;
    double turns=1;

    init_arc(p0,p1,pc,pn,cw,turns,&seg,0);
    plot_arc(&seg);
    print_segment(&seg);
}

static void test_arc_xz_plane(){

    // Arc in yz plane. x is normal.
    struct segment seg;
    double p0[3] = {0,0,50};
    double pc[3] = {50,0,50};
    double p1[3] = {100,100,50};
    double pn[3] = {0,1,0};
    double cw=1;
    double turns=5;

    init_arc(p0,p1,pc,pn,cw,turns,&seg,0);
    plot_arc(&seg);
    print_segment(&seg);
}

static void test_arc_xy_plane(){

    // Arc in yz plane. x is normal.
    struct segment seg;
    double p0[3] = {0,0,0};
    double pc[3] = {50,0,0};
    double p1[3] = {100,0,0};
    double pn[3] = {0,0,1};
    double cw=1;
    double turns=0;

    init_arc(p0,p1,pc,pn,cw,turns,&seg,0);
    plot_arc(&seg);
    print_segment(&seg);
}

#endif // ARC3D_H














