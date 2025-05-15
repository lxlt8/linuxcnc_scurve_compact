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
#ifndef GCODE_COMMON_H
#define GCODE_COMMON_H

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "math/geometry_math.h" // To degrees.

enum clothoid_solver {
    NELDER_MEAD,
    CERES_CAPI_JACOBIAN,
    CERES_CENTRAL_DIFF,
    CERES_FORWARD_DIFF,
    CERES_RIDDERS_DIFF
};

enum segment_type {
    LINE,
    ARC,
    CLOTHOID
};

enum canon_code {
    G0,
    G1,
    G2,
    G3
};

struct segment {

    enum segment_type segment_type;
    enum canon_code canon_code;

    double p0[3];
    double p1[3];
    double pc[3];
    double pn[3];

    // Arc specific.
    int cw;
    double turns;
    double trim_theta_front;
    double trim_theta_back;
    double pitch;
    double radius;
    double angle;
    double length;

    // Arc & line.
    double theta10; // Z.
    double theta20; // XY.
    double kappa10;
    double kappa20;
    double sharpness10;
    double sharpness20;

    double theta14;
    double theta24;
    double kappa14;
    double kappa24;
    double sharpness14;
    double sharpness24;

    // Clothoid data.
    enum clothoid_solver solver_enum;

    double y11; // Z.
    double y21; // XY.
    double s1;

    // To be calculated by eq44.
    double theta11, theta12, theta13;
    double theta21, theta22, theta23;

    double kappa11, kappa12, kappa13;
    double kappa21, kappa22, kappa23;

    double sharpness11, sharpness12, sharpness13;
    double sharpness21, sharpness22, sharpness23;

    double y12, y13, y14;
    double y22, y23, y24;

    // Intermediate points for clothoid spline.
    double x1, y1, z1;
    double x2, y2, z2;
    double x3, y3, z3;

    double s1_lower_limit;
    double s1_upper_limit;

    double original_length;
};

// Function to print all data in the struct
static void zero_segment(struct segment *seg){

    seg->segment_type = CLOTHOID;
    seg->canon_code = G0;

    seg->p0[0]=0;
    seg->p0[1]=0;
    seg->p0[2]=0;

    seg->p1[0]=0;
    seg->p1[1]=0;
    seg->p1[2]=0;

    seg->pc[0]=0;
    seg->pc[1]=0;
    seg->pc[2]=0;

    seg->pn[0]=0;
    seg->pn[1]=0;
    seg->pn[2]=0;

    // Arc specific.
    seg->cw=0;
    seg->turns=0;
    seg->pitch=0;
    seg->radius=0;
    seg->angle=0;
    seg->length=0;

    // Arc & line.
    seg->theta10=0;
    seg->theta20=0;
    seg->kappa10=0;
    seg->kappa20=0;
    seg->sharpness10=0;
    seg->sharpness20=0;

    seg->theta14=0;
    seg->theta24=0;
    seg->kappa14=0;
    seg->kappa24=0;
    seg->sharpness14=0;
    seg->sharpness24=0;

    // Clothoid data.
    seg->solver_enum=CERES_CENTRAL_DIFF;

    seg->y11=0;
    seg->y21=0;
    seg->s1=0;

    // To be calculated by eq44.
    seg->theta11 = seg->theta12 = seg->theta13 = 0;
    seg->theta21 = seg->theta22 = seg->theta23 = 0;

    seg->kappa11 = seg->kappa12 = seg->kappa13 = 0;
    seg->kappa21 = seg->kappa22 = seg->kappa23 = 0;

    seg->sharpness11 = seg->sharpness12 = seg->sharpness13 =0;
    seg->sharpness21 = seg->sharpness22 = seg->sharpness23 =0;

    seg->y12 = seg->y13 = seg->y14 =0;
    seg->y22 = seg->y23 = seg->y24 =0;

    // Intermediate points for clothoid spline.
    seg->x1 = seg->y1 = seg->z1 =0;
    seg->x2 = seg->y2 = seg->z2 =0;
    seg->x3 = seg->y3 = seg->z3 =0;

    seg->s1_lower_limit=0;
    seg->s1_upper_limit=0;

    seg->original_length=0;
}

// Function to print all data in the struct
static void print_segment(const struct segment *seg){
    printf("Segment Data:\n");
    printf("segment_type: %d\n", seg->segment_type);
    printf("canon_code: %d\n", seg->canon_code);

    printf("p0: [%f, %f, %f]\n", seg->p0[0], seg->p0[1], seg->p0[2]);
    printf("p1: [%f, %f, %f]\n", seg->p1[0], seg->p1[1], seg->p1[2]);
    printf("pc: [%f, %f, %f]\n", seg->pc[0], seg->pc[1], seg->pc[2]);
    printf("pn: [%f, %f, %f]\n", seg->pn[0], seg->pn[1], seg->pn[2]);

    printf("cw: %d\n", seg->cw);
    printf("turns: %f\n", seg->turns);
    printf("pitch: %f\n", seg->pitch);
    printf("radius: %f\n", seg->radius);
    printf("angle rad: %f\n", seg->angle);
    printf("angle deg: %f\n", seg->angle*toDegrees);
    printf("length: %f\n", seg->length);

    printf("theta10 rad  Z: %f\n", seg->theta10);
    printf("theta20 rad XY: %f\n", seg->theta20);
    printf("theta10 deg  Z: %f\n", seg->theta10*toDegrees);
    printf("theta20 deg XY: %f\n", seg->theta20*toDegrees);
    printf("kappa10      Z: %f\n", seg->kappa10);
    printf("kappa20     XY: %f\n", seg->kappa20);
    printf("sharpness10  Z: %f\n", seg->sharpness10);
    printf("sharpness20 XY: %f\n", seg->sharpness20);

    printf("theta14 rad  Z: %f\n", seg->theta14);
    printf("theta24 rad XY: %f\n", seg->theta24);
    printf("theta14 deg  Z: %f\n", seg->theta14*toDegrees);
    printf("theta24 deg XY: %f\n", seg->theta24*toDegrees);
    printf("kappa14      Z: %f\n", seg->kappa14);
    printf("kappa24     XY: %f\n", seg->kappa24);
    printf("sharpness14  Z: %f\n", seg->sharpness14);
    printf("sharpness24 XY: %f\n", seg->sharpness24);
}

static void print_clothoid_segment(const struct segment *seg){

    printf("Clothoid Data:\n");
    printf("---------------\n");

    printf("s1: %f\n", seg->s1);

    // Print total length
    printf("total length: %f\n", seg->length);

    // Print solver type
    printf("Solver Enum: %d\n", seg->solver_enum);

    // Print calculated theta values
    printf("theta10 Z : %f, theta11: %f, theta12: %f, theta13: %f, theta14: %f\n", seg->theta10, seg->theta11, seg->theta12, seg->theta13, seg->theta14);
    printf("theta20 XY: %f, theta21: %f, theta22: %f, theta23: %f, theta24: %f\n", seg->theta20, seg->theta21, seg->theta22, seg->theta23, seg->theta24);

    // Print calculated kappa values
    printf("kappa10  Z: %f, kappa11: %f, kappa12: %f, kappa13: %f, kappa14: %f\n", seg->kappa10, seg->kappa11, seg->kappa12, seg->kappa13, seg->kappa14);
    printf("kappa20 XY: %f, kappa21: %f, kappa22: %f, kappa23: %f, kappa24: %f\n", seg->kappa20, seg->kappa21, seg->kappa22, seg->kappa23, seg->kappa24);

    // Print calculated sharpness values
    printf("sharpness10  Z: %f, sharpness11: %f, sharpness12: %f, sharpness13: %f, sharpness14: %f\n", seg->sharpness10, seg->sharpness11, seg->sharpness12, seg->sharpness13, seg->sharpness14);
    printf("sharpness20 XY: %f, sharpness21: %f, sharpness22: %f, sharpness23: %f, sharpness24: %f\n", seg->sharpness20, seg->sharpness21, seg->sharpness22, seg->sharpness23, seg->sharpness24);

    // Print calculated y values
    printf("y11  Z: %f, y12: %f, y13: %f, y14: %f\n", seg->y11, seg->y12, seg->y13, seg->y14);
    printf("y21 XY: %f, y22: %f, y23: %f, y24: %f\n", seg->y21, seg->y22, seg->y23, seg->y24);

    // Print start point
    printf("Start Point (xs, ys, zs): (%f, %f, %f)\n", seg->p0[0], seg->p0[1], seg->p0[2]);
    // Print intermediate points
    printf("Intermediate Point 1 (x1, y1, z1): (%f, %f, %f)\n", seg->x1, seg->y1, seg->z1);
    printf("Intermediate Point 2 (x2, y2, z2): (%f, %f, %f)\n", seg->x2, seg->y2, seg->z2);
    printf("Intermediate Point 3 (x3, y3, z3): (%f, %f, %f)\n", seg->x3, seg->y3, seg->z3);
    // Print end point.
    printf("End Point (xe, ye, ze): (%f, %f, %f)\n", seg->p1[0], seg->p1[1], seg->p1[2]);

    printf("---------------\n");
}

#endif // GCODE_COMMON_H

















