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
#ifndef ARC_H
#define ARC_H

#include "posemath.h"
#include <math.h>
#include <stdio.h>
#include "curve.h"
#include "interpolate.h"

// Function to compute the cross product of two vectors
PmCartesian CrossProduct(PmCartesian v1,
                         PmCartesian v2);
// Function to compute the dot product of two vectors
static double DotProduct(PmCartesian v1,
                         PmCartesian v2);
// Function to normalize a vector
PmCartesian Normalize(PmCartesian v);
// Function to project a 3D point onto a 2D plane
PmCartesian project_to_plane(PmCartesian point,
                             PmCartesian planeOrigin,
                             PmCartesian planeX,
                             PmCartesian planeY);
// Function to map a 2D point back to 3D space
PmCartesian map_to_3d(PmCartesian point2D,
                      PmCartesian planeOrigin,
                      PmCartesian planeX,
                      PmCartesian planeY);
// Function to compute the circle passing through three 2D points
PmCartesian arc_center(PmCartesian p0,
                       PmCartesian p1,
                       PmCartesian p2);
// Function to compute the angle of a 2D point relative to the circle's center
double arc_angle(PmCartesian center,
                 PmCartesian p);
// Function to interpolate a point on the arc at parameter t
void interpolate_3d_arc_length(PmCartesian p0,
                               PmCartesian p1,
                               PmCartesian p2,
                               double progress,
                               double *arc_length,
                               PmCartesian *pi);
// Function to check if a vector is a zero vector
int is_zero_vector(PmCartesian v);
// Function to check if three points are collinear, returns 1 if points are colinear.
int arc_points_collinear(PmCartesian p0,
                         PmCartesian p1,
                         PmCartesian p2);
// Output seems ok.
int arc_test();
// Give 3 arc waypoints. Create arc.
double arc_length(PmCartesian p0,
                  PmCartesian p1,
                  PmCartesian p2);
// Arc derived from 3 waypoints. Progress from 0-1.
PmCartesian interpolate_3d_arc(PmCartesian p0,
                               PmCartesian p1,
                               PmCartesian p2,
                               double progress);

/*
void interpolate_arc_pvec(const struct emcmot_segment *seg,
                          const double progress,
                          struct EmcPose *pos);*/
#endif // ARC_H






