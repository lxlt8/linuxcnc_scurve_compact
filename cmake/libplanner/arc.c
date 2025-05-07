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
#include "arc.h"

PmCartesian CrossProduct(PmCartesian v1,
                         PmCartesian v2) {

    PmCartesian result;
    result.x = v1.y * v2.z - v1.z * v2.y;
    result.y = v1.z * v2.x - v1.x * v2.z;
    result.z = v1.x * v2.y - v1.y * v2.x;
    return result;
}

static double DotProduct(PmCartesian v1,
                         PmCartesian v2) {

    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

PmCartesian Normalize(PmCartesian v) {
    double length = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    PmCartesian res={v.x / length, v.y / length, v.z / length};
    return res;
}

PmCartesian project_to_plane(PmCartesian point,
                             PmCartesian planeOrigin,
                             PmCartesian planeX,
                             PmCartesian planeY) {

    PmCartesian relativePoint = {point.x - planeOrigin.x, point.y - planeOrigin.y, point.z - planeOrigin.z};
    PmCartesian res = {
        DotProduct(relativePoint, planeX),
        DotProduct(relativePoint, planeY),
        0 // Z is 0 in the 2D plane
    };
    return res;
}

PmCartesian map_to_3d(PmCartesian point2D,
                      PmCartesian planeOrigin,
                      PmCartesian planeX,
                      PmCartesian planeY) {

    PmCartesian res = {
        planeOrigin.x + point2D.x * planeX.x + point2D.y * planeY.x,
        planeOrigin.y + point2D.x * planeX.y + point2D.y * planeY.y,
        planeOrigin.z + point2D.x * planeX.z + point2D.y * planeY.z
    };
    return res;
}

PmCartesian arc_center(PmCartesian p0,
                       PmCartesian p1,
                       PmCartesian p2) {

    double A = p1.x - p0.x;
    double B = p1.y - p0.y;
    double C = p2.x - p0.x;
    double D = p2.y - p0.y;

    double E = A * (p0.x + p1.x) + B * (p0.y + p1.y);
    double F = C * (p0.x + p2.x) + D * (p0.y + p2.y);

    double G = 2 * (A * (p2.y - p0.y) - B * (p2.x - p0.x));

    if (G == 0) {
        printf("Points are collinear; no unique circle exists. \n");
    }

    PmCartesian res = {
        (D * E - B * F) / G,
        (A * F - C * E) / G,
        0 // Z is 0 in the 2D plane
    };
    return res;
}

double arc_angle(PmCartesian center, PmCartesian p) {
    return atan2(p.y - center.y, p.x - center.x);
}

void interpolate_3d_arc_length(PmCartesian p0,
                               PmCartesian p1,
                               PmCartesian p2,
                               double progress,
                               double *arc_length,
                               PmCartesian *pi){

    // Compute the normal vector of the plane
    PmCartesian v1 = {p1.x - p0.x, p1.y - p0.y, p1.z - p0.z};
    PmCartesian v2 = {p2.x - p0.x, p2.y - p0.y, p2.z - p0.z};
    PmCartesian normal = Normalize(CrossProduct(v1, v2));

    // Define the plane's basis vectors
    PmCartesian planeX = Normalize(v1);
    PmCartesian planeY = Normalize(CrossProduct(normal, planeX));

    // Project the points onto the plane
    PmCartesian p0_2D = project_to_plane(p0, p0, planeX, planeY);
    PmCartesian p1_2D = project_to_plane(p1, p0, planeX, planeY);
    PmCartesian p2_2D = project_to_plane(p2, p0, planeX, planeY);

    // Compute the circle in the plane
    PmCartesian center_2D = arc_center(p0_2D, p1_2D, p2_2D);

    // Compute the angles
    double theta0 = arc_angle(center_2D, p0_2D);
    double theta2 = arc_angle(center_2D, p2_2D);

    // Ensure the arc goes from theta0 to theta2 in the correct direction
    if (theta2 < theta0) {
        theta2 += 2 * M_PI;
    }

    // Interpolate the angle
    double theta = theta0 + progress * (theta2 - theta0);

    // Generate the interpolated point in 2D
    double radius = sqrt(pow(p0_2D.x - center_2D.x, 2) + pow(p0_2D.y - center_2D.y, 2));
    PmCartesian interpolatedPoint_2D = {
        center_2D.x + radius * cos(theta),
        center_2D.y + radius * sin(theta),
        0
    };

    // Compute the total arc length
    *arc_length = radius * (theta2 - theta0);

    // Map the interpolated point back to 3D
    *pi = map_to_3d(interpolatedPoint_2D, p0, planeX, planeY);
}

int is_zero_vector(PmCartesian v) {
    return (v.x == 0 && v.y == 0 && v.z == 0);
}

int arc_points_collinear(PmCartesian p0,
                         PmCartesian p1,
                         PmCartesian p2){

    double d0 = distance(p0,p1);
    double d1 = distance(p1,p2);
    double d2 = distance(p0,p2);
    double toll = 1e-9; // Seems to be best value.

    if(d0 + d1 > d2 - toll && d0 + d1 < d2 + toll){
        return 1;
    }
    return 0;
}

int arc_test() {
    PmCartesian p0 = {0, 0, 0};
    PmCartesian p1 = {1, 1, };
    PmCartesian p2 = {2, 0, 0};
    PmCartesian pi = {};
    int points=10;
    double progress=0;
    double arc_length=0;

    for(int i=0; i<=points; i++){
        progress = (double)i / points;
        printf("progress: %f \n",progress);

        interpolate_3d_arc_length(p0, p1, p2, progress, &arc_length, &pi);
        printf("x: %f y: %f z: %f \n",pi.x, pi.y, pi.z);

    }

    printf("arc_length: %f \n",arc_length);
    return 0;
}

double arc_length(PmCartesian p0,
                  PmCartesian p1,
                  PmCartesian p2){

    if(arc_points_collinear(p0,p1,p2)){ // Its a line.
        return distance(p0,p2);
    }

    PmCartesian pi = {};
    double arc_length=0;
    double progress=0;
    interpolate_3d_arc_length(p0, p1, p2, progress, &arc_length, &pi);
    return arc_length;
}

PmCartesian interpolate_3d_arc(PmCartesian p0,
                               PmCartesian p1,
                               PmCartesian p2,
                               double progress){

    PmCartesian pi = {};
    if(arc_points_collinear(p0,p1,p2)){ // Its a line.
        interpolate_pm_line(p0,p2,progress,&pi);
        return pi;
    }

    double arc_length=0;
    interpolate_3d_arc_length(p0, p1, p2, progress, &arc_length, &pi);
    return pi;
}

/*
void interpolate_arc_pvec(const struct emcmot_segment *seg,
                          const double progress,
                          struct EmcPose *pos){

    // Calculate the target length based on progress (from 0 to totalLength)
    double targetLength = progress * seg->length_xyz;

    // Interpolate the position based on the progress
    double accumulatedLength = 0.0;

    int psize = PVEC_SIZE;
    if (psize % 2 == 0) { // Is even value?
        psize+=1;
    }

    for (int i = 0; i < psize-2; i+=2) {

        // printf("arc nr: %d \n",i);

        PmCartesian p0 = seg->pvec[i+0];
        PmCartesian p1 = seg->pvec[i+1];
        PmCartesian p2 = seg->pvec[i+2];

        double segmentLength = arc_length(p0,p1,p2); // distance(p0, p1);
        accumulatedLength += segmentLength;

        // If the target length is within this segment, interpolate between p0 and p1
        if (accumulatedLength >= targetLength) {
            double remainingLength = targetLength - (accumulatedLength - segmentLength);
            double ratio = remainingLength / segmentLength;

            PmCartesian pi = interpolate_3d_arc(p0,p1,p2,ratio);
            pos->tran.x = pi.x;
            pos->tran.y = pi.y;
            pos->tran.z = pi.z;
            return;
        }
    }

    interpolate_abc_uvw(seg,progress,pos);
}*/














