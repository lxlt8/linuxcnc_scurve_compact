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
#ifndef DEVIATION_POINT_H
#define DEVIATION_POINT_H

#include "common.h"
#include "curve.h"
#include "interpolate.h"

/*
* Calculate the deviation intersection point given 2 segments.
*
* The deviation point is the point between 2 segments wich is
* the absolute limit for a fillet curve.
*
* The deviation point is calculated given the G64 P[x] value.
*
* Steps :
*
*   1. Get the minimal lenght of segments.
*
*/
PmCartesian deviation_point(
        struct emcmot_segment *seg0,
        struct emcmot_segment *seg1,
        PmCartesian *deviation_point, // Deviation point.
        PmCartesian *fillet_start,    // fillet start. distance = 2*deviation
        PmCartesian *fillet_end       // fillet end.   distance = 2*deviation
        ){

    double deviation = seg0->tag.fields_float[3]; // Deviation G64 P[x] value.

    double lmin = fmax(seg0->length_xyz, seg1->length_xyz);
    if(lmin>0.01){
        lmin=0.01;
    }

    // Offset point on circumfence.
    PmCartesian pi0, pi1;
    interpolate_pm_seg_line_arc(seg0, ( 1- (lmin / seg0->length_xyz)) ,&pi0);
    interpolate_pm_seg_line_arc(seg1, (lmin / seg1->length_xyz) ,&pi1);

    // Create a virtual line from pi0 to pi1, then return the midpoint.
    PmCartesian pi2;
    interpolate_pm_line(pi0,pi1,0.5,&pi2);

    // Offset a point on the line seg0.end to pi2, given the deviation.
    *deviation_point = offset_point_on_line(seg0->end.tran, pi2, deviation);

    // Upper limit of fillet start, end point on curve.
    double limit = fmax(seg0->length_xyz*0.5, seg1->length_xyz*0.5);

    // Reduce deviation. Update deviation point.
    if(limit < 2*deviation){
        deviation = 0.5 * limit;
        *deviation_point = offset_point_on_line(seg0->end.tran, pi2, deviation);
    }

    // Calcutate fillet start & endpoint.
    double offset = 2*deviation;
    interpolate_pm_seg_line_arc(seg0, ( 1- (offset / seg0->length_xyz)) ,fillet_start);
    interpolate_pm_seg_line_arc(seg1, (offset / seg1->length_xyz) ,fillet_end);

    printf("p0 x: %f y: %f z: %f \n",fillet_start->x, fillet_start->y, fillet_start->z);
    printf("p1 x: %f y: %f z: %f \n",fillet_end->x, fillet_end->y, fillet_end->z);
    printf("pi x: %f y: %f z: %f \n",deviation_point->x, deviation_point->y, deviation_point->z);
}

#endif // DEVIATION_POINT_H












