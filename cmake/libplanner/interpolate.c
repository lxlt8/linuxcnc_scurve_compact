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
#include "interpolate.h"
#include "curve.h"

void interpolate_abc_uvw(const struct emcmot_segment *seg,
                         const double progress,
                         struct EmcPose *pos){

    pos->a = seg->start.a + (seg->end.a - seg->start.a) * progress;
    pos->b = seg->start.b + (seg->end.b - seg->start.b) * progress;
    pos->c = seg->start.c + (seg->end.c - seg->start.c) * progress;

    pos->u = seg->start.u + (seg->end.u - seg->start.u) * progress;
    pos->v = seg->start.v + (seg->end.v - seg->start.v) * progress;
    pos->w = seg->start.w + (seg->end.w - seg->start.w) * progress;
}

void interpolate_abc_uvw_(const struct emcmot_segment *seg,
                         const double progress,
                         struct EmcPose *pos){

    pos->a = seg->a_start + (seg->a_end - seg->a_start) * progress;
    pos->b = seg->b_start + (seg->b_end - seg->b_start) * progress;
    pos->c = seg->c_start + (seg->c_end - seg->c_start) * progress;
    pos->u = seg->u_start + (seg->u_end - seg->u_start) * progress;
    pos->v = seg->v_start + (seg->v_end - seg->v_start) * progress;
    pos->w = seg->w_start + (seg->w_end - seg->w_start) * progress;
}

void interpolate_abc_uvw_peak(const struct emcmot_segment *seg,
                              const double progress,
                              struct EmcPose *pos){

    if(progress<=0.5){
        pos->a = seg->a_start + (seg->a_peak - seg->a_start) * (progress*2);
        pos->b = seg->b_start + (seg->b_peak - seg->b_start) * (progress*2);
        pos->c = seg->c_start + (seg->c_peak - seg->c_start) * (progress*2);
        pos->u = seg->u_start + (seg->u_peak - seg->u_start) * (progress*2);
        pos->v = seg->v_start + (seg->v_peak - seg->v_start) * (progress*2);
        pos->w = seg->w_start + (seg->w_peak - seg->w_start) * (progress*2);

    } else {
        pos->a = seg->a_peak + (seg->a_end - seg->a_peak) * ((progress-0.5)*2);
        pos->b = seg->b_peak + (seg->b_end - seg->b_peak) * ((progress-0.5)*2);
        pos->c = seg->c_peak + (seg->c_end - seg->c_peak) * ((progress-0.5)*2);
        pos->u = seg->u_peak + (seg->u_end - seg->u_peak) * ((progress-0.5)*2);
        pos->v = seg->v_peak + (seg->v_end - seg->v_peak) * ((progress-0.5)*2);
        pos->w = seg->w_peak + (seg->w_end - seg->w_peak) * ((progress-0.5)*2);
    }
}

void interpolate_line(const struct emcmot_segment *seg,
                      const double progress,
                      struct EmcPose *pos){

    // Calculate the interpolated point
    pos->tran.x = seg->start.tran.x + ((seg->end.tran.x - seg->start.tran.x) * progress);
    pos->tran.y = seg->start.tran.y + ((seg->end.tran.y - seg->start.tran.y) * progress);
    pos->tran.z = seg->start.tran.z + ((seg->end.tran.z - seg->start.tran.z) * progress);

    interpolate_abc_uvw(seg,progress,pos);
}

// Function to interpolate between two points p0 and p1
void interpolate_pm_line(PmCartesian p0,
                         PmCartesian p1,
                         double progress,
                         PmCartesian *pi){

    // Linear interpolation for each coordinate
    pi->x = p0.x + progress * (p1.x - p0.x);
    pi->y = p0.y + progress * (p1.y - p0.y);
    pi->z = p0.z + progress * (p1.z - p0.z);
}

void interpolate_pm_seg_line(const struct emcmot_segment *seg,
                             const double progress,
                             PmCartesian *pi){

    // Calculate the interpolated point
    pi->x = seg->start.tran.x + ((seg->end.tran.x - seg->start.tran.x) * progress);
    pi->y = seg->start.tran.y + ((seg->end.tran.y - seg->start.tran.y) * progress);
    pi->z = seg->start.tran.z + ((seg->end.tran.z - seg->start.tran.z) * progress);
}

void interpolate_arc(const struct emcmot_segment *seg,
                     const double progress,
                     struct EmcPose *pos){


    double length=progress*seg->length_xyz;

    double angle;
    pmCircleAngleFromProgress(&seg->coords.circle.xyz,
                              &seg->coords.circle.fit,
                              length, &angle);

    // Calculate position along the circular arc
    PmCartesian pi;
    pmCirclePoint(&seg->coords.circle.xyz, angle, &pi);

    pos->tran.x=pi.x;
    pos->tran.y=pi.y;
    pos->tran.z=pi.z;

    interpolate_abc_uvw(seg,progress,pos);
}

void interpolate_pm_seg_arc(const struct emcmot_segment *seg,
                            const double progress,
                            PmCartesian *pi){

    double length=progress*seg->length_xyz;

    double angle;
    pmCircleAngleFromProgress(&seg->coords.circle.xyz,
                              &seg->coords.circle.fit,
                              length, &angle);

    // Calculate position along the circular arc
    pmCirclePoint(&seg->coords.circle.xyz, angle, pi);
}

// Interpolate a line or orc segment.
void interpolate_pm_seg_line_arc(const struct emcmot_segment *seg,
                                 const double progress,
                                 PmCartesian *pi){

    if(seg->canon_motion_type==3){ // Arc.
        interpolate_pm_seg_arc(seg,progress,pi);
    }
    if(seg->canon_motion_type==1 || seg->canon_motion_type==2){ // Line
        interpolate_pm_seg_line(seg,progress,pi);
    }
}

/*
// Interpolate a 3d pvec.
void interpolate_pvec(const struct emcmot_segment *seg,
                      const double progress,
                      struct EmcPose *pos){

    // Calculate the target length based on progress (from 0 to totalLength)
    double targetLength = progress * seg->length_xyz;

    // Interpolate the position based on the progress
    double accumulatedLength = 0.0;

    // First point.
    PmCartesian p0 = seg->pvec[0];

    for (int i = 1; i < PVEC_SIZE; i++) {
        PmCartesian p1 = seg->pvec[i];

        double segmentLength = distance(p0, p1);
        accumulatedLength += segmentLength;

        // If the target length is within this segment, interpolate between p0 and p1
        if (accumulatedLength >= targetLength) {
            double remainingLength = targetLength - (accumulatedLength - segmentLength);
            double ratio = remainingLength / segmentLength;

            // Interpolate position along this segment
            pos->tran.x = p0.x + ratio * (p1.x - p0.x);
            pos->tran.y = p0.y + ratio * (p1.y - p0.y);
            pos->tran.z = p0.z + ratio * (p1.z - p0.z);

            interpolate_abc_uvw(seg,progress,pos);
            return;  // Successfully interpolated the position
        }

        // Move to the next segment
        p0 = p1;
    }
}*/
























