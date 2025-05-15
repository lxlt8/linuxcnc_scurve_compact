#ifndef RESIDUAL_FILLET_FIT_H
#define RESIDUAL_FILLET_FIT_H

#include "segment.h"
#include "line3d.h"
#include "arc3d.h"
#include "clothoid3d.h"
#include "algorithm1.h"
#include "math.h"

static void trim_segment(struct segment *seg, int front, int back, const double *s){

    double trim = *s;
    // Limit trim lenght to not exceed half segment length.
    // Use same 0.499 as in ceres_intf.cpp file.
    trim=fmin(trim,seg->original_length*0.499);

    if(front){

        // Interpolate from the front, given s.
        double pi1[3];
        if(seg->segment_type==LINE){
            double progress = trim / seg->length;
            interpolate_line3d(seg,progress,pi1);

            // Re-initialize seg with new endpoint.
            init_line(pi1,seg->p1,seg,0);
        }
        // Here the difficulty is that the nr. of helix turns can change by used trim dist.
        // Therefore we calculate the nr. of helix turns again.
        if(seg->segment_type==ARC){
            double theta_start;
            double progress = trim / seg->length;
            interpolate_arc3d_update_center(seg,progress,pi1,&theta_start);

            double theta_end;
            progress=1.0;
            double pi[3];
            interpolate_arc3d(seg,progress,pi,&theta_end);

            // Floor is round to below.
            seg->turns = (int)floor( fabs(theta_end-theta_start) / (2*M_PI) );

            // Re-initialize seg.
            init_arc(pi1,seg->p1,seg->pc,seg->pn,seg->cw,seg->turns,seg,0);
        }
    }

    if(back){
        // Interpolate from the back, given s.
        double pi0[3];
        if(seg->segment_type==LINE){
            double progress = (seg->length - trim) / seg->length;
            interpolate_line3d(seg,progress,pi0);

            // Re-initialize seg with new start point.
            init_line(seg->p0,pi0,seg,0);
        }
        // Here the difficulty is that the nr. of helix turns can change by used trim dist.
        // Therefore we calculate the nr. of helix turns again.
        if(seg->segment_type==ARC){

            double theta_start;
            double progress=0.0;
            double pi[3];
            interpolate_arc3d(seg,progress,pi,&theta_start);

            double theta_end;
            progress = (seg->length - trim) / seg->length;
            interpolate_arc3d(seg,progress,pi0,&theta_end);

            // Floor is round to below.
            seg->turns = (int)floor( fabs(theta_end-theta_start) / (2*M_PI) );

            // Re-initialize seg.
            init_arc(seg->p0,pi0,seg->pc,seg->pn,seg->cw,seg->turns,seg,0);
        }
    }
}

/* Residual function for ceres solver.
 */
static void residual_fillet_fit(struct segment seg0,
                                struct segment seg1,
                                struct segment seg2,
                                const double *s,
                                double *deviation){

    // Intersection seg0, seg1. Gcode corner.
    double corner[3];
    copy_vector(seg0.p1,corner);

    trim_segment(&seg0,0,1,s);
    trim_segment(&seg1,1,0,s);

    // Add inbetween clothoid.
    init_inbetween_clothoid(&seg0,&seg1,&seg2,-INFINITY,INFINITY);

    // print_clothoid_segment(&seg2);

    // Example to use the clothoid midpoint.
    // double pi[3] = {seg2.x2,seg2.y2,seg2.z2};
    // *deviation = distance(pi,corner);
    // printf("deviation: %f \n",*deviation);

    // Find deviation from corner to clothoid.
    double d;
    double pi[3];
    algorithm1(corner,seg2,1e-6,pi,&d,deviation);
}

#endif // RESIDUAL_FILLET_FIT_H
