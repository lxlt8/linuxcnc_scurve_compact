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
#ifndef PATH_STANDARD_SUBSEG_LINE_FILLET_H
#define PATH_STANDARD_SUBSEG_LINE_FILLET_H

#include "common.h"
#include "curve.h"
#include "feed.h"
#include "fit.h"
#include "interpolate.h"
// clothoid library includes.
#include "clothoid3d.h"
#include "arc3d.h"
#include "line3d.h"

#include "sub_segment.h"

/* Add segments to buffer. Test path with a simple line fillet.
 *
 */
static inline int path_standard_subseg_line_fillet(TP_STRUCT * const tp,
                                                   struct path_data *path,
                                                   struct emcmot_segment s2,
                                                   struct vector *ptr){

    struct emcmot_segment *s0;
    struct emcmot_segment s1;

    copy_emcmot_segment_to_sub_segment(&s2);

    // Look for extern axis motions, if xyz has no motion.
    abc_uvw_lenght(&s2);
    enum enum_motion_set set = slowest_motion_set(emcmotConfig, &s2);
    if(s2.length_netto==0 && set==abc){
        s2.length_netto=s2.length_abc;
    }
    if(s2.length_netto==0 && set==uvw){
        s2.length_netto=s2.length_uvw;
    }

    // Add no fillet segment when vector is empty and no deviation is set.
    if(push_counter(vector_ptr)==0){
        s2.trajectory_length_begin=0;
        s2.trajectory_length_end=s2.length_netto;
        path->trajectory_length=s2.trajectory_length_end;
        vector_push_back(ptr, s2); // New segment.
        return 0;
    }

    s0 = vector_at(vector_ptr, push_counter(vector_ptr)-1);
    if(s0 == NULL){
        printf("s0 error. \n");
    }

    // No path deviation. Only add the new segment. No fillet is added.
    // No fillet for rigid tap, tool change, rapid.
    if(s2.tag.fields_float[3] < 1e-6        // Deviation.
            || s2.canon_motion_type==4      // Rigid tap.
            || s2.canon_motion_type==5      // Tool change.
            || s2.canon_motion_type==1 ){   // Rapic.
        s2.trajectory_length_begin = s0->trajectory_length_end;
        s2.trajectory_length_end = s2.trajectory_length_begin + s2.length_netto;
        path->trajectory_length=s2.trajectory_length_end;
        vector_push_back(ptr, s2);
        return 0;
    }

    // No path deviation if previous motion is a rapid.
    if( s0->canon_motion_type==1){
        s2.trajectory_length_begin = s0->trajectory_length_end;
        s2.trajectory_length_end = s2.trajectory_length_begin + s2.length_netto;
        path->trajectory_length=s2.trajectory_length_end;
        vector_push_back(ptr, s2);
        return 0;
    }

    // Test a line fit.
    apply_line_fit(&s0->subseg, &s2.subseg, &s1.subseg, s2.tag.fields_float[3]);
    s1.vel=s2.vel;
    s1.ini_maxvel=s2.ini_maxvel;
    s1.acc=s2.acc;
    s1.start.a=0;
    s1.start.b=0;
    s1.start.c=0;
    s1.end.a=0;
    s1.end.b=0;
    s1.end.c=0;
    s1.canon_motion_type=2;
    s1.tag=s2.tag;          // Copy G64 P[x]. Used to set endvel=0.
    s1.id=s2.id;            // Copy gcode line id.

    if(s1.tag.fields_float[3]==0){
        printf("deveation error. \n");
    }

    // Update trajectory length.
    s0->trajectory_length_end = s0->trajectory_length_begin + s0->length_netto;
    s1.trajectory_length_begin = s0->trajectory_length_end;
    s1.trajectory_length_end = s1.trajectory_length_begin + s1.length_netto;
    s2.trajectory_length_begin = s1.trajectory_length_end;
    s2.trajectory_length_end = s2.trajectory_length_begin + s2.length_netto;
    path->trajectory_length=s2.trajectory_length_end;

    // Add new segments to queue.
    vector_push_back(ptr, s1);
    vector_push_back(ptr, s2);

    return 0;
}

/* Interpolation for segments.
 * This interpolation is done by the clothoid library.
 * No external axis are added yet.
 */
static inline void path_standard_subseg_line_fillet_interpolate(TP_STRUCT * const tp,
                                                                struct path_data *path,
                                                                struct emcmot_segment *seg){

    double pi[3];

    // Interpolate if segment is line.
    if (seg->subseg.segment_type == LINE) {

        interpolate_line3d(&seg->subseg, path->progress, pi);
        tp->currentPos.tran.x=pi[0];
        tp->currentPos.tran.y=pi[1];
        tp->currentPos.tran.z=pi[2];

        // Interpolate extra axis.
        struct EmcPose abc_uvw_pi;
        interpolate_abc_uvw(seg,path->progress,&abc_uvw_pi);
        tp->currentPos.a=abc_uvw_pi.a;
        tp->currentPos.b=abc_uvw_pi.b;
        tp->currentPos.c=abc_uvw_pi.c;
        tp->currentPos.u=abc_uvw_pi.u;
        tp->currentPos.v=abc_uvw_pi.v;
        tp->currentPos.w=abc_uvw_pi.w;
    }

    // Interpolate if segment is arc.
    if (seg->subseg.segment_type == ARC) {

        double theta;
        interpolate_arc3d(&seg->subseg, path->progress, pi,&theta);
        tp->currentPos.tran.x=pi[0];
        tp->currentPos.tran.y=pi[1];
        tp->currentPos.tran.z=pi[2];

        // Interpolate extra axis.
        struct EmcPose abc_uvw_pi;
        interpolate_abc_uvw(seg,path->progress,&abc_uvw_pi);
        tp->currentPos.a=abc_uvw_pi.a;
        tp->currentPos.b=abc_uvw_pi.b;
        tp->currentPos.c=abc_uvw_pi.c;
        tp->currentPos.u=abc_uvw_pi.u;
        tp->currentPos.v=abc_uvw_pi.v;
        tp->currentPos.w=abc_uvw_pi.w;
    }
}

#endif // PATH_STANDARD_SUBSEG_LINE_FILLET_H













