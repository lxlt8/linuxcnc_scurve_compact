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
#ifndef PATH_STANDARD_SUBSEG_H
#define PATH_STANDARD_SUBSEG_H

#include "common.h"
#include "curve.h"
#include "feed.h"
#include "interpolate.h"
#include "sub_segment.h"

// Adding segment's to queue. No path optimalistion.
static inline int path_standard_subseg(TP_STRUCT * const tp,
                                       struct path_data *path,
                                       struct emcmot_segment seg,
                                       struct vector *ptr){

    copy_emcmot_segment_to_sub_segment(&seg);

    // Look for extern axis motions, if xyz has no motion.
    abc_uvw_lenght(&seg);
    enum enum_motion_set set = slowest_motion_set(emcmotConfig, &seg);
    if(seg.length_netto==0 && set==abc){
        seg.length_netto=seg.length_abc;
    }
    if(seg.length_netto==0 && set==uvw){
        seg.length_netto=seg.length_uvw;
    }

    // Update segment trajectory length.
    if(push_counter(vector_ptr)==0){
        seg.trajectory_length_begin=0;
        seg.trajectory_length_end=seg.length_netto;
        path->trajectory_length=seg.trajectory_length_end;
    } else {
        // Get previous segment.
        struct emcmot_segment *prev_seg = vector_at(vector_ptr, push_counter(vector_ptr)-1);
        // Update trajectory length's for this segment.
        seg.trajectory_length_begin=prev_seg->trajectory_length_end;
        seg.trajectory_length_end=prev_seg->trajectory_length_end + seg.length_netto;
        path->trajectory_length=seg.trajectory_length_end;
    }

    // Add segment to queue.
    vector_push_back(ptr, seg);

    return 0;
}

/* Interpolation for segments.
 * This contains a 3d vector based interpolation model for arc-helix.
 */
static inline void path_standard_subseg_interpolate(TP_STRUCT * const tp,
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

#endif // PATH_STANDARD_SUBSEG_H
