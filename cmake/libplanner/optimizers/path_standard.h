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
#ifndef PATH_STANDARD_H
#define PATH_STANDARD_H

#include "common.h"
#include "curve.h"
#include "feed.h"
#include "interpolate.h"

// Adding segment's to queue. No path optimalistion.
static inline int path_standard(TP_STRUCT * const tp,
                                struct path_data *path,
                                struct emcmot_segment seg,
                                struct vector *ptr){

    // Calculate lenght of xyz, abc, uvw motions.
    init_curve(tp,&seg);

    // Calculate feeds for the motion. Apply G93, G94, G95.
    init_feed(tp,&seg);

    // Get the netto lenght from xyz and or external axis.
    init_segment_length(&seg);

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

    // printf("seg length: %f \n",seg.length_netto);
    // printf("trajector length: %f \n",path->trajectory_length);

    // Add segment to queue.
    vector_push_back(ptr, seg);

    return 0;
}

// Interpolation for segments.
static inline void path_standard_interpolate(TP_STRUCT * const tp,
                                             struct path_data *path,
                                             struct emcmot_segment *seg){

    // Interpolate if segment is a line.
    if (seg->canon_motion_type == 1 || seg->canon_motion_type == 2
            || seg->canon_motion_type == 4 || seg->canon_motion_type == 5) {
        interpolate_line(seg,path->progress,&tp->currentPos);
    } else

    // Interpolate if segment is an arc.
    if (seg->canon_motion_type == 3) {
        interpolate_arc(seg,path->progress,&tp->currentPos);
    } else {
        printf("unknown motion type: %d \n",seg->canon_motion_type);
    }
}

#endif // PATH_STANDARD_H
