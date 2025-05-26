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
#ifndef LOOK_AHEAD_H
#define LOOK_AHEAD_H

#include "emcmot_segment.h"
#include "vector.h"
#include "common.h"
#include "scurve.h"

// Trapezium motion profile.
static inline double tpFinalVelocity(double vo, double maxvel, double maxacc, double length) {

    if(length<1e-20){ // Just return the input if no lenght.
        return vo;
    }

    double ve_squared = vo * vo + 2 * maxacc * length;
    double ve = sqrt(ve_squared);

    ve = fmin(ve, maxvel); // Limit by the max velocity (vm)
    // printf("ve trapezium profile: %f \n",ve);
    return ve;
}

// Scurve motion profile. Downside: Starts with acc = 0, ends with acc = 0.
static inline double tpFinalVelocityScurve(double vo, double maxvel, double maxacc, double length, double max_jerk) {

    if(length<1e-20){ // Just return the input if no lenght.
        return vo;
    }

    // double ve_squared = vo * vo + 2 * maxacc * length;
    // double ve = sqrt(ve_squared);

    double ve = 0;
    double time = 0;

    // Setup a scurve
    struct scurve_data data;
    scurve_reset_data(&data);

    scurve_init(&data,
                max_jerk,
                maxacc,
                maxvel,
                time);

    scurve_solver_build_curve(&data);

    data.interval_time_ms = scurve_time_all_periods(&data);
    scurve_solver_update(&data);

    ve = fmin(data.curvel, maxvel); // Limit by the max velocity (vm)
    // printf("ve scurve profile: %f \n",ve);
    return ve;
}

static inline void tpForwardSweep(struct vector *ptr,
                                  struct path_data *path) {

    // Early exit if the vector buffer is empty.
    if (vector_size(ptr) == 0) {
        return;
    }

    int size = push_counter(ptr)-path->global_index;
    int start = path->ringbuffer_index;
    int end = start + size;
    int inor = 0; // normalized i.
    int inor_next = 0;
    int inor_last = 0;

    for (int i = start; i < end-1; i++) {

        inor = i % VECTOR_BUFFER_SIZE;
        inor_next = i+1 % VECTOR_BUFFER_SIZE;
        inor_last = end-1 % VECTOR_BUFFER_SIZE;

        struct emcmot_segment *seg = vector_at(ptr,inor);
        struct emcmot_segment *seg_next = vector_at(ptr,inor_next);
        struct emcmot_segment *seg_last = vector_at(ptr,inor_last);

        // printf("seg id: %d \n",seg->id);
        // printf("seg id next: %d \n",seg_next->id);
        // printf("seg id last: %d \n",seg_last->id);

        // Calculate final velocity ve. Given vo, vel, acc, length.
        /*
        seg->ve = tpFinalVelocity(
                    seg->vo,
                    seg->vel,
                    seg->acc,
                    seg->length_netto
                    ); */

        seg->ve = tpFinalVelocityScurve(seg->vo, seg->vel, seg->acc, seg->length_netto, path->max_jerk);

        // Limit ve to vm;
        seg->ve = fmax( seg->ve, seg->vel);
        // Set next segment vo to this ve.
        seg_next->vo = seg->ve;
        // Set the last motion's velocity to 0.
        seg_last->ve = 0;

        // Below overwites above if condition is met.

        // No inbetween fillets. Stop each motion.
        if( seg_next->canon_motion_type == 2 /* linear */ && seg->canon_motion_type == 2 /* linear */ ){
            seg->vo = 0;
            seg->ve = 0;
            seg_next->vo = 0;
            continue;
        }

        // If next segment is rapid etc. End this motion with ve=0.
        // Set the next motion rapid etc. to vo=0, ve=0.
        if (    seg_next->subseg.length == 0 ||             // Motion has no length for xyz. Then abc uvw will have ve=0.
                seg_next->canon_motion_type == 1 ||         // Motion is rapid.
                seg_next->tag.fields_float[3] == 0 ||       // No deviation.
                seg_next->canon_motion_type == 4 ||         // Motion is tool change.
                seg_next->canon_motion_type == 5) {         // Motion is rigid tap.
            seg->ve = 0;
            seg_next->vo = 0;
            seg_next->ve = 0;
        }

        // If this motion is rapid etc. Use vo=0, ve=0. Also set next motion vo=0.
        if (    seg->subseg.length == 0 ||          // Motion has no length for xyz. Then abc uvw will have ve=0.
                seg->canon_motion_type == 1 ||      // Motion is rapid.
                seg->tag.fields_float[3] == 0 ||    // No deviation.
                seg->canon_motion_type == 4 ||      // Motion is tool change.
                seg->canon_motion_type == 5) {      // Motion is rigid tap.
            seg->vo = 0;
            seg->ve = 0;
            seg_next->vo = 0;
        }
    }
}

// Look ahead, reverse sweep.
static inline void tpReverseSweep(struct vector *ptr,
                                  struct path_data *path) {

    // Early exit if the vector buffer is empty.
    if (vector_size(ptr) == 0) {
        return;
    }

    int size = push_counter(ptr)-path->global_index;
    int start = path->ringbuffer_index;
    int end = start + size;
    int inor = 0; // normalized i.
    int inor_next = 0;
    // int inor_first = 0;

    for (int i = end-1; i > start; --i) {

        inor = i % VECTOR_BUFFER_SIZE;
        inor_next = i-1 % VECTOR_BUFFER_SIZE;
        // inor_first = start % VECTOR_BUFFER_SIZE;

        struct emcmot_segment *seg = vector_at(ptr,inor);
        struct emcmot_segment *seg_prev = vector_at(ptr,inor_next);
        // struct emcmot_segment *seg_first = vector_at(ptr,inor_first);

        // printf("reverse seg id: %d \n",seg->id);
        // printf("reverse seg id prev: %d \n",seg_prev->id);
        // printf("reverse seg id first: %d \n",seg_first->id);

        // Calculate final velocity.
        /* double vo = tpFinalVelocity(
                    seg->ve,
                    seg->vel,
                    seg->acc,
                    seg->length_netto
                    ); */

         double vo = tpFinalVelocityScurve(seg->vo, seg->vel, seg->acc, seg->length_netto, path->max_jerk);

        // Limit vo to vm.
        vo = fmin(seg->vel, vo);
        // Limit vo to previous ve;
        vo = fmin(seg_prev->ve,vo);
        // Apply vo to this seg.
        seg->vo = vo;
        // Apply vo previous seg ve.
        seg_prev->ve = vo;
    }
}

static inline void print_Look_ahead(struct vector *ptr,
                                    struct path_data *path){

    int size = push_counter(ptr)-path->global_index;
    int start = path->ringbuffer_index;
    int end = start + size;
    for(int i=start; i<end; i++){
        int inor = i % VECTOR_BUFFER_SIZE; // i normalized.
        struct emcmot_segment *seg = vector_at(ptr,inor);
        print_emcmot_segment_id_length_vo_vm_ve(seg);
    }
    printf("\n");

    vector_print_content(ptr);

    printf("current index: %d id: %d \n",
           path->ringbuffer_index,
           vector_at(ptr,path->ringbuffer_index)->id);

}

#endif // LOOK_AHEAD_H














