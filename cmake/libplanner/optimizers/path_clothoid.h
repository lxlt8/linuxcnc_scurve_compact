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
#ifndef PATH_CLOTHOID_H
#define PATH_CLOTHOID_H

#include "mot_priv.h"
#include "common.h"
#include "emcmot_segment.h"
#include "sub_segment.h"

#include "feed.h"
#include "fit.h"
#include "clothoid3d.h"
#include "arc3d.h"
#include "line3d.h"

#include "time.h"
#include <stdio.h>
#include <stdlib.h>

/* Add segments to buffer. Add clothoid fillets given the max deviation.
 *
 */
static inline int path_clothoid(TP_STRUCT * const tp,
                                struct path_data *path,
                                struct emcmot_segment s2,
                                struct vector *ptr){

    struct emcmot_segment *s0;
    struct emcmot_segment s1;

    // Get the previous segment, if the buffer size>0.
    if(push_counter(vector_ptr)>0){
        s0 = vector_at(vector_ptr, push_counter(vector_ptr)-1);
        if(s0 == NULL){
            printf("s0 error. \n");
        }
    }

    // Copy segment values to clothoid sub segment struct.
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

    // Add no fillet segment when vector is empty.
    if(push_counter(vector_ptr)==0){
        s2.trajectory_length_begin=0;
        s2.trajectory_length_end=s2.length_netto;
        path->trajectory_length=s2.trajectory_length_end;
        kmax_arc_line(&s2);
        kmax_vel(&s2);
        init_feed(tp,&s2);
        vector_push_back(ptr, s2);
        return 0;
    }

    /* Conditions to not add a clothoid fillet, but only add the end segment :
     *
     *      - No path deviation is given. -> G64 P0.0
     *      - Segment has no lenght for xyz.
     *      - Motion is off type rigid tap.
     *      - Motion is off type tool change.
     *      - Motion is off type rapid.
     */
    if(     s2.subseg.length==0                 // Segment has no length.
            || s2.tag.fields_float[3] < 1e-6    // No deviation.
            || s2.canon_motion_type==4          // Rigid tap.
            || s2.canon_motion_type==5          // Tool change.
            || s2.canon_motion_type==1 ){       // Rapid.
        s2.trajectory_length_begin = s0->trajectory_length_end;
        s2.trajectory_length_end = s2.trajectory_length_begin + s2.length_netto;
        path->trajectory_length=s2.trajectory_length_end;
        init_feed(tp,&s2);
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

    // Clothoid fillet may be added.
    // Set clothoid fillet values.
    s1.subseg.segment_type=CLOTHOID;
    s1.ini_maxvel = s2.ini_maxvel;
    s1.vel = s2.vel;
    s1.acc = s2.acc;
    s1.start.a=s0->end.a;
    s1.start.b=s0->end.b;
    s1.start.c=s0->end.c;
    s1.end.a=s0->end.a;
    s1.end.b=s0->end.b;
    s1.end.c=s0->end.c;
    s1.tag=s2.tag;          // Copy G64 P[x]. Used to set endvel=0.
    s1.id=s2.id;            // Copy gcode line id.

    if(s1.tag.fields_float[3]==0){
        printf("deveation error. \n");
    }

    // Start measuring time
    // clock_t start = clock();

    // Fit clothoid given deviation.
    if(path->use_real_deviation){
        // Compute expensive function.
        if(0!=fit(&s0->subseg, &s2.subseg, &s1.subseg, s2.tag.fields_float[3])){
            printf("clothoid fit error. \n");
            s1.subseg.length=0;
        }
    } else {
        // Fit clothoid given trim distance. No deviation.
        // Calculation time ~0.43 - ~0.64ms.
        // Very short segment ~53 - ~72 cycles.
        double s = (double)s2.tag.fields_float[3];

        trim_segment(&s0->subseg,0,1,&s);
        trim_segment(&s2.subseg,1,0,&s);

        init_inbetween_clothoid(&s0->subseg,&s2.subseg,&s1.subseg,-INFINITY,INFINITY);

        // Check for extra torsion turns.
        int fix_torsion_turns=0;
        if(s1.subseg.theta10!=0 && s1.subseg.theta20==0 && s1.subseg.theta14==0 && s1.subseg.theta24!=0){
            printf("registered extra torsion turns. clothoid fit's again. \n");
            if(s0->subseg.segment_type==LINE){
                fix_torsion_turns=1;
            }
        }

        if(fix_torsion_turns){
            s0->subseg.p1[0]+=0.001;
            s0->subseg.p1[1]+=0.001;
            init_line(s0->subseg.p0,s0->subseg.p1,&s0->subseg,1);
            init_inbetween_clothoid(&s0->subseg,&s2.subseg,&s1.subseg,-INFINITY,INFINITY);
        }
    }

    // Stop measuring time
    // clock_t end = clock();

    // Calculate elapsed time in milliseconds
    // double elapsed_time = (double)(end - start) / CLOCKS_PER_SEC * 1000;

    // printf("Clothoid fit, given trim distance : %.2f ms\n", elapsed_time);

    // Calculate feeds for the motion. Apply G93, G94, G95.
    init_feed(tp,s0);
    init_feed(tp,&s1);
    init_feed(tp,&s2);

    // Update trajectory length.
    s0->trajectory_length_end = s0->trajectory_length_begin + s0->subseg.length;
    s1.trajectory_length_begin = s0->trajectory_length_end;
    s1.trajectory_length_end = s1.trajectory_length_begin + s1.subseg.length;
    s2.trajectory_length_begin = s1.trajectory_length_end;
    s2.trajectory_length_end = s2.trajectory_length_begin + s2.subseg.length;
    path->trajectory_length=s2.trajectory_length_end;

    // Planner needs also emcmot_segment netto lenghts, used for gui position update.
    s0->length_netto = s0->subseg.length;
    s1.length_netto = s1.subseg.length;
    s2.length_netto = s2.subseg.length;

    // Curvature, kappa values. Curvature extrema for clothoid s1.
    kmax_arc_line(s0);
    s1.kmax = curvature_extrema(&s1.subseg);
    s1.radius = 1/s1.kmax;
    s1.canon_motion_type=0; // Set to 0 type so we can see the cloithoid in brown color in gui.

    kmax_arc_line(&s2);

    kmax_vel(s0);
    kmax_vel(&s1);
    kmax_vel(&s2);

    // Add new segments to queue.
    vector_push_back(ptr, s1); // Clothoid fillet.
    vector_push_back(ptr, s2); // New segment.

    // Print clothoid data.
    // print_clothoid_segment(&s1.subseg);

    return 0;
}

/* Interpolation for segments.
 * This interpolation is done by the clothoid library.
 * No external axis are added yet.
 */
static inline void path_clothoid_interpolate(TP_STRUCT * const tp,
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
        interpolate_abc_uvw(seg, path->progress,&abc_uvw_pi);
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
        interpolate_arc3d(&seg->subseg, path->progress, pi, &theta);
        tp->currentPos.tran.x=pi[0];
        tp->currentPos.tran.y=pi[1];
        tp->currentPos.tran.z=pi[2];

        // Interpolate extra axis.
        struct EmcPose abc_uvw_pi;
        interpolate_abc_uvw(seg, path->progress,&abc_uvw_pi);
        tp->currentPos.a=abc_uvw_pi.a;
        tp->currentPos.b=abc_uvw_pi.b;
        tp->currentPos.c=abc_uvw_pi.c;
        tp->currentPos.u=abc_uvw_pi.u;
        tp->currentPos.v=abc_uvw_pi.v;
        tp->currentPos.w=abc_uvw_pi.w;
    }

    // Interpolate if segment is clothoid.
    if (seg->subseg.segment_type == CLOTHOID) {

        interpolate_clothoid3d(&seg->subseg, path->progress, pi);
        tp->currentPos.tran.x=pi[0];
        tp->currentPos.tran.y=pi[1];
        tp->currentPos.tran.z=pi[2];

        // Interpolate extra axis.
        struct EmcPose abc_uvw_pi;
        interpolate_abc_uvw(seg, path->progress,&abc_uvw_pi);
        tp->currentPos.a=abc_uvw_pi.a;
        tp->currentPos.b=abc_uvw_pi.b;
        tp->currentPos.c=abc_uvw_pi.c;
        tp->currentPos.u=abc_uvw_pi.u;
        tp->currentPos.v=abc_uvw_pi.v;
        tp->currentPos.w=abc_uvw_pi.w;
    }
}

#endif // PATH_CLOTHOID_H













