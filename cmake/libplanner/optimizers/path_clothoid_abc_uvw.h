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
#ifndef PATH_CLOTHOID_ABC_UVW_H
#define PATH_CLOTHOID_ABC_UVW_H

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

// Function to determine if to lines are in opposite direction. Motion has to stop. No fillet.
int opposite_line_direction(const struct emcmot_segment *s0, const struct emcmot_segment *s2){

    if(s0->canon_motion_type==2 /* linear */ && s2->canon_motion_type==2 /* linear */ ){

        // Check for colinear and opposite direction:

        double vector_s0[3];    // 3d Vector s0.
        double s0_p1[3];        // Startpoint s0.
        double s0_p2[3];        // Endpoint s0.

        s0_p1[0] = s0->start.tran.x;
        s0_p1[1] = s0->start.tran.y;
        s0_p1[2] = s0->start.tran.z;

        s0_p2[0] = s0->end.tran.x;
        s0_p2[1] = s0->end.tran.y;
        s0_p2[2] = s0->end.tran.z;

        double vector_s2[3];    // 3d Vector s2.
        double s2_p1[3];        // Startpoint s2.
        double s2_p2[3];        // Endpoint s2.

        s2_p1[0] = s2->start.tran.x;
        s2_p1[1] = s2->start.tran.y;
        s2_p1[2] = s2->start.tran.z;

        s2_p2[0] = s2->end.tran.x;
        s2_p2[1] = s2->end.tran.y;
        s2_p2[2] = s2->end.tran.z;

        unit_vector(s0_p1, s0_p2, vector_s0);
        unit_vector(s2_p1, s2_p2, vector_s2);

        // If dot product -1
        double result = dot_product(vector_s0, vector_s2);
        if(fabs(result - (-1.0)) < 1e-8 ){
            // printf("lines are colinear and opposite direction. \n");
            return 1;
        }
    }
    return 0;
}

/* Add segments to buffer. Add clothoid fillets given the max deviation.
 *
 */
static inline int path_clothoid_abc_uvw(TP_STRUCT * const tp,
                                        struct path_data *path,
                                        struct emcmot_segment s2,
                                        struct vector *ptr){

    struct emcmot_segment *s0;
    struct emcmot_segment s1;

    // Check and set deviations. G64 P[..]
    if(s2.canon_motion_type==1 /* rapid */){
        //s2.tag.fields_float[3] = 0.0;
    }
    if(s2.canon_motion_type==4 /* rigid tap */ || s2.canon_motion_type==5 /* tool change */){
        //s2.tag.fields_float[3] = 0.0;
    }

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
        // kmax_vel(&s2);
        kmax_vel_factor(&s2, path->inertia_factor);
        init_feed(tp,&s2);

        s2.a_start = s2.start.a;
        s2.b_start = s2.start.b;
        s2.c_start = s2.start.c;
        s2.u_start = s2.start.u;
        s2.v_start = s2.start.v;
        s2.w_start = s2.start.w;

        s2.a_end = s2.end.a;
        s2.b_end = s2.end.b;
        s2.c_end = s2.end.c;
        s2.u_end = s2.end.u;
        s2.v_end = s2.end.v;
        s2.w_end = s2.end.w;

        vector_push_back(ptr, s2);
        return 0;
    }

    // Conditions to not add a clothoid fillet, but only add the end segment :
    if(     s0->canon_motion_type==1            // Previous motion is a rapid.
            || opposite_line_direction(s0, &s2) // Motion is colinear and in opposite direction.
            // Motion has to stop. Valid for canon motion type: 2.
            // *** So colinear arc's in opposite direction are not coded yet.
            || s2.subseg.length==0              // Segment has no length.
            || s2.tag.fields_float[3] < 1e-6    // No path deviation is given. -> G64 P0.0
            || s2.canon_motion_type==4          // Motion is off type: Rigid tap.
            || s2.canon_motion_type==5          // Motion is off type: Tool change.
            || s2.canon_motion_type==1          // Motion is off type: Rapid.
            ){
        s2.trajectory_length_begin = s0->trajectory_length_end;
        s2.trajectory_length_end = s2.trajectory_length_begin + s2.length_netto;
        path->trajectory_length=s2.trajectory_length_end;

        kmax_arc_line(&s2);
        // kmax_vel(&s2);
        kmax_vel_factor(&s2, path->inertia_factor);
        init_feed(tp,&s2);

        s2.a_start = s2.start.a;
        s2.b_start = s2.start.b;
        s2.c_start = s2.start.c;
        s2.u_start = s2.start.u;
        s2.v_start = s2.start.v;
        s2.w_start = s2.start.w;

        s2.a_end = s2.end.a;
        s2.b_end = s2.end.b;
        s2.c_end = s2.end.c;
        s2.u_end = s2.end.u;
        s2.v_end = s2.end.v;
        s2.w_end = s2.end.w;

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
    s1.start.u=s0->end.u;
    s1.start.v=s0->end.v;
    s1.start.w=s0->end.w;
    s1.end.a=s0->end.a;
    s1.end.b=s0->end.b;
    s1.end.c=s0->end.c;
    s1.end.u=s0->end.u;
    s1.end.v=s0->end.v;
    s1.end.w=s0->end.w;
    s1.tag=s2.tag;          // Copy G64 P[x]. Used to set endvel=0.
    s1.id=s2.id;            // Copy gcode line id.

    if(s1.tag.fields_float[3]==0){
        // printf("deveation error. \n");
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

    kmax_vel_factor(s0, path->inertia_factor);
    kmax_vel_factor(&s1, path->inertia_factor);
    kmax_vel_factor(&s2, path->inertia_factor);

    // When clothoid is a line, the kmax=0 and the radius is INF. The velocity value is ok.
    // printf("kmax: %f \n",s1.kmax);
    // printf("radius: %f \n",s1.radius);
    // printf("vel %f \n",s1.vel);
    // printf("length %f \n",s1.length_netto);

    // We need interpolation for abc, uvw axis to cover 100% of the trajectory.
    // After the fillets where made, we want to interpolate the abc, uvw axis also
    // over the fillet trajectory. Therefore we spread out the abc, uvw values to be
    // covering the fillets. The result is a interpolated peak value for
    // abc, uvw half way the fillet.

    double progress = 0;
    double l00 = 0;

    // 1. Fillet peak value. Copy the segment s0 end value to the fillet peak value. This is half way the fillet.
    s1.a_peak = s0->end.a;
    s1.b_peak = s0->end.b;
    s1.c_peak = s0->end.c;
    s1.u_peak = s0->end.u;
    s1.v_peak = s0->end.v;
    s1.w_peak = s0->end.w;

    // Get the previous prevous fillet segment, if it excists.
    struct emcmot_segment *s00;
    s00 = 0;
    if(push_counter(vector_ptr)>1){
        s00 = vector_at(vector_ptr, push_counter(vector_ptr)-2);
        if(s00 != NULL && s00->subseg.segment_type == CLOTHOID){
            l00 = s00->subseg.length;
        }
    }

    double ltot = 0;

    ltot = s0->subseg.length + (0.5*s1.subseg.length) + (0.5*l00);

    progress = (0.5*l00) / ltot;
    s0->a_start = s0->start.a + (s0->end.a - s0->start.a) * progress;
    s0->b_start = s0->start.b + (s0->end.b - s0->start.b) * progress;
    s0->c_start = s0->start.c + (s0->end.c - s0->start.c) * progress;
    s0->u_start = s0->start.u + (s0->end.u - s0->start.u) * progress;
    s0->v_start = s0->start.v + (s0->end.v - s0->start.v) * progress;
    s0->w_start = s0->start.w + (s0->end.w - s0->start.w) * progress;

    progress = (s0->subseg.length+(0.5*l00)) / ltot;
    s0->a_end = s0->start.a + (s0->end.a - s0->start.a) * progress;
    s0->b_end = s0->start.b + (s0->end.b - s0->start.b) * progress;
    s0->c_end = s0->start.c + (s0->end.c - s0->start.c) * progress;
    s0->u_end = s0->start.u + (s0->end.u - s0->start.u) * progress;
    s0->v_end = s0->start.v + (s0->end.v - s0->start.v) * progress;
    s0->w_end = s0->start.w + (s0->end.w - s0->start.w) * progress;

    if(s00 != NULL){
        s00->a_end = s0->a_start;
        s00->b_end = s0->b_start;
        s00->c_end = s0->c_start;
        s00->u_end = s0->u_start;
        s00->v_end = s0->v_start;
        s00->w_end = s0->w_start;
    }

    s1.a_start = s0->a_end;
    s1.b_start = s0->b_end;
    s1.c_start = s0->c_end;
    s1.u_start = s0->u_end;
    s1.v_start = s0->v_end;
    s1.w_start = s0->w_end;

    ltot = s2.subseg.length + (0.5*s1.subseg.length) ;
    progress = (0.5*s1.subseg.length) / ltot;

    s1.a_end = s2.start.a + (s2.end.a - s2.start.a) * progress;
    s2.a_start = s1.a_end;
    s2.a_end = s2.end.a;

    s1.b_end = s2.start.b + (s2.end.b - s2.start.b) * progress;
    s2.b_start = s1.b_end;
    s2.b_end = s2.end.b;

    s1.c_end = s2.start.c + (s2.end.c - s2.start.c) * progress;
    s2.c_start = s1.c_end;
    s2.c_end = s2.end.c;

    s1.u_end = s2.start.u + (s2.end.u - s2.start.u) * progress;
    s2.u_start = s1.u_end;
    s2.u_end = s2.end.u;

    s1.v_end = s2.start.v + (s2.end.v - s2.start.v) * progress;
    s2.v_start = s1.v_end;
    s2.v_end = s2.end.v;

    s1.w_end = s2.start.w + (s2.end.w - s2.start.w) * progress;
    s2.w_start = s1.w_end;
    s2.w_end = s2.end.w;

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
static inline void path_clothoid_interpolate_abc_uvw(TP_STRUCT * const tp,
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
        interpolate_abc_uvw_(seg, path->progress,&abc_uvw_pi);
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
        interpolate_abc_uvw_(seg, path->progress,&abc_uvw_pi);
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

        // Interpolate extra axis with integrated fillet peak value.
        struct EmcPose abc_uvw_pi;
        interpolate_abc_uvw_peak(seg, path->progress,&abc_uvw_pi);
        tp->currentPos.a=abc_uvw_pi.a;
        tp->currentPos.b=abc_uvw_pi.b;
        tp->currentPos.c=abc_uvw_pi.c;
        tp->currentPos.u=abc_uvw_pi.u;
        tp->currentPos.v=abc_uvw_pi.v;
        tp->currentPos.w=abc_uvw_pi.w;
    }
}

#endif // PATH_CLOTHOID_ABC_UVW_H













