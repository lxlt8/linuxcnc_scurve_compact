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
#ifndef EMCMOT_SEGMENT_H
#define EMCMOT_SEGMENT_H

#include "posemath.h"		/* PmCartesian, PmPose, pmCartMag() */
#include "emcpos.h"		    /* EmcPose */
#include "cubic.h"		    /* CUBIC_STRUCT, CUBIC_COEFF */
#include "emcmotcfg.h"		/* EMCMOT_MAX_JOINTS */
#include "kinematics.h"
#include "simple_tp.h"
#include "rtapi_limits.h"
#include <stdarg.h>
#include "rtapi_bool.h"
#include "state_tag.h"
#include "tp_types.h"
#include <string.h>  // For memset
#include <math.h>
#include "segment.h"

enum enum_arc_helix_dir {
    dir_none,
    dir_G2,
    dir_G3
};

enum enum_motion_set {
    xyz,
    abc,
    uvw
};

struct emcmot_segment {

    EmcPose start, end;
    PmCartesian center;
    PmCartesian normal;

    // Fillets have a half way peak value.
    double a_start, b_start, c_start, u_start, v_start, w_start;
    double a_end, b_end, c_end, u_end, v_end, w_end;
    double a_peak, b_peak, c_peak, u_peak, v_peak, w_peak;

    /* Canon motion type:
     * 0 = clothoid
     * 1  = rapid
     * 2  = linear
     * 3  = arc
     * 4  = tool_change
     * 5  = rigid_tap
     * 11 = ..
     */
    int canon_motion_type;
    double vel; // mm/s
    double ini_maxvel; // mm/s. used for G0.
    double acc; // mm/s^2
    unsigned char enables;
    char atspeed;
    int indexer_jnum;
    int turn; // See tpAddCircle for more info.
    int consistent_turn; // Consistent helix turns.
    enum enum_arc_helix_dir arc_helix_dir;
    double scale; // Rigrid tap value, faster return move scale.
    double pitch; // Rigrid tap value, revolutions per mm, or pitch.
    int spindle;  // Rigrid tap value, spindle nr to use. Staring at 0.
    double radius;

    double kmax; // Curvature.

    struct state_tag_t tag;
    // tag.fields_float[x].
    // [0] = Gcode line nr.
    // [1] = Feed = original unmodified gcode FEED in mm/min.
    // [2] = Speed = 0
    // [3] = G64 P[x] Path max deviation value.
    // [4] = G64 Q[x] Tollerance segment lenght filter.

    // tag.fields[x].
    // [4] = Plane.
    // G17, xy, [170].
    // G18, zx, [180].
    // G19, yz, [190].

    int id; // Gcode line nr.

    union {                 // describes the segment's start and end positions
        PmLine9 line;
        PmCircle9 circle;
        PmRigidTap rigidtap;
        Arc9 arc;
    } coords;

    // Used for the planner's interpolation and scurve.
    double length_xyz, length_abc, length_uvw;
    double length_netto;
    double trajectory_length_begin, trajectory_length_end;

    // When a new segment is loaded, determine wich motion xyz, abc, uvw takes longest time.
    enum enum_motion_set motion_set; // 0=xyz, 1=abc, 2=uvw.

    // Look ahead.
    // length_netto, acc, vel, Kmax is already above.
    double vo, ve;

    // Used by clothoid library to create fillets.
    struct segment subseg;
};

/* Calculate length for abc, uvw motions.
 * This is the magnitude of abc & uvw vectors.
 */
static void abc_uvw_lenght(struct emcmot_segment *seg){

    double v_abc[3]={seg->end.a - seg->start.a, seg->end.b - seg->start.b, seg->end.c - seg->start.c};
    seg->length_abc = magnitude_xyz(v_abc);

    double v_uvw[3]={seg->end.u - seg->start.u, seg->end.v - seg->start.v, seg->end.w - seg->start.w};
    seg->length_uvw = magnitude_xyz(v_uvw);

    // printf("lenght abc motion: %f \n",seg->length_abc);
    // printf("lenght uvw motion: %f \n",seg->length_uvw);
}

static void print_emcmot_segment_id_length_vo_vm_ve(struct emcmot_segment *seg) {
    printf("id: %d lenght: %f vo: %f vm: %f ve: %f \n",seg->id, seg->length_netto, seg->vo, seg->vel, seg->ve);
}

static void kmax_arc_line (struct emcmot_segment *seg){

    if(seg->subseg.segment_type==LINE){
        seg->kmax=0;
    }
    if(seg->subseg.segment_type==ARC){
        seg->kmax = 1/seg->subseg.radius;
        seg->radius = seg->subseg.radius;
    }
}

static void kmax_vel(struct emcmot_segment *seg){

    // If kmax is zero, the motion is a straight line.
    if (seg->kmax == 0) {
        return;
    }

    // Calculate maximum velocity based on maximum acceleration and kmax:
    // v_max = sqrt(maxacc / kmax)
    // Explanation:
    // - The centripetal acceleration required to maintain a curved path is given by the formula:
    //   a_c = v^2 * kmax
    // - Rearranging this gives us: v^2 = a_c / kmax
    // - To find the maximum velocity before exceeding maxacc, we set a_c = maxacc:
    //   v^2 = maxacc / kmax
    // - Finally, taking the square root gives us the maximum velocity:
    //   v_max = sqrt(maxacc / kmax)
    double computed_maxvel = sqrt(seg->acc / seg->kmax); // mm/min.

    // Return the minimum of the calculated max velocity and the input max velocity limit
    seg->vel = fmin(computed_maxvel, seg->vel);
}

// Functions to zero a segment.
static void zero_emc_pose(EmcPose *pos){
    pos->tran.x=0;
    pos->tran.y=0;
    pos->tran.z=0;
    pos->a=0;
    pos->b=0;
    pos->c=0;
    pos->u=0;
    pos->v=0;
    pos->w=0;
}

static void zero_cartesian(PmCartesian *pos){
    pos->x=0;
    pos->y=0;
    pos->z=0;
}

static void zero_state_tag(struct state_tag_t* tag) {
    if (!tag) return;

    // Zero out the floating-point fields
    memset(tag->fields_float, 0, sizeof(tag->fields_float));

    // Zero out the integer fields
    memset(tag->fields, 0, sizeof(tag->fields));

    // Zero the packed flags
    tag->packed_flags = 0;

    // Zero the filename (up to 256 characters)
    // memset(tag->filename, 0, sizeof(tag->filename));
}

// Function to zero out all coordinates in the segment
static void zero_coords(struct emcmot_segment** seg) {
    if (!seg || !*seg) return; // Check if seg is NULL or the pointed segment is NULL

    // Zero out the entire coords union
    memset(&(*seg)->coords, 0, sizeof((*seg)->coords));
}

static void zero_emcmot_segment(struct emcmot_segment *seg) {
    if (!seg) return;

    zero_emc_pose(&seg->start);
    zero_emc_pose(&seg->end);

    zero_cartesian(&seg->center);
    zero_cartesian(&seg->normal);

    seg->canon_motion_type = 0;
    seg->vel = 0.0;
    seg->ini_maxvel = 0.0;
    seg->acc = 0.0;
    seg->enables = 0;
    seg->atspeed = 0;
    seg->indexer_jnum = 0;
    seg->turn = 0;
    seg->arc_helix_dir = dir_none;
    seg->scale = 0.0;
    seg->pitch = 0.0;
    seg->spindle = 0.0;
    seg->radius = 0.0;

    seg->kmax = 0.0;

    zero_coords(&seg);
    zero_state_tag(&seg->tag);

    seg->id = 0;

    seg->length_xyz = 0;

    seg->trajectory_length_begin = 0;
    seg->trajectory_length_end = 0;

    seg->vo=0;
    seg->ve=0;

    seg->length_abc = 0;
    seg->length_uvw = 0;

    seg->length_netto = 0;
}

#endif // EMCMOT_SEGMENT_H
