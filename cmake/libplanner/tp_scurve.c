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
#include "rtapi.h"              /* rtapi_print_msg */
#include "posemath.h"           /* Geometry types & functions */
#include "emcpose.h"
#include "rtapi_math.h"
#include "motion.h"
#include "tp.h"
#include "tp_scurve.h"
#include "tc.h"
#include "motion_types.h"
#include "spherical_arc.h"
#include "blendmath.h"
#include "axis.h"

#include <stdio.h>
#include "vector.h"
#include "look_ahead.h"
#include <time.h>
#include "scurve.h"
#include "curve.h"
#include "interpolate.h"
#include "halsection.h"
#include "common.h"
#include "module.h"
#include "arc.h"

// Clothoid includes.
#include "fit.h"
#include "clothoid3d.h"

// Trajectory optimalisation algo includes :
#include "path_standard.h"
#include "path_standard_subseg.h"
#include "path_standard_subseg_line_fillet.h"
#include "path_clothoid.h"
#include "path_clothoid_abc_uvw.h"

// Key capture for jog in pause.
#include "keyboard.h"

struct timespec start_0, end_0;
struct timespec start_1, end_1;

// Start the planner. Make connections to scurve lib etc.
int tpCreate(TP_STRUCT * const tp, int _queueSize,int id){

    // printf("tpCreate. \n");

    // Create pointers to c++.
    vector_ptr = vector_init_ptr();
    // motionizer_ptr = motionizer_init_ptr();

    // Example for clearing content.
    vector_clear(vector_ptr);
    scurve_reset_data(&sc_data);
    path_reset(&path);

    path.ringbuffer_index=0;
    path.motion_enable=1;
    path.max_jerk=50;
    path.cycle=0;
    path.use_real_deviation=0;

    path.jog_x_min=0;
    path.jog_x_plus=0;
    path.jog_y_min=0;
    path.jog_y_plus=0;
    path.jog_z_min=0;
    path.jog_z_plus=0;
    path.must_jog_back=0;

    // Set the hal pins, params to above path values at startup.
    set_hal_path_values(&path);

    tp->done=0;
    tp->aborting=0;

    // For sure.
    zero_emc_pose(&tp->currentPos);

    return 1;
}

// Add segments to the vector.
static inline int tpProcessSegment(TP_STRUCT * const tp,
                                   struct path_data *path,
                                   struct emcmot_segment seg){

    int res=0;

    // Note: now includes motions for abc, uvw.
    // path->path_algo=file_path_standard;
    // path->path_algo=file_path_standard_subseg;
    // path->path_algo=file_path_standard_subseg_line_fillet;
    path->path_algo=file_path_clothoid_abc_uvw;

    if(path->path_algo==file_path_standard){
        res=path_standard(tp,path,seg,vector_ptr);
        // printf("using standard. \n");
    }
    if(path->path_algo==file_path_standard_subseg){
        res=path_standard_subseg(tp,path,seg,vector_ptr);
        // printf("using standard. \n");
    }
    if(path->path_algo==file_path_standard_subseg_line_fillet){
        res=path_standard_subseg_line_fillet(tp,path,seg,vector_ptr);
        // printf("using standard. \n");
    }
    if(path->path_algo==file_path_clothoid){
        res=path_clothoid(tp,path,seg,vector_ptr);
        // printf("using clothoid. \n");
    }
    // Todo: use the new interpolation model.
    if(path->path_algo==file_path_clothoid_abc_uvw){
        res=path_clothoid_abc_uvw(tp,path,seg,vector_ptr);
        // printf("using clothoid. \n");
    }

    return res;
}

// Add a line segment to the vector.
int tpAddLine(TP_STRUCT * const tp, EmcPose end, int canon_motion_type,
              double vel, double ini_maxvel, double acc, unsigned char enables,
              char atspeed, int indexer_jnum, struct state_tag_t tag){

    // printf("tpAddLine, vector size: %d \n",vector_size(vector_ptr));

    struct emcmot_segment s2;
    zero_emcmot_segment(&s2);
    s2.start=tp->goalPos;
    s2.end=end;
    s2.canon_motion_type=canon_motion_type; // 1=rapid, 2=linear, 3=arc, 4=tool_change.

    // Toolchange motion type.
    if(canon_motion_type==4){
        printf("ptAddLine, added motion type 4 Tool change. \n");
    }

    s2.vel=vel;
    s2.ini_maxvel=ini_maxvel;
    s2.acc=acc;
    s2.enables=enables;
    s2.atspeed=atspeed;
    s2.indexer_jnum=indexer_jnum;
    s2.tag=tag;

    s2.id=tp->execId; // Attach gcode line nr.

    // Update next segment's startpoint.
    tp->goalPos=s2.end;

    return tpProcessSegment(tp,&path,s2);
}

// Add a circle to the vector.
int tpAddCircle(TP_STRUCT * const tp,
                EmcPose end,
                PmCartesian center,
                PmCartesian normal,
                int turn,
                int canon_motion_type,
                double vel,
                double ini_maxvel,
                double acc,
                unsigned char enables,
                char atspeed,
                struct state_tag_t tag){

    // printf("tpAddCircle. \n");

    // Create a segment from given circle values.
    struct emcmot_segment seg;
    zero_emcmot_segment(&seg);
    seg.start=tp->goalPos;
    seg.end=end;
    seg.center=center;
    seg.normal=normal;
    seg.turn=turn; //  >= 0 = G3, < 0 = G2.
    seg.canon_motion_type=canon_motion_type;
    seg.vel=vel;
    seg.ini_maxvel=ini_maxvel;
    seg.acc=acc;
    seg.enables=enables;
    seg.atspeed=atspeed;
    seg.tag=tag;
    seg.id=tp->execId; // Attach gcode line nr.

    // Update next segment's startpoint.
    tp->goalPos=seg.end;

    /* Information about G2, G3, helix turns from interpreter.
     * If you ask me, this is not the way it should be.
     *
     * G2.. P0 = not valid.
     * G2.. P1 = Arc as is. [turn=-1]
     * G2.. P2 = +1 turn    [turn=-2]
     *
     * G3.. P1 = Arc as is. [turn=0]
     * G3.. P2 = +1 turn    [turn=1]
     *
     * Then the arc center.z is aligned with the arc end.z in linuxcnc.
     * In the clothoid lib, the arc center.z is aligned with the arc start.z
     */

    // Save arc-helix direction in the segment struct.
    if(turn<0){
        seg.arc_helix_dir=dir_G2;
        // printf("arc-helix dir G2. \n");
    } else {
        seg.arc_helix_dir=dir_G3;
        // printf("arc-helix dir G3. \n");
    }

    // Turn to positive value. 1 extra helix turn = 1.
    if(turn<0){
        seg.consistent_turn = fabs(turn)-1;
    } else {
        seg.consistent_turn = turn;
    }

    // printf("normalized arc-helix turn: %d \n",seg.consistent_turn);

    // return 0;
    // return process_line_or_arc_segment(tp,s2);
    return tpProcessSegment(tp,&path,seg);
}

/* G33.1 X- Y- Z- K- I- $-
    K - distance per revolution
    I - optional spindle speed multiplier for faster return move
    $ - optional spindle selector

    This is the only active segment during runtime. No other segments are in the vector.
    Therefore i added a static segment called : seg_tab
    After this tpAddRigidTap function is executed, the spindle is turned on by other process.

    When at tap depth, to reverse the spindle a call to emcmot status channel is
    done. A request to a status channel is fundamentally wrong to execute a command.
    So don't blame me for that. It is what it is.
*/
int tpAddRigidTap(TP_STRUCT * const tp,
                  EmcPose end,
                  double vel,
                  double ini_maxvel,
                  double acc,
                  unsigned char enables,
                  double scale,             // Gcode I, faster return move scale.
                  struct state_tag_t tag){

    // printf("tpAddRigidTap. \n");

    // We have a permanent struct for tapping "seg_tap".
    zero_emcmot_segment(&seg_tap);
    seg_tap.start=tp->goalPos;
    seg_tap.end=end;
    seg_tap.canon_motion_type=5;                // 1=rapid, 2=linear, 3=arc, 4=toolchange, 5=rigidtap, 10=spline
    seg_tap.vel=vel;                            // Used for motion.
    seg_tap.ini_maxvel=ini_maxvel;
    seg_tap.acc=acc;
    seg_tap.enables=enables;
    seg_tap.atspeed=0;
    seg_tap.indexer_jnum=0;
    seg_tap.tag=tag;
    seg_tap.id=tp->execId;                      // Attach gcode line nr.
    seg_tap.scale=scale;                        // Gcode I, faster return move scale.
    seg_tap.pitch=tp->uu_per_rev;               // emcmotCommand->spindlesync;  // Gcode K value, distance per revolution or pitch.
    seg_tap.spindle=tp->spindle.spindle_num;    // emcmotCommand->spindle;      // Gcode $ value, spindle nr. to use.

    // The tap start is the tap end position. No position update for next segment.
    // tp->goalPos=s2.end;

    zero_tap_struct(tap);
    tap.tapping=1;
    tap_state=tap_init;

    return 0;
}

// Create a empty queue. Not used.
int tpInit(TP_STRUCT * const tp)
{
    printf("tpInit. \n");
    return 0;
}

// Update the gui's distance to go. Diff current emcPose to Target emcPose
void update_gui_dtg_emcpose(TP_STRUCT * const tp,
                            struct path_data *path){

    // Exit if vector is empty.
    if (vector_size(vector_ptr)==0) {
        return;
    }

    emcmotStatus->dtg.tran.x = vector_at(vector_ptr,path->ringbuffer_index)->end.tran.x - tp->currentPos.tran.x;
    emcmotStatus->dtg.tran.y = vector_at(vector_ptr,path->ringbuffer_index)->end.tran.y - tp->currentPos.tran.y;
    emcmotStatus->dtg.tran.z = vector_at(vector_ptr,path->ringbuffer_index)->end.tran.z - tp->currentPos.tran.z;

    if(path->path_algo==file_path_clothoid_abc_uvw){
        emcmotStatus->dtg.a =  fabs(vector_at(vector_ptr,path->ringbuffer_index)->a_end - tp->currentPos.a);
        emcmotStatus->dtg.b = fabs(vector_at(vector_ptr,path->ringbuffer_index)->b_end - tp->currentPos.b);
        emcmotStatus->dtg.c =  fabs(vector_at(vector_ptr,path->ringbuffer_index)->c_end - tp->currentPos.c);

        emcmotStatus->dtg.u =  fabs(vector_at(vector_ptr,path->ringbuffer_index)->u_end - tp->currentPos.u);
        emcmotStatus->dtg.v =  fabs(vector_at(vector_ptr,path->ringbuffer_index)->v_end - tp->currentPos.v);
        emcmotStatus->dtg.w =  fabs(vector_at(vector_ptr,path->ringbuffer_index)->w_end - tp->currentPos.w);
    } else {
        emcmotStatus->dtg.a = vector_at(vector_ptr,path->ringbuffer_index)->end.a - tp->currentPos.a;
        emcmotStatus->dtg.b = vector_at(vector_ptr,path->ringbuffer_index)->end.b - tp->currentPos.b;
        emcmotStatus->dtg.c = vector_at(vector_ptr,path->ringbuffer_index)->end.c - tp->currentPos.c;

        emcmotStatus->dtg.u = vector_at(vector_ptr,path->ringbuffer_index)->end.u - tp->currentPos.u;
        emcmotStatus->dtg.v = vector_at(vector_ptr,path->ringbuffer_index)->end.v - tp->currentPos.v;
        emcmotStatus->dtg.w = vector_at(vector_ptr,path->ringbuffer_index)->end.w - tp->currentPos.w;
    }
}

/* Rigid tap cycle.
*
*  Concern's. If acceleration is set too slow, the tap acceleration, deceleration distance can
*  so big, that the tap won't be synchronized during tapping.
*  We could add a hal out pin to visualize when in acc or dcc stage.
*
*  Follow the case's, to find out how this sequence is programmed.
*/
inline int tpTapCycle(TP_STRUCT * const tp){

    tap.acc=tp->aMax; // *TRAPEZIUM_ACC_SCALE;

    switch (tap_state) {
    case tap_init:
        // Gcode hole depth is positive value.
        tap.hole_depth=fabs(seg_tap.end.tran.z - seg_tap.start.tran.z);
        tap.pitch=seg_tap.pitch;
        tap.return_scale=seg_tap.scale;
        tap.spindle_nr=seg_tap.spindle;
        tap_state=wait_spindle_at_speed;
        break;
    case wait_spindle_at_speed:
        // Wait for spindle at speed. Spindle speed in rpm.
        if(!emcmotStatus->spindle_status[seg_tap.spindle].at_speed){
            return 0;
        } else {
            tap.feed_fwd=(emcmotStatus->spindle_status[seg_tap.spindle].speed/60) * seg_tap.pitch;
            // For the reverse move, the velocity is faster if scale is set > 1.
            tap.feed_rev = tap.feed_fwd * tap.return_scale;
            // Stop length reverse move.
            tap.rev_stop_dist = (tap.feed_rev*tap.feed_rev) / (2*tap.acc);
            // printf("tap stop dist: %f \n",tap.rev_stop_dist);
            tap_state=wait_spindle_at_position;
        }
        break;
    case wait_spindle_at_position:
        // When spindle is running, wait for spindle_rev's passes 0 degree. Or decimal=0.
        if (fabs(fmod( emcmotStatus->spindle_status[seg_tap.spindle].spindleRevs, 1.0)) < 0.1) {
            // printf("spindle in position: %f \n",spindle_rev);
            tap_state=z_axis_acc_stage;
        } else {
            return 0;
        }
        break;
    case z_axis_acc_stage:
        // Run the z axis up to tap feed. If at feed, save current spindle revolutions as reference.
        tap.z_vel = tap.z_vel + tap.acc * tp->cycleTime; // V_next = V_current + a * t.
        emcmotStatus->current_vel=path.curvel=tap.z_vel;

        // P_next = P_currrent + V_current * t + 0.5 * a * t^2.
        tap.z_acc_pos = tap.z_acc_pos + tap.z_vel * tp->cycleTime + 0.5 * tap.acc * tp->cycleTime * tp->cycleTime;
        tap.z_pos=tap.z_acc_pos;
        tap.z_pos_prev=tap.z_pos;
        tp->currentPos.tran.z=seg_tap.start.tran.z-tap.z_pos;

        emcmotStatus->distance_to_go=tap.hole_depth-tap.z_acc_pos;
        emcmotStatus->dtg.tran.z =  emcmotStatus->distance_to_go;

        if(tap.z_vel>=tap.feed_fwd){
            tap.z_vel=tap.feed_fwd;
            tap.start_turns=emcmotStatus->spindle_status[seg_tap.spindle].spindleRevs;
            tap_state=tap_fwd;
            // printf("end acc stage, pos: %f ve: %f \n",tap.z_acc_pos, tap.z_vel);
        }
        return 0;
    case tap_fwd:
        // Z axis follows spindle revolutions. Tapping cw.
        tap.turns = emcmotStatus->spindle_status[seg_tap.spindle].spindleRevs - tap.start_turns;
        tap.z_pos = (tap.turns * tap.pitch) + tap.z_acc_pos;
        tp->currentPos.tran.z=seg_tap.start.tran.z-tap.z_pos;

        emcmotStatus->current_vel=path.curvel=tap.z_vel=(tap.z_pos-tap.z_pos_prev)/tp->cycleTime; // Vel = (S_cur - S_prev) / t
        tap.z_pos_prev=tap.z_pos;

        emcmotStatus->distance_to_go=tap.hole_depth-tap.z_pos;
        emcmotStatus->dtg.tran.z =  emcmotStatus->distance_to_go;

        // Reverse spindle and retract.
        if(tap.z_pos>=tap.hole_depth){
            emcmotStatus->spindle_status[seg_tap.spindle].speed=-fabs(emcmotStatus->spindle_status[seg_tap.spindle].speed * tap.return_scale);
            // printf("tap bottom pos, pos: %f ve: %f \n",tap.z_pos, tap.z_vel);
            tap_state=tap_rev;
        }
        return 0;
    case tap_rev:
        // Z axis follows spindle revolutions. Tapping ccw.
        tap.turns = emcmotStatus->spindle_status[seg_tap.spindle].spindleRevs - tap.start_turns;
        tap.z_pos = (tap.turns * tap.pitch) + tap.z_acc_pos;
        tp->currentPos.tran.z=seg_tap.start.tran.z-tap.z_pos;

        emcmotStatus->current_vel=path.curvel=tap.z_vel=(tap.z_pos-tap.z_pos_prev)/tp->cycleTime; // Vel = (S_cur - S_prev) / t
        tap.z_pos_prev=tap.z_pos;

        emcmotStatus->distance_to_go=tap.z_pos;
        emcmotStatus->dtg.tran.z =  emcmotStatus->distance_to_go;

        // End reverse tap stage, need dcc z axis stage now.
        if(tap.z_pos <= tap.rev_stop_dist){
            // printf("end tap rev stage, pos: %f ve: %f \n",tap.z_pos, tap.z_vel);
            tap_state=z_axis_dcc_stage;
        }
        return 0;
    case z_axis_dcc_stage:
        // Z axis deceleration stage.
        tap.z_vel = tap.z_vel + tap.acc * tp->cycleTime; // V_next = V_current + a * t.
        tap.z_pos = tap.z_pos + tap.z_vel * tp->cycleTime + 0.5 * tap.acc * tp->cycleTime * tp->cycleTime;

        emcmotStatus->distance_to_go=tap.z_pos;
        emcmotStatus->dtg.tran.z =  emcmotStatus->distance_to_go;

        if(tap.z_vel>=0){
            // printf("tap cycle finished, pos: %f ve: %f \n",tap.z_pos, tap.z_vel);
            tap.z_vel=0; // Here we set absolute end value's.
            tap.z_pos=0;
            tap_state=tap_finished;
        }
        tp->currentPos.tran.z=seg_tap.start.tran.z-tap.z_pos;
        emcmotStatus->current_vel=path.curvel=tap.z_vel;
        return 0;
    case tap_finished:
        emcmotStatus->spindle_status[0].speed=0;
        emcmotStatus->spindle_status[0].direction=0;
        emcmotStatus->spindle_status[0].brake=1;
        tap.tapping=0;
        // printf("tap finished. \n");
        return 0;
    default:
        break;
    }

    return 0;
}

void increment_index(struct path_data *path){

    path->ringbuffer_index++;

    // Upper limit.
    path->ringbuffer_index=fmin(path->ringbuffer_index, push_counter(vector_ptr)-1);
    // Synchronise with rotating buffer index.
    path->ringbuffer_index = path->ringbuffer_index % VECTOR_BUFFER_SIZE;

    // Increment motion.
    path->global_index++;

    // Reset for reverse motion.
    path->reverse_motion_reset=0;
    path->reverse_motion_back_count=0;
}

void decrement_index(struct path_data *path){
    // When switched to motion reverse, get the motion count to go back.
    if(!path->reverse_motion_reset){
        // Push counter & global index tell us how far we can go back.
        path->reverse_motion_back_count = push_counter(vector_ptr) - path->global_index;
        path->reverse_motion_reset=1;
    }

    // Go back, we have counts left.
    if(path->reverse_motion_back_count>0){
        path->reverse_motion_back_count--;
    }
    // At max back, cancel.
    if(path->reverse_motion_back_count==0){
        return;
    }

    // Decrement motion.
    path->ringbuffer_index--;

    // Here it needs a full ringbuffer rotation.
    if(path->ringbuffer_index<0){
        path->ringbuffer_index=VECTOR_BUFFER_SIZE-1;
    }

    // Decrement global index.
    path->global_index--;
    // Lower limit global index = 0.
    path->global_index=fmax(0,path->global_index);
}

/* Adaptive feed can run motion forward or motion reverse.
 * Forward halpin : motion.adaptive-feed = 1
 * Reverse halpin : motion.adaptive-feed = -1
 */
void tpSetNextMotion(TP_STRUCT * const tp,
                     struct path_data *path){

    if(vector_size(vector_ptr)<2){
        return;
    }

    if(!tp->reverse_run){ // Motion forward.

        increment_index(path);
        path->motion_increments_a_cycle = 1;
        path->buffer_overflow=0;

        int count=0;
        while(path->curpos > vector_at(vector_ptr, path->ringbuffer_index)->trajectory_length_end){
            increment_index(path);

            // Add a limit to prevent lcnc from hanging.
            count++;
            if(count>VECTOR_BUFFER_SIZE){
                printf("tpSetNextMotion, buffer overflow. \n");
                path->buffer_overflow=1;
                break;
            }
            // printf("Warning, extra segments added: %d \n",count);
            path->motion_increments_a_cycle++;
        }

        // Update the path trajectory lenght.
        if(push_counter(vector_ptr)>0){
            int ringbuffer_last_index = push_counter(vector_ptr)-1;
            ringbuffer_last_index = ringbuffer_last_index % VECTOR_BUFFER_SIZE;
            path->trajectory_length = vector_at(vector_ptr,ringbuffer_last_index)->trajectory_length_end;
            // printf("last index: %d path trajectory length end: %f \n",ringbuffer_last_index,path->trajectory_length);
        }

    } else { // Motion reverse. In combination with the ringbuffer this is tricky.
        decrement_index(path);
    }
}

// Given the current trajectory position, interpolate the trajectory's xyz positions.
void tpUpdateGui(TP_STRUCT * const tp,
                 struct path_data *path) {

    // Exit if no segments are in the ringbuffer.
    if (push_counter(vector_ptr)==0) {
        return;
    }

    // Prevent a gui position jump when trajectory is finished and ringbuffer still
    // contains position data.
    if(path->curpos>=path->trajectory_length){
        return;
    }

    int i=path->ringbuffer_index;
    struct emcmot_segment *segment = vector_at(vector_ptr, i);

    if(segment==NULL){
        printf("segment error. \n");
        return;
    }

    double length_end = 0;
    double length_begin = 0;

    length_begin = vector_at(vector_ptr, i)->trajectory_length_begin;
    length_end = vector_at(vector_ptr, i)->trajectory_length_end;
    double length = vector_at(vector_ptr, i)->length_netto;

    path->progress = (path->curpos - length_begin) / length;  // Calculate progress
    path->progress = fmax(path->progress,0);
    path->progress = fmin(path->progress,1);

    // Choose interpolation model.
    if(path->path_algo==file_path_standard){
        path_standard_interpolate(tp,path,segment);
    } else
        if(path->path_algo==file_path_standard_subseg){
            path_standard_subseg_interpolate(tp,path,segment);
        } else
            if(path->path_algo==file_path_standard_subseg_line_fillet){
                path_standard_subseg_line_fillet_interpolate(tp,path,segment);
            } else
                if(path->path_algo==file_path_clothoid){
                    path_clothoid_interpolate(tp,path,segment);
                } else
                    if(path->path_algo==file_path_clothoid_abc_uvw){
                        path_clothoid_interpolate_abc_uvw(tp,path,segment);
                    } else {
                        printf("no interpolation model to update gui found! Abort. \n");
                    }

    /* // Update radius to path -> hal pin for plasma's
     * Canon motion type:
     * 0  = clothoid
     * 1  = rapid
     * 2  = linear
     * 3  = arc
     * 4  = tool_change
     * 5  = rigid_tap
     * 11 = ..
     */
    if(segment->canon_motion_type==3){
        path->segment_radius = segment->radius;
    }
    if(segment->canon_motion_type==1
            || segment->canon_motion_type==2
            || segment->canon_motion_type==4
            || segment->canon_motion_type==5 ){
        path->segment_radius = 0;
    }
    if(segment->canon_motion_type==0){
        path->segment_radius = 1/segment->kmax; // radius = 1 / kmax.
    }

    // Set a motion type so that : control.c Line 1867 motion.feed-... is not flippering
    // and has a motion type all the time.
    emcmotStatus->motionType = EMC_MOTION_TYPE_FEED;

    // Update GUI.
    emcmotStatus->current_vel = path->curvel;
    emcmotStatus->distance_to_go = length_end - path->curpos;
    update_gui_dtg_emcpose(tp,path);

    if(segment->canon_motion_type!=10){
        tp->motionType=segment->canon_motion_type;
        tp->execTag=segment->tag;
    }

    // Update tangential knife.
    path->tangential_knife_current_pos[0] = tp->currentPos.tran.x;
    path->tangential_knife_current_pos[1] = tp->currentPos.tran.y;
    path->tangential_knife_current_pos[2] = tp->currentPos.tran.z;

    // Calculate the 3d vector.
    double tangential_vector[3];
    vector_3d(path->tangential_knife_previous_pos,
              path->tangential_knife_current_pos,
              tangential_vector);

    double magnitude = magnitude_xy(tangential_vector);
    if(magnitude==0){
        // Nothing to do. Vector has no length.
    } else {
        normalize(tangential_vector);
        double x = tangential_vector[0];
        double y = tangential_vector[1];

        // Get raw angle (-180° to 180°)
        double current_angle = atan2(y, x) * toDegrees;

        // Angular unwinding: Ensure no sudden jumps
        double angle_diff = current_angle - path->tangential_previous_angle;
        if (angle_diff > 180.0) {
            angle_diff -= 360.0;  // Wrap backward
        } else if (angle_diff < -180.0) {
            angle_diff += 360.0;  // Wrap forward
        }

        // Update the persistent angle
        path->tangential_previous_angle += angle_diff;

        // Store the smooth angle (can go beyond ±360°)
        path->tangential_knife_angle = path->tangential_previous_angle;

        // printf("Tangential knife angle (continuous): %.2f°\n", path->tangential_knife_angle);
    }

    // Update old pos.
    path->tangential_knife_previous_pos[0] = tp->currentPos.tran.x;
    path->tangential_knife_previous_pos[1] = tp->currentPos.tran.y;
    path->tangential_knife_previous_pos[2] = tp->currentPos.tran.z;
}

// From hal we retrieve the adaptive feed value.
// Adaptive feed value : -1 up to 1.
// This indicates if we are in reverse run mode or not.
void tpUpdateRundir(TP_STRUCT * const tp,
                    struct path_data *path){

    if(*emcmot_hal_data->adaptive_feed<0){
        tp->reverse_run=1;
    } else {
        tp->reverse_run=0;
    }
}

void tpTrajectFinished(TP_STRUCT * const tp,
                       struct path_data *path){

    if((path->curpos>=path->trajectory_length) && !tp->reverse_run && push_counter(vector_ptr)>0){
        // Remove all segments.
        vector_clear(vector_ptr);
        path_reset(path);
        scurve_reset_data(&sc_data);
        path->traject_finished=1;
        printf("traject done. \n");
    }
}

void tpUpdateTarpos(TP_STRUCT * const tp,
                    struct path_data *path){

    if(vector_size(vector_ptr)==0){
        path->tarpos=0;
        return;
    }

    // Motion forward.
    if(!tp->reverse_run){
        path->tarpos=vector_at(vector_ptr, path->ringbuffer_index)->trajectory_length_end;
    } else { // Motion reverse.
        path->tarpos=vector_at(vector_ptr, path->ringbuffer_index)->trajectory_length_begin;
    }
}

int time_count=0;
int debug_look_ahead=0;

void tpUpdateEndvel(TP_STRUCT * const tp,
                    struct path_data *path){
    time_count++;

    // No segments. exit.
    if(vector_size(vector_ptr)<2){
        path->endvel=0;
        return;
    }

    /* Look ahead.
     * This consists of 2 cycles. A forward sweep and a backward sweep.
     */
    tpForwardSweep(vector_ptr, path);
    tpReverseSweep(vector_ptr, path);

    // Debug look ahead. Print once a second.
    // This function is left for info.
    if(time_count>1000 && debug_look_ahead){
        print_Look_ahead(vector_ptr, path);
        time_count=0;
    }

    struct emcmot_segment *seg=vector_at(vector_ptr, path->ringbuffer_index);

    if(!tp->reverse_run){ // Motion forward.
        // Set endvel for current segment.
        path->endvel=seg->ve;
    } else { // Motion reverse.
        // Set endvel for current segment.
        path->endvel=seg->vo;
    }

    // Apply feed override for non rapids. emcmotStatus->feed_scale
    if(seg->canon_motion_type==1){ // Rapid.
        // Motion type rapid.
        path->endvel*=emcmotStatus->rapid_scale;
    } else {
        // Motion type linear, arc, tool_change.
        path->endvel*=emcmotStatus->feed_scale;
    }

    // Limit to maxvel.
    path->endvel=fmin(path->endvel, tp->vLimit);

    // Apply adaptive feed ratio. -1, 0, 1.
    path->endvel = fabs( ( *emcmot_hal_data->adaptive_feed) * path->endvel );
}

// Conditions to calculate the max velocity, given the segment index nr.
void tpUpdateMaxVel(TP_STRUCT * const tp,
                    struct path_data *path){

    if(vector_size(vector_ptr)==0){
        path->maxvel=0;
        return;
    }

    struct emcmot_segment *seg=vector_at(vector_ptr, path->ringbuffer_index);

    path->maxvel=vector_at(vector_ptr, path->ringbuffer_index)->vel;

    // Apply feed override for non rapids. emcmotStatus->feed_scale
    if(seg->canon_motion_type==1){ // Rapid.
        path->maxvel*=emcmotStatus->rapid_scale;
    } else {
        path->maxvel*=emcmotStatus->feed_scale;
    }

    // Limit to maxvel.
    path->maxvel=fmin(path->maxvel, tp->vLimit);

    // Apply adaptive feed ratio. -1, 0, 1.
    path->maxvel = fabs( ( *emcmot_hal_data->adaptive_feed) * path->maxvel );
}

inline void tpUpdateScurveCycle(TP_STRUCT * const tp,
                                struct path_data *path){

    // Get start time
    clock_gettime(CLOCK_REALTIME, &start_1);

    // printf("ringbuffer index: %d \n",path->ringbuffer_index);

    // Set scurve values for jerk, acceleration, max velocity, cycletime.
    scurve_init(&sc_data,
                path->max_jerk,
                tp->aMax,
                path->maxvel,
                tp->cycleTime);

    // Update scurve cycle.
    scurve_set_target_state(&sc_data,
                            path->endvel,
                            path->endacc,
                            path->tarpos,
                            tp->pausing);

    path->scurve_return=scurve_update(&sc_data);

    // scurve_print_data(&sc_data);

    // Update scurve results.
    path->curvel=sc_data.curvel;
    path->curacc=sc_data.curacc;
    path->curpos=sc_data.curpos;

    // Travel a cycle.
    path->cycle_travel=sc_data.incpos;

    // Get end time
    clock_gettime(CLOCK_REALTIME, &end_1);

    // Calculate elapsed time in nanoseconds
    long elapsed_ns = (end_1.tv_sec - start_1.tv_sec) * 1000000000L;
    elapsed_ns += end_1.tv_nsec - start_1.tv_nsec;

    *hal_component_max_cycle_time_scurve_ns->Pin = fmax(*hal_component_max_cycle_time_scurve_ns->Pin, (double)elapsed_ns);

    if(*hal_reset_max_cycle_time->Pin){
        *hal_component_max_cycle_time_scurve_ns->Pin = 0;
    }

    // Monitor acceleration extrema used by scurve.
    *hal_acc_extrema->Pin = fmax(*hal_acc_extrema->Pin,fabs(path->curacc));

    // Reset acceleration extrema.
    if(*hal_reset_acc_extrema->Pin){
        *hal_acc_extrema->Pin = 0;
    }
}

// When program run's this is the cycle function.
int tpRunCycle(TP_STRUCT * const tp, long period){

    // Get start time
    clock_gettime(CLOCK_REALTIME, &start_0);

    if(tp->cycleTime!=(double)period * 0.000000001){
        printf("tpRunCycle differs in cycletime. \n");
    }

    tp->done=1; // Circulair buffer recieves new segments based on the tcqFull function.

    if(tap.tapping){                    // Tapping cycle active.
        return tpTapCycle(tp);          // Perform tap cycle.
    }

    if(emcmotStatus->net_feed_scale==0){        // Halt. Motion pause request from programs like plasmac.
        return 0;                               // External offsets still active to perform probing etc.
    }

    if(path.enable_keyboard_jog==1){            // Enable jogging in pause state.
        if(tpUpdateKeyPressJog(tp,&path)){         // Update keyboard inputs for jog in pause state.
            return 0;
        }
    }
    if(tpUpdateHalPinJog(tp,&path)){                // Update halui inputs for jog in pause state.
        tpUpdateHal(tp,&path,emcmotStatus);     // This needs to update hal pins before return is done.
        return 0;
    }

    tpUpdateRundir(tp,&path);           // Set forward or reverse run.
    tpUpdateTarpos(tp,&path);           // If motion forward, tarpos is end of segment. If motion reverse tarpos is start of segment.
    tpUpdateMaxVel(tp,&path);           // Update max velocity, given gui's slider input's and current segment maxvel.
    tpUpdateEndvel(tp,&path);           // Update end velocity for the current motion.
    tpUpdateScurveCycle(tp,&path);      // Update the scurve cycle.

    // Segment completed, still within the trajectory.
    if(path.scurve_return==RETURN_FINISHED){
        tpSetNextMotion(tp,&path);
        tpTrajectFinished(tp,&path);        // Check traject is finished.
    }

    path.cycle++;                       // Planner cycle monitoring.
    if(path.cycle>1000){
        path.cycle=0;
    }

    tpUpdateGui(tp,&path);              // Update gui positions.
    tpUpdateHal(tp,&path,emcmotStatus); // Update hal pins.

    // Get end time
    clock_gettime(CLOCK_REALTIME, &end_0);

    // Calculate elapsed time in nanoseconds
    long elapsed_ns = (end_0.tv_sec - start_0.tv_sec) * 1000000000L;
    elapsed_ns += end_0.tv_nsec - start_0.tv_nsec;

    // Time a cycle.
    *hal_component_cycle_time_ns->Pin = (double)elapsed_ns;

    // Time upper extrema.
    *hal_component_max_cycle_time_ns->Pin = fmax(*hal_component_max_cycle_time_ns->Pin, (double)elapsed_ns);
    if((double)elapsed_ns>1000000){
        // printf("cycle time alarm. \n");
    }

    // Reset cycle time extrema.
    if(*hal_reset_max_cycle_time->Pin){
        *hal_component_max_cycle_time_ns->Pin = 0;
    }

    return 0;
}

// Closing linux cnc. Free memory.
int tpClear(TP_STRUCT * const tp){

    /*  This triggers a signal 11, dumping core:
        vector_remove_ptr(vector_ptr);
        filletizer_remove_ptr(filletizer_ptr);
        scurve_remove_ptr(scurve_ptr);
        motionizer_remove_ptr(motionizer_ptr);
    */
    // printf("tpClear. \n");

    keyboard_cleanup(&path);

    return 0;
}

// Set the cycletime. Not used.
int tpSetCycleTime(TP_STRUCT * const tp, double secs){

    if (!tp || secs <= 0.0) {
        return -1;
    }
    // printf("tpSetCycleTime. %f \n",secs);
    tp->cycleTime = secs;
    return 0;
}

// Set the maximum velocity's. Not used.
int tpSetVmax(TP_STRUCT * const tp, double vMax, double ini_maxvel){

    if (!tp || vMax <= 0.0 || ini_maxvel <= 0.0) {
        return -1;
    }

    tp->vMax = vMax;
    tp->ini_maxvel = ini_maxvel;
    return 0;
}

// Program speed, taken from the gui velocity slider widget.
int tpSetVlimit(TP_STRUCT * const tp, double vLimit){

    if(!tp){ return -1;}

    if (vLimit < 0.0){
        tp->vLimit = 0.;
    } else {
        tp->vLimit = vLimit;
    }
    // printf("tpVLimit, INI MAX_LINEAR_VELOCITY: %f \n",tp->vLimit);
    return 0;
}

int tpSetAmax(TP_STRUCT * const tp, double aMax){

    if (!tp || aMax <= 0.0) {
        return -1;
    }
    // printf("tpAmax, INI MAX_LINEAR_ACCELERATION: %f \n",aMax);
    tp->aMax=aMax;
    return 0;
}

// Set gcode exec id for upcoming new line, arc.
int tpSetId(TP_STRUCT * const tp, int id){

    if(!tp){return -1;}
    // printf("tpSetId, execId: %d \n",id);
    tp->execId=id;
    return 0;
}

// This is the current executed gcode line nr.
// The gui's gcode preview uses this to highlight and set
// the gcode line in the gcode editor.
int tpGetExecId(TP_STRUCT * const tp)
{
    if(!tp){return -1;}

    int gcode_line_nr=0;
    if(vector_size(vector_ptr)>0 /*&& path.motion_index<vector_size(vector_ptr)*/){
        gcode_line_nr=vector_at(vector_ptr, path.ringbuffer_index)->id;
    }
    if(tap.tapping){
        gcode_line_nr=seg_tap.id;
    }
    return gcode_line_nr;
}

// Not used.
int tpSetTermCond(TP_STRUCT * const tp, int cond, double tolerance)
{
    return 0;
}

// Used to tell the tp the initial position.
// It sets the current position AND the goal position to be the same.  Used
// only at TP initialization and when switching modes.
int tpSetPos(TP_STRUCT * const tp, EmcPose const * const pos){

    if(!tp){return -1;}

    tp->currentPos=*pos;
    tp->goalPos=*pos;
    // printf("tpSetPos x: %f y: %f z: %f \n",pos->tran.x,pos->tran.y,pos->tran.z);
    return 0;
}

// The gui's toolposition tp is updated from here.
int tpGetPos(TP_STRUCT const * const tp, EmcPose * const pos){

    if(!tp){return -1;}
    *pos = tp->currentPos;
    // printf("tpGetPos x: %f y: %f z: %f \n",pos->tran.x,pos->tran.y,pos->tran.z);
    return 0;
}

// Not used.
int tpErrorCheck(TP_STRUCT const * const tp){

    if(!tp){return -1;}
    return 0;
}

// This function is called just befure tpAddRigidTap to set pitch & spindle_nr to
// use with tapping cycle.
int tpSetSpindleSync(TP_STRUCT * const tp, int spindle, double sync, int mode){

    if(!tp){return -1;}
    if(sync) {
        if (mode) {
            tp->synchronized = TC_SYNC_VELOCITY;
        } else {
            tp->synchronized = TC_SYNC_POSITION;
        }
        // printf("spindle nr: %d pitch: %f", tp->uu_per_rev, tp->spindle.spindle_num);
        tp->uu_per_rev = sync;
        tp->spindle.spindle_num = spindle;
    } else
        tp->synchronized = 0;
    return 0;
}

// Pause button pressed.
int tpPause(TP_STRUCT * const tp){

    tp->pausing=1;
    return 0;
}

// Resume button pressed.
int tpResume(TP_STRUCT * const tp){

    tp->pausing=0;
    return 0;
}

// Stop button pressed.
int tpAbort(TP_STRUCT * const tp){

    tp->pausing=0;      // Reset pause.
    // tp->aborting=1;     // Not used.

    // Remove all segments.
    vector_clear(vector_ptr);
    scurve_reset_data(&sc_data);
    path_reset(&path);

    printf("tpAbort. \n");

    return 0;
}

// This info is problably used by pyvcp gui's.
// However pyvcp is not getting the bspline motion type.
int tpGetMotionType(TP_STRUCT * const tp){

    return tp->motionType;
}

// Return 1 for is done. Return 0 for not done.
int tpIsDone(TP_STRUCT * const tp){

    int done=tp->done;
    if(done){
        tp->done=0;
    }
    return done;
}

// Ensures fininshing the traject. Otherwise the planner stop's the program too early.
int tpQueueDepth(TP_STRUCT * const tp){

    if(!tp){return -1;}
    if(push_counter(vector_ptr)>0){
        return 1;
    }
    return 0;
}

// Current active buffer depth.
int tpActiveDepth(TP_STRUCT * const tp){

    if(!tp){return -1;}

    if(push_counter(vector_ptr)>0){
        return 1;
    }
    return 0;
}

// Used implementation from original planner.
int tpSetAout(TP_STRUCT * const tp, unsigned char index, double start, double end){

    if(!tp){return -1;}
    tp->syncdio.anychanged = 1; //something has changed
    tp->syncdio.aio_mask |= (1 << index);
    tp->syncdio.aios[index] = start;
    return 0;
}

// Not used.
int tpSetDout(TP_STRUCT * const tp, int index, unsigned char start, unsigned char end){

    if(!tp){return -1;}
    return 0;
}

// Motion rundir forward or reverse. This is done in the tpRun function where we read the hal pin.
// adaptive feed.
int tpSetRunDir(TP_STRUCT * const tp, tc_direction_t dir){

    if(!tp){return -1;}

    if(dir==0){
        // printf("tpSetRunDir, forward \n");
        return 0;
    }
    if(dir==1){
        // printf("tpSetRunDir, reverse \n");
        return 0;
    }
    return -1;
}

// Each segment has a tag in the struct containing extra info.
struct state_tag_t tpGetExecTag(TP_STRUCT * const tp){

    if(!tp){
        struct state_tag_t empty = {0};
        return empty;
    }
    return tp->execTag;
}

/*
// This function is responsible for long startup delay if return=1.
int tcqFull(TC_QUEUE_STRUCT const * const tcq){

    // Priority fill buffer up to ** half ** the buffer size.
    // Dont fill the entire buffer at program start.
    // This results in:
    // When adding fillets, 2 segments at at time
    // are added to the buffer, resulting in overwriting the todo segments.
    if(push_counter(vector_ptr)<VECTOR_BUFFER_SIZE){
        return 0; // Add segments.
    }

    // Add new segments to the ringbuffer.
    if(path.global_index > push_counter(vector_ptr) - (0.5*VECTOR_BUFFER_SIZE)){
        // printf("add segment. \n");
        return 0; // Add segments.
    }

    // Full. No new segments are added.
    return 1;
}*/

// This function is responsible for long startup delay if return=1.
int tcqFull(TC_QUEUE_STRUCT const * const tcq){

    // Priority fill buffer up to ** half ** the buffer size.
    // Dont fill the entire buffer at program start.
    // This results in:
    // When adding fillets, 2 segments at at time
    // are added to the buffer, resulting in overwriting the todo segments.
    if(push_counter(vector_ptr)<VECTOR_BUFFER_SIZE*0.5){
        return 0; // Add segments.
    }

    // Add new segments to the ringbuffer.
    if(path.global_index > push_counter(vector_ptr) - ((0.5*VECTOR_BUFFER_SIZE)-1)){
        // printf("add segment. \n");
        return 0; // Add segments.
    }

    // Full. No new segments are added.
    return 1;
}

// Functions called by motion:
EXPORT_SYMBOL(tpMotFunctions);
EXPORT_SYMBOL(tpMotData);
EXPORT_SYMBOL(tpMotExtraData);
EXPORT_SYMBOL(tpAbort);
EXPORT_SYMBOL(tpActiveDepth);
EXPORT_SYMBOL(tpAddCircle);
EXPORT_SYMBOL(tpAddLine);
EXPORT_SYMBOL(tpAddRigidTap);
EXPORT_SYMBOL(tpClear);
EXPORT_SYMBOL(tpCreate);
EXPORT_SYMBOL(tpGetExecId);
EXPORT_SYMBOL(tpGetExecTag);
EXPORT_SYMBOL(tpGetMotionType);
EXPORT_SYMBOL(tpGetPos);
EXPORT_SYMBOL(tpIsDone);
EXPORT_SYMBOL(tpPause);
EXPORT_SYMBOL(tpQueueDepth);
EXPORT_SYMBOL(tpResume);
EXPORT_SYMBOL(tpRunCycle)
EXPORT_SYMBOL(tpSetAmax);
EXPORT_SYMBOL(tpSetAout);
EXPORT_SYMBOL(tpSetCycleTime);
EXPORT_SYMBOL(tpSetDout);
EXPORT_SYMBOL(tpSetId);
EXPORT_SYMBOL(tpSetPos);
EXPORT_SYMBOL(tpSetRunDir);
EXPORT_SYMBOL(tpSetSpindleSync);
EXPORT_SYMBOL(tpSetTermCond);
EXPORT_SYMBOL(tpSetVlimit);
EXPORT_SYMBOL(tpSetVmax);
EXPORT_SYMBOL(tcqFull);
