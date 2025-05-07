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

#include "feed.h"

int G93_active(const struct emcmot_segment *seg){
    if((seg->tag.packed_flags >> GM_FLAG_FEED_INVERSE_TIME) & 1 ){
        return 1;
    }
    return 0;
}

int G94_active(const struct emcmot_segment *seg){
    if((seg->tag.packed_flags >>  GM_FLAG_FEED_UPM) & 1 ){
        return 1;
    }
    return 0;
}

int G95_spindle(TP_STRUCT * const tp){
    return tp->spindle.spindle_num;
}

int G95_active(const struct emcmot_segment *seg){
    if((seg->tag.packed_flags >>  GM_FLAG_FEED_UPM) & 0 ){
        return 1;
    }
    return 0;
}

double FEED_MM_MIN(const struct emcmot_segment *seg){
    return seg->tag.fields_float[1];
}
double FEED_MM_SEC(const struct emcmot_segment *seg){
    return seg->tag.fields_float[1]/60;
}

double SPEED_RPM(const struct emcmot_segment *seg){
    return seg->tag.fields_float[2];
}

double max_velocity(double distance,
                    double amax,
                    double total_time) {

    // Calculate the maximum velocity (vm) based on the given constraints
    // The motion profile can be either triangular or trapezoidal, depending on the distance and time.

    // Check if a triangular profile is possible (max velocity is never reached)
    double vm_triangular = sqrt(distance * amax);
    double time_triangular = 2 * sqrt(distance / amax);

    if (time_triangular <= total_time) {
        // Triangular profile is sufficient
        return vm_triangular;
    } else {
        // Trapezoidal profile is required
        // Solve for vm in the trapezoidal profile equation:
        // total_time = (vm / amax) + (distance - (vm^2 / amax)) / vm + (vm / amax)
        // Simplify: total_time = (2 * vm / amax) + (distance / vm) - (vm / amax)
        // Further simplify: total_time = (vm / amax) + (distance / vm)

        // Use numerical methods (e.g., Newton-Raphson) to solve for vm
        double vm = sqrt(distance * amax); // Initial guess
        double tolerance = 1e-6;
        double error = tolerance + 1;

        while (error > tolerance) {
            double f = (vm / amax) + (distance / vm) - total_time;
            double df = (1 / amax) - (distance / (vm * vm));
            double vm_new = vm - f / df;
            error = fabs(vm_new - vm);
            vm = vm_new;
        }

        return vm;
    }
}

// This function uses vo=0 and ve=0.
double motion_time(double distance, double vm, double amax) {
    // Calculate distance for acceleration phase
    double d_acc = (vm * vm) / (2 * amax);

    if (distance <= 2 * d_acc) {
        // Triangular profile: max velocity is never reached
        return 2 * sqrt(distance / amax);
    } else {
        // Trapezoidal profile: max velocity is reached
        double t_acc = vm / amax;               // Time to accelerate to vm
        double d_const = distance - 2 * d_acc;  // Distance at constant velocity
        double t_const = d_const / vm;          // Time at constant velocity
        return 2 * t_acc + t_const;             // Total time = acc + const + dec
    }
}

enum enum_motion_set slowest_motion_set(struct emcmot_config_t *emcmotConfig,
                                        struct emcmot_segment *seg) {

    emcmot_joint_t *joint;
    double joint_times[9] = {0}; // Array to store motion times for each joint
    enum enum_motion_set motion_set = xyz; // Slowest motion set.

    // Copy joint position feedback to local array
    for (int i = 0; i < emcmotConfig->numJoints; i++) {
        /* Point to joint struct */
        joint = &joints[i];

        // Calculate motion time for each joint
        switch (i) {
        case 0: // X
            joint_times[i] = motion_time(fabs(seg->end.tran.x - seg->start.tran.x),
                                         joint->vel_limit,
                                         joint->acc_limit);
            break;
        case 1: // Y
            joint_times[i] = motion_time(fabs(seg->end.tran.y - seg->start.tran.y),
                                         joint->vel_limit,
                                         joint->acc_limit);
            break;
        case 2: // Z
            joint_times[i] = motion_time(fabs(seg->end.tran.z - seg->start.tran.z),
                                         joint->vel_limit,
                                         joint->acc_limit);
            break;
        case 3: // A
            joint_times[i] = motion_time(fabs(seg->end.a - seg->start.a),
                                         joint->vel_limit,
                                         joint->acc_limit);
            break;
        case 4: // B
            joint_times[i] = motion_time(fabs(seg->end.b - seg->start.b),
                                         joint->vel_limit,
                                         joint->acc_limit);
            break;
        case 5: // C
            joint_times[i] = motion_time(fabs(seg->end.c - seg->start.c),
                                         joint->vel_limit,
                                         joint->acc_limit);
            break;
        case 6: // U
            joint_times[i] = motion_time(fabs(seg->end.u - seg->start.u),
                                         joint->vel_limit,
                                         joint->acc_limit);
            break;
        case 7: // V
            joint_times[i] = motion_time(fabs(seg->end.v - seg->start.v),
                                         joint->vel_limit,
                                         joint->acc_limit);
            break;
        case 8: // W
            joint_times[i] = motion_time(fabs(seg->end.w - seg->start.w),
                                         joint->vel_limit,
                                         joint->acc_limit);
            break;
        default:
            break;
        }
    }

    // Find the slowest joint in each motion set
    double max_time_xyz = 0;
    double max_time_abc = 0;
    double max_time_uvw = 0;

    for (int i = 0; i < emcmotConfig->numJoints; i++) {
        if (i < 3) { // xyz joints (0, 1, 2)
            if (joint_times[i] > max_time_xyz) {
                max_time_xyz = joint_times[i];
            }
        } else if (i < 6) { // abc joints (3, 4, 5)
            if (joint_times[i] > max_time_abc) {
                max_time_abc = joint_times[i];
            }
        } else { // uvw joints (7, 8, 9)
            if (joint_times[i] > max_time_uvw) {
                max_time_uvw = joint_times[i];
            }
        }
    }

    // Set the slowest motion set
    if (max_time_xyz >= max_time_abc && max_time_xyz >= max_time_uvw) {
        motion_set = xyz;
        // printf("Slowest motion set: xyz. Time: %f\n", max_time_xyz);
    } else if (max_time_abc >= max_time_xyz && max_time_abc >= max_time_uvw) {
        motion_set = abc;
        // printf("Slowest motion set: abc. Time: %f\n", max_time_abc);
    } else {
        motion_set = uvw;
        // printf("Slowest motion set: uvw. Time: %f\n", max_time_uvw);
    }

    return motion_set;
}

double joint_dist(struct emcmot_segment *seg,
                  int joint){

    double dist=0;
    switch (joint) {
    case 0: // X
        dist = fabs(seg->start.tran.x - seg->end.tran.x);
        break;
    case 1: // Y
        dist = fabs(seg->start.tran.y - seg->end.tran.y);
        break;
    case 2: // Z
        dist = fabs(seg->start.tran.z - seg->end.tran.z);
        break;
    case 3: // A
        dist = fabs(seg->start.a - seg->end.a);
        break;
    case 4: // B
        dist = fabs(seg->start.b - seg->end.b);
        break;
    case 5: // C
        dist = fabs(seg->start.c - seg->end.c);
        break;
    case 6: // U
        dist = fabs(seg->start.u - seg->end.u);
        break;
    case 7: // V
        dist = fabs(seg->start.v - seg->end.v);
        break;
    case 8: // W
        dist = fabs(seg->start.w - seg->end.w);
        break;
    default:
        break;
    }
    return dist;
}

// When in G93 inverse time mode.
// The given feed is calcuated by:
// ~/src/emc/rs274ngc/interp_inverse.cc L129
// We want to get the original F number from the gcode file.
// Resore feed by :
// rate = length * block->f_number
// FEED = rate / lenght.
double restore_inverse_feed_to_gcode_feed(const struct emcmot_segment *seg){

    // This sequence is used by:  ~/src/emc/rs274ngc/interp_inverse.cc L129
    if(seg->length_xyz>0){
        return FEED_MM_MIN(seg)/seg->length_xyz;
    }
    if(seg->length_uvw>0){
        return FEED_MM_MIN(seg)/seg->length_uvw;
    }
    if(seg->length_abc>0){
        return FEED_MM_MIN(seg)/seg->length_abc;
    }
    return 0;
}

// Apply feed for inverse time G93.
void update_g93_feed(struct emcmot_segment *seg, double time){

    if(seg->length_xyz>0){
        // s = v * t.
        seg->vel = seg->length_xyz / time;
        return;
    }
    // This uvw sequence is also used in
    // ~/src/emc/rs274ngc/interp_inverse.cc L129
    if(seg->length_uvw>0){
        seg->vel = seg->length_uvw / time;
        return;
    }
    if(seg->length_abc>0){
        seg->vel = seg->length_abc / time;
        return;
    }
}

int init_feed(TP_STRUCT * const tp,
              struct emcmot_segment *seg){

    // Rapid.
    if(seg->canon_motion_type==1){
        // G0 takes seg->vel already in units/sec.
    } else { // Feed move of type linear, arc, etc.

        // Pre calculated value from interpreter.
        // Is lowering xyz speeds to ensure abc speeds are in limits.
        // seg->vel;

        // This value is the bruto gcode Feed.
        // Is able to go faster then abc max speeds.
        // seg->vel = FEED_MM_SEC(seg);
    }

    // G93 inverse time mode. Tested ok.
    // ~/src/emc/rs274ngc/interp_inverse.cc L129
    // rate = length * block->f_number;
    if(G93_active(seg)){

        double feed = restore_inverse_feed_to_gcode_feed(seg);

        // Inverse time = 1 / FEED = [x] minutes.
        double time = (1/feed) * 60 ; // Time in sec.

        // Set the feed for xyz, abc, uvw.
        update_g93_feed(seg, time);
    }
    if(G94_active(seg)){ // Units per minute
        // Here the interpreter also is lowering speeds for xyz motions
        // if abc, uvw motions thake longer time. The seg.vel is modified.
        // We want the original gcode line input value for our feed input.

        // double gcode_feed = seg->tag.fields_float[1]/60; // From mm/min to mm/sec.

        // At linuxcnc startup this value can be zero. Then it takes the tpAddLine
        // or tpAddCircle value.
        // if(gcode_feed>0){
        //     seg->vel = gcode_feed;
        // }

        // printf("feed: %f \n",seg->tag.fields_float[1]);  // Unmodified gcode feed F in mm/min.
        // printf("speed: %f \n",seg->tag.fields_float[2]); // =0 at runtime.
    }

    // G95, units per revolution. Tested ok.
    if(G95_active(seg)){ // Units per revolution.
        // Velocity = (F1000*S100) / 60  [mm/sec]
        seg->vel = (FEED_MM_MIN(seg) / SPEED_RPM(seg)) / 60;
    }
}








