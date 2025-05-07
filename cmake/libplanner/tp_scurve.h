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
#ifndef TPS_H
#define TPS_H

#include "vector.h"
#include "mot_priv.h"

enum tap_cycle {
    tap_init,
    wait_spindle_at_speed,
    wait_spindle_at_position,
    z_axis_acc_stage,
    tap_fwd,
    tap_rev,
    z_axis_dcc_stage,
    tap_finished
};

struct tap_struct {
    // Vars for tapping sequence.
    int tapping;
    double hole_depth;
    double pitch;
    double return_scale;
    int spindle_nr;
    double feed_fwd;
    double feed_rev;
    double acc;
    double start_turns;
    double turns;
    double z_pos;
    double z_pos_prev;
    double z_acc_pos;
    double z_vel;
    double rev_stop_dist;
};

void zero_tap_struct(struct tap_struct tap){
    tap.tapping=0;
    tap.hole_depth=0;
    tap.pitch=0;
    tap.return_scale=0;
    tap.spindle_nr=0;
    tap.feed_fwd=0;
    tap.feed_rev=0;
    tap.acc=0;
    tap.start_turns=0;
    tap.turns=0;
    tap.z_pos=0;
    tap.z_pos_prev=0;
    tap.z_acc_pos=0;
    tap.z_vel=0;
    tap.rev_stop_dist=0;
}


struct vector *vector_ptr;
struct filletizer *filletizer_ptr;
struct motionizer *motionizer_ptr;
struct scurve_data sc_data;

enum tap_cycle tap_state;
struct tap_struct tap;
struct emcmot_segment seg_tap;

emcmot_status_t *emcmotStatus;
emcmot_config_t *emcmotConfig;
emcmot_command_t *emcmotCommand;
emcmot_hal_data_t *emcmot_hal_data;
// Also used in : motion-logger.c, declared extern in mot_priv.h
emcmot_joint_t joints[EMCMOT_MAX_JOINTS];

#endif // TPS_H
