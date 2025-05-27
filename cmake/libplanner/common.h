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
#ifndef COMMON_H
#define COMMON_H

enum path_optimalisation_algo {
    file_path_standard,
    file_path_standard_subseg,
    file_path_standard_subseg_line_fillet,
    file_path_clothoid,
    file_path_clothoid_abc_uvw
};

struct path_data {

    double cycletime;
    int ringbuffer_index;
    int global_index;
    int motion_enable;
    int traject_finished;
    int buffer_overflow;
    int use_real_deviation;

    double curvel;
    double curacc;
    double curpos;
    double endacc;
    double maxvel;
    double tarpos;
    double endvel;
    int scurve_return;
    double progress;
    int cycle;
    double cycle_travel;
    int motion_increments_a_cycle;
    double trajectory_length;

    double inertia_factor;

    // For reverse motion and ringbuffer we have to count
    // how many segments we can go back related to the ringbuffer.
    int reverse_motion_reset;
    int reverse_motion_back_count;

    enum path_optimalisation_algo path_algo;

    double max_jerk;        // A nice jerk value is 2*tp->aMax. Axis_mm.ini aMax=500

    int jog_x_plus;
    int jog_x_min;
    int jog_y_plus;
    int jog_y_min;
    int jog_z_plus;
    int jog_z_min;
    int must_jog_back;
    int enable_keyboard_jog;

    double tangential_knife_angle;              // Output angle to hal in degrees.
    double tangential_knife_previous_pos[3];    // Xyz coordinate of previous cycle position.
    double tangential_knife_current_pos[3];     // Xyz coordinate of current cycle position.
    double tangential_previous_angle;

    double segment_radius;
};

static void path_reset(struct path_data *path){
    path->cycletime=0;
    path->ringbuffer_index=0;
    path->global_index=0;
    path->motion_enable=0;
    path->curvel=0;
    path->curacc=0;
    path->curpos=0;
    path->endacc=0;
    path->maxvel=0;
    path->tarpos=0;
    path->endvel=0;
    path->scurve_return=0;
    path->progress=0;
    path->cycle=0;
    path->path_algo=file_path_standard;
    path->max_jerk=0;
    path->trajectory_length=0;
    path->traject_finished=0;
    path->motion_increments_a_cycle=0;
    path->buffer_overflow=0;
    path->use_real_deviation=0;

    path->inertia_factor = 0;

    path->jog_x_plus=0;
    path->jog_x_min=0;
    path->jog_y_plus=0;
    path->jog_y_min=0;
    path->jog_z_plus=0;
    path->jog_z_min=0;
    path->must_jog_back=0;
    // path->enable_keyboard_jog=0;
}

#endif // COMMON_H
