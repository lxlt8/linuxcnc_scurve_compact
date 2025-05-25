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
#ifndef HALSECTION_H
#define HALSECTION_H

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"
#include "common.h"
#include "vector.h"

typedef struct { // Pin float.
    hal_float_t *Pin;
} float_data_t;

typedef struct { // Pin int.
    hal_s32_t *Pin;
} s32_data_t;

typedef struct { // Pin bit.
    hal_bit_t *Pin;
} bit_data_t;

typedef struct { // Param float.
    hal_float_t Pin;
} param_float_data_t;

typedef struct { // Param int.
    hal_s32_t Pin;
} param_s32_data_t;

bit_data_t
*hal_jog_x_plus,                // Jog when program is in pause.
*hal_jog_x_min,
*hal_jog_y_plus,
*hal_jog_y_min,
*hal_jog_z_plus,
*hal_jog_z_min,
*hal_enable_feed_scale_zero,    // Enable feed scale zero. Used by motion pause request from programs like plasmac.
                                // External offsets still active to perform probing etc.
*hal_enable_keyboard_jog,       // Enable keyboard jog. Hal jog enabled by default.
*hal_reset_max_cycle_time,      // Reset max cycle time extrema to zero.
*hal_reset_acc_extrema;         // Reset acceleration extrema to zero.

float_data_t
*hal_tp_curvel,                 // Current velocity of motion. mm/s, gui display's: mm/min.
*hal_tp_curacc,                 // Current acceleration of motion. mm/s^2
*hal_tp_curpos,                 // Current position in mm.
*hal_tp_tarpos,                 // Target position in mm.
*hal_tp_endvel,                 // Target velocity of motion. mm/s
*hal_index_length,              // Current active segment length.
*hal_index_progress,            // Current active segment progress 0-1
*hal_rapid_override,            // Current rapid override in %, for G0 moves.
*hal_feed_override,             // Current feed override in %, for G1,G2,G3 moves.
*hal_max_vel,                   // Current max velocity.
*hal_trajectory_end,            // Trajectory end postition.
*hal_scurve_cycle_dist,         // The distance the scurve has travelled in a servo cycle.
*hal_traject_buffer_dist,       // How much distance is buffered into the future.
*hal_segment_vm,                // Current segment velocity max.
*hal_segment_ve,                // Current segment velocity begin.
*hal_segment_radius,            // Radius of current segment. May vary on clothoid.
*hal_tangential_knife_angle,    // Current tangential knife angle.
*hal_component_cycle_time_ns,               // Monitor cycle time in ms.
*hal_component_max_cycle_time_ns,           // Monitor max cycle time in ns.
*hal_component_max_cycle_time_scurve_ns,    // Monitor max cycle time for scurve in ns.
*hal_acc_extrema;                           // Max acceleration used by scurve.

s32_data_t
*hal_ring_buffer_index,         // Current active segment nr. 0-Buffer size.
*hal_global_buffer_index,       // Current global segment.
*hal_index_size,                // Ammount of segments stored in the program
*hal_push_counter,              // Current segment nr from program start.
*hal_index_return_code,         // Scurve return code, 0=ok, 1=error, 2=finished, 3=busy.
*hal_index_motion_type,         // Current motion type, 1=rapid, 2=linear, 3=arc, 4=tool_change, 5=rigid_tap, 10=spline fillet.
*hal_tap_synchronized_motion,   // If true, the spindle and the z axis are synchronized in motion, safe for tapping.
*hal_cycle,                     // Active monitoring.
*hal_future_buffer,             // How many segments are buffered in the future.
*hal_motion_increments_a_cycle, // How many motions are incremented a servo cycle. This happens when scurve skips motions at high speeds.
*hal_buffer_overflow,           // Buffer overflow, velocity set to high. Planner can't keep up.
*hal_use_real_deviation;        // Use the clothoid deviation fit method, or use a fast trim deviation method.

param_float_data_t
*hal_max_jerk;                  // Scurve max jerk. Edit value when motion is not active.

// Set hal startup values.
static inline void set_hal_path_values(struct path_data *path){

    // Set default value.

    // For params dont use at front : *
    hal_max_jerk->Pin=path->max_jerk;

    // For pins use at front : *
    *hal_use_real_deviation->Pin=path->use_real_deviation;
}

// Setup hal pins, no safety checks are done here. Any typo's will punish you at runtime.
static inline void setup_hal_pins(int tpmod_id,
                                  struct path_data *path){

    // Hal pins.
    hal_enable_feed_scale_zero = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    hal_pin_bit_new("tpmod.hal_enable_feed_scale_zero",HAL_IN,&(hal_enable_feed_scale_zero->Pin),tpmod_id);

    hal_enable_keyboard_jog = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    hal_pin_bit_new("tpmod.hal_enable_keyboard_jog",HAL_IN,&(hal_enable_keyboard_jog->Pin),tpmod_id);

    hal_jog_x_plus = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    hal_pin_bit_new("tpmod.hal_jog_x_plus",HAL_IN,&(hal_jog_x_plus->Pin),tpmod_id);
    hal_jog_x_min = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    hal_pin_bit_new("tpmod.hal_jog_x_min",HAL_IN,&(hal_jog_x_min->Pin),tpmod_id);

    hal_jog_y_plus = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    hal_pin_bit_new("tpmod.hal_jog_y_plus",HAL_IN,&(hal_jog_y_plus->Pin),tpmod_id);
    hal_jog_y_min = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    hal_pin_bit_new("tpmod.hal_jog_y_min",HAL_IN,&(hal_jog_y_min->Pin),tpmod_id);

    hal_jog_z_plus = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    hal_pin_bit_new("tpmod.hal_jog_z_plus",HAL_IN,&(hal_jog_z_plus->Pin),tpmod_id);
    hal_jog_z_min = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    hal_pin_bit_new("tpmod.hal_jog_z_min",HAL_IN,&(hal_jog_z_min->Pin),tpmod_id);

    hal_reset_max_cycle_time = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    hal_pin_bit_new("tpmod.hal_reset_max_cycle_time",HAL_IN,&(hal_reset_max_cycle_time->Pin),tpmod_id);

    hal_reset_acc_extrema = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    hal_pin_bit_new("tpmod.hal_reset_acc_extrema",HAL_IN,&(hal_reset_acc_extrema->Pin),tpmod_id);

    hal_acc_extrema = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_acc_extrema",HAL_OUT,&(hal_acc_extrema->Pin),tpmod_id);

    hal_component_cycle_time_ns = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_component_cycle_time_ns",HAL_OUT,&(hal_component_cycle_time_ns->Pin),tpmod_id);

    hal_component_max_cycle_time_ns = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_component_max_cycle_time_ns",HAL_OUT,&(hal_component_max_cycle_time_ns->Pin),tpmod_id);

    hal_component_max_cycle_time_scurve_ns = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_component_max_cycle_time_scurve_ns",HAL_OUT,&(hal_component_max_cycle_time_scurve_ns->Pin),tpmod_id);


    hal_tangential_knife_angle = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_tangential_knife_angle",HAL_OUT,&(hal_tangential_knife_angle->Pin),tpmod_id);

    hal_segment_radius = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_segment_radius",HAL_OUT,&(hal_segment_radius->Pin),tpmod_id);

    hal_tp_curvel = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_tp_curvel",HAL_OUT,&(hal_tp_curvel->Pin),tpmod_id);

    hal_tp_curacc = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_tp_curacc",HAL_OUT,&(hal_tp_curacc->Pin),tpmod_id);

    hal_tp_curpos = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_tp_curpos",HAL_OUT,&(hal_tp_curpos->Pin),tpmod_id);

    hal_tp_tarpos = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_tp_tarpos",HAL_OUT,&(hal_tp_tarpos->Pin),tpmod_id);

    hal_tp_endvel = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_tp_endvel",HAL_OUT,&(hal_tp_endvel->Pin),tpmod_id);

    hal_index_length = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_index_length",HAL_OUT,&(hal_index_length->Pin),tpmod_id);

    hal_index_progress = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_index_progress",HAL_OUT,&(hal_index_progress->Pin),tpmod_id);

    hal_feed_override = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_feed_override",HAL_OUT,&(hal_feed_override->Pin),tpmod_id);

    hal_rapid_override = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_rapid_override",HAL_OUT,&(hal_rapid_override->Pin),tpmod_id);

    hal_max_vel = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_max_vel",HAL_OUT,&(hal_max_vel->Pin),tpmod_id);

    hal_trajectory_end = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_trajectory_end",HAL_OUT,&(hal_trajectory_end->Pin),tpmod_id);

    hal_scurve_cycle_dist = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_scurve_cycle_dist",HAL_OUT,&(hal_scurve_cycle_dist->Pin),tpmod_id);

    hal_traject_buffer_dist = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_traject_buffer_dist",HAL_OUT,&(hal_traject_buffer_dist->Pin),tpmod_id);

    hal_segment_vm = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_segment_vm",HAL_OUT,&(hal_segment_vm->Pin),tpmod_id);

    hal_segment_ve = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("tpmod.hal_segment_ve",HAL_OUT,&(hal_segment_ve->Pin),tpmod_id);

    // Parameter pins. These are used to set values at runtime.
    hal_max_jerk = (param_float_data_t*)hal_malloc(sizeof(param_float_data_t));
    hal_param_float_new("tpmod.hal_max_jerk",HAL_RW,&(hal_max_jerk->Pin),tpmod_id);

    hal_ring_buffer_index = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    hal_pin_s32_new("tpmod.hal_ringbuffer_index",HAL_OUT,&(hal_ring_buffer_index->Pin),tpmod_id);

    hal_global_buffer_index = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    hal_pin_s32_new("tpmod.hal_global_buffer_index",HAL_OUT,&(hal_global_buffer_index->Pin),tpmod_id);

    hal_index_size = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    hal_pin_s32_new("tpmod.hal_index_size",HAL_OUT,&(hal_index_size->Pin),tpmod_id);

    hal_push_counter = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    hal_pin_s32_new("tpmod.hal_push_counter",HAL_OUT,&(hal_push_counter->Pin),tpmod_id);

    hal_index_return_code = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    hal_pin_s32_new("tpmod.hal_index_return_code",HAL_OUT,&(hal_index_return_code->Pin),tpmod_id);

    hal_index_motion_type = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    hal_pin_s32_new("tpmod.hal_index_moton_type",HAL_OUT,&(hal_index_motion_type->Pin),tpmod_id);

    hal_tap_synchronized_motion = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    hal_pin_s32_new("tpmod.hal_tap_synchronized_motion",HAL_OUT,&(hal_tap_synchronized_motion->Pin),tpmod_id);

    hal_cycle = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    hal_pin_s32_new("tpmod.hal_cycles",HAL_OUT,&(hal_cycle->Pin),tpmod_id);

    hal_future_buffer = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    hal_pin_s32_new("tpmod.hal_future_buffer",HAL_OUT,&(hal_future_buffer->Pin),tpmod_id);

    hal_motion_increments_a_cycle = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    hal_pin_s32_new("tpmod.hal_motion_increments_a_cycle",HAL_OUT,&(hal_motion_increments_a_cycle->Pin),tpmod_id);

    hal_buffer_overflow = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    hal_pin_s32_new("tpmod.hal_buffer_overflow",HAL_OUT,&(hal_buffer_overflow->Pin),tpmod_id);

    hal_use_real_deviation = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    hal_pin_s32_new("tpmod.hal_use_real_deviation",HAL_IN,&(hal_use_real_deviation->Pin),tpmod_id);
}

// At runtime we can update values given from the gui.
static inline void tpUpdateHal(TP_STRUCT * const tp,
                               struct path_data *path,
                               struct emcmot_status_t *emc_status){

    *hal_tangential_knife_angle->Pin = path->tangential_knife_angle;
    *hal_segment_radius->Pin = path->segment_radius;

    path->enable_keyboard_jog = *hal_enable_keyboard_jog->Pin;
    path->jog_x_min = *hal_jog_x_min->Pin;
    path->jog_x_plus = *hal_jog_x_plus->Pin;
    path->jog_y_min = *hal_jog_y_min->Pin;
    path->jog_y_plus = *hal_jog_y_plus->Pin;
    path->jog_z_min = *hal_jog_z_min->Pin;
    path->jog_z_plus = *hal_jog_z_plus->Pin;

    // Changing the jerk at runtime is allowed.
    path->max_jerk=hal_max_jerk->Pin;
    path->max_jerk=fmax(path->max_jerk,1); // Lower limit.

    path->use_real_deviation = *hal_use_real_deviation->Pin;

    // Pin's to be monitored by halscope. These values can plot the scurve motion profile.
    *hal_tp_curvel->Pin=path->curvel; // Has vel spikes so not and then at end of motion using endevel.
    *hal_tp_curacc->Pin=path->curacc;
    *hal_tp_curpos->Pin=path->curpos;
    *hal_tp_tarpos->Pin=path->tarpos;
    *hal_tp_endvel->Pin=path->endvel;
    *hal_feed_override->Pin=emc_status->feed_scale;
    *hal_rapid_override->Pin=emc_status->rapid_scale;
    *hal_max_vel->Pin=path->maxvel;
    *hal_trajectory_end->Pin=path->trajectory_length;
    *hal_cycle->Pin=path->cycle;
    *hal_scurve_cycle_dist->Pin=path->cycle_travel;
    *hal_traject_buffer_dist->Pin=path->trajectory_length-path->curpos;
    *hal_segment_vm->Pin=path->maxvel;
    *hal_segment_ve->Pin=path->endvel;

    // Pin's to be monitored by halview.
    if(vector_size(vector_ptr)>0){
        *hal_index_motion_type->Pin=vector_at(vector_ptr,path->ringbuffer_index)->canon_motion_type;
    }
    *hal_index_progress->Pin=path->progress;

    *hal_ring_buffer_index->Pin=path->ringbuffer_index;
    *hal_global_buffer_index->Pin=path->global_index;
    *hal_index_size->Pin=vector_size(vector_ptr);
    *hal_push_counter->Pin=push_counter(vector_ptr);
    *hal_index_return_code->Pin=path->scurve_return;
    *hal_future_buffer->Pin=push_counter(vector_ptr)-path->global_index;
    *hal_motion_increments_a_cycle->Pin=path->motion_increments_a_cycle;
    *hal_buffer_overflow->Pin=path->buffer_overflow;

    if(tap_state==tap_fwd || tap_state==tap_rev){
        *hal_tap_synchronized_motion->Pin=1;
    } else {
        *hal_tap_synchronized_motion->Pin=0;
    }
}

#endif // HALSECTION_H
