/********************************************************************
*   Written by Grotius, alias Skynet
*
* Author: Michel Wijnja
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2024 All rights reserved.
********************************************************************/

#include "rtapi.h"
#include "rtapi_ctype.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "rtapi_errno.h"
#include "rtapi_math64.h"
#include "rtapi_io.h"
#include "hal.h"
#include "stdio.h"

typedef struct {
    bool ok;
} skynet_t;
skynet_t *skynet;

typedef struct {
    hal_float_t *Pin;
} float_data_t;
float_data_t *pos_cmd;
float_data_t *pos_fb;
float_data_t *velocity_cmd;
float_data_t *velocity_fb;

//! Pins
typedef struct {
    hal_bit_t *Pin;
} bit_data_t;
bit_data_t *enable;
bit_data_t *home;
bit_data_t *fault_reset;
bit_data_t *opmode_cyclic_position;
bit_data_t *opmode_cyclic_velocity;
bit_data_t *opmode_no_mode;
bit_data_t *opmode_homing;
bit_data_t *stat_homing;
bit_data_t *stat_homed;
bit_data_t *stat_switchon_ready;
bit_data_t *stat_switched_on;
bit_data_t *stat_op_enabled;
bit_data_t *stat_fault;
bit_data_t *stat_voltage_enabled;
bit_data_t *stat_quick_stop;
bit_data_t *stat_switchon_disabled;
bit_data_t *stat_warning;
bit_data_t *stat_remote;
bit_data_t *stat_target_reached;
bit_data_t *stat_internal_limit;
bit_data_t *stat_acknowledged;
bit_data_t *stat_following_error;
bit_data_t *stat_referenced;
bit_data_t *drv_fault;

typedef struct {
    hal_u32_t *Pin;
} u32_data_t;
u32_data_t *statusword;
u32_data_t *controlword;

typedef struct {
    hal_s32_t *Pin;
} s32_data_t;
s32_data_t *opmode;
s32_data_t *opmode_display;
s32_data_t *drv_actual_position;
s32_data_t *drv_actual_velocity;
s32_data_t *drv_target_position;
s32_data_t *drv_target_velocity;

typedef struct {
    hal_port_t *Pin;
} port_data_t;
port_data_t *port;

//! Params
typedef struct {
    hal_float_t Pin;
} param_float_data_t;
param_float_data_t *pos_scale;
param_float_data_t *velo_scale;

typedef struct {
    hal_bit_t Pin;
} param_bit_data_t;
param_bit_data_t *auto_fault_reset;
param_bit_data_t *csp_mode;

// To clean hal environment after runtest :
// ~/linuxcnc_scurve_compact/scripts$ ./halrun -U

/* module information */
MODULE_AUTHOR("Michel Wijnja");
MODULE_DESCRIPTION("Halmodule cia402");
MODULE_LICENSE("GPL");

static int comp_idx;            // Component ID

//constants
#define FAULT_AUTORESET_DELAY_NS 100000000LL
#define OPMODE_CYCLIC_POSITION 8
#define OPMODE_CYCLIC_VELOCITY 9
#define OPMODE_HOMING 6
#define OPMODE_NONE 0

// Function defenitions.
static void read_all();
static void write_all();
static int setup_pins();

float pos_scale_old;
float velo_scale_old;
bool enable_old;
bool stat_homed_old;
bool stat_fault_old;
double pos_scale_rcpt;
double velo_scale_rcpt;
bool pos_mode;
bool init_pos_mode;
long auto_fault_reset_delay;

void init_vars(){
    // initialize variables
    pos_scale->Pin = 1.0;
    pos_scale_old = pos_scale->Pin + 1.0;
    pos_scale_rcpt = 1.0;
    velo_scale->Pin = 1.0;
    velo_scale_old = velo_scale->Pin + 1.0;
    velo_scale_rcpt = 1.0;
    auto_fault_reset->Pin = 1;
    csp_mode->Pin = 1;
}

void check_scales(hal_float_t *scale,float *scale_old, double *scale_rcpt) {
  // check for change in scale value
  if (*scale != *scale_old) {
    // scale value has changed, test and update it
    if ((*scale < 1e-20) && (*scale > -1e-20)) {
      // value too small, divide by zero is a bad thing
      *scale = 1.0;
    }
    // save new scale to detect future changes
    *scale_old = *scale;
    // we actually want the reciprocal
    *scale_rcpt = 1.0 / *scale;
  }
}

// Dlopen ..
int rtapi_app_main(void) {

    int r = 0;
    comp_idx = hal_init("cia402");
    if(comp_idx < 0) return comp_idx;

    skynet = hal_malloc(sizeof(skynet_t));
    r = hal_export_funct("cia402.read-all", read_all, &skynet,0,0,comp_idx);
    r = hal_export_funct("cia402.write-all", write_all, &skynet,0,0,comp_idx);

    r+=setup_pins();

    if(r) {
        hal_exit(comp_idx);
    } else {
        hal_ready(comp_idx);
    }

    init_vars();

    return 0;
}

void rtapi_app_exit(void){
    hal_exit(comp_idx);
}

double hartbeat = 0;

// Update from servo-cycle.
static void read_all(){
    // check for change in scale value
     check_scales(&pos_scale->Pin, &pos_scale_old, &pos_scale_rcpt);
     check_scales(&velo_scale->Pin, &velo_scale_old, &velo_scale_rcpt);

     // read position feedback
     *pos_fb->Pin = ((double)*drv_actual_position->Pin) * pos_scale_rcpt;

     // read velocity feedback
     *velocity_fb->Pin = ((double)*drv_actual_velocity->Pin) * velo_scale_rcpt;

     // read Modes of Operation
     *opmode_no_mode->Pin = (*opmode_display->Pin == OPMODE_NONE);
     *opmode_homing->Pin = (*opmode_display->Pin == OPMODE_HOMING);
     *opmode_cyclic_velocity->Pin = (*opmode_display->Pin == OPMODE_CYCLIC_VELOCITY);
     *opmode_cyclic_position->Pin = (*opmode_display->Pin == OPMODE_CYCLIC_POSITION);

     // read status
     *stat_switchon_ready->Pin    = (*statusword->Pin >> 0) & 1;
     *stat_switched_on->Pin       = (*statusword->Pin >> 1) & 1;
     *stat_op_enabled->Pin        = (*statusword->Pin >> 2) & 1;
     *stat_fault->Pin             = (*statusword->Pin >> 3) & 1;
     *stat_voltage_enabled->Pin   = (*statusword->Pin >> 4) & 1;
     *stat_quick_stop->Pin        = (*statusword->Pin >> 5) & 1;
     *stat_switchon_disabled->Pin = (*statusword->Pin >> 6) & 1;
     *stat_warning->Pin           = (*statusword->Pin >> 7) & 1;
     *stat_remote->Pin            = (*statusword->Pin >> 9) & 1;

     if (opmode_cyclic_position || opmode_cyclic_velocity) {
       *stat_target_reached->Pin = (*statusword->Pin >> 10) & 1;
       } else {
         *stat_target_reached->Pin = 0;
       }

     //home states
     if (*opmode_homing->Pin) {
       *stat_homed->Pin    = ((*statusword->Pin >> 10) & 1) && ((*statusword->Pin >> 12) & 1);
       *stat_homing->Pin   = !stat_homed && !((*statusword->Pin >> 10) & 1);
     }

     // update fault output
     if (auto_fault_reset_delay > 0) {
       double period = 1;
       auto_fault_reset_delay -= period;
       *drv_fault->Pin = 0;
     } else {
       *drv_fault->Pin = *stat_fault->Pin && *enable->Pin;
     }
}

static void write_all(){

    int enable_edge;

     //init opmode
     if (!init_pos_mode) {
       pos_mode = csp_mode->Pin;
       init_pos_mode = 1;
     }

     // detect enable edge
     enable_edge = *enable->Pin && !enable_old;
     enable_old = *enable->Pin;

     // write control register
     *controlword->Pin = (1 << 2); // quick stop is not supported
     if (*stat_fault->Pin) {
       home = 0;
       if (*fault_reset->Pin) {
         *controlword->Pin |= (1 << 7); // fault reset
       }
       if (auto_fault_reset && enable_edge) {
         auto_fault_reset_delay = FAULT_AUTORESET_DELAY_NS;
         *controlword->Pin |= (1 << 7); // fault reset
       }
     } else {
       if (*enable->Pin) {
         *controlword->Pin |= (1 << 1); // enable voltage
         if (*stat_switchon_ready->Pin) {
           *controlword->Pin |= (1 << 0); // switch on
           if (*stat_switched_on->Pin) {
             *controlword->Pin |= (1 << 3); // enable op
           }
         }
       }
     }

     // write position command
     *drv_target_position->Pin = (int32_t) (*pos_cmd->Pin * pos_scale->Pin);
     // write velocity command
     *drv_target_velocity->Pin = (int32_t) (*velocity_cmd->Pin * velo_scale->Pin);

     // reset home command
     if (*home->Pin && (*stat_homed->Pin && !stat_homed_old) && *opmode_homing->Pin) {
       *home->Pin = 0;
     }
     stat_homed_old = *stat_homed->Pin;

     // OP Mode
     // set to position mode
     if (*stat_voltage_enabled->Pin && !*home->Pin ) {
       *opmode->Pin = OPMODE_CYCLIC_POSITION;
     }
     // set velo mode
     if (*stat_voltage_enabled->Pin && !pos_mode && !*home->Pin) {
       *opmode->Pin = OPMODE_CYCLIC_VELOCITY;
     }
     // mode Home and start homing
     if (*home->Pin) {
       *opmode->Pin = OPMODE_HOMING;
       *controlword->Pin |= (*home->Pin << 4);
     }
}

static int setup_pins(){
    int r=0;

    // Bit pins.
    enable = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.enable",HAL_IN,&(enable->Pin),comp_idx);

    home = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.home",HAL_IO,&(home->Pin),comp_idx);

    fault_reset = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.drv-fault-reset",HAL_IN,&(fault_reset->Pin),comp_idx);

    drv_fault = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.drv-fault",HAL_OUT,&(drv_fault->Pin),comp_idx);

    opmode_cyclic_position = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.opmode-cyclic-position",HAL_OUT,&(opmode_cyclic_position->Pin),comp_idx);

    opmode_cyclic_velocity = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.opmode-cyclic-velocity",HAL_OUT,&(opmode_cyclic_velocity->Pin),comp_idx);

    opmode_no_mode = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.opmode-no-mode",HAL_OUT,&(opmode_no_mode->Pin),comp_idx);

    opmode_homing = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.opmode-homing",HAL_OUT,&(opmode_homing->Pin),comp_idx);

    stat_homed = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat-homed",HAL_OUT,&(stat_homed->Pin),comp_idx);

    stat_homing = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat-homing",HAL_OUT,&(stat_homing->Pin),comp_idx);

    stat_switchon_ready = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat-switchon-ready",HAL_OUT,&(stat_switchon_ready->Pin),comp_idx);

    stat_switched_on = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat-switched-on",HAL_OUT,&(stat_switched_on->Pin),comp_idx);

    stat_op_enabled = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat-op-enabled",HAL_OUT,&(stat_op_enabled->Pin),comp_idx);

    stat_fault = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat-fault",HAL_OUT,&(stat_fault->Pin),comp_idx);

    stat_voltage_enabled = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat_voltage_enabled",HAL_OUT,&(stat_voltage_enabled->Pin),comp_idx);

    stat_quick_stop = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat-quick-stop",HAL_OUT,&(stat_quick_stop->Pin),comp_idx);

    stat_switchon_disabled = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat-switchon-disabled",HAL_OUT,&(stat_switchon_disabled->Pin),comp_idx);

    stat_warning = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat-warning",HAL_OUT,&(stat_warning->Pin),comp_idx);

    stat_remote = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat-remote",HAL_OUT,&(stat_remote->Pin),comp_idx);

    stat_target_reached = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat-target-reached",HAL_OUT,&(stat_target_reached->Pin),comp_idx);

    stat_internal_limit = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat-internal-limit",HAL_OUT,&(stat_internal_limit->Pin),comp_idx);

    stat_acknowledged = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat-acknowledged",HAL_OUT,&(stat_acknowledged->Pin),comp_idx);

    stat_following_error = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat_following_error",HAL_OUT,&(stat_following_error->Pin),comp_idx);

    stat_referenced = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cia402.stat-referenced",HAL_OUT,&(stat_referenced->Pin),comp_idx);

    // Float pins.
    pos_cmd = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("cia402.pos-cmd",HAL_IN,&(pos_cmd->Pin),comp_idx);

    pos_fb = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("cia402.pos-fb",HAL_OUT,&(pos_fb->Pin),comp_idx);

    velocity_fb = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("cia402.velocity-fb",HAL_OUT,&(velocity_fb->Pin),comp_idx);

    velocity_cmd = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("cia402.velocity-cmd",HAL_OUT,&(velocity_cmd->Pin),comp_idx);

    // Param pins.
    auto_fault_reset = (param_bit_data_t*)hal_malloc(sizeof(param_bit_data_t));
    r+=hal_param_bit_new("cia402.auto-fault-reset",HAL_RW,&(auto_fault_reset->Pin),comp_idx);

    csp_mode = (param_bit_data_t*)hal_malloc(sizeof(param_bit_data_t));
    r+=hal_param_bit_new("cia402.csp-mode",HAL_RW,&(csp_mode->Pin),comp_idx);

    pos_scale = (param_float_data_t*)hal_malloc(sizeof(param_float_data_t));
    r+=hal_param_float_new("cia402.pos-scale",HAL_RW,&(pos_scale->Pin),comp_idx);

    velo_scale = (param_float_data_t*)hal_malloc(sizeof(param_float_data_t));
    r+=hal_param_float_new("cia402.velo-scale",HAL_RW,&(velo_scale->Pin),comp_idx);

    // U32 pins.
    statusword = (u32_data_t*)hal_malloc(sizeof(u32_data_t));
    r+=hal_pin_u32_new("cia402.statusword",HAL_IN,&(statusword->Pin),comp_idx);

    controlword = (u32_data_t*)hal_malloc(sizeof(u32_data_t));
    r+=hal_pin_u32_new("cia402.controlword",HAL_OUT,&(controlword->Pin),comp_idx);

    // S32 pins.
    opmode= (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    r+=hal_pin_s32_new("cia402.opmode",HAL_OUT,&(opmode->Pin),comp_idx);

    opmode_display = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    r+=hal_pin_s32_new("cia402.opmode-display",HAL_IN,&(opmode_display->Pin),comp_idx);

    drv_actual_position = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    r+=hal_pin_s32_new("cia402.drv-actual-position",HAL_IN,&(drv_actual_position->Pin),comp_idx);

    drv_actual_velocity = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    r+=hal_pin_s32_new("cia402.drv-actual-velocity",HAL_IN,&(drv_actual_velocity->Pin),comp_idx);

    drv_target_position = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    r+=hal_pin_s32_new("cia402.drv-target-position",HAL_OUT,&(drv_target_position->Pin),comp_idx);

    drv_target_velocity = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    r+=hal_pin_s32_new("cia402.drv-target-velocity",HAL_OUT,&(drv_target_velocity->Pin),comp_idx);

    return r;
}






































