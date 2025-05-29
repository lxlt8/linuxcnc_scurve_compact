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

// Single pins
typedef struct {
    hal_bit_t *Pin;
} bit_data_t;

typedef struct {
    hal_u32_t *Pin;
} u32_data_t;

typedef struct {
    hal_s32_t *Pin;
} s32_data_t;

typedef struct {
    hal_port_t *Pin;
} port_data_t;

// Single params
typedef struct {
    hal_float_t Pin;
} param_float_data_t;

typedef struct {
    hal_bit_t Pin;
} param_bit_data_t;

enum current_status {
    NOT_READY_TO_SWITCH_ON,
    SWITCH_ON_DISABLED,
    READY_TO_SWITCH_ON,
    SWITCH_ON,
    OPERATION_ENABLED,
    QUICK_STOP_ACTIVE,
    FAULT_REACTION_ACTIVE,
    SERVO_FAULT
};

enum current_opmode {
    HOMING = 6,
    NONE = 0,
    CYCLIC_POSITION = 8,
    CYCLIC_VELOCITY = 9
};

enum current_fault {
    OK,
    FAULT,
    SHUTDOWN
};

enum current_home {
    HOME_START,
    HOME_BUSY,
    HOME_READY
};

// Constants
#define FAULT_AUTORESET_DELAY_NS 100000000LL


typedef struct {

    enum current_fault enum_fault;
    enum current_home enum_home;

    // Pin.
    hal_float_t *pos_cmd;
    hal_float_t *pos_fb;
    hal_float_t *velocity_cmd;
    hal_float_t *velocity_fb;
    // Param.
    hal_float_t pos_scale;
    hal_float_t velo_scale;
    // Pin.

    hal_bit_t *enable;
    hal_bit_t *home;
    hal_bit_t *opmode_cyclic_position;
    hal_bit_t *opmode_cyclic_velocity;
    hal_bit_t *opmode_no_mode;
    hal_bit_t *opmode_homing;
    hal_bit_t *stat_homing;
    hal_bit_t *stat_homed;
    hal_bit_t *stat_switchon_ready;
    hal_bit_t *stat_switched_on;
    hal_bit_t *stat_op_enabled;
    hal_bit_t *stat_fault;
    hal_bit_t *stat_voltage_enabled;
    hal_bit_t *stat_quick_stop;
    hal_bit_t *stat_switchon_disabled;
    hal_bit_t *stat_warning;
    hal_bit_t *stat_remote;
    hal_bit_t *stat_target_reached;
    hal_bit_t *stat_internal_limit;
    hal_bit_t *stat_acknowledged;
    hal_bit_t *stat_following_error;
    hal_bit_t *stat_referenced;
    hal_bit_t *stat_positive_limit;
    hal_bit_t *stat_negative_limit;
    hal_bit_t *stat_bit_11;
    hal_bit_t *stat_bit_12;
    hal_bit_t *stat_bit_13;
    hal_bit_t *drv_fault;
    // Test.
    hal_bit_t switch_enable_voltage;
    hal_bit_t switch_on;
    hal_bit_t switch_enable_operation;
    hal_bit_t switch_pos;
    hal_bit_t switch_vel;
    hal_bit_t switch_home;
    hal_bit_t switch_shutdown;
    hal_bit_t switch_reset_fault;
    // Pin.
    hal_u32_t *statusword;
    hal_u32_t *controlword;
    // Param.
    hal_u32_t status;
    // Pin.
    hal_s32_t *opmode;                  // OUT.
    hal_s32_t *opmode_display;          // IN.
    hal_s32_t *drv_actual_position;
    hal_s32_t *drv_actual_velocity;
    hal_s32_t *drv_target_position;
    hal_s32_t *drv_target_velocity;
    // Param.
    hal_bit_t auto_fault_reset;
    hal_bit_t csp_mode;
    // Vars.
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

    int trigger0, trigger1, trigger2;

} joint_data_t;
joint_data_t *JD;



// To clean hal environment after runtest :
// ~/linuxcnc_scurve_compact/scripts$ ./halrun -U

/* module information */
MODULE_AUTHOR("Michel Wijnja");
MODULE_DESCRIPTION("Halmodule ethercat cia402");
MODULE_LICENSE("GPL");

static int comp_idx;    // Component ID.
static int count;       // How much servo's to use.
RTAPI_MP_INT(count, "number of servo drives");

// Function defenitions.
static void read_all();
static void write_all();
static int setup_pins(int count);

// initialize variables
void init_vars(){
    for(int i=0; i<count; i++){
        JD[i].pos_scale = 1.0;
        JD[i].pos_scale_old = JD[i].pos_scale + 1.0;
        JD[i].pos_scale_rcpt = 1.0;
        JD[i].velo_scale = 1.0;
        JD[i].velo_scale_old = JD[i].velo_scale + 1.0;
        JD[i].velo_scale_rcpt = 1.0;
        JD[i].auto_fault_reset = 1;
        JD[i].csp_mode = 1;

        JD[i].trigger0=0;
        JD[i].trigger1=0;
        JD[i].trigger2=0;

        JD[i].enum_fault = OK;
        JD[i].enum_home = HOME_READY;
    }
}

// check for change in scale value
void check_scales(hal_float_t *scale, float *scale_old, double *scale_rcpt) {
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
    // rtapi_print_msg(RTAPI_MSG_ERR, "count: %d\n", count);
    int r = 0;
    comp_idx = hal_init("cia402");
    if(comp_idx < 0) return comp_idx;

    skynet = hal_malloc(sizeof(skynet_t));
    r += hal_export_funct("cia402.read-all", read_all, &skynet,0,0,comp_idx);
    r += hal_export_funct("cia402.write-all", write_all, &skynet,0,0,comp_idx);
    r += setup_pins(count);

    if(r) {
        hal_exit(comp_idx);
    } else {
        hal_ready(comp_idx);
    }
    init_vars();
    return 0;
}

// Exit.
void rtapi_app_exit(void){
    hal_exit(comp_idx);
}

// Update from servo-cycle.
static void read_all(){

    for(int i=0; i<count; i++){

        // Check for change in scale value
        check_scales(&JD[i].pos_scale, &JD[i].pos_scale_old, &JD[i].pos_scale_rcpt);
        check_scales(&JD[i].velo_scale, &JD[i].velo_scale_old, &JD[i].velo_scale_rcpt);

        *JD[i].pos_fb = (double)*JD[i].drv_actual_position * JD[i].pos_scale_rcpt;
        // Read velocity feedback
        *JD[i].velocity_fb = ((double)*JD[i].drv_actual_velocity) * JD[i].velo_scale_rcpt;

        // Read Modes of Operation
        *JD[i].opmode_no_mode = (*JD[i].opmode_display == NONE);
        *JD[i].opmode_homing = (*JD[i].opmode_display == HOMING);
        *JD[i].opmode_cyclic_velocity = (*JD[i].opmode_display == CYCLIC_VELOCITY);
        *JD[i].opmode_cyclic_position = (*JD[i].opmode_display == CYCLIC_POSITION);

        // Read status
        *JD[i].stat_switchon_ready      = (*JD[i].statusword >> 0) & 1;
        *JD[i].stat_switched_on         = (*JD[i].statusword >> 1) & 1;
        *JD[i].stat_op_enabled          = (*JD[i].statusword >> 2) & 1;
        *JD[i].stat_fault               = (*JD[i].statusword >> 3) & 1;
        *JD[i].stat_voltage_enabled     = (*JD[i].statusword >> 4) & 1;
        *JD[i].stat_quick_stop          = (*JD[i].statusword >> 5) & 1;
        *JD[i].stat_switchon_disabled   = (*JD[i].statusword >> 6) & 1;
        *JD[i].stat_warning             = (*JD[i].statusword >> 7) & 1;
        *JD[i].stat_remote              = (*JD[i].statusword >> 9) & 1;
        *JD[i].stat_target_reached      = (*JD[i].statusword >> 10) & 1;
        *JD[i].stat_bit_11              = (*JD[i].statusword >> 11) & 1;
        *JD[i].stat_bit_12              = (*JD[i].statusword >> 12) & 1;
        *JD[i].stat_bit_13              = (*JD[i].statusword >> 13) & 1;
        *JD[i].stat_positive_limit      = (*JD[i].statusword >> 14) & 1;
        *JD[i].stat_negative_limit      = (*JD[i].statusword >> 15) & 1;

        // Set the enum status based on the status bits.
        int bit0 = (*JD[i].statusword >> 0) & 1;
        int bit1 = (*JD[i].statusword >> 1) & 1;
        int bit2 = (*JD[i].statusword >> 2) & 1;
        int bit3 = (*JD[i].statusword >> 3) & 1;
        int bit4 = (*JD[i].statusword >> 4) & 1;
        int bit5 = (*JD[i].statusword >> 5) & 1;
        int bit6 = (*JD[i].statusword >> 6) & 1;

        int bit10 = (*JD[i].statusword >> 10) & 1;
        int bit12 = (*JD[i].statusword >> 12) & 1;

        if(!bit0 && !bit1 && !bit2 && !bit3 && !bit6){
            JD[i].status = NOT_READY_TO_SWITCH_ON;
        }
        if(!bit0 && !bit1 && !bit2 && !bit3 && !bit6){
            JD[i].status = SWITCH_ON_DISABLED;
        }
        if(bit0 && !bit1 && !bit2 && !bit3 && bit5 && !bit6){
            JD[i].status = READY_TO_SWITCH_ON;
        }
        if(bit0 && bit1 && !bit2 && !bit3 && bit5 && !bit6){
            JD[i].status = SWITCH_ON;
        }
        if(bit0 && bit1 && bit2 && !bit3 && bit5 && !bit6){
            JD[i].status = OPERATION_ENABLED;
        }
        if(bit0 && bit1 && bit2 && !bit3 && !bit5 && !bit6){
            JD[i].status = QUICK_STOP_ACTIVE;
        }
        if(bit0 && bit1 && bit2 && bit3 && !bit6){
            JD[i].status = FAULT_REACTION_ACTIVE;
        }
        if(!bit0 && !bit1 && !bit2 && bit3 && !bit6){
            JD[i].status = SERVO_FAULT;
        }

        if(bit10 && bit12){
            *JD[i].stat_homed = 1;
        } else {
            *JD[i].stat_homed = 0;
        }

        if(*JD[i].opmode_homing && !bit10 && bit12){
            *JD[i].stat_homing = 1;
        } else {
            *JD[i].stat_homing = 0;
        }
    }
}

// Function to write shutdown to the drive nr i.
void inline shutdown_(int i){
    uint8_t mask;
    mask = (0 << 0) | (1 << 1) | (1 << 2) | (0 << 3) | (0 << 7);
    *JD[i].controlword = mask;
}

// Function to switch on drive nr i.
void inline switch_on_(int i){
    uint8_t mask;
    mask = (1 << 0) | (1 << 1) | (1 << 2) | (0 << 3)  | (0 << 7);
    *JD[i].controlword = mask;
}

// Function to enable operation for drive nr i.
void inline enable_operation_(int i){
    uint8_t mask;
    mask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (0 << 7);
    *JD[i].controlword = mask;
}

// Function to reset fault for drive nr i.
void inline reset_fault_(int i){
    uint8_t mask;
    mask = (1 << 7);
    *JD[i].controlword = mask;
}

// Function to perform home sequence for drive nr i.
// Here the servo feedback gives error, because it want to change position by itself.
void inline home_(int i){
    uint8_t mask;
    // mask = (1 << 4);

    mask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (0 << 7);
    *JD[i].controlword = mask;
    JD[i].enum_home = HOME_BUSY;
}

// Function to set homed for drive nr i.
void inline set_homed_(int i){
    uint8_t mask;
    mask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (0 << 4) | (0 << 7);
    *JD[i].controlword = mask;
    JD[i].enum_home = HOME_READY;
}

// Function to write position & velocity to drive nr i.
void inline write_position_velocity_(int i){
    // Write position command
    *JD[i].drv_target_position = (int32_t) (*JD[i].pos_cmd * JD[i].pos_scale);
    // Write velocity command
    *JD[i].drv_target_velocity = (int32_t) (*JD[i].velocity_cmd * JD[i].velo_scale);
}

static void write_all(){

    for(int i=0; i<count; i++){

        switch (JD[i].enum_fault) {
        case OK:
            // Go on.
            break;
        case SHUTDOWN:
            shutdown_(i);
            JD[i].enum_fault = OK;
            return;
        case FAULT:
            // Reset fault.
            reset_fault_(i);
            JD[i].enum_fault = SHUTDOWN;
            return;
        default:
            break;
        }

        switch (JD[i].enum_home) {
        case HOME_START:
            home_(i);
            break;
        case HOME_BUSY:
            // set_homed_(i);
            // Don't update the position feedback to other values.
            break;
        case HOME_READY:
            // Do nothing.
            break;
        default:
            break;
        }

        switch (JD[i].status) {
        case NOT_READY_TO_SWITCH_ON:
        case SWITCH_ON_DISABLED:
            // Shutdown.
            shutdown_(i);
            break;
        case READY_TO_SWITCH_ON:
            // Switch on.
            switch_on_(i);
            break;
        case SWITCH_ON:
            // Enable operation.
            enable_operation_(i);
            break;
        case OPERATION_ENABLED:
            // Drive is running.
            // Set to position mode, velocity mode.

            if (JD[i].switch_home) {
                *JD[i].opmode = HOMING;
                JD[i].enum_home = HOME_START;
            } else if(JD[i].switch_pos){
                *JD[i].opmode = CYCLIC_POSITION;
            } else if(JD[i].switch_vel){
                *JD[i].opmode = CYCLIC_VELOCITY;
            } else {
                *JD[i].opmode = NONE;
            }

            break;
        case QUICK_STOP_ACTIVE:
            // Hmm. What to do?
            break;
        case FAULT_REACTION_ACTIVE:
            JD[i].enum_fault = FAULT;
            break;
        case SERVO_FAULT:
            JD[i].enum_fault = FAULT;
            break;
        default:
            break;
        }

        // Write position command
        write_position_velocity_(i);
    }
}

static int setup_pins(int count){
    int r=0;
    char pin_name[64]; // Store name.
    JD = hal_malloc(sizeof(joint_data_t) * count); // Allocate memory.

    for(int i=0; i<count; i++){

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.enable", i);
        r+= hal_pin_bit_new(pin_name, HAL_IN, &(JD[i].enable), comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.switch-reset-fault", i);
        r+= hal_param_bit_new(pin_name, HAL_RW, &(JD[i].switch_reset_fault), comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.switch-home", i);
        r+= hal_param_bit_new(pin_name, HAL_RW, &(JD[i].switch_home), comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.switch-vel", i);
        r+= hal_param_bit_new(pin_name, HAL_RW, &(JD[i].switch_vel), comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.switch-pos", i);
        r+= hal_param_bit_new(pin_name, HAL_RW, &(JD[i].switch_pos), comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.switch-enable-operation", i);
        r+= hal_param_bit_new(pin_name, HAL_RW, &(JD[i].switch_enable_operation), comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.switch-on", i);
        r+= hal_param_bit_new(pin_name, HAL_RW, &(JD[i].switch_on), comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.switch-enable-voltage", i);
        r+= hal_param_bit_new(pin_name, HAL_RW, &(JD[i].switch_enable_voltage), comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.home", i);
        r+= hal_pin_bit_new(pin_name, HAL_IO, &(JD[i].home), comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.switch-shutdown", i);
        r+=hal_param_bit_new(pin_name,HAL_RW,&(JD[i].switch_shutdown),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.drv-fault", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].drv_fault),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.opmode-cyclic-position", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].opmode_cyclic_position),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.opmode-cyclic-velocity", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].opmode_cyclic_velocity),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.opmode-no-mode", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].opmode_no_mode),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.opmode-homing", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].opmode_homing),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-homed", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_homed),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-homing", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_homing),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-switchon-ready", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_switchon_ready),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-switched-on", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_switched_on),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-op-enabled", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_op_enabled),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-fault", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_fault),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat_voltage_enabled", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_voltage_enabled),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-quick-stop", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_quick_stop),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-switchon-disabled", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_switchon_disabled),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-warning", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_warning),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-remote", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_remote),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-target-reached", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_target_reached),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-internal-limit", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_internal_limit),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-acknowledged", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_acknowledged),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat_following_error", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_following_error),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-referenced", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_referenced),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-positive-limit", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_positive_limit),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-negative-limit", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_negative_limit),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-bit-11", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_bit_11),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-bit-12", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_bit_12),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-bit-13", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(JD[i].stat_bit_13),comp_idx);

        // Float pins.
        snprintf(pin_name, sizeof(pin_name), "cia402.%d.pos-cmd", i);
        r+=hal_pin_float_new(pin_name,HAL_IN,&(JD[i].pos_cmd),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.pos-fb", i);
        r+=hal_pin_float_new(pin_name,HAL_OUT,&(JD[i].pos_fb),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.velocity-fb", i);
        r+=hal_pin_float_new(pin_name,HAL_OUT,&(JD[i].velocity_fb),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.velocity-cmd", i);
        r+=hal_pin_float_new(pin_name,HAL_OUT,&(JD[i].velocity_cmd),comp_idx);

        // Param pins.
        snprintf(pin_name, sizeof(pin_name), "cia402.%d.auto-fault-reset", i);
        r+=hal_param_bit_new(pin_name,HAL_RW,&(JD[i].auto_fault_reset),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.csp-mode", i);
        r+=hal_param_bit_new(pin_name,HAL_RW,&(JD[i].csp_mode),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.pos-scale", i);
        r+=hal_param_float_new(pin_name,HAL_RW,&(JD[i].pos_scale),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.velo-scale", i);
        r+=hal_param_float_new(pin_name,HAL_RW,&(JD[i].velo_scale),comp_idx);

        // U32 pins.
        snprintf(pin_name, sizeof(pin_name), "cia402.%d.statusword", i);
        r+=hal_pin_u32_new(pin_name,HAL_IN,&(JD[i].statusword),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.controlword", i);
        r+=hal_pin_u32_new(pin_name,HAL_OUT,&(JD[i].controlword),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.status", i);
        r+=hal_param_u32_new(pin_name,HAL_RW,&(JD[i].status),comp_idx);

        // S32 pins.
        snprintf(pin_name, sizeof(pin_name), "cia402.%d.opmode", i);
        r+=hal_pin_s32_new(pin_name,HAL_OUT,&(JD[i].opmode),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.opmode-display", i);
        r+=hal_pin_s32_new(pin_name,HAL_IN,&(JD[i].opmode_display),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.drv-actual-position", i);
        r+=hal_pin_s32_new(pin_name,HAL_IN,&(JD[i].drv_actual_position),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.drv-actual-velocity", i);
        r+=hal_pin_s32_new(pin_name,HAL_IN,&(JD[i].drv_actual_velocity),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.drv-target-position", i);
        r+=hal_pin_s32_new(pin_name,HAL_OUT,&(JD[i].drv_target_position),comp_idx);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.drv-target-velocity", i);
        r+=hal_pin_s32_new(pin_name,HAL_OUT,&(JD[i].drv_target_velocity),comp_idx);
    }
    return r;
}






































