/********************************************************************
*   Written by Julian Koning, alias ..
*
* Author: Julian Koning
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
#include "halsection.h"
#include "halpins.h"
#include "drive_command.h"

// To clean hal environment after runtest :
// ~/linuxcnc_scurve_compact/scripts$ ./halrun -U

/* module information */
MODULE_AUTHOR("Michel Wijnja");
MODULE_DESCRIPTION("Halmodule cia402");
MODULE_LICENSE("GPL");

static int comp_idx;            // Component ID
static int count;               // How much servo's to use.
RTAPI_MP_INT(count, "number of servo drives");

// Function defenitions.
static void read_all();
static void write_all();
static joint_data_t *jd = NULL;  // Initialize to NULL

// Dlopen ..
int rtapi_app_main(void) {
    int r = 0;
    comp_idx = hal_init("cia402");
    if(comp_idx < 0) return comp_idx;

    r = hal_export_funct("cia402.read-all", read_all, NULL, 0, 0, comp_idx);
    r = hal_export_funct("cia402.write-all", write_all, NULL, 0, 0, comp_idx);
    if(r) {
        hal_exit(comp_idx);
        return r;
    }

    // Allocate memory - use the global jd, not a local one
    jd = allocate_joint_data(count);
    if (!jd) {
        hal_exit(comp_idx);
        return -1;
    }

    r = setup_pins(jd, count, comp_idx);
    if(r) {
        jd = NULL;
        hal_exit(comp_idx);
        return r;
    }

    hal_ready(comp_idx);
    return 0;
}

void rtapi_app_exit(void) {
    if(jd) {
        // hal_free(jd);
        jd = NULL;
    }
    hal_exit(comp_idx);
}

void read_drive_status(joint_data_t *joint) {

    // Check input values for pos & vel scale.
    drive_check_scales(joint);

    // Update run timer.
    *joint->stat_runtime += 0.001;

    // Get pos & vel feedback.
    *joint->pos_fb = (double)(*joint->actual_position) * (1/(double)(*joint->var_pos_scale));
    *joint->vel_fb = (double)(*joint->actual_velocity) * (1/(double)(*joint->var_vel_scale));

    // Read Modes of Operation
    *joint->stat_opmode_no_mode = (*joint->opmode_display == OPMODE_NONE);
    *joint->stat_opmode_homing = (*joint->opmode_display == OPMODE_HOMING);
    *joint->stat_opmode_cyclic_velocity = (*joint->opmode_display == OPMODE_CYCLIC_VELOCITY);
    *joint->stat_opmode_cyclic_position = (*joint->opmode_display == OPMODE_CYCLIC_POSITION);

    // Read status
    *joint->stat_switchon_ready      = (*joint->statusword >> 0) & 1;
    *joint->stat_switched_on         = (*joint->statusword >> 1) & 1;
    *joint->stat_op_enabled          = (*joint->statusword >> 2) & 1;
    *joint->stat_fault               = (*joint->statusword >> 3) & 1;
    *joint->stat_voltage_enabled     = (*joint->statusword >> 4) & 1;
    *joint->stat_quick_stop          = (*joint->statusword >> 5) & 1;
    *joint->stat_switchon_disabled   = (*joint->statusword >> 6) & 1;
    *joint->stat_warning             = (*joint->statusword >> 7) & 1;
    *joint->stat_remote              = (*joint->statusword >> 9) & 1;
    *joint->stat_target_reached      = (*joint->statusword >> 10) & 1;
    *joint->stat_bit_11              = (*joint->statusword >> 11) & 1;
    *joint->stat_bit_12              = (*joint->statusword >> 12) & 1;
    *joint->stat_bit_13              = (*joint->statusword >> 13) & 1;
    *joint->stat_positive_limit      = (*joint->statusword >> 14) & 1;
    *joint->stat_negative_limit      = (*joint->statusword >> 15) & 1;

    // Set the enum status based on the status bits.
    int bit0 = (*joint->statusword >> 0) & 1;
    int bit1 = (*joint->statusword >> 1) & 1;
    int bit2 = (*joint->statusword >> 2) & 1;
    int bit3 = (*joint->statusword >> 3) & 1;
    int bit4 = (*joint->statusword >> 4) & 1;
    int bit5 = (*joint->statusword >> 5) & 1;
    int bit6 = (*joint->statusword >> 6) & 1;

    int bit10 = (*joint->statusword >> 10) & 1;
    int bit12 = (*joint->statusword >> 12) & 1;

    if(!bit0 && !bit1 && !bit2 && !bit3 && !bit6){
        joint->drive_state = NOT_READY_TO_SWITCH_ON;
    }
    if(!bit0 && !bit1 && !bit2 && !bit3 && !bit6){
        joint->drive_state = SWITCH_ON_DISABLED;
    }
    if(bit0 && !bit1 && !bit2 && !bit3 && bit5 && !bit6){
        joint->drive_state = READY_TO_SWITCH_ON;
    }
    if(bit0 && bit1 && !bit2 && !bit3 && bit5 && !bit6){
        joint->drive_state = SWITCH_ON;
    }
    if(bit0 && bit1 && bit2 && !bit3 && bit5 && !bit6){
        joint->drive_state = OPERATION_ENABLED;
    }
    if(bit0 && bit1 && bit2 && !bit3 && !bit5 && !bit6){
        joint->drive_state = QUICK_STOP_ACTIVE;
    }
    if(bit0 && bit1 && bit2 && bit3 && !bit6){
        joint->drive_state = FAULT_REACTION_ACTIVE;
    }
    if(!bit0 && !bit1 && !bit2 && bit3 && !bit6){
        joint->drive_state = SERVO_FAULT;
    }
}

// Function when drive is in state : Operation enabled.
void drive_run(joint_data_t *joint){

    // Check for inputs.
    if(*joint->home){
        printf("home request. \n");

        joint->runmode = RUN_HOME;
        joint->home_struct.hs = HOME_INIT;
        *joint->home=0;
    }

    switch (joint->runmode) {
    case RUN_NONE:

        break;
    case RUN_POSITION:

        break;
    case RUN_VELOCITY:

        break;
    case RUN_HOME:
        drive_home(joint);
        break;

    default:
        break;
    }
}

// Servo drive cia402 state machine workflow.
void drive_state_machine(joint_data_t *joint){

    switch (joint->enum_fault) {
    case FAULT_OK:
        // Go on.
        break;
    case FAULT_SHUTDOWN_DRIVE:
        drive_shutdown(joint);
        joint->enum_fault = FAULT_OK;
        return;
    case FAULT_ACTIVE:
        // Reset fault.
        drive_reset_fault(joint);
        joint->enum_fault = FAULT_SHUTDOWN_DRIVE;
        return;
    default:
        break;
    }

    switch (joint->drive_state) {
    case NOT_READY_TO_SWITCH_ON:
    case SWITCH_ON_DISABLED:
        // Shutdown.
        drive_shutdown(joint);
        break;
    case READY_TO_SWITCH_ON:
        // Switch on.
        drive_switch_on(joint);
        break;
    case SWITCH_ON:
        // Enable operation.
        drive_enable_operation(joint);
        break;
    case OPERATION_ENABLED:
        // Drive is running.
        drive_run(joint);
        break;
    case QUICK_STOP_ACTIVE:
        // Hmm. What to do?
        break;
    case FAULT_REACTION_ACTIVE:
        joint->enum_fault = FAULT_ACTIVE;
        break;
    case SERVO_FAULT:
        joint->enum_fault = FAULT_ACTIVE;
        break;
    default:
        break;
    }
}

// Read from servo-cycle.
static void read_all() {

    if(!jd) return;  // Safety check

    for(int i = 0; i < count; i++) {
        joint_data_t *joint = &jd[i];
        read_drive_status(joint);
    }
}

// Write to servo-cycle.
static void write_all() {

    if(!jd) return;  // Safety check

    for(int i = 0; i < count; i++) {
        joint_data_t *joint = &jd[i];

        read_drive_status(joint);

        if(*joint->enable){
            drive_state_machine(joint);
            drive_write_position(joint);
            drive_write_velocity(joint);
            drive_write_torque(joint);

        } else {
            drive_shutdown(joint);
            *joint->pos_offset = *joint->pos_fb - *joint->pos_cmd;
        }
    }
}















