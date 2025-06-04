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
#include "drive_status.h"

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

    // Add component.
    comp_idx = hal_init("cia402");
    if(comp_idx < 0){
        return -1;
    }

    // Add read function.
    int r = hal_export_funct("cia402.read-all", read_all, NULL, 0, 0, comp_idx);
    if(r){
        hal_exit(comp_idx);
        printf("function read-all fails. \n");
        return r;
    }

    // Add write function.
    r = hal_export_funct("cia402.write-all", write_all, NULL, 0, 0, comp_idx);
    if(r){
        hal_exit(comp_idx);
        printf("function write-all fails. \n");
        return r;
    }

    // Allocate memory for multiple joints.
    jd = allocate_joint_data(count);
    if (!jd) {
        hal_exit(comp_idx);
        return -1;
    }

    // Add hal pins.
    r = setup_pins(jd, count, comp_idx);
    if(r) {
        jd = NULL;
        hal_exit(comp_idx);
        return r;
    }

    // Ok set ready.
    hal_ready(comp_idx);

    // Returns succes.
    return 0;
}

void rtapi_app_exit(void) {

    for(int i = 0; i < count; i++) {
        joint_data_t *joint = &jd[i];
        drive_shutdown(joint);
    }
    if(jd) {
        // hal_free(jd);
        jd = NULL;
    }
    hal_exit(comp_idx);
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

        if(*joint->enable){
            drive_state_machine(joint);
            drive_write_position(joint);
            drive_write_velocity(joint);
            drive_write_torque(joint);

        } else {
            drive_shutdown(joint);
            *joint->pos_offset = *joint->pos_fb_raw - *joint->pos_cmd;
            joint->home_struct.pos_cmd_offset = 0;
        }
    }
}















