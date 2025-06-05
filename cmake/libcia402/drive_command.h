#ifndef DRIVE_COMMAND_H
#define DRIVE_COMMAND_H

#include "hal.h"
#include "halsection.h"

static void drive_home_on_index(joint_data_t *joint);

// Function to check input values and prevent dividing by zero.
static void drive_check_scales(joint_data_t *joint){
    if( (*joint->var_pos_scale)<1e-20 && (*joint->var_pos_scale)>1e-20 ){
        *joint->var_pos_scale = 1;
        printf("drive pos scale set to 1. \n");
    }
    if( (*joint->var_vel_scale)<1e-20 && (*joint->var_vel_scale)>1e-20 ){
        *joint->var_vel_scale = 1;
        printf("drive vel scale set to 1. \n");
    }
}

// Write position command
static void drive_write_position(joint_data_t *joint){
    *joint->target_position = (hal_s32_t)( (*joint->pos_cmd) * (*joint->var_pos_scale) );
}

// Write velocity command
static void drive_write_velocity(joint_data_t *joint){
    *joint->target_velocity = (hal_s32_t)( (*joint->vel_cmd) * (*joint->var_vel_scale) );
}

// Write torque command
static void drive_write_torque(joint_data_t *joint){

    // Lower torque when searching for index pulse.
    // Lower torque when linuxcnc is doing a home sequence.
    if(*joint->index_enable || *joint->home_torque_enable){
         *joint->max_torque = *joint->var_max_torque_home_stop;
    } else {
        // At runtime, normal:
        *joint->max_torque = *joint->var_max_torque;
    }
}

// Function to write shutdown to the drive.
static void drive_shutdown(joint_data_t *joint){
    uint8_t mask;
    mask = (0 << 0) | (1 << 1) | (1 << 2) | (0 << 3) | (0 << 7);
    *joint->controlword = mask;
}

// Function to switch on drive.
static void drive_switch_on(joint_data_t *joint){
    uint8_t mask;
    mask = (1 << 0) | (1 << 1) | (1 << 2) | (0 << 3)  | (0 << 7);
    *joint->controlword = mask;
}

// Function to enable operation for drive.
static void drive_enable_operation(joint_data_t *joint){
    uint8_t mask;
    mask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (0 << 7);
    *joint->controlword = mask;
    *joint->opmode = *joint->var_opmode;
}

// Function to reset fault for driver.
static void drive_reset_fault(joint_data_t *joint){
    uint8_t mask;
    mask = (1 << 7);
    *joint->controlword = mask;
}

// Function to perform home sequence for drive.
// Home program: 0 for default, 33 for z pulse index.
static void drive_do_home(joint_data_t *joint, int home_program){
    uint8_t mask;
    mask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (0 << 7);
    *joint->controlword = mask;
    *joint->opmode = OPMODE_HOMING;
    *joint->homing_method = home_program;
}

// Function to set homed for drive nr i.
static void drive_set_homed(joint_data_t *joint){
    uint8_t mask;
    mask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (0 << 4) | (0 << 7);
    *joint->controlword = mask;
    *joint->opmode = *joint->var_opmode;
}

// Function to home drive on index z pulse.
static void drive_home(joint_data_t *joint){

    double home_delay   = 0.1;      // Delay in seconds.

    switch (joint->home_struct.hs) {
    case HOME_NONE:
        break;
    case HOME_INIT:
        printf("home init joint: %d \n",joint->joint_nr);
        drive_do_home(joint,*joint->var_home_program);
        joint->home_struct.hs = HOME_DELAY;
        joint->home_struct.delay = 0;
        joint->home_struct.home_busy_message = 0;
        *joint->stat_homing=1;
        *joint->stat_homed=0;
        break;
    case HOME_DELAY:
        if(!joint->home_struct.home_busy_message){
            printf("home busy joint: %d \n",joint->joint_nr);
            joint->home_struct.home_busy_message=1;
        }
        joint->home_struct.delay+=0.001;
        if(joint->home_struct.delay>home_delay){
            joint->home_struct.hs = HOME_BUSY;
        }
        break;
    case HOME_BUSY:
        if(*joint->stat_bit_12 && *joint->stat_target_reached || *joint->var_home_program==0){
            joint->home_struct.hs = HOME_FINISHED;
        } else {
            joint->home_struct.hs = HOME_BUSY;
        }
        break;
    case HOME_FINISHED:
        printf("home finished joint: %d \n",joint->joint_nr);
        joint->home_struct.hs = HOME_NONE;
        joint->runmode = RUN_NONE;
        // Set opmode back to position.
        drive_set_homed(joint);

        *joint->stat_homing=0;
        *joint->stat_homed=1;

        *joint->index_enable = 0;
        joint->index_enable_trigger = 0;

        break;
    default:
        break;
    }
}

#endif // DRIVE_COMMAND_H
