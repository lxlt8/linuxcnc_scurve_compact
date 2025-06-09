#ifndef DRIVE_COMMAND_H
#define DRIVE_COMMAND_H

#include "hal.h"
#include "halsection.h"

void drive_home_on_index(cia_joint_data_t *joint);

// Write position command
void cia_drive_write_position(cia_joint_data_t *joint){
    *joint->target_position = (hal_s32_t)( (*joint->pos_cmd) * (*joint->var_pos_scale) );
}

// Write velocity command
void cia_drive_write_velocity(cia_joint_data_t *joint){
    *joint->target_velocity = (hal_s32_t)( (*joint->vel_cmd) * (*joint->var_vel_scale) );
}

// Write torque command
void cia_drive_write_torque(cia_joint_data_t *joint){

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
void cia_drive_shutdown(cia_joint_data_t *joint){
    uint8_t mask;
    mask = (0 << 0) | (1 << 1) | (1 << 2) | (0 << 3) | (0 << 7);
    *joint->controlword = mask;
}

// Function to switch on drive.
void cia_drive_switch_on(cia_joint_data_t *joint){
    uint8_t mask;
    mask = (1 << 0) | (1 << 1) | (1 << 2) | (0 << 3)  | (0 << 7);
    *joint->controlword = mask;
}

// Function to enable operation for drive.
void cia_drive_enable_operation(cia_joint_data_t *joint){
    uint8_t mask;
    mask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (0 << 7);
    *joint->controlword = mask;
    *joint->opmode = *joint->var_opmode;
}

// Function to reset fault for driver.
void cia_drive_reset_fault(cia_joint_data_t *joint){
    uint8_t mask;
    mask = (1 << 7);
    *joint->controlword = mask;
}

// Function to perform home sequence for drive.
// Home program: 0 for default, 33 for z pulse index.
void cia_drive_do_home(cia_joint_data_t *joint, int home_program){
    // printf("used home program %d \n",home_program);
    uint8_t mask;
    mask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (0 << 7);
    *joint->controlword = mask;
    *joint->opmode = OPMODE_HOMING;
    *joint->homing_method = home_program;
}

// Function to set homed for drive nr i.
void cia_drive_set_homed(cia_joint_data_t *joint){
    uint8_t mask;
    mask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (0 << 4) | (0 << 7);
    *joint->controlword = mask;
    *joint->opmode = *joint->var_opmode;
}

// Function to home drive on index z pulse.
void cia_drive_home(cia_joint_data_t *joint){

    double home_delay   = 0.1;      // Delay in seconds.

    switch (joint->home_struct.hs) {
    case HOME_NONE:
        break;
    case HOME_INIT:
        printf("home init joint: %d \n",joint->joint_nr);
        cia_drive_do_home(joint,*joint->var_home_program);
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
        cia_drive_set_homed(joint);

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
