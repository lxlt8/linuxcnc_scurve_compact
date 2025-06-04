#ifndef DRIVE_STATUS_H
#define DRIVE_STATUS_H

#include "hal.h"
#include "halsection.h"

static void read_drive_status(joint_data_t *joint) {

    // Check input values for pos & vel scale.
    drive_check_scales(joint);

    // Update run timer.
    *joint->stat_runtime += 0.001;

    // Get pos & vel feedback.
    *joint->pos_fb_raw = ((double)(*joint->actual_position) * (1/(double)(*joint->var_pos_scale)));
    *joint->pos_fb = *joint->pos_cmd; // *joint->pos_fb_raw + joint->home_struct.pos_fb_offset;
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

#endif // DRIVE_STATUS_H
