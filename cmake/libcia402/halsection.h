#ifndef HALSECTION_H
#define HALSECTION_H

#include "hal.h"

typedef struct {
    bool ok;
} skynet_t;
skynet_t *skynet;

enum enum_drive_state {
    NOT_READY_TO_SWITCH_ON,
    SWITCH_ON_DISABLED,
    READY_TO_SWITCH_ON,
    SWITCH_ON,
    OPERATION_ENABLED,
    QUICK_STOP_ACTIVE,
    FAULT_REACTION_ACTIVE,
    SERVO_FAULT
};

enum enum_drive_opmode_state {
    OPMODE_NONE = 0,
    OPMODE_HOMING = 6,
    OPMODE_CYCLIC_POSITION = 8,
    OPMODE_CYCLIC_VELOCITY = 9
};

enum enum_drive_home_state {
    HOME_NONE,
    HOME_INIT,
    HOME_DELAY,
    HOME_BUSY,
    HOME_FINISHED
};

enum enum_drive_run_mode {
    RUN_NONE,
    RUN_HOME,
    RUN_POSITION,
    RUN_VELOCITY
};

enum enum_drive_fault {
    FAULT_OK,
    FAULT_ACTIVE,
    FAULT_SHUTDOWN_DRIVE
};

struct home_struct {
    enum enum_drive_home_state hs;
    double delay;
    int home_busy_message;
    hal_float_t pos_cmd_snapshot;
    hal_float_t pos_cmd_offset;
    hal_float_t pos_fb_snapshot;
    hal_float_t pos_fb_offset;
};

typedef struct {

    enum enum_drive_state drive_state;
    enum enum_drive_fault enum_fault;
    struct home_struct home_struct;
    enum enum_drive_run_mode runmode;

    int joint_nr;

    // Control pins, bit.
    hal_bit_t *enable;
    hal_bit_t *home;

    // Control pins, float.
    hal_float_t *pos_cmd;
    hal_float_t *vel_cmd;
    hal_float_t *pos_fb;
    hal_float_t *pos_fb_raw;
    hal_float_t *vel_fb;
    hal_float_t *pos_offset;

    // Control pins, U32.
    hal_u32_t *var_opmode;
    hal_u32_t *var_max_torque;
    // Control pins, S32.
    hal_s32_t *var_pos_scale;
    hal_s32_t *var_vel_scale;
    hal_s32_t *var_home_program;

    // Status pins, bit.
    hal_bit_t *stat_opmode_cyclic_position;
    hal_bit_t *stat_opmode_cyclic_velocity;
    hal_bit_t *stat_opmode_no_mode;
    hal_bit_t *stat_opmode_homing;
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
    hal_bit_t *stat_virtual_home_switch;
    hal_bit_t *stat_homing;
    hal_bit_t *stat_homed;
    hal_bit_t *stat_drv_fault;
    // Status pins, float.
    hal_float_t *stat_runtime;

    // Lcec connected pins, u32.
    hal_u32_t *statusword;
    hal_u32_t *controlword;
    hal_u32_t *max_torque;
    hal_u32_t *home_speed_zero_search;
    hal_u32_t *home_speed_switch_search;
    hal_u32_t *home_acceleration;
    // Lcec connected pins, s32.
    hal_s32_t *homing_method;
    hal_s32_t *opmode;
    hal_s32_t *opmode_display;
    hal_s32_t *actual_position;
    hal_s32_t *actual_velocity;
    hal_s32_t *actual_torque;
    hal_s32_t *actual_following_error;
    hal_s32_t *target_position;
    hal_s32_t *target_velocity;

} joint_data_t;

#endif // HALSECTION_H
