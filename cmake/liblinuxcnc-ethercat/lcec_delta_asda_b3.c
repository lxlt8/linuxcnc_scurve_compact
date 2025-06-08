//
//    Copyright (C) 2015 Jakob Flierl  <jakob.flierl@gmail.com>
//    Copyright (C) 2011 Sascha Ittner <sascha.ittner@modusoft.de>
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

#include "lcec.h"
#include "lcec_delta_asda_b3.h"
#include "stdio.h"

void lcec_delta_asda_b3_read(struct lcec_slave *slave, long period);
void lcec_delta_asda_b3_write(struct lcec_slave *slave, long period);

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
};

typedef struct {

    // Control pins, bit.
    hal_bit_t *enable;
    hal_bit_t *home;
    hal_bit_t *index_enable;
    hal_bit_t *index_enable_clear;
    hal_bit_t *home_torque_enable;

    // Control pins, float.
    hal_float_t *pos_cmd;
    hal_float_t *vel_cmd;
    hal_float_t *pos_fb;
    hal_float_t *vel_fb;

    // Control pins, U32.
    hal_u32_t *var_opmode;
    hal_u32_t *var_max_torque;
    hal_u32_t *var_torque_release_delay_ms;

    // Control pins, S32.
    hal_s32_t *var_pos_scale;
    hal_s32_t *var_vel_scale;
    hal_s32_t *var_home_program;
    hal_s32_t *var_max_torque_home_stop;

    // Lcec connected pins, u32.
    hal_u32_t *statusword;
    hal_u32_t *controlword;
    hal_u32_t *max_torque;
    hal_u32_t *home_speed_zero_search;
    hal_u32_t *home_speed_switch_search;
    hal_u32_t *home_acceleration;
    hal_u32_t *servo_alarm;
    hal_u32_t *following_error_window;
    // Lcec connected pins, s32.
    hal_s32_t *opmode_display;
    hal_s32_t *actual_position;
    hal_s32_t *actual_velocity;
    hal_s32_t *actual_torque;
    hal_s32_t *actual_following_error;
    hal_s32_t *position_demand_value;

    hal_s32_t *opmode;
    hal_s32_t *target_position;
    hal_s32_t *target_velocity;
    hal_s32_t *homing_method;

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
    hal_bit_t *stat_homing;
    hal_bit_t *stat_homed;
    hal_bit_t *stat_drv_fault;
    hal_bit_t *stat_torque_home_stop;

    // Status pins, float.
    hal_float_t *stat_runtime;

    unsigned int status_word_pdo_os;
    unsigned int opmode_display_pdo_os;
    unsigned int actual_position_pdo_os;
    unsigned int actual_velocity_pdo_os;
    unsigned int actual_torque_pdo_os;
    unsigned int actual_following_error_pdo_os;
    unsigned int position_demand_value_pdo_os;
    unsigned int servo_alarm_pdo_os;
    unsigned int control_word_pdo_os;
    unsigned int opmode_pdo_os;
    unsigned int following_error_window_pdo_os;
    unsigned int target_position_pdo_os;
    unsigned int target_velocity_pdo_os;
    unsigned int max_torque_pdo_os;
    unsigned int homing_method_pdo_os;

    enum enum_drive_state drive_state;
    enum enum_drive_fault enum_fault;
    struct home_struct home_struct;
    enum enum_drive_run_mode runmode;

    // Var.
    float torque_timer;

    // Test param.
    hal_bit_t test_param;

    // Var bit.
    hal_bit_t torque_timer_enable;
    hal_bit_t index_enable_trigger;

} lcec_delta_asda_b3_data_t;

static void drive_home_on_index(lcec_delta_asda_b3_data_t *joint);

static lcec_pindesc_t slave_pins[] = {
    { HAL_BIT, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, enable), "%s.%s.%s.enable" },
    { HAL_BIT, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, home), "%s.%s.%s.home" },
    { HAL_BIT, HAL_IO, offsetof(lcec_delta_asda_b3_data_t, index_enable), "%s.%s.%s.index-enable" },
    { HAL_BIT, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, index_enable_clear), "%s.%s.%s.index-enable-clear" },
    { HAL_BIT, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, home_torque_enable), "%s.%s.%s.home-torque-enable" },
    { HAL_FLOAT, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, pos_cmd), "%s.%s.%s.pos-cmd" },
    { HAL_FLOAT, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, vel_cmd), "%s.%s.%s.vel-cmd" },
    { HAL_FLOAT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, pos_fb), "%s.%s.%s.pos-fb" },
    { HAL_FLOAT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, vel_fb), "%s.%s.%s.vel-fb" },
    { HAL_U32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, var_opmode), "%s.%s.%s.var-opmode" },
    { HAL_U32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, var_max_torque), "%s.%s.%s.var-max-torque" },
    { HAL_U32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, var_torque_release_delay_ms), "%s.%s.%s.var-torque-release-delay-ms" },
    { HAL_S32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, var_pos_scale), "%s.%s.%s.var-pos-scale" },
    { HAL_S32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, var_vel_scale), "%s.%s.%s.var-vel-scale" },
    { HAL_S32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, var_home_program), "%s.%s.%s.var-home-program" },
    { HAL_S32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, var_max_torque_home_stop), "%s.%s.%s.var-max-torque-home-stop" },
    { HAL_U32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, statusword), "%s.%s.%s.statusword" },
    { HAL_U32, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, controlword), "%s.%s.%s.controlword" },
    { HAL_U32, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, max_torque), "%s.%s.%s.max-torque" },
    { HAL_U32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, home_speed_zero_search), "%s.%s.%s.home-speed-zero-search" },
    { HAL_U32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, home_speed_switch_search), "%s.%s.%s.home-speed-sw-search" },
    { HAL_U32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, home_acceleration), "%s.%s.%s.home-acceleration" },
    { HAL_U32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, servo_alarm), "%s.%s.%s.servo-alarm" },
    { HAL_U32, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, following_error_window), "%s.%s.%s.following-error-window" },
    { HAL_S32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, opmode_display), "%s.%s.%s.opmode-display" },
    { HAL_S32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, actual_position), "%s.%s.%s.actual-position" },
    { HAL_S32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, actual_velocity), "%s.%s.%s.actual-velocity" },
    { HAL_S32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, actual_torque), "%s.%s.%s.actual-torque" },
    { HAL_S32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, actual_following_error), "%s.%s.%s.actual-following-error" },
    { HAL_S32, HAL_IN, offsetof(lcec_delta_asda_b3_data_t, position_demand_value), "%s.%s.%s.position-demand-value" },
    { HAL_S32, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, opmode), "%s.%s.%s.opmode" },
    { HAL_S32, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, target_position), "%s.%s.%s.target-position" },
    { HAL_S32, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, target_velocity), "%s.%s.%s.target-velocity" },
    { HAL_S32, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, homing_method), "%s.%s.%s.homing_method" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_opmode_cyclic_position), "%s.%s.%s.stat-opmode-cyclic-position" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_opmode_cyclic_velocity), "%s.%s.%s.stat-opmode-cyclic-velocity" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_opmode_no_mode), "%s.%s.%s.stat-opmode-no-mode" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_opmode_homing), "%s.%s.%s.stat-opmode-homing" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_switchon_ready), "%s.%s.%s.stat-switchon-ready" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_switched_on), "%s.%s.%s.stat-switched-on" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_op_enabled), "%s.%s.%s.stat-op-enabled" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_fault), "%s.%s.%s.stat-fault" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_voltage_enabled), "%s.%s.%s.stat_voltage_enabled" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_quick_stop), "%s.%s.%s.stat-quick-stop" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_switchon_disabled), "%s.%s.%s.stat-switchon-disabled" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_warning), "%s.%s.%s.stat-warning" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_remote), "%s.%s.%s.stat-remote" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_target_reached), "%s.%s.%s.stat-target-reached" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_internal_limit), "%s.%s.%s.stat-internal-limit" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_acknowledged), "%s.%s.%s.stat-acknowledged" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_following_error), "%s.%s.%s.stat_following_error" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_referenced), "%s.%s.%s.stat-referenced" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_positive_limit), "%s.%s.%s.stat-positive-limit" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_negative_limit), "%s.%s.%s.stat-negative-limit" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_bit_11), "%s.%s.%s.stat-bit-11" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_bit_12), "%s.%s.%s.stat-bit-12" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_bit_13), "%s.%s.%s.stat-bit-13" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_homing), "%s.%s.%s.stat-homing" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_homed), "%s.%s.%s.stat-homed" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_drv_fault), "%s.%s.%s.stat-drv-fault" },
    { HAL_BIT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_torque_home_stop), "%s.%s.%s.stat-torque-home-stop" },
    { HAL_FLOAT, HAL_OUT, offsetof(lcec_delta_asda_b3_data_t, stat_runtime), "%s.%s.%s.stat-runtime" },
    { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

static const lcec_pindesc_t slave_params[] = {
    { HAL_BIT, HAL_RW, offsetof(lcec_delta_asda_b3_data_t, test_param), "%s.%s.%s.test-param" },
    { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

static ec_pdo_entry_info_t lcec_in[] = {
    {0x6041, 0x00, 16}, // Status Word              (U32)
    {0x6061, 0x00, 8},  // Opmode-display           (S32)
    {0x6064, 0x00, 32}, // Actual-position          (S32)
    {0x606C, 0x00, 32}, // Actual-velocity          (S32)
    {0x6077, 0x00, 16}, // Actual-torque            (S32)
    {0x60F4, 0x00, 32}, // Actual-following-error   (S32)
    {0x60FC, 0x00, 32}, // Position-demand-value    (S32)
    {0x2001, 0x00, 16}  // Servo-alarm              (U32)
};

static ec_pdo_entry_info_t lcec_out[] = {
    {0x6040, 0x00,  16}, // Control Word            (U32)
    {0x6060, 0x00,  8},  // Opmode                  (S32)
    {0x6065, 0x00,  32}, // Following-error-window  (U32)
    {0x607A, 0x00,  32}, // Target-position         (S32)
    {0x60FF, 0x00,  32}, // Target-velocity         (S32)
    {0x6072, 0x00,  16}, // Max-torque              (U32)
    {0x6098, 0x00,  8}   // Homing-method           (S32)
};

static ec_pdo_info_t lcec_delta_asda_b3_pdos_out[] = {
    {0x1600, 7, lcec_out}  /* ENC RxPDO-Map Control compact */
};

static ec_pdo_info_t lcec_delta_asda_b3_pdos_in[] = {
    {0x1a00, 8, lcec_in}  /* ENC TxPDO-Map Status compact */
};

static ec_sync_info_t lcec_delta_asda_b3_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL},
    {1, EC_DIR_INPUT,  0, NULL},
    {2, EC_DIR_OUTPUT, 1, lcec_delta_asda_b3_pdos_out},
    {3, EC_DIR_INPUT,  1, lcec_delta_asda_b3_pdos_in},
    {0xff}
};

int lcec_delta_asda_b3_init(int comp_id, struct lcec_slave *slave, ec_pdo_entry_reg_t *r) {
    lcec_master_t *master = slave->master;
    lcec_delta_asda_b3_data_t *hal_data;
    int err;

    // Initialize callback functions.
    slave->proc_read  = lcec_delta_asda_b3_read;
    slave->proc_write = lcec_delta_asda_b3_write;

    // Allococate hal memory
    if ((hal_data = hal_malloc(sizeof(lcec_delta_asda_b3_data_t))) == NULL) {
        rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", master->name, slave->name);
        return -EIO;
    }
    memset(hal_data, 0, sizeof(lcec_delta_asda_b3_data_t));
    slave->hal_data = hal_data;

//    // Set an sdo.
//    if (ecrt_slave_config_sdo32(slave->config, 0x6093, 0x01, 16777216) != 0) {
//      rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo 6093h subindex 1. \n", master->name, slave->name);
//    }
//    if (ecrt_slave_config_sdo32(slave->config, 0x6093, 0x02, 500000) != 0) {
//      rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo 6093h subindex 2. \n", master->name, slave->name);
//    }

    // Initialize sync info
    slave->sync_info = lcec_delta_asda_b3_syncs;

    // Initialize PDO entries, this is like the .xml entry list.
    LCEC_PDO_INIT(r, slave->index, slave->vid, slave->pid, 0x6041, 0x00, &hal_data->status_word_pdo_os, NULL);
    LCEC_PDO_INIT(r, slave->index, slave->vid, slave->pid, 0x6061, 0x00, &hal_data->opmode_display_pdo_os, NULL);
    LCEC_PDO_INIT(r, slave->index, slave->vid, slave->pid, 0x6064, 0x00, &hal_data->actual_position_pdo_os, NULL);
    LCEC_PDO_INIT(r, slave->index, slave->vid, slave->pid, 0x606C, 0x00, &hal_data->actual_velocity_pdo_os, NULL);
    LCEC_PDO_INIT(r, slave->index, slave->vid, slave->pid, 0x6077, 0x00, &hal_data->actual_torque_pdo_os, NULL);
    LCEC_PDO_INIT(r, slave->index, slave->vid, slave->pid, 0x60F4, 0x00, &hal_data->actual_following_error_pdo_os, NULL);
    LCEC_PDO_INIT(r, slave->index, slave->vid, slave->pid, 0x60FC, 0x00, &hal_data->position_demand_value_pdo_os, NULL);
    LCEC_PDO_INIT(r, slave->index, slave->vid, slave->pid, 0x2001, 0x00, &hal_data->servo_alarm_pdo_os, NULL);

    LCEC_PDO_INIT(r, slave->index, slave->vid, slave->pid, 0x6040, 0x00, &hal_data->control_word_pdo_os, NULL);
    LCEC_PDO_INIT(r, slave->index, slave->vid, slave->pid, 0x6060, 0x00, &hal_data->opmode_pdo_os, NULL);
    LCEC_PDO_INIT(r, slave->index, slave->vid, slave->pid, 0x6065, 0x00, &hal_data->following_error_window_pdo_os, NULL);
    LCEC_PDO_INIT(r, slave->index, slave->vid, slave->pid, 0x607A, 0x00, &hal_data->target_position_pdo_os, NULL);
    LCEC_PDO_INIT(r, slave->index, slave->vid, slave->pid, 0x60FF, 0x00, &hal_data->target_velocity_pdo_os, NULL);
    LCEC_PDO_INIT(r, slave->index, slave->vid, slave->pid, 0x6072, 0x00, &hal_data->max_torque_pdo_os, NULL);
    LCEC_PDO_INIT(r, slave->index, slave->vid, slave->pid, 0x6098, 0x00, &hal_data->homing_method_pdo_os, NULL);

    // Export pins
    if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
        return err;
    }

    // Export parameters
    if ((err = lcec_param_newf_list(hal_data, slave_params, LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
        return err;
    }

    // Initialize variables
    hal_data->torque_timer = 0;
    hal_data->test_param = 1;

    return 0;
}

// Function to check input values and prevent dividing by zero.
static void drive_check_scales(lcec_delta_asda_b3_data_t *joint){
    if( (*joint->var_pos_scale)<1e-20 && (*joint->var_pos_scale)>1e-20 ){
        *joint->var_pos_scale = 1;
        printf("drive pos scale set to 1. \n");
    }
    if( (*joint->var_vel_scale)<1e-20 && (*joint->var_vel_scale)>1e-20 ){
        *joint->var_vel_scale = 1;
        printf("drive vel scale set to 1. \n");
    }
}

static void read_drive_status(lcec_delta_asda_b3_data_t *joint) {

    // Check input values for pos & vel scale.
    drive_check_scales(joint);
    // printf("pos scale: %f \n",(float)(*joint->var_pos_scale));

    // Update run timer.
    *joint->stat_runtime += 0.001;

    // Get pos & vel feedback.
    *joint->pos_fb = ((double)(*joint->actual_position) * (1/(double)(*joint->var_pos_scale)));
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

    if(fabs(*joint->actual_torque) >= fabs(*joint->var_max_torque_home_stop)){
        joint->torque_timer_enable = 1;
        joint->torque_timer = 0;
        *joint->stat_torque_home_stop = 1;
    }

    if(joint->torque_timer_enable){
        joint->torque_timer += 1;
        // printf("timer: %f \n",joint->torque_timer);
    }

    if(joint->torque_timer > *joint->var_torque_release_delay_ms){
        // printf("timer completed. \n");
        *joint->stat_torque_home_stop = 0;
        joint->torque_timer_enable = 0;
    }

    if(*joint->index_enable_clear){
        *joint->index_enable = 0;
        // printf("index enable cleared. \n");
    }
}

// Write position command
static void drive_write_position_(lcec_delta_asda_b3_data_t *joint){
    *joint->target_position = (hal_s32_t)( (*joint->pos_cmd) * (*joint->var_pos_scale) );
}

// Write velocity command
static void drive_write_velocity(lcec_delta_asda_b3_data_t *joint){
    *joint->target_velocity = (hal_s32_t)( (*joint->vel_cmd) * (*joint->var_vel_scale) );
}

// Write torque command
static void drive_write_torque(lcec_delta_asda_b3_data_t *joint){

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
static void drive_shutdown(lcec_delta_asda_b3_data_t *joint){
    uint8_t mask;
    mask = (0 << 0) | (1 << 1) | (1 << 2) | (0 << 3) | (0 << 7);
    *joint->controlword = mask;
}

// Function to switch on drive.
static void drive_switch_on(lcec_delta_asda_b3_data_t *joint){
    uint8_t mask;
    mask = (1 << 0) | (1 << 1) | (1 << 2) | (0 << 3)  | (0 << 7);
    *joint->controlword = mask;
}

// Function to enable operation for drive.
static void drive_enable_operation(lcec_delta_asda_b3_data_t *joint){
    uint8_t mask;
    mask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (0 << 7);
    *joint->controlword = mask;
    *joint->opmode = *joint->var_opmode;
}

// Function to reset fault for driver.
static void drive_reset_fault(lcec_delta_asda_b3_data_t *joint){
    uint8_t mask;
    mask = (1 << 7);
    *joint->controlword = mask;
}

// Function to perform home sequence for drive.
// Home program: 0 for default, 33 for z pulse index.
static void drive_do_home(lcec_delta_asda_b3_data_t *joint, int home_program){
    uint8_t mask;
    mask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (0 << 7);
    *joint->controlword = mask;
    *joint->opmode = OPMODE_HOMING;
    *joint->homing_method = home_program;
}

// Function to set homed for drive nr i.
static void drive_set_homed(lcec_delta_asda_b3_data_t *joint){
    uint8_t mask;
    mask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (0 << 4) | (0 << 7);
    *joint->controlword = mask;
    *joint->opmode = *joint->var_opmode;
}

// Function to home drive on index z pulse.
static void drive_home(lcec_delta_asda_b3_data_t *joint){

    double home_delay   = 0.1;      // Delay in seconds.

    switch (joint->home_struct.hs) {
    case HOME_NONE:
        break;
    case HOME_INIT:
        drive_do_home(joint,*joint->var_home_program);
        joint->home_struct.hs = HOME_DELAY;
        joint->home_struct.delay = 0;
        joint->home_struct.home_busy_message = 0;
        *joint->stat_homing=1;
        *joint->stat_homed=0;
        break;
    case HOME_DELAY:
        if(!joint->home_struct.home_busy_message){
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

// Function when drive is in state : Operation enabled.
void drive_run(lcec_delta_asda_b3_data_t *joint){

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
void drive_state_machine_(lcec_delta_asda_b3_data_t *joint){

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

void lcec_delta_asda_b3_read(struct lcec_slave *slave, long period) {
    lcec_master_t *master = slave->master;
    lcec_delta_asda_b3_data_t *hal_data = (lcec_delta_asda_b3_data_t *) slave->hal_data;
    uint8_t *pd = master->process_data;

    // Read cia402 values from ethercat bus.
    *(hal_data->statusword) = EC_READ_U16(&pd[hal_data->status_word_pdo_os]);
    *(hal_data->opmode_display) = EC_READ_S8(&pd[hal_data->opmode_display_pdo_os]);
    *(hal_data->actual_position) = EC_READ_S32(&pd[hal_data->actual_position_pdo_os]);
    *(hal_data->actual_velocity) = EC_READ_S32(&pd[hal_data->actual_velocity_pdo_os]);
    *(hal_data->actual_torque) = EC_READ_S16(&pd[hal_data->actual_torque_pdo_os]);
    *(hal_data->actual_following_error) = EC_READ_S32(&pd[hal_data->actual_following_error_pdo_os]);
    *(hal_data->position_demand_value) = EC_READ_S32(&pd[hal_data->position_demand_value_pdo_os]);
    *(hal_data->servo_alarm) = EC_READ_U16(&pd[hal_data->servo_alarm_pdo_os]);

    read_drive_status(hal_data);
}

void lcec_delta_asda_b3_write(struct lcec_slave *slave, long period) {
    lcec_master_t *master = slave->master;
    lcec_delta_asda_b3_data_t *hal_data = (lcec_delta_asda_b3_data_t *) slave->hal_data;
    uint8_t *pd = master->process_data;

    if(*hal_data->enable){
        drive_state_machine_(hal_data);
        drive_write_position_(hal_data);
        drive_write_velocity(hal_data);
        drive_write_torque(hal_data);

        // Check for home-index pin.
        if(*hal_data->index_enable && !hal_data->index_enable_trigger){
            printf("home on index request. \n");
            hal_data->runmode = RUN_HOME;
            hal_data->home_struct.hs = HOME_INIT;
            hal_data->index_enable_trigger=1;
        }

    } else {
        drive_shutdown(hal_data);
        hal_data->index_enable_trigger = 0;
    }

    EC_WRITE_U16(&pd[hal_data->control_word_pdo_os], *hal_data->controlword);
    EC_WRITE_S8(&pd[hal_data->opmode_pdo_os], *hal_data->opmode);
    EC_WRITE_U32(&pd[hal_data->following_error_window_pdo_os], *hal_data->following_error_window);
    EC_WRITE_S32(&pd[hal_data->target_position_pdo_os], *hal_data->target_position );
    EC_WRITE_S32(&pd[hal_data->target_velocity_pdo_os], *hal_data->target_velocity);
    EC_WRITE_U16(&pd[hal_data->max_torque_pdo_os], *hal_data->max_torque);
    EC_WRITE_S8(&pd[hal_data->homing_method_pdo_os], *hal_data->homing_method);
}

