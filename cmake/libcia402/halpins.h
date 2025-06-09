#ifndef HALPINS_H
#define HALPINS_H

#include "hal.h"
#include "halsection.h"
#include "printf.h"

/**
 * Allocates memory for joint data array
 * @param count Number of joints to allocate memory for
 * @return Pointer to allocated joint_data_t array, or NULL on failure
 */
cia_joint_data_t* cia_allocate_joint_data(int count) {
    if (count <= 0) {
        printf("Error: Invalid joint count %d\n", count);
        return NULL;
    }

    cia_joint_data_t *jd = hal_malloc(sizeof(cia_joint_data_t) * count);
    if (!jd) {
        printf("Error: Failed to allocate memory for %d joints\n", count);
        return NULL;
    }

    // Initialize all memory to zero
    memset(jd, 0, sizeof(cia_joint_data_t) * count);

    printf("Allocated memory for %d joints\n", count);
    return jd;
}

int cia_setup_pins(cia_joint_data_t *jd, int count, int comp_id){
    int r=0; // Return 0 on succes, return -1 on failure.
    char pin_name[64]; // Store name.
    // jd = hal_malloc(sizeof(joint_data_t) * count); // Allocate memory.

    if(!jd){
        printf("allocating memory failed. \n");
        return -1;
    }
    printf("allocating memory for joints: %d \n",count);

    printf("creating hal pins. \n");

    // Status pins.
    for(int i=0; i<count; i++){

        // Set the joint nr's.
        jd[i].joint_nr=count;

        printf("\t creating status pins for joint: %d \n",jd[i].joint_nr);

        // Bit pins.
        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-opmode-cyclic-position", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_opmode_cyclic_position),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-opmode-cyclic-velocity", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_opmode_cyclic_velocity),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-opmode-no-mode", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_opmode_no_mode),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-opmode-homing", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_opmode_homing),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-switchon-ready", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_switchon_ready),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-switched-on", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_switched_on),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-op-enabled", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_op_enabled),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-fault", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_fault),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat_voltage_enabled", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_voltage_enabled),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-quick-stop", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_quick_stop),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-switchon-disabled", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_switchon_disabled),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-warning", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_warning),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-remote", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_remote),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-target-reached", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_target_reached),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-internal-limit", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_internal_limit),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-acknowledged", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_acknowledged),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat_following_error", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_following_error),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-referenced", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_referenced),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-positive-limit", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_positive_limit),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-negative-limit", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_negative_limit),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-bit-11", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_bit_11),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-bit-12", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_bit_12),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-bit-13", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_bit_13),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-homing", i);
        r+= hal_pin_bit_new(pin_name, HAL_OUT, &(jd[i].stat_homing), comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-homed", i);
        r+= hal_pin_bit_new(pin_name, HAL_OUT, &(jd[i].stat_homed), comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-drv-fault", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_drv_fault),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-torque-home-stop", i);
        r+=hal_pin_bit_new(pin_name,HAL_OUT,&(jd[i].stat_torque_home_stop),comp_id);

        // Float pins.
        snprintf(pin_name, sizeof(pin_name), "cia402.%d.stat-runtime", i);
        r+=hal_pin_float_new(pin_name,HAL_OUT,&(jd[i].stat_runtime),comp_id);

        if(r==0){
            printf("\t status pins succes. \n");
        } else {
            printf("\t status pins failure. \n");
            return r;
        }
    }

    // Lcec connected pins.
    for(int i=0; i<count; i++){

        printf("\t creating lcec connected pins joint: %d \n",jd[i].joint_nr);

        // U32 pins.
        snprintf(pin_name, sizeof(pin_name), "cia402.%d.statusword", i);
        r+=hal_pin_u32_new(pin_name,HAL_IN,&(jd[i].statusword),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.controlword", i);
        r+=hal_pin_u32_new(pin_name,HAL_OUT,&(jd[i].controlword),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.max-torque", i);
        r+=hal_pin_u32_new(pin_name,HAL_OUT,&(jd[i].max_torque),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.home-speed-zero-search", i);
        r+=hal_pin_u32_new(pin_name,HAL_IN,&(jd[i].home_speed_zero_search),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.home-speed-sw-search", i);
        r+=hal_pin_u32_new(pin_name,HAL_IN,&(jd[i].home_speed_switch_search),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.home-acceleration", i);
        r+=hal_pin_u32_new(pin_name,HAL_IN,&(jd[i].home_acceleration),comp_id);

        // S32 pins.
        snprintf(pin_name, sizeof(pin_name), "cia402.%d.opmode-display", i);
        r+=hal_pin_s32_new(pin_name,HAL_IN,&(jd[i].opmode_display),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.actual-position", i);
        r+=hal_pin_s32_new(pin_name,HAL_IN,&(jd[i].actual_position),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.actual-velocity", i);
        r+=hal_pin_s32_new(pin_name,HAL_IN,&(jd[i].actual_velocity),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.actual-torque", i);
        r+=hal_pin_s32_new(pin_name,HAL_IN,&(jd[i].actual_torque),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.actual-following-error", i);
        r+=hal_pin_s32_new(pin_name,HAL_IN,&(jd[i].actual_following_error),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.opmode", i);
        r+=hal_pin_s32_new(pin_name,HAL_OUT,&(jd[i].opmode),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.target-position", i);
        r+=hal_pin_s32_new(pin_name,HAL_OUT,&(jd[i].target_position),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.target-velocity", i);
        r+=hal_pin_s32_new(pin_name,HAL_OUT,&(jd[i].target_velocity),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.homing-method", i);
        r+=hal_pin_s32_new(pin_name,HAL_OUT,&(jd[i].homing_method),comp_id);

        if(r==0){
            printf("\t lcec connected pins succes. \n");
        } else {
            printf("\t lcec connected pins failure. \n");
            return r;
        }
    }

    // Control pins.
    for(int i=0; i<count; i++){

        printf("\t creating control pins joint: %d \n",jd[i].joint_nr);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.enable", i);
        r+= hal_pin_bit_new(pin_name, HAL_IN, &(jd[i].enable), comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.home", i);
        r+= hal_pin_bit_new(pin_name, HAL_IN, &(jd[i].home), comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.home-torque-enable", i);
        r+= hal_pin_bit_new(pin_name, HAL_IN, &(jd[i].home_torque_enable), comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.index-enable", i);
        r+= hal_pin_bit_new(pin_name, HAL_IO, &(jd[i].index_enable), comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.index-enable-clear", i);
        r+= hal_pin_bit_new(pin_name, HAL_IN, &(jd[i].index_enable_clear), comp_id);

        // Float pins.
        snprintf(pin_name, sizeof(pin_name), "cia402.%d.pos-cmd", i);
        r+=hal_pin_float_new(pin_name,HAL_IN,&(jd[i].pos_cmd),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.pos-fb", i);
        r+=hal_pin_float_new(pin_name,HAL_OUT,&(jd[i].pos_fb),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.vel-cmd", i);
        r+=hal_pin_float_new(pin_name,HAL_IN,&(jd[i].vel_cmd),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.vel-fb", i);
        r+=hal_pin_float_new(pin_name,HAL_OUT,&(jd[i].vel_fb),comp_id);

        // U32 pins.
        snprintf(pin_name, sizeof(pin_name), "cia402.%d.var-opmode", i);
        r+=hal_pin_u32_new(pin_name,HAL_IN,&(jd[i].var_opmode),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.var-max-torque", i);
        r+=hal_pin_u32_new(pin_name,HAL_IN,&(jd[i].var_max_torque),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.var-torque-release-delay-ms", i);
        r+=hal_pin_u32_new(pin_name,HAL_IN,&(jd[i].var_torque_release_delay_ms),comp_id);

        // S32 pins.
        snprintf(pin_name, sizeof(pin_name), "cia402.%d.var-home-program", i);
        r+=hal_pin_s32_new(pin_name,HAL_IN,&(jd[i].var_home_program),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.var-pos-scale", i);
        r+=hal_pin_s32_new(pin_name,HAL_IN,&(jd[i].var_pos_scale),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.var-vel-scale", i);
        r+=hal_pin_s32_new(pin_name,HAL_IN,&(jd[i].var_vel_scale),comp_id);

        snprintf(pin_name, sizeof(pin_name), "cia402.%d.var-max-torque-home-stop", i);
        r+=hal_pin_s32_new(pin_name,HAL_IN,&(jd[i].var_max_torque_home_stop),comp_id);

        if(r==0){
            printf("\t control pins succes. \n");
        } else {
            printf("\t control pins failure. \n");
            return r;
        }
    }

    printf("hal pin setup succes. \n");
    return r;
}

#endif // HALPINS_H
