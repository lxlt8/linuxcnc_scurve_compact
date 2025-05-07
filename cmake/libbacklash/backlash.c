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
#include "backlash.h"
#include "scurve.h"
#include "stdio.h"

typedef struct {
    bool ok;
} skynet_t;
skynet_t *skynet;

typedef struct {
    hal_float_t *Pin;
} float_data_t;
float_data_t
*hal_x_vel_in,
*hal_x_vel_out,
*hal_x_pos_in,
*hal_x_pos_out,
*hal_x_acc_in,
*hal_x_acc_out;

//! Pins
typedef struct {
    hal_bit_t *Pin;
} bit_data_t;
bit_data_t *enable;

typedef struct {
    hal_s32_t *Pin;
} s32_data_t;

typedef struct {
    hal_u32_t *Pin;
} u32_data_t;

typedef struct {
    hal_port_t *Pin;
} port_data_t;
port_data_t *port;

//! Params
typedef struct {
    hal_float_t Pin;
} param_float_data_t;

typedef struct {
    hal_bit_t Pin;
} param_bit_data_t;

// To clean hal environment after runtest :
// ~/linuxcnc_scurve_compact/scripts$ ./halrun -U


/* module information */
MODULE_AUTHOR("Julian_Koning");
MODULE_DESCRIPTION("Halmodule");
MODULE_LICENSE("GPL");

static int comp_idx;            // Component ID

// Function defenitions.
static void backlash_update();
static int setup_pins();

// Scurve data struct.
struct scurve_data sc_x, sc_y;

int sc_res_x = 0;

double maxjerk      = 20;
double maxvel       = 50;
double maxacc       = 10;
double cycletime    = 0.001;

double endvel       = 0;
double endacc       = 0;
double tarpos       = 0;
int pausing         = 0;

double hartbeat     = 0;

double x_vel_old    = 0;
double x_backlash   = 5;

// Dlopen ..
int rtapi_app_main(void) {

    int r = 0;
    comp_idx = hal_init("backlash");
    if(comp_idx < 0) return comp_idx;
    r = hal_export_funct("backlash_update", backlash_update, &skynet,0,0,comp_idx);

    r+=setup_pins();

    if(r) {
        hal_exit(comp_idx);
    } else {
        hal_ready(comp_idx);
    }

    // Init scurve x, y with zero values.
    scurve_reset_data(&sc_x);
    scurve_reset_data(&sc_y);

    return 0;
}

void rtapi_app_exit(void){
    hal_exit(comp_idx);
}

static inline void print_scurve_result(struct scurve_data *data, int res){

    // Scurve return states.
    if(res==0){
        printf("scurve busy. \n");
    }
    if(res==1){
        printf("scurve finished. \n");
    }
    if(res==2){
        printf("scurve error. \n");
    }

    printf("scurve curvel: %f \n",data->curvel);
    printf("scurve curacc: %f \n",data->curacc);
    printf("scurve curpos: %f \n",data->curpos);
}

static inline int update_scurve_axis(struct scurve_data *data, double tarpos){

    // Set scurve values for jerk, acceleration, max velocity, cycletime.
    scurve_init(data,
                maxjerk,
                maxacc,
                maxvel,
                cycletime);

    // Update scurve cycle.
    scurve_set_target_state(data,
                            0,
                            0,
                            tarpos,
                            0);

    return scurve_update(data);
}

static inline int check_axis_dir_change(double vel_new, double vel_old){

    if( (vel_new>=0 && vel_old>=0) ||  (vel_new<0 && vel_old<0) ){
        // No direction change.
    } else { // Direciton change.
        return 1;
    }
    return 0;
}

// Update from servo-cycle.
static void backlash_update(){

    // For test, trigger dir change, to be removed:
    *hal_x_vel_in->Pin=-10;


    if(check_axis_dir_change( *hal_x_vel_in->Pin, x_vel_old )){  // Direction changed.
        x_backlash = x_backlash *-1;    // Reverse tarpos.
        x_vel_old = *hal_x_vel_in->Pin; // Update old value.
    }

    // Update the scurve x axis, given a tarpos.
    sc_res_x = update_scurve_axis(&sc_x, x_backlash);
    print_scurve_result(&sc_x, sc_res_x);

    // Result curve = scurve planner + scurve backlash
    *hal_x_pos_out->Pin = sc_x.curpos + *hal_x_pos_in->Pin;
    *hal_x_vel_out->Pin = sc_x.curvel + *hal_x_vel_in->Pin;
    *hal_x_acc_out->Pin = sc_x.curacc + *hal_x_acc_in->Pin;

    // Show active state.
    hartbeat+=0.001;
    printf("time sec: %f \n",hartbeat);

}

static int setup_pins(){
    int r=0;

    // IN pins.
    enable = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("backlash.enable",HAL_IN,&(enable->Pin),comp_idx);

    hal_x_vel_in = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("backlash.x_vel_in",HAL_IN,&(hal_x_vel_in->Pin),comp_idx);

    hal_x_pos_in = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("backlash.x_pos_in",HAL_IN,&(hal_x_pos_in->Pin),comp_idx);

    hal_x_acc_in = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("backlash.x_acc_in",HAL_IN,&(hal_x_acc_in->Pin),comp_idx);

    // OUT pins.
    hal_x_vel_out = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("backlash.x_vel_out",HAL_OUT,&(hal_x_vel_out->Pin),comp_idx);

    hal_x_pos_out = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("backlash.x_pos_out",HAL_OUT,&(hal_x_pos_out->Pin),comp_idx);

    hal_x_acc_out = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("backlash.x_acc_out",HAL_OUT,&(hal_x_acc_out->Pin),comp_idx);

    return r;
}






































