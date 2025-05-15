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

typedef struct {
    bool ok;
} skynet_t;
skynet_t *skynet;

typedef struct {
    hal_float_t *Pin;
} float_data_t;

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
MODULE_AUTHOR("Michel Wijnja");
MODULE_DESCRIPTION("Halmodule template");
MODULE_LICENSE("GPL");

static int comp_idx;            // Component ID

// Function defenitions.
static void template_update();
static int setup_pins();

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
    comp_idx = hal_init("template");
    if(comp_idx < 0) return comp_idx;
    r = hal_export_funct("template_update", template_update, &skynet,0,0,comp_idx);

    r+=setup_pins();

    if(r) {
        hal_exit(comp_idx);
    } else {
        hal_ready(comp_idx);
    }

    return 0;
}

void rtapi_app_exit(void){
    hal_exit(comp_idx);
}

// Update from servo-cycle.
static void template_update(){

    // Show active state.
    hartbeat+=0.001;
    printf("time sec: %f \n",hartbeat);

}

static int setup_pins(){
    int r=0;

    // IN pins.
    enable = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("template.enable",HAL_IN,&(enable->Pin),comp_idx);

    return r;
}






































