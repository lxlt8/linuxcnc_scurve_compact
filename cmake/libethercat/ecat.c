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
#include "hal_data_types.h"
#include "time.h"

extern int init_ecat(int comp_id, int debug_xml);
extern int run_ecat();
extern int stop_ecat();

// module information
MODULE_AUTHOR("Micel Wijnja");
MODULE_DESCRIPTION("Halmodule");
MODULE_LICENSE("GPL");

// Component ID
static int comp_idx;

// Function defenitions.
static void ecat_update();
static int setup_pins();

struct timespec start, end;
bit_data_t *enable;
float_data_t *update_time;
skynet_t *skynet;

// Dlopen ..
int rtapi_app_main(void) {

    int r = 0;
    comp_idx = hal_init("ecat");
    if(comp_idx < 0) return comp_idx;
    r = hal_export_funct("ecat_update", ecat_update, &skynet,0,0,comp_idx);

    r+=setup_pins();

    int debug_xml=1;
    init_ecat(comp_idx, debug_xml);

    if(r) {
        hal_exit(comp_idx);
    } else {
        hal_ready(comp_idx);
    }

    return 0;
}

void rtapi_app_exit(void){
    stop_ecat();
    hal_exit(comp_idx);
}

// Update from servo-cycle.
// We need a hal pin converter tool, sinds ethercat master esi does not care about INT or UINT differences.

static void ecat_update(){

    clock_gettime(CLOCK_MONOTONIC, &start);

    run_ecat();

    clock_gettime(CLOCK_MONOTONIC, &end);

    long ns = (end.tv_sec - start.tv_sec) * 1000000000L;
    ns += end.tv_nsec - start.tv_nsec;
    *update_time->Pin = (float)ns;
}

static int setup_pins(){
    int r=0;

    // IN pins.
    enable = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("ecat.enable",HAL_IN,&(enable->Pin),comp_idx);

    update_time = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("ecat.update_time",HAL_OUT,&(update_time->Pin),comp_idx);

    return r;
}






































