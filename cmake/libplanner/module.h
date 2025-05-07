/********************************************************************
*   Written by Grotius, alias Skynet.
*   michelwijnja@gmail.com
*
* Author: Michel Wijnja
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2024 All rights reserved.
********************************************************************/
#ifndef MODULE_H
#define MODULE_H

// This is the tpmod.c file integrated into the planner.
// We do this to make usage of hal pin's. This avoids editing
// the original lcnc source code, where the old planner relies on.
#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"
#include "tp_debug.h"
#include "common.h"
#include "halsection.h"

struct path_data path;

MODULE_LICENSE("GPL");

// provision for testing use of module parameters:
static char *tp_parms;
RTAPI_MP_STRING(tp_parms,"Example tp parms");

static int tpmod_id;

int rtapi_app_main(void)
{
    char* emsg;
    tpmod_id = hal_init("tpmod_scurve"); // dlopen(".../tpmod_scurve.so")
    if (tpmod_id < 0) {emsg="hal_init()"; goto error;}

    setup_hal_pins(tpmod_id,&path);

    hal_ready(tpmod_id);
    return 0;

error:
    rtapi_print_msg(RTAPI_MSG_ERR,"\ntpmod FAIL:<%s>\n",emsg);
    hal_exit(tpmod_id);
    return -1;
}

void rtapi_app_exit(void)
{
    hal_exit(tpmod_id);
    return;
}
// End of tpmod.c file.

//==========================================================
// tp module interface
// motmod function ptrs for functions called by tp:
static void(  *_DioWrite)(int,char);
static void(  *_AioWrite)(int,double);
static void(  *_SetRotaryUnlock)(int,int);
static int (  *_GetRotaryIsUnlocked)(int);
static double(*_axis_get_vel_limit)(int);
static double(*_axis_get_acc_limit)(int);

void tpMotFunctions( void(  *pDioWrite)(int,char)
                     ,void(  *pAioWrite)(int,double)
                     ,void(  *pSetRotaryUnlock)(int,int)
                     ,int (  *pGetRotaryIsUnlocked)(int)
                     ,double(*paxis_get_vel_limit)(int)
                     ,double(*paxis_get_acc_limit)(int) ){

    _DioWrite            = *pDioWrite;
    _AioWrite            = *pAioWrite;
    _SetRotaryUnlock     = *pSetRotaryUnlock;
    _GetRotaryIsUnlocked = *pGetRotaryIsUnlocked;
    _axis_get_vel_limit  = *paxis_get_vel_limit;
    _axis_get_acc_limit  = *paxis_get_acc_limit;
}

// Without this function, symbol tpMotData not found.
void tpMotData(emcmot_status_t *pstatus,
               emcmot_config_t *pconfig ){

    emcmotStatus = pstatus;
    emcmotConfig = pconfig;
}

// Extra data links to command & hal. We use this to
// directly read in the adaptive feed value from hal pin.
// We want to avoid using this soup :
// https://github.com/LinuxCNC/linuxcnc/blob/8afe7129a5f49a3972b982cdc74d1b18e87eef37/src/emc/motion/control.c#L381
void tpMotExtraData(emcmot_command_t *pcommand,
                    emcmot_hal_data_t *phal ){

    emcmotCommand = pcommand;
    emcmot_hal_data = phal;
}


#endif // MODULE_H
