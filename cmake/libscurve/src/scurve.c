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
#include "scurve.h"
#include "stdio.h"
#include "math.h"
#include "time.h"
#include <stdint.h>

inline void scurve_reset_period(struct scurve_period *data){
    data->accbeg = 0;
    data->accend = 0;
    data->velini = 0;
    data->velbeg = 0;
    data->velend = 0;
    data->disbeg = 0;
    data->disend = 0;
    data->timbeg = 0;
    data->timend = 0;
    data->jermax = 0;
    data->accinf = 0;
}

inline void scurve_reset_periods(struct scurve_data *data) {
    scurve_reset_period(&data->c0);
    scurve_reset_period(&data->c1);
    scurve_reset_period(&data->c2);
    scurve_reset_period(&data->c3);
    scurve_reset_period(&data->c4);
}

inline void scurve_reset_overshoot(struct scurve_data *data) {
    data->pd.stopdist = 0;
    data->pd.stoptime = 0;
    data->pd.overshoot = 0;
    data->pd.cycles = 0;
}

inline void scurve_reset_data(struct scurve_data *data) {

    data->interval_time_ms = 0;
    data->maxjerk = 0;
    data->maxvel = 0;
    data->maxacc = 0;

    data->curpos = 0;
    data->curvel = 0;
    data->curacc = 0;

    data->taracc = 0;
    data->tarvel = 0;
    data->tarpos = 0;

    data->tarvel_max = 0;

    data->saved_acc = 0;
    data->saved_vel = 0;
    data->saved_pos = 0;

    data->direction = FORWARD_DIRECTION;
    data->int_direction = FORWARD_DIRECTION;

    data->incpos = 0;
    data->oldpos = 0;

    data->vr = 0;
    data->sr = 0;
    data->ar = 0;
    data->curtime = 0;

    scurve_reset_periods(data);
    scurve_reset_overshoot(data);
}

inline double scurve_displacement_period(struct scurve_period *p) {
    return p->disend - p->disbeg;
}

inline double scurve_delta_velocity(struct scurve_data *data) {
    double jm = data->maxjerk;         // "jm"  Jerk max. Steepness value of a scurve.
    double as = data->maxacc * 2;      // "as"  Max acceleration at inflection point. as=2*A.
    double dvt = 2 * as / jm;          // "dvt" Delta velocity time, derived from: jm=2*as/t1.
    double dv = fabs((dvt * as) / 2);  // "dv"  Delta velocity, dv=vo-ve, derived from: t1=2*(ve-vo)/as;
    return dv;
}

inline double scurve_time_all_periods(struct scurve_data *data) {
    return scurve_time_period(&data->c0) + scurve_time_period(&data->c1) +
            scurve_time_period(&data->c2) + scurve_time_period(&data->c3) +
            scurve_time_period(&data->c4);
}

inline double scurve_displacement_all_periods(struct scurve_data *data) {
    return scurve_displacement_period(&data->c0) + scurve_displacement_period(&data->c1) +
            scurve_displacement_period(&data->c2) + scurve_displacement_period(&data->c3) +
            scurve_displacement_period(&data->c4);
}

inline double scurve_time_period(struct scurve_period *p){
    return p->timend-p->timbeg;
}

// stop_lenght function
void scurve_stop_lenght_time(struct scurve_data *data, double *length, double *time) {
    struct scurve_data copy = *data; // Copy the data structure

    // Call the solver to build the stop curve
    scurve_solver_build_stop_curve(&copy);

    // Compute total displacement and time from all periods
    *length =
            scurve_displacement_period(&copy.c0) +
            scurve_displacement_period(&copy.c1) +
            scurve_displacement_period(&copy.c2) +
            scurve_displacement_period(&copy.c3);

    *time =
            scurve_time_period(&copy.c0) +
            scurve_time_period(&copy.c1) +
            scurve_time_period(&copy.c2) +
            scurve_time_period(&copy.c3);

    if(isnan(*length)){
        // printf("scurve warning: reset isnan for length. \n");
        *length=0;
    }
    if(isnan(*time)){
        // printf("scurve warning: reset isnan for time. \n");
        *time=0;
    }
}

inline double scurve_stop_length(struct scurve_data *data){
    struct scurve_data copy = *data; // Copy the data structure

    // Call the solver to build the stop curve
    scurve_solver_build_stop_curve(&copy);

    // Compute total displacement and time from all periods
    double length =
            scurve_displacement_period(&copy.c0) +
            scurve_displacement_period(&copy.c1) +
            scurve_displacement_period(&copy.c2) +
            scurve_displacement_period(&copy.c3);

    return length;
}

inline int scurve_duplicate_cycle(const struct scurve_data *data){

    if(
            data->curpos==data->saved_pos &&
            data->curacc==data->saved_acc &&
            data->curvel==data->saved_vel){

        // printf("duplicate cycle at pos: %f \n",data->saved_pos);
        // printf("duplicate cycle at vel: %f \n",data->saved_vel);
        // printf("duplicate cycle at acc: %f \n",data->saved_acc);

        return 1;
    }
    return 0;
}

inline int scurve_init(struct scurve_data *data,
                       double max_jerk,
                       double max_acceleration,
                       double max_velocity,
                       double cycletime) {

    data->maxjerk = max_jerk;
    data->maxacc = max_acceleration;
    data->maxvel = max_velocity;
    data->interval_time_ms = cycletime;

    if (max_jerk == 0.0) {
        // printf("error, maxjerk is zero.\n");
        return RETURN_ERROR;
    }
    if (max_acceleration == 0.0) {
        // printf("error, max acceleration is zero.\n");
        return RETURN_ERROR;
    }
    if (max_velocity == 0.0) {
        // printf("error, max velocity is zero.\n");
        return RETURN_ERROR;
    }
    if (cycletime == 0.0) {
        // printf("error, cycletime set to 0.001.\n");
        return RETURN_ERROR;
    }

    return RETURN_OK;
}

inline void scurve_set_target_state(struct scurve_data *data,
                                    const double endvel,
                                    const double endacc,
                                    const double endpos,
                                    const int pausing) {

    data->tarvel = endvel;
    data->taracc = endacc;
    data->tarpos = endpos;
    data->pausing = pausing;

    data->tarvel_max = endvel;

    // Normal scenario.
    if (data->curpos <= data->tarpos) {
        data->direction = FORWARD_DIRECTION;
        // printf("forward direction. \n");
    }
    if (data->curpos > data->tarpos) {
        data->direction = REVERSE_DIRECTION;
        // printf("reverse direction. \n");
    }
}

inline void scurve_stop_analyzer(struct scurve_data *data){

    data->pd.need_stop=0;

    if(data->direction==FORWARD_DIRECTION){
        scurve_stop_lenght_time(data,&data->pd.stopdist,&data->pd.stoptime);
        data->pd.overshoot=(data->curpos+data->pd.stopdist) - data->tarpos;

        if(data->curvel>=0 && data->pd.overshoot>0){

            data->pd.need_stop=1;
            scurve_solver_build_stop_curve(data);
            scurve_solver_update(data);
            return;
        }
    }

    if(data->direction==REVERSE_DIRECTION){
        scurve_stop_lenght_time(data,&data->pd.stopdist,&data->pd.stoptime);
        data->pd.stopdist=-fabs(data->pd.stopdist); // Turn to negative value for reverse motion.
        data->pd.overshoot=(data->curpos+data->pd.stopdist) - data->tarpos;

        if(data->curvel<=0 && data->pd.overshoot<0){

            data->pd.need_stop=1;
            scurve_solver_build_stop_curve(data);
            scurve_solver_update(data);
            return;
        }
    }

    scurve_solver_build_curve(data);
    scurve_solver_update(data);
}

inline int scurve_update(struct scurve_data *data) {

    data->saved_pos=data->curpos;
    data->saved_acc=data->curacc;
    data->saved_vel=data->curvel;

    // Use a little endvel, avoid a zero.
    double tollerance = 0.0;
    data->tarvel=fmax(data->tarvel,tollerance);

    int pause=0;
    // Need direction change. Stop motion first.
    if(data->curvel>0 && data->direction==REVERSE_DIRECTION){
        data->int_direction=FORWARD_DIRECTION;
        pause=1; // Easy solution.
    }
    if(data->curvel<0 && data->direction==FORWARD_DIRECTION){
        data->int_direction=REVERSE_DIRECTION;
        pause=1;
    }
    // Synchronize internal motion direction. Motion stopped. Direction change ok now.
    if(data->curvel==0 && data->direction==FORWARD_DIRECTION){
        data->int_direction=FORWARD_DIRECTION;
    }
    if(data->curvel==0 && data->direction==REVERSE_DIRECTION){
        data->int_direction=REVERSE_DIRECTION;
    }

    // Pause request, stop.
    if(data->pausing || pause){
        data->tarvel=0;
        scurve_solver_build_stop_curve(data);
        scurve_solver_update(data);
        // printf("pause. \n");

        // Check finished motion in pause.
        if(data->direction==FORWARD_DIRECTION && data->curpos>data->tarpos){
            return RETURN_FINISHED;
        }
        if(data->direction==REVERSE_DIRECTION && data->curpos<data->tarpos){
            return RETURN_FINISHED;
        }
        return RETURN_OK;
    }

    // Lower speed to user velocity max, stop to tarvel.
    if(fabs(data->curvel)>fabs(data->maxvel)){
        data->tarvel=fmin(data->tarvel_max, data->maxvel);
        scurve_solver_build_stop_curve(data);
        scurve_solver_update(data);
        // printf("lowering speed to velmax. curvel: %f maxvel: %f \n", data->curvel, data->maxvel);

        // Check finished motion while lowering maxvel.
        if(data->direction==FORWARD_DIRECTION && data->curpos>data->tarpos){
            return RETURN_FINISHED;

        }
        if(data->direction==REVERSE_DIRECTION && data->curpos<data->tarpos){
            return RETURN_FINISHED;
        }

        return RETURN_OK;
    }

    // Look 2 cycles ahead to calculate when to stop motion.
    // Works for endvel > 0.
    data->pd.need_stop=0;
    struct scurve_data copy=*data;
    scurve_stop_analyzer(&copy);

    if(data->tarvel==tollerance){ // Solves velocity spikes for joint.0 & joint.1
        scurve_stop_analyzer(&copy);
    }
    if(copy.pd.need_stop==1){
        data->pd.need_stop=1;
    }

    // Stop using endvel > 0.
    if(data->pd.need_stop==1 && data->tarvel>tollerance){
        scurve_solver_build_stop_curve(data);
        scurve_solver_update(data);

        if(data->curpos>data->tarpos && data->int_direction==FORWARD_DIRECTION){
            return RETURN_FINISHED;
        }
        if(data->curpos<data->tarpos && data->int_direction==REVERSE_DIRECTION){
            return RETURN_FINISHED;
        }
        return RETURN_OK;
    }

    // Stop using endvel = 0.
    // Using exact stop.
    if(data->pd.need_stop==1 && data->tarvel==tollerance){

        scurve_stop_lenght_time(data,&data->pd.stopdist,&data->pd.stoptime);
        data->pd.cycles=data->pd.stoptime/data->interval_time_ms;
        if(data->pd.cycles==0){ // In position.
            data->curpos=data->tarpos; // Set to absolute position values.
        }

        scurve_solver_build_stop_curve(data);
        scurve_solver_update(data);

        if(scurve_duplicate_cycle(data)){
            printf("scurve warning: duplicate cycle. \n");
            return RETURN_FINISHED;
        }
        if(data->curpos>data->tarpos && data->int_direction==FORWARD_DIRECTION){
            return RETURN_FINISHED;
        }
        if(data->curpos<data->tarpos && data->int_direction==REVERSE_DIRECTION){
            return RETURN_FINISHED;
        }
        // printf("stop to reach tarpos. endvel: %f \n",data->tarvel);
        return RETURN_OK;
    }

    // Normal forward curve.
    if(data->curpos<data->tarpos && data->direction==FORWARD_DIRECTION){
        data->int_direction=FORWARD_DIRECTION;
        scurve_solver_build_curve(data);
        scurve_solver_update(data);
        // printf("fwd curve. \n");

        if(scurve_duplicate_cycle(data)){
            printf("scurve warning: duplicate cycle. \n");
            return RETURN_FINISHED;
        }
        if(data->curpos>data->tarpos){
            return RETURN_FINISHED;
        }
        return RETURN_OK;
    }

    // Normal reverse curve.
    if(data->curpos>data->tarpos && data->direction==REVERSE_DIRECTION){
        data->int_direction=REVERSE_DIRECTION;
        scurve_solver_build_curve(data);
        scurve_solver_update(data);
        // printf("rev curve. \n");

        if(scurve_duplicate_cycle(data)){
            printf("scurve warning: duplicate cycle. \n");
            return RETURN_FINISHED;
        }
        if(data->curpos<data->tarpos){
            return RETURN_FINISHED;
        }
        return RETURN_OK;
    }

    return RETURN_ERROR;
}

inline void scurve_solver_build_stop_curve(struct scurve_data *data) {

    double jermax = data->maxjerk;
    double accinf = data->maxacc * 2;
    double curvel = data->vr;
    double curacc = data->ar;
    double delvel = scurve_delta_velocity(data);
    double endacc = data->taracc;
    double endvel = data->tarvel;

    // Reset previous position and set the current time.
    data->oldpos += data->sr;
    data->sr = 0;
    data->curtime = 0;

    // Reset the periods.
    scurve_reset_period(&data->c0);
    scurve_reset_period(&data->c1);
    scurve_reset_period(&data->c2);
    scurve_reset_period(&data->c3);
    scurve_reset_period(&data->c4);

    // Handle the case where current acceleration is zero (curve from curvel to endvel).
    if (curacc == 0) {
        jermax = -fabs(jermax);
        accinf = -fabs(accinf);
        scurve_t1_t2_t3_build(jermax,
                              accinf,
                              curvel,
                              endvel,
                              &data->c1,
                              &data->c2,
                              &data->c3);
    }

    // Handle the case where current acceleration is positive.
    if (curacc > 0) {
        jermax = fabs(jermax);
        accinf = fabs(accinf);
        scurve_t3_build(jermax,
                        accinf,
                        curvel,
                        curacc,
                        endacc,
                        &data->c0);

        jermax = -fabs(jermax);
        accinf = -fabs(accinf);
        scurve_t1_t2_t3_build(jermax,
                              accinf,
                              data->c0.velend,
                              endvel,
                              &data->c1,
                              &data->c2,
                              &data->c3);
    }

    // Handle the case where current acceleration is negative.
    if (curacc < 0) {
        jermax = -fabs(jermax);
        accinf = -fabs(accinf);
        scurve_t3_build(jermax, accinf, curvel, curacc, endacc, &data->c3); // Check if t3 is valid to endvel.

        if (data->c3.velend == endvel) {
            // printf("use t3 only. \n");
        } else if (curacc == accinf) { // If at maxacc, use period t2+t3.
            scurve_t2_build(curvel, 0.5 * delvel, accinf, &data->c2);
            scurve_t3_build(jermax, accinf, data->c2.velend, accinf, endacc, &data->c3);
            if (data->c3.velend == endvel) {
                // printf("use t2,t3. \n");
            }
        } else { // Use periods t1,t2,t3, check for custom accinf.
            scurve_t1_build(jermax, curvel, curacc, accinf, &data->c1); // Get vo of t1.
            double vo = data->c1.velini;

            scurve_t1_t2_t3_build(jermax, accinf, vo, endvel, &data->c1, &data->c2, &data->c3); // Get eventual custom as.

            double as = data->c1.accend;
            scurve_t1_build(jermax, curvel, curacc, as, &data->c1); // Apply custom as.
            // printf("use t1,t2,t3, accinf: %f \n", as);
        }
    }

    // Build the final stop phase (t4).
    scurve_t4_build(endvel, &data->c4);
    data->c4.timbeg = 0;
    data->c4.timend = INFINITY;
    data->c4.velbeg = data->tarvel;
    data->c4.velend = data->tarvel;
    data->c4.disbeg = 0;
    data->c4.disend = INFINITY;
    data->c4.accbeg = 0;
    data->c4.accend = 0;
}

// Forward scurve algorithm to build the curve
inline void scurve_solver_build_curve(struct scurve_data *data) {

    double jermax = data->maxjerk;
    double accinf = data->maxacc * 2;
    double maxvel = data->maxvel;
    double curvel = data->vr;
    double curacc = data->ar;
    double delvel = scurve_delta_velocity(data);
    double endacc = 0;

    data->oldpos += data->sr;
    data->sr = 0;
    data->curtime = 0;

    scurve_reset_period(&data->c0);
    scurve_reset_period(&data->c1);
    scurve_reset_period(&data->c2);
    scurve_reset_period(&data->c3);
    scurve_reset_period(&data->c4);

    if (curacc == 0) {
        jermax = fabs(jermax);
        accinf = fabs(accinf);
        scurve_t1_t2_t3_build(jermax, accinf, curvel, maxvel, &data->c1, &data->c2, &data->c3);
    }
    if (curacc < 0) {
        jermax = -fabs(jermax);
        accinf = -fabs(accinf);
        scurve_t3_build(jermax, accinf, curvel, curacc, endacc, &data->c0);

        jermax = fabs(jermax);
        accinf = fabs(accinf);
        scurve_t1_t2_t3_build(jermax, accinf, data->c0.velend, maxvel, &data->c1, &data->c2, &data->c3);
    }
    if (curacc > 0) {
        jermax = fabs(jermax);
        accinf = fabs(accinf);
        scurve_t3_build(jermax, accinf, curvel, curacc, endacc, &data->c3);

        if (data->c3.velend == maxvel) {
            // Use t3 only
        } else if (curacc == accinf) {
            scurve_t2_build(curvel, maxvel - (0.5 * delvel), accinf, &data->c2);
            scurve_t3_build(jermax, accinf, data->c2.velend, accinf, endacc, &data->c3);
        } else {
            scurve_t1_build(jermax, curvel, curacc, accinf, &data->c1);
            double vo = data->c1.velini;
            scurve_t1_t2_t3_build(jermax, accinf, vo, maxvel, &data->c1, &data->c2, &data->c3);
            double as = data->c1.accend;
            scurve_t1_build(jermax, curvel, curacc, as, &data->c1);
        }
    }

    scurve_t4_build(data->maxvel, &data->c4);
    data->c4.timbeg = 0;
    data->c4.timend = INFINITY;
    data->c4.velbeg = data->maxvel;
    data->c4.disbeg = 0;
    data->c4.disend = INFINITY;
    data->c4.accbeg = 0;
    data->c4.accend = 0;
}

// Update function for the scurve
inline void scurve_solver_update(struct scurve_data *data) {
    data->curtime += data->interval_time_ms;
    data->oldpos = data->sr;

    if (data->curtime < scurve_time_period(&data->c0)) {
        scurve_t3_play(data->curtime,
                       data->c0.accinf,
                       data->c0.jermax,
                       data->c0.timbeg,
                       data->c0.timend,
                       data->c0.velini,
                       data->c0.disbeg,
                       &data->vr,
                       &data->sr,
                       &data->ar);
    }
    if (data->curtime >= scurve_time_period(&data->c0) &&
            data->curtime <= scurve_time_period(&data->c0) + scurve_time_period(&data->c1)) {
        scurve_t1_play(data->curtime - scurve_time_period(&data->c0),
                       data->c1.accinf,
                       data->c1.jermax,
                       data->c1.timbeg,
                       data->c1.timend,
                       data->c1.velini,
                       data->c1.disbeg,
                       &data->vr,
                       &data->sr,
                       &data->ar);
        data->sr += scurve_displacement_period(&data->c0);
    }
    if (data->curtime > scurve_time_period(&data->c0) + scurve_time_period(&data->c1) &&
            data->curtime < scurve_time_period(&data->c0) + scurve_time_period(&data->c1) + scurve_time_period(&data->c2)) {
        scurve_t2_play(data->curtime - (scurve_time_period(&data->c0) + scurve_time_period(&data->c1)),
                       data->c2.accinf,
                       data->c2.timbeg,
                       data->c2.timend,
                       data->c2.velbeg,
                       &data->vr,
                       &data->sr,
                       &data->ar);
        data->sr += scurve_displacement_period(&data->c0) + scurve_displacement_period(&data->c1);
    }
    if (data->curtime >= scurve_time_period(&data->c0) + scurve_time_period(&data->c1) + scurve_time_period(&data->c2) &&
            data->curtime <= scurve_time_period(&data->c0) + scurve_time_period(&data->c1) + scurve_time_period(&data->c2) + scurve_time_period(&data->c3)) {
        scurve_t3_play(data->curtime - (scurve_time_period(&data->c0) + scurve_time_period(&data->c1) + scurve_time_period(&data->c2)),
                       data->c3.accinf,
                       data->c3.jermax,
                       data->c3.timbeg,
                       data->c3.timend,
                       data->c3.velini,
                       data->c3.disbeg,
                       &data->vr,
                       &data->sr,
                       &data->ar);
        data->sr += scurve_displacement_period(&data->c0) + scurve_displacement_period(&data->c1) + scurve_displacement_period(&data->c2);
    }
    if (data->curtime > scurve_time_period(&data->c0) + scurve_time_period(&data->c1) + scurve_time_period(&data->c2) + scurve_time_period(&data->c3)) {
        scurve_t4_play(data->curtime - (scurve_time_period(&data->c0) + scurve_time_period(&data->c1) + scurve_time_period(&data->c2) + scurve_time_period(&data->c3)),
                       data->c4.timbeg,
                       data->c4.velbeg,
                       &data->vr,
                       &data->sr,
                       &data->ar);
        data->sr += scurve_displacement_period(&data->c0) + scurve_displacement_period(&data->c1) + scurve_displacement_period(&data->c2) + scurve_displacement_period(&data->c3);
    }

    data->incpos = data->sr - data->oldpos;

    if (data->int_direction == REVERSE_DIRECTION) {
        data->curpos -= data->incpos;
        data->curvel = -fabs(data->vr);
        data->curacc = data->ar;
    }
    if (data->int_direction == FORWARD_DIRECTION) {
        data->curpos += data->incpos;
        data->curvel = data->vr;
        data->curacc = data->ar;
    }
}

// Function definitions
inline void scurve_t3_play(double attime,
                           double accinf,
                           double jermax,
                           double timbeg,
                           double timend,
                           double velini,
                           double disbeg,
                           double *velend,
                           double *disend,
                           double *accend) {

    double jm = jermax;
    double as = accinf;
    double ts = timbeg;
    double te = timend;
    double vo = velini;

    ts += attime;
    ts = fmin(ts, te);

    double vr = vo + as * ts - jm * (ts * ts) / 2;
    double sr = vo * ts + as * (ts * ts) / 2 - jm * (ts * ts * ts) / 6;
    double ar = as - jm * ts;

    *velend = vr;
    *disend = sr;
    *accend = ar;

    *disend -= disbeg;
}

inline void scurve_t3_build(double jermax,
                            double accinf,
                            double curvel,
                            double curacc,
                            double endacc,
                            struct scurve_period *c3) {

    double vrs,srs,ars;
    double vre,sre,are;

    double jm=jermax;
    double as=accinf;

    double ts=(as-curacc)/jm;   // Time start.
    double te=(as-endacc)/jm;   // Time end.

    double vo=curvel-as*ts+0.5*jm*(ts*ts);              // Derived from v=vh+as*t-jm*(t*t)/2;

    //vrs=vo + as*ts - jm*(ts*ts)/2;                    // v=vo + as*t - jm*(t*t)/2;
    srs=vo*ts + as*(ts*ts)/2 - jm*(ts*ts*ts)/6;         // s=vo*t + as*(t*t)/2 - jm*(t*t*t)/6;
    //ars=as-jm*ts;                                     // a=as-jm*t;

    vre=vo + as*te - jm*(te*te)/2;                      // v=vo + as*t - jm*(t*t)/2;
    sre=vo*te + as*(te*te)/2 - jm*(te*te*te)/6;         // s=vo*t + as*(t*t)/2 - jm*(t*t*t)/6;
    //are=as-jm*te;                                     // a=as-jm*t;

    c3->disbeg = srs;
    c3->disend = sre;
    c3->velend = vre;
    c3->velini = vo;
    c3->accbeg = curacc;
    c3->accend = endacc;
    c3->timbeg = ts;
    c3->timend = te;
    c3->accinf = as;
    c3->jermax = jm;
}

inline void scurve_t1_play(double at_time,
                           double accinf,
                           double jermax,
                           double timbeg,
                           double timend,
                           double velini,
                           double disbeg,
                           double *velend,
                           double *disend,
                           double *accend) {

    double as = accinf;
    double jm = jermax;
    double ts = timbeg;
    double te = timend;
    double vo = velini;

    ts += at_time;
    ts = fmin(ts, te);

    double vr = vo + jm * (ts * ts) / 2;
    double sr = vo * ts + jm * (ts * ts * ts) / 6;
    double ar = jm * ts;

    *velend = vr;
    *disend = sr;
    *accend = ar;

    *disend -= disbeg;
}

inline void scurve_t1_build(double jermax,
                            double curvel,
                            double curacc,
                            double endacc,
                            struct scurve_period *c1) {

    double vrs,srs,ars;
    double vre,sre,are;

    double jm=jermax;

    double ts=curacc/jm; // Time start.
    double te=endacc/jm; // Time end.

    double vo=curvel-(jm*ts*ts)/2.0; // Calculate the vo given the start time ts.

    // vrs=vo + jm*(ts*ts)/2;           // vo+jm*(t*t)/2;
    srs=vo*ts + jm*(ts*ts*ts)/6;        // vo*t+jm*(t*t*t)/6;
    // ars=jm*ts;                       // jm*t;

    vre=vo + jm*(te*te)/2;              // vo+jm*(t*t)/2;
    sre=vo*te + jm*(te*te*te)/6;        // vo*t+jm*(t*t*t)/6;
    // are=jm*te;                       // jm*t;

    c1->velini = vo;
    c1->velend = vre;
    c1->disbeg = srs;
    c1->disend = sre;
    c1->accbeg = curacc;
    c1->accend = endacc;
    c1->timbeg = ts;
    c1->timend = te;
    c1->jermax = jm;
}

inline void scurve_t2_play(double attime,
                           double accinf,
                           double timsta,
                           double timend,
                           double velbeg,
                           double *velend,
                           double *disend,
                           double *accend) {

    double ts = timsta;
    double te = timend;
    double vo = velbeg;
    double as = accinf;

    ts += attime;
    ts = fmin(ts, te);

    double vr = vo + as * ts;
    double sr = (vr * vr - vo * vo) / (2 * as);
    double ar = as;

    *velend = vr;
    *disend = sr;
    *accend = ar;
}

inline void scurve_t2_build(double curvel,
                            double endvel,
                            double accinf,
                            struct scurve_period *c2) {

    double vo = curvel;
    double ve = endvel;
    double as = accinf;

    double tr = (ve - vo) / as;
    double sr = (ve * ve - vo * vo) / (2 * as);

    if (vo == ve) {
        tr = 0;
        sr = 0;
    }

    c2->velini = curvel;
    c2->velbeg = curvel;
    c2->velend = endvel;
    c2->disbeg = 0;
    c2->disend = sr;
    c2->accbeg = accinf;
    c2->accend = accinf;
    c2->timbeg = 0;
    c2->timend = tr;
    c2->accinf = accinf;
}

inline void scurve_t4_play(double attime,
                           double timsta,
                           double velbeg,
                           double *velend,
                           double *disend,
                           double *accend) {

    double ts = timsta;
    double vo = velbeg;

    ts += attime;

    double vr = vo;
    double sr = vo * ts;
    double ar = 0;

    *velend = vr;
    *disend = sr;
    *accend = ar;
}

inline void scurve_t4_build(double curvel,
                            struct scurve_period *c4) {

    c4->velini = curvel;
    c4->velbeg = curvel;
    c4->velend = curvel;
    c4->disbeg = 0;
    c4->disend = 0;
    c4->accbeg = 0;
    c4->accend = 0;
    c4->timbeg = 0;
    c4->timend = 0;
}

inline void scurve_t1_t2_t3_build(double jermax,
                                  double accinf,
                                  double curvel,
                                  double endvel,
                                  struct scurve_period *c1,
                                  struct scurve_period *c2,
                                  struct scurve_period *c3) {

    double dv = scurve_delta_vel(jermax, accinf);
    double velshif = fabs(curvel - endvel);

    double zeroac = 0;

    if (velshif < dv) {
        double custas = 0;
        double velhal = (curvel + endvel) * 0.5;

        // printf("velhal: %f \n",velhal);

        scurve_t1_t3_custom_as(jermax, curvel, endvel, &custas);
        scurve_t1_build(jermax, curvel, zeroac, custas, c1);
        scurve_t2_build(velhal, velhal, custas, c2);
        scurve_t3_build(jermax, accinf, velhal, custas, zeroac, c3);
    }

    if (velshif >= dv) {
        scurve_t1_build(jermax, curvel, zeroac, accinf, c1);

        double v = velshif - dv;
        if (jermax < 0) {
            v = -fabs(v);
        }
        scurve_t2_build(c1->velend, c1->velend + v, accinf, c2);
        scurve_t3_build(jermax, accinf, c2->velend, accinf, zeroac, c3);
    }
}

inline double scurve_delta_vel(double jm,
                               double as){

    double dvt = (2 * as) / jm;
    double dv = fabs((dvt * as) / 2);

    if(isnan(dv)){
        printf("dv isnan. \n");
    }

    return dv;
}

inline void scurve_t1_t3_custom_as(double jm,
                                   double curvel,
                                   double endvel,
                                   double *as){

    double vo = curvel;
    double ve = endvel;
    double t1 = 0;

    // Exception to avoid isnan produced by sqrt function.
    if(vo==ve){
        t1=0;
    } else
        if(jm==0){
            t1=0;
        } else {

            // t1=2*(ve-vo)/as;
            // jm=2*as/t1.
            // as=2*A.
            // dv = fabs((dvt * as) / 2);
            // t1 = jm / (2*as)
            // dvt = 2 * as / jm;

            double val = (ve - vo) / jm;

            if(isnan(val)){
                printf("val isnan \n");
            }

            t1 = sqrt( (ve - vo) / jm); // Takes 50000ns wow.
        }

    *as = jm * t1;

    if(isnan(*as)){
        // printf("as inan. \n");
    }

    // Exception to avoid isnan produced by sqrt function.
    if(jm<0 && isnan(*as) && vo<=ve){ // vo == ve.
        t1 = sqrt( (ve - vo) / fabs(jm));
        *as = fabs(jm) * t1;
    }

    if(isnan(*as)){
        /*
        printf("vo: %f \n",vo);
        printf("ve: %f \n",ve);
        printf("t1: %f \n",t1);
        printf("jm: %f \n",jm);
        printf("as isnan. \n");
        */
    }
}

int scurve_has_nan(const struct scurve_data *data, int debug) {
    if (data == NULL) {
        return 0; // or handle the NULL case as per your requirements
    }

    // Check settings
    if (isnan(data->interval_time_ms) || isnan(data->maxjerk) || isnan(data->maxvel) || isnan(data->maxacc)) {
        if(debug){ printf("has nan 1. \n"); }
        return 1;
    }

    // Check current values
    if (isnan(data->curpos) || isnan(data->curvel) || isnan(data->curacc)) {
        if(debug){ printf("has nan 2. \n"); }
        return 1;
    }

    // Check target values
    if (isnan(data->taracc) || isnan(data->tarvel) || isnan(data->tarpos)) {
        if(debug){ printf("has nan 3. \n"); }
        return 1;
    }

    // Check internals
    if (isnan(data->incpos) || isnan(data->oldpos) || isnan(data->vr) || isnan(data->ar) || isnan(data->sr) || isnan(data->curtime)) {
        if(debug){ printf("has nan 4. \n"); }
        return 1;
    }

    // Check scurve_overshoot (pd)
    if (isnan(data->pd.stopdist)) {
        if(debug){ printf("has nan 5.0. \n"); }
        return 1;
    }

    // Check scurve_overshoot (pd)
    if (isnan(data->pd.stoptime)) {
        if(debug){ printf("has nan 5.1. \n"); }
        return 1;
    }

    // Check scurve_overshoot (pd)
    if (isnan(data->pd.overshoot)) {
        if(debug){ printf("has nan 5.2. \n"); }
        return 1;
    }

    // Check scurve_period (c0)
    if (isnan(data->c0.accbeg) || isnan(data->c0.accend) || isnan(data->c0.velini) || isnan(data->c0.velbeg) || isnan(data->c0.velend) ||
            isnan(data->c0.disbeg) || isnan(data->c0.disend) || isnan(data->c0.timbeg) || isnan(data->c0.timend) || isnan(data->c0.jermax) || isnan(data->c0.accinf)) {
        if(debug){ printf("has nan 6. \n"); }
        return 1;
    }

    // Check scurve_period (c1)
    if (isnan(data->c1.accbeg) || isnan(data->c1.accend) || isnan(data->c1.velini) || isnan(data->c1.velbeg) || isnan(data->c1.velend) ||
            isnan(data->c1.disbeg) || isnan(data->c1.disend) || isnan(data->c1.timbeg) || isnan(data->c1.timend) || isnan(data->c1.jermax) || isnan(data->c1.accinf)) {
        if(debug){ printf("has nan 7. \n"); }
        return 1;
    }

    // Check scurve_period (c2)
    if (isnan(data->c2.accbeg) || isnan(data->c2.accend) || isnan(data->c2.velini) || isnan(data->c2.velbeg) || isnan(data->c2.velend) ||
            isnan(data->c2.disbeg) || isnan(data->c2.disend) || isnan(data->c2.timbeg) || isnan(data->c2.timend) || isnan(data->c2.jermax) || isnan(data->c2.accinf)) {
        if(debug){  printf("has nan 8. \n"); }
        return 1;
    }

    // Check scurve_period (c3)
    if (isnan(data->c3.accbeg) || isnan(data->c3.accend) || isnan(data->c3.velini) || isnan(data->c3.velbeg) || isnan(data->c3.velend) ||
            isnan(data->c3.disbeg) || isnan(data->c3.disend) || isnan(data->c3.timbeg) || isnan(data->c3.timend) || isnan(data->c3.jermax) || isnan(data->c3.accinf)) {
        if(debug){ printf("has nan 9. \n"); }
        return 1;
    }

    // Check scurve_period (c4)
    if (isnan(data->c4.accbeg) || isnan(data->c4.accend) || isnan(data->c4.velini) || isnan(data->c4.velbeg) || isnan(data->c4.velend) ||
            isnan(data->c4.disbeg) || isnan(data->c4.disend) || isnan(data->c4.timbeg) || isnan(data->c4.timend) || isnan(data->c4.jermax) || isnan(data->c4.accinf)) {
        if(debug){ printf("has nan 10. \n"); }
        return 1;
    }

    // If no NaN values were found
    return 0;
}

void scurve_print_data(const struct scurve_data *data) {
    if (data == NULL) {
        printf("scurve_data is NULL.\n");
        return;
    }

    printf("scurve_data:\n");
    printf("  Settings:\n");
    printf("    interval_time_ms: %f\n", data->interval_time_ms);
    printf("    maxjerk: %f\n", data->maxjerk);
    printf("    maxvel: %f\n", data->maxvel);
    printf("    maxacc: %f\n", data->maxacc);

    printf("  Current values:\n");
    printf("    curpos: %f\n", data->curpos);
    printf("    curvel: %f\n", data->curvel);
    printf("    curacc: %f\n", data->curacc);

    printf("  Target values:\n");
    printf("    taracc: %f\n", data->taracc);
    printf("    tarvel: %f\n", data->tarvel);
    printf("    tarpos: %f\n", data->tarpos);

    printf("  States:\n");
    printf("    direction: %d\n", data->direction);

    printf("  Commands:\n");
    printf("    pausing: %d\n", data->pausing);

    printf("  Internals:\n");
    printf("    incpos: %f\n", data->incpos);
    printf("    oldpos: %f\n", data->oldpos);
    printf("    vr: %f\n", data->vr);
    printf("    ar: %f\n", data->ar);
    printf("    sr: %f\n", data->sr);
    printf("    curtime: %f\n", data->curtime);

    printf("  scurve_overshoot (pd):\n");
    printf("    stopdist: %f\n", data->pd.stopdist);
    printf("    stoptime: %f\n", data->pd.stoptime);
    printf("    overshoot: %f\n", data->pd.overshoot);
    printf("    need_stop: %d\n", data->pd.need_stop);
    printf("    cycles: %d\n", data->pd.cycles);

    printf("  scurve_period (c0):\n");
    printf("    accbeg: %f, accend: %f\n", data->c0.accbeg, data->c0.accend);
    printf("    velini: %f, velbeg: %f, velend: %f\n", data->c0.velini, data->c0.velbeg, data->c0.velend);
    printf("    disbeg: %f, disend: %f\n", data->c0.disbeg, data->c0.disend);
    printf("    timbeg: %f, timend: %f\n", data->c0.timbeg, data->c0.timend);
    printf("    jermax: %f, accinf: %f\n", data->c0.jermax, data->c0.accinf);

    printf("  scurve_period (c1):\n");
    printf("    accbeg: %f, accend: %f\n", data->c1.accbeg, data->c1.accend);
    printf("    velini: %f, velbeg: %f, velend: %f\n", data->c1.velini, data->c1.velbeg, data->c1.velend);
    printf("    disbeg: %f, disend: %f\n", data->c1.disbeg, data->c1.disend);
    printf("    timbeg: %f, timend: %f\n", data->c1.timbeg, data->c1.timend);
    printf("    jermax: %f, accinf: %f\n", data->c1.jermax, data->c1.accinf);

    printf("  scurve_period (c2):\n");
    printf("    accbeg: %f, accend: %f\n", data->c2.accbeg, data->c2.accend);
    printf("    velini: %f, velbeg: %f, velend: %f\n", data->c2.velini, data->c2.velbeg, data->c2.velend);
    printf("    disbeg: %f, disend: %f\n", data->c2.disbeg, data->c2.disend);
    printf("    timbeg: %f, timend: %f\n", data->c2.timbeg, data->c2.timend);
    printf("    jermax: %f, accinf: %f\n", data->c2.jermax, data->c2.accinf);

    printf("  scurve_period (c3):\n");
    printf("    accbeg: %f, accend: %f\n", data->c3.accbeg, data->c3.accend);
    printf("    velini: %f, velbeg: %f, velend: %f\n", data->c3.velini, data->c3.velbeg, data->c3.velend);
    printf("    disbeg: %f, disend: %f\n", data->c3.disbeg, data->c3.disend);
    printf("    timbeg: %f, timend: %f\n", data->c3.timbeg, data->c3.timend);
    printf("    jermax: %f, accinf: %f\n", data->c3.jermax, data->c3.accinf);

    printf("  scurve_period (c4):\n");
    printf("    accbeg: %f, accend: %f\n", data->c4.accbeg, data->c4.accend);
    printf("    velini: %f, velbeg: %f, velend: %f\n", data->c4.velini, data->c4.velbeg, data->c4.velend);
    printf("    disbeg: %f, disend: %f\n", data->c4.disbeg, data->c4.disend);
    printf("    timbeg: %f, timend: %f\n", data->c4.timbeg, data->c4.timend);
    printf("    jermax: %f, accinf: %f\n", data->c4.jermax, data->c4.accinf);
}

