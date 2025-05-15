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
#ifndef SCURVE_H
#define SCURVE_H

enum scurve_direction_mode {
    FORWARD_DIRECTION,
    REVERSE_DIRECTION
};

enum scurve_return_code {
    RETURN_OK,
    RETURN_FINISHED,
    RETURN_ERROR
};

//! Struct used to save scurve periodic data.
struct scurve_period {
    double accbeg, accend;
    double velini, velbeg, velend;
    double disbeg, disend;
    double timbeg, timend;
    double jermax, accinf;
};

struct scurve_overshoot {
    double stopdist;
    double stoptime;
    double overshoot;
    int need_stop;
    int cycles;
};

//! Struct used to store forward or stop scurve data.
struct scurve_data {

    // Setings.
    double interval_time_ms;
    double maxjerk;
    double maxvel;
    double maxacc;

    // Current values.
    double curpos;
    double curvel;
    double curacc;

    // Check for duplicte cycles.
    double saved_pos;
    double saved_vel;
    double saved_acc;

    // Target values.
    double taracc;
    double tarvel;
    double tarpos;

    // States.
    enum scurve_direction_mode direction;
    enum scurve_direction_mode int_direction;   // Internal run direction.

    // Commands.
    int pausing;      

    // Internals.
    double incpos;              // Positive increment position for each cycle.
    double oldpos;              // Previous displacement.

    double vr, ar, sr;          // Internal "vr" velocity, "ar" acceleration, "sr" displacement.
    double curtime;

    struct scurve_overshoot pd;                 // Internal position data.
    struct scurve_period c0,c1,c2,c3,c4;        // Periods to contruct a forward or stop scurve.
};

// Initialize struct scurve_data with settings.
int scurve_init(struct scurve_data *data,
                double max_jerk,
                double max_acceleration,
                double max_velocity,
                double cycletime);

// Set the target state.
void scurve_set_target_state(struct scurve_data *data,
                             const double endvel,
                             const double endacc,
                             const double endpos,
                             const int pausing);

// Update cycle, called periodically. Usually every 1ms.
int scurve_update(struct scurve_data *data);

// Reset or clear motion data.
void scurve_reset_data(struct scurve_data *data);

void scurve_stop_analyzer(struct scurve_data *data);
void scurve_solver_build_curve(struct  scurve_data *data);
void scurve_solver_build_stop_curve(struct scurve_data *data);

void scurve_solver_update(struct scurve_data *data);

// Period and displacement calculation functions.
double scurve_time_period(struct scurve_period *p);
double scurve_displacement_period(struct scurve_period *p);

double scurve_delta_velocity(struct scurve_data *data);
double scurve_time_all_periods(struct scurve_data *data);
double scurve_displacement_all_periods(struct scurve_data *data);

// Reset functions for periods and overshoot.
void scurve_reset_period(struct scurve_period *data);
void scurve_reset_periods(struct scurve_data *data);
void scurve_reset_overshoot(struct scurve_data *data);

// Stop length calculation.
void scurve_stop_length_time(struct scurve_data *data, double *length, double *time);
double scurve_stop_length(struct scurve_data *data);

// Error solving.
int scurve_has_nan(const struct scurve_data *data, int debug);
int scurve_duplicate_cycle(const struct scurve_data *data);

// Print the output of the data structure.
void scurve_print_data(const struct scurve_data *data);

// Build the segments for the scurve.
void scurve_t1_t2_t3_build(double jermax,
                            double accinf,
                            double curvel,
                            double endvel,
                            struct scurve_period *c1,
                            struct scurve_period *c2,
                            struct scurve_period *c3);

void scurve_t1_build(double jermax,
                     double curvel,
                     double curacc,
                     double endacc,
                     struct scurve_period *c1);

void scurve_t2_build(double curvel,
                     double endvel,
                     double accinf,
                     struct scurve_period *c2);

void scurve_t3_build(double jermax,
                     double accinf,
                     double curvel,
                     double curacc,
                     double endacc,
                     struct scurve_period *c3);

void scurve_t4_build(double curvel,
                     struct scurve_period *c4);

// Play the segments for the scurve.
void scurve_t1_play(double at_time,
                    double accinf,
                    double jermax,
                    double timbeg,
                    double timend,
                    double velini,
                    double disbeg,
                    double *velend,
                    double *disend,
                    double *accend);

void scurve_t2_play(double attime,
                    double accinf,
                    double timsta,
                    double timend,
                    double velbeg,
                    double *velend,
                    double *disend,
                    double *accend);

void scurve_t3_play(double attime,
                    double accinf,
                    double jermax,
                    double timbeg,
                    double timend,
                    double velini,
                    double disbeg,
                    double *velend,
                    double *disend,
                    double *accend);

void scurve_t4_play(double attime,
                    double timsta,
                    double velbeg,
                    double *velend,
                    double *disend,
                    double *accend);

// "dv"  Delta velocity, dv=vo-ve, derived from: t1=2*(ve-vo)/as;
double scurve_delta_vel(double jm, double as);

// Custom acceleration segment for small velocity changes.
void scurve_t1_t3_custom_as(double jm, double curvel, double endvel, double *as);

#endif // SCURVE_H
