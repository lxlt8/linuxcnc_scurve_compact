#ifndef TRAPEZIUM_H
#define TRAPEZIUM_H

#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include "segment.h"

// Define the structure for motion segments
/* As used in the source code.
struct emcmot_segment {
    double vel; // maximum velocity (vm)
    double acc; // acceleration/deceleration
    double length_bruto, length_netto; // lengths
    double trajectory_length_begin, trajectory_length_end;
    double vo, ve; // initial and final velocities
};*/

// Define the motion state enumeration
enum motion_state_trapezium {
    forward_acc,
    forward_dcc,
    finished,
    forward_steady,
    reverse_acc,
    reverse_dcc,
    reverse_steady,
    stop,
    error
};

// Main function to generate a trapezium motion profile.
// Implementation example is in main.c
enum motion_state_trapezium trapezium_update(double *curpos,
                                             double *curvel,
                                             double *curacc,
                                             double tarpos,
                                             double maxvel,
                                             double maxacc,
                                             double endvel,
                                             double cycletime,
                                             double finish_tollerance,
                                             int motion_rev,
                                             int debug);

// Function declarations for motion handling
double stopdist(double curvel, double endvel, double acc);
void stop_motion(double *curvel, double *curpos, double *curacc, double maxacc, double cycletime);
void motion_fwd_dcc(double *curvel, double *curpos, double acc, double cycletime);
void motion_rev_dcc(double *curvel, double *curpos, double acc, double cycletime);
void motion_fwd_acc(double *curvel, double *curpos, double acc, double maxvel, double cycletime);
void motion_rev_acc(double *curvel, double *curpos, double acc, double maxvel, double cycletime);
void steady_motion_fwd(double *curvel, double *curpos, double cycletime);
void steady_motion_rev(double *curvel, double *curpos, double cycletime);

enum motion_state_trapezium motion_sub_phase(double curpos,
                                             double tarpos,
                                             double curvel,
                                             double endvel,
                                             double maxacc,
                                             double maxvel);

enum motion_state_trapezium update_next_cycle(double curpos,
                                              double curvel,
                                              double tarpos,
                                              double maxvel,
                                              double maxacc,
                                              double endvel);

#endif // TRAPEZIUM_H
