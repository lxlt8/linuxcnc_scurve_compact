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
#ifndef MOTIONIZER_H
#define MOTIONIZER_H

#include "segment.h"
#include "vector.h"

#ifdef __cplusplus

#include <vector>

// Class to calculate optimzal vo's, ve's, over a trajectory.
class motionizer {
public:

    // This is a vector implementation, to pass at once.
    // A motionizer segment must have at least:
    // - vm         (gui maxvel)
    // - length     (calculated by the filletizer library)
    // - kmax       (calculated by the filletizer library)
    // - amax       (gui maxacc)

    // Optimize full trajectory.
    int init(vector *ptr, int debug);

    // Optimize a portion of the trajectory.
    // Index is the current executed gcode line.
    // Look_ahead_lines is the ammount of lines too look forward, and reverse.
    int init(vector *ptr,
             int index,
             int look_ahead_lines,
             double speed_factor,
             int debug);

    int init(vector *ptr,
             int index,
             int look_ahead_lines,
             double speed_factor,
             double vo,
             int debug);

private:

    // std::vector<emcmot_segment> my_motions;

    // Function to calculate the maximum curvature from a line and an arc.
    // Line has radius 0 as input.
    double process_kmax(double radius);

    // Maxvel & maxacc are the gui's maxvel. Kmax "curvature max" is retrieved from a segment.
    int velocity_from_curvature(const double &maxacc, const double &kmax, double &maxvel);

    // Update the vector maxvel based on curvature inputs.
    int update_vector_maxvel_given_curvature(std::vector<emcmot_segment>& motions);

    // Update the vector maxvel based on curvature inputs. Do this for a p
    int update_vector_maxvel_given_curvature(std::vector<emcmot_segment>& motions,
                                             int index,
                                             int look_ahead_lines,
                                             double speed_factor);


    // Function to calculate the final velocity based on vo, maxvel, maxacc, and length
    double final_velocity(double vo, double maxvel, double maxacc, double length);

    // Function for forward sweep
    void forward_sweep(std::vector<emcmot_segment>& motions);

    // Function for forward sweep
    void forward_sweep(std::vector<emcmot_segment>& motions, int index, int look_ahead_lines);

    // Function for forward sweep
    void forward_sweep(std::vector<emcmot_segment>& motions, int index, int look_ahead_lines, double vo);

    // Function for reverse sweep
    void reverse_sweep(std::vector<emcmot_segment>& motions);

    // Function for reverse sweep
    void reverse_sweep(std::vector<emcmot_segment>& motions, int index, int look_ahead_lines);

    // Function for reverse sweep
    void reverse_sweep(std::vector<emcmot_segment>& motions, int index, int look_ahead_lines, double vo);

    // Function to print the motion segments
    void print_motions(const std::vector<emcmot_segment>& motions) const;
};

#else
typedef struct motionizer motionizer;
#endif

#endif // MOTIONIZER_H
