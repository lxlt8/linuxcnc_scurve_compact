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
#ifndef MOTIONIZER_WRAPPER_H
#define MOTIONIZER_WRAPPER_H

#include "motionizer.h"  // Header where motionizer class is defined

// Creates a new motionizer instance and returns its pointer
extern struct motionizer* motionizer_init_ptr();

// Initializes motion segments and calculates velocities.
// Calls forward and reverse sweeps after updating max velocities.
// Does this for entire trajectory.
extern int motionizer_process_segments_all(motionizer* motionizer_ptr,
                                           struct vector* ptr,
                                           int debug);

// As above, then for a portion of the trajectory.
// using the number off look_ahead_lines before index and after index.
// 20 look ahead lines, will process 40 segments, 20 in the past, 20 in the future.
extern int motionizer_process_segments_portion(motionizer* motionizer_ptr,
                                               struct vector* ptr,
                                               int index,
                                               int look_ahead_lines,
                                               double speed_factor,
                                               int debug);

extern int motionizer_process_segments_portion_vo(motionizer* motionizer_ptr,
                                                  struct vector* ptr,
                                                  int index,
                                                  int look_ahead_lines,
                                                  double speed_factor,
                                                  double vo,
                                                  int debug);

#endif // MOTIONIZER_WRAPPER_H
