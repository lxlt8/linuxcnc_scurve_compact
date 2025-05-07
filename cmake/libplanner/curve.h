#ifndef CURVE_H
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
#define CURVE_H

#include "emcmot_segment.h"
#include "mot_priv.h"
#include "stdio.h"
#include "vector.h"

/* This file is only used by the standard path algo.
 * It uses arc interpolation using original lcnc method.
 * The clothoid library has it's own arc interpolation.
 */

// Distance between 2 points.
double distance(PmCartesian p0,
                PmCartesian p1);

// Offset a point on a line.
PmCartesian offset_point_on_line(PmCartesian p0,
                                 PmCartesian p1,
                                 double offset);

// Calculate segment lenght, kmax, etc.
void init_curve(TP_STRUCT * const tp,
                struct emcmot_segment *seg);

// Calculate trajectory lenghts for xyz, abc, uvw.
// Return the total traject length.
double init_curve_trajectory_length(struct vector *ptr);

double init_segment_length(struct emcmot_segment *seg);

// Calculate if segment has any motion at all over the xyz, abc, uvw joints.
// If segment has no length. This happens if you start without jogging.
// Lcnc will send a segment without any length.
int segment_has_motion_xyz_abc_uvw(struct emcmot_segment *seg);

int pmCircleAngleFromParam(PmCircle const * const circle,
                           SpiralArcLengthFit const * const fit,
                           double t,
                           double * const angle);
int pmCircleAngleFromProgress(PmCircle const * const circle,
                              SpiralArcLengthFit const * const fit,
                              double progress,
                              double * const angle);
int emcPoseToPmCartesian(EmcPose const * const pose,
                         PmCartesian * const xyz,
                         PmCartesian * const abc,
                         PmCartesian * const uvw);
double fsign(double f);
int findSpiralArcLengthFit(PmCircle const * const circle,
                           SpiralArcLengthFit * const fit);
int pmCircle9InitCart(PmCircle9 * const circ9,
                      PmCartesian  const * const start_xyz,
                      PmCartesian const * const end_xyz,
                      PmCartesian const * const center,
                      PmCartesian const * const normal,
                      int turn);
int pmCircle9Init(PmCircle9 * const circ9,
                  EmcPose const * const start,
                  EmcPose const * const end,
                  PmCartesian const * const center,
                  PmCartesian const * const normal,
                  int turn);
double pmCircle9Target_xyz(PmCircle9 const * const circ9);

int pmLine9Init(PmLine9 * const line9,
                EmcPose const * const start,
                EmcPose const * const end);
double pmLine9Target_xyz(const struct emcmot_segment *seg);
double pmLine9Target_abc(const struct emcmot_segment *seg);
double pmLine9Target_uvw(const struct emcmot_segment *seg);

#endif // CURVE_H
