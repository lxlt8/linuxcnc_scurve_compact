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
#ifndef INTERPOLATE_H
#define INTERPOLATE_H

#include "emcmot_segment.h"
#include "curve.h"

// Interpolate abc, uvw axis given progress.
void interpolate_abc_uvw(const struct emcmot_segment *seg,
                         const double progress,
                         struct EmcPose *pos);

// Interpolate abc, uvw axis given progress.
void interpolate_abc_uvw_(const struct emcmot_segment *seg,
                         const double progress,
                         struct EmcPose *pos);

// Interpolate abc, uvw axis given progress.
void interpolate_abc_uvw_peak(const struct emcmot_segment *seg,
                              const double progress,
                              struct EmcPose *pos);

// Interpolate a 3d line.
void interpolate_line(const struct emcmot_segment *seg,
                      const double progress,
                      struct EmcPose *pos);

// Interpolate a arc, circle or helix along the way.
void interpolate_arc(const struct emcmot_segment *seg,
                     const double progress,
                     struct EmcPose *pos);

void interpolate_pm_line(PmCartesian p0,
                         PmCartesian p1,
                         double progress,
                         PmCartesian *pi);

void interpolate_pm_seg_line(const struct emcmot_segment *seg,
                             const double progress,
                             PmCartesian *pi);

void interpolate_pm_seg_arc(const struct emcmot_segment *seg,
                            const double progress,
                            PmCartesian *pi);

// Interpolate a line or orc segment.
void interpolate_pm_seg_line_arc(const struct emcmot_segment *seg,
                                 const double progress,
                                 PmCartesian *pi);
/*
// Interpolate a 3d pvec.
void interpolate_pvec(const struct emcmot_segment *seg,
                      const double progress,
                      struct EmcPose *pos);

*/

#endif // INTERPOLATE_H
