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
#ifndef FEED_H
#define FEED_H

#include "common.h"
#include "curve.h"

/* Process gcode feeds.
 *
 * Input data from segment's tag :
 *
 *  - G93
 *  - G94
 *  - G95
 *  - FEED
 *  - SPEED
 *
 *  Gcode Feed "F" value from interpreter, ~/src/emc/rs274ngc/interp_write.cc L330
 *
 *  Gcode inverse time :    ~/src/emc/rs274ngc/interp_inverse.cc L129
 *                          ~/src/emc/rs274ngc/interp_find.cc:644 L644
 *
 *  tp->vLimit -> INI's MAX_LINEAR_VELOCITY.
 *  tp->aMax   -> INI's MAX_LINEAR_ACCELERATION.
 *
 *  FEED = tag.fields_float[1]   -> is modified by G93.
 *  SPEED = tag.fields_float[2]  -> is spindle speed.
 */

// G93 ; Inverse time G93 = 1/F minutes.
// G1 X100 F2 ; -> 1/2 minutes = 30 seconds.
int G93_active(const struct emcmot_segment *seg);
// G94 ; Units per minute has no effect here. Value passed to here is already unit/sec.
// Therefore G94 does not need further calculations.
int G94_active(const struct emcmot_segment *seg);
// Spindle nr. to use.
int G95_spindle(TP_STRUCT * const tp);
// G95 ; Units per revolution -> GM_FLAG_FEED_UPM = off.
// M3
// G1 X100 F1000 S100 ; -> Move 1000 mm/rev for each spindle rotation.
// M5
int G95_active(const struct emcmot_segment *seg);
// G1, G2, G3 takes FEED in units/min but here we use units/mm, then do /60.
// If in gcode F1000, the value is here 1000 too.
double FEED_MM_MIN(const struct emcmot_segment *seg);
double FEED_MM_SEC(const struct emcmot_segment *seg);
// Spindle speed "S" in rotations per minute.
double SPEED_RPM(const struct emcmot_segment *seg);
// Used to calculate inverse time mode -> G93.
double max_velocity(double distance,
                    double amax,
                    double total_time);
// Set the seg->motion_set leader to xyz, abc, uvw.
// Calculate motion time given the distance, acceleration and velocity.
double motion_time(double distance,
                   double vm,
                   double amax);
// Returns the joint wich takes the longest move duration.
enum enum_motion_set slowest_motion_set(struct emcmot_config_t *emcmotConfig,
                                        struct emcmot_segment *seg);
// Netto length to move.
double joint_dist(struct emcmot_segment *seg,
                  int joint);
// The interpreter calculates values. We restore them here.
double restore_inverse_feed_to_gcode_feed(const struct emcmot_segment *seg);
// Apply inverse time feed to xyz, abc, uvw moves.
void update_g93_feed(struct emcmot_segment *seg,
                     double time);
// Initialise feeds for this segment, apply G93, G94, G95.
// This restores the F value to original gcode line input value.
int init_feed(TP_STRUCT * const tp,
              struct emcmot_segment *seg);

#endif // FEED_H
