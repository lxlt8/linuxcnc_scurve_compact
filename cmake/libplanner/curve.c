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
#include "curve.h"
#include <math.h>
#include <stdio.h>

double distance(PmCartesian p0, PmCartesian p1){
    return sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2) + pow(p0.z - p1.z, 2));
}

PmCartesian offset_point_on_line(PmCartesian p0,
                                 PmCartesian p1,
                                 double offset){

    //    A-----------B------------C
    // (Xa,Ya)     (Xb,Yb)      (Xc,Yc)

    PmCartesian pi;
    pi.x = p0.x + (offset * (p1.x - p0.x) / distance(p0,p1));
    pi.y = p0.y + (offset * (p1.y - p0.y) / distance(p0,p1));
    pi.z = p0.z + (offset * (p1.z - p0.z) / distance(p0,p1));

    if(distance(p0,p1)==0){
        pi=p0;
    }
    return pi;
}

void init_curve(TP_STRUCT * const tp, struct emcmot_segment *seg){

    // If the .ini file has no acceleration value's for [TRAJ] given, set a value.
    if(tp->aMax<1e-3){
        printf("add acceleration value's to your .ini file, section [TRAJ] \n"
               "DEFAULT_LINEAR_ACCELERATION = 500 \n"
               "MAX_LINEAR_ACCELERATION = 500 \n");
        tp->aMax=500;
    }

    int type = seg->canon_motion_type; // See "motion_types.h"

    if (type == 1 || type == 2 || type == 4 || type == 5) {

        pmLine9Init(&seg->coords.line, &seg->start, &seg->end);

        // Line length xyz.
        seg->length_xyz = pmLine9Target_xyz(seg);
        // Line length abc, uvw.
        seg->length_abc = pmLine9Target_abc(seg);
        seg->length_uvw = pmLine9Target_uvw(seg);

        seg->kmax=0; // Zero curvature for a straight line.
    }

    if (type == 3) {

        pmCircle9Init(&seg->coords.circle,
                      &seg->start,
                      &seg->end,
                      &seg->center,
                      &seg->normal,
                      seg->turn);

        // Circle or spiral length xyz.
        seg->length_xyz = pmCircle9Target_xyz(&seg->coords.circle);
        // Line length abc, uvw.
        seg->length_abc = pmLine9Target_abc(seg);
        seg->length_uvw = pmLine9Target_uvw(seg);

        seg->radius = distance(seg->center,seg->end.tran);
        seg->kmax=1/seg->radius;
    }
}

double init_segment_length(struct emcmot_segment *seg){

    if(seg->length_xyz>0){
        seg->length_netto=seg->length_xyz;
    }
    if(seg->length_xyz==0 && seg->length_abc>0){
        seg->length_netto=seg->length_abc;
    }
    if(seg->length_xyz==0 && seg->length_abc==0 && seg->length_uvw>0){
        seg->length_netto=seg->length_uvw;
    }

    return seg->length_netto;
}

double init_curve_trajectory_length(struct vector *ptr){
    double length = 0;
    for (int i = 0; i < vector_size(ptr); i++) {
        struct emcmot_segment *seg = vector_at(ptr, i);

        if(seg->length_xyz>0){
            seg->length_netto=seg->length_xyz;
        }
        if(seg->length_xyz==0 && seg->length_abc>0){
            seg->length_netto=seg->length_abc;
        }
        if(seg->length_xyz==0 && seg->length_abc==0 && seg->length_uvw>0){
            seg->length_netto=seg->length_uvw;
        }

        seg->trajectory_length_begin = length;
        length += seg->length_netto;
        seg->trajectory_length_end = length;
    }
    return length;
}

int pmCircleAngleFromParam(PmCircle const * const circle,
                           SpiralArcLengthFit const * const fit,
                           double t,
                           double * const angle)
{
    if (fit->spiral_in) {
        t = 1.0 - t;
    }
    //TODO error or cleanup input to prevent param outside 0..1
    double s_in = t * fit->total_planar_length;

    // Quadratic formula to invert arc length -> angle

    double A = fit->b0;
    double B = fit->b1;
    double C = -s_in;

    double disc = pmSq(B) - 4.0 * A * C ;
    if (disc < 0) {
        return -1;
    }

    /*
     * Stability of inverting the arc-length relationship.
     * Since the b1 coefficient is analogous to arc radius, we can be
     * reasonably assured that it will be large enough not to cause numerical
     * errors. If this is not the case, then the arc itself is degenerate (very
     * small radius), and this condition should be caught well before here.
     *
     * Since an arc with a very small spiral coefficient will have a small b0
     * coefficient in the fit, we use the Citardauq Formula to ensure that the
     * positive root does not lose precision due to subtracting near-similar values.
     *
     * For more information, see:
     * http://people.csail.mit.edu/bkph/articles/Quadratics.pdf
     */

    double angle_out = (2.0 * C) / ( -B - pmSqrt(disc));

    if (fit->spiral_in) {
        // Spiral fit assumes that we're spiraling out, so
        // parameterize from opposite end
        angle_out = circle->angle - angle_out;
    }

    *angle = angle_out;
    return 0;
}

int pmCircleAngleFromProgress(PmCircle const * const circle,
                              SpiralArcLengthFit const * const fit,
                              double progress,
                              double * const angle)
{
    double h2;
    pmCartMagSq(&circle->rHelix, &h2);
    double s_end = pmSqrt(pmSq(fit->total_planar_length) + h2);
    // Parameterize by total progress along helix
    double t = progress / s_end;
    return pmCircleAngleFromParam(circle, fit, t, angle);
}

int emcPoseToPmCartesian(EmcPose const * const pose,
                         PmCartesian * const xyz, PmCartesian * const abc, PmCartesian * const uvw)
{
    //Direct copy of translation struct for xyz
    *xyz = pose->tran;

    //Convert ABCUVW axes into 2 pairs of 3D lines
    abc->x = pose->a;
    abc->y = pose->b;
    abc->z = pose->c;

    uvw->x = pose->u;
    uvw->y = pose->v;
    uvw->z = pose->w;
    return 0;
}

double fsign(double f)
{
    if (f>0) {
        return 1.0;
    } else if (f < 0) {
        return -1.0;
    } else {
        //Technically this should be NAN but that's a useless result for tp purposes
        return 0;
    }
}

int findSpiralArcLengthFit(PmCircle const * const circle,
                           SpiralArcLengthFit * const fit)
{
    // Additional data for arc length approximation
    double spiral_coef = circle->spiral / circle->angle;
    double min_radius = circle->radius;

    if (fsign(circle->spiral) < 0.0) {
        // Treat as positive spiral, parameterized in opposite
        // direction
        spiral_coef*=-1.0;
        // Treat final radius as starting radius for fit, so we add the
        // negative spiral term to get the minimum radius
        //
        min_radius+=circle->spiral;
        fit->spiral_in = true;
    } else {
        fit->spiral_in = false;
    }

    //Compute the slope of the arc length vs. angle curve at the start and end of the segment
    double slope_start = pmSqrt(pmSq(min_radius) + pmSq(spiral_coef));
    double slope_end = pmSqrt(pmSq(min_radius + spiral_coef * circle->angle) + pmSq(spiral_coef));

    fit->b0 = (slope_end - slope_start) / (2.0 * circle->angle);
    fit->b1 = slope_start;

    fit->total_planar_length = fit->b0 * pmSq(circle->angle) + fit->b1 * circle->angle;

    // Check against start and end angle
    double angle_end_chk = 0.0;
    int res_angle = pmCircleAngleFromParam(circle, fit, 1.0, &angle_end_chk);
    if (res_angle != 0) {
        return -1;
    }

    // Check fit against angle
    double fit_err = angle_end_chk - circle->angle;
    if (fabs(fit_err) > TP_ANGLE_EPSILON) {
        return -1;
    }

    return 0;
}

int pmCircle9InitCart(PmCircle9 * const circ9,
                      PmCartesian  const * const start_xyz,
                      PmCartesian const * const end_xyz,
                      PmCartesian const * const center,
                      PmCartesian const * const normal,
                      int turn)
{
    int xyz_fail = pmCircleInit(&circ9->xyz, start_xyz, end_xyz, center, normal, turn);
    int res_fit = findSpiralArcLengthFit(&circ9->xyz,&circ9->fit);

    if (xyz_fail || res_fit) {
        return -1;
    }
    return 0;
}

int pmCircle9Init(PmCircle9 * const circ9,
                  EmcPose const * const start,
                  EmcPose const * const end,
                  PmCartesian const * const center,
                  PmCartesian const * const normal,
                  int turn)
{
    PmCartesian start_xyz, end_xyz;
    PmCartesian start_uvw, end_uvw;
    PmCartesian start_abc, end_abc;

    emcPoseToPmCartesian(start, &start_xyz, &start_abc, &start_uvw);
    emcPoseToPmCartesian(end, &end_xyz, &end_abc, &end_uvw);

    int xyz_fail = pmCircleInit(&circ9->xyz, &start_xyz, &end_xyz, center, normal, turn);
    //Initialize line parts of Circle9
    int abc_fail = pmCartLineInit(&circ9->abc, &start_abc, &end_abc);
    int uvw_fail = pmCartLineInit(&circ9->uvw, &start_uvw, &end_uvw);

    int res_fit = findSpiralArcLengthFit(&circ9->xyz,&circ9->fit);

    if (xyz_fail || abc_fail || uvw_fail || res_fit) {
        return -1;
    }
    return 0;
}

double pmCircle9Target_xyz(PmCircle9 const * const circ9)
{
    double h2;
    pmCartMagSq(&circ9->xyz.rHelix, &h2);
    double helical_length = pmSqrt(pmSq(circ9->fit.total_planar_length) + h2);
    return helical_length;
}

int pmLine9Init(PmLine9 * const line9,
                EmcPose const * const start,
                EmcPose const * const end)
{
    // Scratch variables
    PmCartesian start_xyz, end_xyz;
    PmCartesian start_uvw, end_uvw;
    PmCartesian start_abc, end_abc;

    // Convert endpoint to cartesian representation
    emcPoseToPmCartesian(start, &start_xyz, &start_abc, &start_uvw);
    emcPoseToPmCartesian(end, &end_xyz, &end_abc, &end_uvw);

    // Initialize cartesian line members
    int xyz_fail = pmCartLineInit(&line9->xyz, &start_xyz, &end_xyz);
    int abc_fail = pmCartLineInit(&line9->abc, &start_abc, &end_abc);
    int uvw_fail = pmCartLineInit(&line9->uvw, &start_uvw, &end_uvw);

    if (xyz_fail || abc_fail || uvw_fail) {
        return -1;
    }
    return 0;
}

double pmLine9Target_xyz(const struct emcmot_segment *seg)
{
    return seg->coords.line.xyz.tmag;
}

double pmLine9Target_abc(const struct emcmot_segment *seg)
{
    return seg->coords.line.abc.tmag;
}

double pmLine9Target_uvw(const struct emcmot_segment *seg)
{
    return seg->coords.line.uvw.tmag;
}

int segment_has_motion_xyz_abc_uvw(struct emcmot_segment *seg){

    if(seg->length_xyz<1e-12 && seg->length_abc<1e-12 && seg->length_uvw<1e-12 ){
        // printf("segment has zero xyz, abc, uvw length. \n");
        return 0;
    }
    return 1;
}


















