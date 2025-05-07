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
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <ctime> // for time()
#include <fstream>
#include <iomanip> // For std::fixed and std::setprecision
#include <algorithm> // For std::max and std::min
#include <cmath>
#include <limits>
#include <iostream>
#include "filletizer.h"
#include <tinysplinecxx.h>
#include "tinyspline.h"

// Here are copies of .c file functions to avoid symbol errors.
// After these functions the filletizer class begins.

void filletizer::interpolate_abc_uvw(const struct emcmot_segment *seg,
                        const double progress,
                        struct EmcPose *pos){

    pos->a = seg->start.a + (seg->end.a - seg->start.a) * progress;
    pos->b = seg->start.b + (seg->end.b - seg->start.b) * progress;
    pos->c = seg->start.c + (seg->end.c - seg->start.c) * progress;

    pos->u = seg->start.u + (seg->end.u - seg->start.u) * progress;
    pos->v = seg->start.v + (seg->end.v - seg->start.v) * progress;
    pos->w = seg->start.w + (seg->end.w - seg->start.w) * progress;
}

void filletizer::interpolate_line(const struct emcmot_segment *seg,
                     const double progress,
                     struct EmcPose *pos){

    // Calculate the interpolated point
    pos->tran.x = seg->start.tran.x + ((seg->end.tran.x - seg->start.tran.x) * progress);
    pos->tran.y = seg->start.tran.y + ((seg->end.tran.y - seg->start.tran.y) * progress);
    pos->tran.z = seg->start.tran.z + ((seg->end.tran.z - seg->start.tran.z) * progress);

    interpolate_abc_uvw(seg,progress,pos);
}

void filletizer::interpolate_arc(const struct emcmot_segment *seg,
                    const double progress,
                    struct EmcPose *pos){


    double length=progress*seg->length_xyz;

    double angle;
    pmCircleAngleFromProgress(&seg->coords.circle.xyz,
                              &seg->coords.circle.fit,
                              length, &angle);

    // Calculate position along the circular arc
    PmCartesian pi;
    pmCirclePoint(&seg->coords.circle.xyz, angle, &pi);

    pos->tran.x=pi.x;
    pos->tran.y=pi.y;
    pos->tran.z=pi.z;

    interpolate_abc_uvw(seg,progress,pos);
}


int filletizer::pmCircleAngleFromParam(PmCircle const * const circle,
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

int filletizer::pmCircleAngleFromProgress(PmCircle const * const circle,
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

int filletizer::emcPoseToPmCartesian(EmcPose const * const pose,
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

double filletizer::fsign(double f)
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

int filletizer::findSpiralArcLengthFit(PmCircle const * const circle,
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

int filletizer::pmCircle9InitCart(PmCircle9 * const circ9,
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

int filletizer::pmCircle9Init(PmCircle9 * const circ9,
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

double filletizer::pmCircle9Target_xyz(PmCircle9 const * const circ9)
{
    double h2;
    pmCartMagSq(&circ9->xyz.rHelix, &h2);
    double helical_length = pmSqrt(pmSq(circ9->fit.total_planar_length) + h2);
    return helical_length;
}

int filletizer::pmLine9Init(PmLine9 * const line9,
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

double filletizer::pmLine9Target_xyz(const emcmot_segment& seg)
{
    return seg.coords.line.xyz.tmag;
}

double filletizer::pmLine9Target_abc(const emcmot_segment& seg)
{
    return seg.coords.line.abc.tmag;
}

double filletizer::pmLine9Target_uvw(const emcmot_segment& seg)
{
    return seg.coords.line.uvw.tmag;
}

filletizer::filletizer(){}

int filletizer::filletize(emcmot_segment *s0,   // Gcode line or arc.
                          emcmot_segment *s1,   // Empty fillet segment.
                          emcmot_segment *s2,   // Gcode line or arc.
                          const fillet_types &fillet_type,
                          int debug){

    double s0_trim, s2_trim, trim;

    // Get the trim value that satisfies both segments without
    // the possibility of going trough the workplane.
    // This can occur when trim values between segments are non equal.
    correct_trim_length(s0,s0_trim);
    correct_trim_length(s2,s2_trim);
    trim=std::fmin(s0_trim,s2_trim);

    if(trim==0){   // When G64 P0, no fillet can be made.
        // This results in a empty fillet segment.
        // the start point, end point are then identical.
        // The segment has zero length and curvature.
        s1->fillet_type=FILLET_TYPE_NONE;
    }

    if(trim>0){
        s1->fillet_type=fillet_type;
        // Trim the curves with identical trim value.

        // Trim back.
        trim_curve(s0,false,true,trim,false);
        // Trim front.
        trim_curve(s2,true,false,trim*0.5,false);
    }

    // Process the inbetween fillet segment.
    int res=init_fillet_segment(s0,s1,s2,debug);

    // Print results in order.
    if(debug){
        std::cout<<""<<std::endl;
        std::cout<<"-- Filletizer start set of 3 segments in order. --"<<std::endl;
        //print_short(s0);
        //print_short(s1);
        //print_short(s2);
        std::cout<<"-- Filletizer end. --"<<std::endl;
        std::cout<<""<<std::endl;
    }

    return res;
}

int filletizer::interpolate_segment(const emcmot_segment *seg,
                                    const double progress,
                                    EmcPose *pos){

    // Output motion type using switch for better readability
    switch (seg->canon_motion_type) {
    case 1:
        // std::cout << " Motion rapid" << std::endl;
        interpolate_line(seg,progress,pos);
        break;
    case 2:
        // std::cout << " Motion Linear." << std::endl;
        interpolate_line(seg,progress,pos);
        break;
    case 3:
        // std::cout << " Motion Arc." << std::endl;
        interpolate_arc(seg,progress,pos);
        break;
    case 10:
        // std::cout << " Motion Fillet." << std::endl;

        switch (seg->fillet_type) { // Access using dot operator
        case FILLET_TYPE_NONE:
            // std::cout << " Fillet None" << std::endl;
            break;
        case FILLET_TYPE_BSPLINE:
            // std::cout << " Fillet Bspline" << std::endl;
            // interpolateBSpline(seg,progress,pos);
            interpolatePvec(seg,progress,pos);
            break;
        case FILLET_TYPE_LINE:
            interpolate_line(seg,progress,pos);
            // std::cout << " Fillet Line." << std::endl;
            break;
        default:
            // std::cout << " Fillet Unknown." << std::endl;
            break;
        }

        break;
    default:
        // std::cout << " Motion Unknown." << std::endl;
        break;
    }

    return 0;
}

// Helper function to calculate Euclidean distance between two points
double filletizer::distance(const PmCartesian& p0, const PmCartesian& p1){
    return std::sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2) + pow(p0.z - p1.z, 2));
}

int filletizer::interpolateCircleByLength(const PmCartesian& p0,
                                          const PmCartesian& pc,
                                          const PmCartesian& pn,
                                          const PmCartesian& p1,
                                          int turn,
                                          const double &length, PmCartesian& pi){

    PmCircle9 circle;
    EmcPose start;
    start.tran=p0;
    EmcPose end;
    end.tran=p1;

    start.tran=p0;
    pmCircle9Init(&circle, &start, &end, &pc, &pn, turn);

    double angle;
    pmCircleAngleFromProgress(&circle.xyz,
                              &circle.fit,
                              length, &angle);

    // Calculate position along the circular arc
    pmCirclePoint(&circle.xyz, angle, &pi);

    return 0;
}

double filletizer::lineLength(struct emcmot_segment& seg){
    return distance(seg.start.tran,seg.end.tran);
}

double filletizer::circleLength(struct emcmot_segment& seg){
    pmCircle9Init(&seg.coords.circle, &seg.start, &seg.end, &seg.center, &seg.normal, seg.turn);
    return pmCircle9Target_xyz(&seg.coords.circle);
}

int filletizer::isClosedCircle(const struct emcmot_segment *seg){

    const PmCartesian p0 = seg->start.tran;  // Original start position
    const PmCartesian p1 = seg->end.tran;    // Original end position

    if(distance(p0,p1)<1e-12){
        return 1;
    }
    return 0;
}

int filletizer::trimCircle(struct emcmot_segment *seg,
                           const double trim,
                           const int &trim_front,
                           const int &trim_back,
                           bool debug) {

    const PmCartesian p0 = seg->start.tran;  // Original start position
    const PmCartesian pc = seg->center;
    const PmCartesian pn = seg->normal;
    const PmCartesian p1 = seg->end.tran;    // Original end position
    const int turn=seg->turn;
    // const int plane=seg->tag.fields[4];

    // Dont trim a full closed circle.
    if(isClosedCircle(seg)){
        std::cout<<"don't trim a full closed circle, pass."<<std::endl;
        return 0;
    }

    // Declare PmCartesian variables for new positions
    PmCartesian pi0, pi1, pi2, pi3;

    // Update segment with new positions
    if(trim_front){
        double tsc=trim*CONTROL_POINT_OFFSET;
        // std::cout<<"trim_start_control:"<<tsc<<std::endl;
         interpolateCircleByLength(p0,pc,pn,p1,turn,tsc,pi0);

        double ts=trim;
        // std::cout<<"trim_start:"<<ts<<std::endl;
       interpolateCircleByLength(p0,pc,pn,p1,turn,ts,pi1);

        seg->start_control = pi0;
        seg->start.tran = pi1;
    }
    // Update segment with new positions
    if(trim_back){
        double te=seg->length_xyz-trim;
        // std::cout<<"trim_end:"<<te<<std::endl;
        interpolateCircleByLength(p0,pc,pn,p1,turn,te,pi2);

        double tse=seg->length_xyz-(trim*CONTROL_POINT_OFFSET);
        // std::cout<<"trim_end_control:"<<tse<<std::endl;
       interpolateCircleByLength(p0,pc,pn,p1,turn,tse,pi3);

        seg->end.tran = pi2;
        seg->end_control = pi3;
    }

    // Update segment length after trim. Store value in length_netto
    // Create a copy of seg, then calculate length given
    // trim points.
    emcmot_segment s=*seg;
    // Function to input PmCartesian.
    pmCircle9InitCart(&s.coords.circle,
                      &s.start.tran,
                      &s.end.tran,
                      &s.center,
                      &s.normal,
                      s.turn);
    s.length_netto = pmCircle9Target_xyz(&s.coords.circle);
    seg->length_netto=s.length_netto; // Copy in result.

    if (debug) {
        if (debug) {
            std::cout<<""<<std::endl;
            std::cout<<"circle id:"<<seg->id<<std::endl;
            //printPoint(seg->start_trim,"start");
            //printPoint(seg->end_trim,"end");
            std::cout<<"length netto:"<<seg->length_netto<<std::endl;
            std::cout<<""<<std::endl;
        }
    }

    return 0;
}

int filletizer::trimLine(struct emcmot_segment* seg,
                         double trim,
                         const int& trim_front,
                         const int& trim_back,
                         bool debug) {

    const PmCartesian p0 = seg->start.tran;  // Original start position
    const PmCartesian p1 = seg->end.tran;    // Original end position

    // Declare PmCartesian variables for new positions
    EmcPose *pi0, *pi1, *pi2, *pi3;
    double progress;

    // Update segment with new positions.
    if(trim_front){
        // Start trim
        progress = trim / seg->length_xyz;
        interpolate_line(seg, progress, pi0);

        // Start control
        progress = (trim * CONTROL_POINT_OFFSET) / seg->length_xyz;
        interpolate_line(seg, progress, pi1);

        seg->start.tran = pi0->tran;
        seg->start_control = pi1->tran;
    }
    if(trim_back){
        // End trim
        progress = (seg->length_xyz - trim) / seg->length_xyz;
        interpolate_line(seg, progress, pi2);

        // End control
        progress = (seg->length_xyz - (trim * CONTROL_POINT_OFFSET)) / seg->length_xyz;
        interpolate_line(seg, progress, pi3);

        seg->end.tran = pi2->tran;
        seg->end_control = pi3->tran;
    }
    // Update segment length after trim.
    seg->length_netto=distance(seg->start.tran, seg->end.tran);

    if (debug) {
        std::cout<<""<<std::endl;
        std::cout<<"line id:"<<seg->id<<std::endl;
        //printPoint(seg->start_trim,"start");
        //printPoint(seg->end_trim,"end");
        std::cout<<"length netto:"<<seg->length_netto<<std::endl;
        std::cout<<""<<std::endl;
    }

    return 0;
}

void filletizer::correct_trim_length(const emcmot_segment *seg, double& max_trim){

    max_trim=seg->tag.fields_float[3]; // Gcode path deviation.

    // Trim conditions.
    if(max_trim<=1e-6){
        max_trim = 0;
    }

    // Let the trim be valid if done from both sides.
    if (max_trim >= seg->length_netto) {
        max_trim = seg->length_netto-1e-3;
    }
}

int filletizer::trim_curve(emcmot_segment* seg,
                           const int& trim_front,
                           const int& trim_back,
                           double trim_length,
                           int debug) {

    if(seg->canon_motion_type==1 || seg->canon_motion_type==2){
        trimLine(seg,trim_length,trim_front,trim_back,debug);
    }
    if(seg->canon_motion_type==3){
        trimCircle(seg,trim_length,trim_front,trim_back,debug);
    }

    return 0;
}

int filletizer::init_fillet_segment(const emcmot_segment *s0,
                                    emcmot_segment *s1,
                                    const emcmot_segment *s2,
                                    int debug){

    s1->canon_motion_type = 10; // Custom motion type for clothoid or bspline fillets

    // Initialize essential properties for the new fillet segment
    s1->start.tran = s0->end.tran;
    s1->start_control = s0->end_control;
    s1->end.tran = s2->start.tran;
    s1->end_control = s2->start_control;

    // Use properties from the first segment.
    s1->vel = s0->vel;
    s1->ini_maxvel = s0->ini_maxvel;
    s1->acc = s0->acc;
    s1->enables = s0->enables;
    s1->atspeed = s0->atspeed;
    s1->indexer_jnum = s0->indexer_jnum;
    s1->turn = 0;  // Make sure this is set appropriately based on your logic
    s1->tag = s0->tag;
    s1->id = s0->id;
    s1->length_xyz = 0;
    s1->length_netto = 0;
    s1->kmax = 0;

    // Calculate segment length's based on fillet type.
    if(s1->fillet_type==FILLET_TYPE_NONE){
        s1->length_xyz=0;
        s1->length_netto=0;
        s1->kmax=0;

        if (debug) {
            if (debug) {
                std::cout<<""<<std::endl;
                std::cout<<"fillet none , no fillet type given for id:"<<s1->id<<std::endl;
                std::cout<<""<<std::endl;
            }
        }

        return -1;
    }

    if(s1->fillet_type==FILLET_TYPE_BSPLINE){

        std::vector<PmCartesian> controlPoints = {
            { s1->start.tran.x, s1->start.tran.y, s1->start.tran.z },
            { s1->start_control.x, s1->start_control.y, s1->start_control.z },
            { s1->end_control.x, s1->end_control.y, s1->end_control.z },
            { s1->end.tran.x, s1->end.tran.y, s1->end.tran.z }
        };

        PmCartesian p0,p1,p2;
        double kappa;
        double kmax;
        double progress;
        int res;

        // Find curvature near the bspline start.
        progress=0.00;
        res=interpolateBSpline(controlPoints,progress,p0);
        progress=0.05;
        res=interpolateBSpline(controlPoints,progress,p1);
        progress=0.10;
        res=interpolateBSpline(controlPoints,progress,p2);
        kappa=bSplineCurvature(p0,p1,p2);
        kmax=std::fmax(kmax,kappa);

        // Find curvature near the bspline mid.
        progress=0.45;
        res=interpolateBSpline(controlPoints,progress,p0);
        progress=0.50;
        res=interpolateBSpline(controlPoints,progress,p1);
        progress=0.55;
        res=interpolateBSpline(controlPoints,progress,p2);
        kappa=bSplineCurvature(p0,p1,p2);
        kmax=std::fmax(kmax,kappa);

        // Find curvature near the bspline end.
        progress=0.90;
        res=interpolateBSpline(controlPoints,progress,p0);
        progress=0.95;
        res=interpolateBSpline(controlPoints,progress,p1);
        progress=1.00;
        res=interpolateBSpline(controlPoints,progress,p2);
        kappa=bSplineCurvature(p0,p1,p2);
        kmax=std::fmax(kmax,kappa);

        //s1->length_bruto=bSplineLength(controlPoints,20);
        //s1->length_netto=s1->length_bruto;
        s1->kmax=kmax;

        std::vector<PmCartesian> pv;
        PmCartesian pi;

        // Number of sample points you want
        int numPoints = PVEC_SIZE;

        // Calculate points along the B-spline
        for (int i = 0; i < numPoints; i++) {
            // Calculate progress in the range [0, 1] based on the index i
            double progress = static_cast<double>(i) / (numPoints - 1);  // Progress from 0 to 1

            // Interpolate the B-spline at the current progress and store the result in pi
            interpolateBSpline(controlPoints, progress, pi);

            // Add the interpolated point to the vector
            pv.push_back(pi);

            // std::cout<<"progress:"<<progress<<" px:"<<pi.x<<" y:"<<pi.y<<" z:"<<pi.z<<std::endl;
        }

        if (numPoints <= PVEC_SIZE) {
            // Copy elements from vector to array
            for (int i = 0; i < numPoints; i++) {
                s1->pvec[i] = pv[i];  // Assign each element from vector to array
            }
        } else {
            // Handle the case where there are more than 20 points
            std::cerr << "Error: Vector size exceeds array size!" << std::endl;
        }

        // Initialize the first point
        PmCartesian pii = s1->pvec[0];  // First point in the array

        // Calculate total length of the segment by summing distances between consecutive points
        double totalLength = 0.0;
        for (int i = 1; i < PVEC_SIZE; i++) {
            PmCartesian p1 = s1->pvec[i];
            totalLength += distance(pii, p1);
            pii = p1;  // Move to the next point
        }
        s1->length_xyz=totalLength;
        s1->length_netto=s1->length_xyz;

        pv.clear();

        if (debug) {
            if (debug) {
                std::cout<<""<<std::endl;
                std::cout<<"fillet bspline id:"<<s1->id<<std::endl;
                // printPoint(s1->start_trim,"start");
                // printPoint(s1->end_trim,"end");
                std::cout<<"length netto:"<<s1->length_netto<<std::endl;
                std::cout<<"kmax:"<<s1->kmax<<std::endl;
                std::cout<<""<<std::endl;
            }
        }

        return 0;
    }

    if(s1->fillet_type==FILLET_TYPE_LINE){
        s1->length_xyz=distance(s1->start.tran,s1->end.tran);
        s1->length_netto=s1->length_xyz;
        s1->kmax=0; // Straight line has no curvature.

        if (debug) {
            if (debug) {
                std::cout<<""<<std::endl;
                std::cout<<"fillet line id:"<<s1->id<<std::endl;
                // printPoint(s1->start_trim,"start");
                // printPoint(s1->end_trim,"end");
                std::cout<<"length netto:"<<s1->length_netto<<std::endl;
                std::cout<<"kmax:"<<s1->kmax<<std::endl;
                std::cout<<""<<std::endl;
            }
        }

        return 0;
    }

    return -1;
}

// Function to compute B-spline from a serie off controlpoints. Progress = 0 to 1.
int filletizer::interpolateBSpline(const std::vector<PmCartesian>& controlPoints,
                                   const double &progress,
                                   PmCartesian& pi) {

    // Create a TinySpline BSpline object with the appropriate number of control points
    int degree=3;
    tinyspline::BSpline spline(controlPoints.size(), 3, degree);

    // Setup control points
    std::vector<tinyspline::real> ctrlp;
    for (const auto& point : controlPoints) {
        ctrlp.push_back(point.x); // X coordinate
        ctrlp.push_back(point.y); // Y coordinate
        ctrlp.push_back(point.z); // Z coordinate
    }
    spline.setControlPoints(ctrlp);

    std::vector<tinyspline::real> result = spline.eval(progress).result(); // Evaluate the spline at u
    pi.x=result[0];
    pi.y=result[1];
    pi.z=result[2];

    return 0;
}

int filletizer::interpolatePvec(const struct emcmot_segment *seg,
                                const double &progress,
                                struct EmcPose *pos) {
    // Initialize the first point
    PmCartesian p0 = seg->pvec[0];  // First point in the array

    // Calculate total length of the segment by summing distances between consecutive points
    double totalLength = 0.0;
    for (int i = 1; i < PVEC_SIZE; i++) {
        PmCartesian p1 = seg->pvec[i];
        totalLength += distance(p0, p1);
        p0 = p1;  // Move to the next point
    }

    // Calculate the target length based on progress (from 0 to totalLength)
    double targetLength = progress * totalLength;

    // Interpolate the position based on the progress
    double accumulatedLength = 0.0;
    p0 = seg->pvec[0];  // Start with the first point again

    for (int i = 1; i < PVEC_SIZE; i++) {
        PmCartesian p1 = seg->pvec[i];

        double segmentLength = distance(p0, p1);
        accumulatedLength += segmentLength;

        // If the target length is within this segment, interpolate between p0 and p1
        if (accumulatedLength >= targetLength) {
            double remainingLength = targetLength - (accumulatedLength - segmentLength);
            double ratio = remainingLength / segmentLength;

            // Interpolate position along this segment
            pos->tran.x = p0.x + ratio * (p1.x - p0.x);
            pos->tran.y = p0.y + ratio * (p1.y - p0.y);
            pos->tran.z = p0.z + ratio * (p1.z - p0.z);

           // interpolate_abc_uvw(seg,progress,pos);

            return 0;  // Successfully interpolated the position
        }

        // Move to the next segment
        p0 = p1;
    }

    return -1;  // If we couldn't interpolate (in case something goes wrong)
}

//void filletizer::interpolate_abc_uvw(const struct emcmot_segment& seg,
//                                     const double& progress,
//                                     struct EmcPose& pos){

//    if(seg.canon_motion_type!=10){
//        pos.a = seg.a_start + (seg.a_end - seg.a_start) * progress;
//        pos.b = seg.b_start + (seg.b_end - seg.b_start) * progress;
//        pos.c = seg.c_start + (seg.c_end - seg.c_start) * progress;

//        pos.u = seg.u_start + (seg.u_end - seg.u_start) * progress;
//        pos.v = seg.v_start + (seg.v_end - seg.v_start) * progress;
//        pos.w = seg.w_start + (seg.w_end - seg.w_start) * progress;
//    } else {
//        // The fillet interpolation is 2 stage. 2 Times 0-100% sequence.
//        // First sequent 0-50% is in fact 0-100%.
//        // First 0-50% is from start to peak.
//        // Second 50_100% is from peak to end.

//        if(progress<0.5){
//            // Using coordinates that match the fillets progress.
//            // First stage start's ot 0% up to 100%
//            pos.a = seg.a_start + (seg.a_peak - seg.a_start) * (progress*2);
//            pos.b = seg.b_start + (seg.b_peak - seg.b_start) * (progress*2);
//            pos.c = seg.c_start + (seg.c_peak - seg.c_start) * (progress*2);

//            pos.u = seg.u_start + (seg.u_peak - seg.u_start) * (progress*2);
//            pos.v = seg.v_start + (seg.v_peak - seg.v_start) * (progress*2);
//            pos.w = seg.w_start + (seg.w_peak - seg.w_start) * (progress*2);
//        } else {
//            // Using coordinates that match the fillets progress.
//            // Second stage start's ot 0% up to 100%
//            pos.a = seg.a_peak + (seg.a_end - seg.a_peak) * ((progress-0.5)*2);
//            pos.b = seg.b_peak + (seg.b_end - seg.b_peak) * ((progress-0.5)*2);
//            pos.c = seg.c_peak + (seg.c_end - seg.c_peak) * ((progress-0.5)*2);

//            pos.u = seg.u_peak + (seg.u_end - seg.u_peak) * ((progress-0.5)*2);
//            pos.v = seg.v_peak + (seg.v_end - seg.v_peak) * ((progress-0.5)*2);
//            pos.w = seg.w_peak + (seg.w_end - seg.w_peak) * ((progress-0.5)*2);
//        }
//    }
//}

// Function to compute B-spline from a serie off controlpoints. Progress = 0 to 1.
int filletizer::interpolateBSpline(const struct emcmot_segment *seg,
                                   const double &progress,
                                   struct EmcPose *pos){

    std::vector<PmCartesian> controlPoints = {
        { seg->start.tran.x, seg->start.tran.y, seg->start.tran.z },
        { seg->start_control.x, seg->start_control.y, seg->start_control.z },
        { seg->end_control.x, seg->end_control.y, seg->end_control.z },
        { seg->end.tran.x, seg->end.tran.y, seg->end.tran.z }
    };
    PmCartesian pi;
    interpolateBSpline(controlPoints, progress, pi);

    pos->tran.x=pi.x;
    pos->tran.y=pi.y;
    pos->tran.z=pi.z;

    // interpolate_abc_uvw(seg,progress,pos);

    return 0;
}

// Function to calculate curvature given three points
double filletizer::bSplineCurvature(const PmCartesian& p0, const PmCartesian& p1, const PmCartesian& p2){
    // Calculate lengths of sides
    double a = distance(p1, p2); // Length between p1 and p2
    double b = distance(p0, p2); // Length between p0 and p2
    double c = distance(p0, p1); // Length between p0 and p1

    // Calculate semi-perimeter
    double s = (a + b + c) / 2.0;

    // Calculate area using Heron's formula
    double A = std::sqrt(s * (s - a) * (s - b) * (s - c));

    // Check for collinearity (area == 0)
    if (A < std::numeric_limits<double>::epsilon()) {
        // Collinear points: a straight line has zero curvature.
        return 0;
    }

    // Calculate radius of the circumcircle
    double R = (a * b * c) / (4.0 * A);

    // Return curvature (1/R)
    return 1.0 / R;
}

// Function to calculate the approximate length of a B-spline
double filletizer::bSplineLength(const std::vector<PmCartesian>& controlPoints, int numSegments){

    double length = 0;
    double progress = 0;
    PmCartesian p0, p1;

    // Ensure numSegments is positive and non-zero
    if (numSegments <= 0) {
        return 0;
    }

    // First point (t=0)
    interpolateBSpline(controlPoints, 0.0, p0);

    // Subdivide the spline into small segments and accumulate distances
    for (int i = 1; i <= numSegments; i++) {

        progress = static_cast<double>(i) / numSegments;  // Progress between 0 and 1

        // Interpolate and get the point at the current progress
        interpolateBSpline(controlPoints, progress, p1);

        // Calculate the distance between consecutive points and add it to the total length
        length += distance(p0, p1);

        // Move to the next segment (current point becomes previous point)
        p0 = p1;

        // Optionally print progress for debugging
        // std::cout << "Progress: " << progress << ", Length so far: " << length << std::endl;
    }

    return length;
}


int filletizer::bSplineTest() {
    // Define control points that the B-spline should pass through
    std::vector<PmCartesian> controlPoints = {
        {0.0, 0.0, 0.0}, // Start point
        {0.0, 100.0, 0.0}, // Waypoint 1
        {100.0, 100.0, 0.0}, // Waypoint 2
        {100.0, 0.0, 0.0}  // End point
    };

    int count = 10;  // Number of interpolation points
    for (int i = 0; i <= count; i++) {
        double progress = static_cast<double>(i) / count; // Normalize progress
        // Compute B-spline curve points
        PmCartesian pos;
        interpolateBSpline(controlPoints, progress, pos);
        std::cout << "Point: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
    }

    return 0;
}

int filletizer::isEqual_EmcPose_xyz(const struct EmcPose &p0, const struct EmcPose &p1) {
    PmCartesian xyz0, abc0, uvw0;
    emcPoseToPmCartesian(&p0, &xyz0, &abc0, &uvw0);  // Convert p0 to Cartesian

    PmCartesian xyz1, abc1, uvw1;
    emcPoseToPmCartesian(&p1, &xyz1, &abc1, &uvw1);  // Convert p1 to Cartesian

    // Compare Cartesian coordinates and return 1 for equal, 0 for not equal
    return (pmCartCartCompare(&xyz0, &xyz1) == 0) ? 1 : 0;
}

void filletizer::print_short(const emcmot_segment &seg){

    std::cout << " "<< std::endl;
    std::cout << "Id: " << seg.id << std::endl; // Access using dot operator

    // Output motion type using switch for better readability
    switch (seg.canon_motion_type) { // Access using dot operator
    case 1:
        std::cout << " Motion rapid" << std::endl;
        break;
    case 2:
        std::cout << " Motion Linear." << std::endl;
        break;
    case 3:
        std::cout << " Motion Arc." << std::endl;
        break;
    case 10:
        std::cout << " Motion Fillet." << std::endl;
        break;
    default:
        std::cout << " Motion Unknown." << std::endl;
        break;
    }

    // Motion is a fillet, add fillet type to debug info.
    if(seg.canon_motion_type==10){
        // Output motion type using switch for better readability
        switch (seg.fillet_type) { // Access using dot operator
        case FILLET_TYPE_NONE:
            std::cout << " Fillet None" << std::endl;
            break;
        case FILLET_TYPE_BSPLINE:
            std::cout << " Fillet Bspline" << std::endl;
            break;
        case FILLET_TYPE_LINE:
            std::cout << " Fillet Line." << std::endl;
            break;
        default:
            std::cout << " Fillet Unknown." << std::endl;
            break;
        }
    }

    // printPoint(seg.start_trim,"start");
    // printPoint(seg.end_trim,"end");
    std::cout<<"a start:"<<seg.a_start<<std::endl;
    std::cout<<"a end:"<<seg.a_end<<std::endl;
    std::cout<<"length netto:"<<seg.length_netto<<std::endl;
    std::cout<<"kmax:"<<seg.kmax<<std::endl;
    std::cout<<"g64p:"<<seg.tag.fields_float[3]<<std::endl;

    std::cout<<" "<<std::endl;
}

void filletizer::print(const emcmot_segment &seg){

    std::cout << " "<< std::endl;
    std::cout << "G-code line nr: " << seg.id << std::endl;  // Access using dot operator

    // Output motion type using switch for better readability
    switch (seg.canon_motion_type) { // Access using dot operator
    case 1:
        std::cout << "Motion type: Traverse, rapid." << std::endl;
        break;
    case 2:
        std::cout << "Motion type: Linear feed." << std::endl;
        break;
    case 3:
        std::cout << "Motion type: Arc feed." << std::endl;
        break;
    case 10:
        std::cout << "Motion type: Spline fillet." << std::endl;
        break;
    default:
        std::cout << "Unknown motion type." << std::endl;
        break;
    }
    std::cout << " "<< std::endl;

    // Print related start points.
    printPoint(seg.start.tran,"start");
    printPoint(seg.start_control,"start control");
    //printPoint(seg.start_trim,"start trim");
    // Print related end points.
    //printPoint(seg.end_trim,"end_trim");
    printPoint(seg.end_control,"end_control");
    printPoint(seg.end.tran,"end");

}

void filletizer::printPoint(const PmCartesian& p0, std::string intro){
    std::cout << intro << std::fixed << std::setprecision(3) << " x:"<< p0.x << " y: " << p0.y << " z: " << p0.z << std::endl;
}

int filletizer::isColinear(const PmCartesian& p0, const PmCartesian& p1, const PmCartesian& p2){

    double l0=distance(p0,p1);
    double l1=distance(p1,p2);
    double l2=distance(p0,p2);

    // Check if the sum of d01_squared and d12_squared is approximately equal to d02_squared
    const double tolerance = 1e-6; // Tolerance for floating-point comparison

    if(l0+l1>l2-1e-6 && l0+l1<l2+1e-6){
        return 1;
    }

    return 0;
}

int filletizer::planePoints(const struct emcmot_segment& seg,
                            const struct emcmot_segment& seg_next,
                            PmCartesian& p0,
                            PmCartesian& p1,
                            PmCartesian& p2) {

    // Assign points for the first segment
    if (seg.canon_motion_type == 1 || seg.canon_motion_type == 2 || seg.canon_motion_type == 3) {
        p0 = seg.start.tran; // Start point
        p1 = seg.end.tran;   // End point
    }

    // Assign point for the next segment
    if (seg_next.canon_motion_type == 1 || seg_next.canon_motion_type == 2 || seg_next.canon_motion_type == 3) {
        p2 = seg_next.end.tran; // Next segment's end point
    }

    // Check for collinearity
    if (isColinear(p0, p1, p2)) {
        std::cout << "Plane is not valid, points are collinear, abort." << std::endl;
        return -1;
    }

    return 0;
}

int filletizer::pointOnPlane(const PmCartesian& p0,
                             const PmCartesian& p1,
                             const PmCartesian& p2,
                             const PmCartesian& pi) {

    // Convert PmCartesian to Eigen vectors
    Eigen::Vector3d v0(p0.x, p0.y, p0.z);
    Eigen::Vector3d v1(p1.x, p1.y, p1.z);
    Eigen::Vector3d v2(p2.x, p2.y, p2.z);
    Eigen::Vector3d vi(pi.x, pi.y, pi.z);

    // Calculate vectors for the plane
    Eigen::Vector3d v01 = v1 - v0;
    Eigen::Vector3d v02 = v2 - v0;

    // Calculate the normal to the plane (cross product of v01 and v02)
    Eigen::Vector3d normal = v01.cross(v02);

    // Check if the point pi lies on the plane by checking the dot product
    // The vector from p0 to pi
    Eigen::Vector3d v0i = vi - v0;

    // Dot product of the normal vector and v0i should be close to 0 for pi to be on the plane
    double dotProduct = normal.dot(v0i);
    double tolerance = 1e-3; // Define a tolerance for floating-point comparison

    if (std::fabs(dotProduct) <= tolerance) {
        return 0; // pi is on the plane
    } else {
        return -1; // pi is not on the plane
    }
}

double filletizer::arcStartAngle(const PmCartesian& p0, const PmCartesian& pc) {
    // Calculate the angle of the direction vector from the center to the start point
    return atan2(p0.y - pc.y, p0.x - pc.x);
}

double filletizer::arcEndAngle(const PmCartesian& p1, const PmCartesian& pc) {
    // Calculate the angle of the direction vector from the center to the end point
    return atan2(p1.y - pc.y, p1.x - pc.x);
}

// Function to align the plane with the XY plane
int filletizer::AlignPlaneToXYPlane(const Eigen::Vector3d& p0,
                                    const Eigen::Vector3d& p1,
                                    const Eigen::Vector3d& p2,
                                    Eigen::Matrix3d &rotationMatrix){
    // Calculate vectors
    Eigen::Vector3d v1 = p1 - p0;
    Eigen::Vector3d v2 = p2 - p0;

    // Calculate the normal vector
    Eigen::Vector3d normal = v1.cross(v2);

    if (normal.norm() < 1e-6) {
        // std::cout<<"The points are collinear or too close together. Plane not valid. Revert to bspline."<<std::endl;
        return -1;
    }

    // Normalize the normal vector
    normal.normalize();

    // Calculate the angle to align with the Z-axis
    Eigen::Vector3d zAxis(0, 0, 1);
    double angle = std::acos(normal.dot(zAxis));

    // Calculate the rotation axis
    Eigen::Vector3d rotationAxis = normal.cross(zAxis).normalized();

    // Create a rotation matrix
    Eigen::AngleAxisd rotation(angle, rotationAxis);
    rotationMatrix = rotation.toRotationMatrix();

    return 0;
}

// Function to convert PmCartesian to Eigen::Vector3d
Eigen::Vector3d filletizer::toEigen(const PmCartesian& p) {
    return Eigen::Vector3d(p.x, p.y, p.z);
}

// Function to convert Eigen::Vector3d to PmCartesian
PmCartesian filletizer::toCartesian(const Eigen::Vector3d& v) {
    return {v.x(), v.y(), v.z()};
}

// Creates a new filletizer instance and returns its pointer
extern "C" struct filletizer* filletizer_init_ptr() {
    return new filletizer();
}

extern "C" void filletizer_remove_ptr(struct filletizer* filletizer_ptr){
    delete filletizer_ptr;
}

// Applies fillet transformations on segments in the filletizer
extern "C" int filletizer_filletize(struct filletizer* filletizer_ptr,
                                    struct emcmot_segment* s0,
                                    struct emcmot_segment* s1,
                                    struct emcmot_segment* s2,
                                    enum fillet_types fillet_type,
                                    int debug) {
    return filletizer_ptr->filletize(s0, s1, s2, fillet_type, debug);
}

extern "C" int filletizer_interpolate_segment(struct filletizer* filletizer_ptr,
                                              struct emcmot_segment* seg,
                                              const double progress,
                                              struct EmcPose *pos) {

    return filletizer_ptr->interpolate_segment(seg,progress,pos);
}
