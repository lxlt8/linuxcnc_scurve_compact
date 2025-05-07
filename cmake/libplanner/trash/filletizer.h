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
#ifndef FILLETIZER_H
#define FILLETIZER_H

enum fillet_types {
    FILLET_TYPE_NONE,
    FILLET_TYPE_BSPLINE,
    FILLET_TYPE_LINE
};

#ifdef __cplusplus

#include "segment.h"
#include "eigen3/Eigen/Dense"
#include "vector"

// Used for constructing bsplines.
// input = ratio 0-1 for trim distance.
// 1= controlpoint is at trim point.
// 0= controlpoint is at end point.
#define CONTROL_POINT_OFFSET 0.5d

// Gcode file output format.
#define PRINT_PRECISION 3
// Points used by writing a gcode file.
#define SPLINE_POINTS  20
#define CLOTHOID_POINTS  20

class filletizer
{
public:

    filletizer();

    // To initialise a curve during loading by
    // tpAddLine & tpAddcircle.
    // Calculates curve parameters like length_bruto, kmax, etc.
    int initCurve(emcmot_segment& seg, int debug);

    // s0=line or arc segment.
    // s1=empty fillet.
    // s3=line or arc segment.
    // The segments are carrying a state_tag field containing the trim values of G64 P..
    int filletize(emcmot_segment *s0,
                  emcmot_segment *s1,
                  emcmot_segment *s2,
                  const fillet_types& fillet_type,
                  int debug);

    // We only need previous and nect segment if we have a segment type clothoid.
    int interpolate_segment(const emcmot_segment *seg,
                            const double progress,
                            EmcPose *pos);


    double circleLength(struct emcmot_segment& seg);
    double lineLength(struct emcmot_segment& seg);

private:

    // Interpolate abc, uvw axis given progress.
    void interpolate_abc_uvw(const struct emcmot_segment *seg,
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
    double pmLine9Target_xyz(const emcmot_segment& seg);
    double pmLine9Target_abc(const emcmot_segment& seg);
    double pmLine9Target_uvw(const emcmot_segment& seg);

    // Declare a constant of type FILLET_TYPES.
    // const FILLET_TYPES CURRENT_FILLET_TYPE = FILLET_TYPES::CLOTHOID;

    void print_short(const emcmot_segment &seg);

    void print(const emcmot_segment &seg);

    void correct_trim_length(const emcmot_segment *seg,
                             double& max_trim);
    int trim_curve(emcmot_segment *seg,
                   const int &trim_front,
                   const int &trim_back,
                   double trim_length,
                   int debug); // Returns max allowable trim for this segment.
    int init_fillet_segment(const emcmot_segment *s0,   // Gcode line or arc.
                            emcmot_segment *s1,         // Fillet segment.
                            const emcmot_segment *s2,   // Gcode line or arc.
                            int debug);

    int trimLine(emcmot_segment *seg, double trim, const int &trim_front, const int &trim_back, bool debug);
    int isClosedCircle(const struct emcmot_segment *seg);
    int trimCircle(struct emcmot_segment *seg, const double trim, const int &trim_front, const int &trim_back, bool debug);

    int interpolateCircleByLength(const PmCartesian &p0,
                                  const PmCartesian &pc,
                                  const PmCartesian &pn,
                                  const PmCartesian &p1,
                                  int turn,
                                  const double& length,
                                  PmCartesian& pi);

    int interpolateBSpline(const std::vector<PmCartesian>& controlPoints,
                           const double& progress,
                           PmCartesian &pi);
    int interpolateBSpline(const emcmot_segment *seg,
                           const double& progress,
                           EmcPose *pos);
    int interpolatePvec(const emcmot_segment *seg,
                        const double& progress,
                        EmcPose *pos);
    double calculateBSplineLength(const std::vector<PmCartesian>& controlPoints);
    double bSplineLength(const std::vector<PmCartesian>& controlPoints, int numSegments);
    double bSplineCurvature(const PmCartesian& p0, const PmCartesian& p1, const PmCartesian& p2);
    int bSplineTest();
    double distance(const PmCartesian& p0, const PmCartesian& p1);
    int isEqual_EmcPose_xyz(const struct EmcPose& p0, const struct EmcPose& p1);
    void printPoint(const PmCartesian& p0, std::string intro="");
    int isColinear(const PmCartesian& p0, const PmCartesian& p1, const PmCartesian& p2);

    double arcStartAngle(const PmCartesian& p0, const PmCartesian& pc);
    double arcEndAngle(const PmCartesian& p1, const PmCartesian& pc);

    int AlignPlaneToXYPlane(const Eigen::Vector3d& p0,
                            const Eigen::Vector3d& p1,
                            const Eigen::Vector3d& p2,
                            Eigen::Matrix3d& rotationMatrix);

    int planePoints(const emcmot_segment &seg,
                    const emcmot_segment &seg_next,
                    PmCartesian& p0,
                    PmCartesian& p1,
                    PmCartesian& p2);
    int pointOnPlane(const PmCartesian &p0,
                     const PmCartesian &p1,
                     const PmCartesian &p2,
                     const PmCartesian &pi);
    Eigen::Vector3d toEigen(const PmCartesian& p);
    PmCartesian toCartesian(const Eigen::Vector3d& v);
    int segmentToXY(struct emcmot_segment& seg, const PmCartesian& p0, const PmCartesian& p1, const PmCartesian& p2);
    int segmentTo3d(struct emcmot_segment& seg, const PmCartesian& p0, const PmCartesian& p1, const PmCartesian& p2);

};

#else
typedef struct filletizer filletizer;
#endif

#endif // FILLETIZER_H



