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
#ifndef EQ19_H
#define EQ19_H

#include "segment.h"

/* eq19, page 4.
 *
 * stot = total clothoid length
 * s = global length at clothoid to retrieve sharpness c.
 *
 * Retrieve the kappa for s.
 *
 */
static void eq19(const double s, const double stot,
          const double sharpnessi0, const double sharpnessi1, const double sharpnessi2, const double sharpnessi3,
          const double kappai0, const double kappai1, const double kappai2, const double kappai3,
          const double yi1, const double yi2, const double yi3, const double yi4,
          double *kappa_at_s) {

    double s1 = stot / 4.0;
    double s2 = s1 + s1;
    double s3 = s1 + s1 + s1;
    double s4 = stot;

    if (s <= s1) {
        *kappa_at_s = kappai0 + sharpnessi0 * s + 0.5 * yi1 * (s * s);
    } else if (s > s1 && s <= s2) {
        *kappa_at_s = kappai1 + sharpnessi1 * (s - s1) + 0.5 * yi2 * ((s - s1) * (s - s1));
    } else if (s > s2 && s <= s3) {
        *kappa_at_s = kappai2 + sharpnessi2 * (s - s2) + 0.5 * yi3 * ((s - s2) * (s - s2));
    } else if (s > s3 && s <= s4) {
        *kappa_at_s = kappai3 + sharpnessi3 * (s - s3) + 0.5 * yi4 * ((s - s3) * (s - s3));
    } else {
        // Handle the case where s is out of bounds (optional)
        *kappa_at_s = 0.0; // or some other default value
        printf("error: eq19. \n");
    }
}

static void eq19_single_clothoid(const double s,
                          const double sharpnessi0,
                          const double kappai0,
                          const double yi1,
                          double *kappa_at_s) {
    *kappa_at_s = kappai0 + sharpnessi0 * s + 0.5 * yi1 * (s * s);
}

/* eq19, page 4, used to retrieve the curvature extrema, kappa max for the
 * clothoid compound.
 *
 */
static void eq19_curvature_extrema(const struct segment *seg, double *curvature_extrema) {

    if(seg->segment_type!=CLOTHOID){
        // printf("error, eq.19 segment type is wrong. \n");
        *curvature_extrema=0;
        return;
    }

    if(seg->length<1e-6){
        // printf("error, eq.19 segment has no length. \n");
        *curvature_extrema=0;
        return;
    }

    double kappa_z;
    double kappa_xy;

    double kappa_z_max=0;
    double kappa_xy_max=0;

    int segments = 20; // Increase resolution if desired.

    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);
        double s = progress * seg->length;

        // Retrieve kappa for z-component.
        eq19(s,seg->length,
            seg->sharpness10,seg->sharpness11,seg->sharpness12,seg->sharpness13,
            seg->kappa10,seg->kappa11,seg->kappa12,seg->kappa13,
            seg->y11,seg->y12,seg->y13,seg->y14, &kappa_z);

        kappa_z_max = fmax(kappa_z_max, fabs(kappa_z) );
        // printf("kappa_z: %f \n",kappa_z);

        // Retrieve kappa for xy-component.
        eq19(s,seg->length,
            seg->sharpness20,seg->sharpness21,seg->sharpness22,seg->sharpness23,
            seg->kappa20,seg->kappa21,seg->kappa22,seg->kappa23,
            seg->y21,seg->y22,seg->y23,seg->y14, &kappa_xy);

        kappa_xy_max = fmax(kappa_xy_max, fabs(kappa_xy) );
        // printf("kappa_xy: %f \n",kappa_xy);
    }

    // Compare z extrema with xy extrema and take highes value.
    *curvature_extrema=fmax(kappa_z_max,kappa_xy_max);
}

#endif // EQ19_H
