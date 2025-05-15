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
#ifndef EQ44_H
#define EQ44_H

#include "eq14.h"
#include "eq17.h"
#include "eq18.h"
#include "eq19.h"
#include "eq20.h"
#include "eq21.h"
#include "eq22.h"
#include "eq23.h"
#include "eq45.h"
#include "segment.h"

/* eq44, page 7. Four-clothoid
 *
 * First step, calculate G3 continuity.
 *
 * Ci,0​ is the initial sharpness.
 * Ci,4​ is the final sharpness.
 * Yi,h​ represents coefficients affecting the transition.
 * Sh​ is a scaling factor for each term in the sum.
 * Ci,0 ​+ (Yi,1​⋅S1​)+(Yi,2​⋅S2​)+(Yi,3​⋅S3​)+(Yi,4​⋅S4​) = Ci,4
 *
 */
static void eq44_G123(const double thetai0, const double thetai4,
               const double kappai0, const double kappai4,
               const double sharpnessi0, const double sharpnessi4,
               double yi1, // Initial guess value, unknown var.
               double s1,  // Initial guess value, unknown var.

               double *sharpnessi1,  double *sharpnessi2, double *sharpnessi3,
               double *kappai1, double *kappai2, double *kappai3,
               double *thetai1, double *thetai2, double *thetai3,
               double *yi2, double *yi3, double *yi4
               ){

    double sum=0;
    double s2=0, s3=0, s4=0;

    s4 = s3 = s2 = s1; // Paper states identical length's.

    // eq45, page 7. Given the guess value yi1, calculate yi2, yi3, yi4.
    eq45(thetai0, thetai4, kappai0, kappai4, sharpnessi0, sharpnessi4, s1, yi1, yi2, yi3, yi4);

    // Eq 21, page 4. Calculate the sharpnes c at each spline junction.
    // G3 continuity:
    double sharpnessi4_check;
    eq21(yi1,*yi2,*yi3,*yi4,
         s1,s2,s3,s4,
         sharpnessi0,sharpnessi1,sharpnessi2,sharpnessi3,&sharpnessi4_check);

    // eq44, page 7, G3 Continuity for loop h1 to h4.
    // Check sharpness c.
    sum+= yi1    * s1;
    sum+= (*yi2) * s2;
    sum+= (*yi3) * s3;
    sum+= (*yi4) * s4;
    if((sharpnessi0 + sum) - sharpnessi4 > 1e-6){
        // printf("error: eq44 sharpness c. \n");
    }
    sum=0;

    // Eq 22, page 4. Calculate the kappa k at each spline junction.
    // G2 Continuity:
    double kappai4_check;
    eq22(yi1,*yi2,*yi3,*yi4,
         s1,s2,s3,s4,
         sharpnessi0,*sharpnessi1,*sharpnessi2,*sharpnessi3,
         kappai0,kappai1,kappai2,kappai3,&kappai4_check);

    // eq44, page 7, G2 Continuity for loop h1 to h4.
    // Check kappa k.
    sum+= sharpnessi0    * s1 + 0.5 * yi1    * (s1*s1);
    sum+= (*sharpnessi1) * s2 + 0.5 * (*yi2) * (s2*s2);
    sum+= (*sharpnessi2) * s3 + 0.5 * (*yi3) * (s3*s3);
    sum+= (*sharpnessi3) * s4 + 0.5 * (*yi4) * (s4*s4);
    if((kappai0 + sum) - kappai4 > 1e-6){
        // printf("error: eq44 kappa k. \n");
    }
    sum=0;

    // Eq 23, page 4. Calculate the theta θ at each spline junction.
    // G1 Continuity:
    double thetai4_check;
    eq23(yi1,*yi2,*yi3,*yi4,
         s1,s2,s3,s4,
         sharpnessi0,*sharpnessi1,*sharpnessi2,*sharpnessi3,
         kappai0,*kappai1,*kappai2,*kappai3,
         thetai0,thetai1,thetai2,thetai3,&thetai4_check);

    sum+= kappai0    * s1 + 0.5 * sharpnessi0    * (s1*s1) + (1.0/6.0) * yi1    * ((s1*s1)*(s1*s1)*(s1*s1));
    sum+= (*kappai1) * s2 + 0.5 * (*sharpnessi1) * (s2*s2) + (1.0/6.0) * (*yi2) * ((s2*s2)*(s2*s2)*(s2*s2));
    sum+= (*kappai2) * s3 + 0.5 * (*sharpnessi2) * (s3*s3) + (1.0/6.0) * (*yi3) * ((s3*s3)*(s3*s3)*(s3*s3));
    sum+= (*kappai3) * s4 + 0.5 * (*sharpnessi3) * (s4*s4) + (1.0/6.0) * (*yi4) * ((s4*s4)*(s4*s4)*(s4*s4));
    if((thetai0 + sum) - thetai4 > 1e-6){
        // printf("error: eq44 theta θ. \n");
    }
}

/* Helper function to pass in struct for both z and xy components.
 *
 */
static void eq44_G123_compact(struct segment *data){

    // Z.
    eq44_G123(data->theta10, data->theta14,
              data->kappa10, data->kappa14,
              data->sharpness10, data->sharpness14,
              data->y11, // Initial guess value, unknown var.
              data->s1,  // Initial guess value, unknown var.

              // Calulated values:
              &data->sharpness11,  &data->sharpness12, &data->sharpness13,
              &data->kappa11, &data->kappa12, &data->kappa13,
              &data->theta11, &data->theta12, &data->theta13,
              &data->y12, &data->y13, &data->y14);

    // Xy.
    eq44_G123(data->theta20, data->theta24,
              data->kappa20, data->kappa24,
              data->sharpness20, data->sharpness24,
              data->y21, // Initial guess value, unknown var.
              data->s1,  // Initial guess value, unknown var.

              // Calulated values:
              &data->sharpness21,  &data->sharpness22, &data->sharpness23,
              &data->kappa21, &data->kappa22, &data->kappa23,
              &data->theta21, &data->theta22, &data->theta23,
              &data->y22, &data->y23, &data->y24);

}

#endif // EQ44_H
















