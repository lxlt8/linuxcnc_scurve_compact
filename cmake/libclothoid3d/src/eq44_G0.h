#ifndef EQ44_G0_H
#define EQ44_G0_H

#include "segment.h"
// #include "solver/nelder_mead.h"
#include "solver/ceres_capi.h"
#include "solver/ceres_cppapi.h"

/* eq44, page 7. G0 Continuity.
 *
 * For clothoid interpolation we use eq14_gauss.
 * This is very accurate inperpolation using gauss legendre.
 *
 * In this section we solve the unknown vars y11, y21, s1 using dogleg method.
 *
 *  1.  Calculate clothoid constraints using guess values for y11, y21, s1.
 *  2.  Calculate clothoid endpoint p1.
 *  3.  Compare p1 with xe,ye,ze endpoint p2. Calclate distance p1 -> p2.
 *  4.  Use dogleg method to try out new value's for y11, y21, s1 to
 *      minimize the distance p1 -> p2. Optimal is p1 -> p2 < 1e-6.
 *  5.  Return optimal fit values y11, y21, s1.
 *
 *  We use eq44_G123 to calculate the clothoid transition constraints for clothoid 1,2,3,4.
 *  So that start angle, end angle of the compound and all inbetween angles
 *  are aligned correctly. This the same for kappa, sharpness.
 *
 */
static void eq44_G0(struct segment *data){

    /*
    if(data->solver_enum==NELDER_MEAD){
        init_nelder_mead(
                    data->theta10, data->theta14, data->kappa10, data->kappa14, data->sharpness10, data->sharpness14,
                    data->theta20, data->theta24, data->kappa20, data->kappa24, data->sharpness20, data->sharpness24,
                    data->p0[0], data->p0[1], data->p0[2], data->p1[0], data->p1[1], data->p1[2],
                    &data->y11, &data->y21, &data->s1
                    );
    }*/

    if(data->solver_enum==CERES_CAPI_JACOBIAN){
        init_ceres_capi(
                    data->theta10, data->theta14, data->kappa10, data->kappa14, data->sharpness10, data->sharpness14,
                    data->theta20, data->theta24, data->kappa20, data->kappa24, data->sharpness20, data->sharpness24,
                    data->p0[0], data->p0[1], data->p0[2], data->p1[0], data->p1[1], data->p1[2],
                    &data->y11, &data->y21, &data->s1
                    );
    }

    if(data->solver_enum==CERES_CENTRAL_DIFF){
        int ceres_solver_type = 0;
        init_ceres_cpp(
                    data->theta10, data->theta14, data->kappa10, data->kappa14, data->sharpness10, data->sharpness14,
                    data->theta20, data->theta24, data->kappa20, data->kappa24, data->sharpness20, data->sharpness24,
                    data->p0[0], data->p0[1], data->p0[2], data->p1[0], data->p1[1], data->p1[2],
                    &data->y11, &data->y21, &data->s1, ceres_solver_type, data->s1_lower_limit, data->s1_upper_limit
                    );
    }

    if(data->solver_enum==CERES_FORWARD_DIFF){
        int ceres_solver_type = 1;
        init_ceres_cpp(
                    data->theta10, data->theta14, data->kappa10, data->kappa14, data->sharpness10, data->sharpness14,
                    data->theta20, data->theta24, data->kappa20, data->kappa24, data->sharpness20, data->sharpness24,
                    data->p0[0], data->p0[1], data->p0[2], data->p1[0], data->p1[1], data->p1[2],
                    &data->y11, &data->y21, &data->s1, ceres_solver_type, data->s1_lower_limit, data->s1_upper_limit
                    );
    }

    if(data->solver_enum==CERES_RIDDERS_DIFF){
        int ceres_solver_type = 2;
        init_ceres_cpp(
                    data->theta10, data->theta14, data->kappa10, data->kappa14, data->sharpness10, data->sharpness14,
                    data->theta20, data->theta24, data->kappa20, data->kappa24, data->sharpness20, data->sharpness24,
                    data->p0[0], data->p0[1], data->p0[2], data->p1[0], data->p1[1], data->p1[2],
                    &data->y11, &data->y21, &data->s1, ceres_solver_type, data->s1_lower_limit, data->s1_upper_limit
                    );
    }

    if(data->s1<0){
        // printf("clothoid length solved negative. \n");
    }

    data->s1 = fabs(data->s1);
    // printf("s1 eq44_G0 %f \n",data->s1);

    // Total clothoid length is the sum off all individual attached clothoids.
    data->length = data->s1*4;
}

/* Calculate inbetween points at the clothoid junctions.
 *
 */
static void eq44_intermediate_points(struct segment *data){

    double l1,l2,l3,l4;
    eq14_gauss(data->s1, data->p0[0], data->p0[1], data->p0[2],
               data->theta10, data->theta20,
               data->kappa10, data->kappa20,
               data->sharpness10, data->sharpness20,
               data->y11, data->y21,
               &data->x1, &data->y1, &data->z1,
               &l1);

    eq14_gauss(data->s1, data->x1, data->y1, data->z1,
               data->theta11, data->theta21,
               data->kappa11, data->kappa21,
               data->sharpness11, data->sharpness21,
               data->y12, data->y22,
               &data->x2, &data->y2, &data->z2,
               &l2);

    eq14_gauss(data->s1, data->x2, data->y2, data->z2,
               data->theta12, data->theta22,
               data->kappa12, data->kappa22,
               data->sharpness12, data->sharpness22,
               data->y13, data->y23,
               &data->x3, &data->y3, &data->z3,
               &l3);

    eq14_gauss(data->s1, data->x3, data->y3, data->z3,
               data->theta13, data->theta23,
               data->kappa13, data->kappa23,
               data->sharpness13, data->sharpness23,
               data->y14, data->y24,
               &data->p1[0], &data->p1[1], &data->p1[2],
               &l4);

}

#endif // EQ44_G0_H






















