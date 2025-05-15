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
#ifndef EQ32_H
#define EQ32_H

#include "eq30.h"

/* Eq32, page 5.
 *
 * Calculate the normal n to be used for the rotation matrix at page 6.
 *
 * v1 = tangent vector θ "theta" at the end of the first segment.
 * v2 = tangent vector θ "theta" at the start of the second segment.
 * n1 = normal vector of the first segment, has value if arc.
 * n2 = normal vector of the second segment, has value if arc.
 * n  = normal vector of the corner.
 *
 */
static void eq32(double v1[3], double v2[3], double n1[3], double n2[3], double n[3]){

    // Check if v1 = v2. Colinear situation.
    if(v1[0] == v1[0] && v1[1] == v1[1] && v1[2] == v1[2]){

        // Check n1 excists.
        if(n1[0]!=0 || n1[1]!=0 || n1[2]!=0){
            n[0]=n1[0];
            n[1]=n1[1];
            n[2]=n1[2];
            return;
        }

        // Check n2 excists.
        if(n1[0]!=0 || n1[1]!=0 || n1[2]!=0){
            n[0]=n2[0];
            n[1]=n2[1];
            n[2]=n2[2];
            return;
        }

        // Impossible situation.
        printf("eq32 exception!");

    } else { // Valid to do v1 x v2, as v1 differs from v2
        cross_product_normalized(v1,v2,n);
    }
}

#endif // EQ32_H
