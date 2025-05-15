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
#ifndef EQ35_H
#define EQ35_H

#include "eq33.h" // Transformation matrix.

// Copy 3 values from source to target.
static void copy_3(double src[3], double tar[3]){
    tar[0]=src[0];
    tar[1]=src[1];
    tar[2]=src[2];
}

// Print function.
static void print_name_value_3(const char *name, double v[3]) {
    printf("%s: [%.6f, %.6f, %.6f]\n", name, v[0], v[1], v[2]);
}

/* Do a forward transformation for 2 segments. Page 7.
 *
 * pa0  = start point first segment.
 * pae0 = end point first segment.
 * pc0  = center first segment.
 * na0  = normal first segment.
 *
 * pa1  = start point second segment.
 * pae1 = end point second segment.
 * pc1  = center second segment.
 * na1  = normal second segment.
 *
 * rev  = flag for inverse transformation.
 */
static void eq35(  int rev,
            double pa0[3], double pae0[3], double pc0[3], double na0[3],
            double pa1[3], double pae1[3], double pc1[3], double na1[3],
            // Transformed values are identified by _
            double pa0_[3], double pae0_[3], double pc0_[3], double na0_[3],
            double pa1_[3], double pae1_[3], double pc1_[3], double na1_[3]){

    // Make a copy of the original values, keep the original values as is.
    copy_3(pa0,pa0_);
    copy_3(pae0,pae0_);
    copy_3(pc0,pc0_);
    copy_3(na0,na0_);

    copy_3(pa1,pa1_);
    copy_3(pae1,pae1_);
    copy_3(pc1,pc1_);
    copy_3(na1,na1_);

    // Transformation matrix.
    double T[3][3];

    // Init matrix values.
    eq33(pa0_,pae0_,pc0_,na0_, pa1_,pae1_,pc1_,na1_, T);

    if(rev){ // Inverse transformation.
        inv_trans(pa0_, T);
        inv_trans(pae0_, T);
        inv_trans(pc0_, T);
        inv_trans(na0_, T);

        inv_trans(pa1_, T);
        inv_trans(pae1_, T);
        inv_trans(pc1_, T);
        inv_trans(na1_, T);
    } else { // Formward transformation.
        fwd_trans(pa0_, T);
        fwd_trans(pae0_, T);
        fwd_trans(pc0_, T);
        fwd_trans(na0_, T);

        fwd_trans(pa1_, T);
        fwd_trans(pae1_, T);
        fwd_trans(pc1_, T);
        fwd_trans(na1_, T);
    }
}

// Proof function.
// After transformation, calculate all derivates in transformed position.
static void eq35_proof(){

    // Segment 1.
    double pa0[3]={0,0,0};
    double pae0[3]={100,0,0};
    double pc0[3]={50,0,0};
    double na0[3]={0,0,1};

    // Segment 2.
    double pa1[3]={100,0,0};
    double pae1[3]={200,0,0};
    double pc1[3]={150,0,0};
    double na1[3]={0,0,-1};

    // Flag for inverse transformation.
    int rev = 0;

    // Storage for copy values.
    double pa0_[3];
    double pae0_[3];
    double pc0_[3];
    double na0_[3];
    double pa1_[3];
    double pae1_[3];
    double pc1_[3];
    double na1_[3];

    // Transformation of 2 segments.
    eq35(rev, pa0, pae0, pc0, na0, pa1, pae1, pc1, na1, pa0_, pae0_ ,pc0_ ,na0_ ,pa1_ ,pae1_ ,pc1_ ,na1_ );

    print_name_value_3("pa0_", pa0_);   // Segment 1 start point.
    print_name_value_3("pae0_", pae0_); // Segment 1 end point.
    print_name_value_3("pc0_", pc0_);
    print_name_value_3("na0_", na0_);
    print_name_value_3("pa1_", pa1_);
    print_name_value_3("pae1_", pae1_);
    print_name_value_3("pc1_", pc1_);
    print_name_value_3("na1_", na1_);

    /* Terminal output for forward transformation:
        pa0_: [0.000000, 0.000000, 0.000000]
        pae0_: [0.000000, 0.000000, -100.000000]
        pc0_: [0.000000, 0.000000, -50.000000]
        na0_: [-1.000000, 0.000000, 0.000000]
        pa1_: [0.000000, 0.000000, -100.000000]
        pae1_: [0.000000, 0.000000, -200.000000]
        pc1_: [0.000000, 0.000000, -150.000000]
        na1_: [1.000000, 0.000000, 0.000000]
    */
}

#endif // EQ35_H














