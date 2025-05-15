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
#ifndef CLOTHOID_2_NO_FIT_H
#define CLOTHOID_2_NO_FIT_H

#include "../eq14.h"
#include "../eq20.h"
#include "../eq29.h"
#include "../eq30.h"
#include "../eq31.h"
#include "../eq35.h"
#include "../eq44.h"

void clothoid_2_no_fit() {

    double degrees_xy_start = 0.0;
    double degrees_xy_end = 0.0;

    double degrees_z_start = 0.0;
    double degrees_z_end = 0.0;

    double ltot = 200;

    // Example for z component:
    // Initial values at spline start, end point for z.
    double theta10 = degrees_z_start * (M_PI / 180.0);
    double theta14 = degrees_z_end * (M_PI / 180.0);
    double kappa10 = 0;
    double kappa14 = 0;
    double sharpness10 = 0;
    double sharpness14 = 0;

    // Example for xy component:
    // Initial values at spline start, end point for xy.
    double theta20 = degrees_xy_start * (M_PI / 180.0);
    double theta24 = degrees_xy_end * (M_PI / 180.0);
    double kappa20 = 0;
    double kappa24 = 0;
    double sharpness20 = 0;
    double sharpness24 = 0;

    // Initial guess values.
    double y11 = 0.0;
    double y21 = 0.00015;
    double s1 = ltot/2;

    // To be updated for z.
    double sharpness11, sharpness12, sharpness13;
    double kappa11, kappa12, kappa13;
    double theta11, theta12, theta13;
    double y12, y13, y14;

    // To be updated for xy.
    double sharpness21, sharpness22, sharpness23;
    double kappa21, kappa22, kappa23;
    double theta21, theta22, theta23;
    double y22, y23, y24;

    // eq44, first step, solve G3 continuity for i=1.
    eq44_G123_2(theta10, theta14, kappa10, kappa14, sharpness10, sharpness14, y11, s1,
              // Updated values for:
              &sharpness11, &sharpness12, &sharpness13,
              &kappa11, &kappa12, &kappa13,
              &theta11, &theta12, &theta13,
              &y12, &y13, &y14
              );

    // eq44, second step, solve G3 continuity for i=2.
    eq44_G123_2(theta20, theta24, kappa20, kappa24, sharpness20, sharpness24, y21, s1,
              // Updated values for:
              &sharpness21, &sharpness22, &sharpness23,
              &kappa21, &kappa22, &kappa23,
              &theta21, &theta22, &theta23,
              &y22, &y23, &y24
              );

    printf("y11: %f \n", y11);
    printf("y12: %f \n", y12);
    printf("y13: %f \n", y13);
    printf("y14: %f \n", y14);

    printf("y21: %f \n", y21);
    printf("y22: %f \n", y22);
    printf("y23: %f \n", y23);
    printf("y24: %f \n", y24);

    printf("theta11: %f \n", theta11);
    printf("theta12: %f \n", theta12);
    printf("theta13: %f \n", theta13);
    printf("theta14: %f \n", theta14);

    printf("theta21: %f \n", theta21);
    printf("theta22: %f \n", theta22);
    printf("theta23: %f \n", theta23);
    printf("theta24: %f \n", theta24);

    double x0 = 0;
    double y0 = 0;
    double z0 = 0;

    double x1 = 0;
    double y1 = 0;
    double z1 = 0;

    double x2 = 0;
    double y2 = 0;
    double z2 = 0;

    double x3 = 0;
    double y3 = 0;
    double z3 = 0;

    double x4 = 0;
    double y4 = 0;
    double z4 = 0;

    double l1=0,l2=0,l3=0,l4=0;

    int segments = 100;

    // Print to file for visualization (if needed)
    FILE *data_file = fopen("clothoid_point.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    y11=0, y21=0.00001;

    double L0,Y0;
    kappa20=0;
    kappa21=0.01;
    theta20=0;
    theta21= 45* (M_PI / 180.0);

    testx(kappa20,kappa21,theta20,theta21,&s1,&Y0);

    printf("L0: %f \n",s1);
    printf("Y0: %f \n",Y0);

    // Clothoid 1. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double s = progress * 100;
        // eq14_drift(s, x0, y0, z0, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, y11, y21, &x1, &y1, &z1, &l1);
        eq14_gauss(s, x0, y0, z0, 0, theta20, 0, kappa20, 0, 0, 0, 0.0001, &x1, &y1, &z1, &l1);
        fprintf(data_file, "%lf %lf %lf\n", x1, y1, z1);
    }

//    // Clothoid 1. Trajectory.
//    for (int i = 0; i < segments; i++) {
//        double progress = (double)i / (segments - 1);  // Correct floating-point division
//        double s = progress * s1;
//        // eq14_drift(s, x0, y0, z0, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, y11, y21, &x1, &y1, &z1, &l1);
//        eq14_gauss(s1-s, x0, y0+50, z0, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, y11, -y21, &x1, &y1, &z1, &l1);
//        fprintf(data_file, "%lf %lf %lf\n", x1, y1, z1);
//    }

//    // Clothoid 3. Trajectory.
//    for (int i = 0; i < segments; i++) {
//        double progress = (double)i / (segments - 1);  // Correct floating-point division
//        double s = progress * s1;
//        // eq14_drift(s, x2, y2, z2, theta12, theta22, kappa12, kappa22, sharpness12, sharpness22, y13, y23, &x3, &y3, &z3, &l3);
//        eq14_gauss(s, x2, y2, z2, theta12, theta22, kappa12, kappa22, sharpness12, sharpness22, y13, y23, &x3, &y3, &z3, &l3);
//        fprintf(data_file, "%lf %lf %lf\n", x3, y3, z3);
//    }

//    // Clothoid 4. Trajectory.
//    for (int i = 0; i < segments; i++) {
//        double progress = (double)i / (segments - 1);  // Correct floating-point division
//        double s = progress * s1;
//        // eq14_drift(s, x3, y3, z3, theta13, theta23, kappa13, kappa23, sharpness13, sharpness23, y14, y24, &x4, &y4, &z4, &l4);
//        eq14_gauss(s, x1, y1, z1, theta13, theta23, kappa13, kappa23, sharpness13, sharpness23, y14, y24, &x4, &y4, &z4, &l4);
//        fprintf(data_file, "%lf %lf %lf\n", x4, y4, z4);
//    }



    printf("endpoint trajectory clothoid 4: \n");
    printf("x4: %f \n",x4);
    printf("y4: %f \n",y4);
    printf("z4: %f \n",z4);
    printf("ltot: %f \n",l1+l2+l3+l4);

    fclose(data_file);

    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up 3D plotting
        fprintf(gnuplot, "set title 'Clothoid 3D Trajectory'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");

        // Set fixed ranges for X, Y, and Z axes
        fprintf(gnuplot, "set xrange [-20:80]\n");   // Set the fixed range for X
        fprintf(gnuplot, "set yrange [-20:80]\n");   // Set the fixed range for Y
        fprintf(gnuplot, "set zrange [-20:80]\n");   // Set the fixed range for Z

        // Set view angle for better 3D visualization
        fprintf(gnuplot, "set view 45, 65\n");  // You can adjust the angle here

        // Plot the data in 3D (use correct filename)
        fprintf(gnuplot, "splot 'clothoid_point.dat' using 1:2:3 with lines\n");

        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

#endif // CLOTHOID_2_NO_FIT_H
