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
#ifndef TEST_DOGLEG_3D_H
#define TEST_DOGLEG_3D_H

#include "../eq44.h"

void test_dogleg_3d() {

    // Clothoid compound start point.
    double xs = 0;
    double ys = 0;
    double zs = 0;

    // Clothoid compound end point.
    double xe = 2;
    double ye = 2;
    double ze = 2;

    double degrees_xy_start = 0.0;
    double degrees_xy_end = 0.0;

    double degrees_z_start = 0.0;
    double degrees_z_end = 0.0;

    double ltot = 2;

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
    double y11 = 0.001;
    double y21 = 0.001;
    double s1 = ltot/4;

    dogleg_fit(theta10,theta14,
               kappa10, kappa14,
               sharpness10, sharpness14,
               theta20, theta24,
               kappa20,  kappa24,
               sharpness20,sharpness24,
               xs,  ys,  zs,
               xe,  ye,  ze,
               &y11, &y21, &s1);

    printf("y11: %f \n",y11);
    printf("y21: %f \n",y21);
    printf("s1: %f \n",s1);

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
    eq44_G123(theta10, theta14, kappa10, kappa14, sharpness10, sharpness14, y11, s1,
              &sharpness11, &sharpness12, &sharpness13,
              &kappa11, &kappa12, &kappa13,
              &theta11, &theta12, &theta13,
              &y12, &y13, &y14);

    // eq44, second step, solve G3 continuity for i=2.
    eq44_G123(theta20, theta24, kappa20, kappa24, sharpness20, sharpness24, y21, s1,
              &sharpness21, &sharpness22, &sharpness23,
              &kappa21, &kappa22, &kappa23,
              &theta21, &theta22, &theta23,
              &y22, &y23, &y24);

    double x1, y1, z1; // Endpoint clothoid 1.
    double x2, y2, z2; // Endpoint clothoid 2.
    double x3, y3, z3; // Endpoint clothoid 3.
    double x4, y4, z4; // Endpoint clothoid 4.
    double l1, l2, l3, l4;
    int segments = 5;

    // Print to file for visualization (if needed)
    FILE *data_file = fopen("clothoid_point.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }


    // Clothoid 1. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double s = progress * s1;
        eq14_gauss(s, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, y11, y21, &x1, &y1, &z1, &l1);
        fprintf(data_file, "%lf %lf %lf\n", x1, y1, z1);
    }

    // Clothoid 2. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double s = progress * s1;
        eq14_gauss(s, x1, y1, z1, theta11, theta21, kappa11, kappa21, sharpness11, sharpness21, y12, y22, &x2, &y2, &z2, &l2);
        fprintf(data_file, "%lf %lf %lf\n", x2, y2, z2);
    }

    // Clothoid 3. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double s = progress * s1;
        eq14_gauss(s, x2, y2, z2, theta12, theta22, kappa12, kappa22, sharpness12, sharpness22, y13, y23, &x3, &y3, &z3, &l3);
        fprintf(data_file, "%lf %lf %lf\n", x3, y3, z3);
    }

    // Clothoid 4. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double s = progress * s1;
        eq14_gauss(s, x3, y3, z3, theta13, theta23, kappa13, kappa23, sharpness13, sharpness23, y14, y24, &x4, &y4, &z4, &l4);
        fprintf(data_file, "%lf %lf %lf\n", x4, y4, z4);
    }

    fclose(data_file);

    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up 3D plotting
        fprintf(gnuplot, "set title 'Clothoid 3D Trajectory'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");

        // Set fixed ranges for X, Y, and Z axes
        fprintf(gnuplot, "set xrange [-1:10]\n");   // Set the fixed range for X
        fprintf(gnuplot, "set yrange [-1:10]\n");   // Set the fixed range for Y
        fprintf(gnuplot, "set zrange [-1:10]\n");   // Set the fixed range for Z

        // Set view angle for better 3D visualization
        fprintf(gnuplot, "set view 70, 20\n");  // You can adjust the angle here

        // Plot the data in 3D (use correct filename)
        fprintf(gnuplot, "splot 'clothoid_point.dat' using 1:2:3 with lines\n");

        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}


#endif // TEST_DOGLEG_3D_H
