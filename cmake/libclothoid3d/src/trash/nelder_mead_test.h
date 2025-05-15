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
#ifndef NELDER_MEAD_TEST_H
#define NELDER_MEAD_TEST_H

#include <stdio.h>
#include "../solver/nelder_mead.h"

/* Example. Fit a single clothoid given a target position.
 * Plot the result in GNU plot.
 *
 * Tested ok.
 */
int nelder_mead_test(){

    // Print to file for visualization (if needed)
    FILE *data_file = fopen("clothoid_point.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return 0;
    }

    // Start point.
    double xs = 0.0;
    double ys = 0.0;
    double zs = 0.0;

    // Target point.
    double xe = 100.0;
    double ye = 100.0;
    double ze = 10.0;

    // Start angles, end angles.
    double degrees_xy_start = 0.0;
    double degrees_xy_end = 0.0;
    double degrees_z_start = 0.0;
    double degrees_z_end = -20.0;

    // Guess values, 3 unknowns.
    double s1 = 0.0001;  // Lenght of individual clothoid.
    double y11 = 0.000; // Z rigid factor.
    double y21 = 0.000; // Xy rigid factor.

    // Initial values at z spline start.
    double theta10 = degrees_z_start * (M_PI / 180.0); // To radians.
    double kappa10 = 0;
    double sharpness10 = 0;
    // Initial values at z spline end.
    double theta14 = degrees_z_end * (M_PI / 180.0); // To radians.
    double kappa14 = 0;
    double sharpness14 = 0;

    // Initial values at xy spline start.
    double theta20 = degrees_xy_start * (M_PI / 180.0);
    double kappa20 = 0;
    double sharpness20 = 0;
    // Initial values at xy spline end.
    double theta24 = degrees_xy_end * (M_PI / 180.0);
    double kappa24 = 0;
    double sharpness24 = 0;

    // Gsl solver call. Iterate the clothoid to given end position.
    init_nelder_mead(
                theta10, theta14, kappa10, kappa14, sharpness10, sharpness14,
                theta20, theta24, kappa20, kappa24, sharpness20, sharpness24,
                xs, ys, zs, xe, ye, ze,
                &y11, &y21, &s1
                );

    // To be calculated by eq44.
    double theta11, theta12, theta13;
    double theta21, theta22, theta23;

    double kappa11, kappa12, kappa13;
    double kappa21, kappa22, kappa23;

    double sharpness11, sharpness12, sharpness13;
    double sharpness21, sharpness22, sharpness23;

    double y12, y13, y14;
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


    double l1, l2, l3, l4;
    double x1, y1, z1;
    double x2, y2, z2;
    double x3, y3, z3;
    double x4, y4, z4;
    double segments=100;

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

    printf("endpoint x: %f y: %f z: %f \n",x4,y4,z4);
    printf("clothoid compound length: %f \n",l1+l2+l3+l4);

    fclose(data_file);

    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up 3D plotting
        fprintf(gnuplot, "set title 'Clothoid 3D Trajectory'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");

        // Set fixed ranges for X, Y, and Z axes
        fprintf(gnuplot, "set xrange [-5:500]\n");   // Set the fixed range for X
        fprintf(gnuplot, "set yrange [-5:500]\n");   // Set the fixed range for Y
        fprintf(gnuplot, "set zrange [-5:300]\n");   // Set the fixed range for Z

        // Set view angle for better 3D visualization
        fprintf(gnuplot, "set view 70, 20\n");  // You can adjust the angle here

        // Plot the data in 3D (use correct filename)
        fprintf(gnuplot, "splot 'clothoid_point.dat' using 1:2:3 with lines\n");

        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
    return 0;
}

#endif // NELDER_MEAD_TEST_H
