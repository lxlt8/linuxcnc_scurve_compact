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
#ifndef PLOT_C_H
#define PLOT_C_H

#include <stdio.h>
#include "../segment.h"
#include "../eq14.h"

/* Plot the results in GNU plot.
 *
 * min = plot area min limit.
 * max = plot area max limit.
 *
 */
int plot_c(struct segment data, double min, double max){

    // Draw result.
    // Print to file for visualization (if needed)
    FILE *data_file = fopen("clothoid_point_1.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return 0;
    }

    double l1=0, l2=0, l3=0, l4=0;
    double x1=0, y1=0, z1=0;
    double x2=0, y2=0, z2=0;
    double x3=0, y3=0, z3=0;
    double x4=0, y4=0, z4=0;
    double segments=100;

    // Clothoid 1. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double s = progress * data.s1;
        eq14_gauss(s, data.xs, data.ys, data.zs, data.theta10, data.theta20, data.kappa10, data.kappa20, data.sharpness10, data.sharpness20, data.y11, data.y21, &x1, &y1, &z1, &l1);
        fprintf(data_file, "%lf %lf %lf\n", x1, y1, z1);
    }
    // printf("x1: %f y1: %f z1: %f \n",x1,y1,z1);

    // Clothoid 2. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double s = progress * data.s1;
        eq14_gauss(s, x1, y1, z1, data.theta11, data.theta21, data.kappa11, data.kappa21, data.sharpness11, data.sharpness21, data.y12, data.y22, &x2, &y2, &z2, &l2);
        fprintf(data_file, "%lf %lf %lf\n", x2, y2, z2);
    }
    // printf("x2: %f y2: %f z2: %f \n",x2,y2,z2);

    // Clothoid 3. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double s = progress * data.s1;
        eq14_gauss(s, x2, y2, z2, data.theta12, data.theta22, data.kappa12, data.kappa22, data.sharpness12, data.sharpness22, data.y13, data.y23, &x3, &y3, &z3, &l3);
        fprintf(data_file, "%lf %lf %lf\n", x3, y3, z3);
    }
    // printf("x3: %f y3: %f z3: %f \n",x3,y3,z3);

    // Clothoid 4. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double s = progress * data.s1;
        eq14_gauss(s, x3, y3, z3, data.theta13, data.theta23, data.kappa13, data.kappa23, data.sharpness13, data.sharpness23, data.y14, data.y24, &x4, &y4, &z4, &l4);
        fprintf(data_file, "%lf %lf %lf\n", x4, y4, z4);
    }
    // printf("x4: %f y4: %f z4: %f \n",x4,y4,z4);

    printf("endpoint x: %f y: %f z: %f \n",x4,y4,z4);
    printf("clothoid compound length: %f \n",l1+l2+l3+l4);

    fflush(data_file);
    fclose(data_file);

    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up 3D plotting
        fprintf(gnuplot, "set title 'Clothoid 3D Trajectory'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");

        fprintf(gnuplot, "set xrange [%f:%f]\n", min, max); // Use %f to format the double value
        fprintf(gnuplot, "set yrange [%f:%f]\n", min, max); // Use %f to format the double value
        fprintf(gnuplot, "set zrange [%f:%f]\n", min, max); // Use %f to format the double value

        // Set view angle for better 3D visualization
        fprintf(gnuplot, "set view 70, 20\n");  // You can adjust the angle here

        // Plot the data in 3D (use correct filename)
        fprintf(gnuplot, "splot 'clothoid_point_1.dat' using 1:2:3 with lines\n");

        fflush(gnuplot);
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Test ~/interpolate/interpolate.h
 *
 * min = plot area min limit.
 * max = plot area max limit.
 *
 */
int plot_interpolate(struct clothoid_data data, double min, double max, double stot){

    // Draw result.
    // Print to file for visualization (if needed)
    FILE *data_file = fopen("clothoid_point.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return 0;
    }

    double segments=100;

    // Test interpolation.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double pi[3]={0,0,0};
        double s = stot * progress;
        eq14_interpolate_clothoid(&data,s,pi);
        fprintf(data_file, "%lf %lf %lf\n", pi[0], pi[1], pi[2]);
    }

    fclose(data_file);

    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up 3D plotting
        fprintf(gnuplot, "set title 'Clothoid 3D Trajectory'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");

        fprintf(gnuplot, "set xrange [%f:%f]\n", min, max); // Use %f to format the double value
        fprintf(gnuplot, "set yrange [%f:%f]\n", min, max); // Use %f to format the double value
        fprintf(gnuplot, "set zrange [%f:%f]\n", min, max); // Use %f to format the double value

        // Set view angle for better 3D visualization
        fprintf(gnuplot, "set view 70, 20\n");  // You can adjust the angle here

        // Plot the data in 3D (use correct filename)
        fprintf(gnuplot, "splot 'clothoid_point.dat' using 1:2:3 with lines\n");

        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Plot the results in GNU plot.
 *
 * min = plot area min limit.
 * max = plot area max limit.
 *
 * Ptraj = Point in space.
 * Pi = Closest point on clothoid to Ptraj.
 *
 */
int plot_algorithm1(struct clothoid_data data, double Ptraj[3], double Pi[3], double min, double max) {
    // Draw result.
    // Print to file for visualization (if needed)
    FILE *data_file = fopen("clothoid_point.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return 0;
    }

    double l1 = 0, l2 = 0, l3 = 0, l4 = 0;
    double x1 = 0, y1 = 0, z1 = 0;
    double x2 = 0, y2 = 0, z2 = 0;
    double x3 = 0, y3 = 0, z3 = 0;
    double x4 = 0, y4 = 0, z4 = 0;
    double segments = 100;

    // Clothoid 1. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double s = progress * data.s1;
        eq14_gauss(s, data.xs, data.ys, data.zs, data.theta10, data.theta20, data.kappa10, data.kappa20, data.sharpness10, data.sharpness20, data.y11, data.y21, &x1, &y1, &z1, &l1);
        fprintf(data_file, "%lf %lf %lf\n", x1, y1, z1);
    }

    // Clothoid 2. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double s = progress * data.s1;
        eq14_gauss(s, x1, y1, z1, data.theta11, data.theta21, data.kappa11, data.kappa21, data.sharpness11, data.sharpness21, data.y12, data.y22, &x2, &y2, &z2, &l2);
        fprintf(data_file, "%lf %lf %lf\n", x2, y2, z2);
    }

    // Clothoid 3. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double s = progress * data.s1;
        eq14_gauss(s, x2, y2, z2, data.theta12, data.theta22, data.kappa12, data.kappa22, data.sharpness12, data.sharpness22, data.y13, data.y23, &x3, &y3, &z3, &l3);
        fprintf(data_file, "%lf %lf %lf\n", x3, y3, z3);
    }

    // Clothoid 4. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division
        double s = progress * data.s1;
        eq14_gauss(s, x3, y3, z3, data.theta13, data.theta23, data.kappa13, data.kappa23, data.sharpness13, data.sharpness23, data.y14, data.y24, &x4, &y4, &z4, &l4);
        fprintf(data_file, "%lf %lf %lf\n", x4, y4, z4);
    }

    // Add Ptraj and Pi to the data file
    fprintf(data_file, "\n\n"); // Separate clothoid trajectory from points
    fprintf(data_file, "%lf %lf %lf\n", Ptraj[0], Ptraj[1], Ptraj[2]); // Ptraj
    fprintf(data_file, "%lf %lf %lf\n", Pi[0], Pi[1], Pi[2]); // Pi

    printf("Endpoint x: %f y: %f z: %f \n", x4, y4, z4);
    printf("Clothoid compound length: %f \n", l1 + l2 + l3 + l4);

    fclose(data_file);

    // Plot using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up 3D plotting
        fprintf(gnuplot, "set title 'Clothoid 3D Trajectory with Line Ptraj-Pi'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");

        // Set plot range
        fprintf(gnuplot, "set xrange [%f:%f]\n", min, max);
        fprintf(gnuplot, "set yrange [%f:%f]\n", min, max);
        fprintf(gnuplot, "set zrange [%f:%f]\n", min, max);

        // Set view angle for better 3D visualization
        fprintf(gnuplot, "set view 70, 20\n");  // You can adjust the angle here

        // Plot the data in 3D (use correct filename)
        fprintf(gnuplot, "splot 'clothoid_point.dat' using 1:2:3 with lines\n");

        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }

    return 1; // Success
}

#endif // PLOT_C_H
