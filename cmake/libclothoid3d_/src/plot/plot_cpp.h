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
#ifndef PLOT_CPP_H
#define PLOT_CPP_H

#include <iostream>
#include <fstream>
#include <cstdio>
#include "../segment.h"
#include "../eq14.h"

int plot_cpp(struct segment data, double min, double max) {
    // Use std::ofstream for file handling
    std::ofstream data_file("clothoid_point_1.dat");
    if (!data_file) {
        std::cerr << "Error opening file!" << std::endl;
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
        double progress = (double)i / (segments - 1);
        double s = progress * data.s1;
        eq14_gauss(s, data.xs, data.ys, data.zs, data.theta10, data.theta20, data.kappa10, data.kappa20, data.sharpness10, data.sharpness20, data.y11, data.y21, &x1, &y1, &z1, &l1);
        data_file << x1 << " " << y1 << " " << z1 << "\n";
    }

    // Clothoid 2. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);
        double s = progress * data.s1;
        eq14_gauss(s, x1, y1, z1, data.theta11, data.theta21, data.kappa11, data.kappa21, data.sharpness11, data.sharpness21, data.y12, data.y22, &x2, &y2, &z2, &l2);
        data_file << x2 << " " << y2 << " " << z2 << "\n";
    }

    // Clothoid 3. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);
        double s = progress * data.s1;
        eq14_gauss(s, x2, y2, z2, data.theta12, data.theta22, data.kappa12, data.kappa22, data.sharpness12, data.sharpness22, data.y13, data.y23, &x3, &y3, &z3, &l3);
        data_file << x3 << " " << y3 << " " << z3 << "\n";
    }

    // Clothoid 4. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);
        double s = progress * data.s1;
        eq14_gauss(s, x3, y3, z3, data.theta13, data.theta23, data.kappa13, data.kappa23, data.sharpness13, data.sharpness23, data.y14, data.y24, &x4, &y4, &z4, &l4);
        data_file << x4 << " " << y4 << " " << z4 << "\n";
    }

    std::cout << "endpoint x: " << x4 << " y: " << y4 << " z: " << z4 << std::endl;
    std::cout << "clothoid compound length: " << l1 + l2 + l3 + l4 << std::endl;

    data_file.close();

    FILE* gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up 3D plotting
        fprintf(gnuplot, "set title 'Clothoid 3D Trajectory'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");

        fprintf(gnuplot, "set xrange [%f:%f]\n", min, max);
        fprintf(gnuplot, "set yrange [%f:%f]\n", min, max);
        fprintf(gnuplot, "set zrange [%f:%f]\n", min, max);

        // Set view angle for better 3D visualization
        fprintf(gnuplot, "set view 70, 20\n");

        // Plot the data in 3D
        fprintf(gnuplot, "splot 'clothoid_point_1.dat' using 1:2:3 with lines\n");

        fflush(gnuplot);
        pclose(gnuplot);
    } else {
        std::cerr << "Error: Unable to open GNUplot" << std::endl;
    }

    return 1;
}

/* Test ~/interpolate/interpolate.h
 *
 * min = plot area min limit.
 * max = plot area max limit.
 *
 */
int plot_interpolate(struct segment data, double min, double max, double stot) {
    // Use std::ofstream for file handling
    std::ofstream data_file("clothoid_point.dat");
    if (!data_file) {
        std::cerr << "Error opening file!" << std::endl;
        return 0;
    }

    double segments = 100;

    // Test interpolation.
    for (int i = 0; i < segments; i++) {
        double progress = static_cast<double>(i) / (segments - 1);  // Correct floating-point division
        double pi[3] = {0, 0, 0};
        double s = stot * progress;
        eq14_interpolate_clothoid(&data, s, pi);
        data_file << pi[0] << " " << pi[1] << " " << pi[2] << "\n";
    }

    data_file.close();

    // Open GNUplot
    FILE* gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != nullptr) {
        // Set up 3D plotting
        fprintf(gnuplot, "set title 'Clothoid 3D Trajectory'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");

        fprintf(gnuplot, "set xrange [%f:%f]\n", min, max);
        fprintf(gnuplot, "set yrange [%f:%f]\n", min, max);
        fprintf(gnuplot, "set zrange [%f:%f]\n", min, max);

        // Set view angle for better 3D visualization
        fprintf(gnuplot, "set view 70, 20\n");

        // Plot the data in 3D
        fprintf(gnuplot, "splot 'clothoid_point.dat' using 1:2:3 with lines\n");

        fclose(gnuplot);
    } else {
        std::cerr << "Error: Unable to open GNUplot" << std::endl;
    }

    return 1; // Success
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
int plot_algorithm1(struct segment data, double Ptraj[3], double Pi[3], double min, double max) {
    // Use std::ofstream for file handling
    std::ofstream data_file("clothoid_point.dat");
    if (!data_file) {
        std::cerr << "Error opening file!" << std::endl;
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
        double progress = static_cast<double>(i) / (segments - 1);
        double s = progress * data.s1;
        eq14_gauss(s, data.xs, data.ys, data.zs, data.theta10, data.theta20, data.kappa10, data.kappa20, data.sharpness10, data.sharpness20, data.y11, data.y21, &x1, &y1, &z1, &l1);
        data_file << x1 << " " << y1 << " " << z1 << "\n";
    }

    // Clothoid 2. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = static_cast<double>(i) / (segments - 1);
        double s = progress * data.s1;
        eq14_gauss(s, x1, y1, z1, data.theta11, data.theta21, data.kappa11, data.kappa21, data.sharpness11, data.sharpness21, data.y12, data.y22, &x2, &y2, &z2, &l2);
        data_file << x2 << " " << y2 << " " << z2 << "\n";
    }

    // Clothoid 3. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = static_cast<double>(i) / (segments - 1);
        double s = progress * data.s1;
        eq14_gauss(s, x2, y2, z2, data.theta12, data.theta22, data.kappa12, data.kappa22, data.sharpness12, data.sharpness22, data.y13, data.y23, &x3, &y3, &z3, &l3);
        data_file << x3 << " " << y3 << " " << z3 << "\n";
    }

    // Clothoid 4. Trajectory.
    for (int i = 0; i < segments; i++) {
        double progress = static_cast<double>(i) / (segments - 1);
        double s = progress * data.s1;
        eq14_gauss(s, x3, y3, z3, data.theta13, data.theta23, data.kappa13, data.kappa23, data.sharpness13, data.sharpness23, data.y14, data.y24, &x4, &y4, &z4, &l4);
        data_file << x4 << " " << y4 << " " << z4 << "\n";
    }

    // Add Ptraj and Pi to the data file
    data_file << "\n\n"; // Separate clothoid trajectory from points
    data_file << Ptraj[0] << " " << Ptraj[1] << " " << Ptraj[2] << "\n"; // Ptraj
    data_file << Pi[0] << " " << Pi[1] << " " << Pi[2] << "\n"; // Pi

    std::cout << "Endpoint x: " << x4 << " y: " << y4 << " z: " << z4 << std::endl;
    std::cout << "Clothoid compound length: " << l1 + l2 + l3 + l4 << std::endl;

    data_file.close();

    // Plot using GNUplot
    FILE* gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != nullptr) {
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
        fprintf(gnuplot, "set view 70, 20\n");

        // Plot the data in 3D
        fprintf(gnuplot, "splot 'clothoid_point.dat' using 1:2:3 with lines\n");

        fclose(gnuplot);
    } else {
        std::cerr << "Error: Unable to open GNUplot" << std::endl;
    }

    return 1; // Success
}

#endif // PLOT_CPP_H
