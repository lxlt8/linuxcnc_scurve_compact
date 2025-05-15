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
#ifndef LINE3D_H
#define LINE3D_H

#include "stdio.h"
#include "math.h"
#include "../segment.h"
#include "../math/vector_math.h"
#include "../math/geometry_math.h"
#include "../eq44_G0.h"
#include "../eq30.h"

/* Interpolate a line.
 */
static void interpolate_line3d(const struct segment *seg, const double progress, double pi[3]){

    // Linear interpolation for each component (x, y, z)
    pi[0] = seg->p0[0] + progress * (seg->p1[0] - seg->p0[0]);  // x-component
    pi[1] = seg->p0[1] + progress * (seg->p1[1] - seg->p0[1]);  // y-component
    pi[2] = seg->p0[2] + progress * (seg->p1[2] - seg->p0[2]);  // z-component
}

static void test_line();

/* Initialize a gcode line segment.
 *
 * Somehow this function fails to compile if in .c file.
 *
 * Input start, end coordinates for xyz.
 * Returns a gcode segment where values like
 * kappa, unit vector, length are calculated.
 */
static void init_line(double p0[3], double p1[3], struct segment *seg, int update_original_length){

    seg->segment_type=LINE;
    seg->canon_code=G1;

    seg->cw = 0;
    seg->turns = 0;
    seg->radius = 0;
    seg->angle = 0;

    seg->theta10=0;
    seg->theta14=0;
    seg->theta20=0;
    seg->theta24=0;

    seg->kappa10=0;
    seg->kappa14=0;
    seg->kappa20=0;
    seg->kappa24=0;

    seg->sharpness10=0;
    seg->sharpness14=0;
    seg->sharpness20=0;
    seg->sharpness24=0;

    copy_vector(p0,seg->p0);
    copy_vector(p1,seg->p1);

    // Distances x,y,z
    double dx = seg->p1[0] - seg->p0[0];
    double dy = seg->p1[1] - seg->p0[1];
    double dz = seg->p1[2] - seg->p0[2];

    // Line vector in 3d space.
    double line_vector[3] = {dx,dy,dz};

    // Length
    seg->length = distance_3d(p0,p1);

    // Vn = vector in 3d space normalized to 0-1.
    double vn[3];
    copy_vector(line_vector,vn);
    normalize(vn);

    // Decompose vector in xy & z components.
    double theta1i, theta2i;
    eq30_decompose_theta_components(vn,&theta1i,&theta2i);

    seg->theta14 = seg->theta10 = theta1i;
    seg->theta24 = seg->theta20 = theta2i;

    if(update_original_length){
        // Used for trimming.
        seg->original_length = seg->length;
    }
}


/* Plot to file
 */
static void plot_line_file(struct segment *seg, FILE *data_file){

    double segments=20;
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division

        double pi[3]={};
        interpolate_line3d(seg,progress,pi);
        fprintf(data_file, "%lf %lf %lf\n", pi[0], pi[1], pi[2]);
    }
    fprintf(data_file, "\n\n");  // Add two blank lines to separate datasets
}

/* Plot line.
 */
static void plot_line(struct segment *seg){

    // Draw result.
    // Print to file for visualization (if needed)
    FILE *data_file = fopen("arc_helix.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    double segments=20;
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);  // Correct floating-point division

        double pi[3]={};
        interpolate_line3d(seg,progress,pi);
        fprintf(data_file, "%lf %lf %lf\n", pi[0], pi[1], pi[2]);
        // printf("pi: (%f, %f, %f)\n", pi[0], pi[1], pi[2]);
    }

    fflush(data_file);
    fclose(data_file);

    double min = 0;
    double max = 100;

    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up 3D plotting
        fprintf(gnuplot, "set title 'Arc helix test'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");

        fprintf(gnuplot, "set xrange [%f:%f]\n", min, max); // Use %f to format the double value
        fprintf(gnuplot, "set yrange [%f:%f]\n", min, max); // Use %f to format the double value
        fprintf(gnuplot, "set zrange [%f:%f]\n", min, max); // Use %f to format the double value

        // Set view angle for better 3D visualization
        fprintf(gnuplot, "set view 70, 20\n");  // You can adjust the angle here

        // Plot the data in 3D (use correct filename)
        fprintf(gnuplot, "splot 'arc_helix.dat' using 1:2:3 with lines\n");

        fflush(gnuplot);
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Test line init, plot line interpolation.
 */
static void test_line(){

    struct segment seg;
    double l0[3]={0,0,0};
    double l1[3]={100,100,50};
    init_line(l0,l1,&seg,0);
    plot_line(&seg);
    print_segment(&seg);
}


#endif // LINE3D_H








