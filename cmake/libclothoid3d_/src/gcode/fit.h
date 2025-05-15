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
#ifndef FIT_H
#define FIT_H

#include "time.h"
#include "../segment.h"
#include "../math/vector_math.h"
#include "../math/geometry_math.h"
#include "../solver/ceres_cppapi.h"
#include "../solver/residual_fillet_fit.h"
#include "eq19.h" // Curvature extrema.
#include "line3d.h"
#include "arc3d.h"

/* Fit a clothoid between 2 segments, given max path deviation G64 P[x].
 *
 * seg0 = First segment.
 * seg1 = Second segment.
 * seg2 = Clothoid result segment.
 * s = Initial sample trim value for ceres. In ceres set to min 1e-6.
 * max_deviation = Max path deviation.
 *
 */
static void find_fit(struct segment *seg0,
                     struct segment *seg1,
                     struct segment *seg2,
                     double *max_deviation,
                     double *s
                     ){

    // Calculate trim dist s.
    init_ceres_cpp_fit(seg0,seg1,seg2,max_deviation,s);
}

/* Modify segments to apply the clothoid fillet.
 *
 * seg0 = First segment.
 * seg1 = Second segment.
 * seg2 = Clothoid result segment.
 * s = trim lenght for seg0, seg1. Calculated by find_fit().
 */
static int apply_fit(struct segment *seg0,
                     struct segment *seg1,
                     struct segment *seg2,
                     double s
                     ){

    // Temponary copy.
    struct segment seg0_temp = *seg0;
    struct segment seg1_temp = *seg1;

    trim_segment(seg0,0,1,&s);
    trim_segment(seg1,1,0,&s);
    init_inbetween_clothoid(seg0,seg1,seg2,-INFINITY,INFINITY);

    // If clothoid is self intersecting, undo operation.
    if(self_intersecting_clothoid(seg2)){
        printf("clothoid; undo operation, self intersecting.\n");
        *seg0 = seg0_temp;
        *seg1 = seg1_temp;
        return -1;
    }

    // If clothoid is not at end point after init_inbetween_clothoid, undo operation.
    if(distance_3d(seg2->p1,seg1->p0)>1e-3){
        printf("clothoid; undo operation, no endpoint fit found.\n");
        *seg0 = seg0_temp;
        *seg1 = seg1_temp;
        return -1;
    }
    return 0;
}

/* Modify segments to apply test line fillet.
 *
 * seg0 = First segment.
 * seg1 = Second segment.
 * seg2 = Line result segment.
 * s = trim lenght for seg0, seg1. Calculated by find_fit().
 */
static void apply_line_fit(struct segment *seg0,
                           struct segment *seg1,
                           struct segment *seg2,
                           double s
                           ){

    trim_segment(seg0,0,1,&s);
    trim_segment(seg1,1,0,&s);

    init_line(seg0->p1,seg1->p0,seg2,1);
}


/* General fit algoritme to fit a clothoid between 2 segments.
 *
 * seg0 = First segment.
 * seg1 = Second segment.
 * seg2 = Clothoid result segment.
 * max_deviation = Max path deviation G64 P[x].
 */
static int fit(struct segment *seg0,
               struct segment *seg1,
               struct segment *seg2,
               double max_deviation
               ){

    if(max_deviation==0){
        seg2->length=0;
        return -1;
    }

    // Holds the trim distance for seg0 & seg1.
    double s;

    // Calculate trim dist s.
    init_ceres_cpp_fit(seg0,seg1,seg2,&max_deviation,&s);

    printf("trim dist: %f \n",s);

    // Modify segments and construct inbetween clothoid.
    return apply_fit(seg0,seg1,seg2,s);
}

/* Curvature extrema, kappa max.
 *
 * Used in motion profile to determine max velocity based
 * on curvature.
 *
 * Kappa line = 0.
 * Kappa arc = 1/Radius.
 */
static double curvature_extrema(struct segment *seg){

    double curvature_extrema=0;

    if(seg->segment_type==LINE){
        curvature_extrema=0;
    } else if (seg->segment_type==ARC){
        curvature_extrema=1/seg->radius;
    } else if (seg->segment_type==CLOTHOID){
        eq19_curvature_extrema(seg,&curvature_extrema);
    } else {
        printf("error: curvature extrema. \n");
    }

    return curvature_extrema;
}

/* Test a clothoid fillet algoritme, given 2 segments
 * and max deviation value G64 P[x].
 *
 * s = trim lenght for seg0, seg1.
 *
 */
static void test_fit(){

    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    // Start the timer
    clock_t start = clock();

    double corner[3]={0,0,0};

    struct segment seg0;
    seg0.segment_type=LINE;
    double l0_start[3]={-100,0,0};
    double l0_end[3]={0,0,0};
    init_line(l0_start,l0_end,&seg0,1);
    // print_segment(&seg0);

    struct segment seg1;
    seg1.segment_type=LINE;
    double l1_start[3]={0,0,0};
    double l1_end[3]={100,-100,-100};
    init_line(l1_start,l1_end,&seg1,1);
    // print_segment(&seg1);

    plot_line_file(&seg0,data_file);
    plot_line_file(&seg1,data_file);

    struct segment seg2;
    seg2.segment_type=CLOTHOID;

    double max_deviation = 5;

    fit(&seg0,&seg1,&seg2,max_deviation);

    printf("Curvature extrema: %f \n",curvature_extrema(&seg2));

    // Check deviation for second time.
    double deviation_result;
    double d;
    double pi[3];
    algorithm1(corner,seg2,1e-6,pi,&d,&deviation_result);
    printf("Deviation result = %f \n",deviation_result);

    // print_segment(&seg0);
    // print_segment(&seg1);
    // print_segment(&seg2);
    plot_clothoid_file(&seg2,data_file);

    struct segment seg3;
    seg3.segment_type=LINE;
    init_line(pi,corner,&seg3,0);
    plot_line_file(&seg3,data_file);

    // Stop the timer
    clock_t end = clock();

    // Calculate the elapsed time in milliseconds
    double elapsed_time = (double)(end - start) * 1000.0 / CLOCKS_PER_SEC;

    // Print the elapsed time
    printf("Performance: %.2f ms\n", elapsed_time);

    // Close the data file
    fclose(data_file);

    // Plot the data using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up the plot
        fprintf(gnuplot, "set title 'Multiple Segments Plot'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");
        fprintf(gnuplot, "set xrange [-100:100]\n");  // Adjust ranges as needed
        fprintf(gnuplot, "set yrange [-100:100]\n");
        fprintf(gnuplot, "set zrange [-100:100]\n");
        fprintf(gnuplot, "set view 70, 20\n");  // Adjust view angle

        // Plot all datasets with titles
        fprintf(gnuplot, "splot 'plot.dat' index 0 using 1:2:3 with lines title 'Segment 1', "
                         "'plot.dat' index 1 using 1:2:3 with lines title 'Segment 2', "
                         "'plot.dat' index 2 using 1:2:3 with lines title 'Segment 3', "
                         "'plot.dat' index 3 using 1:2:3 with lines title 'segment 4'\n");

        // Close GNUplot
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Test a clothoid fillet algoritme, given 2 segments
 * and max deviation value G64 P[x].
 *
 * s = trim lenght for seg0, seg1.
 *
 */
static void test_fit_no_plot(){

    // Start the timer
    clock_t start = clock();

    double corner[3]={0,0,0};

    struct segment seg0;
    seg0.segment_type=LINE;
    double l0_start[3]={-100,0,0};
    double l0_end[3]={0,0,0};
    init_line(l0_start,l0_end,&seg0,0);
    // print_segment(&seg0);

    struct segment seg1;
    seg1.segment_type=LINE;
    double l1_start[3]={0,0,0};
    double l1_end[3]={100,-100,-100};
    init_line(l1_start,l1_end,&seg1,0);
    // print_segment(&seg1);

    struct segment seg2;
    seg2.segment_type=CLOTHOID;

    double max_deviation = 20;

    fit(&seg0,&seg1,&seg2,max_deviation);

    // Stop the timer
    clock_t end = clock();

    // Calculate the elapsed time in milliseconds
    double elapsed_time = (double)(end - start) * 1000.0 / CLOCKS_PER_SEC;

    // Print the elapsed time
    printf("Fit done, performance: %.2f ms\n", elapsed_time);
}

/* Test a clothoid fillet algoritme, given 2 segments
 * and max deviation value G64 P[x].
 *
 * s = trim lenght for seg0, seg1.
 *
 */
static void test_line_line_fit(){

    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    // Start the timer
    clock_t start = clock();

    double corner[3]={0,0,0};

    struct segment seg0;
    seg0.segment_type=LINE;
    double l0_start[3]={100,0,100};
    double l0_end[3]={100,0,0};
    init_line(l0_start,l0_end,&seg0,1);
    // print_segment(&seg0);

    struct segment seg1;
    seg1.segment_type=LINE;
    double l1_start[3]={100,0,0};
    double l1_end[3]={0,0,0};
    init_line(l1_start,l1_end,&seg1,1);
    // print_segment(&seg1);

    plot_line_file(&seg0,data_file);
    plot_line_file(&seg1,data_file);

    struct segment seg2;
    seg2.segment_type=CLOTHOID;

    double max_deviation = 20;
    double s = 0;

    // Find the best fit trim length s.
    find_fit(&seg0,&seg1,&seg2,&max_deviation,&s);
    printf("segments trim length = %f \n",s);
    printf("max deviation = %f \n",max_deviation);

    // Modify segments.
    apply_fit(&seg0,&seg1,&seg2,s);

    // Check deviation for second time.
    double deviation_result;
    double d;
    double pi[3];
    algorithm1(corner,seg2,1e-6,pi,&d,&deviation_result);
    printf("deviation result = %f \n",deviation_result);

    // print_segment(&seg0);
    // print_segment(&seg1);
    // print_segment(&seg2);
    plot_clothoid_file(&seg2,data_file);

    struct segment seg3;
    seg3.segment_type=LINE;
    init_line(pi,corner,&seg3,0);
    plot_line_file(&seg3,data_file);

    // Stop the timer
    clock_t end = clock();

    // Calculate the elapsed time in milliseconds
    double elapsed_time = (double)(end - start) * 1000.0 / CLOCKS_PER_SEC;

    // Print the elapsed time
    printf("Elapsed time: %.2f ms\n", elapsed_time);

    // Close the data file
    fclose(data_file);

    // Plot the data using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up the plot
        fprintf(gnuplot, "set title 'Multiple Segments Plot'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");
        fprintf(gnuplot, "set xrange [-100:100]\n");  // Adjust ranges as needed
        fprintf(gnuplot, "set yrange [-100:100]\n");
        fprintf(gnuplot, "set zrange [-100:100]\n");
        fprintf(gnuplot, "set view 20, 20\n");  // Adjust view angle

        // Plot all datasets with titles
        fprintf(gnuplot, "splot 'plot.dat' index 0 using 1:2:3 with lines title 'Segment 1', "
                         "'plot.dat' index 1 using 1:2:3 with lines title 'Segment 2', "
                         "'plot.dat' index 2 using 1:2:3 with lines title 'Segment 3', "
                         "'plot.dat' index 3 using 1:2:3 with lines title 'segment 4'\n");

        // Close GNUplot
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Test a clothoid fillet algoritme, given 2 segments
 * and max deviation value G64 P[x].
 *
 * s = trim lenght for seg0, seg1.
 *
 */
static void test_line_arc_fit_g2(){

    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    // Start the timer
    clock_t start = clock();

    double corner[3]={0,50,0};

    struct segment seg0;
    seg0.segment_type=LINE;
    double l0_start[3]={0,0,0};
    double l0_end[3]={0,50,0};
    init_line(l0_start,l0_end,&seg0,0);
    // print_segment(&seg0);

    struct segment seg1;
    seg1.segment_type=ARC;
    double l1_p0[3]={0,50,0};
    double l1_p1[3]={0,0,0};
    double l1_pc[3]={0,25,0};
    double l1_pn[3]={0,0,1};
    int cw=1;
    int turns=0;
    init_arc(l1_p0,l1_p1,l1_pc,l1_pn,cw,turns,&seg1,0);
    // print_segment(&seg1);

    plot_line_file(&seg0,data_file);
    plot_arc_file(&seg1,data_file);

    struct segment seg2;
    seg2.segment_type=CLOTHOID;

    double max_deviation = 5;
    double s = 0;

    // Find the best fit trim length s.
    find_fit(&seg0,&seg1,&seg2,&max_deviation,&s);
    printf("segments trim length = %f \n",s);
    printf("max deviation = %f \n",max_deviation);

    // Modify segments.
    apply_fit(&seg0,&seg1,&seg2,s);

    // Check deviation for second time.
    double deviation_result;
    double d;
    double pi[3];
    algorithm1(corner,seg2,1e-6,pi,&d,&deviation_result);
    printf("deviation result = %f \n",deviation_result);

    // print_segment(&seg0);
    // print_segment(&seg1);
    // print_segment(&seg2);
    plot_clothoid_file(&seg2,data_file);

    struct segment seg3;
    seg3.segment_type=LINE;
    init_line(pi,corner,&seg3,0);
    plot_line_file(&seg3,data_file);

    // Stop the timer
    clock_t end = clock();

    // Calculate the elapsed time in milliseconds
    double elapsed_time = (double)(end - start) * 1000.0 / CLOCKS_PER_SEC;

    // Print the elapsed time
    printf("Elapsed time: %.2f ms\n", elapsed_time);

    // Close the data file
    fclose(data_file);

    // Plot the data using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up the plot
        fprintf(gnuplot, "set title 'Multiple Segments Plot'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");
        fprintf(gnuplot, "set xrange [-60:60]\n");  // Adjust ranges as needed
        fprintf(gnuplot, "set yrange [-60:60]\n");
        fprintf(gnuplot, "set zrange [-60:60]\n");
        fprintf(gnuplot, "set view 70, 20\n");  // Adjust view angle

        // Plot all datasets with titles
        fprintf(gnuplot, "splot 'plot.dat' index 0 using 1:2:3 with lines title 'Segment 1', "
                         "'plot.dat' index 1 using 1:2:3 with lines title 'Segment 2', "
                         "'plot.dat' index 2 using 1:2:3 with lines title 'Segment 3', "
                         "'plot.dat' index 3 using 1:2:3 with lines title 'segment 4'\n");

        // Close GNUplot
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Test a clothoid fillet algoritme, given 2 segments
 * and max deviation value G64 P[x].
 *
 * s = trim lenght for seg0, seg1.
 *
 */
static void test_line_arc_fit_g3_0(){

    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    // Start the timer
    clock_t start = clock();

    double corner[3]={50,50,0};

    struct segment seg0;
    seg0.segment_type=LINE;
    double l0_start[3]={50,0,0};
    double l0_end[3]={50,50,0};
    init_line(l0_start,l0_end,&seg0,1);
    // print_segment(&seg0);

    struct segment seg1;
    seg1.segment_type=ARC;
    double l1_p0[3]={50,50,0};
    double l1_p1[3]={50,0,0};
    double l1_pc[3]={50,25,0};
    double l1_pn[3]={0,0,1};
    int cw=0;
    int turns=0;
    init_arc(l1_p0,l1_p1,l1_pc,l1_pn,cw,turns,&seg1,1);
    // print_segment(&seg1);

    plot_line_file(&seg0,data_file);
    plot_arc_file(&seg1,data_file);

    struct segment seg2;
    seg2.segment_type=CLOTHOID;

    double max_deviation = 5;
    double s = 0;

    // Find the best fit trim length s.
    find_fit(&seg0,&seg1,&seg2,&max_deviation,&s);
    printf("segments trim length = %f \n",s);
    printf("max deviation = %f \n",max_deviation);

    // Modify segments.
    apply_fit(&seg0,&seg1,&seg2,s);

    // Check deviation for second time.
    double deviation_result;
    double d;
    double pi[3];
    algorithm1(corner,seg2,1e-6,pi,&d,&deviation_result);
    printf("deviation result = %f \n",deviation_result);

    // print_segment(&seg0);
    // print_segment(&seg1);
    // print_segment(&seg2);
    plot_clothoid_file(&seg2,data_file);

    struct segment seg3;
    seg3.segment_type=LINE;
    init_line(pi,corner,&seg3,0);
    plot_line_file(&seg3,data_file);

    // Stop the timer
    clock_t end = clock();

    // Calculate the elapsed time in milliseconds
    double elapsed_time = (double)(end - start) * 1000.0 / CLOCKS_PER_SEC;

    // Print the elapsed time
    printf("Elapsed time: %.2f ms\n", elapsed_time);

    // Close the data file
    fclose(data_file);

    // Plot the data using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up the plot
        fprintf(gnuplot, "set title 'Multiple Segments Plot'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");
        fprintf(gnuplot, "set xrange [-60:60]\n");  // Adjust ranges as needed
        fprintf(gnuplot, "set yrange [-60:60]\n");
        fprintf(gnuplot, "set zrange [-60:60]\n");
        fprintf(gnuplot, "set view 70, 20\n");  // Adjust view angle

        // Plot all datasets with titles
        fprintf(gnuplot, "splot 'plot.dat' index 0 using 1:2:3 with lines title 'Segment 1', "
                         "'plot.dat' index 1 using 1:2:3 with lines title 'Segment 2', "
                         "'plot.dat' index 2 using 1:2:3 with lines title 'Segment 3', "
                         "'plot.dat' index 3 using 1:2:3 with lines title 'segment 4'\n");

        // Close GNUplot
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Test a clothoid fillet algoritme, given 2 segments
 * and max deviation value G64 P[x].
 *
 * s = trim lenght for seg0, seg1.
 *
 */
static void test_line_arc_fit_g3_1(){

    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    // Start the timer
    clock_t start = clock();

    double corner[3]={-50,0,0};

    struct segment seg0;
    seg0.segment_type=LINE;
    double l0_start[3]={0,0,0};
    double l0_end[3]={-50,0,0};
    init_line(l0_start,l0_end,&seg0,1);
    // print_segment(&seg0);

    struct segment seg1;
    seg1.segment_type=ARC;
    double l1_p0[3]={-50,0,0};
    double l1_p1[3]={0,0,0};
    double l1_pc[3]={-25,0,0};
    double l1_pn[3]={0,0,1};
    int cw=0;
    int turns=0;
    init_arc(l1_p0,l1_p1,l1_pc,l1_pn,cw,turns,&seg1,1);
    // print_segment(&seg1);

    plot_line_file(&seg0,data_file);
    plot_arc_file(&seg1,data_file);

    struct segment seg2;
    seg2.segment_type=CLOTHOID;

    double max_deviation = 5;
    double s = 0;

    // Find the best fit trim length s.
    find_fit(&seg0,&seg1,&seg2,&max_deviation,&s);
    printf("segments trim length = %f \n",s);
    printf("max deviation = %f \n",max_deviation);

    // Modify segments.
    apply_fit(&seg0,&seg1,&seg2,s);

    // Check deviation for second time.
    double deviation_result;
    double d;
    double pi[3];
    algorithm1(corner,seg2,1e-6,pi,&d,&deviation_result);
    printf("deviation result = %f \n",deviation_result);

    // print_segment(&seg0);
    // print_segment(&seg1);
    // print_segment(&seg2);
    plot_clothoid_file(&seg2,data_file);

    struct segment seg3;
    seg3.segment_type=LINE;
    init_line(pi,corner,&seg3,0);
    plot_line_file(&seg3,data_file);

    // Stop the timer
    clock_t end = clock();

    // Calculate the elapsed time in milliseconds
    double elapsed_time = (double)(end - start) * 1000.0 / CLOCKS_PER_SEC;

    // Print the elapsed time
    printf("Elapsed time: %.2f ms\n", elapsed_time);

    // Close the data file
    fclose(data_file);

    // Plot the data using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up the plot
        fprintf(gnuplot, "set title 'Multiple Segments Plot'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");
        fprintf(gnuplot, "set xrange [-60:60]\n");  // Adjust ranges as needed
        fprintf(gnuplot, "set yrange [-60:60]\n");
        fprintf(gnuplot, "set zrange [-60:60]\n");
        fprintf(gnuplot, "set view 70, 20\n");  // Adjust view angle

        // Plot all datasets with titles
        fprintf(gnuplot, "splot 'plot.dat' index 0 using 1:2:3 with lines title 'Segment 1', "
                         "'plot.dat' index 1 using 1:2:3 with lines title 'Segment 2', "
                         "'plot.dat' index 2 using 1:2:3 with lines title 'Segment 3', "
                         "'plot.dat' index 3 using 1:2:3 with lines title 'segment 4'\n");

        // Close GNUplot
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Test a clothoid fillet algoritme, given 2 segments
 * and max deviation value G64 P[x].
 *
 * s = trim lenght for seg0, seg1.
 *
 */
static void test_line_arc_fit_g3_2(){

    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    // Start the timer
    clock_t start = clock();

    double corner[3]={0,-50,0};

    struct segment seg0;
    seg0.segment_type=LINE;
    double l0_start[3]={0,0,0};
    double l0_end[3]={0,-50,0};
    init_line(l0_start,l0_end,&seg0,1);
    // print_segment(&seg0);

    struct segment seg1;
    seg1.segment_type=ARC;
    double l1_p0[3]={0,-50,0};
    double l1_p1[3]={0,0,0};
    double l1_pc[3]={5,-25,0};
    double l1_pn[3]={0,0,1};
    int cw=0;
    int turns=0;
    init_arc(l1_p0,l1_p1,l1_pc,l1_pn,cw,turns,&seg1,1);
    // print_segment(&seg1);

    plot_line_file(&seg0,data_file);
    plot_arc_file(&seg1,data_file);

    struct segment seg2;
    seg2.segment_type=CLOTHOID;

    double max_deviation = 5;
    double s = 0;

    // Find the best fit trim length s.
    find_fit(&seg0,&seg1,&seg2,&max_deviation,&s);
    printf("segments trim length = %f \n",s);
    printf("max deviation = %f \n",max_deviation);

    // Modify segments.
    apply_fit(&seg0,&seg1,&seg2,s);

    // Check deviation for second time.
    double deviation_result;
    double d;
    double pi[3];
    algorithm1(corner,seg2,1e-6,pi,&d,&deviation_result);
    printf("deviation result = %f \n",deviation_result);

    // print_segment(&seg0);
    // print_segment(&seg1);
    // print_segment(&seg2);
    plot_clothoid_file(&seg2,data_file);

    struct segment seg3;
    seg3.segment_type=LINE;
    init_line(pi,corner,&seg3,0);
    plot_line_file(&seg3,data_file);

    // Stop the timer
    clock_t end = clock();

    // Calculate the elapsed time in milliseconds
    double elapsed_time = (double)(end - start) * 1000.0 / CLOCKS_PER_SEC;

    // Print the elapsed time
    printf("Elapsed time: %.2f ms\n", elapsed_time);

    // Close the data file
    fclose(data_file);

    // Plot the data using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up the plot
        fprintf(gnuplot, "set title 'Multiple Segments Plot'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");
        fprintf(gnuplot, "set xrange [-60:50]\n");  // Adjust ranges as needed
        fprintf(gnuplot, "set yrange [-60:50]\n");
        fprintf(gnuplot, "set zrange [-60:50]\n");
        fprintf(gnuplot, "set view 70, 20\n");  // Adjust view angle

        // Plot all datasets with titles
        fprintf(gnuplot, "splot 'plot.dat' index 0 using 1:2:3 with lines title 'Segment 1', "
                         "'plot.dat' index 1 using 1:2:3 with lines title 'Segment 2', "
                         "'plot.dat' index 2 using 1:2:3 with lines title 'Segment 3', "
                         "'plot.dat' index 3 using 1:2:3 with lines title 'segment 4'\n");

        // Close GNUplot
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Test a clothoid fillet algoritme, given 2 segments
 * and max deviation value G64 P[x].
 *
 * s = trim lenght for seg0, seg1.
 *
 */
static void test_line_arc_fit_g3_3(){

    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    // Start the timer
    clock_t start = clock();

    double corner[3]={50,0,0};

    struct segment seg0;
    seg0.segment_type=LINE;
    double l0_start[3]={0,0,0};
    double l0_end[3]={50,0,0};
    init_line(l0_start,l0_end,&seg0,1);
    // print_segment(&seg0);

    struct segment seg1;
    seg1.segment_type=ARC;
    double l1_p0[3]={50,0,0};
    double l1_p1[3]={0,0,0};
    double l1_pc[3]={25,0,0};
    double l1_pn[3]={0,0,1};
    int cw=0;
    int turns=0;
    init_arc(l1_p0,l1_p1,l1_pc,l1_pn,cw,turns,&seg1,1);
    // print_segment(&seg1);

    plot_line_file(&seg0,data_file);
    plot_arc_file(&seg1,data_file);

    struct segment seg2;
    seg2.segment_type=CLOTHOID;

    double max_deviation = 5;
    double s = 0;

    // Find the best fit trim length s.
    find_fit(&seg0,&seg1,&seg2,&max_deviation,&s);
    printf("segments trim length = %f \n",s);
    printf("max deviation = %f \n",max_deviation);

    // Modify segments.
    apply_fit(&seg0,&seg1,&seg2,s);

    // Check deviation for second time.
    double deviation_result;
    double d;
    double pi[3];
    algorithm1(corner,seg2,1e-6,pi,&d,&deviation_result);
    printf("deviation result = %f \n",deviation_result);

    // print_segment(&seg0);
    // print_segment(&seg1);
    // print_segment(&seg2);
    plot_clothoid_file(&seg2,data_file);

    struct segment seg3;
    seg3.segment_type=LINE;
    init_line(pi,corner,&seg3,0);
    plot_line_file(&seg3,data_file);

    // Stop the timer
    clock_t end = clock();

    // Calculate the elapsed time in milliseconds
    double elapsed_time = (double)(end - start) * 1000.0 / CLOCKS_PER_SEC;

    // Print the elapsed time
    printf("Elapsed time: %.2f ms\n", elapsed_time);

    // Close the data file
    fclose(data_file);

    // Plot the data using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up the plot
        fprintf(gnuplot, "set title 'Multiple Segments Plot'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");
        fprintf(gnuplot, "set xrange [-60:50]\n");  // Adjust ranges as needed
        fprintf(gnuplot, "set yrange [-60:50]\n");
        fprintf(gnuplot, "set zrange [-60:50]\n");
        fprintf(gnuplot, "set view 70, 20\n");  // Adjust view angle

        // Plot all datasets with titles
        fprintf(gnuplot, "splot 'plot.dat' index 0 using 1:2:3 with lines title 'Segment 1', "
                         "'plot.dat' index 1 using 1:2:3 with lines title 'Segment 2', "
                         "'plot.dat' index 2 using 1:2:3 with lines title 'Segment 3', "
                         "'plot.dat' index 3 using 1:2:3 with lines title 'segment 4'\n");

        // Close GNUplot
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Test a clothoid fillet algoritme, given 2 segments
 * and max deviation value G64 P[x].
 *
 * s = trim lenght for seg0, seg1.
 *
 */
static void test_line_arc_fit_g2_0(){

    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    // Start the timer
    clock_t start = clock();

    double corner[3]={50,0,0};

    struct segment seg0;
    seg0.segment_type=LINE;
    double l0_start[3]={0,0,0};
    double l0_end[3]={50,0,0};
    init_line(l0_start,l0_end,&seg0,1);
    // print_segment(&seg0);

    struct segment seg1;
    seg1.segment_type=ARC;
    double l1_p0[3]={50,0,0};
    double l1_p1[3]={0,0,0};
    double l1_pc[3]={25,0,0};
    double l1_pn[3]={0,0,1};
    int cw=1;
    int turns=0;
    init_arc(l1_p0,l1_p1,l1_pc,l1_pn,cw,turns,&seg1,1);
    // print_segment(&seg1);

    plot_line_file(&seg0,data_file);
    plot_arc_file(&seg1,data_file);

    struct segment seg2;
    seg2.segment_type=CLOTHOID;

    double max_deviation = 5;
    double s = 0;

    // Find the best fit trim length s.
    find_fit(&seg0,&seg1,&seg2,&max_deviation,&s);
    printf("segments trim length = %f \n",s);
    printf("max deviation = %f \n",max_deviation);

    // Modify segments.
    apply_fit(&seg0,&seg1,&seg2,s);

    // Check deviation for second time.
    double deviation_result;
    double d;
    double pi[3];
    algorithm1(corner,seg2,1e-6,pi,&d,&deviation_result);
    printf("deviation result = %f \n",deviation_result);

    // print_segment(&seg0);
    // print_segment(&seg1);
    // print_segment(&seg2);
    plot_clothoid_file(&seg2,data_file);

    struct segment seg3;
    seg3.segment_type=LINE;
    init_line(pi,corner,&seg3,0);
    plot_line_file(&seg3,data_file);

    // Stop the timer
    clock_t end = clock();

    // Calculate the elapsed time in milliseconds
    double elapsed_time = (double)(end - start) * 1000.0 / CLOCKS_PER_SEC;

    // Print the elapsed time
    printf("Elapsed time: %.2f ms\n", elapsed_time);

    // Close the data file
    fclose(data_file);

    // Plot the data using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up the plot
        fprintf(gnuplot, "set title 'Multiple Segments Plot'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");
        fprintf(gnuplot, "set xrange [-50:50]\n");  // Adjust ranges as needed
        fprintf(gnuplot, "set yrange [-50:50]\n");
        fprintf(gnuplot, "set zrange [-50:50]\n");
        fprintf(gnuplot, "set view 0, 20\n");  // Adjust view angle

        // Plot all datasets with titles
        fprintf(gnuplot, "splot 'plot.dat' index 0 using 1:2:3 with lines title 'Segment 1', "
                         "'plot.dat' index 1 using 1:2:3 with lines title 'Segment 2', "
                         "'plot.dat' index 2 using 1:2:3 with lines title 'Segment 3', "
                         "'plot.dat' index 3 using 1:2:3 with lines title 'segment 4'\n");

        // Close GNUplot
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Test a clothoid fillet algoritme, given 2 segments
 * and max deviation value G64 P[x].
 *
 * s = trim lenght for seg0, seg1.
 *
 */
static void test_line_arc_fit_g2_1(){

    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    // Start the timer
    clock_t start = clock();

    double corner[3]={-50,0,0};

    struct segment seg0;
    seg0.segment_type=LINE;
    double l0_start[3]={0,0,0};
    double l0_end[3]={-50,0,0};
    init_line(l0_start,l0_end,&seg0,1);
    // print_segment(&seg0);

    struct segment seg1;
    seg1.segment_type=ARC;
    double l1_p0[3]={-50,0,0};
    double l1_p1[3]={0,0,0};
    double l1_pc[3]={-25,0,0};
    double l1_pn[3]={0,0,1};
    int cw=1;
    int turns=0;
    init_arc(l1_p0,l1_p1,l1_pc,l1_pn,cw,turns,&seg1,1);
    // print_segment(&seg1);

    plot_line_file(&seg0,data_file);
    plot_arc_file(&seg1,data_file);

    struct segment seg2;
    seg2.segment_type=CLOTHOID;

    double max_deviation = 5;
    double s = 0;

    // Find the best fit trim length s.
    find_fit(&seg0,&seg1,&seg2,&max_deviation,&s);
    printf("segments trim length = %f \n",s);
    printf("max deviation = %f \n",max_deviation);

    // Modify segments.
    apply_fit(&seg0,&seg1,&seg2,s);

    // Check deviation for second time.
    double deviation_result;
    double d;
    double pi[3];
    algorithm1(corner,seg2,1e-6,pi,&d,&deviation_result);
    printf("deviation result = %f \n",deviation_result);

    // print_segment(&seg0);
    // print_segment(&seg1);
    // print_segment(&seg2);
    plot_clothoid_file(&seg2,data_file);

    struct segment seg3;
    seg3.segment_type=LINE;
    init_line(pi,corner,&seg3,0);
    plot_line_file(&seg3,data_file);

    // Stop the timer
    clock_t end = clock();

    // Calculate the elapsed time in milliseconds
    double elapsed_time = (double)(end - start) * 1000.0 / CLOCKS_PER_SEC;

    // Print the elapsed time
    printf("Elapsed time: %.2f ms\n", elapsed_time);

    // Close the data file
    fclose(data_file);

    // Plot the data using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up the plot
        fprintf(gnuplot, "set title 'Multiple Segments Plot'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");
        fprintf(gnuplot, "set xrange [-60:60]\n");  // Adjust ranges as needed
        fprintf(gnuplot, "set yrange [-60:60]\n");
        fprintf(gnuplot, "set zrange [-60:60]\n");
        fprintf(gnuplot, "set view 70, 20\n");  // Adjust view angle

        // Plot all datasets with titles
        fprintf(gnuplot, "splot 'plot.dat' index 0 using 1:2:3 with lines title 'Segment 1', "
                         "'plot.dat' index 1 using 1:2:3 with lines title 'Segment 2', "
                         "'plot.dat' index 2 using 1:2:3 with lines title 'Segment 3', "
                         "'plot.dat' index 3 using 1:2:3 with lines title 'segment 4'\n");

        // Close GNUplot
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Test a clothoid fillet algoritme, given 2 segments
 * and max deviation value G64 P[x].
 *
 * s = trim lenght for seg0, seg1.
 *
 */
static void test_line_arc_fit_g2_2(){

    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    // Start the timer
    clock_t start = clock();

    double corner[3]={0,-50,0};

    struct segment seg0;
    seg0.segment_type=LINE;
    double l0_start[3]={0,0,0};
    double l0_end[3]={0,-50,0};
    init_line(l0_start,l0_end,&seg0,1);
    // print_segment(&seg0);

    struct segment seg1;
    seg1.segment_type=ARC;
    double l1_p0[3]={0,-50,0};
    double l1_p1[3]={0,0,0};
    double l1_pc[3]={0,-25,0};
    double l1_pn[3]={0,0,1};
    int cw=1;
    int turns=0;
    init_arc(l1_p0,l1_p1,l1_pc,l1_pn,cw,turns,&seg1,1);
    // print_segment(&seg1);

    plot_line_file(&seg0,data_file);
    plot_arc_file(&seg1,data_file);

    struct segment seg2;
    seg2.segment_type=CLOTHOID;

    double max_deviation = 5;
    double s = 0;

    // Find the best fit trim length s.
    find_fit(&seg0,&seg1,&seg2,&max_deviation,&s);
    printf("segments trim length = %f \n",s);
    printf("max deviation = %f \n",max_deviation);

    // Modify segments.
    apply_fit(&seg0,&seg1,&seg2,s);

    // Check deviation for second time.
    double deviation_result;
    double d;
    double pi[3];
    algorithm1(corner,seg2,1e-6,pi,&d,&deviation_result);
    printf("deviation result = %f \n",deviation_result);

    // print_segment(&seg0);
    // print_segment(&seg1);
    // print_segment(&seg2);
    plot_clothoid_file(&seg2,data_file);

    struct segment seg3;
    seg3.segment_type=LINE;
    init_line(pi,corner,&seg3,0);
    plot_line_file(&seg3,data_file);

    // Stop the timer
    clock_t end = clock();

    // Calculate the elapsed time in milliseconds
    double elapsed_time = (double)(end - start) * 1000.0 / CLOCKS_PER_SEC;

    // Print the elapsed time
    printf("Elapsed time: %.2f ms\n", elapsed_time);

    // Close the data file
    fclose(data_file);

    // Plot the data using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up the plot
        fprintf(gnuplot, "set title 'Multiple Segments Plot'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");
        fprintf(gnuplot, "set xrange [-60:50]\n");  // Adjust ranges as needed
        fprintf(gnuplot, "set yrange [-60:50]\n");
        fprintf(gnuplot, "set zrange [-60:50]\n");
        fprintf(gnuplot, "set view 70, 20\n");  // Adjust view angle

        // Plot all datasets with titles
        fprintf(gnuplot, "splot 'plot.dat' index 0 using 1:2:3 with lines title 'Segment 1', "
                         "'plot.dat' index 1 using 1:2:3 with lines title 'Segment 2', "
                         "'plot.dat' index 2 using 1:2:3 with lines title 'Segment 3', "
                         "'plot.dat' index 3 using 1:2:3 with lines title 'segment 4'\n");

        // Close GNUplot
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Test a clothoid fillet algoritme, given 2 segments
 * and max deviation value G64 P[x].
 *
 * s = trim lenght for seg0, seg1.
 *
 */
static void test_line_arc_fit_g2_3(){

    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    // Start the timer
    clock_t start = clock();

    double corner[3]={50,0,0};

    struct segment seg0;
    seg0.segment_type=LINE;
    double l0_start[3]={0,0,0};
    double l0_end[3]={50,0,0};
    init_line(l0_start,l0_end,&seg0,1);
    // print_segment(&seg0);

    struct segment seg1;
    seg1.segment_type=ARC;
    double l1_p0[3]={50,0,0};
    double l1_p1[3]={0,0,0};
    double l1_pc[3]={25,0,0};
    double l1_pn[3]={0,0,1};
    int cw=1;
    int turns=0;
    init_arc(l1_p0,l1_p1,l1_pc,l1_pn,cw,turns,&seg1,1);
    // print_segment(&seg1);

    plot_line_file(&seg0,data_file);
    plot_arc_file(&seg1,data_file);

    struct segment seg2;
    seg2.segment_type=CLOTHOID;

    double max_deviation = 5;
    double s = 0;

    // Find the best fit trim length s.
    find_fit(&seg0,&seg1,&seg2,&max_deviation,&s);
    printf("segments trim length = %f \n",s);
    printf("max deviation = %f \n",max_deviation);

    // Modify segments.
    apply_fit(&seg0,&seg1,&seg2,s);

    // Check deviation for second time.
    double deviation_result;
    double d;
    double pi[3];
    algorithm1(corner,seg2,1e-6,pi,&d,&deviation_result);
    printf("deviation result = %f \n",deviation_result);

    // print_segment(&seg0);
    // print_segment(&seg1);
    // print_segment(&seg2);
    plot_clothoid_file(&seg2,data_file);

    struct segment seg3;
    seg3.segment_type=LINE;
    init_line(pi,corner,&seg3,0);
    plot_line_file(&seg3,data_file);

    // Stop the timer
    clock_t end = clock();

    // Calculate the elapsed time in milliseconds
    double elapsed_time = (double)(end - start) * 1000.0 / CLOCKS_PER_SEC;

    // Print the elapsed time
    printf("Elapsed time: %.2f ms\n", elapsed_time);

    // Close the data file
    fclose(data_file);

    // Plot the data using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up the plot
        fprintf(gnuplot, "set title 'Multiple Segments Plot'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");
        fprintf(gnuplot, "set xrange [-60:50]\n");  // Adjust ranges as needed
        fprintf(gnuplot, "set yrange [-60:50]\n");
        fprintf(gnuplot, "set zrange [-60:50]\n");
        fprintf(gnuplot, "set view 70, 20\n");  // Adjust view angle

        // Plot all datasets with titles
        fprintf(gnuplot, "splot 'plot.dat' index 0 using 1:2:3 with lines title 'Segment 1', "
                         "'plot.dat' index 1 using 1:2:3 with lines title 'Segment 2', "
                         "'plot.dat' index 2 using 1:2:3 with lines title 'Segment 3', "
                         "'plot.dat' index 3 using 1:2:3 with lines title 'segment 4'\n");

        // Close GNUplot
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Test a clothoid fillet algoritme, given 2 segments
 * and max deviation value G64 P[x].
 *
 * s = trim lenght for seg0, seg1.
 *
 */
static void test_arc_arc_fit(){

    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    // Start the timer
    clock_t start = clock();

    double corner[3]={0,0,0};

    struct segment seg0;
    seg0.segment_type=ARC;
    double l0_p0[3]={-50,0,0};
    double l0_p1[3]={0,0,0};
    double l0_pc[3]={-25,0,0};
    double l0_pn[3]={0,0,1};
    int cw0=1;
    int turns0=0;
    init_arc(l0_p0,l0_p1,l0_pc,l0_pn,cw0,turns0,&seg0,1);
    // print_segment(&seg0);

    struct segment seg1;
    seg1.segment_type=ARC;
    double l1_p0[3]={0,0,0};
    double l1_p1[3]={0,50,0};
    double l1_pc[3]={0,25,0};
    double l1_pn[3]={0,0,1};
    int cw1=0;
    int turns1=0;
    init_arc(l1_p0,l1_p1,l1_pc,l1_pn,cw1,turns1,&seg1,1);
    // print_segment(&seg1);

    plot_arc_file(&seg0,data_file);
    plot_arc_file(&seg1,data_file);

    struct segment seg2;
    seg2.segment_type=CLOTHOID;

    double max_deviation = 5;
    double s = 0;

    // Find the best fit trim length s.
    find_fit(&seg0,&seg1,&seg2,&max_deviation,&s);
    printf("segments trim length = %f \n",s);
    printf("max deviation = %f \n",max_deviation);

    // Modify segments.
    apply_fit(&seg0,&seg1,&seg2,s);

    // Check deviation for second time.
    double deviation_result;
    double d;
    double pi[3];
    algorithm1(corner,seg2,1e-6,pi,&d,&deviation_result);
    printf("deviation result = %f \n",deviation_result);

    // print_segment(&seg0);
    // print_segment(&seg1);
    // print_segment(&seg2);
    plot_clothoid_file(&seg2,data_file);

    struct segment seg3;
    seg3.segment_type=LINE;
    init_line(pi,corner,&seg3,0);
    plot_line_file(&seg3,data_file);

    // Stop the timer
    clock_t end = clock();

    // Calculate the elapsed time in milliseconds
    double elapsed_time = (double)(end - start) * 1000.0 / CLOCKS_PER_SEC;

    // Print the elapsed time
    printf("Elapsed time: %.2f ms\n", elapsed_time);

    // Close the data file
    fclose(data_file);

    // Plot the data using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up the plot
        fprintf(gnuplot, "set title 'Multiple Segments Plot'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");
        fprintf(gnuplot, "set xrange [-50:50]\n");  // Adjust ranges as needed
        fprintf(gnuplot, "set yrange [-50:50]\n");
        fprintf(gnuplot, "set zrange [-50:50]\n");
        fprintf(gnuplot, "set view 0, 25\n");  // Adjust view angle

        // Plot all datasets with titles
        fprintf(gnuplot, "splot 'plot.dat' index 0 using 1:2:3 with lines title 'Segment 1', "
                         "'plot.dat' index 1 using 1:2:3 with lines title 'Segment 2', "
                         "'plot.dat' index 2 using 1:2:3 with lines title 'Segment 3', "
                         "'plot.dat' index 3 using 1:2:3 with lines title 'segment 4'\n");

        // Close GNUplot
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

#endif // FIT_H
