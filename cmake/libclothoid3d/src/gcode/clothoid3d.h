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
#ifndef CLOTHOID3D_H
#define CLOTHOID3D_H

#include "../segment.h"
#include "../eq44_G0.h"
#include "line3d.h"
#include "arc3d.h"
#include "../eq14.h"
#include "../math/geometry_math.h"

/* Initialize a clothoid.
 *
 * Already known values:
 *
 * p0, p1.
 * s1,y11,y21.
 * theta 10,14,20,24.
 * kappa 10,14,20,24.
 * sharpness 10,14,20,24.
 */
static void init_clothoid(struct segment *seg){

    // Choose solver, set limits.
    seg->solver_enum = CERES_CENTRAL_DIFF;
    seg->s1_lower_limit = -INFINITY;
    seg->s1_upper_limit = INFINITY;

    // Solve clothoid compound.
    eq44_G0(seg);

    // Update junction parameters.
    eq44_G123_compact(seg);

    // Update clothoid intermediate points.
    eq44_intermediate_points(seg);
}

/* Initialize a clothoid between 2 segments.
 */
static void init_inbetween_clothoid(const struct segment *seg0,
                                    const struct segment *seg1,
                                    struct segment *seg2,
                                    double s1_lower_limit,
                                    double s1_upper_limit){

    copy_vector(seg0->p1, seg2->p0);
    copy_vector(seg1->p0, seg2->p1);

    // Guess values, 3 unknowns.
    seg2->s1 = 0.0001;   // Length of individual clothoid.
    seg2->y11 = 0.0000;  // Z rigid factor.
    seg2->y21 = 0.0000;  // XY rigid factor.

    // Initial values at z clothoid start.
    seg2->theta10 = seg0->theta14;
    seg2->kappa10 = seg0->kappa14;
    seg2->sharpness10=0;// seg0->sharpness14;
    // Initial values at z clothoid end.
    seg2->theta14 = seg1->theta10;
    seg2->kappa14 = seg1->kappa10;
    seg2->sharpness14=0;// = seg1->sharpness10;

    // Initial values at xy spline start.
    seg2->theta20 = seg0->theta24;
    seg2->kappa20 = seg0->kappa24;
    seg2->sharpness20=0;// = seg0->sharpness24;
    // Initial values at xy spline end.
    seg2->theta24 = seg1->theta20;
    seg2->kappa24 = seg1->kappa20;
    seg2->sharpness24=0;// = seg1->sharpness20;

    // Choose solver, set limits.
    seg2->solver_enum = CERES_CENTRAL_DIFF;
    seg2->s1_lower_limit = s1_lower_limit;
    seg2->s1_upper_limit = s1_upper_limit;

    // Solve clothoid compound. Using ceres solver.
    eq44_G0(seg2);

    // Update junction parameters.
    eq44_G123_compact(seg2);

    // Update clothoid intermediate points.
    eq44_intermediate_points(seg2);
}

/* Check self intersection.
 */
static int self_intersecting_clothoid(struct segment *seg){

    int segments = 100;
    double vector[segments][3];

    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);
        double pi[3] = {0, 0, 0};
        double s = (seg->s1*4) * progress;

        eq14_interpolate_clothoid(seg, s, pi);

        vector[i][0]=pi[0];
        vector[i][1]=pi[1];
        vector[i][2]=pi[2];
    }

    for (int i=0; i<segments-1; i++){
        for (int j=i+2; j<segments-1; j++){
            if(intersect(vector[i],vector[i+1], vector[j], vector[j+1]) && j!=i){
                return 1;
            }
        }
    }
    return 0;
}


/* Plot to file.
 */
static void plot_clothoid_file(struct segment *seg, FILE *data_file){

    int segments = 100;
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);
        double pi[3] = {0, 0, 0};
        double s = (seg->s1*4) * progress;

        eq14_interpolate_clothoid(seg, s, pi);
        fprintf(data_file, "%lf %lf %lf\n", pi[0], pi[1], pi[2]);
    }
    fprintf(data_file, "\n\n");  // Add two blank lines to separate datasets
}

/* Plot clothoid
 */
static void plot_clothoid(struct segment *seg){

    // Print to file for visualization.
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    int segments = 100;
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);
        double pi[3] = {0, 0, 0};
        double s = seg->s1 * 4 * progress;

        eq14_interpolate_clothoid(seg, s, pi);

        fprintf(data_file, "%lf %lf %lf\n", pi[0], pi[1], pi[2]);
    }

    fflush(data_file);
    fclose(data_file);

    double min = 0;
    double max = 100;

    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up 3D plotting
        fprintf(gnuplot, "set title 'plot'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");

        fprintf(gnuplot, "set xrange [%f:%f]\n", min, max); // Use %f to format the double value
        fprintf(gnuplot, "set yrange [%f:%f]\n", min, max); // Use %f to format the double value
        fprintf(gnuplot, "set zrange [%f:%f]\n", min, max); // Use %f to format the double value

        // Set view angle for better 3D visualization
        fprintf(gnuplot, "set view 70, 20\n");  // You can adjust the angle here

        // Plot the data in 3D (use correct filename)
        fprintf(gnuplot, "splot 'plot.dat' using 1:2:3 with lines\n");

        fflush(gnuplot);
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }

}

/* Interpolate a clothoid
 */
static void interpolate_clothoid3d(const struct segment *seg,
                                 const double progress,
                                 double pi[3]){

    double stot = seg->s1*4;
    double s = progress * stot;
    eq14_interpolate_clothoid(seg,s,pi);
}

static void test_clothoid_line_line(){

    struct segment seg0;
    double l0_start[3]={-2,0,0};
    double l0_end[3]={0,0,0};
    init_line(l0_start,l0_end,&seg0,1);
    print_segment(&seg0);

    struct segment seg1;
    double l1_start[3]={0.8,1.6,0};
    double l1_end[3]={2.8,1.6,0};
    init_line(l1_start,l1_end,&seg1,1);
    print_segment(&seg1);

    struct segment seg2;

    // Start point.
    seg2.p0[0] = l0_end[0];
    seg2.p0[1] = l0_end[1];
    seg2.p0[2] = l0_end[2];

    // Target point.
    seg2.p1[0] = l1_start[0];
    seg2.p1[1] = l1_start[1];
    seg2.p1[2] = l1_start[2];

    // Guess values, 3 unknowns.
    seg2.s1 = 0.0001;  // Length of individual clothoid.
    seg2.y11 = 0.0000;  // Z rigid factor.
    seg2.y21 = 0.0000;  // XY rigid factor.

    // Initial values at z spline start.
    seg2.theta10 = seg0.theta14;
    seg2.kappa10 = 0;
    seg2.sharpness10 = 0;
    // Initial values at z spline end.
    seg2.theta14 = seg1.theta10;
    seg2.kappa14 = 0;
    seg2.sharpness14 = 0;

    // Initial values at xy spline start.
    seg2.theta20 = seg0.theta24;
    seg2.kappa20 = 0;
    seg2.sharpness20 = 0;
    // Initial values at xy spline end.
    seg2.theta24 = seg1.theta20;
    seg2.kappa24 = 0;
    seg2.sharpness24 = 0;

    seg2.solver_enum = CERES_CENTRAL_DIFF;
    seg2.s1_lower_limit = -INFINITY;
    seg2.s1_upper_limit = INFINITY;

    // Solve clothoid compound.
    eq44_G0(&seg2);

    // Update junction parameters.
    eq44_G123_compact(&seg2);

    // Update clothoid intermediate points.
    eq44_intermediate_points(&seg2);

    FILE *start_file = fopen("clothoid_start.dat", "w");
    FILE *clothoid_file = fopen("clothoid_curve.dat", "w");
    FILE *end_file = fopen("clothoid_end.dat", "w");

    if (!start_file || !clothoid_file || !end_file) {
        printf("Error opening file!\n");
        return;
    }

    // Red Line (Start)
    fprintf(start_file, "%lf %lf %lf\n", l0_start[0], l0_start[1], l0_start[2]);
    fprintf(start_file, "%lf %lf %lf\n", l0_end[0], l0_end[1], l0_end[2]);

    // Blue Clothoid Curve
    int segments = 100;
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);
        double pi[3] = {0, 0, 0};
        double s = seg2.s1 * 4 * progress;

        eq14_interpolate_clothoid(&seg2, s, pi);

        fprintf(clothoid_file, "%lf %lf %lf\n", pi[0], pi[1], pi[2]);
    }

    // Green Line (End)
    fprintf(end_file, "%lf %lf %lf\n", l1_start[0], l1_start[1], l1_start[2]);
    fprintf(end_file, "%lf %lf %lf\n", l1_end[0], l1_end[1], l1_end[2]);

    fclose(start_file);
    fclose(clothoid_file);
    fclose(end_file);

    // Open GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        fprintf(gnuplot, "set title 'Clothoid 3D Trajectory'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");

        fprintf(gnuplot, "set xrange [-5:5]\n");
        fprintf(gnuplot, "set yrange [-5:5]\n");
        fprintf(gnuplot, "set zrange [-5:5]\n");

        fprintf(gnuplot, "set view 70, 20\n");

        // Plot with both points and lines for start and end
        fprintf(gnuplot,
                "splot 'clothoid_start.dat' using 1:2:3 with lines lc rgb 'orange' lw 2 title 'Start',"
                " 'clothoid_curve.dat' using 1:2:3 with lines lc rgb 'blue' lw 2 title 'Clothoid',"
                " 'clothoid_end.dat' using 1:2:3 with lines lc rgb 'orange' lw 2 title 'End'\n"
                );

        // Keep window open
        fprintf(gnuplot, "pause -1\n");

        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");

    }
}

static void test_clothoid_helix_helix(){
    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    // Arc helix.
    struct segment seg0;
    double p00[3] = {0,0,-100};
    double pc0[3] = {0,50,-100};
    double p10[3] = {0,100,0};
    double pn0[3] = {0,0,1};
    int turns0 = 2;
    int cw0 = 1;
    init_arc(p00,p10,pc0,pn0,cw0,turns0,&seg0,0);
    plot_arc_file(&seg0, data_file);
    print_segment(&seg0);

    // Arc helix.
    struct segment seg1;
    double p0[3] = {100,80,0};
    double pc[3] = {100,50,0};
    double p1[3] = {100,20,-100};
    double pn[3] = {0,0,1};
    int turns = 2;
    int cw = 1;

    init_arc(p0,p1,pc,pn,cw,turns,&seg1,0);
    plot_arc_file(&seg1, data_file);
    print_segment(&seg1);

    struct segment seg2;
    init_inbetween_clothoid(&seg0, &seg1, &seg2,-INFINITY,INFINITY);
    plot_clothoid_file(&seg2, data_file);
    print_clothoid_segment(&seg2);

    struct segment seg3;
    init_inbetween_clothoid(&seg1, &seg0, &seg3,-INFINITY,INFINITY);
    plot_clothoid_file(&seg3, data_file);
    print_clothoid_segment(&seg3);

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
        fprintf(gnuplot, "set xrange [-180:180]\n");  // Adjust ranges as needed
        fprintf(gnuplot, "set yrange [-180:180]\n");
        fprintf(gnuplot, "set zrange [-180:180]\n");
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

static void test_clothoid_line_line_remove_extra_torsion_turn(){
    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    // Line
    struct segment seg0;
    double p00[3] = {0,0.00,0};
    double p10[3] = {0,0,50};
    init_line(p00,p10,&seg0,1);
    plot_line_file(&seg0, data_file);
    // print_segment(&seg0);

    // Line
    struct segment seg1;
    double p0[3] = {-10,0,60};
    double p1[3] = {-50,0,60};

    init_line(p0,p1,&seg1,1);
    plot_line_file(&seg1, data_file);
    // print_segment(&seg1);

    struct segment seg2;
    init_inbetween_clothoid(&seg0, &seg1, &seg2,-INFINITY,INFINITY);
    plot_clothoid_file(&seg2, data_file);
    print_clothoid_segment(&seg2);

    // Check for extra torsion turns.
    if(eq45_extra_torsion_turns(&seg2)==1){
        printf("registered extra torsion turns. clothoid fit's again. \n");
        if(seg0.segment_type==LINE){
            seg0.p1[0]+=0.001;
            seg0.p1[1]+=0.001;
            init_line(seg0.p0,seg0.p1,&seg0,1);
            init_inbetween_clothoid(&seg0, &seg1, &seg2,-INFINITY,INFINITY);
            plot_clothoid_file(&seg2, data_file);
            print_clothoid_segment(&seg2);
        }
    }

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

#endif // CLOTHOID3D_H














