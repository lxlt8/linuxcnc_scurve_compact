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
#include "scurve.h"
#include "stdio.h"
#include <stdlib.h> // For system()
#include <time.h>   // For clock() and clock_t

/* Example scurve implementation using gnu plot to view result in graph.
 *
 */
int main() {

    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return 0;
    }

    struct scurve_data scd;
    enum scurve_return_code ret;

    double max_jerk         = 100;
    double max_acceleration = 20;
    double max_velocity     = 120;
    double cycletime        = 0.001;

    double endvel           = 0;
    double endacc           = 0;
    double endpos           = 100;
    int pausing             = 0;

    double time             = 0;

    scurve_init(&scd,
                max_jerk,
                max_acceleration,
                max_velocity,
                cycletime);

    while(1){

        // Start time measurement
        clock_t start = clock();

        scurve_set_target_state(&scd,
                                endvel,
                                endacc,
                                endpos,
                                pausing);

        ret = scurve_update(&scd);

        // End time measurement
        clock_t end = clock();

        // Calculate duration in milliseconds
        double duration_ms = ((double)(end - start)) * 1000.0 / CLOCKS_PER_SEC;

        // Calculate duration in microseconds
        double duration_us = ((double)(end - start)) * 1000000.0 / CLOCKS_PER_SEC;

        // Write data to the file
        fprintf(data_file, "%f %f %f %f\n", time, scd.curpos, scd.curvel, scd.curacc);

        printf("time: %f, pos: %f, vel: %f, acc: %f duration ms: %f duration us: %f \n",
               time, scd.curpos, scd.curvel, scd.curacc, duration_ms, duration_us);

        time += cycletime;

        if(ret == RETURN_FINISHED){
            break;
        }
    }

    // Close the data file
    fclose(data_file);

    // Plot the data using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up the plot
        fprintf(gnuplot, "set title 'S-Curve Trajectory: Position, Velocity, and Acceleration'\n");
        fprintf(gnuplot, "set xlabel 'Time (s)'\n");
        fprintf(gnuplot, "set ylabel 'Position'\n");
        fprintf(gnuplot, "set y2label 'Velocity, Acceleration'\n");
        fprintf(gnuplot, "set grid\n");
        fprintf(gnuplot, "set ytics nomirror\n"); // Separate y-axis for position
        fprintf(gnuplot, "set y2tics\n");         // Enable second y-axis for velocity and acceleration

        // Plot the data
        fprintf(gnuplot, "plot 'plot.dat' using 1:2 with lines title 'Position' axes x1y1, "
                         "'plot.dat' using 1:3 with lines title 'Velocity' axes x1y2, "
                         "'plot.dat' using 1:4 with lines title 'Acceleration' axes x1y2\n");

        // Close GNUplot
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }

    return 0;
}
