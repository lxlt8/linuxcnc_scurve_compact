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
#ifndef VELOCITY_H
#define VELOCITY_H

#include <stdio.h>
#include <math.h>
#include "eq14.h"
#include "gauss_legendre.h"
#include <stdio.h>
#include <time.h>  // For clock() and CLOCKS_PER_SEC
#include "ceres_motion_cppapi.h"
#include "segment.h"

/* Acc = deltaV / deltaT.
 * deltaV = y axis.
 * deltaT = x axis.
 */
static void theta_to_acc(double theta, double *acc){

    double x = cos(theta);
    double y = sin(theta);
    *acc = y/x;
    // printf("acc from theta: %f \n",*acc);
}

/* Acc = deltaV / deltaT.
 * Acc = dy / dx
 */
static void acc_to_theta(double dx, double dy, double *theta){

    *theta = atan2(dy,dx);
    // printf("theta from dx,dy in radians: %f \n",*theta);
    // printf("theta from dx,dy in degrees: %f \n",*theta * toDegrees);
}

/* Trapezium velocity acceleration stage profile used to fit a clothoid curve.
 * This to retrieve the rate of change value "gamma" for the clothoid as
 * this specifies a fixed curve shape over time.
 *
 *      Acceleration (a) = maxacc (constant during acceleration phase).
 *      Velocity (v) = v₀ + a * t (starts from 0).
 *      Displacement (s) = s₀ + v₀ * t + 0.5 * a * t² (optional, if needed).
 *
 * dv = Delta velocity. Is
 */
static void initialise_null_frame(double a, double dv, double *t){

    *t = dv/a;                          // v=a*t -> t=v/a
    double s = 0.5 * a * (*t) * (*t);   // s=0.5*a*t²

    // This includes a 4 clothoid fit.
    double x0 = 0;
    double y0 = 0;
    double z0 = 0;

    double x4 = x0 + *t;
    double y4 = y0 + dv;

    double x2 = 10;//(x0 + x4) / 2;
    double y2 = 4; //(y0 + y4) / 2;

    double theta20=0;
    double kappa20=0;
    double sharpness20=0;

    double y21 = 0.0000;
    double s1 = 0.0001;
    double kappa = 0;

    printf("t = %f \n", *t);
    printf("dv = %f \n", dv);
    printf("s = %f \n", s);

    init_ceres_motion_cpp(theta20,
                          kappa20,
                          sharpness20,
                          x0,y0, x2,y2,
                          &kappa,
                          &y21, &s1,
                          CERES_CENTRAL_DIFF,
                          -INFINITY,
                          INFINITY);

    // Open the data file for writing
    FILE *data_file = fopen("plot.dat", "w");
    if (!data_file) {
        printf("Error opening file!\n");
        return;
    }

    double p0[3];
    p0[0]=x0;
    p0[1]=y0;
    p0[2]=z0;
    double pi[3] = {0, 0, 0};
    int segments = 100;
    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);

        double s = s1 * progress;
        eq14_interpolate_single_2d_clothoid(s,
                                            y21,
                                            p0,
                                            theta20,
                                            kappa20,
                                            sharpness20,
                                            pi);
        fprintf(data_file, "%lf %lf %lf\n", pi[0], pi[1], pi[2]);
    }
    printf("x: %f y: %f z: %f \n",pi[0],pi[1],pi[2]);

    double p1[3];
    copy_vector(pi,p1);

    // End values first clothoid.
    double sharpness21 = sharpness20 + y21 * s1;
    double kappa21 = kappa20 + sharpness20 * s1 + 0.5 * y21 * (s1 * s1);
    double theta21 = theta20 + kappa20 * s1 + (0.5*sharpness20) * (s1 * s1) + (1.0 / 6.0) * y21 * (s1 * s1 * s1);

    printf("sharpness21 %f \n",sharpness21);
    printf("kappa21 %f \n",kappa21);
    printf("theta21 %f \n",theta21);

    y21 = -fabs(y21);

    for (int i = 0; i < segments; i++) {
        double progress = (double)i / (segments - 1);

        double s = s1 * progress;
        eq14_interpolate_single_2d_clothoid(s,
                                            -y21,
                                            p1,
                                            theta21,
                                            kappa21,
                                            sharpness21,
                                            pi);
        fprintf(data_file, "%lf %lf %lf\n", pi[0], pi[1], pi[2]);
    }
    printf("x: %f y: %f z: %f \n",pi[0],pi[1],pi[2]);

    double sharpness22 = sharpness21 + y21 * s1;
    double kappa22 = kappa21 + sharpness21 * s1 + 0.5 * y21 * (s1 * s1);
    double theta22 = theta21 + kappa21 * s1 + (0.5*sharpness21) * (s1 * s1) + (1.0 / 6.0) * y21 * (s1 * s1 * s1);

    printf("sharpness22 %f \n",sharpness22);
    printf("kappa22 %f \n",kappa22);
    printf("theta22 %f \n",theta22);

    fflush(data_file);
    fclose(data_file);

    // Plot the data using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up the plot
        fprintf(gnuplot, "set title 'Plot'\n");
        fprintf(gnuplot, "set xlabel 'X-axis'\n");
        fprintf(gnuplot, "set ylabel 'Y-axis'\n");
        fprintf(gnuplot, "set zlabel 'Z-axis'\n");
        fprintf(gnuplot, "set xrange [-4:14]\n");  // Adjust ranges as needed
        fprintf(gnuplot, "set yrange [-4:14]\n");
        fprintf(gnuplot, "set zrange [-4:14]\n");
        fprintf(gnuplot, "set view 0, 0\n");  // Adjust view angle

        // Plot all datasets with titles
        fprintf(gnuplot, "splot 'plot.dat' index 0 using 1:2:3 with lines title 'Segment 1' \n ");

        // Close GNUplot
        fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

/* Given jerk and acceleration, calculate the dv (delta velocity)
 * Delta velocity is valid where a scurve is using acceleration * 2 at
 * inflection point.
 *
 * jerk = Steepness value of a scurve.
 * a    = Acceleration.
 * dv   = Delta velocity.   All velocity's > dv contain a inbetween linear acc stage.
 *                          All velocity's < dv have a max acceleration < 2*a or as.
 */
static void jerk_to_dv(double jerk, double a, double *dv){

    double as = a * 2;          // "as"  Max acceleration at inflection point. as=2*A.
    double dvt = 2* as / jerk;  // "dvt" Delta velocity time, derived from: jm=2*as/t1.
    *dv = fabs((dvt * as) / 2); // "dv"  Delta velocity, dv=vo-ve, derived from: t1=2*(ve-vo)/as;

    printf("jerk = %f \n", jerk);
    printf("a = %f \n", a);
    printf("dv = %f \n\n", *dv);
}

/* Build a single 2d clothoid.
 */
static void eq14_gauss_2d(double s1,
                          double x0,
                          double y0,
                          double theta0,
                          double kappa0,
                          double sharpness0,
                          double gamma0,
                          double *x1,
                          double *y1,
                          double *theta_at_s,
                          double *kappa_at_s,
                          double *sharpness_at_s,
                          double *clothoid_length) {

    // Initialize outputs
    *x1 = x0;
    *y1 = y0;
    *clothoid_length = 0;

    // First pass: Compute time and velocity using clothoid integration
    for (int i = 0; i < 16; i++) {
        double ss = 0.5 * s1 * (gauss_nodes_16[i] + 1);
        double weight = 0.5 * s1 * gauss_weights_16[i];

        // Compute theta using the clothoid parameters
        double theta = theta0 + kappa0 * ss + (sharpness0 / 2.0) * ss * ss + (gamma0 / 6.0) * ss * ss * ss;
        // theta_to_acc(theta, acceleration_at_s);  // Computes acceleration

        // Compute displacement increments using Gauss-Legendre quadrature
        double dt = cos(theta) * weight;
        double dy = sin(theta) * weight;

        *x1 += dt;
        *y1 += dy;

        // Update clothoid length considering the displacement in both x and y directions
        *clothoid_length += sqrt(dt * dt + dy * dy);
    }

    // Update theta, kappa, sharpness at final clothoid length
    double t = *clothoid_length;
    *theta_at_s = theta0 + kappa0 * t + (0.5 * sharpness0) * t * t + (1.0 / 6.0) * gamma0 * t * t * t;
    *kappa_at_s = kappa0 + sharpness0 * t + 0.5 * gamma0 * t * t;
    *sharpness_at_s = sharpness0 + gamma0 * t;

    // Sanity check for clothoid length
    if (fabs(*clothoid_length - s1) > 1e-3) {
        // printf("Warning: clothoid length (%.6f) != input s (%.6f)\n", *clothoid_length, s1);
    } else {
        // printf("clothoid fits ok. \n");
    }
}

/* Test a clothoid velocity profile. Experirimental code.
 * This is a 2d clothoid curve, modified to be used as velocity curve.
 *
 * The example shows 2 connected clothoid curves.
 * The clothoid uses gamma0 & ltot to direct the motion profile.
 * These 2 input parameters are eventually used by the ceres non linear solver to
 * fit a motion profile later on.
 *
 *  1. The motion starts at zero.
 *  2. The first clothoid stops at acc: 0.7
 *  3. The second clothoid stops at curvature kappa : 0
 *
 *  This all could be a acceleration stage curve, to attach a linear acc stage.
 *  Where the linear stage is starting at curvature 0. Wich is theoreticly perfect.
 */
//void velocity_test(){

//    // Decompose vector in xy & z components.
//    double gamma0=0.002;            // Also known as y21, Rate of change.  -> Ceres non linear solver input.
//    double ltot=10;                 // Clothoid curve length.   -> Ceres non linear solver input.

//    double velocity0=0;
//    double theta0=0;
//    double kappa0=0;
//    double sharpness0=0;

//    double time_at_s=0;             // Time.
//    double velocity_at_s=0;         // Velocity.
//    double displacement_at_s=0;     // Displacement.
//    double acceleration_at_s=0;     // Acceleration.

//    double sharpness_at_s=0;
//    double theta_at_s=0;
//    double kappa_at_s=0;
//    double clothoid_length=0;

//    // Open files to save the data
//    FILE *data_file_velocity = fopen("clothoid_velocity.dat", "w");
//    FILE *data_file_displacement = fopen("clothoid_displacement.dat", "w");
//    FILE *data_file_acceleration = fopen("clothoid_acceleration.dat", "w");
//    FILE *data_file_junction = fopen("clothoid_junction.dat", "w");
//    FILE *data_file_jerk = fopen("clothoid_jerk.dat", "w");

//    if (!data_file_velocity || !data_file_displacement || !data_file_acceleration) {
//        printf("Error opening file!\n");
//        return;
//    }

//    int N = 100;
//    double vel=0;
//    double time=0;
//    double acc=0;
//    double jerk=0;
//    double dis=0;
//    for(int i=0; i<=N; i++){
//        double iteration_length = ((double)i/N) * ltot;
//        velocity_profile(iteration_length,
//                         velocity0,
//                         theta0,
//                         kappa0,
//                         sharpness0,
//                         gamma0,
//                         &time_at_s,
//                         &velocity_at_s,
//                         &acceleration_at_s,
//                         &sharpness_at_s,
//                         &theta_at_s,
//                         &kappa_at_s,
//                         &clothoid_length
//                         );

//        printf("\n** Clothoid velocity curve 0 ** \n");
//        printf("clothoid length :   %f \n",clothoid_length);
//        printf("time_at_s:          %f \n",time_at_s);
//        printf("theta_at_s degree:  %f \n",theta_at_s*toDegrees);
//        printf("velocity:           %f \n",velocity_at_s);
//        printf("acc_at_s :          %f \n",acceleration_at_s);
//        printf("kappa_at_s :        %f \n",kappa_at_s);

//        displacement_at_s+=  0.5 * (vel + velocity_at_s) * (time_at_s-time);
//        printf("displacement:       %f \n",displacement_at_s);

//        jerk = (acceleration_at_s - acc) / (time_at_s-time); // Jerk = delta Acc / delta Time.
//        printf("jerk:               %f \n",jerk);

//        fprintf(data_file_velocity,     "%lf %lf\n", time_at_s, velocity_at_s);
//        fprintf(data_file_acceleration, "%lf %lf\n", time_at_s, acceleration_at_s);
//        fprintf(data_file_displacement, "%lf %lf\n", time_at_s, displacement_at_s);
//        fprintf(data_file_jerk,         "%lf %lf\n", time_at_s, jerk);

//        // Update.
//        vel=velocity_at_s;
//        time=time_at_s;
//        acc = acceleration_at_s;

//        if(acc>0.7){
//            break;
//        }
//    }

//    fprintf(data_file_junction, "%lf %lf\n", time_at_s, 0);
//    fprintf(data_file_junction, "%lf %lf\n", time_at_s, velocity_at_s+10);

//    time=0;

//    double tprev = time_at_s;
//    theta0=theta_at_s;
//    kappa0=kappa_at_s;
//    sharpness0=sharpness_at_s;
//    velocity0=vel;
//    ltot=10;
//    gamma0=-0.01;

//    // Second clothoid.
//    for(int i=0; i<=N; i++){
//        double iteration_length = ((double)i/N) * ltot;
//        velocity_profile(iteration_length,
//                         velocity0,
//                         theta0,
//                         kappa0,
//                         sharpness0,
//                         gamma0,
//                         &time_at_s,
//                         &velocity_at_s,
//                         &acceleration_at_s,
//                         &sharpness_at_s,
//                         &theta_at_s,
//                         &kappa_at_s,
//                         &clothoid_length
//                         );

//        printf("\n** Clothoid velocity curve 0 ** \n");
//        printf("clothoid length :   %f \n",clothoid_length);
//        printf("time_at_s:          %f \n",time_at_s);
//        printf("theta_at_s degree:  %f \n",theta_at_s*toDegrees);
//        printf("velocity:           %f \n",velocity_at_s);
//        printf("acc_at_s :          %f \n",acceleration_at_s);
//        printf("kappa_at_s :        %f \n",kappa_at_s);

//        displacement_at_s+=  0.5 * (vel + velocity_at_s) * (time_at_s-time);
//        printf("displacement:       %f \n",displacement_at_s);

//        jerk = (acceleration_at_s - acc) / (time_at_s-time);
//        printf("jerk:               %f \n",jerk);

//        fprintf(data_file_velocity,     "%lf %lf\n", time_at_s+tprev, velocity_at_s);
//        fprintf(data_file_acceleration, "%lf %lf\n", time_at_s+tprev, acceleration_at_s);
//        fprintf(data_file_displacement, "%lf %lf\n", time_at_s+tprev, displacement_at_s);
//        fprintf(data_file_jerk,         "%lf %lf\n", time_at_s+tprev, jerk);

//        // Update.
//        vel=velocity_at_s;
//        time=time_at_s;
//        acc = acceleration_at_s;

//        if(kappa_at_s<0){
//            break;
//        }
//    }

//    fclose(data_file_velocity);
//    fclose(data_file_displacement);
//    fclose(data_file_acceleration);
//    fclose(data_file_junction);
//    fclose(data_file_jerk);

//    // Plot the data using GNUplot
//    FILE *gnuplot = popen("gnuplot -persistent", "w");
//    if (gnuplot != NULL) {
//        // Set up the plot
//        fprintf(gnuplot, "set title 'Clothoid 2D Trajectory'\n");
//        fprintf(gnuplot, "set xlabel 'Time (t)'\n");
//        fprintf(gnuplot, "set ylabel 'Velocity'\n");
//        fprintf(gnuplot, "set grid\n");

//        // Set the aspect ratio to make X and Y the same scale
//        fprintf(gnuplot, "set size ratio -1\n");

//        // Set the line color and style for velocity, displacement, and acceleration
//        fprintf(gnuplot, "set style line 1 lc rgb 'red' pt 7 ps 1.5 title 'Velocity'\n");
//        fprintf(gnuplot, "set style line 2 lc rgb 'blue' pt 7 ps 1.5 title 'Displacement'\n");
//        fprintf(gnuplot, "set style line 3 lc rgb 'green' pt 7 ps 1.5 title 'Acceleration'\n");
//        fprintf(gnuplot, "set style line 4 lc rgb 'brown' pt 7 ps 1.5 title 'Junction'\n");
//        fprintf(gnuplot, "set style line 5 lc rgb 'orange' pt 7 ps 1.5 title 'Jerk'\n");

//        fprintf(gnuplot, "set margin 5,5,5,10\n"); // Set margins (left, right, bottom, top)

//        // Dynamically set the range (use autoscale for automatic range adjustment)
//        // fprintf(gnuplot, "set autoscale\n");  // This line auto-adjusts the range based on data.

//        // Plot the 3 curves without fitting
//        fprintf(gnuplot,
//                "plot 'clothoid_velocity.dat' using 1:2 with lines linestyle 1, \
//                'clothoid_displacement.dat' using 1:2 with lines linestyle 2, \
//                'clothoid_acceleration.dat' using 1:2 with lines linestyle 3, \
//                'clothoid_junction.dat' using 1:2 with lines linestyle 4, \
//                'clothoid_jerk.dat' using 1:2 with lines linestyle 5  \n");

//                fflush(gnuplot);
//                fclose(gnuplot);
//    } else {
//        printf("Error: Unable to open GNUplot\n");
//    }
//}

/* Solve the clothoid given a time position.
 *
 *
 */
static void interpolate_clothoid_by_time(double total_time,
                                         double gamma0,
                                         double velocity0,
                                         double theta0,
                                         double kappa0,
                                         double sharpness0,
                                         double *velocity1,
                                         double *theta1,
                                         double *kappa1,
                                         double *sharpness1,
                                         double *acceleration1,
                                         double *clothoid_length
                                         ) {

    // Initialize variables
    double current_velocity = velocity0;
    double current_theta = theta0;
    double current_kappa = kappa0;
    double current_sharpness = sharpness0;
    double current_acceleration = 0;
    double current_length = 0;

    theta_to_acc(theta0, &current_acceleration);

    // Gaussian integration for this time step
    double ds = 0.0;
    double dv = 0.0;

    for (int i = 0; i < 16; i++) {
        double t_gauss = 0.5 * total_time * (gauss_nodes_16[i] + 1);
        double weight = 0.5 * total_time * gauss_weights_16[i];

        // Compute intermediate clothoid length at Gauss point
        double s_gauss = current_length + t_gauss; // Using t_gauss as fraction of step

        // Compute theta at Gauss point using current clothoid parameters
        double theta_g = theta0 + kappa0 * s_gauss +
                (0.5 * sharpness0) * s_gauss * s_gauss +
                (gamma0 / 6.0) * s_gauss * s_gauss * s_gauss;

        // Compute derivatives at Gauss point
        double dx = cos(theta_g);
        double dy = sin(theta_g);

        // Contribute to integrals
        ds += weight / dx;  // ds = dt/cos(theta)
        dv += dy * weight;   // velocity component
    }

    // Update state variables
    current_length += ds;
    current_velocity += dv;

    // Update clothoid parameters at new length
    current_theta = theta0 + kappa0 * current_length +
            (0.5 * sharpness0) * current_length * current_length +
            (1.0/6.0) * gamma0 * current_length * current_length * current_length;

    current_kappa = kappa0 + sharpness0 * current_length +
            0.5 * gamma0 * current_length * current_length;

    current_sharpness = sharpness0 + gamma0 * current_length;
    theta_to_acc(current_theta, &current_acceleration);

    // Set final output values
    *clothoid_length = current_length;
    *velocity1 = current_velocity;
    *acceleration1 = current_acceleration;
    *sharpness1 = current_sharpness;
    *theta1 = current_theta;
    *kappa1 = current_kappa;
}

/* Instead of using clothoid length as imput, we use time as input.
 *
 */
void velocity_time_test() {
    // Parameters
    double gamma0 = 0.001;
    double velocity0 = 0;
    double theta0 = 0;
    double kappa0 = 0;
    double sharpness0 = 0;
    double interval = 0.001;
    double total_time = 10.0;  // sec
    double time=0;

    // Results.
    double clothoid_length=0;
    double velocity1=0;
    double acceleration1=0;
    double sharpness1=0;
    double theta1=0;
    double kappa1=0;

    double vel=0;
    double acc=0;
    double displacement=0;
    double jerk=0;

    FILE *data_file_velocity = fopen("clothoid_velocity.dat", "w");
    FILE *data_file_displacement = fopen("clothoid_displacement.dat", "w");
    FILE *data_file_acceleration = fopen("clothoid_acceleration.dat", "w");
    FILE *data_file_jerk = fopen("clothoid_jerk.dat", "w");

    // Simulation loop (100 steps)
    while(time<total_time) {

        time += interval;
        interpolate_clothoid_by_time( time,
                                      gamma0,
                                      velocity0,
                                      theta0,
                                      kappa0,
                                      sharpness0,
                                      &velocity1,
                                      &theta1,
                                      &kappa1,
                                      &sharpness1,
                                      &acceleration1,
                                      &clothoid_length);

        displacement+=  0.5 * (vel + velocity1) * interval;
        jerk = (acceleration1 - acc) / interval; // Jerk = delta Acc / delta Time.

        // Plot to file.
        fprintf(data_file_velocity,     "%lf %lf\n", time, velocity1);
        fprintf(data_file_acceleration, "%lf %lf\n", time, acceleration1*0.1);
        fprintf(data_file_displacement, "%lf %lf\n", time, displacement);
        fprintf(data_file_jerk,         "%lf %lf\n", time, jerk*0.01);

        // Update.
        acc = acceleration1;
        vel = velocity1;
    }

    printf("time: %f length: %f velocity: %f acceleration: %f theta degrees: %f ",
           total_time,
           clothoid_length,
           velocity1,
           acceleration1,
           theta1*toDegrees);
    printf("displacement: %f ",displacement);
    printf("jerk: %f \n",jerk);

    /* Output:
            time: 14.915549
            length: 14.998445
            velocity: 1.025892
            acceleration: 0.288813
            theta degrees: 16.109426
            displacement: 3.070362
            jerk: -0.028566
    */

    fclose(data_file_velocity);
    fclose(data_file_displacement);
    fclose(data_file_acceleration);
    fclose(data_file_jerk);

    // Plot the data using GNUplot
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    if (gnuplot != NULL) {
        // Set up the plot
        fprintf(gnuplot, "set title 'Clothoid 2D Trajectory'\n");
        fprintf(gnuplot, "set xlabel 'Time (t)'\n");
        fprintf(gnuplot, "set ylabel 'Velocity'\n");
        fprintf(gnuplot, "set grid\n");

        // Set the line color and style for velocity, displacement, and acceleration
        fprintf(gnuplot, "set style line 1 lc rgb 'red' pt 7 ps 1.5 title 'Velocity'\n");
        fprintf(gnuplot, "set style line 2 lc rgb 'blue' pt 7 ps 1.5 title 'Displacement'\n");
        fprintf(gnuplot, "set style line 3 lc rgb 'green' pt 7 ps 1.5 title 'Acceleration'\n");
        fprintf(gnuplot, "set style line 5 lc rgb 'orange' pt 7 ps 1.5 title 'Jerk'\n");

        fprintf(gnuplot, "set margin 5,5,5,10\n"); // Set margins (left, right, bottom, top)

        // Dynamically set the range (use autoscale for automatic range adjustment)
        fprintf(gnuplot, "set autoscale\n");  // This line auto-adjusts the range based on data.

        // Plot the 3 curves without fitting
        fprintf(gnuplot,
                "plot 'clothoid_velocity.dat' using 1:2 with lines linestyle 1, \
                'clothoid_displacement.dat' using 1:2 with lines linestyle 2, \
                'clothoid_acceleration.dat' using 1:2 with lines linestyle 3, \
                'clothoid_jerk.dat' using 1:2 with lines linestyle 4  \n");

                fflush(gnuplot);
                fclose(gnuplot);
    } else {
        printf("Error: Unable to open GNUplot\n");
    }
}

#endif // VELOCITY_H
