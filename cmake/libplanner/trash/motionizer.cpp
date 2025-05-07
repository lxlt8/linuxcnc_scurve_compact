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
#include "motionizer.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include "vector.h"
#include "segment.h"

double motionizer::final_velocity(double vo, double maxvel, double maxacc, double length) {
    double ve_squared = vo * vo + 2 * maxacc * length;
    double ve = std::sqrt(ve_squared);
    return std::min(ve, maxvel); // Limit by the max velocity (vm)
}

// This is used to sweep entire trajectory. Using flag : USE_LOOK_AHEAD_ALL is tp.c
void motionizer::forward_sweep(std::vector<emcmot_segment>& motions) {

    for (auto it = motions.begin(); it != motions.end(); ++it) {
        if (it == motions.begin()) {
            it->vo = 0.0;
        } else {
            it->vo = (it - 1)->ve;
        }

        it->ve = final_velocity(it->vo, it->vel, it->acc, it->length_netto);
    }

    if (!motions.empty()) {
        (motions.end() - 1)->ve = 0.0;
    }
}

// This is used to sweep a portion of the trajectory. This is a runtime look ahead.
void motionizer::forward_sweep(std::vector<emcmot_segment>& motions, int index, int look_ahead_lines) {

    int start=std::fmax(0, index-look_ahead_lines);
    int end=std::fmin(motions.size(), index+look_ahead_lines);

    for(int i=start; i<end; i++){

        if(i==start){
            motions[i].vo=0;
        } else {
            motions[i].vo = motions[i-1].ve;
        }
        motions[i].ve = final_velocity(motions[i].vo, motions[i].vel, motions[i].acc, motions[i].length_netto);
    }

    if(motions.size()>0){
        motions[end-1].ve=0;
    }
}

// This is used to sweep a portion of the trajectory. This is a runtime look ahead.
void motionizer::forward_sweep(std::vector<emcmot_segment>& motions, int index, int look_ahead_lines, double vo) {

    int start=std::fmax(0, index-look_ahead_lines);
    int end=std::fmin(motions.size(), index+look_ahead_lines);

    for(int i=start; i<end; i++){

        if(i==start){
            motions[i].vo=vo;
        } else {
            motions[i].vo = motions[i-1].ve;
        }
        motions[i].ve = final_velocity(motions[i].vo, motions[i].vel, motions[i].acc, motions[i].length_netto);
    }

    if(motions.size()>0){
        motions[end-1].ve=0;
    }
}

// This is used to sweep entire trajectory. Using flag : USE_LOOK_AHEAD_ALL is tp.c
void motionizer::reverse_sweep(std::vector<emcmot_segment>& motions){

    for (int i = motions.size() - 1; i > 0; --i) {
        double current_vo = motions[i].vo;
        motions[i].vo = final_velocity(motions[i].ve, motions[i].vel, motions[i].acc, motions[i].length_netto);
        motions[i].vo = std::min(motions[i].vo, motions[i].vel);

        if(i>0){
            motions[i].vo = std::min(motions[i].vo, motions[i - 1].ve);
        }

        motions[i].vo = std::min(motions[i].vo, current_vo);

        if(i>0){
            motions[i - 1].ve = std::min(motions[i - 1].ve, motions[i].vo);
        }
    }
}

// This is used to sweep a portion of the trajectory. This is a runtime look ahead.
void motionizer::reverse_sweep(std::vector<emcmot_segment>& motions, int index, int look_ahead_lines){

    int start=std::fmax(0, index-look_ahead_lines);
    int end=std::fmin(motions.size(), index+look_ahead_lines);

    for (int i = end-1; i > start; --i) {
        double current_vo = motions[i].vo;
        motions[i].vo = final_velocity(motions[i].ve, motions[i].vel, motions[i].acc, motions[i].length_netto);
        motions[i].vo = std::min(motions[i].vo, motions[i].vel);

        if(i>start){
            motions[i].vo = std::min(motions[i].vo, motions[i - 1].ve);
        }

        motions[i].vo = std::min(motions[i].vo, current_vo);

        if(i>start){
            motions[i - 1].ve = std::min(motions[i - 1].ve, motions[i].vo);
        }
    }
}

// This is used to sweep a portion of the trajectory. This is a runtime look ahead.
void motionizer::reverse_sweep(std::vector<emcmot_segment>& motions, int index, int look_ahead_lines, double vo){

    int start=std::fmax(0, index-look_ahead_lines);
    int end=std::fmin(motions.size(), index+look_ahead_lines);

    for (int i = end-1; i > start; --i) {
        double current_vo = motions[i].vo;
        motions[i].vo = final_velocity(motions[i].ve, motions[i].vel, motions[i].acc, motions[i].length_netto);
        motions[i].vo = std::min(motions[i].vo, motions[i].vel);

        if(i>start){
            motions[i].vo = std::min(motions[i].vo, motions[i - 1].ve);
        }

        motions[i].vo = std::min(motions[i].vo, current_vo);

        if(i>start){
            motions[i - 1].ve = std::min(motions[i - 1].ve, motions[i].vo);
        }
    }

    // Not very elegant, but check if this works.
    if(motions.size()>0){
        motions[start].vo = vo;
    }
}

// Print using flag : DEBUG_MOTIONIZER in tp.c
void motionizer::print_motions(const std::vector<emcmot_segment>& motions) const {
    for (const auto& motion : motions) {
        std::cout << "vo: " << motion.vo
                  << ", ve: " << motion.ve
                  << ", vm: " << motion.vel
                  << ", length: " << motion.length_netto
                  << ", amax: " << motion.acc
                  << ", kmax: " << motion.kmax
                  << ", trajectory begin: " << motion.trajectory_length_begin
                  << ", trajectory end: " << motion.trajectory_length_end
                  << std::endl;
    }
}

int motionizer::velocity_from_curvature(const double& maxacc, const double& kmax, double& maxvel) {

    // Ensure kmax (maximum curvature) is positive (since curvature is 1/r, it must be non-negative)
    if (kmax < 0) {
        std::cout<<"kmax error"<<std::endl;
        return -1;
    }

    // If kmax is zero, the motion is a straight line, so max velocity is only limited by maxvel
    if (kmax == 0) {
        return 0; // Straight line, return max velocity limit
    }

    // Calculate maximum velocity based on maximum acceleration and kmax:
    // v_max = sqrt(maxacc / kmax)
    // Explanation:
    // - The centripetal acceleration required to maintain a curved path is given by the formula:
    //   a_c = v^2 * kmax
    // - Rearranging this gives us: v^2 = a_c / kmax
    // - To find the maximum velocity before exceeding maxacc, we set a_c = maxacc:
    //   v^2 = maxacc / kmax
    // - Finally, taking the square root gives us the maximum velocity:
    //   v_max = sqrt(maxacc / kmax)
    double computed_maxvel = std::sqrt(maxacc / kmax);

    // Return the minimum of the calculated max velocity and the input max velocity limit
    maxvel = std::min(computed_maxvel, maxvel);

    return 0;
}

// Edited to only modify maxvel for fillet motions types.
int motionizer::update_vector_maxvel_given_curvature(std::vector<emcmot_segment>& motions) {
    for (auto it = motions.begin(); it != motions.end(); ++it) {

        if(it->canon_motion_type==10){
            // Assuming velocity_from_curvature returns an int status (0 for success)
            int status = velocity_from_curvature(it->acc, it->kmax, it->vel);
            // Check if the velocity calculation was successful
            if (status == -1) {
                std::cout << "Error calculating maxvel for segment." << std::endl;
                return -1;
            }
        }
    }
    return 0;
}

// Calculate speeds, based on curvature extrema.
// Curvature extrema is the point where the radius of a curve is smallest or highest.
// We use the lower limit extrema value to calculate speeds.
int motionizer::update_vector_maxvel_given_curvature(std::vector<emcmot_segment>& motions,
                                                     int index,
                                                     int look_ahead_lines,
                                                     double speed_factor) {

    int start=std::fmax(0, index-look_ahead_lines);
    int end=std::fmin(motions.size(), index+look_ahead_lines);

    for(int i=start; i<end; i++){

        // Seems to stop in a program?
        // Arc segments G2, G3
        if(motions[i].canon_motion_type==3){

            // Programmed feed :
            double feed = motions[i].vel;
            double feed_curvature = motions[i].vel;

            // Calculate feed based on curvature :
            int status = velocity_from_curvature(motions[i].acc, motions[i].kmax, feed_curvature);
            // Check if the velocity calculation was successful
            if (status == -1) {
                std::cout << "Error calculating maxvel for segment." << std::endl;
                return -1;
            }

            // Limit feed curvature to programmed feed.
            feed_curvature = fmin(feed_curvature, feed);

            // Lat's flatten out speed fluctuations given a factor.
            double diff = fabs(feed - feed_curvature);

            motions[i].vel=feed_curvature + (diff*speed_factor);

            // std::cout<<"arc acc:"<<motions[i].acc<<std::endl;
            // std::cout<<"arc kmax:"<<motions[i].kmax<<std::endl;
            // std::cout<<"arc feed:"<<motions[i].vel<<std::endl;
        }

        // Only edit maxvel for fillet types.
        if(motions[i].canon_motion_type==10){

            int status = velocity_from_curvature(motions[i].acc, motions[i].kmax, motions[i].vel);
            // Check if the velocity calculation was successful
            if (status == -1) {
                std::cout << "Error calculating maxvel for segment." << std::endl;
                return -1;
            }

            // Limit the speed of this fillet to the previous attached segment.
            // Wich will be more valid and give's less speed fluctuations.
            if(i>0){
                double maxvel_previous_segment = motions[i-1].vel;
                motions[i].vel = fmin(motions[i].vel, maxvel_previous_segment);
            }

            // Mow if the fillet speed is too low for satisfying the user.
            // The user may tune his machine on this setting.
            // Sinds each machine has different inertia, corner speeds may be different.
            // User may speed up the blend velocity so their machine can
            // eventually walk out the workshop.
            if(i>0){
                double maxvel_previous_segment = motions[i-1].vel;

                // Difference in speed previous segment and this blend segment. (speed based on curvature)
                // Lat's flatten out the diff by a factor.
                double diff = fabs(maxvel_previous_segment - motions[i].vel);

                motions[i].vel=motions[i].vel + (diff*speed_factor);
            }
        }
    }
    return 0;
}

int motionizer::init(vector *ptr, int debug){

    std::vector<emcmot_segment>& motions = ptr->vec;

    // The gui gives a velocity max "vm".
    // If we have a tiny arc, we must lower the vm given the segment's curvature.
    if(0!=update_vector_maxvel_given_curvature(motions)){
        std::cout<<"mavel error"<<std::endl;
        return -1;
    }

    // Forward sweep over the trajectory to colculate vo's, ve's.
    forward_sweep(motions);
    // Reverse sweep over the trajectory to colculate vo's, ve's.
    reverse_sweep(motions);

    if(debug){
        print_motions(motions);
    }

    return 0;
}

int motionizer::init(vector *ptr, int index, int look_ahead_lines, double speed_factor, int debug){

    std::vector<emcmot_segment>& motions = ptr->vec;

    // The gui gives a velocity max "vm".
    // If we have a tiny arc, we must lower the vm given the segment's curvature.
    if(0!=update_vector_maxvel_given_curvature(motions,index,look_ahead_lines,speed_factor)){
        std::cout<<"mavel error"<<std::endl;
        return -1;
    }

    // Forward sweep over the trajectory to colculate vo's, ve's.
    forward_sweep(motions,index,look_ahead_lines);
    // Reverse sweep over the trajectory to colculate vo's, ve's.
    reverse_sweep(motions,index,look_ahead_lines);

    if(debug){
        print_motions(motions);
    }

    return 0;
}

int motionizer::init(vector *ptr, int index, int look_ahead_lines, double speed_factor, double vo, int debug){

    std::vector<emcmot_segment>& motions = ptr->vec;

    // The gui gives a velocity max "vm".
    // If we have a tiny arc, we must lower the vm given the segment's curvature.
    if(0!=update_vector_maxvel_given_curvature(motions,index,look_ahead_lines,speed_factor)){
        std::cout<<"mavel error"<<std::endl;
        return -1;
    }

    // Forward sweep over the trajectory to colculate vo's, ve's.
    forward_sweep(motions,index,look_ahead_lines,vo);
    // Reverse sweep over the trajectory to colculate vo's, ve's.
    reverse_sweep(motions,index,look_ahead_lines,vo);

    if(debug){
        print_motions(motions);
    }

    return 0;
}

// Sample function how to calculate the kmax, curvature value.
double motionizer::process_kmax(double radius) {
    // Curvature of a line segment is 0
    double k_line = 0.0;

    // Curvature of an arc: k_arc = 1/radius
    double k_arc = 1.0 / fabs(radius);

    // Maximum curvature is the maximum of line curvature and arc curvature
    double kmax = std::max(k_line, k_arc);
    return kmax;
}


// Wrapper functions
extern "C" motionizer* motionizer_init_ptr() {
    return new motionizer();
}

extern "C" int motionizer_process_segments_all(motionizer* motionizer_ptr,
                                               vector* ptr,
                                               int debug) {

    return motionizer_ptr->init(ptr,debug);
}

extern "C" int motionizer_process_segments_portion(motionizer* motionizer_ptr,
                                                   vector* ptr,
                                                   int index,
                                                   int look_ahead_lines,
                                                   double speed_factor,
                                                   int debug) {

    return motionizer_ptr->init(ptr, index, look_ahead_lines, speed_factor, debug);
}

extern "C" int motionizer_process_segments_portion_vo(motionizer* motionizer_ptr,
                                                      vector* ptr,
                                                      int index,
                                                      int look_ahead_lines,
                                                      double speed_factor,
                                                      double vo,
                                                      int debug) {

    return motionizer_ptr->init(ptr, index, look_ahead_lines, speed_factor, vo, debug);
}




