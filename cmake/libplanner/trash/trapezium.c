#include "trapezium.h"

inline void stop_motion(double *curvel, double *curpos, double *curacc, double maxacc, double cycletime) {
    // If the velocity is already zero, we can exit early
    if (*curvel == 0) {
        return;
    }

    // Decelerate the current velocity towards zero
    if (*curvel > 0) {
        *curacc=-maxacc;
        *curvel -= maxacc * cycletime; // Decelerate (positive velocity)
        if (*curvel < 0) *curvel = 0; // Ensure the velocity doesn't go negative
    } else if (*curvel < 0) {
        *curacc=maxacc;
        *curvel += maxacc * cycletime; // Decelerate (negative velocity)
        if (*curvel > 0) *curvel = 0; // Ensure the velocity doesn't go positive
    }

    // If the velocity is approaching zero, stop overshoot
    if (fabs(*curvel) < maxacc * cycletime) {
        *curvel = 0;  // Clamps the velocity to zero to avoid overshooting
        *curacc = 0;
    }

    // Update the position based on the current velocity
    *curpos += *curvel * cycletime;

    // Print the current state for debugging
    // printf("Stopping: Position = %.2f, Velocity = %.2f\n", *curpos, *curvel);
}

// Function to handle forward deceleration stage
inline void motion_fwd_dcc(double *curvel, double *curpos, double acc, double cycletime) {
    // Decelerating in the forward direction (decreasing positive velocity)
    *curvel -= acc * cycletime;  // Decrease the velocity in the positive direction

    // Ensure the velocity doesn't go below 0 (stopping the motion when it reaches zero)
    if (*curvel < 0) {
        *curvel = 0;  // Stop the forward motion once the velocity reaches zero
    }

    // Update the position based on the current velocity
    *curpos += *curvel * cycletime;

    // Print the current state for debugging
    // printf("Forward Deceleration: Position = %.2f, Velocity = %.2f\n", *curpos, *curvel);
}

// Function to handle reverse deceleration stage with velocity limit
inline void motion_rev_dcc(double *curvel, double *curpos, double acc, double cycletime) {
    // Decelerating in the reverse direction (decreasing negative velocity)
    *curvel += acc * cycletime;  // Increase the velocity towards zero (less negative)

    // Ensure the velocity doesn't go above 0 (stop when the velocity reaches 0)
    if (*curvel > 0) {
        *curvel = 0;  // Stop the reverse motion once the velocity reaches zero
    }

    // Update the position based on the current velocity
    *curpos += *curvel * cycletime;

    // Print the current state for debugging
    // printf("Reverse Deceleration: Position = %.2f, Velocity = %.2f\n", *curpos, *curvel);
}

// Function to handle forward acceleration stage with velocity limit
inline void motion_fwd_acc(double *curvel, double *curpos, double acc, double maxvel, double cycletime) {
    // Accelerating in the forward direction (increasing positive velocity)
    *curvel += acc * cycletime;  // Increase the velocity in the positive direction

    // Ensure the velocity doesn't exceed the maximum velocity
    if (*curvel > maxvel) {
        *curvel = maxvel;  // Cap the velocity at maxvel
    }

    // Update the position based on the current velocity
    *curpos += *curvel * cycletime;

    // Print the current state for debugging
    // printf("Forward Acceleration: Position = %.2f, Velocity = %.2f\n", *curpos, *curvel);
}

// Function to handle reverse acceleration stage with velocity limit
inline void motion_rev_acc(double *curvel, double *curpos, double acc, double maxvel, double cycletime) {
    // Accelerating in the reverse direction (increasing negative velocity)
    *curvel -= acc * cycletime;  // Increase the negative velocity (reverse direction)

    // Ensure the velocity doesn't exceed the maximum velocity in reverse
    if (*curvel < -maxvel) {
        *curvel = -maxvel;  // Cap the velocity at -maxvel (reverse max velocity)
    }

    // Update the position based on the current velocity
    *curpos += *curvel * cycletime;

    // Print the current state for debugging
    // printf("Reverse Acceleration: Position = %.2f, Velocity = %.2f\n", *curpos, *curvel);
}

// Function to handle steady motion in the forward direction
inline void steady_motion_fwd(double *curvel, double *curpos, double cycletime) {
    // In steady motion, the velocity remains constant
    // We just need to update the position based on the current velocity

    *curpos += *curvel * cycletime;

    // Print the current state for debugging
    // printf("Steady Forward Motion: Position = %.2f, Velocity = %.2f\n", *curpos, *curvel);
}

// Function to handle steady motion in the reverse direction
inline void steady_motion_rev(double *curvel, double *curpos, double cycletime) {
    // In steady motion, the velocity remains constant
    // We just need to update the position based on the current velocity

    *curpos += *curvel * cycletime;

    // Print the current state for debugging
    // printf("Steady Reverse Motion: Position = %.2f, Velocity = %.2f\n", *curpos, *curvel);
}

// Function to check what phase (acceleration, constant velocity, or deceleration) is next
inline enum motion_state_trapezium trapezium_update(double *curpos,
                                                    double *curvel,
                                                    double *curacc,
                                                    double tarpos,
                                                    double maxvel,
                                                    double maxacc,
                                                    double endvel,
                                                    double cycletime,
                                                    double finish_tollerance,
                                                    int motion_rev,
                                                    int debug) {

    if(motion_rev){
        endvel=-fabs(endvel);
    }

    int res = update_next_cycle(*curpos, *curvel, tarpos, maxvel, maxacc, endvel);

    if (res == stop) {
        stop_motion(curvel, curpos, curacc, maxacc, cycletime);
    }
    if (res == forward_acc) {
        *curacc = maxacc;
        motion_fwd_acc(curvel, curpos, maxacc, maxvel, cycletime);
    }
    if (res == forward_dcc) {
        *curacc = -maxacc;
        motion_fwd_dcc(curvel, curpos, maxacc, cycletime);
    }
    if (res == forward_steady) {
        *curacc = 0;
        steady_motion_fwd(curvel, curpos, cycletime);
    }
    if (res == reverse_acc) {
        *curacc = -maxacc;
        motion_rev_acc(curvel, curpos, maxacc, maxvel, cycletime);
    }
    if (res == reverse_dcc) {
        *curacc = maxacc;
        motion_rev_dcc(curvel, curpos, maxacc, cycletime);
    }
    if (res == reverse_steady) {
        *curacc = 0;
        steady_motion_rev(curvel, curpos, cycletime);
    }

    if(debug){
        printf("pos: %.3f, vel: %.3f, acc: %.2f, \n", *curpos, *curvel, *curacc);
    }

    if (res == error) {
        if(debug){
            printf("error.\n");
        }
    }

    // Check for finished motion here. Cycles are updated above.
    // Using endvel, motion fwd.
    if( (*curvel) > 0 && endvel>0 && (*curpos > tarpos) && !motion_rev){
        return  finished;
    }
    // Using endvel, motion reverse.
    if( (*curvel) < 0 && endvel<0 && (*curpos < tarpos) && motion_rev){
        return  finished;
    }


    // Target reached using endvel=0, motion forward.
    if ( ((tarpos - (*curpos) ) <= finish_tollerance) && endvel==0 && !motion_rev ) { // Finished
        // printf("stop finished.");
        *curpos=tarpos;
        *curvel=0;
        *curacc=0;
        return finished;
    }

    // Target reached using endvel=0, motion reverse.
    if ( (fabs(tarpos - (*curpos) ) <= finish_tollerance) && endvel==0 && motion_rev ) { // Finished
        // printf("stop finished.");
        *curpos=tarpos;
        *curvel=0;
        *curacc=0;
        return finished;
    }

    return res;
}


// Function to check what phase (acceleration, constant velocity, or deceleration) is next
inline enum motion_state_trapezium update_next_cycle(double curpos,
                                                     double curvel,
                                                     double tarpos,
                                                     double maxvel,
                                                     double maxacc,
                                                     double endvel) {

    maxvel=fabs(maxvel);

    // Calculate the remaining distance
    double dtg = tarpos - curpos;

    // Lower curvel to vm;
    if (curvel > 0 && fabs(curvel) > maxvel) {
        // printf("fwd stop, lower curvel to vm.");
        return stop;
    }
    if (curvel < 0 && fabs(curvel) > maxvel) {
        // printf("rev stop, lower curvel to vm.");
        return stop;
    }

    // Reverse direction check
    if (dtg < 0) { // Reverse
        if (curvel > 0) {
            // printf("stop, motion is fwd, change dir to rev first.");
            return stop;  // Switch to stop if moving forward when it should be reverse
        }

        // printf("reverse.");
        // Check sub state acc,dcc,stead.
        return motion_sub_phase(curpos,tarpos,curvel,endvel,maxacc,maxvel);
    }

    // Forward direction check
    if (dtg > 0) { // Forward
        if (curvel < 0) {
            // printf("stop, motion is rev, change dir to fwd first.");
            return stop;
        }
        // printf("forward.");
        // Check sub state acc,dcc,stead.
        return motion_sub_phase(curpos,tarpos,curvel,endvel,maxacc,maxvel);
    }

    // printf("error.");
    return error;
}

// For negatvie motion, input curvel, endvel negative. Input acc positive.
inline double stopdist(double curvel, double endvel, double acc) {
    curvel=fabs(curvel);
    endvel=fabs(endvel);
    return (curvel * curvel - endvel * endvel) / (2 * acc);
}

// Works for pos or neg motion.
inline enum motion_state_trapezium motion_sub_phase(double curpos,
                                                    double tarpos,
                                                    double curvel,
                                                    double endvel,
                                                    double maxacc,
                                                    double maxvel) {

    maxacc=fabs(maxacc);
    maxvel=fabs(maxvel);

    double stoplength=stopdist(curvel,endvel,maxacc);
    // printf("stoplength %f \n",stoplength);
    double dtg = tarpos - curpos;
    // printf("dtg %f \n",dtg);

    if(fabs(dtg)>stoplength && fabs(curvel)<maxvel){
        if(dtg<0){
            // printf("rev acc. \n");
            return reverse_acc;
        } else {
            // printf("fwd acc. \n");
            return forward_acc;
        }
    }

    if(fabs(dtg)<stoplength){
        if(dtg<0){
            // printf("rev dcc. \n");
            return reverse_dcc;
        } else {
            // printf("fwd dcc. \n");
            return forward_dcc;
        }
    }

    if(fabs(curvel)==maxvel){
        if(dtg<0){
            // printf("rev steady. \n");
            return reverse_steady;
        } else {
            // printf("fwd steady. \n");
            return forward_steady;
        }
    }

    if(fabs(curvel)>maxvel){
        if(dtg<0){
            // printf("rev dcc. \n");
            return reverse_dcc;
        } else {
            // printf("fwd dcc. \n");
            return forward_dcc;
        }
    }

    // printf("finished. \n");
    return error;
}
