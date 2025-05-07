#ifndef POSITION_UPDATE_H
#define POSITION_UPDATE_H

#include <stdint.h>
#include <math.h>
#include <stdio.h>


#include <stdint.h>
#include <math.h>

// Constants
#define POSITION_UPDATE_CYCLE_TIME_MS 1  // 1ms cycle time
#define POSITION_UPDATE_CYCLE_TIME_S (POSITION_UPDATE_CYCLE_TIME_MS / 1000.0)

// Structure to hold position update data
typedef struct {
    double current_position;    // Current position in user units
    double target_position;     // Target position in user units
    double max_acceleration;    // Maximum acceleration in user units per second squared
    double max_velocity;        // Maximum velocity in user units per second
    double current_velocity;    // Current velocity in user units per second
    int motion_complete;        // Flag to indicate if motion is complete
} PositionUpdateData;

// Function prototypes
void position_update_init(PositionUpdateData *data, double initial_position);
void position_update_set_target(PositionUpdateData *data, double target_position);
void position_update_set_max_acceleration(PositionUpdateData *data, double max_acceleration);
void position_update_set_max_velocity(PositionUpdateData *data, double max_velocity);
void position_update(PositionUpdateData *data);

#endif // POSITION_UPDATE_H
