#include "stepgen.h"

// Initialize the position update system with an initial position
void position_update_init(PositionUpdateData *data, double initial_position) {
    data->current_position = initial_position;
    data->target_position = initial_position;
    data->max_acceleration = 0.0;  // Default to 0, must be set separately
    data->max_velocity = 0.0;      // Default to 0, must be set separately
    data->current_velocity = 0.0;
    data->motion_complete = 1;     // Start with motion complete
}

// Set the target position
void position_update_set_target(PositionUpdateData *data, double target_position) {
    data->target_position = target_position;
    data->motion_complete = 0;    // Motion is not complete
}

// Set the maximum acceleration
void position_update_set_max_acceleration(PositionUpdateData *data, double max_acceleration) {
    if (max_acceleration >= 0) {
        data->max_acceleration = max_acceleration;
    } else {
        data->max_acceleration = 0.0;  // Clamp to 0 if negative
    }
}

// Set the maximum velocity
void position_update_set_max_velocity(PositionUpdateData *data, double max_velocity) {
    if (max_velocity >= 0) {
        data->max_velocity = max_velocity;
    } else {
        data->max_velocity = 0.0;  // Clamp to 0 if negative
    }
}

// Update the position using a trapezoidal motion profile
void position_update(PositionUpdateData *data) {
    if (data->motion_complete) {
        return;  // No motion to perform
    }

    double distance_to_target = data->target_position - data->current_position;
    double direction = (distance_to_target > 0) ? 1.0 : -1.0;
    double max_velocity_change = data->max_acceleration * POSITION_UPDATE_CYCLE_TIME_S;

    // Calculate the distance required to decelerate to zero velocity
    double deceleration_distance = (data->current_velocity * data->current_velocity) / (2.0 * data->max_acceleration);

    // Calculate the desired velocity based on the distance to the target
    double desired_velocity;
    if (fabs(distance_to_target) <= deceleration_distance) {
        // Deceleration phase: reduce velocity to stop at the target
        desired_velocity = sqrt(2.0 * data->max_acceleration * fabs(distance_to_target)) * direction;
    } else {
        // Acceleration or constant velocity phase
        desired_velocity = direction * data->max_velocity;
    }

    // Limit the velocity change based on the maximum acceleration
    if (desired_velocity > data->current_velocity + max_velocity_change) {
        data->current_velocity += max_velocity_change;
    } else if (desired_velocity < data->current_velocity - max_velocity_change) {
        data->current_velocity -= max_velocity_change;
    } else {
        data->current_velocity = desired_velocity;
    }

    // Update the position based on the current velocity
    data->current_position += data->current_velocity * POSITION_UPDATE_CYCLE_TIME_S;

    // Check if the target position is reached
    if (fabs(distance_to_target) < 1e-6) {
        data->current_position = data->target_position;
        data->current_velocity = 0.0;
        data->motion_complete = 1;  // Motion is complete
    }
}
