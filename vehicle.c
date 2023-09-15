#include "vehicle.h"
#include "configuration.h"
#include <math.h>

#define DT 0.01  // Sample time, you might need to adjust this - will be linked later to odometry calculation timer


void init_vehicle(Vehicle *vehicle, Motor left_motor, Motor right_motor, Motor steering_wheel) {
    vehicle->current_state.x = 0.0;
    vehicle->current_state.y = 0.0;
    vehicle->current_state.heading = 0.0;
    vehicle->current_state.velocity_x = 0.0;
    vehicle->current_state.velocity_y = 0.0;
    vehicle->current_state.angular_velocity = 0.0;

    vehicle->last_state.x = 0.0;
    vehicle->last_state.y = 0.0;
    vehicle->last_state.heading = 0.0;
    vehicle->last_state.velocity_x = 0.0;
    vehicle->last_state.velocity_y = 0.0;
    vehicle->last_state.angular_velocity = 0.0;

    vehicle->desired_state.x = 0.0;
    vehicle->desired_state.y = 0.0;
    vehicle->desired_state.heading = 0.0;
    vehicle->desired_state.velocity_x = 0.0;
    vehicle->desired_state.velocity_y = 0.0;
    vehicle->desired_state.angular_velocity = 0.0;

    vehicle->left_motor = left_motor;
    vehicle->right_motor = right_motor;
    vehicle->steering_wheel = steering_wheel;

    vehicle->vehicle_width = VEHICLE_WIDTH;
    vehicle->vehicle_length = VEHICLE_LENGTH;

    vehicle->odometry_variance.static_error = ENCODER_ERROR * ENCODER_ERROR;
}

void compute_odometry(Vehicle *vehicle) {
    // Compute the steering change
    float steering_change = vehicle->steering_motor->current_position - vehicle->steering_motor->last_position;

    // Update the last position of the steering motor
    vehicle->steering_motor->last_position = vehicle->steering_motor->current_position;

    float rear_x_delta = vehicle->velocity * cos(vehicle->heading) * DT;
    float rear_y_delta = vehicle->velocity * sin(vehicle->heading) * DT;

    vehicle->heading += vehicle->velocity / vehicle->vehicle_length * tan(steering_change) * DT;
    vehicle->velocity = (vehicle->left_motor->current_velocity + vehicle->right_motor->current_velocity) / 2.0;

    // Adjusting for center of the car
    vehicle->x += rear_x_delta + 0.5 * vehicle->vehicle_length * cos(vehicle->heading);
    vehicle->y += rear_y_delta + 0.5 * vehicle->vehicle_length * sin(vehicle->heading);
}



void compute_variance(Vehicle *vehicle) {
    // Base variance based on encoder error for wheels and steering


    // Calculate the differences between current and last odometry states
    float x_difference = vehicle->current_state.x - vehicle->last_state.x;
    float y_difference = vehicle->current_state.y - vehicle->last_state.y;
    float heading_difference = vehicle->current_state.heading - vehicle->last_state.heading;
    float velocity_x_difference = vehicle->current_state.velocity_x - vehicle->last_state.velocity_x;
    float velocity_y_difference = vehicle->current_state.velocity_y - vehicle->last_state.velocity_y;
    float angular_velocity_difference = vehicle->current_state.angular_velocity - vehicle->last_state.angular_velocity;

    // Compute variances based on differences
    vehicle->odometry_variance.x = vehicle->odometry_variance.static_error * x_difference;
    vehicle->odometry_variance.y = vehicle->odometry_variance.static_error * y_difference;
    vehicle->odometry_variance.heading = steering_encoder_variance * heading_difference;
    vehicle->odometry_variance.velocity_x = vehicle->odometry_variance.static_error * velocity_x_difference;
    vehicle->odometry_variance.velocity_y = vehicle->odometry_variance.static_error * velocity_y_difference;
    vehicle->odometry_variance.angular_velocity = vehicle->odometry_variance.static_error * angular_velocity_difference;
}


void move(Vehicle *vehicle){  //updating desired velocity/position to different motors based on the received message from serial
    vehicke->desired_state->velocity_x = vehicle->left_motor->desired_velocity
    vehicle->right_motor->desired_velocity
} 
