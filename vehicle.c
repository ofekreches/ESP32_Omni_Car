#include "vehicle.h"
#include <math.h>

#define DT 0.01  // Sample time, you might need to adjust this


void init_vehicle(Vehicle *vehicle, Motor left_motor, Motor right_motor, Motor steering_wheel) {
    vehicle->current_x = 0.0;
    vehicle->current_y = 0.0;
    vehicle->current_heading = 0.0;
    vehicle->v = 0.0;
    vehicle->left_motor = left_motor;
    vehicle->right_motor = right_motor;
    vehicle->steering_wheel = steering_wheel;
    vehicle->vehicle_width = VEHICLE_WIDTH;
    vehicle->vehicle_length = VEHICLE_LENGTH;
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


void move(Vehicle *vehicle){  //updating desired velocity/position to different motors based on the received message from serial
    vehicle->left_motor->desired_velocity
    vehicle->right_motor->desired_velocity
} 
