#ifndef VEHICLE_H
#define VEHICLE_H

#include "motor.h"
#include "configuration.h"

typedef struct {
    float vehicle_width;
    float vehicle_length;
    float current_x;  // refrence point is decided to be the center of the car
    float current_y; 
    float current_angular_velocity; 
    float current_heading;
    float desired_velocity;
    float desired_heading;
    float velocity;
    Motor left_motor;
    Motor right_motor;
    Motor steering_wheel;
} Vehicle;

void init_vehicle(Vehicle *vehicle, Motor left_motor, Motor right_motor, Motor steering_wheel);
void compute_odometry(Vehicle *vehicle;
void move(Vehicle *vehicle);

#endif // VEHICLE_H
