#ifndef VEHICLE_H
#define VEHICLE_H

#include "motor.h"
#include <esp_timer.h>

typedef struct {
    float x;  // Center of the car
    float y;  // Center of the car
    float heading;
    float v;
    Motor left_motor;
    Motor right_motor;
    Motor steering_wheel;
    float vehicle_width;
    float vehicle_length;
} Vehicle;

void init_vehicle(Vehicle *vehicle, Motor left_motor, Motor right_motor, Motor steering_wheel);
void compute_odometry(Vehicle *vehicle, float throttle_change, float steering_change);
void computeVelocity(Vehicle *vehicle);  // Added this function prototype

#endif // VEHICLE_H
