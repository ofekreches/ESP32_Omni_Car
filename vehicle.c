#include "vehicle.h"
#include <math.h>

#define DT 0.01  // Sample time, you might need to adjust this
#define WHEELS_BASE 2.5  // Distance between front and rear axles, adjust this

void init_vehicle(Vehicle *vehicle, Motor left_motor, Motor right_motor, Motor steering_wheel) {
    vehicle->x = 0.0;
    vehicle->y = 0.0;
    vehicle->heading = 0.0;
    vehicle->v = 0.0;
    vehicle->left_motor = left_motor;
    vehicle->right_motor = right_motor;
    vehicle->steering_wheel = steering_wheel;
}

void compute_odometry(Vehicle *vehicle) {
    float rear_x_delta = vehicle->v * cos(vehicle->heading) * DT;
    float rear_y_delta = vehicle->v * sin(vehicle->heading) * DT;

    vehicle->heading += vehicle->v / WHEELS_BASE * tan(steering_change) * DT;
    vehicle->v += throttle_change * DT;

    // Adjusting for center of the car
    vehicle->x += rear_x_delta + 0.5 * WHEELS_BASE * cos(vehicle->heading);
    vehicle->y += rear_y_delta + 0.5 * WHEELS_BASE * sin(vehicle->heading);
}


