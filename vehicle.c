#include "vehicle.h"
#include <math.h>

#define DT 0.01  // Sample time, you might need to adjust this
#define WHEELS_BASE 2.5  // Distance between front and rear axles, adjust this

int64_t lastTime = 0; // Store the last time the position was updated
float lastPosition = 0; // Store the last position

void init_vehicle(Vehicle *vehicle, Motor left_motor, Motor right_motor, Motor steering_wheel) {
    vehicle->x = 0.0;
    vehicle->y = 0.0;
    vehicle->heading = 0.0;
    vehicle->v = 0.0;
    vehicle->left_motor = left_motor;
    vehicle->right_motor = right_motor;
    vehicle->steering_wheel = steering_wheel;
}

void compute_odometry(Vehicle *vehicle, float throttle_change, float steering_change) {
    computeVelocity(vehicle);  // Compute the velocity based on encoder readings

    float rear_x_delta = vehicle->v * cos(vehicle->heading) * DT;
    float rear_y_delta = vehicle->v * sin(vehicle->heading) * DT;

    vehicle->heading += vehicle->v / WHEELS_BASE * tan(steering_change) * DT;

    // Adjusting for center of the car
    vehicle->x += rear_x_delta + 0.5 * WHEELS_BASE * cos(vehicle->heading);
    vehicle->y += rear_y_delta + 0.5 * WHEELS_BASE * sin(vehicle->heading);
}

void computeVelocity(Vehicle *vehicle) {
    float currentPosition = (vehicle->left_motor.current_position + vehicle->right_motor.current_position) / 2.0; // Average position of both motors
    int64_t currentTime = esp_timer_get_time(); // Get the current time in microseconds

    float deltaPosition = currentPosition - lastPosition;
    int64_t deltaTime = currentTime - lastTime; // Time difference in microseconds

    float dt = deltaTime / 1000000.0; // Convert time difference to seconds
    vehicle->v = deltaPosition / dt;

    // Update the last time and position for the next iteration
    lastTime = currentTime;
    lastPosition = currentPosition;
}
