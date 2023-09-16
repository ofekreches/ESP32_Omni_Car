#ifndef VEHICLE_H
#define VEHICLE_H

#include "motor.h"
#include "configuration.h"

typedef struct {
    float x;
    float y;
    float angular;  //heading in position
} vector_3;

typedef struct {    
    vector_3 position;
    vector_3 velocity;
} Odometry;


typedef struct {
    vector_3 position_error;
    vector_3 velocity_error;
    float static_error;  // the encoder error (right now its an assumption) . TODO dynamic error - based on experiments in the future
} Variance;

typedef struct {   //omni wheels version
    float vehicle_width;
    float vehicle_length;
    Odometry desired_state;
    Odometry current_state;
    Odometry last_state;
    Variance odometry_variance;  
    Motor left_front_motor;
    Motor right_front_motor;
    Motor left_rear_motor;
    Motor right_rear_motor;
    Motor steering_wheel;
} Vehicle;


// typedef struct {   //skid steering version
//     float vehicle_width;
//     float vehicle_length;
//     Odometry desired_state;
//     Odometry current_state;
//     Odometry last_state;
//     Variance odometry_variance;  
//     Motor left_motor;
//     Motor right_motor;
//     Motor steering_wheel;
// } Vehicle;






void init_vehicle(Vehicle *vehicle, Motor left_front_motor, Motor right_front_motor,  Motor left_rear_motor, Motor right_rear_motor);
void compute_odometry(Vehicle *vehicle);
void compute_variance(Vehicle *vehicle);
void translate_twist_to_motor_commands(Vehicle *vehicle);

#endif // VEHICLE_H
