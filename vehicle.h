#ifndef VEHICLE_H
#define VEHICLE_H

#include "motor.h"
#include "configuration.h"
#include "pos_pid.h"
#include "velocity_pid.h"


typedef struct {
    float x;
    float y;
    float angular;  //heading in position
} vector_3;


typedef struct {    
    vector_3 position;
    vector_3 velocity;
    Variance odometry_variance;
    int64_t time_stamp;
} Odometry;


typedef struct {
    vector_3 position_error;
    vector_3 velocity_error;
    float static_error;  // the encoder error (right now its an assumption) . TODO dynamic error - based on experiments in the future
} Variance;

typedef struct {
    VEL_PID velocity_pid_x;
    VEL_PID velocity_pid_y;
    VEL_PID velocity_pid_heading;
    POS_PID pos_pid_x;
    POS_PID pos_pid_y;
    POS_PID pos_pid_heading;
} Vehicle_PIDs;


typedef struct {   //omni wheels version
    float vehicle_width;
    float vehicle_length;
    Odometry desired_state;
    Odometry current_state;
    Odometry last_state;
    Odometry signal_state;  //for storing the control signal being outputed from the pids
    Motor left_front_motor;
    Motor right_front_motor;
    Motor left_rear_motor;
    Motor right_rear_motor;
    Vehicle_PIDs vehicle_pids;
} Vehicle;




void init_vehicle_pids(Vehicle_PIDs *vehicle_pids , VEL_PID velocity_pid_x ,  VEL_PID velocity_pid_y, VEL_PID velocity_pid_heading , POS_PID pos_pid_x , POS_PID pos_pid_y , POS_PID pos_pid_heading);
void init_vehicle(Vehicle *vehicle, Motor left_front_motor, Motor right_front_motor, Motor left_rear_motor, Motor right_rear_motor, Vehicle_PIDs vehicle_pids);
void compute_odometry_from_encoders(Vehicle *vehicle);
void compute_variance_from_encoders(Vehicle *vehicle);
void translate_twist_to_motor_commands(Vehicle *vehicle);
void vehicle_step(Vehicle *vehicle);

#endif // VEHICLE_H
