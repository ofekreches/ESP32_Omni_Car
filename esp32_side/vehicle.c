#include <Arduino.h>
#include <math.h>
#include <esp_timer.h>
#include "vehicle.h"
#include "configuration.h"


void init_vehicle(Vehicle *vehicle, Motor left_front_motor, Motor right_front_motor, Motor left_rear_motor, Motor right_rear_motor, Vehicle_PIDs vehicle_pids) {
    vehicle->current_state.odometry_variance.static_error = ENCODER_ERROR * ENCODER_ERROR;
    vehicle->current_state.odometry_variance.position_error.x = 0.0;
    vehicle->current_state.odometry_variance.position_error.y = 0.0;
    vehicle->current_state.odometry_variance.position_error.angular = 0.0;
    vehicle->current_state.odometry_variance.velocity_error.x = 0.0;
    vehicle->current_state.odometry_variance.velocity_error.y = 0.0;
    vehicle->current_state.odometry_variance.velocity_error.angular = 0.0;

    vehicle->current_state.time_stamp = 0;

    vehicle->current_state.position.x = 0.0;
    vehicle->current_state.position.y = 0.0;
    vehicle->current_state.position.angular = 0.0;
    vehicle->current_state.velocity.x = 0.0;
    vehicle->current_state.velocity.y = 0.0;
    vehicle->current_state.velocity.angular = 0.0;

    vehicle->last_state.position.x = 0.0;
    vehicle->last_state.position.y = 0.0;
    vehicle->last_state.position.angular = 0.0;
    vehicle->last_state.velocity.x = 0.0;
    vehicle->last_state.velocity.y = 0.0;
    vehicle->last_state.velocity.angular = 0.0;
    vehicle->last_state.time_stamp = 0;

    vehicle->desired_state.position.x = 0.0;
    vehicle->desired_state.position.y = 0.0;
    vehicle->desired_state.position.angular = 0.0;
    vehicle->desired_state.velocity.x = 0.0;
    vehicle->desired_state.velocity.y = 0.0;
    vehicle->desired_state.velocity.angular = 0.0;
    vehicle->desired_state.time_stamp = 0;

    vehicle->signal_state.position.x = 0.0;
    vehicle->signal_state.position.y = 0.0;
    vehicle->signal_state.position.angular = 0.0;
    vehicle->signal_state.velocity.x = 0.0;
    vehicle->signal_state.velocity.y = 0.0;
    vehicle->signal_state.velocity.angular = 0.0;
    vehicle->signal_state.time_stamp = 0;

    vehicle->left_front_motor = left_front_motor;   
    vehicle->right_front_motor = right_front_motor;
    vehicle->left_rear_motor = left_rear_motor;
    vehicle->right_rear_motor = right_rear_motor;
    
    vehicle->vehicle_width = VEHICLE_WIDTH;
    vehicle->vehicle_length = VEHICLE_LENGTH;
}


void init_vehicle_pids(Vehicle_PIDs *vehicle_pids , VEL_PID velocity_pid_x ,  VEL_PID velocity_pid_y, VEL_PID velocity_pid_angular , POS_PID pos_pid_x , POS_PID pos_pid_y , POS_PID pos_pid_angular) {
    vehicle_pids->velocity_pid_x = velocity_pid_x;
    vehicle_pids->velocity_pid_y = velocity_pid_y;
    vehicle_pids->velocity_pid_angular = velocity_pid_angular;

    vehicle_pids->pos_pid_x = pos_pid_x;
    vehicle_pids->pos_pid_y = pos_pid_y;
    vehicle_pids->pos_pid_angular = pos_pid_angular;
}


void compute_odometry_from_encoders(Vehicle *vehicle) {
    vehicle->last_state = vehicle->current_state;

    // Wheel radius
    float r = vehicle->left_front_motor.wheelDiameter / 2.0; 
    float wheel_base = vehicle->vehicle_width;

    // Calculating base velocities using forward kinematics
    float v_x = r / 2.0 * (vehicle->left_front_motor.current_velocity + 
                           vehicle->right_front_motor.current_velocity);

    float v_angular = r / wheel_base * (vehicle->right_front_motor.current_velocity - 
                                        vehicle->left_front_motor.current_velocity);

    // Get the current time in microseconds
    vehicle->current_state.time_stamp = esp_timer_get_time(); 
    float dt = (vehicle->current_state.time_stamp - vehicle->last_state.time_stamp) / 1000000.0;

    // Convert velocities to world frame
    float heading = vehicle->last_state.position.angular;
    float cos_heading = cos(heading);
    float sin_heading = sin(heading);

    // World frame velocities
    float world_v_x = v_x * cos_heading;
    float world_v_y = v_x * sin_heading;

    // Update current state velocities
    vehicle->current_state.velocity.x = world_v_x;
    vehicle->current_state.velocity.y = world_v_y;
    vehicle->current_state.velocity.angular = v_angular;

    // Integrate to update position
    vehicle->current_state.position.x += world_v_x * dt;
    vehicle->current_state.position.y += world_v_y * dt;
    vehicle->current_state.position.angular += v_angular * dt;

    compute_variance_from_encoders(vehicle);
}



void translate_twist_to_motor_commands(Vehicle *vehicle) {
    // Wheel radius
    float r = vehicle->left_front_motor.wheelDiameter / 2.0;
    float wheel_base = vehicle->vehicle_width;

    // Desired linear and angular velocities
    float v_x = vehicle->desired_state.velocity.x;
    float v_angular = vehicle->desired_state.velocity.angular;

    // Calculating wheel angular velocities using inverse kinematics
    vehicle->left_front_motor.desired_velocity = (1.0 / r) * (v_x - (wheel_base / 2.0) * v_angular);
    vehicle->right_front_motor.desired_velocity = (1.0 / r) * (v_x + (wheel_base / 2.0) * v_angular);
}





void compute_variance_from_encoders(Vehicle *vehicle) {
    // Calculate the differences between current and last odometry states
    float x_difference = vehicle->current_state.position.x - vehicle->last_state.position.x;
    float y_difference = vehicle->current_state.position.y - vehicle->last_state.position.y;
    float angular_difference = vehicle->current_state.position.angular - vehicle->last_state.position.angular;
    float velocity_x_difference = vehicle->current_state.velocity.x - vehicle->last_state.velocity.x;
    float velocity_y_difference = vehicle->current_state.velocity.y - vehicle->last_state.velocity.y;
    float angular_velocity_difference = vehicle->current_state.velocity.angular - vehicle->last_state.velocity.angular;

    // Compute variances based on differences and static error
    vehicle->current_state.odometry_variance.position_error.x += vehicle->current_state.odometry_variance.static_error * x_difference;
    vehicle->current_state.odometry_variance.position_error.y += vehicle->current_state.odometry_variance.static_error * y_difference;
    vehicle->current_state.odometry_variance.position_error.angular += vehicle->current_state.odometry_variance.static_error * angular_difference;
    vehicle->current_state.odometry_variance.velocity_error.x += vehicle->current_state.odometry_variance.static_error * velocity_x_difference;
    vehicle->current_state.odometry_variance.velocity_error.y += vehicle->current_state.odometry_variance.static_error * velocity_y_difference;
    vehicle->current_state.odometry_variance.velocity_error.angular += vehicle->current_state.odometry_variance.static_error * angular_velocity_difference;
}


void vehicle_step(Vehicle *vehicle) {
    compute_odometry_from_encoders(vehicle);
    vehicle->signal_state.velocity.x = vel_pid_step(&vehicle->vehicle_pids.velocity_pid_x, vehicle->desired_state.velocity.x, vehicle->current_state.velocity.x);
    vehicle->signal_state.velocity.y = vel_pid_step(&vehicle->vehicle_pids.velocity_pid_y, vehicle->desired_state.velocity.y, vehicle->current_state.velocity.y);
    vehicle->signal_state.velocity.angular = vel_pid_step(&vehicle->vehicle_pids.velocity_pid_angular, vehicle->desired_state.velocity.angular, vehicle->current_state.velocity.angular);
    translate_twist_to_motor_commands(vehicle);
}


