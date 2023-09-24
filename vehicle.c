#include "vehicle.h"
#include "configuration.h"
#include <math.h>



void init_vehicle(Vehicle *vehicle, Motor left_front_motor, Motor right_front_motor,  Motor left_rear_motor, Motor right_rear_motor) {
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

    vehicle->left_front_motor = left_front_motor;
    vehicle->right_front_motor = right_front_motor;
    vehicle->left_rear_motor = left_rear_motor;
    vehicle->right_rear_motor = right_rear_motor;
    

    vehicle->vehicle_width = VEHICLE_WIDTH;
    vehicle->vehicle_length = VEHICLE_LENGTH;

    vehicle->odometry_variance.static_error = ENCODER_ERROR **2;
}

// void compute_odometry(Vehicle *vehicle) {
//     // Compute the steering change
//     float steering_change = vehicle->steering_motor->current_position - vehicle->steering_motor->last_position;

//     // Update the last position of the steering motor
//     vehicle->steering_motor->last_position = vehicle->steering_motor->current_position;

//     float rear_x_delta = vehicle->velocity * cos(vehicle->heading) * DT;
//     float rear_y_delta = vehicle->velocity * sin(vehicle->heading) * DT;

//     vehicle->heading += vehicle->velocity / vehicle->vehicle_length * tan(steering_change) * DT;
//     vehicle->velocity = (vehicle->left_motor->current_velocity + vehicle->right_motor->current_velocity) / 2.0;

//     // Adjusting for center of the car
//     vehicle->x += rear_x_delta + 0.5 * vehicle->vehicle_length * cos(vehicle->heading);
//     vehicle->y += rear_y_delta + 0.5 * vehicle->vehicle_length * sin(vehicle->heading);
// }



void compute_odometry(Vehicle *vehicle) {
    // Extracting wheel angular velocities
    float omega_fl = vehicle->left_front_motor.current_velocity;
    float omega_fr = vehicle->right_front_motor.current_velocity;
    float omega_rl = vehicle->left_rear_motor.current_velocity;
    float omega_rr = vehicle->right_rear_motor.current_velocity;
    
    // Wheel radius
    float r = vehicle->left_front_motor.wheelDiameter / 2.0; 
    float lx_ly_sum = vehicle->vehicle_width + vehicle->vehicle_length;

    // Calculating base velocities using forward kinematics
    vehicle->current_state.velocity.x = r/4.0 * (omega_fl + omega_fr + omega_rl + omega_rr);
    vehicle->current_state.velocity.y = r/4.0 * (-omega_fl + omega_fr + omega_rl - omega_rr);
    vehicle->current_state.velocity.angular = r/(4.0 * lx_ly_sum) * (-omega_fl + omega_fr - omega_rl + omega_rr);
    
    // Computing odometry (integrating velocities to get position)
    vehicle->current_state.position.x += vehicle->current_state.velocity.x * ODOMETRY_DT;
    vehicle->current_state.position.y += vehicle->current_state.velocity.y * ODOMETRY_DT;
    vehicle->current_state.position.angular += vehicle->current_state.velocity.angular * ODOMETRY_DT;
    compute_variance(vehicle);
}



void translate_twist_to_motor_commands(Vehicle *vehicle) {
    // Wheel radius
    float r = vehicle->left_front_motor.wheelDiameter / 2.0; 
    float lx_ly_sum = vehicle->vehicle_width + vehicle->vehicle_length;

    // Calculating wheel angular velocities using inverse kinematics
    vehicle->left_front_motor.desired_velocity = (1.0/r) * (vehicle->desired_state.velocity.x - vehicle->desired_state.velocity.y - lx_ly_sum * vehicle->desired_state.velocity.angular);
    vehicle->right_front_motor.desired_velocity = (1.0/r) * (vehicle->desired_state.velocity.x + vehicle->desired_state.velocity.y + lx_ly_sum * vehicle->desired_state.velocity.angular);
    vehicle->left_rear_motor.desired_velocity = (1.0/r) * (vehicle->desired_state.velocity.x + vehicle->desired_state.velocity.y - lx_ly_sum * vehicle->desired_state.velocity.angular);
    vehicle->right_rear_motor.desired_velocity = (1.0/r) * (vehicle->desired_state.velocity.x - vehicle->desired_state.velocity.y + lx_ly_sum * vehicle->desired_state.velocity.angular);
}



// void compute_variance(Vehicle *vehicle) {   //skid steering logic
//     // Base variance based on encoder error for wheels and steering


//     // Calculate the differences between current and last odometry states
//     float x_difference = vehicle->current_state.x - vehicle->last_state.x;
//     float y_difference = vehicle->current_state.y - vehicle->last_state.y;
//     float heading_difference = vehicle->current_state.heading - vehicle->last_state.heading;
//     float velocity_x_difference = vehicle->current_state.velocity_x - vehicle->last_state.velocity_x;
//     float velocity_y_difference = vehicle->current_state.velocity_y - vehicle->last_state.velocity_y;
//     float angular_velocity_difference = vehicle->current_state.angular_velocity - vehicle->last_state.angular_velocity;

//     // Compute variances based on differences
//     vehicle->odometry_variance.x = vehicle->odometry_variance.static_error * x_difference;
//     vehicle->odometry_variance.y = vehicle->odometry_variance.static_error * y_difference;
//     vehicle->odometry_variance.heading = steering_encoder_variance * heading_difference;
//     vehicle->odometry_variance.velocity_x = vehicle->odometry_variance.static_error * velocity_x_difference;
//     vehicle->odometry_variance.velocity_y = vehicle->odometry_variance.static_error * velocity_y_difference;
//     vehicle->odometry_variance.angular_velocity = vehicle->odometry_variance.static_error * angular_velocity_difference;
// }




void compute_variance(Vehicle *vehicle) {
    // Calculate the differences between current and last odometry states
    float x_difference = vehicle->current_state.position.x - vehicle->last_state.position.x;
    float y_difference = vehicle->current_state.position.y - vehicle->last_state.position.y;
    float angular_difference = vehicle->current_state.position.angular - vehicle->last_state.position.angular;
    float velocity_x_difference = vehicle->current_state.velocity.x - vehicle->last_state.velocity.x;
    float velocity_y_difference = vehicle->current_state.velocity.y - vehicle->last_state.velocity.y;
    float angular_velocity_difference = vehicle->current_state.velocity.angular - vehicle->last_state.velocity.angular;

    // Compute variances based on differences and static error
    vehicle->odometry_variance.position_error.x = vehicle->odometry_variance.static_error * x_difference;
    vehicle->odometry_variance.position_error.y = vehicle->odometry_variance.static_error * y_difference;
    vehicle->odometry_variance.position_error.angular = vehicle->odometry_variance.static_error * angular_difference;
    vehicle->odometry_variance.velocity_error.x = vehicle->odometry_variance.static_error * velocity_x_difference;
    vehicle->odometry_variance.velocity_error.y = vehicle->odometry_variance.static_error * velocity_y_difference;
    vehicle->odometry_variance.velocity_error.angular = vehicle->odometry_variance.static_error * angular_velocity_difference;
}



