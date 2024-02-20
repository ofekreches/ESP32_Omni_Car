#include "velocity_pid.h"
#include "configuration.h"


void initVelPID(VEL_PID *pid, float kp , float ki , float kd, float i_windup) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_windup = i_windup;
    pid->control_signal = 0;
    pid->last_error = 0;
    pid->integral = 0;
}

float vel_pid_step(VEL_PID *pid, float desired_velocity, float current_velocity) {
    float error = desired_velocity - current_velocity;
    pid->integral += error;
    // Anti-windup
    if (pid->integral > pid->i_windup) pid->integral = pid->i_windup;
    if (pid->integral < -pid->i_windup) pid->integral = -pid->i_windup;

    float derivative = error - pid->last_error;

    pid->control_signal = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    pid->last_error = error;

    return pid->control_signal;
}
