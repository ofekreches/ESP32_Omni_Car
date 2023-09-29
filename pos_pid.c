#include "pos_pid.h"
#include "configuration.h"

void initPosPID(POS_PID *pid, float kp , float ki , float kd, float i_windup){
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_windup = i_windup;
    pid->control_signal = 0;
    pid->last_error = 0;
    pid->integral = 0;
}

float pos_pid_step(POS_PID *pid, float desired_position, float current_position) {
    float error = desired_position - current_position;
    pid->integral += error;
    // Anti-windup
    if (pid->integral > pid->i_windup) pid->integral = pid->i_windup;
    if (pid->integral < -pid->i_windup) pid->integral = -pid->i_windup;

    float derivative = error - pid->last_error;

    pid->control_signal = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    pid->last_error = error;

    return pid->control_signal;
}
