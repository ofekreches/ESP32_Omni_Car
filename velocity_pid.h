#ifndef VEL_PID_H
#define VEL_PID_H

typedef struct {
  float kp;
  float ki;
  float kd;
  float i_windup; // Maximum allowed integral value
  float control_signal;
  float last_error;
  float integral;
} VEL_PID;

// Function prototypes
void initVelPID(VEL_PID *pid, float kp , float ki , float kd, float i_windup);
float vel_pid_step(VEL_PID *pid, float desired_velocity, float current_velocity);

#endif // VEL_PID_H
