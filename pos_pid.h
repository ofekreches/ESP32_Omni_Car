#ifndef POS_PID_H
#define POS_PID_H

typedef struct {
  float kp;
  float ki;
  float kd;
  float i_windup; // Maximum allowed integral value
  float control_signal;
  float last_error;
  float integral;
} POS_PID;

// Function prototypes
void initPosPID(POS_PID *pid, float kp , float ki , float kd, float i_windup);
float pos_pid_step(POS_PID *pid, float desired_position, float current_position);

#endif // POS_PID_H
