#ifndef MOTOR_H
#define MOTOR_H

#include "encoder.h"
#include "pos_pid.h"
#include "l298n.h"
#include "velocity_pid.h"

typedef enum {
  POS_PID,
  VELOCITY_PID
} ControlMode;

typedef struct {
    float desired_position;
    float current_position;
    float last_position;
    float desired_velocity; // Desired velocity
    float current_velocity; // Current velocity based on encoder readings
    ControlMode controlMode;
    Encoder encoder;
    L298N l298n;
    POS_PID pos_pid;
    VEL_PID vel_pid;
    int ticksPerTurn;
    float wheelDiameter;
    float distancePerTick;
    int64_t lastUpdateTime;
} Motor;

// Function prototypes
void initMotor(Motor *motor, Encoder enc, L298N driver, ControlMode control_Mode, POS_PID pos_pid, VEL_PID vel_pid, int encoderPinA, int encoderPinB, int l298nENA, int l298nIN1, int l298nIN2);
void computeVelocity(Motor *motor);
void updateMotor(Motor *motor);
void motor_step(Motor *motor);
#endif // MOTOR_H
