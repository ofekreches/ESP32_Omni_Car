#ifndef MOTOR_H
#define MOTOR_H

#include "encoder.h"
#include "pos_pid.h"
#include "l298n.h"


typedef enum {
  POS_PID,
  VELOCITY_PID
} ControlMode;

typedef struct {
  float desired_position;
  float current_position;
  ControlMode controlMode;
  Encoder encoder;
  L298N l298n;
  int ticksPerTurn;
  float wheelDiameter;
  float distancePerTick;
} Motor;

// Function prototypes
void initMotor(Motor *motor, Encoder enc, L298N driver, ControlMode control_Mode);

#endif // MOTOR_H
