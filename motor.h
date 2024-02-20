#ifndef MOTOR_H
#define MOTOR_H

#include "configuration.h"
#include "encoder.h"
#include "pos_pid.h"
#include "velocity_pid.h"
#include "l298n.h"
#include "velocity_pid.h"

typedef enum {
  POSITION,
  VELOCITY
} ControlMode;

typedef struct {
    float desired_position;
    float current_position;
    float last_position;
    float desired_velocity; // Desired velocity
    float current_velocity; // Current velocity based on encoder readings
    ControlMode controlMode;
    L298N l298n;
    Encoder encoder;
    POS_PID pos_pid;
    VEL_PID vel_pid;
    int ticksPerTurn;
    float wheelDiameter;
    float distancePerTick;
    int64_t lastUpdateTime;
    int direction;
} Motor;

#ifdef __cplusplus
extern "C" {
#endif

// Function prototypes
void initMotor(Motor *motor, Encoder enc ,L298N driver, POS_PID pos_pid, VEL_PID vel_pid ,int direction);
void computeVelocity(Motor *motor);
void updateMotor(Motor *motor);
void motor_step(Motor *motor);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_H
