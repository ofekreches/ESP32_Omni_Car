#include "motor.h"
#include "configuration.h"
#include "encoder.h"
#include "pos_pid.h"
#include "l298n.h"

void initMotor(Motor *motor, Encoder enc, L298N driver, ControlMode control_Mode) {
    motor->desired_position = 0;
    motor->current_position = 0;
    motor->controlMode = control_Mode;
    motor->encoder = enc;
    motor->l298n = driver;
    motor->ticksPerTurn = TICKS_PER_TURN;
    motor->wheelDiameter = WHEEL_DIAMETER;
    motor->distancePerTick = motor->wheelDiameter * PI / motor->ticksPerTurn;

    initEncoder(motor->encoder, ENCODER_PIN_A, ENCODER_PIN_B);
    initL298N(motor->l298n, iL298N_ENA, L298N_IN1, L298N_IN2);


}
