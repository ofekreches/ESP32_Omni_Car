#include "motor.h"
#include "configuration.h"
#include <esp_timer.h>


void initMotor(Motor *motor, Encoder enc, L298N driver, ControlMode control_Mode, int encoderPinA, int encoderPinB, int l298nENA, int l298nIN1, int l298nIN2) {
    motor->desired_position = 0;
    motor->current_position = 0;
    motor->desired_velocity = 0;
    motor->current_velocity = 0;
    motor->controlMode = control_Mode;
    motor->encoder = enc;
    motor->l298n = driver;
    motor->ticksPerTurn = TICKS_PER_TURN;
    motor->wheelDiameter = WHEEL_DIAMETER;
    motor->distancePerTick = motor->wheelDiameter * PI / motor->ticksPerTurn;
    initEncoder(&motor->encoder, encoderPinA, encoderPinB);
    initL298N(&motor->l298n, l298nENA, l298nIN1, l298nIN2);
    motor->lastUpdateTime = esp_timer_get_time();
}

void computeVelocity(Motor *motor) {
    int64_t currentTime = esp_timer_get_time(); // Get the current time in microseconds
    float deltaPosition = motor->current_position - motor->desired_position;
    int64_t deltaTime = currentTime - motor->lastUpdateTime; // Time difference in microseconds
    float dt = deltaTime / 1000000.0; // Convert time difference to seconds
    motor->current_velocity = deltaPosition / dt;
    motor->lastUpdateTime = currentTime;
}

void updateMotor(Motor *motor) {
    motor->current_position = getEncoderCount(&motor->encoder) * motor->distancePerTick;  //update position
    computeVelocity(motor); // update velocity
}
