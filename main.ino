#include <ESP32Encoder.h>
#include <esp_timer.h>
#include "motor.h"
#include "vehicle.h"
#include "comm_controller.h"
#include "configuration.h"
#include "pos_pid.h"
#include "velocity_pid.h"
#include "encoder.h"
#include "l298n.h"

POS_PID pos_pid_left_front_motor;
POS_PID pos_pid_right_front_motor;
POS_PID pos_pid_left_rear_motor;
POS_PID pos_pid_right_rear_motor;

VEL_PID vel_pid_left_front_motor;
VEL_PID vel_pid_right_front_motor;
VEL_PID vel_pid_left_rear_motor;
VEL_PID vel_pid_right_rear_motor;

POS_PID pos_pid_x; 
POS_PID pos_pid_y;
POS_PID pos_pid_heading;

VEL_PID vel_pid_x;
VEL_PID vel_pid_y;
VEL_PID vel_pid_heading;

Vehicle_PIDs vehicle_pids;

Encoder encoderLF;
Encoder encoderRR;
Encoder encoderLR;
Encoder encoderRF;

L298N driverLF;
L298N driverRR;
L298N driverLR;
L298N driverRF;

Motor left_front_motor;
Motor right_front_motor;
Motor left_rear_motor;
Motor right_rear_motor;

Vehicle vehicle;

CommController comm;

TaskHandle_t MotorControlTask;
TaskHandle_t vehicleControlTask;
TaskHandle_t CommunicationTask;

SemaphoreHandle_t vehicleMutex;

void setup() {

    // init vehicle PIDs
    initPosPID(&pos_pid_x, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
    initPosPID(&pos_pid_y, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
    initPosPID(&pos_pid_heading, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);

    initVelPID(&vel_pid_x, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);
    initVelPID(&vel_pid_y, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);
    initVelPID(&vel_pid_heading, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);

    // init PIDs for each motor
    initPosPID(&pos_pid_left_front_motor, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
    initVelPID(&vel_pid_left_front_motor, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);

    initPosPID(&pos_pid_right_front_motor, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
    initVelPID(&vel_pid_right_front_motor, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);

    initPosPID(&pos_pid_left_rear_motor, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
    initVelPID(&vel_pid_left_rear_motor, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);

    initPosPID(&pos_pid_right_rear_motor, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
    initVelPID(&vel_pid_right_rear_motor, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);


    init_vehicle_pids(&vehicle_pids, &vel_pid_x, &vel_pid_y, &vel_pid_heading, &pos_pid_x, &pos_pid_y, &pos_pid_heading);

    // init encoders
    initEncoder(&encoderLF, LF_ENCODER_PIN_A, LF_ENCODER_PIN_B);  
    initEncoder(&encoderRR, RR_ENCODER_PIN_A, RR_ENCODER_PIN_B);
    initEncoder(&encoderLR, LR_ENCODER_PIN_A, LR_ENCODER_PIN_B);
    initEncoder(&encoderRF, RF_ENCODER_PIN_A, RF_ENCODER_PIN_B);

    // init motor driver l298n
    initL298N(&driverLF, LF_L298N_ENA, LF_L298N_IN1, LF_L298N_IN2);
    initL298N(&driverRR, RR_L298N_ENA, RR_L298N_IN1, RR_L298N_IN2);
    initL298N(&driverLR, LR_L298N_ENA, LR_L298N_IN1, LR_L298N_IN2);
    initL298N(&driverRF, RF_L298N_ENA, RF_L298N_IN1, RF_L298N_IN2);

    //init motors
    initMotor(&left_front_motor, encoderLF, driverLF, pos_pid_left_front_motor, vel_pid_left_front_motor); 
    initMotor(&right_front_motor, encoderRF, driverRF, pos_pid_right_front_motor, vel_pid_right_front_motor);
    initMotor(&left_rear_motor, encoderLR, driverLR, pos_pid_left_rear_motor, vel_pid_left_rear_motor);
    initMotor(&right_rear_motor, encoderRR, driverRR, pos_pid_right_rear_motor, vel_pid_right_rear_motor);

    //init vehicle
    init_vehicle(&vehicle, left_front_motor, right_front_motor, left_rear_motor, right_rear_motor, vehicle_pids);

    //init communication
    comm_controller_init(&comm);

    // Create Mutex for Vehicle Data
    vehicleMutex = xSemaphoreCreateMutex();

    //tasks init
    xTaskCreatePinnedToCore(
        motorControlTask,       /* Task function */
        "MotorControlTask",     /* Name of task */
        10000,                  /* Stack size of task */
        &vehicle,                   /* Parameter of the task */
        3,                      /* Priority of the task */
        &MotorControlTask,      /* Task handle to keep track of created task */
        MOTOR_CONTROL_CORE);    /* Core where the task should run */


    xTaskCreatePinnedToCore(
        vehicleControlTask,           /* Task function */
        "OdometryTask",         /* Name of task */
        10000,                  /* Stack size of task */
        &vehicle,               /* Parameter of the task - passing vehicle pointer */
        2,                      /* Priority of the task  */
        &OdometryTask,          /* Task handle to keep track of created task */
        ODOMETRY_CORE);         /* Core where the task should run */


    xTaskCreatePinnedToCore(
        CommunicationTask,          /* Task function */
        "CommunicationTask",        /* Name of task */
        10000,                      /* Stack size of task */
        &vehicle,                   /* Parameter of the task - passing vehicle pointer */
        1,                          /* Priority of the task - set lower than motor and odometry tasks for demonstration */
        NULL,                       /* We're not using the task handle here */
        COMMUNICATION_CORE);        /* Core where the task should run */

}



void motorControlTask(void * parameter) {
    Vehicle *vehicle = (Vehicle *)parameter;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(MOTOR_CONTROL_DT * 1000);

    xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if(xSemaphoreTake(vehicleMutex, portMAX_DELAY)) {
            motor_step(&vehicle->left_front_motor);
            motor_step(&vehicle->right_front_motor);
            motor_step(&vehicle->left_rear_motor);
            motor_step(&vehicle->right_rear_motor);
            xSemaphoreGive(vehicleMutex);
        }
    }
}

void vehicleControlTask(void * parameter) {
    Vehicle *vehicle = (Vehicle *)parameter;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(ODOMETRY_DT * 1000);

    xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        if(xSemaphoreTake(vehicleMutex, portMAX_DELAY)) {
            vehicle_step(vehicle);
            xSemaphoreGive(vehicleMutex);
        }
    }
}

void communicationTask(void * parameter) {
    Vehicle *vehicle = (Vehicle *)parameter;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(COMMUNICATION_DT * 1000);

    xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        if(xSemaphoreTake(vehicleMutex, portMAX_DELAY)) {
            receiveData(&comm, vehicle);
            xSemaphoreGive(vehicleMutex);
        }
    }
}

