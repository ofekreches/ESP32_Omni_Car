#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// Encoder configuration for 4 motors
#define LF_ENCODER_PIN_A 19
#define LF_ENCODER_PIN_B 18

#define RR_ENCODER_PIN_A 5
#define RR_ENCODER_PIN_B 4

#define LR_ENCODER_PIN_A 15
#define LR_ENCODER_PIN_B 14

#define RF_ENCODER_PIN_A 23
#define RF_ENCODER_PIN_B 22

#define ENCODER_ERROR 0.1

// Motor configuration
#define TICKS_PER_TURN 660
#define WHEEL_DIAMETER 80.0

// L298N configuration for 4 motors
#define LF_L298N_ENA 25
#define LF_L298N_IN1 26
#define LF_L298N_IN2 27

#define RR_L298N_ENA 32
#define RR_L298N_IN1 33
#define RR_L298N_IN2 13

#define LR_L298N_ENA 12
#define LR_L298N_IN1 2
#define LR_L298N_IN2 4

#define RF_L298N_ENA 16
#define RF_L298N_IN1 17
#define RF_L298N_IN2 21

// Pos PID configuration
#define POS_KP 1.0
#define POS_KI 0.01
#define POS_KD 0.1
#define POS_I_WINDUP 1000.0

// Vel PID configuration
#define VEL_KP 1.0
#define VEL_KI 0.01
#define VEL_KD 0.1
#define VEL_I_WINDUP 1000.0

//vehicle dimensions
#define VEHICLE_WIDTH 0.2 // meter
#define VEHICLE_LENGTH 0.3 //meter

//timer configurations
#define ODOMETRY_DT 0.01  // Sample time, you might need to adjust this - will be linked later to odometry calculation timer
#define MOTOR_CONTROL_DT 0.01        // Run the motor task every 10ms
#define COMMUNICATION_DT 0.01  // handle communication every 0.01 seconds = 10 ms

// communication configuration
#define SIZE_OF_RX_DATA 18  // 2 headrs +type of command + vector 3 of commands + check sum + tail
#define SIZE_OF_TX_DATA 52  // 2 headers + odometry + variance + check sum + tail
#define HEADER 200
#define TAIL 199


// tasks configuration
#define MOTOR_CONTROL_CORE  1      // Using Core 1 for Motor Control
#define ODOMETRY_CORE  0        // Using Core 0 for Odometry computation
#define COMMUNICATION_CORE  0        // Using Core 0 for communication handling


// imu configs
#define DEG_TO_RAD (3.14159265358979323846 / 180.0)

#endif // CONFIGURATION_H
