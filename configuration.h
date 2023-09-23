#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// Encoder configuration for 4 motors
#define ENCODER1_PIN_A 19
#define ENCODER1_PIN_B 18

#define ENCODER2_PIN_A 5
#define ENCODER2_PIN_B 4

#define ENCODER3_PIN_A 15
#define ENCODER3_PIN_B 14

#define ENCODER4_PIN_A 23
#define ENCODER4_PIN_B 22

#define ENCODER_ERROR 0.1

// Motor configuration
#define TICKS_PER_TURN 660
#define WHEEL_DIAMETER 80.0

// L298N configuration for 4 motors
#define L298N1_ENA 25
#define L298N1_IN1 26
#define L298N1_IN2 27

#define L298N2_ENA 32
#define L298N2_IN1 33
#define L298N2_IN2 13

#define L298N3_ENA 12  // OK, but ensure it doesn't interfere with boot if pulled high.
#define L298N3_IN1 2   // OK, but ensure it doesn't interfere with boot if pulled low.
#define L298N3_IN2 4   // Safe to use

#define L298N4_ENA 16
#define L298N4_IN1 17
#define L298N4_IN2 21

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
#define DT 0.01  // Sample time, you might need to adjust this - will be linked later to odometry calculation timer

// communication configuration
#define SIZE_OF_RX_DATA 64
#define SIZE_OF_TX_DATA 64

#endif // CONFIGURATION_H
