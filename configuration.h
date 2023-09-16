#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// Encoder configuration
#define ENCODER_PIN_A 19
#define ENCODER_PIN_B 18
#define ENCODER_ERROR 0.1


// Motor configuration
#define TICKS_PER_TURN 660
#define WHEEL_DIAMETER 67.0


// L298N configuration
#define L298N_ENA 14
#define L298N_IN1 27
#define L298N_IN2 13


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


//vehicle dimentions
#define VEHICLE_WIDTH 0.2 // meter
#define VEHICLE_LENGTH 0.3 //meter


//timer configurations
#define DT 0.01  // Sample time, you might need to adjust this - will be linked later to odometry calculation timer

#endif // CONFIGURATION_H
