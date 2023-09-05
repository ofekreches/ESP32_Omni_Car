#ifndef ENCODER_H
#define ENCODER_H

#include <ESP32Encoder.h>



typedef struct {
  int encoderPinA; // First encoder pin
  int encoderPinB; // Second encoder pin
  ESP32Encoder instance; // Instance of the ESP32Encoder
} Encoder;

// Function prototypes
void initEncoder(Encoder *enc, int encoderPinA, int encoderPinB);
int readEncoder(Encoder *enc);

#endif // ENCODER_H
