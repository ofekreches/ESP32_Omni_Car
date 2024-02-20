#ifndef ENCODER_H
#define ENCODER_H

#include "ESP32EncoderWrapper.h"

typedef struct {
  int encoderPinA; // First encoder pin
  int encoderPinB; // Second encoder pin
  ESP32Encoder* instance; // Pointer to ESP32Encoder instance
} Encoder;

#ifdef __cplusplus
extern "C" {
#endif

// Function prototypes
void initEncoder(Encoder *enc, int encoderPinA, int encoderPinB);
int readEncoder(Encoder *enc);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H
