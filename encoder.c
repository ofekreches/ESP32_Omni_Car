#include "encoder.h"

void initEncoder(Encoder *enc, int encoderPinA, int encoderPinB) {  
  enc->encoderPinA = encoderPinA;
  enc->encoderPinB = encoderPinB;
  
  // Initialize the ESP32Encoder instance
  enc->instance = createEncoder();
  attachHalfQuad(enc->instance, encoderPinA, encoderPinB);
  clearCount(enc->instance);
}

int readEncoder(Encoder *enc) {
  return getEncoderCount(enc->instance);
}
