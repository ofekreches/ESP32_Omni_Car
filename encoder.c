#include "/home/ofek/code/autonomous_car_control_mcu/main/Inc/encoder.h"

void initEncoder(Encoder *enc, int encoderPinA, int encoderPinB) {
  
  enc->encoderPinA = encoderPinA;
  enc->encoderPinB = encoderPinB;
  
  // Initialize the ESP32Encoder instance
  enc->instance.attachHalfQuad(encoderPinA, encoderPinB);
  enc->instance.clearCount();
}

int readEncoder(Encoder *enc) {
  return enc->instance.getCount();
}
