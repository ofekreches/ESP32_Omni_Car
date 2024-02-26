#include "ESP32Encoder.h"
#include "ESP32EncoderWrapper.h" // Include the header file for the wrapper

extern "C" {

ESP32Encoder* createEncoder() {
    return new ESP32Encoder();
}

void attachHalfQuad(ESP32Encoder* encoder, int aPinNumber, int bPinNumber) {
    if(encoder) {
        encoder->attachHalfQuad(aPinNumber, bPinNumber);
    }
}

int64_t getEncoderCount(ESP32Encoder* encoder) {
    if(encoder) {
        return encoder->getCount();
    }
    return 0;
}

void deleteEncoder(ESP32Encoder* encoder) {
    if(encoder) {
        delete encoder;
    }
}

int64_t clearCount(ESP32Encoder* encoder) {
    if(encoder) {
        return encoder->clearCount();
    }
    return 0;
}

}
