#include "comm_controller.h"
#include <Arduino.h>

bool receiveData(CommController *comm) {
    if (Serial.available() >= (2 + 2 * sizeof(float) + 1)) { // 2 bytes for header, 2x4 bytes for payload, 1 byte for tail
        if (Serial.read() == HEADER_1 && Serial.read() == HEADER_2) {
            Serial.readBytes(comm->RxData, 2 * sizeof(float)); // Assuming RxData is a byte array
            if (Serial.read() == TAIL) {
                return true; // Data received successfully
            }
        }
    }
    return false; // Data not received or incorrect format
}

void sendData(CommController *comm) {
    Serial.write(HEADER_1);
    Serial.write(HEADER_2);
    Serial.write(comm->TxData, 3 * sizeof(float)); // Assuming TxData is a byte array and you're sending 3 floats
    Serial.write(TAIL);
}

void comm_controller_init(CommController *comm) {
    Serial.begin(comm->comm_buad_rate);
    Serial.setTimeout(1); ///TODO maybe need to alter
    // Any other initialization steps can be added here
}

