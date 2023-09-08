#include "comm_controller.h"

bool receiveData(ReceivedData *data) {
    if (Serial.available() >= 10) { // 2 bytes for header, 2x4 bytes for payload, 1 byte for tail
        if (Serial.read() == HEADER_1 && Serial.read() == HEADER_2) {
            Serial.readBytes((byte*)&data->acceleration_required, sizeof(float));
            Serial.readBytes((byte*)&data->steering_angle_required, sizeof(float));
            if (Serial.read() == TAIL) {
                return true; // Data received successfully
            }
        }
    }
    return false; // Data not received or incorrect format
}

void sendData(const SentData *data) {
    Serial.write(HEADER_1);
    Serial.write(HEADER_2);
    Serial.write((byte*)&data->current_x_pos, sizeof(float));
    Serial.write((byte*)&data->current_y_pos, sizeof(float));
    Serial.write((byte*)&data->current_heading, sizeof(float));
    Serial.write(TAIL);
}
