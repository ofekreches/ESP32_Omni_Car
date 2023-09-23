#include "comm_controller.h"
#include <Arduino.h>

#include "vehicle.h"



void comm_controller_init(CommController *comm) {
    Serial.begin(comm->comm_buad_rate);
    Serial.setTimeout(1); ///TODO maybe need to alter
}


#define HEADER_1 0xAA  // Assuming the values for your header and tail
#define HEADER_2 0x55
#define TAIL 0x5A



bool receiveData(CommController *comm, Vehicle *vehicle) {
    
    if (Serial.available() >= SIZE_OF_RX_DATA) {
        Serial.readBytes(comm->RxData, SIZE_OF_RX_DATA);

        if (comm->RxData[0] == HEADER_1 && comm->RxData[1] == HEADER_2) {
            byte payloadSize = comm->RxData[2];
            

            float desiredX, desiredY, desiredAngular;
            memcpy(&desiredX, &comm->RxData[4], 4);
            memcpy(&desiredY, &comm->RxData[8], 4);
            memcpy(&desiredAngular, &comm->RxData[12], 4);

            // Checksum verification
            byte checksum = 0;
            for (int i = 4; i < 12; i++) {
                checksum += comm->RxData[i];
            }

            if (comm->RxData[SIZE_OF_RX_DATA - 2] == checksum && comm->RxData[SIZE_OF_RX_DATA - 1] == TAIL) {
                if (comm->RxData[3] == POSITION) {
                    vehicle->desired_state.position.x = desiredX;
                    vehicle->desired_state.position.y = desiredY;
                    vehicle->desired_state.position.angular = desiredAngular;
                } else if (comm->RxData[3] == VELOCITY) {
                    vehicle->desired_state.velocity.x = desiredX;
                    vehicle->desired_state.velocity.y = desiredY;
                    vehicle->desired_state.velocity.angular = desiredAngular;
                }
                return true; // Data received and parsed successfully
            }
        }
    }
    return false; // Data not received or incorrect format
}



void sendData(CommController *comm, Vehicle *Vehicle) {
    Serial.write(HEADER_1);
    Serial.write(HEADER_2);
    Serial.write(comm->TxData, 3 * sizeof(float)); // Assuming TxData is a byte array and you're sending 3 floats
    Serial.write(TAIL);
}



