#include "comm_controller.h"
#include <Arduino.h>

#include "vehicle.h"
#include "configuration.h"


void comm_controller_init(CommController *comm) {
    Serial.begin(comm->comm_buad_rate);
    memset(comm->RxData, 0, SIZE_OF_RX_DATA);
    memset(comm->TxData, 0, SIZE_OF_TX_DATA);
    Serial.setTimeout(1); ///time in mili-seconds
}



#define HEADER 0xAA  // Example definition, adjust as needed
#define TAIL 0x5A
#define POSITION 0x00
#define VELOCITY 0x01

void receiveData(CommController *comm, Vehicle *vehicle) {
    if (Serial.available() >= SIZE_OF_RX_DATA) {
        Serial.readBytes(comm->RxData, SIZE_OF_RX_DATA);

        if (comm->RxData[0] == HEADER && comm->RxData[1] == HEADER && comm->RxData[SIZE_OF_RX_DATA - 1] == TAIL) {
            byte checksum = 0;
            for (int i = 4; i < 12; i++) {
                checksum += comm->RxData[i];
            }
            if (comm->RxData[SIZE_OF_RX_DATA - 2] == checksum) {  //passed all integrity checks
                if (comm->RxData[3] == POSITION) {
                    memcpy(&vehicle->desired_state.position.x, &comm->RxData[4], 4);
                    memcpy(&vehicle->desired_state.position.y, &comm->RxData[8], 4);
                    memcpy(&vehicle->desired_state.position.angular, &comm->RxData[12], 4);
                }
                else if (comm->RxData[3] == VELOCITY) {
                    memcpy(&vehicle->desired_state.velocity.x, &comm->RxData[4], 4);
                    memcpy(&vehicle->desired_state.velocity.y, &comm->RxData[8], 4);
                    memcpy(&vehicle->desired_state.velocity.angular, &comm->RxData[12], 4);
                }
                sendData(comm ,vehicle);
            }
            else {
                memset(comm->RxData, 0, SIZE_OF_RX_DATA);  // Clear the buffer
            }
        }
        else {
            memset(comm->RxData, 0, SIZE_OF_RX_DATA);  // Clear the buffer
        }
    }
}



void sendData(CommController *comm, const Vehicle *vehicle) {
    comm->TxData[0] = HEADER;
    comm->TxData[1] = HEADER;

    memcpy(&comm->TxData[2], &vehicle->current_state.position.x, 4);
    memcpy(&comm->TxData[6], &vehicle->current_state.position.y, 4);
    memcpy(&comm->TxData[10], &vehicle->current_state.position.angular, 4);

    memcpy(&comm->TxData[14], &vehicle->current_state.velocity.x, 4);
    memcpy(&comm->TxData[18], &vehicle->current_state.velocity.y, 4);
    memcpy(&comm->TxData[22], &vehicle->current_state.velocity.angular, 4);

    memcpy(&comm->TxData[26], &vehicle->odometry_variance.position_error.x, 4);
    memcpy(&comm->TxData[30], &vehicle->odometry_variance.position_error.y, 4);
    memcpy(&comm->TxData[34], &vehicle->odometry_variance.position_error.angular, 4);

    memcpy(&comm->TxData[38], &vehicle->odometry_variance.velocity_error.x, 4);
    memcpy(&comm->TxData[42], &vehicle->odometry_variance.velocity_error.y, 4);
    memcpy(&comm->TxData[46], &vehicle->odometry_variance.velocity_error.angular, 4);

    // Compute checksum
    byte checksum = 0;
    for (int i = 2; i < 50; i++) {  
        checksum += comm->TxData[i];
    }
    comm->TxData[50] = checksum;
    comm->TxData[51] = TAIL;

    // Send data
    Serial.write(comm->TxData, SIZE_OF_TX_DATA);
}




