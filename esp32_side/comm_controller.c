#include <stdint.h>       // Standard library headers first
#include <Arduino.h>

#include "configuration.h"
#include "vehicle.h"
#include "comm_controller.h"



void comm_controller_init(CommController *comm) {
    comm->comm_baud_rate = SERIAL_BAUDRATE;
    memset(comm->RxData, 0, SIZE_OF_RX_DATA);
    memset(comm->TxData, 0, SIZE_OF_TX_DATA);
}


int receiveData(CommController *comm, Vehicle *vehicle) {
  int valid_data = 0;
  if (comm->RxData[0] == HEADER && comm->RxData[1] == HEADER) {
    valid_data = 1;
    uint8_t checksum = 0;
    for (int i = 3; i < SIZE_OF_RX_DATA - 2 ; i++) {
        checksum += comm->RxData[i];
    }
    if (comm->RxData[SIZE_OF_RX_DATA - 2] == checksum) {  //passed all integrity checks
      valid_data = 2;
      if (comm->RxData[2] == POSITION_MODE) {
          memcpy(&vehicle->desired_state.position.x, &comm->RxData[3], 4);
          memcpy(&vehicle->desired_state.position.y, &comm->RxData[7], 4);
          memcpy(&vehicle->desired_state.position.angular, &comm->RxData[11], 4);
      }
      else if (comm->RxData[2] == VELOCITY_MODE) {
          memcpy(&vehicle->desired_state.velocity.x, &comm->RxData[3], 4);
          memcpy(&vehicle->desired_state.velocity.y, &comm->RxData[7], 4);
          memcpy(&vehicle->desired_state.velocity.angular, &comm->RxData[11], 4);
      }
      else if (comm->RxData[2] == ODOMETRY_MODE) {
        
          memcpy(&vehicle->current_state.position.x, &comm->RxData[3], 4);
          memcpy(&vehicle->current_state.position.y, &comm->RxData[7], 4);
          memcpy(&vehicle->current_state.position.angular, &comm->RxData[11], 4);
          memcpy(&vehicle->current_state.odometry_variance.position_error.x, &comm->RxData[15], 4);
          memcpy(&vehicle->current_state.odometry_variance.position_error.y, &comm->RxData[19], 4);
          memcpy(&vehicle->current_state.odometry_variance.position_error.angular, &comm->RxData[23], 4);
          memcpy(&vehicle->current_state.velocity.x, &comm->RxData[27], 4);
          memcpy(&vehicle->current_state.velocity.y, &comm->RxData[31], 4);
          memcpy(&vehicle->current_state.velocity.angular, &comm->RxData[35], 4);
          memcpy(&vehicle->current_state.odometry_variance.velocity_error.x, &comm->RxData[39], 4);
          memcpy(&vehicle->current_state.odometry_variance.velocity_error.y, &comm->RxData[43], 4);
          memcpy(&vehicle->current_state.odometry_variance.velocity_error.angular, &comm->RxData[47], 4);
      }
    }
    else {
        memset(comm->RxData, 0, SIZE_OF_RX_DATA);  // Clear the buffer
    }
  }
  else {
      memset(comm->RxData, 0, SIZE_OF_RX_DATA);  // Clear the buffer
  }
  return valid_data;
}




void ProcessDataToSend(CommController *comm, const Vehicle *vehicle) {
    comm->TxData[0] = HEADER;
    comm->TxData[1] = HEADER;

    memcpy(&comm->TxData[2], &vehicle->current_state.position.x, 4);
    memcpy(&comm->TxData[6], &vehicle->current_state.position.y, 4);
    memcpy(&comm->TxData[10], &vehicle->current_state.position.angular, 4);

    memcpy(&comm->TxData[14], &vehicle->current_state.velocity.x, 4);
    memcpy(&comm->TxData[18], &vehicle->current_state.velocity.y, 4);
    memcpy(&comm->TxData[22], &vehicle->current_state.velocity.angular, 4);

    memcpy(&comm->TxData[26], &vehicle->current_state.odometry_variance.position_error.x, 4);
    memcpy(&comm->TxData[30], &vehicle->current_state.odometry_variance.position_error.y, 4);
    memcpy(&comm->TxData[34], &vehicle->current_state.odometry_variance.position_error.angular, 4);

    memcpy(&comm->TxData[38], &vehicle->current_state.odometry_variance.velocity_error.x, 4);
    memcpy(&comm->TxData[42], &vehicle->current_state.odometry_variance.velocity_error.y, 4);
    memcpy(&comm->TxData[46], &vehicle->current_state.odometry_variance.velocity_error.angular, 4);

    // Compute checksum
    uint8_t checksum = 0;
    for (int i = 2; i < 50; i++) {  
        checksum += comm->TxData[i];
    }
    comm->TxData[50] = checksum;
    comm->TxData[51] = TAIL;

    // Send data
   
}
