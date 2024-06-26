#ifndef COMM_CONTROLLER_H
#define COMM_CONTROLLER_H

#include <stdint.h>
#include <Arduino.h>

#include "configuration.h"
#include "vehicle.h"

typedef struct {
    uint8_t RxData[SIZE_OF_RX_DATA];
    uint8_t TxData[SIZE_OF_TX_DATA];
    int comm_baud_rate;
} CommController;

typedef enum {
    POSITION_MODE,
    VELOCITY_MODE,
    ODOMETRY_MODE
} DesiredControl;

#ifdef __cplusplus
extern "C" {
#endif

// Function prototypes
void comm_controller_init(CommController *comm);
int receiveData(CommController *comm, Vehicle *vehicle);
void ProcessDataToSend(CommController *comm, const Vehicle *vehicle);

#ifdef __cplusplus
}
#endif

#endif // COMM_CONTROLLER_H
