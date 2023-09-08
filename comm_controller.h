#ifndef COMM_CONTROLLER_H
#define COMM_CONTROLLER_H

// Communication protocol definitions
#define HEADER_1 200
#define HEADER_2 200
#define TAIL 199

typedef struct {
    RxData[SIZE_OF_RX_DATA];
    TxData[SIZE_OF_TX_dATA];
    int comm_buad_rate = 115200;
} CommController;



// Function prototypes
bool receiveData(CommController *comm);
void sendData(CommController *comm);
void comm_controller_init(CommController *comm);

#endif // COMM_CONTROLLER_H
