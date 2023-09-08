#ifndef COMM_CONTROLLER_H
#define COMM_CONTROLLER_H

// Communication protocol definitions
#define HEADER_1 200
#define HEADER_2 200
#define TAIL 199

typedef struct {
    float acceleration_required;
    float steering_angle_required;
} ReceivedData;

typedef struct {
    float current_x_pos;
    float current_y_pos;
    float current_heading;
} SentData;

// Function prototypes
bool receiveData(ReceivedData *data);
void sendData(const SentData *data);

#endif // COMM_CONTROLLER_H
