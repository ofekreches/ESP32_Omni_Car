#ifndef L298N_H
#define L298N_H

typedef struct {
  int enA;
  int in1;
  int in2;
} L298N;

// Function prototypes
void initL298N(L298N *driver, int enA, int in1, int in2);
void setL298NForward(L298N *driver, int speed);
void setL298NBackward(L298N *driver, int speed);
void stopL298NMotors(L298N *driver);

#endif // L298N_H
