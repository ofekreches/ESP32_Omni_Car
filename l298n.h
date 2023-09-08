#ifndef L298N_H
#define L298N_H

typedef struct {
  int enA;
  int in1;
  int in2;
} L298N;

// Function prototypes
void initL298N(L298N *driver, int enA, int in1, int in2);
void move_forward(L298N *driver, float speed);
void move_backward(L298N *driver, float speed);
void stop(L298N *driver);

#endif // L298N_H
