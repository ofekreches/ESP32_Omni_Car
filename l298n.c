#include "l298n.h"
#include <Arduino.h>

void initL298N(L298N *driver, int enA, int in1, int in2) {
    driver->enA = enA;
    driver->in1 = in1;
    driver->in2 = in2;

    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}

void setL298NForward(L298N *driver, int speed) {
    speed = constrain(speed, 0, 255);
    analogWrite(driver->enA, speed);
    digitalWrite(driver->in1, HIGH);
    digitalWrite(driver->in2, LOW);
}

void setL298NBackward(L298N *driver, int speed) {
    speed = constrain(speed, 0, 255);
    analogWrite(driver->enA, speed);
    digitalWrite(driver->in1, LOW);
    digitalWrite(driver->in2, HIGH);
}

void stopL298NMotors(L298N *driver) {
    analogWrite(driver->enA, 0);
    digitalWrite(driver->in1, HIGH);
    digitalWrite(driver->in2, HIGH);
}
