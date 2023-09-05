#include <ESP32Encoder.h>

#define TICKS_PER_TURN 660
#define WHEEL_DIAMETER 67 // mm
#define DISTANCE_PER_TICK WHEEL_DIAMETER*PI/TICKS_PER_TURN

int enA = 14;
int in1 = 27;
int in2 = 13;

float desired_position = 0; // Desired position in mm
float current_position = 0; // Current position in mm
float last_error = 0;
float integral = 0;

typedef struct {
  float kp;
  float ki;
  float kd;
  float i_windup; // Maximum allowed integral value
  float control_signal;
} POS_PID;

POS_PID pos_pid;

ESP32Encoder encoder;

void setup() {
  initPID();
  initEncoder();
  
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  
  Serial.begin(115200);
}

void loop() {
  current_position = (float)encoder.getCount() * DISTANCE_PER_TICK;
  Serial.println("Current position in mm : " + String(current_position));

  if (Serial.available() > 0) {
    desired_position = Serial.readString().toFloat();
    Serial.println("Desired position: " + String(desired_position));
  }

  control_motor_with_pid();
}

void initPID() {
  pos_pid.kp = 1;
  pos_pid.ki = 0.01;
  pos_pid.kd = 0.1;
  pos_pid.i_windup = 1000;
  pos_pid.control_signal = 0;
}

void initEncoder() {
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachHalfQuad(19, 18); //  Oori!  you can also use full QUAD , increases encoder resolution - read about it
  encoder.clearCount();
}

void control_motor_with_pid() {
  float error = desired_position - current_position;

  integral += error;
  // Anti-windup
  if (integral > pos_pid.i_windup) integral = pos_pid.i_windup;
  if (integral < -pos_pid.i_windup) integral = -pos_pid.i_windup;

  float derivative = error - last_error;
  
  pos_pid.control_signal = pos_pid.kp * error + pos_pid.ki * integral + pos_pid.kd * derivative;

  last_error = error;

  if (pos_pid.control_signal > 0) {
    forward(abs(pos_pid.control_signal));
  } else {
    backward(abs(pos_pid.control_signal));
  }
}

void backward(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(enA, speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void forward(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(enA, speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void stop_motors() {
  analogWrite(enA, 0);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
}
