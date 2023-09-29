# RTmcu - autonomous car low level control
**the codes on the esp32 mcu:**

**core 1 -control:**

1)computes vel pid for 4 throttle motors in omni wheels configuration


**core 0 - odometry and communication handling:**

1) receiving velocity twist message from serial
2) computing odometry from encoders
5) computing vel pid for vehicle's 3 axis of movement. those control signal are later fed into the vel pid for each motor
6) sending odometry data to host


**TODO:**
1) add imu
2) odometry readings from imu
3) kalman filter - imu + encoders
4) add current sensing for each motor
5) implement machine learning algorithm to predict wheel sleepage based on current readings
6) correcting encoder odometry and variance based on sleepage


# maybe this is too much for the little fellow?


