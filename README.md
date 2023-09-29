# RTmcu - autonomous car low level control
**the codes on the esp32 mcu:**

**core 1 -control:**

1)computes vel pid for 4 throttle motors in omni wheels configuration


**core 0 - odometry and communication handling:**

1) receiving velocity twist message from serial
2) computing odometry from encoders
3) computing odometry from imu - need to add
4) kalman filter - need to add
5) computing vel pid for vehicle's 3 axis of movement. those control signal are later fed into the vel pid for each motor
6) sending odometry data to host


