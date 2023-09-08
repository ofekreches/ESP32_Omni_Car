# RTmcu - autonomous car low level control
**the codes on the esp32 mcu:**

**core 0- odometry and control:**

1)computes vel pid for 2 throttle motors - located on the rear axle of the vehicle
2) compute pos pid for 1 steering motor
3) compute odometry based on the bicycle model, refrence point is the middle of the vehicle

**core 1 - communication handling:**

1) receiving steering and throttle data from host and converts it to different motor commands
2) sending odometry data to host


