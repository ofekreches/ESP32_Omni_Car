# RTmcu - autonomous car low level control
**the codes on the esp32 mcu:**

**core 1 -control:**

1)computes vel pid for 4 throttle motors in omni wheels configuration


**core 0 - odometry and communication handling:**

1) receiving steering and throttle data from host and converts it to different motor commands
2) computing odometry
3) sending odometry data to host


