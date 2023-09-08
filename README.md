# RTmcu - autonomous car low level control
**the codes on the esp32 mcu:**

core 0:

computes vel pid for 2 throttle motors - located on the rear axle of the vehicle

compute pos pid for 1 steering motor

compute odometry based on the bicycle model, refrence point is the middle of the vehicle

core 1:

communication handling

receiving steering and throttle data from host and converts it to different motor commands
sending odometry data to host

