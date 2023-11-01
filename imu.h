#ifndef IMU_H
#define IMU_H

#include "vehicle.h"
#include "configuration.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <esp_timer.h>

typedef struct {
    float x;
    float y;
    float angular;  //heading in position
} vector_3;


typedef struct {    
    vector_3 position;
    vector_3 velocity;
    Variance odometry_variance;
    int64_t time_stamp;
} Odometry;

typedef struct {
   Odometry* odometry;
   Adafruit_BNO055 bno;
} Imu;

void imu_init(Imu imu);
void imu_calculate_odometry(Imu imu);

#endif
