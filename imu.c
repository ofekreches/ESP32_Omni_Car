#include "imu.h"

void imu_init(Imu* imu) {
    imu->bno = Adafruit_BNO055(55, 0x28, &Wire);
    imu->bno.begin();
    
    imu->odometry->position.x = 0;
    imu->odometry->position.y = 0;
    imu->odometry->position.angular = 0;
    imu->odometry->velocity.x = 0;
    imu->odometry->velocity.y = 0;
    imu->odometry->velocity.angular = 0;
    imu->odometry->time_stamp = esp_timer_get_time();
}

void imu_calculate_odometry(Imu* imu) {
    sensors_event_t linearAccelData, orientationData, angVelocityData;
    imu->bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu->bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    imu->bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

    int64_t currentTime = esp_timer_get_time();  // ESP32 high resolution timer in microseconds
    float deltaTime = (currentTime - imu->odometry->time_stamp) / 1000000000.0f;  // Convert to seconds
    imu->odometry->time_stamp = currentTime;

    // Transform acceleration based on orientation
    float cosY = cos(orientationData.orientation.y);
    float sinY = sin(orientationData.orientation.y);
    float cosX = cos(orientationData.orientation.x);
    float sinX = sin(orientationData.orientation.x);
    float cosZ = cos(orientationData.orientation.z);
    float sinZ = sin(orientationData.orientation.z);

    float transformedAccelX = linearAccelData.acceleration.x * (cosY * cosZ) + linearAccelData.acceleration.y * (cosZ * sinX * sinY - cosX * sinZ) + linearAccelData.acceleration.z * (sinX * sinZ + cosX * cosZ * sinY);
    float transformedAccelY = linearAccelData.acceleration.x * (cosY * sinZ) + linearAccelData.acceleration.y * (cosX * cosZ + sinX * sinY * sinZ) + linearAccelData.acceleration.z * (cosX * sinY * sinZ - cosZ * sinX);
    
    // Compute odometry based on transformed acceleration
    imu->odometry->velocity.x += transformedAccelX * deltaTime;
    imu->odometry->velocity.y += transformedAccelY * deltaTime;
    
    imu->odometry->position.x += imu->odometry->velocity.x * deltaTime;
    imu->odometry->position.y += imu->odometry->velocity.y * deltaTime;
    imu->odometry->position.angular = orientationData.orientation.x;  // Assuming x is the heading in Euler representation

    // Update angular velocities
    imu->odometry->angular_velocity.x = angVelocityData.gyro.x;
    imu->odometry->angular_velocity.y = angVelocityData.gyro.y;
    imu->odometry->angular_velocity.angular = angVelocityData.gyro.z;  // Assuming z-axis is the heading in gyro representation
}
