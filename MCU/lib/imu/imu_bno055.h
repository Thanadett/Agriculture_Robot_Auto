#pragma once
#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

class IMU_BNO055
{
public:
    IMU_BNO055();

    bool begin();
    void update();

    // Quaternion
    float qx = 0, qy = 0, qz = 0, qw = 1;

    // Gyro (rad/s)
    float wx = 0, wy = 0, wz = 0;

    // Linear accel (m/s^2)
    float ax = 0, ay = 0, az = 0;

private:
    Adafruit_BNO055 bno_{55};
};
