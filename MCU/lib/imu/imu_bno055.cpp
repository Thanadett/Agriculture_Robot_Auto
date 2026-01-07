#include "imu_bno055.h"

IMU_BNO055::IMU_BNO055() {}

bool IMU_BNO055::begin()
{
    if (!bno_.begin(OPERATION_MODE_NDOF))
        return false;

    delay(1000);
    bno_.setExtCrystalUse(true);

    return true;
}

void IMU_BNO055::update()
{
    // Quaternion
    imu::Quaternion q = bno_.getQuat();
    qx = q.x();
    qy = q.y();
    qz = q.z();
    qw = q.w();

    // Gyro (rad/s)
    sensors_event_t gyro;
    bno_.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
    wx = gyro.gyro.x;
    wy = gyro.gyro.y;
    wz = gyro.gyro.z;

    // Linear acceleration (gravity removed)
    sensors_event_t lin;
    bno_.getEvent(&lin, Adafruit_BNO055::VECTOR_LINEARACCEL);
    ax = lin.acceleration.x;
    ay = lin.acceleration.y;
    az = lin.acceleration.z;
}
