#pragma once
#include <Arduino.h>
#include <math.h>

/**
 * Lightweight 2-state Kalman filter for yaw + gyro bias.
 * State x = [psi, b]^T
 * Input u = gyro yaw-rate (rad/s) from IMU (world or body asที่คุณเลือก)
 * Measurement z = yaw from encoder/odometry (rad), unwrapped
 */
class KalmanYaw
{
public:
    KalmanYaw();

    // Set noise covariances
    // Qpsi ~ drift in yaw model (rad^2 per step), Qb ~ bias random walk (rad^2/s^2 * dt^2 approx)
    // Rz   ~ measurement variance of yaw from encoder (rad^2)
    void setNoise(float Qpsi, float Qb, float Rz);

    // Initialize yaw (rad) and bias (rad/s)
    void init(float psi0, float b0);

    // One step (predict + update).
    // u_gyro = yaw-rate from IMU (rad/s); z_enc = yaw from encoder (rad, UNWRAPPED); dt seconds
    void step(float u_gyro, float z_enc, float dt);

    // Accessors
    float yaw() const { return x_[0]; }  // rad, unwrapped
    float bias() const { return x_[1]; } // rad/s

private:
    // State and covariance
    float x_[2];    // [psi, b]
    float P_[2][2]; // covariance

    // Noise
    float Qpsi_, Qb_, Rz_;

    // Utility: wrap to [-pi, pi]
    static inline float wrapPi(float a)
    {
        while (a > M_PI)
            a -= 2.f * M_PI;
        while (a < -M_PI)
            a += 2.f * M_PI;
        return a;
    }
};