#include "kalman_yaw.h"

KalmanYaw::KalmanYaw()
{
    // default conservative noise
    setNoise(1e-5f, 1e-6f, 1e-3f);
    init(0.f, 0.f);
}

void KalmanYaw::setNoise(float Qpsi, float Qb, float Rz)
{
    Qpsi_ = Qpsi;
    Qb_ = Qb;
    Rz_ = Rz;
}

void KalmanYaw::init(float psi0, float b0)
{
    x_[0] = psi0;
    x_[1] = b0;
    // initial covariance (uncertainty)
    P_[0][0] = 1e-2f;
    P_[0][1] = 0.f;
    P_[1][0] = 0.f;
    P_[1][1] = 1e-3f;
}

void KalmanYaw::step(float u_gyro, float z_enc, float dt)
{
    // ----------------- Predict -----------------
    // x_k+ = f(x,u) = [ psi + (u - b)*dt,  b ]
    float psi_pred = x_[0] + (u_gyro - x_[1]) * dt;
    float b_pred = x_[1];

    // Jacobian F = df/dx:
    // dpsi/db = -dt
    float F00 = 1.f, F01 = -dt;
    float F10 = 0.f, F11 = 1.f;

    // Process noise GQG' -> we'll add diag(Qpsi_, Qb_) scaled by 1 here
    float P00 = P_[0][0], P01 = P_[0][1];
    float P10 = P_[1][0], P11 = P_[1][1];

    // P = F P F^T + Q
    float PF00 = F00 * P00 + F01 * P10;
    float PF01 = F00 * P01 + F01 * P11;
    float PF10 = F10 * P00 + F11 * P10;
    float PF11 = F10 * P01 + F11 * P11;

    float Pp00 = PF00 * F00 + PF01 * F01 + Qpsi_;
    float Pp01 = PF00 * F10 + PF01 * F11 + 0.f;
    float Pp10 = PF10 * F00 + PF11 * F01 + 0.f;
    float Pp11 = PF10 * F10 + PF11 * F11 + Qb_;

    // Commit predict
    x_[0] = psi_pred;
    x_[1] = b_pred;
    P_[0][0] = Pp00;
    P_[0][1] = Pp01;
    P_[1][0] = Pp10;
    P_[1][1] = Pp11;

    // ----------------- Update (z = psi + v) -----------------
    // H = [1 0], innovation y = z - Hx
    float y = z_enc - x_[0];
    // unwrap innovation to avoid 2π jumps if z_enc is wrapped differently
    // (ถ้า z_enc เป็น unwrapped จริง ๆ บรรทัดนี้จะเกือบไม่ทำอะไร)
    if (y > M_PI)
        y -= 2.f * M_PI;
    if (y < -M_PI)
        y += 2.f * M_PI;

    float S = P_[0][0] + Rz_;
    float K0 = P_[0][0] / S; // Kalman gain for psi
    float K1 = P_[1][0] / S; // Kalman gain for b

    // x = x + K y
    x_[0] += K0 * y;
    x_[1] += K1 * y;

    // P = (I - K H) P
    // With H=[1 0], (I-KH) = [[1-K0, -K0*0],[ -K1, 1-0 ]]
    float P00_new = (1.f - K0) * P_[0][0];
    float P01_new = (1.f - K0) * P_[0][1];
    float P10_new = P_[1][0] - K1 * P_[0][0];
    float P11_new = P_[1][1] - K1 * P_[0][1];

    P_[0][0] = P00_new;
    P_[0][1] = P01_new;
    P_[1][0] = P10_new;
    P_[1][1] = P11_new;
}