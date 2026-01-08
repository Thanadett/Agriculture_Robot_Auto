#pragma once
#include <Arduino.h>

/** Simple PID for rate control (rad/s).
 * - Derivative on measurement (robust to steps in setpoint)
 * - Anti-windup via clamped integrator
 * - Optional output clamp (e.g., [-1, 1] duty command)
 */
class PIDRate
{
public:
    PIDRate(float kp = 0.6f, float ki = 0.0f, float kd = 0.02f,
            float out_min = -1.0f, float out_max = 1.0f,
            float i_min = -0.2f, float i_max = 0.2f, float d_lpf_hz = 10.0f)
        : kp_(kp), ki_(ki), kd_(kd),
          out_min_(out_min), out_max_(out_max),
          i_min_(i_min), i_max_(i_max)
    {
        setDLpf(d_lpf_hz);
    }

    void setGains(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }
    void setOutputClamp(float mn, float mx)
    {
        out_min_ = mn;
        out_max_ = mx;
    }
    void setIClamp(float mn, float mx)
    {
        i_min_ = mn;
        i_max_ = mx;
    }
    void setDLpf(float hz)
    {
        // 1st-order LPF for derivative: alpha = dt/(RC+dt), RC=1/(2πfc)
        d_fc_ = hz;
    }

    float step(float set_w, float meas_w, float dt)
    {
        // ===== ADD THIS BLOCK =====
        if (fabs(set_w) < 0.001f)
        {
            i_ = 0.0f;
            d_filt_ = 0.0f;
            prev_meas_ = meas_w;
            return 0.0f;
        }
        // =========================

        // error
        float e = set_w - meas_w;

        // integral (clamped)
        i_ += e * dt * ki_;
        if (i_ > i_max_)
            i_ = i_max_;
        if (i_ < i_min_)
            i_ = i_min_;

        // derivative on measurement with LPF
        float raw_d = (meas_w - prev_meas_) / (dt > 1e-6f ? dt : 1e-6f);
        float alpha = dt / ((1.0f / (2.0f * PI * d_fc_)) + dt);
        d_filt_ += alpha * (raw_d - d_filt_);
        prev_meas_ = meas_w;

        float u = kp_ * e + i_ - kd_ * d_filt_;

        if (u > out_max_)
            u = out_max_;
        if (u < out_min_)
            u = out_min_;

        return u;
    }

    void reset()
    {
        i_ = 0;
        d_filt_ = 0;
        prev_meas_ = 0;
    }

private:
    float kp_, ki_, kd_;
    float out_min_, out_max_, i_min_, i_max_;
    float d_fc_ = 10.0f;
    float i_ = 0, d_filt_ = 0, prev_meas_ = 0;
};
