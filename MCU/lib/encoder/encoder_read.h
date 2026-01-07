#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>
#include "config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- Wheel index order: FL, FR, RL, RR ---
enum WheelIndex : uint8_t
{
  W_FL = 0,
  W_FR = 1,
  W_RL = 2,
  W_RR = 3,
  W_COUNT = 4
};

static constexpr float ENCODER_PPR_OUTPUT_DEFAULT =
    ENCODER_PPR_MOTOR * REDUCTION_RATIO * QUAD_MULTIPLIER;

// ===== 4-wheel quadrature encoder reader =====
class QuadEncoderReader
{
public:
  QuadEncoderReader(
      int flA = ENC_FL_A, int flB = ENC_FL_B,
      int frA = ENC_FR_A, int frB = ENC_FR_B,
      int rlA = ENC_RL_A, int rlB = ENC_RL_B,
      int rrA = ENC_RR_A, int rrB = ENC_RR_B,
      float pulses_per_rev_output = ENCODER_PPR_OUTPUT_DEFAULT)
      : ppr_out_(pulses_per_rev_output)
  {
    pins_[W_FL][0] = flA;
    pins_[W_FL][1] = flB;
    pins_[W_FR][0] = frA;
    pins_[W_FR][1] = frB;
    pins_[W_RL][0] = rlA;
    pins_[W_RL][1] = rlB;
    pins_[W_RR][0] = rrA;
    pins_[W_RR][1] = rrB;
  }

  void begin(bool enable_internal_pullups = true);
  void update();
  void reset();

  // --- getters ---
  float positionRad(WheelIndex w) const { return pos_rad_[w]; }         // [rad] signed
  float velocityRadPerSec(WheelIndex w) const { return vel_rad_s_[w]; } // [rad/s]
  long counts(WheelIndex w) { return enc_[w].getCount(); }
  float totalDistanceM(WheelIndex w) const { return total_dist_m_[w]; } // accumulated meters (abs)

  // wheel geometry
  void setWheelRadius(float r_m) { wheel_radius_m_ = r_m; }
  float wheelRadius() const { return wheel_radius_m_; }
  float circumferenceM() const { return 2.0f * (float)M_PI * wheel_radius_m_; }

  // PPR
  void setPPR(float pulses_per_rev_output) { ppr_out_ = pulses_per_rev_output; }
  float getPPR() const { return ppr_out_; }

  // invert direction per wheel
  void setInvert(bool invFL, bool invFR, bool invRL, bool invRR)
  {
    inv_[W_FL] = invFL ? -1 : 1;
    inv_[W_FR] = invFR ? -1 : 1;
    inv_[W_RL] = invRL ? -1 : 1;
    inv_[W_RR] = invRR ? -1 : 1;
  }

private:
  int pins_[W_COUNT][2];
  ESP32Encoder enc_[W_COUNT];

  float ppr_out_ = ENCODER_PPR_OUTPUT_DEFAULT;
  int inv_[W_COUNT] = {1, 1, 1, 1};
  float wheel_radius_m_ = ENC_WHEEL_RADIUS;

  long last_counts_[W_COUNT] = {0, 0, 0, 0};
  uint32_t last_ts_ms_ = 0;

  float pos_rad_[W_COUNT] = {0, 0, 0, 0};
  float vel_rad_s_[W_COUNT] = {0, 0, 0, 0};
  float total_dist_m_[W_COUNT] = {0, 0, 0, 0};
};