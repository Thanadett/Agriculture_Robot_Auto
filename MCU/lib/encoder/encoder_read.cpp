#include "encoder_read.h"
#include <math.h>

void QuadEncoderReader::begin(bool enable_internal_pullups)
{
  ESP32Encoder::useInternalWeakPullResistors = enable_internal_pullups ? UP : DOWN;

  enc_[W_FL].attachHalfQuad(pins_[W_FL][0], pins_[W_FL][1]);
  enc_[W_FR].attachHalfQuad(pins_[W_FR][0], pins_[W_FR][1]);
  enc_[W_RL].attachHalfQuad(pins_[W_RL][0], pins_[W_RL][1]);
  enc_[W_RR].attachHalfQuad(pins_[W_RR][0], pins_[W_RR][1]);

  for (int i = 0; i < W_COUNT; ++i)
  {
    enc_[i].clearCount();
    last_counts_[i] = enc_[i].getCount();
    pos_rad_[i] = 0.0f;
    vel_rad_s_[i] = 0.0f;
    total_dist_m_[i] = 0.0f;
  }
  last_ts_ms_ = millis();
}

void QuadEncoderReader::reset()
{
  for (int i = 0; i < W_COUNT; ++i)
  {
    enc_[i].clearCount();
    last_counts_[i] = 0;
    pos_rad_[i] = 0.0f;
    vel_rad_s_[i] = 0.0f;
    total_dist_m_[i] = 0.0f;
  }
  last_ts_ms_ = millis();
}

void QuadEncoderReader::update()
{
  uint32_t now = millis();
  float dt = (now - last_ts_ms_) / 1000.0f;
  if (dt <= 0.0f)
    dt = 1e-3f;
  last_ts_ms_ = now;

  const float ppr = (ppr_out_ <= 0.0f ? 1.0f : ppr_out_);
  const float two_pi = 2.0f * (float)M_PI;
  const float C = circumferenceM();

  for (int i = 0; i < W_COUNT; ++i)
  {
    long cur = enc_[i].getCount() * inv_[i];
    long d = cur - last_counts_[i];
    last_counts_[i] = cur;

    total_dist_m_[i] += ((float)d / ppr) * C;
    pos_rad_[i] = ((float)cur / ppr) * two_pi; // signed angle
    float rev_dt = ((float)d / ppr) / dt;
    vel_rad_s_[i] = rev_dt * two_pi; // signed angular velocity
  }
}
