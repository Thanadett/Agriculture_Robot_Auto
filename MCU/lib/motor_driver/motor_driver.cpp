#include "motor_driver.h"
#include <math.h>

#ifndef MAX_YAW_RATE_FOR_FULL
#define MAX_YAW_RATE_FOR_FULL 0.63f
#endif

// ========================= State (definitions) =========================
String rx_line;
bool estop = false;
uint32_t last_cmd_ms = 0;

float tgt_LF = 0.f, tgt_LR = 0.f, tgt_RF = 0.f, tgt_RR = 0.f;
float out_LF = 0.f, out_LR = 0.f, out_RF = 0.f, out_RR = 0.f;

// ========================= Utils =========================
// Clamp helper
static inline float clampf(float x, float lo, float hi)
{
  return (x < lo) ? lo : (x > hi ? hi : x);
}
// Sign helper: returns -1, 0, +1
static inline int sgn(float x) { return (x > 0) - (x < 0); }

// LEDC setup for one pin/channel
static void setupPwmPin(int pin, int ch)
{
  pinMode(pin, OUTPUT);
  ledcSetup(ch, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(pin, ch);
  ledcWrite(ch, 0);
}

// Write bidirectional PWM using two LEDC channels (forward/reverse)
// EN: Ensure only one side is driven at a time
// TH: เขียน PWM แบบสองขา ขาหนึ่งเดินหน้า อีกขาถอยหลัง
static void writeSignedPWM(int ch_fwd, int ch_rev, float pwm_norm, bool invert = false)
{
  if (invert)
    pwm_norm = -pwm_norm;

  // 1) clamp ก่อน
  pwm_norm = clampf(pwm_norm, -1.0f, 1.0f);

  // 2) ★ minimum torque (BREAKAWAY TORQUE) ★
  if (fabsf(pwm_norm) > 0.01f && fabsf(pwm_norm) < 0.35f)
  {
    pwm_norm = copysignf(0.35f, pwm_norm);
  }

  // 3) คำนวณ duty
  const int duty = (int)lroundf(fabsf(pwm_norm) * PWM_MAX_DUTY);

  // 4) เขียน PWM
  if (pwm_norm >= 0.f)
  {
    ledcWrite(ch_fwd, duty);
    ledcWrite(ch_rev, 0);
  }
  else
  {
    ledcWrite(ch_fwd, 0);
    ledcWrite(ch_rev, duty);
  }
}

// ===================== Deadtime & Zero-Cross Handling ====================
// EN: To avoid a brief reverse "kick" when changing directions, we:
//     1) Snap output to zero when a sign flip is detected (and |out| > eps)
//     2) Hold both channels at 0 for a short deadtime (3–5 ms)
// TH: ป้องกันอาการ “ถอยแว้บ” โดยชนศูนย์เมื่อเปลี่ยนทิศ และแทรก deadtime สั้นๆ
static uint32_t last_flip_ms_LF = 0, last_flip_ms_LR = 0, last_flip_ms_RF = 0, last_flip_ms_RR = 0;
static int last_sign_LF = 0, last_sign_LR = 0, last_sign_RF = 0, last_sign_RR = 0;

static constexpr uint32_t DEADTIME_MS = 3;     // 3–5 ms is good for most H-bridges
static constexpr float ZERO_CROSS_EPS = 0.03f; // snap-to-zero threshold

// ========================= Core Motor Application ========================
static void applyMotors(float LF, float LR, float RF, float RR)
{
  // E-Stop forces zero
  if (estop)
  {
    LF = LR = RF = RR = 0.f;
  }

  // --- Slew-rate limit (ค่อยๆ ไต่ค่า) ---
  float dLF = clampf(LF - out_LF, -RAMP_STEP, RAMP_STEP);
  float dLR = clampf(LR - out_LR, -RAMP_STEP, RAMP_STEP);
  float dRF = clampf(RF - out_RF, -RAMP_STEP, RAMP_STEP);
  float dRR = clampf(RR - out_RR, -RAMP_STEP, RAMP_STEP);

  out_LF = clampf(out_LF + dLF, -1.f, 1.f);
  out_LR = clampf(out_LR + dLR, -1.f, 1.f);
  out_RF = clampf(out_RF + dRF, -1.f, 1.f);
  out_RR = clampf(out_RR + dRR, -1.f, 1.f);

  // --- Snap-to-zero if sign flip detected (กันดีดข้ามศูนย์) ---
  int sign_tgt_LF = sgn(LF), sign_out_LF = sgn(out_LF);
  int sign_tgt_LR = sgn(LR), sign_out_LR = sgn(out_LR);
  int sign_tgt_RF = sgn(RF), sign_out_RF = sgn(out_RF);
  int sign_tgt_RR = sgn(RR), sign_out_RR = sgn(out_RR);

  uint32_t now = millis();

  auto handle_flip = [&](float &out, int &last_sign, uint32_t &last_flip_ms, int sign_tgt, int sign_out)
  {
    if (sign_tgt != 0 && sign_tgt != sign_out && fabsf(out) > ZERO_CROSS_EPS)
    {
      // Direction change detected -> snap to zero and start deadtime window
      out = 0.f;
      last_sign = sign_tgt;
      last_flip_ms = now;
    }
  };
  handle_flip(out_LF, last_sign_LF, last_flip_ms_LF, sign_tgt_LF, sign_out_LF);
  handle_flip(out_LR, last_sign_LR, last_flip_ms_LR, sign_tgt_LR, sign_out_LR);
  handle_flip(out_RF, last_sign_RF, last_flip_ms_RF, sign_tgt_RF, sign_out_RF);
  handle_flip(out_RR, last_sign_RR, last_flip_ms_RR, sign_tgt_RR, sign_out_RR);

  // --- Deadtime: drive both channels 0 for a short window after flip ---
  auto in_deadtime = [&](uint32_t t0)
  { return (now - t0) < DEADTIME_MS; };

  bool dtLF = in_deadtime(last_flip_ms_LF);
  bool dtLR = in_deadtime(last_flip_ms_LR);
  bool dtRF = in_deadtime(last_flip_ms_RF);
  bool dtRR = in_deadtime(last_flip_ms_RR);

  // Small numeric clean-up to avoid tiny residuals that could flip sign logic
  if (fabsf(out_LF) < 0.01f)
    out_LF = 0.f;
  if (fabsf(out_LR) < 0.01f)
    out_LR = 0.f;
  if (fabsf(out_RF) < 0.01f)
    out_RF = 0.f;
  if (fabsf(out_RR) < 0.01f)
    out_RR = 0.f;

  // --- Final write to hardware ---
  if (dtLF)
  {
    ledcWrite(CH_LF_IN1, 0);
    ledcWrite(CH_LF_IN2, 0);
  }
  else
  {
    writeSignedPWM(CH_LF_IN1, CH_LF_IN2, out_LF, INVERT_LF);
  }

  if (dtLR)
  {
    ledcWrite(CH_LR_IN1, 0);
    ledcWrite(CH_LR_IN2, 0);
  }
  else
  {
    writeSignedPWM(CH_LR_IN1, CH_LR_IN2, out_LR, INVERT_LR);
  }

  if (dtRF)
  {
    ledcWrite(CH_RF_IN1, 0);
    ledcWrite(CH_RF_IN2, 0);
  }
  else
  {
    writeSignedPWM(CH_RF_IN1, CH_RF_IN2, out_RF, INVERT_RF);
  }

  if (dtRR)
  {
    ledcWrite(CH_RR_IN1, 0);
    ledcWrite(CH_RR_IN2, 0);
  }
  else
  {
    writeSignedPWM(CH_RR_IN1, CH_RR_IN2, out_RR, INVERT_RR);
  }
}

// =============== Command Parsing (Serial) =================
static bool startsWith(const String &s, const char *p)
{
  size_t n = strlen(p);
  return s.length() >= (int)n && s.substring(0, n).equalsIgnoreCase(p);
}

static bool parseFloatAfterEquals(const String &s, const char *key, float &out)
{
  int idx = s.indexOf(key);
  if (idx < 0)
    return false;
  idx += strlen(key);
  int end = idx;
  while (end < (int)s.length() && !isWhitespace(s[end]))
    end++;
  String num = s.substring(idx, end);
  out = num.toFloat();
  return true;
}

// แปลง V,W เป็นความเร็วเชิงเส้นล้อซ้าย/ขวา แล้วแมปเป็น PWM [-1..1]
void cmdVW_to_targets(float V_mps, float W_radps)
{
  // ----- linear part (เหมือนเดิม) -----
  float vL = V_mps;
  float vR = V_mps;

  float omegaL_v = vL / WHEEL_RADIUS;
  float omegaR_v = vR / WHEEL_RADIUS;

  float pwm_v_L = omegaL_v / MAX_OMEGA_FOR_FULL;
  float pwm_v_R = omegaR_v / MAX_OMEGA_FOR_FULL;

  // ----- yaw part (แก้ตรงนี้) -----
  float pwm_yaw = W_radps / MAX_YAW_RATE_FOR_FULL;
  pwm_yaw = clampf(pwm_yaw, -1.0f, 1.0f);

  // combine
  float pwmL = pwm_v_L - pwm_yaw;
  float pwmR = pwm_v_R + pwm_yaw;

  // clamp final
  pwmL = clampf(pwmL, -1.0f, 1.0f);
  pwmR = clampf(pwmR, -1.0f, 1.0f);

  tgt_LF = pwmL;
  tgt_LR = pwmL;
  tgt_RF = pwmR;
  tgt_RR = pwmR;
}

static void handleLine(String line)
{
  line.trim();
  if (line.isEmpty())
    return;

  // ---------------- ESTOP ----------------
  if (startsWith(line, "ESTOP"))
  {
    int sp = line.indexOf(' ');
    int val = 1;
    if (sp >= 0)
    {
      String tail = line.substring(sp + 1);
      tail.trim();
      val = tail.toInt();
    }
    estop = (val != 0);
    if (estop)
    {
      // Freeze all targets and outputs to zero (safety)
      tgt_LF = tgt_LR = tgt_RF = tgt_RR = 0.f;
      out_LF = out_LR = out_RF = out_RR = 0.f; // important: clear outputs too
    }
    return;
  }

  // ---------------- VW ----------------
  if (startsWith(line, "VW"))
  {
    float V = 0.f, W = 0.f;
    bool okV = parseFloatAfterEquals(line, "V=", V);
    bool okW = parseFloatAfterEquals(line, "W=", W);
    if (okV && okW)
    {
      last_cmd_ms = millis();
      cmdVW_to_targets(V, W);
    }
    return;
  }

  // ---------------- P / PW4 ----------------
  if (startsWith(line, "P"))
  {
    // Check "PW4" first to avoid collision with "P"
    if (startsWith(line, "PW4"))
    {
      float lf = 0.f, lr = 0.f, rf = 0.f, rr = 0.f;
      bool ok1 = parseFloatAfterEquals(line, "LF=", lf);
      bool ok2 = parseFloatAfterEquals(line, "LR=", lr);
      bool ok3 = parseFloatAfterEquals(line, "RF=", rf);
      bool ok4 = parseFloatAfterEquals(line, "RR=", rr);
      if (ok1 && ok2 && ok3 && ok4)
      {
        last_cmd_ms = millis();
        tgt_LF = clampf(lf, -1.f, 1.f);
        tgt_LR = clampf(lr, -1.f, 1.f);
        tgt_RF = clampf(rf, -1.f, 1.f);
        tgt_RR = clampf(rr, -1.f, 1.f);
      }
    }
    else
    {
      float L = 0.f, R = 0.f;
      bool okL = parseFloatAfterEquals(line, "L=", L);
      bool okR = parseFloatAfterEquals(line, "R=", R);
      if (okL && okR)
      {
        last_cmd_ms = millis();
        L = clampf(L, -1.f, 1.f);
        R = clampf(R, -1.f, 1.f);
        tgt_LF = tgt_LR = L;
        tgt_RF = tgt_RR = R;
      }
    }
    return;
  }

  // Unknown command: ignore silently (can enable debug prints if needed)
}

// ========================= Public API =========================
void motorDrive_begin()
{
  // Setup PWM pins/channels
  setupPwmPin(LF_IN1, CH_LF_IN1);
  setupPwmPin(LF_IN2, CH_LF_IN2);
  setupPwmPin(LR_IN1, CH_LR_IN1);
  setupPwmPin(LR_IN2, CH_LR_IN2);
  setupPwmPin(RF_IN1, CH_RF_IN1);
  setupPwmPin(RF_IN2, CH_RF_IN2);
  setupPwmPin(RR_IN1, CH_RR_IN1);
  setupPwmPin(RR_IN2, CH_RR_IN2);

  // Ensure outputs are zero at boot
  applyMotors(0, 0, 0, 0);
  last_cmd_ms = millis();
}

void motorDrive_handleSerialOnce()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n')
    {
      if (rx_line.length() > 0)
      {
        handleLine(rx_line);
        rx_line = "";
      }
    }
    else
    {
      if (rx_line.length() < 240)
        rx_line += c;
    }
  }
}

void motorDrive_update()
{
  uint32_t now = millis();

  // Watchdog & idle decay (ปล่อยช้าลงถ้าไม่มีคำสั่ง)
  if (!estop && (now - last_cmd_ms > CMD_TIMEOUT_MS))
  {
    tgt_LF *= IDLE_DECAY;
    tgt_LR *= IDLE_DECAY;
    tgt_RF *= IDLE_DECAY;
    tgt_RR *= IDLE_DECAY;

    if (fabsf(tgt_LF) < 0.02f)
      tgt_LF = 0.f;
    if (fabsf(tgt_LR) < 0.02f)
      tgt_LR = 0.f;
    if (fabsf(tgt_RF) < 0.02f)
      tgt_RF = 0.f;
    if (fabsf(tgt_RR) < 0.02f)
      tgt_RR = 0.f;
  }

  // Apply with slew + snap-to-zero + deadtime
  applyMotors(tgt_LF, tgt_LR, tgt_RF, tgt_RR);
}
