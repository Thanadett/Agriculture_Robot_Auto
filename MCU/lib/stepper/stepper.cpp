#include <Arduino.h>
#include "stepper.h"

// ---------------- Configuration helpers ----------------
// Assume TB6600 wiring (active-low inputs):
//   PUL+ -> 3.3V, PUL- -> STEP GPIO
//   DIR+ -> 3.3V, DIR- -> DIR  GPIO
//   ENA+ -> 3.3V, ENA- -> ENA  GPIO
// => STEP signal is effectively active-low, so we invert STEP in code.
//
// สมมติการต่อแบบ active-low ตามด้านบน -> ต้อง invert STEP
#ifndef ENABLE_ACTIVE_LOW
#define ENABLE_ACTIVE_LOW 1
#endif

// ---------------- ENA control ----------------
inline void UnifiedStepper::ena_enable()
{
#if ENABLE_ACTIVE_LOW
  digitalWrite(p_.ena_Pin, LOW); // active-low -> LOW = Enable
#else
  digitalWrite(p_.ena_Pin, HIGH);
#endif
}
inline void UnifiedStepper::ena_disable()
{
#if ENABLE_ACTIVE_LOW
  digitalWrite(p_.ena_Pin, HIGH); // active-low -> HIGH = Disable
#else
  digitalWrite(p_.ena_Pin, LOW);
#endif
}

// ---------------- Constructor ----------------
// stepDelayUs: desired step period (us) used to derive a default speed
// สูตร: 1e6 us = 1 s -> sps = 1e6 / period_us
UnifiedStepper::UnifiedStepper(int stepPin, int dirPin, int enaPin, int stepDelayUs)
    : p_{stepPin, dirPin, enaPin, stepDelayUs},
      stepper_(AccelStepper::DRIVER, stepPin, dirPin)
{
  default_speed_sps_ = 1e6f / (float)stepDelayUs;  // steps per second
  default_accel_sps2_ = default_speed_sps_ * 5.0f; // เริ่มด้วย ~5x ของ speed เป็นค่าเร่งเริ่มต้น
}

// NOTE: these macros/pins are expected to be defined in your config
// หมายเหตุ: คาดว่า PIN_STEP/PIN_DIR/PIN_ENABLE/STEP_DELAY_US มาจากไฟล์ config ของคุณ
UnifiedStepper Nema17(PIN_STEP, PIN_DIR, PIN_ENABLE, STEP_DELAY_US);

// ---------------- Init ----------------
bool UnifiedStepper::begin()
{
  pinMode(p_.step_pin, OUTPUT);
  pinMode(p_.dir_pin, OUTPUT);
  pinMode(p_.ena_Pin, OUTPUT);

  // Make sure outputs are at known idle levels
  digitalWrite(p_.step_pin, LOW);
  digitalWrite(p_.dir_pin, LOW);

  // ★ Enable driver immediately (สำคัญ)
  ena_enable();

  // ★ Invert STEP for active-low wiring on TB6600
  //    STEP = true (invert), DIR = true/false (เลือกตามทิศที่ถูกต้องของคุณ)
  // ถ้าทิศทางกลับ, สลับ true <-> false ที่พารามิเตอร์ตัวที่สอง
  stepper_.setPinsInverted(
      true, // ★ invert STEP (active-low)
      true, // invert DIR (ถ้าทิศผิด ค่อยเปลี่ยนเป็น false)
      false // ENA not handled by AccelStepper; we control it ourselves
  );

  stepper_.enableOutputs();

  // Timing for TB6600: pulse width >= ~2.2us, keep margin 5us
  // เวลาชีพจรขั้นต่ำ: ตั้ง 5us ไว้ปลอดภัย
  stepper_.setMinPulseWidth(5);

  // Reasonable safe defaults for 1/4 microstep (800 steps / rev)
  // ค่าเริ่มทดลองสำหรับ 1/4 microstep
  stepper_.setMaxSpeed(default_speed_sps_); // e.g., 6000–12000 sps to start
  stepper_.setAcceleration(40000.0f);       // เริ่มที่ 8e4 sps^2 แล้วค่อยไล่เพิ่ม

  stepper_.setCurrentPosition(0);
  continuous_ = false;
  dirCW_ = true;

  return true;
}

// ---------------- Manual fixed steps (blocking with ramp) ----------------
void UnifiedStepper::stepCW(unsigned steps)
{
  ena_enable();
  delayMicroseconds(5);

  stepper_.moveTo(stepper_.currentPosition() + (long)steps);
  stepper_.runToPosition(); // uses accel profile
}

void UnifiedStepper::stepCCW(unsigned steps)
{
  ena_enable();
  delayMicroseconds(5);

  stepper_.moveTo(stepper_.currentPosition() - (long)steps);
  stepper_.runToPosition(); // uses accel profile
}

// ---------------- Continuous rotation with ramp (recommended) ----------------
// Use run() (with acceleration) instead of runSpeed() to avoid immediate stall.
// ใช้ run() + acceleration เพื่อไม่ให้สะดุดตอนออกตัว
//
// sps_target: เป้าหมายความเร็ว (steps/sec). ถ้า <= 0 จะใช้ default_speed_sps_
void UnifiedStepper::rotateContinuous(bool cw, float sps_target /* = -1.0f */)
{
  ena_enable();
  continuous_ = true;
  dirCW_ = cw;

  float v = (sps_target > 0.0f) ? sps_target : default_speed_sps_;

  // Clamp to a sane ceiling if needed (optional)
  // กำหนดเพดานความเร็วคร่าว ๆ เผื่อกันพลาด
  if (v < 200.0f)
    v = 200.0f; // low floor to ensure motion
  if (v > 12000.0f)
    v = 12000.0f; // TB6600+NEMA17 typical upper region (โหลดมีผล)

  stepper_.setMaxSpeed(v);

  // Move "far away" in the chosen direction so it keeps running under run()
  // สั่งเป้าหมายไกล ๆ เพื่อให้ run() วิ่งต่อเนื่อง
  const long FAR = LONG_MAX / 4;
  long target = stepper_.currentPosition() + (dirCW_ ? +FAR : -FAR);
  stepper_.moveTo(target);
}

void UnifiedStepper::stop()
{
  continuous_ = false;
  // Graceful stop with decel
  // ชะลอหยุดอย่างนุ่มนวล
  stepper_.stop(); // request stop (with ramp)
  // ไม่ตัด ENA ทันทีเพื่อให้จอดนิ่ม (ถ้าต้องการปลด ENA ค่อยทำหลังหยุดนิ่ง)
  // ena_disable();
}

// ---------------- Periodic tick (call as often as possible) ----------------
// Must be called very frequently (no blocking/delay in main loop).
// ต้องเรียกถี่ ๆ หลีกเลี่ยง delay/blocking ใน loop หลัก
void UnifiedStepper::tick()
{
  // Use accel profile always for stability
  stepper_.run();

  // If running continuous and we are getting too close to target,
  // extend target further to keep going (rare but safe).
  // เผื่อกรณีพิเศษ: ถ้าเข้าใกล้เป้าหมายไกลมาก ๆ ก็ยืดเป้าหมายออกไป
  if (continuous_)
  {
    const long margin = 1000L;
    long dist = stepper_.distanceToGo();
    if ((dirCW_ && dist < margin) ||
        (!dirCW_ && dist > -margin))
    {
      const long FAR = LONG_MAX / 4;
      long target = stepper_.currentPosition() + (dirCW_ ? +FAR : -FAR);
      stepper_.moveTo(target);
    }
  }
}

// ----------------- Command parser & button handlers -----------------
// (unchanged logic; only call rotateContinuous with target sps suitable for 1/4 microstep)
static STP_handlers_step g_handlers_step;
static String g_rx_line_step;

void stepper_set_handlers(const STP_handlers_step &h) { g_handlers_step = h; }

static inline bool _parseTokenAfterEquals(const String &s, const char *key, String &out)
{
  int idx = s.indexOf(key);
  if (idx < 0)
    return false;
  idx += strlen(key);
  int end = idx;
  while (end < (int)s.length() && !isWhitespace(s[end]))
    end++;
  out = s.substring(idx, end);
  out.trim();
  return out.length() > 0;
}

static inline bool _startsWith_ST(const String &s, const char *p)
{
  size_t n = strlen(p);
  return s.length() >= (int)n && s.substring(0, n).equalsIgnoreCase(p);
}

static bool prevUp = false, prevDn = false;

bool stepper_handle_line(const String &raw, UnifiedStepper &stepper)
{
  String line = raw;
  line.trim();
  if (line.isEmpty())
    return false;
  if (!_startsWith_ST(line, "STP"))
    return false;

  Serial.printf("[ESP32] RX: %s\n", line.c_str());
  String tokUp, tokDown;
  bool hasUp = _parseTokenAfterEquals(line, "C_Up=", tokUp);
  bool hasDown = _parseTokenAfterEquals(line, "C_Dn=", tokDown);

  auto toDown = [](const String &t) -> bool
  {
    return t.equalsIgnoreCase("DOWN") || t == "1";
  };

  if (hasUp)
  {
    bool nowUp = toDown(tokUp);
    if (nowUp != prevUp && g_handlers_step.onArrowUp)
      g_handlers_step.onArrowUp(nowUp, stepper);
    prevUp = nowUp;
  }

  if (hasDown)
  {
    bool nowDn = toDown(tokDown);
    if (nowDn != prevDn && g_handlers_step.onArrowDown)
      g_handlers_step.onArrowDown(nowDn, stepper);
    prevDn = nowDn;
  }
  return true;
}

static bool isUpRunning = false;
static bool isDownRunning = false;

// Suggested high speed for 1/4 microstep (tune per load/supply)
// ความเร็วแนะนำเริ่มที่ 12000 sps (≈ 900 RPM @ 1/4) แล้วไล่ขึ้นได้
static constexpr float SUGGESTED_SPS = 4000.0f;

// callbacks
void onStpUp(bool down, UnifiedStepper &stepper)
{
  if (down)
  {
    // CCW with ramp
    stepper.rotateContinuous(true, SUGGESTED_SPS);
    isUpRunning = true;
    isDownRunning = false;
  }
  else
  {
    stepper.stop();
    isUpRunning = false;
  }
}

void onStpDown(bool down, UnifiedStepper &stepper)
{
  if (down)
  {
    // CW with ramp
    stepper.rotateContinuous(false, SUGGESTED_SPS);
    isDownRunning = true;
    isUpRunning = false;
  }
  else
  {
    stepper.stop();
    isDownRunning = false;
  }
}
