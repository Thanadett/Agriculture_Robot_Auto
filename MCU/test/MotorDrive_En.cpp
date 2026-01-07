#include "MotorDrive_En.h"
#include <math.h>

// ========================= ตัวแปรสถานะหลัก =========================
String rx_line;
bool estop = false;
uint32_t last_cmd_ms = 0;

float tgt_LF = 0.f, tgt_LR = 0.f, tgt_RF = 0.f, tgt_RR = 0.f;
float out_LF = 0.f, out_LR = 0.f, out_RF = 0.f, out_RR = 0.f;

// ธง one-shot สำหรับปุ่ม TRACK จากจอย
static bool g_track_toggle_edge = false;

// ========================= ยูทิลิตี้ภายใน =========================
static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi ? hi : x);
}

static void setupPwmPin(int pin, int ch) {
  pinMode(pin, OUTPUT);
  ledcSetup(ch, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(pin, ch);
  ledcWrite(ch, 0);
}

// เขียน PWM แบบ signed ด้วย 2 ช่อง (เดินหน้า/ถอยหลัง)
static void writeSignedPWM(int ch_fwd, int ch_rev, float pwm_norm, bool invert = false) {
  if (invert) pwm_norm = -pwm_norm;
  pwm_norm = clampf(pwm_norm, -1.0f, 1.0f);
  const int duty = (int)lroundf(fabsf(pwm_norm) * PWM_MAX_DUTY);

  if (pwm_norm >= 0.f) {
    ledcWrite(ch_fwd, duty);
    ledcWrite(ch_rev, 0);
  } else {
    ledcWrite(ch_fwd, 0);
    ledcWrite(ch_rev, duty);
  }
}

static void applyMotors(float LF, float LR, float RF, float RR) {
  // ถ้า E-Stop → บังคับหยุด
  if (estop) { LF = LR = RF = RR = 0.f; }

  // จำกัดความชันการเปลี่ยนค่า (slew-rate)
  float dLF = clampf(LF - out_LF, -RAMP_STEP, RAMP_STEP);
  float dLR = clampf(LR - out_LR, -RAMP_STEP, RAMP_STEP);
  float dRF = clampf(RF - out_RF, -RAMP_STEP, RAMP_STEP);
  float dRR = clampf(RR - out_RR, -RAMP_STEP, RAMP_STEP);

  out_LF = clampf(out_LF + dLF, -1.f, 1.f);
  out_LR = clampf(out_LR + dLR, -1.f, 1.f);
  out_RF = clampf(out_RF + dRF, -1.f, 1.f);
  out_RR = clampf(out_RR + dRR, -1.f, 1.f);

  // เขียนค่า PWM ไปยังมอเตอร์
  writeSignedPWM(CH_LF_IN1, CH_LF_IN2, out_LF, INVERT_LF);
  writeSignedPWM(CH_LR_IN1, CH_LR_IN2, out_LR, INVERT_LR);
  writeSignedPWM(CH_RF_IN1, CH_RF_IN2, out_RF, INVERT_RF);
  writeSignedPWM(CH_RR_IN1, CH_RR_IN2, out_RR, INVERT_RR);
}

// ========================= Parsing คำสั่ง Serial =========================
static bool startsWith(const String &s, const char *p) {
  size_t n = strlen(p);
  return s.length() >= (int)n && s.substring(0, n).equalsIgnoreCase(p);
}

static bool parseFloatAfterEquals(const String &s, const char *key, float &out) {
  int idx = s.indexOf(key);
  if (idx < 0) return false;
  idx += strlen(key);
  int end = idx;
  while (end < (int)s.length() && !isWhitespace(s[end])) end++;
  String num = s.substring(idx, end);
  out = num.toFloat();
  return true;
}

static bool parseIntAfterEquals(const String &s, const char *key, int &out) {
  int idx = s.indexOf(key);
  if (idx < 0) return false;
  idx += strlen(key);
  int end = idx;
  while (end < (int)s.length() && !isWhitespace(s[end])) end++;
  String num = s.substring(idx, end);
  out = num.toInt();
  return true;
}

void cmdVW_to_targets(float V_mps, float W_radps) {
  // แปลง V,W → ความเร็วล้อซ้าย/ขวา
  float vL = V_mps - (W_radps * (WHEEL_SEP * 0.5f));
  float vR = V_mps + (W_radps * (WHEEL_SEP * 0.5f));

  // แปลงเป็นความเร็วเชิงมุมล้อ (rad/s)
  float omegaL = vL / WHEEL_RADIUS;
  float omegaR = vR / WHEEL_RADIUS;

  // แมปเป็น PWM (-1..1) ด้วย MAX_OMEGA_FOR_FULL
  float pwmL = clampf(omegaL / MAX_OMEGA_FOR_FULL, -1.f, 1.f);
  float pwmR = clampf(omegaR / MAX_OMEGA_FOR_FULL, -1.f, 1.f);

  // กระจายไปล้อหน้า/หลังของฝั่งเดียวกัน
  tgt_LF = pwmL;  tgt_LR = pwmL;
  tgt_RF = pwmR;  tgt_RR = pwmR;
}

static void handleLine(String line) {
  line.trim();
  if (line.isEmpty()) return;

  // ----- ESTOP -----
  if (startsWith(line, "ESTOP")) {
    int sp = line.indexOf(' ');
    int val = 1;
    if (sp >= 0) {
      String tail = line.substring(sp + 1);
      tail.trim();
      val = tail.toInt();
    }
    estop = (val != 0);
    if (estop) tgt_LF = tgt_LR = tgt_RF = tgt_RR = 0.f;
    Serial.printf("ACK ESTOP=%d\n", estop ? 1 : 0);
    return;
  }

  // ----- VW -----
  if (startsWith(line, "VW")) {
    float V = 0.f, W = 0.f;
    bool okV = parseFloatAfterEquals(line, "V=", V);
    bool okW = parseFloatAfterEquals(line, "W=", W);
    if (okV && okW) {
      last_cmd_ms = millis();
      cmdVW_to_targets(V, W);
      Serial.printf("ACK VW V=%.3f W=%.3f -> L=%.2f R=%.2f\n", V, W, tgt_LF, tgt_RF);
    } else {
      Serial.println("ERR VW FORMAT (ต้องเป็น: VW V=<m/s> W=<rad/s>)");
    }
    return;
  }

  // ----- P / PW4 -----
  if (startsWith(line, "P")) {
    if (startsWith(line, "PW4")) {
      float lf=0.f, lr=0.f, rf=0.f, rr=0.f;
      bool ok1 = parseFloatAfterEquals(line, "LF=", lf);
      bool ok2 = parseFloatAfterEquals(line, "LR=", lr);
      bool ok3 = parseFloatAfterEquals(line, "RF=", rf);
      bool ok4 = parseFloatAfterEquals(line, "RR=", rr);
      if (ok1 && ok2 && ok3 && ok4) {
        last_cmd_ms = millis();
        tgt_LF = clampf(lf, -1.f, 1.f);
        tgt_LR = clampf(lr, -1.f, 1.f);
        tgt_RF = clampf(rf, -1.f, 1.f);
        tgt_RR = clampf(rr, -1.f, 1.f);
        Serial.printf("ACK PW4 LF=%.2f LR=%.2f RF=%.2f RR=%.2f\n", tgt_LF, tgt_LR, tgt_RF, tgt_RR);
      } else {
        Serial.println("ERR PW4 FORMAT (ต้องเป็น: PW4 LF= LR= RF= RR=)");
      }
    } else {
      float L=0.f, R=0.f;
      bool okL = parseFloatAfterEquals(line, "L=", L);
      bool okR = parseFloatAfterEquals(line, "R=", R);
      if (okL && okR) {
        last_cmd_ms = millis();
        L = clampf(L, -1.f, 1.f);
        R = clampf(R, -1.f, 1.f);
        tgt_LF = tgt_LR = L;
        tgt_RF = tgt_RR = R;
        Serial.printf("ACK P L=%.2f R=%.2f\n", L, R);
      } else {
        Serial.println("ERR P FORMAT (ต้องเป็น: P L=<..> R=<..> หรือ PW4 ...)");
      }
    }
    return;
  }

  // ----- BTN (ปุ่มจากจอย) -----
  if (startsWith(line, "BTN")) {
    // รูปแบบง่าย: "BTN TRACK"
    if (line.indexOf("TRACK") >= 0) {
      g_track_toggle_edge = true;
      last_cmd_ms = millis();
      Serial.println("ACK BTN TRACK");
      return;
    }
    // ตัวอย่าง: "BTN A=1" → ขอบขึ้น (rising edge)
    static int lastA = 0;
    int A = 0;
    if (parseIntAfterEquals(line, "A=", A)) {
      if ((A != 0) && (lastA == 0)) {
        g_track_toggle_edge = true;
        last_cmd_ms = millis();
        Serial.println("ACK BTN A rising -> TRACK TOGGLE");
      }
      lastA = (A != 0);
      return;
    }
  }

  Serial.println("ERR UNKNOWN CMD");
}

// ========================= Public API =========================
void motorDrive_begin() {
  // ตั้งค่า PWM
  setupPwmPin(LF_IN1, CH_LF_IN1);
  setupPwmPin(LF_IN2, CH_LF_IN2);
  setupPwmPin(LR_IN1, CH_LR_IN1);
  setupPwmPin(LR_IN2, CH_LR_IN2);
  setupPwmPin(RF_IN1, CH_RF_IN1);
  setupPwmPin(RF_IN2, CH_RF_IN2);
  setupPwmPin(RR_IN1, CH_RR_IN1);
  setupPwmPin(RR_IN2, CH_RR_IN2);

  // เริ่มจากหยุดนิ่ง
  applyMotors(0, 0, 0, 0);
  last_cmd_ms = millis();

  // เมนูคำสั่ง
  Serial.println("READY 4WD MOTOR-ONLY @115200");
  Serial.println("Commands: ");
  Serial.println("  VW V=<m/s> W=<rad/s>");
  Serial.println("  P L=<-1..1> R=<-1..1>");
  Serial.println("  PW4 LF=<..> LR=<..> RF=<..> RR=<..>");
  Serial.println("  ESTOP 0|1");
  Serial.println("  BTN TRACK   (หรือ BTN A=1 เมื่อมีขอบขึ้น)");
}

void motorDrive_handleSerialOnce() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (rx_line.length() > 0) {
        handleLine(rx_line);
        rx_line = "";
      }
    } else {
      if (rx_line.length() < 240) rx_line += c;
    }
  }
}

void motorDrive_update() {
  // Watchdog: ถ้าไม่มีคำสั่งนาน → ลดค่าเป้าหมายลง
  uint32_t now = millis();
  if (!estop && (now - last_cmd_ms > CMD_TIMEOUT_MS)) {
    tgt_LF *= IDLE_DECAY;
    tgt_LR *= IDLE_DECAY;
    tgt_RF *= IDLE_DECAY;
    tgt_RR *= IDLE_DECAY;
    if (fabsf(tgt_LF) < 0.02f) tgt_LF = 0.f;
    if (fabsf(tgt_LR) < 0.02f) tgt_LR = 0.f;
    if (fabsf(tgt_RF) < 0.02f) tgt_RF = 0.f;
    if (fabsf(tgt_RR) < 0.02f) tgt_RR = 0.f;
  }

  applyMotors(tgt_LF, tgt_LR, tgt_RF, tgt_RR);
}

bool motorDrive_consumeTrackToggle() {
  if (g_track_toggle_edge) {
    g_track_toggle_edge = false;
    return true;
  }
  return false;
}
