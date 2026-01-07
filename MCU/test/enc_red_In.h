#pragma once
// ===== enc_red_In.h =====
// โมดูลอ่านเอ็นโค้ดเดอร์ล้อหน้า/หลัง (Quadrature) ด้วย ESP32Encoder
// - ให้ตำแหน่งเชิงมุม [rad], ความเร็วเชิงมุม [rad/s]
// - ตั้งค่า PPR (pulses per revolution ของเพลาขาออก) ได้
// - กลับทิศการนับได้แยกล้อหน้า/หลัง
// - คำนวณเส้นรอบวงล้อ, ระยะสะสมรวม (TOTAL), ระยะที่เหลือจนครบรอบถัดไป
// - ฟีดแบ็ก Serial: FB_ENC, FB_ENC2, TRACK (สถานะจับระยะ)
// - โหมดจับระยะด้วยปุ่ม (FSM): IDLE → ARMED → TRACKING → STOP(ครั้งเดียว) → IDLE

#include <Arduino.h>
#include <ESP32Encoder.h>

// ---------------- กำหนดขาเอ็นโค้ดเดอร์ (ปรับตามฮาร์ดแวร์จริง) ----------------
#define ENC_R_A 16   // ล้อหลัง ขา A
#define ENC_R_B 17   // ล้อหลัง ขา B
#define ENC_F_A 5    // ล้อหน้า ขา A
#define ENC_F_B 18   // ล้อหน้า ขา B

// ---------------- ค่าพื้นฐานจากดาต้าชีตและอัตราทด (ปรับได้) ----------------
#define ENCODER_PPR_MOTOR 11.0f   // พัลส์ต่อรอบที่แกนมอเตอร์
#define REDUCTION_RATIO   270.0f  // อัตราทดเกียร์
#define QUAD_MULTIPLIER   2.0f    // 2=half-quad, 4=full-quad
#define ENC_WHEEL_RADIUS  0.0635f // รัศมีล้อ (เมตร) → D ≈ 0.127 m

// คำนวณ PPR ของเพลาขาออก (รวมโหมดนับ) เป็นค่าเริ่มต้น
static constexpr float ENCODER_PPR_OUTPUT_DEFAULT =
    ENCODER_PPR_MOTOR * REDUCTION_RATIO * QUAD_MULTIPLIER;

class DualEncoderReader {
public:
  // ---------------- สร้างอ็อบเจ็กต์ ----------------
  DualEncoderReader(int rearA = ENC_R_A, int rearB = ENC_R_B,
                    int frontA = ENC_F_A, int frontB = ENC_F_B,
                    float pulses_per_rev_output = ENCODER_PPR_OUTPUT_DEFAULT);

  // ---------------- การใช้งานหลัก ----------------
  void begin(bool enable_internal_pullups = true); // เรียกครั้งเดียวใน setup()
  void update();                                   // เรียกบ่อย ๆ ใน loop()
  void reset();                                    // รีเซ็ตค่าภายในทั้งหมด

  // ---------------- Getter พื้นฐาน ----------------
  float positionRearRad()  const { return posR_rad_; }   // มุมล้อหลัง [rad]
  float positionFrontRad() const { return posF_rad_; }   // มุมล้อหน้า [rad]
  float velocityRearRad()  const { return velR_rad_s_; } // ความเร็วล้อหลัง [rad/s]
  float velocityFrontRad() const { return velF_rad_s_; } // ความเร็วล้อหน้า [rad/s]

  long countsRear()  { return encR_.getCount(); } // พัลส์สะสมล้อหลัง (จากไลบรารี)
  long countsFront() { return encF_.getCount(); } // พัลส์สะสมล้อหน้า (จากไลบรารี)

  // ตั้ง/อ่านค่า PPR (เพลาขาออก)
  void  setPPR(float pulses_per_rev_output) { ppr_out_ = pulses_per_rev_output; }
  float getPPR() const { return ppr_out_; }

  // กลับทิศการนับ (1 หรือ -1) แยกหลัง/หน้า
  void setInvert(bool invert_rear, bool invert_front) {
    invR_ = invert_rear  ? -1 : 1;
    invF_ = invert_front ? -1 : 1;
  }

  // ---------------- ส่วน “ระยะทาง” ----------------
  void  setWheelRadius(float radius_m) { wheel_radius_m_ = radius_m; }
  float wheelRadius() const            { return wheel_radius_m_; }

  float distanceRearM()  const { return wheel_radius_m_ * posR_rad_; } // R*theta (ปัจจุบัน)
  float distanceFrontM() const { return wheel_radius_m_ * posF_rad_; }

  float totalDistanceRearM()  const { return total_dist_R_m_; } // TOTAL จาก |Δcount|
  float totalDistanceFrontM() const { return total_dist_F_m_; }

  float circumferenceM() const;                 // เส้นรอบวงล้อ (2πR)
  float remainingToNextRevRearM()  const;       // ระยะที่เหลือจนครบรอบ (ล้อหลัง)
  float remainingToNextRevFrontM() const;       // ระยะที่เหลือจนครบรอบ (ล้อหน้า)

  // ---------------- ฟีดแบ็กผ่าน Serial ----------------
  void printFB(Stream& s) const;   // FB_ENC + TRACK
  void printFB2(Stream& s) const;  // FB_ENC2

  // ---------------- โหมดจับระยะด้วยปุ่ม (FSM) ----------------
  // กดปุ่ม: เตรียมเริ่มนับ (ตั้ง baseline แล้วเข้า ARMED) — ระยะยังเป็น 0 จนกว่าจะเริ่มขยับ
  void trackArm();

  // ระยะที่นับตั้งแต่ baseline (เฉลี่ยล้อหน้า/หลัง)
  float trackedDistanceM() const;

  // ถึงระยะเป้าหมายหรือยัง (เช่น 0.15 m) — ใช้ตรวจเฉย ๆ
  bool hasReachedM(float goal_m) const;

  // อัปเดต FSM: ARMED→TRACKING เมื่อเริ่มขยับ, และเมื่อถึงเป้าหมายจะคืน true “ครั้งเดียว”
  // แล้วรีเซ็ตกลับ IDLE อัตโนมัติ (ระยะกลับเป็น 0)
  bool updateTrackFSM(float goal_m);

  // พิมพ์บรรทัดสรุปสถานะจับระยะ (state/goal/dist/reached_now)
  void printTrackLine(Stream& s, float goal_m = 0.15f) const;

private:
  // ---------------- ฮาร์ดแวร์/ไลบรารี ----------------
  int rA_, rB_, fA_, fB_;
  ESP32Encoder encR_;
  ESP32Encoder encF_;

  // ---------------- ค่าคาลิเบรต ----------------
  float ppr_out_ = ENCODER_PPR_OUTPUT_DEFAULT;
  int   invR_    = 1; // ทิศการนับล้อหลัง (1 หรือ -1)
  int   invF_    = 1; // ทิศการนับล้อหน้า (1 หรือ -1)
  float wheel_radius_m_ = ENC_WHEEL_RADIUS;

  // ---------------- สถานะคำนวณล่าสุด ----------------
  long     lastR_ = 0, lastF_ = 0;   // ค่าพัลส์ล่าสุด (ไว้คำนวณ delta)
  uint32_t last_ts_ms_ = 0;          // เวลาอัปเดตครั้งก่อน (ms)
  float posR_rad_   = 0.f, posF_rad_   = 0.f;
  float velR_rad_s_ = 0.f, velF_rad_s_ = 0.f;
  float total_dist_R_m_ = 0.f;        // ระยะสะสมรวม (TOTAL) ของล้อหลัง
  float total_dist_F_m_ = 0.f;        // ระยะสะสมรวม (TOTAL) ของล้อหน้า

  // ---------------- สถานะโหมดจับระยะ (FSM) ----------------
  enum class TrackState : uint8_t { IDLE = 0, ARMED = 1, TRACKING = 2 };
  TrackState track_state_ = TrackState::IDLE; // เริ่มต้น: ปกติ

  float track_base_R_m_ = 0.f;  // baseline TOTAL ตอนกดปุ่ม (ล้อหลัง)
  float track_base_F_m_ = 0.f;  // baseline TOTAL ตอนกดปุ่ม (ล้อหน้า)
  mutable bool track_reached_once_ = false; // ธงช่วยพิมพ์/ดีบัก
};
