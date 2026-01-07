#include "enc_red_In.h"
#include <math.h>

// ป้องกันกรณีไม่มี M_PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// เส้นคั่นสวย ๆ บน Serial
String cutline    = "===============================================================================";
String underline  = "_______________________________________________________________________________";

// ======================= Constructor =======================
DualEncoderReader::DualEncoderReader(int rearA, int rearB, int frontA, int frontB,
                                     float pulses_per_rev_output)
: rA_(rearA), rB_(rearB), fA_(frontA), fB_(frontB),
  ppr_out_(pulses_per_rev_output) {}

// ======================= begin() =======================
// เตรียมเอ็นโค้ดเดอร์ เรียกครั้งเดียวใน setup()
void DualEncoderReader::begin(bool enable_internal_pullups) {
  // ใช้ internal pull-up (ถ้าต้องการ)
  ESP32Encoder::useInternalWeakPullResistors = enable_internal_pullups ? UP : DOWN;

  // โหมด half-quadrature (นับขาขึ้น/ลงของ A เทียบ B)
  encR_.attachHalfQuad(rA_, rB_);
  encF_.attachHalfQuad(fA_, fB_);
  encR_.clearCount();
  encF_.clearCount();

  // เก็บค่าเริ่มต้น
  lastR_ = encR_.getCount();
  lastF_ = encF_.getCount();

  posR_rad_ = posF_rad_ = 0.f;
  velR_rad_s_ = velF_rad_s_ = 0.f;

  total_dist_R_m_ = 0.f;
  total_dist_F_m_ = 0.f;

  last_ts_ms_ = millis();

  // สถานะ FSM เริ่มที่ IDLE
  track_state_ = TrackState::IDLE;
  track_base_R_m_ = 0.f;
  track_base_F_m_ = 0.f;
  track_reached_once_ = false;
}

// ======================= reset() =======================
// รีเซ็ตค่าเอ็นโค้ดเดอร์และสถานะทั้งหมด
void DualEncoderReader::reset() {
  encR_.clearCount();
  encF_.clearCount();
  lastR_ = 0;
  lastF_ = 0;
  posR_rad_ = posF_rad_ = 0.f;
  velR_rad_s_ = velF_rad_s_ = 0.f;
  total_dist_R_m_ = 0.f;
  total_dist_F_m_ = 0.f;
  last_ts_ms_ = millis();

  track_state_ = TrackState::IDLE;
  track_base_R_m_ = 0.f;
  track_base_F_m_ = 0.f;
  track_reached_once_ = false;
}

// ======================= update() =======================
// อัปเดตตำแหน่ง/ความเร็ว/ระยะสะสม (เรียกบ่อย ๆ ใน loop)
void DualEncoderReader::update() {
  // อ่านพัลส์ปัจจุบัน (คูณด้วยทิศ)
  long curR = encR_.getCount() * invR_;
  long curF = encF_.getCount() * invF_;

  // คำนวณการเปลี่ยนแปลง
  long dR = curR - lastR_;
  long dF = curF - lastF_;
  lastR_  = curR;
  lastF_  = curF;

  // คำนวณเวลา dt (วินาที)
  uint32_t now = millis();
  float dt = (now - last_ts_ms_) / 1000.0f;
  if (dt <= 0.f) dt = 1e-3f; // กันหารศูนย์
  last_ts_ms_ = now;

  const float ppr = (ppr_out_ <= 0.f ? 1.f : ppr_out_);
  const float two_pi = 2.0f * (float)M_PI;
  const float C = circumferenceM(); // เส้นรอบวงล้อ (เมตร)

  // ----- ระยะสะสมรวม (TOTAL) นับแบบสัมบูรณ์ -----
  total_dist_R_m_ += fabsf(((float)dR / ppr) * C);
  total_dist_F_m_ += fabsf(((float)dF / ppr) * C);

  // ----- ตำแหน่งเชิงมุม -----
  posR_rad_ = ((float)curR / ppr) * two_pi;
  posF_rad_ = ((float)curF / ppr) * two_pi;

  // ----- ความเร็วเชิงมุม -----
  float revR_dt = ((float)dR / ppr) / dt;
  float revF_dt = ((float)dF / ppr) / dt;
  velR_rad_s_ = revR_dt * two_pi;
  velF_rad_s_ = revF_dt * two_pi;
}

// ======================= trackArm() =======================
// กดปุ่ม: ตั้ง baseline จาก TOTAL ปัจจุบัน และเข้าโหมด ARMED (ยังคงรายงานระยะ = 0 จนกว่าจะเริ่มขยับ)
void DualEncoderReader::trackArm() {
  track_base_R_m_ = total_dist_R_m_;
  track_base_F_m_ = total_dist_F_m_;
  track_state_    = TrackState::ARMED;
  track_reached_once_ = false;
}

// ======================= trackedDistanceM() =======================
// รายงานระยะตั้งแต่ baseline:
// - IDLE  → 0
// - ARMED → 0 (แม้ baseline ถูกตั้ง แต่ยังไม่ขยับ)
// - TRACKING → ค่าเฉลี่ย (TOTAL - baseline) ของสองล้อ (ไม่ติดลบ)
float DualEncoderReader::trackedDistanceM() const {
  if (track_state_ == TrackState::IDLE) return 0.f;
  float dR = total_dist_R_m_ - track_base_R_m_;
  float dF = total_dist_F_m_ - track_base_F_m_;
  if (dR < 0.f) dR = 0.f;
  if (dF < 0.f) dF = 0.f;
  return 0.5f * (dR + dF);
}

// ======================= hasReachedM() =======================
bool DualEncoderReader::hasReachedM(float goal_m) const {
  if (track_state_ == TrackState::IDLE) return false;
  return (trackedDistanceM() >= goal_m);
}

// ======================= updateTrackFSM() =======================
// อัปเดตสถานะจับระยะ:
// - ARMED     → ถ้าเริ่มขยับ (dist > 0) → TRACKING
// - TRACKING  → ถ้าถึงเป้าหมาย (>= goal) → คืน true "ครั้งเดียว", รีเซ็ตเป็น IDLE + baseline ขยับตาม ณ จุดนี้
bool DualEncoderReader::updateTrackFSM(float goal_m) {
  if (track_state_ == TrackState::IDLE) {
    track_reached_once_ = false;
    return false;
  }

  float dist = trackedDistanceM();

  // เริ่มขยับจาก ARMED → TRACKING
  if (track_state_ == TrackState::ARMED) {
    if (dist > 0.f) {
      track_state_ = TrackState::TRACKING;
    }
  }

  // ตรวจครบเป้าหมาย
  if (dist >= goal_m) {
    track_reached_once_ = true; // สำหรับฟีดแบ็กรอบนี้

    // รีเซ็ตกลับปกติ: baseline ขยับมา ณ ตำแหน่งนี้ + state = IDLE
    track_base_R_m_ = total_dist_R_m_;
    track_base_F_m_ = total_dist_F_m_;
    track_state_    = TrackState::IDLE;

    return true; // แจ้ง "ถึงเป้าแล้ว" ครั้งเดียว
  }

  track_reached_once_ = false;
  return false;
}

// ======================= printFB() =======================
// แสดง TOTAL + FB_ENC + TRACK (goal=0.15 m)
void DualEncoderReader::printFB(Stream& s) const {
  s.println(cutline);
  s.printf("TOTAL   | Total_R=%.6f Total_F=%.6f\n  |  goal(0.15m): ",
           total_dist_R_m_, total_dist_F_m_);

  const float two_pi = 2.0f * (float)M_PI;
  const float C      = circumferenceM();

  float revR = posR_rad_ / two_pi;
  float revF = posF_rad_ / two_pi;

  float fracR = revR - floorf(revR);
  float fracF = revF - floorf(revF);

  float distR_cycle = fracR * C;
  float distF_cycle = fracF * C;

  s.println(underline);
  s.printf("FB_ENC  | Vel_R=%.6f Vel_F=%.6f Dist_R=%.6f     Dist_F=%.6f\n",
           velR_rad_s_, velF_rad_s_, distR_cycle, distF_cycle);

  printTrackLine(s, 0.15f); // แสดงสถานะจับระยะ
}

// ======================= circumferenceM() =======================
float DualEncoderReader::circumferenceM() const {
  return 2.0f * (float)M_PI * wheel_radius_m_;
}

// ======================= remainingToNextRevRearM() =======================
float DualEncoderReader::remainingToNextRevRearM() const {
  if (wheel_radius_m_ <= 0.f) return 0.f;
  const float C = circumferenceM();
  const float rev = posR_rad_ / (2.0f * (float)M_PI);
  const float rev_next = ceilf(rev);
  float remain_rev = rev_next - rev;
  if (remain_rev < 1e-6f) remain_rev = 0.f;
  return remain_rev * C;
}

// ======================= remainingToNextRevFrontM() =======================
float DualEncoderReader::remainingToNextRevFrontM() const {
  if (wheel_radius_m_ <= 0.f) return 0.f;
  const float C = circumferenceM();
  const float rev = posF_rad_ / (2.0f * (float)M_PI);
  const float rev_next = ceilf(rev);
  float remain_rev = rev_next - rev;
  if (remain_rev < 1e-6f) remain_rev = 0.f;
  return remain_rev * C;
}

// ======================= printFB2() =======================
void DualEncoderReader::printFB2(Stream& s) const {
  s.printf("FB_ENC2 | C=%.6f                    DistLeft_R=%.6f DistLeft_F=%.6f\n",
           circumferenceM(),
           remainingToNextRevRearM(),
           remainingToNextRevFrontM());
  s.println(cutline);
}

// ======================= printTrackLine() =======================
void DualEncoderReader::printTrackLine(Stream& s, float goal_m) const {
  const float d = trackedDistanceM();
  const bool reached_now = (track_state_ != TrackState::IDLE) && (d >= goal_m);

  const char* st =
    (track_state_ == TrackState::IDLE)    ? "IDLE" :
    (track_state_ == TrackState::ARMED)   ? "ARMED" :
                                            "TRACKING";

  s.printf("TRACK   | state=%s goal=%.3fm dist=%.3fm reached_now=%d\n",
           st, goal_m, d, reached_now ? 1 : 0);
}
