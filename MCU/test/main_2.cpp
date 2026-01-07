#include <Arduino.h>
#include "MotorDrive_En.h"
#include "enc_red_In.h"

// ใช้ค่า default จาก enc_red_In.h
DualEncoderReader enc;

void setup() {
  Serial.begin(115200);
  delay(200);

  motorDrive_begin();                 // ตั้งค่า PWM + เมนูคำสั่งจอย
  enc.begin(/*enable_internal_pullups=*/true); // เริ่มใช้งานเอ็นโค้ดเดอร์

  // หากทิศการนับของล้อใดกลับด้าน ให้เปิดใช้บรรทัดนี้
  
  // enc.setInvert(true, false);

  // หากต้องการ override PPR หรือรัศมีล้อ ให้ตั้งค่าที่นี่
  // enc.setPPR(...);
  // enc.setWheelRadius(...);
}

void loop() {
  // 1) อ่านคำสั่งจาก Serial (VW / P / PW4 / ESTOP / BTN)
  motorDrive_handleSerialOnce();

  // 2) เมื่อกดปุ่มจากจอย → “เตรียมเริ่มนับ” (เข้า ARMED) ระยะยัง 0 จนกว่าจะขยับจริง
  if (motorDrive_consumeTrackToggle()) {
    enc.trackArm();
  }

  // 3) อัปเดตเอ็นโค้ดเดอร์/มอเตอร์
  enc.update();
  motorDrive_update();

  // 4) อัปเดต FSM จับระยะ: ถึงเป้า (0.15 m) จะคืน true "ครั้งเดียว"
  const float GOAL_M = 0.15f;
  if (enc.updateTrackFSM(GOAL_M)) {
    // หยุดหนึ่งทีตามโจทย์ แล้ว FSM รีเซ็ตกลับ IDLE อัตโนมัติ (ระยะกลับเป็น 0)
    tgt_LF = tgt_LR = 0.f;
    tgt_RF = tgt_RR = 0.f;
    Serial.println("INFO: Reached 0.15 m -> STOP ONCE and auto-reset to IDLE.");
  }

  // 5) ฟีดแบ็กสถานะเป็นช่วง ๆ
  static uint32_t last = 0;
  const uint32_t FEEDBACK_INTERVAL_MS = 400;
  if (millis() - last >= FEEDBACK_INTERVAL_MS) {
    last = millis();
    enc.printFB(Serial);   // FB_ENC + TRACK
    enc.printFB2(Serial);  // FB_ENC2
  }

  delay(2);
}
