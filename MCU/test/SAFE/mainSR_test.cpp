#ifdef Node2
#include <Arduino.h>
#include "servo.h"
#include "stepper.h"
#include "serial_read2.h"
#include "config.h"
void setup()
{
  Serial.begin(115200);
  delay(1000);
  // -----Serial_Router Setup -----
  sr2::init();
  sr2::register_btn();        // servo
  sr2::register_stp(&Nema17); // stepper

  BTN_handlers servo_h;
  //gripper change to new 
  servo_h.onA = onBtnA; //manual camera =>
  servo_h.onB = onBtnB; //manual camera <=
  // servo_h.onX = onBtnX;
  servo_h.onY = onBtnY; //sg90 45 degree turn
  servo_h.onL = onBtnL; //360 servo disk ctrl by index 6
  servo_h.onR = onBtnR; //

  // callbacks for button A/B/X
  button_set_handlers(servo_h);

  STP_handlers_step stp_h;
  stp_h.onArrowUp = onStpUp;     // D-pad ขึ้น
  stp_h.onArrowDown = onStpDown; // D-pad ลง
  stepper_set_handlers(stp_h);

  TD8120MG.begin(); // Attach the servo on pin , set Hz and min/max pulse width
  MG996R_360.begin();
  MG996R.begin();
  SG90_180.begin();

  //======================== debug info ========================
  // int ch_1 = TD8120MG.begin();
  // int ch_2 = MG996R_360.begin();
  // int ch_3 = MG996R.begin();
  // if (ch_3 >= 0)
  //   Serial.printf("MG996R attached on channel %d (pin=%d)\n", ch_3, MG996R.pin());
  // else
  //   Serial.printf("MG996R attach FAILED (pin=%d)\n", MG996R.pin());
  // if (ch_2 >= 0)
  //   Serial.printf("MG996R_360 attached on channel %d (pin=%d)\n", ch_2, MG996R_360.pin());
  // else
  //   Serial.printf("MG996R_360 attach FAILED (pin=%d)\n", MG996R_360.pin());
  // if (ch_1 >= 0)
  //   Serial.printf("TD8120MG attached on channel %d (pin=%d)\n", ch_1, TD8120MG.pin());
  // else
  //   Serial.printf("TD8120MG attach FAILED (pin=%d)\n", TD8120MG.pin());
  // ======================== end debug info ========================
  // Shared action (works for both types):
  // - 180°: move to mechanical center (~90°)
  // - 360°: stop (neutral)
  TD8120MG.goCenterOrStop();
  MG996R_360.goCenterOrStop();
  // MG996R.setAngleDeg(170); 
  SG90_180.goCenterOrStop();
  // stepper
  Nema17.begin();
}

void loop()
{
  // button_serial_poll();
  sr2::poll(); // serial read2
  // Nema17.rotateContinuous(true);  // เริ่มหมุนต่อเนื่อง
  // stepper_tick(Nema17);  //use to trig step continous in loop (use with digitalw)
  Nema17.tick(); // call in loop to make stepper move (use with accelstepper lib)
  // MG996R_360.setSpeedPercent(+40); delay(800);
  // MG996R_360.goCenterOrStop();     delay(600);
  // MG996R_360.setSpeedPercent(-40); delay(800);
  // MG996R_360.goCenterOrStop();     delay(600);

  // MG996R.setAngleDeg(170);  delay(600);
  // MG996R.setAngleDeg(90); delay(600);
  // MG996R.setAngleDeg(20);  delay(600);
  // delay(5);
  // ===== 180° demo: absolute angles =====
  // MG996R.setAngleDeg(0);
  // delay(600);

  // ===== 360° demo: speed/direction =====
  // TD8120MG.setSpeedPercent(+50); delay(1500);
  // TD8120MG.goCenterOrStop();     delay(800);
  // TD8120MG.setSpeedPercent(-30); delay(1500);
  // TD8120MG.goCenterOrStop();     delay(1200);

  // ===== Shared command example: direct microseconds =====
  // MG996R.setPulseUs(1500); // center for 180°
  // TD8120MG.setPulseUs(1500); // stop for 360°
  // delay(800);

  //   Nema17.stepCCW(1600); // 1600 pulses = 1 revolution
  //   delay(3000);
  //   Nema17.stepCW(1600);
  //   delay(3000);
  //   delay(10);
}
#endif
