#pragma once
#include <Arduino.h>
#include <AccelStepper.h>

// some dude do this and works
// DIR+ => GPIO
// PUL+ => GPIO
// ENA+ => GND
// ENA- => GND
// DIR- => GND
// PUL- => GND

// Need to test | active low
// DIR- to GPIO
// PUL- to GPIO
// ENA- to GPIO (optional, can be connected to VCC)
// ENA+ to 3.3V/5V
// DIR+ to GND
// PUL+ to GND

// the one that works now
// DIR+ => GPIO
// PUL+ => GPIO
// ENA+ => GPIO
// ENA- => GND
// DIR- => GND
// PUL- => GND

// RED.........Phase A
// BLUE........Phase A Return
// GREEN......Phase B
// BLACK......Phase B Return

//  Driver TB6600 (STEP/DIR)
constexpr int PIN_STEP = 32;       //  STEP
constexpr int PIN_DIR = 25;        //  DIR
constexpr int PIN_ENABLE = 26;     //  ENA (optional, can be connected to VCC)
constexpr int STEP_DELAY_US = 800; // microseconds between steps (smaller=faster)

struct StepperProfile
{
  int step_pin;
  int dir_pin;
  int ena_Pin;
  int step_delay_us;
};

// ตัวควบคุม Stepper
class UnifiedStepper
{
public:
  UnifiedStepper(int stepPin, int dirPin, int enaPin, int stepDelayUs);
  bool begin();
  void stepCW(unsigned steps);  //(Clockwise)
  void stepCCW(unsigned steps); //(Counter-Clockwise)
  void rotateContinuous(bool cw, float sps = -1.0f);
  // void rotateCon_CW(); //ไม่ได้ใช้
  // void rotateCon_CCW();//ไม่ได้ใช้

  void stop();

  bool isContinuous() const { return continuous_; }
  bool directionCW() const { return dirCW_; }

  void setMaxSpeed(float steps_per_sec);
  void setAcceleration(float steps_per_sec2);
  void tick();

private:
  StepperProfile p_;
  bool continuous_ = false;
  bool dirCW_ = true;
  unsigned lastMicros_ = 0;

  AccelStepper stepper_{AccelStepper::DRIVER, PIN_STEP, PIN_DIR};

  float default_speed_sps_ = 0.0f;
  float default_accel_sps2_ = 0.0f;

  inline void ena_enable(); // helper ENA
  inline void ena_disable();
};

extern UnifiedStepper Nema17;

// -------------------- ปุ่มควบคุม --------------------
using STP_handler_step = void (*)(bool down, UnifiedStepper &stepper);

struct STP_handlers_step
{
  STP_handler_step onArrowUp = nullptr;   //  ทวนเข็ม
  STP_handler_step onArrowDown = nullptr; //  ตามเข็ม
};

// ตั้ง handler
void stepper_set_handlers(const STP_handlers_step &h);

// helper: case-insensitive startsWith
static inline bool _startsWith_ST(const String &s, const char *p);
// ประมวลผลสตริง "STP UP=DOWN DOWN=UP" (คล้าย servo)
bool stepper_handle_line(const String &line, UnifiedStepper &stepper);

// poll serial
// void stepper_serial_poll(UnifiedStepper& stepper); // no used for now

// call back
void onStpUp(bool down, UnifiedStepper &stepper);
void onStpDown(bool down, UnifiedStepper &stepper);

// เรียกทุก loop เพื่อทำให้การหมุนต่อเนื่องยังเดินสเต็ป (ถ้ากำลัง continuous)
// void stepper_tick(UnifiedStepper& stepper); // no used for now
