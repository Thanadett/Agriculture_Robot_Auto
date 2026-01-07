#pragma once
#include <Arduino.h>
#include "config.h"

#ifndef MAX_YAW_RATE_FOR_FULL
#define MAX_YAW_RATE_FOR_FULL 0.24f
#endif

// ---------------- Globals (defined in .cpp) ----------------
extern String rx_line;       // Serial line buffer
extern bool estop;           // Emergency stop flag
extern uint32_t last_cmd_ms; // Last time we received a command

// Target PWM for each wheel (normalized -1..1)
extern float tgt_LF, tgt_LR, tgt_RF, tgt_RR;
// Output PWM after slew-rate limiting (normalized -1..1)
extern float out_LF, out_LR, out_RF, out_RR;

// ---------------- Public API ----------------
// Initialize LEDC channels, pins and set motors to 0
void motorDrive_begin();

// Parse one or more Serial command lines if available (VW / P / PW4 / ESTOP)
void motorDrive_handleSerialOnce();

// Run watchdog + idle decay + slew-rate + deadtime + write PWM
void motorDrive_update();

// Map (V, W) to left/right wheel linear velocities then to PWM targets
void cmdVW_to_targets(float V_mps, float W_radps);
