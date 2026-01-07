#pragma once
#include <Arduino.h>
#include "servo.h"    //  button_handle_line(...)
#include "stepper.h"  //  stepper_handle_line(...), UnifiedStepper

// ========================= Serial Read + Router (one-file) =========================
// Single Serial reader; dispatch lines to modules by prefix (BTN, STP)

namespace sr2 {


// Set max line length (default: 240 bytes)
void set_max_line_len(size_t n);


// Optional initialize/reset internal state
void init();

// Register built-in routes:
// - "BTN" => goes to button_handle_line(line) in servo module
// - "STP" => goes to stepper_handle_line(line, *stepper) in stepper module
void register_btn();                                 //  servo (BTN)
void register_stp(UnifiedStepper* stepper_ctx_ptr);  //  stepper (STP)

// อ่านจาก Serial และ dispatch ตาม prefix
// Read from Serial and dispatch by prefix
void poll();

// ========================= Advanced (ถ้าอยากเพิ่ม prefix อื่น) =========================
// Register custom routes (e.g., "DRV", "CAM"); handler returns true if handled.

using LineHandlerCtx = bool(*)(const String& line, void* ctx);
bool add_route(const char* prefix, LineHandlerCtx fn, void* ctx);

} // namespace sr2
