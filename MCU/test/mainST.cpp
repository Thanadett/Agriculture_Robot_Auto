
#include <Arduino.h>
#include "stepper.h"

// Create a global driver with pins and polarity from Config.h
Stepper stepper(PIN_PUL, PIN_DIR, PIN_ENA, TB6600_ACTIVE_LOW);

void setup() {
  Serial.begin(115200);
  delay(200);
  stepper.begin();

  // Enable the driver
  stepper.enable();

  // Move 1 revolution forward
  stepper.moveRevolutions(1.0f, STEP_INTERVAL_US);
  delay(300);

  // Move 180 degrees backward
  stepper.moveDegrees(-180.0f, STEP_INTERVAL_US);
  delay(300);

  // Move raw steps (e.g., 2000 microsteps)
  stepper.moveSteps(2000, STEP_INTERVAL_US);
  delay(300);

  // Disable when done (optional)
  stepper.disable();
}

void loop() {
  // Idle
  delay(1000);
}
