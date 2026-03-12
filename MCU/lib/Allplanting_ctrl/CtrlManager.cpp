
#include "CtrlManager.h"
#include <Arduino.h>

void PlantingManager::ena_enable()  { digitalWrite(PIN_ENABLE, LOW); }
void PlantingManager::ena_disable() { digitalWrite(PIN_ENABLE, HIGH); }

long PlantingManager::mmToSteps(float mm) {
        return (long)(mm * STEPS_PER_MM);
    }
// trasnfer degree to Microseconds (500-2500)
int PlantingManager::angleToUs(float angle) {
    float pulse = 500.0 + (angle * (2000.0 / 180.0));
    return (int)(pulse + 0.5); // +0.5 (Rounding up)
}

int PlantingManager::moveToAngle(int desireAngle, int maxAngle){
    return map(desireAngle, 0, maxAngle, 500, 2500);
}

// อ่านค่า limit switch
// NO + INPUT_PULLUP → ปกติ HIGH, กด = LOW
bool PlantingManager::checkLimitSwitch() {
    return digitalRead(limitSwitchPin) == LOW;
}


void PlantingManager::begin() {
    // Setup Servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    servo_gripper.setPeriodHertz(50);
    servo_linear.setPeriodHertz(50);
    servo_plate.setPeriodHertz(50);
    servo_gripper.attach(SERVO_PIN_gripper, 500, 2500); // gripper
    servo_linear.attach(SERVO_PIN_linear, 500, 2500); // linear motion
    servo_plate.attach(SERVO_PIN_plate, 500, 2500);
    servo_cam.attach(SERVO_PIN_cam, 500, 2500);
    // Setup Stepper
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(limitSwitchPin, INPUT_PULLUP);
    ena_enable(); // เปิด Driver ทันที
    
    // set up Invert like before (TB6600)
    _stepper.setPinsInverted(true, true, false); 

    _stepper.setMaxSpeed(1200.0);      // velocity of SUGGESTED_SPS 
    _stepper.setAcceleration(600.0); // Acceleration 
    _stepper.setMinPulseWidth(20);      // TB6600 Pulse width

    _stepper.setCurrentPosition(0); // home position at 0
    servo_plate.writeMicroseconds(angleToUs(HomeAngle));}

void PlantingManager::startPlantPattern() {
    if (_activeMode == IDLE) {
        _activeMode = PLANTING;
        _currentStep = 1;
        _previous_Millis = millis();
    }
}

void PlantingManager::LoadPattern() {
    if (_activeMode == IDLE) {
        _activeMode = LOADING;
        _currentStep = 1;
        _previous_Millis = millis();
    }
}

void PlantingManager::testpattern() {
    if (_activeMode == IDLE) {
        _activeMode = TESTING;
        _currentStep = 1;
        _previous_Millis = millis();
    }
}

void PlantingManager::mv_cam_up_pattern() {
    if (_activeMode == IDLE) {
        _activeMode = MOVECAM_UP;
        _currentStep = 1;
        _previous_Millis = millis();
    }
}

void PlantingManager::mv_cam_down_pattern() {
    if (_activeMode == IDLE) {
        _activeMode = MOVECAM_DOWN;
        _currentStep = 1;
        _previous_Millis = millis();
    }
}

void PlantingManager::resetPattern() {
    if (_activeMode == IDLE) {
        _activeMode = RESET;
        _currentStep = 1;
        _previous_Millis = millis();
    }
}

void PlantingManager::update() {
    // commands Stepper to process every steps (Non-blocking)
    // Serial.println(_stepper.currentPosition());
    // runStepper();

    // === LIMIT SWITCH GUARD ===
        if (checkLimitSwitch()) {
        _stepper.stop();
        _stepper.setCurrentPosition(0); // ถือว่านี่คือ home
    }
    _stepper.run();
    // ==========================

    if (_activeMode == IDLE) return;
    // if (!_isPatternRunning) return;

    unsigned long currentMillis = millis();
    // unsigned long elapsed_stepper = currentMillis - _previous_stepperMillis;
    unsigned long elapsed_pattern = currentMillis - _previous_Millis;

    // Debug Print each Step
    if (_currentStep != _lastPrintedStep) {
        Serial.print("Executing Step: "); Serial.println(_currentStep);
        _lastPrintedStep = _currentStep;
    }
    // static unsigned long lastPrint=0;
    // if(millis()-lastPrint>300){
    //     lastPrint=millis();
    //     Serial.print("pos:");
    //     Serial.print(_stepper.currentPosition());
    //     Serial.print(" target:");
    //     Serial.println(_stepper.targetPosition());
    // }
    switch (_activeMode) {
        
        case PLANTING:
            switch (_currentStep) {
                case 1: // Reset Servo positions
                    servo_gripper.write(160);
                    servo_linear.write(moveToAngle(0, 270));
                    // servo_cam.write(0);
                    if (elapsed_pattern >= 2000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 2: //Move Stepper to idle
                    // wait until ___ms and Stepper reach it's destination
                    if (!_stepperCommandSent) {
                        _stepper.moveTo(mmToSteps(0));
                        _stepperCommandSent = true;
                    }

                    if (_stepper.distanceToGo() == 0) {
                        _stepperCommandSent = false;
                        _currentStep++;
                        _previous_Millis = currentMillis;
                    }
                    break;
                case 3: //servo linear down
                    servo_linear.write(moveToAngle(180, 270));
                    if (elapsed_pattern >= 2500) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 4: //lead linear down
                    if (!_stepperCommandSent) {
                    _stepper.moveTo(mmToSteps(-170.0));
                    _stepperCommandSent = true;
                    }

                    if (_stepper.distanceToGo() == 0) {
                        _stepperCommandSent = false;
                        _currentStep++;
                        _previous_Millis = currentMillis;
                    }
                    break;
                case 5: //linear 
                    servo_linear.write(moveToAngle(190, 270));
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 6: //linear up
                    servo_linear.write(moveToAngle(70, 270));
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 7: //gripper open, drop down plant, prepare digging
                    servo_gripper.write(80);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 8: //gripper dig 
                    servo_gripper.write(145);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 9: //linear
                    servo_linear.write(moveToAngle(140, 270));
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 10: //gripper dig 
                    servo_gripper.write(50);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 11: //linear
                    servo_linear.write(moveToAngle(0, 270));
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 12: //gripper dig 
                    servo_gripper.write(150);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 13: //linear
                    servo_linear.write(60);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 14: //gripper dig 
                    servo_gripper.write(145);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 15: //linear
                    servo_linear.write(0);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 16: //linear
                    servo_linear.write(25);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 17: //gripper dig 
                    servo_gripper.write(100);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 18: //linear
                    servo_linear.write(0);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 19: //lead linear 
                    if (!_stepperCommandSent) {
                    _stepper.moveTo(mmToSteps(-160.0));
                    _stepperCommandSent = true;
                    }

                    if (_stepper.distanceToGo() == 0) {
                        _stepperCommandSent = false;
                        _currentStep++;
                        _previous_Millis = currentMillis;
                    }
                    break;
                case 20: //gripper open,
                    servo_gripper.write(20);
                    if (elapsed_pattern >= 2000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 21: //lead linear 
                    if (!_stepperCommandSent) {
                    _stepper.moveTo(mmToSteps(-175.0));
                    _stepperCommandSent = true;
                    }

                    if (_stepper.distanceToGo() == 0) {
                        _stepperCommandSent = false;
                        _currentStep++;
                        _previous_Millis = currentMillis;
                    }
                    break;
                case 22: //linear
                    servo_linear.write(moveToAngle(100, 270));
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 23: //gripper open,
                    servo_gripper.write(20);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 24: //gripper go beside plant 1/3
                    servo_gripper.write(45);
                    if (elapsed_pattern >= 300) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 25: //gripper go beside plant 2/3
                    servo_gripper.write(70);
                    if (elapsed_pattern >= 300) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 26: //gripper go beside plant 3/3
                    servo_gripper.write(125);
                    if (elapsed_pattern >= 800) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 27: //gripper 
                    servo_gripper.write(80);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 28: //linear up 
                    servo_linear.write(moveToAngle(0, 270));
                    if (elapsed_pattern >= 1500) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 29: // // lead up to to prevert crash
                    if
                     (!_stepperCommandSent) {
                        _stepper.moveTo(mmToSteps(-140.0));
                        _stepperCommandSent = true;
                    }

                    if (_stepper.distanceToGo() == 0) {
                        _stepperCommandSent = false;
                        _currentStep++;
                        _previous_Millis = currentMillis;
                    }
                    break;
                case 30: //
                    servo_gripper.write(160);
                    // servo_gripper.write(100); 
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 31: //gripper, stepper go idle, stepper go to the point 
                    if (!_stepperCommandSent) {
                        _stepper.moveTo(mmToSteps(100.0));
                        _stepperCommandSent = true;
                    }

                    if (checkLimitSwitch()) {
                        // โดน limit switch แล้ว → นี่คือ home
                        _stepper.stop();
                        _stepper.setCurrentPosition(0); // reset encoder เป็น 0
                        _stepperCommandSent = false;
                        _currentStep++;
                        _previous_Millis = currentMillis;                    
                    }
                    break;
                case 32: // จบ Pattern
                    // _isPatternRunning = false;
                    _stepperCommandSent = false; 
                    _currentStep = 0;
                    _activeMode = IDLE;
                    // Serial.println("Plant Pattern Done!");
                    break;
            }
            break;
        case TESTING:
            switch (_currentStep) {
                case 1: // Reset Servo positions
                    servo_gripper.write(160);
                    servo_linear.write(moveToAngle(0, 270));
                    // servo_cam.write(0);
                    if (elapsed_pattern >= 2000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 2: //Move Stepper to idle
                    // wait until ___ms and Stepper reach it's destination
                    if (!_stepperCommandSent) {
                        _stepper.moveTo(mmToSteps(0));
                        _stepperCommandSent = true;
                    }

                    if (_stepper.distanceToGo() == 0) {
                        _stepperCommandSent = false;
                        _currentStep++;
                        _previous_Millis = currentMillis;
                    }
                    break;
                case 3: //servo linear down
                    servo_linear.write(moveToAngle(180, 270));
                    if (elapsed_pattern >= 2500) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 4: //lead linear down
                    if (!_stepperCommandSent) {
                    _stepper.moveTo(mmToSteps(-175.0));
                    _stepperCommandSent = true;
                    }

                    if (_stepper.distanceToGo() == 0) {
                        _stepperCommandSent = false;
                        _currentStep++;
                        _previous_Millis = currentMillis;
                    }
                    break;
                case 5: //linear up
                    servo_linear.write(moveToAngle(80, 270));
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 6: //gripper open, drop down plant, prepare digging
                    servo_gripper.write(100);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 7: //lead linear down
                    if (!_stepperCommandSent) {
                    _stepper.moveTo(mmToSteps(-170.0));
                    _stepperCommandSent = true;
                    }

                    if (_stepper.distanceToGo() == 0) {
                        _stepperCommandSent = false;
                        _currentStep++;
                        _previous_Millis = currentMillis;
                    }
                    break;
                case 8: //gripper dig 
                    servo_gripper.write(145);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 9: //linear
                    servo_linear.write(moveToAngle(150, 270));
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 10: //gripper dig 
                    servo_gripper.write(60);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 11: //lead linear up
                    if (!_stepperCommandSent) {
                    _stepper.moveTo(mmToSteps(-160.0));
                    _stepperCommandSent = true;
                    }

                    if (_stepper.distanceToGo() == 0) {
                        _stepperCommandSent = false;
                        _currentStep++;
                        _previous_Millis = currentMillis;
                    }
                    break;
                case 12: //linear
                    servo_linear.write(moveToAngle(20, 270));
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 13: //gripper dig 
                    servo_gripper.write(155);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 14: //linear
                    servo_linear.write(moveToAngle(140, 270));
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 15: //gripper dig 
                    servo_gripper.write(60);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 16: //linear
                    servo_linear.write(moveToAngle(0, 270));
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 17: //lead linear 
                    if (!_stepperCommandSent) {
                    _stepper.moveTo(mmToSteps(-150.0));
                    _stepperCommandSent = true;
                    }

                    if (_stepper.distanceToGo() == 0) {
                        _stepperCommandSent = false;
                        _currentStep++;
                        _previous_Millis = currentMillis;
                    }
                    break;
                case 18: //gripper open,
                    servo_gripper.write(20);
                    if (elapsed_pattern >= 2000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 19: //lead linear 
                    if (!_stepperCommandSent) {
                    _stepper.moveTo(mmToSteps(-170.0));
                    _stepperCommandSent = true;
                    }

                    if (_stepper.distanceToGo() == 0) {
                        _stepperCommandSent = false;
                        _currentStep++;
                        _previous_Millis = currentMillis;
                    }
                    break;
                case 20: //linear
                    servo_linear.write(moveToAngle(90, 270));
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 21: //gripper open,
                    servo_gripper.write(20);
                    if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 22: //gripper go beside plant 1/3
                    servo_gripper.write(45);
                    if (elapsed_pattern >= 500) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 23: //gripper go beside plant 2/3
                    servo_gripper.write(70);
                    if (elapsed_pattern >= 500) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 24: //gripper go beside plant 3/3
                    servo_gripper.write(120);
                    if (elapsed_pattern >= 500) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 25: //gripper go out
                    servo_gripper.write(80);
                    if (elapsed_pattern >= 500) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 26: //linear up 
                    servo_linear.write(moveToAngle(0, 270));
                    if (elapsed_pattern >= 1500) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 27: // // lead up to to prevert crash
                    if
                     (!_stepperCommandSent) {
                        _stepper.moveTo(mmToSteps(-150.0));
                        _stepperCommandSent = true;
                    }

                    if (_stepper.distanceToGo() == 0) {
                        _stepperCommandSent = false;
                        _currentStep++;
                        _previous_Millis = currentMillis;
                    }
                    break;
                case 28: //
                    servo_gripper.write(160);
                    // servo_gripper.write(100); 
                    if (elapsed_pattern >= 2000) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 29: //gripper, stepper go idle, stepper go to the point 
                    if (!_stepperCommandSent) {
                        _stepper.moveTo(mmToSteps(100.0));
                        _stepperCommandSent = true;
                    }

                    if (checkLimitSwitch()) {
                        // โดน limit switch แล้ว → นี่คือ home
                        _stepper.stop();
                        _stepper.setCurrentPosition(0); // reset encoder เป็น 0
                        _stepperCommandSent = false;
                        _currentStep++;
                        _previous_Millis = currentMillis;                    
                    }
                    break;
                case 30: // จบ Pattern
                    // _isPatternRunning = false;
                    _stepperCommandSent = false; 
                    _currentStep = 0;
                    _activeMode = IDLE;
                    // Serial.println("Plant Pattern Done!");
                    break;
            }
            break;
        case LOADING: 
            switch (_currentStep) {
                case 1: // Case 1 _servo_plate_CurrentAngle starts with 16.3 degrees
                    _servo_plate_CurrentAngle = HomeAngle;
                    servo_plate.writeMicroseconds(angleToUs(HomeAngle)); // 16.3 degrees
                    if (elapsed_pattern >= 2000) { _currentStep++; _previous_Millis = currentMillis; }

                    break;
                case 2: // ค่อยๆ ขยับทีละ 16.3 องศา (Smooth Move)
                // ตรวจสอบว่าถึงเวลาอัปเดตองศาหรือยัง (คล้าย delay(1) แต่ไม่ block)
                    if (currentMillis - _lastServoUpdate >= 15) { 
                        _lastServoUpdate = currentMillis;
                        if (_servo_plate_CurrentAngle < 16.3*2) {
                            _servo_plate_CurrentAngle += 0.8; // ความเร็วการขยับ
                            servo_plate.writeMicroseconds(angleToUs(_servo_plate_CurrentAngle));
                        } else {
                        if (elapsed_pattern >= 1000) { _currentStep++; _previous_Millis = currentMillis; }
                        }
                    }
                    break;
                case 3:
                    // servo_plate.writeMicroseconds(angleToUs(HomeAngle));
                    if (elapsed_pattern >= 1000) { 
                    // Serial.println("Loading System Done!");
                    _activeMode = IDLE; 
                    _currentStep = 0;   
                    _previous_Millis = currentMillis; 
                    }
                    break;
            }
            break;
        case MOVECAM_UP: 
            switch (_currentStep) {
                case 1: // 
                    servo_cam.write(0);
                    if (elapsed_pattern >= 1500) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 2:
                    _currentStep = 0;
                    _activeMode = IDLE;
                    break;
            }
            break;
        case MOVECAM_DOWN: 
            switch (_currentStep) {
                case 1: // 
                    servo_cam.write(140);
                    if (elapsed_pattern >= 1500) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 2:
                    _currentStep = 0;
                    _activeMode = IDLE;
                    break;
            }
            break;
        case RESET:
            switch (_currentStep) {
                case 1: // 
                    servo_cam.write(140);
                    servo_gripper.write(160);
                    servo_linear.write(moveToAngle(0, 270));
                    servo_plate.writeMicroseconds(angleToUs(HomeAngle));
                    if (elapsed_pattern >= 1500) { _currentStep++; _previous_Millis = currentMillis; }
                    break;
                case 2:
                    if (!_stepperCommandSent) {
                        _stepper.moveTo(mmToSteps(100.0));
                        _stepperCommandSent = true;
                    }

                    if (checkLimitSwitch()) {
                        // โดน limit switch แล้ว → นี่คือ home
                        _stepper.stop();
                        _stepper.setCurrentPosition(0); // reset encoder เป็น 0
                        _stepperCommandSent = false;
                        _currentStep++;
                        _previous_Millis = currentMillis;                    
                    }
                    break;
                case 3:
                    _stepperCommandSent = false; 
                    _currentStep = 0;
                    _activeMode = IDLE;
                    break;
            }
            break;
            
    }
}


void PlantingManager::stopAll() {
    _activeMode = IDLE;
    _currentStep = 0;
    _stepper.stop();
    // _stepper.setCurrentPosition(_stepper.currentPosition()); // บังคับให้ Target เท่ากับตำแหน่งปัจจุบันทันที
    _stepperCommandSent = false;
}

