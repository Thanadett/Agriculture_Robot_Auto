#pragma once

#include <ESP32Servo.h>
#include <AccelStepper.h>
#include "config.h"

enum TaskMode { IDLE, PLANTING, LOADING, TESTING };
class PlantingManager {
private:
    TaskMode _activeMode = IDLE;
    Servo servo_gripper, servo_linear, servo_plate;
    AccelStepper _stepper;
    
    
    //step per mm = [(step per rev x microstepping ) x gear ratio ] / lead type (T8)
    const float STEPS_PER_MM = 100.0; //  Microstep 1/8 | ((200*8)*(2/1))8 = 400
    unsigned long _previous_stepperMillis = 0; // collect the lastest time to move
    unsigned long _previous_Millis = 0; // collect the lastest time to move
    int _currentStep = 0; // to check which Step in the Pattern
    // bool _isPatternRunning = false; // to check if the program is running 
    int _lastPrintedStep = -1;
 
    long _currentPosSteps = 0;      // remember current position
    long _targetPosSteps = 0;       // goal to move
    unsigned long _lastStepMicros = 0; // lastest pulse
    // bool _isStepping = false;    // to check if stepper is moving
    bool _stepperCommandSent = false; 
    bool _servoCommandSent = false;
    
    float HomeAngle = 16.3; // plate degrees
    float _servo_plate_CurrentAngle = 16.3; // เก็บองศาปัจจุบันของ Servo 
    unsigned long _lastServoUpdate = 0; // เวลาที่อัปเดตองศาล่าสุด

    int desireAngle = 0; // องศาที่ต้องการ (0-270)
    // Helper for TB6600 (Active-Low)
    void ena_enable();
    void ena_disable();

public:
    PlantingManager() : _stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR) {}
    int getCurrentStep() { return _currentStep; }
    long mmToSteps(float StepPermm);
    int angleToUs(float Angle); //for MG996R on plant plate
    int moveToAngle(int desireAngle, int maxAngle); //function for TD8135
    void begin(); // command to start
    // void runStepper(); 
    void startPlantPattern();
    void LoadPattern();
    void testpattern();
    void update(); // call in loop()
    void stopAll(); // for Emergency Stop
};


