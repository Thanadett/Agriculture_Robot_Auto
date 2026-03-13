#pragma once

#include <ESP32Servo.h>
#include <AccelStepper.h>
#include "config.h"

enum TaskMode { IDLE, PLANTING, LOADING, TESTING, MOVECAM_UP, MOVECAM_DOWN, RESET };
class PlantingManager {
private:
    TaskMode _activeMode = IDLE;
    Servo servo_gripper, servo_linear, servo_plate, servo_cam;
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
    
    const float degree = 19.3;
    float HomeAngle = 16.3; // plate degrees
    float _targetAngle = 0;
    float _servo_plate_CurrentAngle = degree; // เก็บองศาปัจจุบันของ Servo 
    const float MAX_PLATE_ANGLE = 180.0f; 
    unsigned long _lastServoUpdate = 0; // เวลาที่อัปเดตองศาล่าสุด

    bool _limitTriggered = false;    // limit switch โดน trigger ไหม

    int desireAngle = 0; // องศาที่ต้องการ (0-270)
    // Helper for TB6600 (Active-Low)
    void ena_enable();
    void ena_disable();

public:
    PlantingManager() : _stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR) {}
    
    int getCurrentStep() { return _currentStep; }
    bool isBusy() { return _activeMode != IDLE; }
    bool isIDLE() { return _activeMode == IDLE; }


    long mmToSteps(float StepPermm);
    int angleToUs(float Angle); //for MG996R on plant plate
    int moveToAngle(int desireAngle, int maxAngle); //function for TD8135
    void begin(); // command to start
    // void runStepper(); 
    void startPlantPattern();
    void LoadPattern();
    void testpattern();
    void mv_cam_up_pattern();
    void mv_cam_down_pattern();
    void resetPattern();
    void update(); // call in loop()
    void stopAll(); // for Emergency Stop

    bool isLimitTriggered() { return _limitTriggered; }
    bool checkLimitSwitch();   // อ่านค่า switch (LOW = triggered เพราะ INPUT_PULLUP + NO)
    void startHoming();        // เริ่ม homing sequence
};


