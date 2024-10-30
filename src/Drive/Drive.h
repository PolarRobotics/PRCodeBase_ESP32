#pragma once

#include <Arduino.h>
#include <Robot/MotorControl.h>
#include "PolarRobotics.h"

#ifndef NUM_MOTORS
#define NUM_MOTORS 2
#endif // !NUM_MOTORS

// RAMP DEFINES
// rate of change of power with respect to time when accelerating in ramp
// #define ACCELERATION_RATE 0.00375f // [RPM/ms] possibly change to RPM/s for future
#define ACCELERATION_RATE    225 // [RPM^2] revolutions per minute squared
// #define ACCELERATION_RATE 0.004f // [RPM/ms] possibly change to RPM/s for future
#define RB_ACCELERATION_RATE 240 // [RPM^2] revolutions per minute squared

// rate of deceleration/braking
#define BRAKE_PERCENTAGE 0.9

// drive param generation
#define NORMAL_TURN_CONSTANT 0.05
// Value for the tank mode speed reduction percentage
#define TANK_MODE_PCT 0.75
// Value for the tank mode speed reduction percentage
#define RB_TANK_MODE_PCT 0.5
// Value for the Drift Mode Reduction Factor Percentage
#define DRIFT_MODE_PCT 0.8
//these should = normal speed, QB needs 0.5 for both 

// Controller Defines
#define STICK_DEADZONE 0.075 //0.0390625F // 8.0 / 127.0
#define THRESHOLD 0.00001

// the minimum power that can be written to the motor, prevents stalling
#define MOTOR_ZERO_OFFST 0.05f

// for the falcon motors to work correcly, a 100% motor power must be written to "learn" the maximum range
#define FALCON_CALIBRATION_FACTOR 1.0f

#define BRAKE_BUTTON_PCT 0

// !TODO: not sure if this is the correct location for this array
// This array must follow the same order as MotorType to be used effectively
constexpr float MOTORTYPE_BNS_ARRAY[NUM_MOTOR_TYPES][3] = {
// Boost   Normal  Slow
  {0.70f,  0.60f,  0.30f}, // index 0: Big Ampflow Motor
  {0.85f,  0.70f,  0.40f}, // index 1: Small Ampflow Motor
  {0.70f,  0.60f,  0.30f}, // index 2: Pancake Ampflow Motor
  {0.80f,  0.60f,  0.40f}, // index 3: Mecanum Motor (Torquenado)
  {0.70f,  0.40f,  0.25f}, // index 4: Falcon500 motors
  {0.15f,  0.10f,  0.05f}  // index 5: Small 12v motors (old robots)
};

class Drive {
  private:
    BotType botType;
    MotorType motorType; // TODO: Why is this private if we have a setter with no input validation? - MP 2023-05-10
    float gearRatio;
    bool hasEncoders;
    bool hasGyroscope;
    
    float speedScalar;
    float wheelBase;
    int omega;
    int omega_L, omega_R;
    float R, R_Max, R_Min;
    int max_RPM, min_RPM;
    // Turn sensitivity variables
    TurnFunction turnSensitivityType;
    float scaledSensitiveTurn = 0.0f;
    float turnSensitivityScalar = 0.0f;
    float domainAdjustment = 0.0f;

    void calcTurning(float stickTrn, float fwdLinPwr);

  protected:
    // MotorControl* M1;
    // MotorControl* M2;
    MotorControl M1, M2;
    float stickForwardRev, stickTurn;
    float lastTurnPwr;
    float turnPower;

    float requestedMotorPower[NUM_MOTORS];
    float lastRampPower[NUM_MOTORS];
    float turnMotorValues[NUM_MOTORS];

  public:
    enum Speed {
        BOOST,
        NORMAL,
        SLOW,
        BRAKE
    };

    Drive();
    Drive(BotType botType, MotorType motorType);
    Drive(BotType botType, drive_param_t driveParams);
    void setupMotors(uint8_t lpin, uint8_t rpin);
    void setupMotors(uint8_t lpin, uint8_t rpin, uint8_t left_enc_a_pin, uint8_t left_enc_b_pin, uint8_t right_enc_a_pin, uint8_t right_enc_b_pin);
    void setMotorType(MotorType motorType);
    void setStickPwr(int8_t leftY, int8_t rightX);
    float getForwardPower();
    float getTurnPower();
    void setSpeedScalar(Speed bns);
    void setSpeedValue(float speed_pct);
    float getSpeedScalar();
    void emergencyStop();
    void generateMotionValues(float tankModePct = TANK_MODE_PCT);
    virtual void update();
    void printSetup();
    virtual void printDebugInfo();
    virtual void printCsvInfo();
};