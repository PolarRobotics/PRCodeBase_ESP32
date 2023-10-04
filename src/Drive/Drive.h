#pragma once

#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include "PolarRobotics.h"
#include "Robot/MotorControl.h"


#ifndef NUM_MOTORS
#define NUM_MOTORS 2
#endif // !NUM_MOTORS

// RAMP DEFINES
#ifndef ACCELERATION_RATE
// rate of change of power with respect to time when accelerating %power/10th of sec
// #define ACCELERATION_RATE .0375f // probably lower for runningback
#define ACCELERATION_RATE 0.0375f // probably lower for runningback
#endif // !ACCELERATION_RATE
// rate of deceleration/braking
#define BRAKE_PERCENTAGE 0.9
// how often the ramp() function changes the motor power
#define TIME_INCREMENT 5


// drive param generation
#define NORMAL_TURN_CONSTANT 0.05
// Value for the tank mode speed reduction percentage
#define TANK_MODE_PCT 0.75
// Value for the Drift Mode Reduction Factor Percentage
#define DRIFT_MODE_PCT 0.8
//these should = normal speed, QB needs 0.5 for both 


// #if BOT_TYPE == 1 //rx
// #define turnMax 0.8 
// #define turnMin 0.8 
// #elif BOT_TYPE == 2 //old center
// #define turnMax 0.2 
// #define turnMin 0.2 
// #elif BOT_TYPE == 4 // qb
// #define turnMax 0.4
// #define turnMin 0.4
// #elif BOT_TYPE == 6 //runningback
// #define turnMax 0.5
// #define turnMin 0.2
// #else
// #define turnMax 0.65 // the max allowable turning when the bot is traveling at lowest speed
// #define turnMin 0.65 // the min allowable turning when the bot is traveling at full speed
// #endif

// Controller Defines
#define STICK_DEADZONE 0.0390625F // 8.0 / 127.0
#define THRESHOLD 0.00001


// MOTOR MAX SPEED DEFINES;
// this is 1.0, the maximum power possible to the motors.
// #if BOT_TYPE == 4
// #define BIG_BOOST_PCT 0.7  // default: 0.6, this is the typical percentage of power out of the motors' range that is used (to ensure they don't do seven wheelies)
// #define BIG_NORMAL_PCT 0.3 // should be a value less than BIG_NORMAL_PCT, to slow down for precision maneuvering, QB needs this to be 0.3
// #define BIG_SLOW_PCT 0.2
// #else
// #define BIG_BOOST_PCT 0.7  // default: 0.6, this is the typical percentage of power out of the motors' range that is used (to ensure they don't do seven wheelies)
// #define BIG_NORMAL_PCT 0.6 // should be a value less than BIG_NORMAL_PCT, to slow down for precision maneuvering, QB needs this to be 0.3
// #define BIG_SLOW_PCT 0.3   // the value for brake button to slow down the motors at the button press
// #endif

// BSN for Short/Small Motors
#define SMALL_BOOST_PCT 0.85
#define SMALL_NORMAL_PCT 0.7
#define SMALL_SLOW_PCT 0.4

// BSN for the 12v motors used on the new center
#define MECANUM_BOOST_PCT  0.8
#define MECANUM_NORMAL_PCT 0.6
#define MECANUM_SLOW_PCT   0.3

// BSN for the falcon motors used on the runningback
#define FALCON_BOOST_PCT  1.0
#define FALCON_NORMAL_PCT 0.5
#define FALCON_SLOW_PCT   0.1

#define BRAKE_BUTTON_PCT 0

class Drive {
  private:
    MotorType motorType; // TODO: Why is this private if we have a setter with no input validation? - MP 2023-05-10
    BotType botType; // TODO: I added this to private only because motorType was private.
    float BSNscalar;
    float requestedMotorPower[NUM_MOTORS];
    float currentRampPower[NUM_MOTORS];
    float lastRampPower[NUM_MOTORS];
    float turnMotorValues[NUM_MOTORS];
    void calcTurningMotorValues(float stickTrn, float prevPwr, int dir);
    float wheelBase = 9.75;
    float Omega_r = 0;
    float Omega_rL = 0;
    float Omega_rR = 0;
    float R = 0;
    int R_Max = 24;
    int R_High_Min = 48;
    float R_Min = wheelBase/2;
    float max_RPM = 4000;
    float angularPower = 0.0f;
    float intermediateMotorPower[NUM_MOTORS];
    float maxMagnitude = 0.0f;


  protected:
    MotorControl M1, M2;
    float stickForwardRev, stickTurn;
    float lastTurnPwr;
    float turnPower;
    unsigned long lastRampTime[NUM_MOTORS];

  public:
    enum Speed {
        NORMAL,
        BOOST,
        SLOW,
        BRAKE
    };

    Drive();
    Drive(BotType botType, MotorType motorType);
    void setServos(uint8_t lpin, uint8_t rpin);
    void setMotorType(MotorType motorType);
    void setStickPwr(int8_t leftY, int8_t rightX);
    float getForwardPower();
    float getTurnPower();
    void setBSN(Speed bsn); //(float powerMultiplier);
    float getBSN();
    float getReqMotorPwr(uint8_t mtr);
    void setReqMotorPwr(float power, uint8_t mtr);
    void setLastRampPwr(float power, uint8_t mtr);
    void emergencyStop();
    float ramp(float requestedPower, uint8_t mtr, float accelRate = ACCELERATION_RATE);
    void generateMotionValues();
    virtual void update();
    virtual void printDebugInfo();
    float applyClamp(float value);
    float applyDeadzone(float value, float deadzone);
    void diffDriveCurve(float stickForwardRev, float stickTurn);

    //* The following variables are initialized in the constructor
    // the max allowable turning when the bot is traveling at lowest speed
    float turnMax; 

    // the min allowable turning when the bot is traveling at full speed
    float turnMin; 
    
    // maximum speed for these is 1.0
    // percentage of power used when boosting for big motors
    float BIG_BOOST_PCT;
    
    // default: 0.6, this is the typical percentage of power out of the motors' range that is used (to ensure they don't do seven wheelies)
    float BIG_NORMAL_PCT;
    
    // should be a value less than BIG_NORMAL_PCT, to slow down for precision maneuvering, QB needs this to be 0.3
    float BIG_SLOW_PCT;
};

#endif // DRIVE_H