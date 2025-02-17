#pragma once

#ifndef QUARTERBACK_H
#define QUARTERBACK_H

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>
#include <ps5Controller.h> // ESP PS5 library, access using global instance `ps5`

// Flywheel defines 
#define FLYWHEEL_SPEED_FULL 0.4
#define FLYWHEEL_STOP_SPEED 0

#define FW_TIME_INCREMENT 25
#define FW_BRAKE_PERCENTAGE 0.9
#define FW_ACCEL_RATE .025


// Elevation (linear actuators) defines
#define SERVO_SPEED_UP 1
#define SERVO_SPEED_STOP 0
#define SERVO_SPEED_DOWN -1
#define MAX_ELEVATION 100
#define ELEVATION_PERIOD 3750

// Conveyor defines
#define CONVEYOR_ON 1
#define CONVEYOR_OFF 0

// Debounce Vars
#define DEBOUNCE_WAIT 250

#define NUM_SPEED_INCREMENTS 4

// Enum for whether to aim up or aim down
enum QBAim {
  AIM_UP, AIM_DOWN
};

enum QBElevation {
  LOW_ELEVATION,
  HIGH_ELEVATION
};

/**
 * @brief Quarterback Subclass Header
 * @authors Rhys Davies
 */
class Quarterback : public Robot {
  private: 
    uint8_t flywheelPin;
    uint8_t conveyorPin;
    uint8_t elevationPin;
    int arrayPos = 0;
    MotorControl flywheelMotor;
    MotorControl conveyorMotor;
    MotorControl elevationMotors;
    bool flywheelsOn, conveyorOn;
    bool aimingUp, aimingDown;
    bool raise, lower;
    bool setupMotors;
    uint8_t currentElevation, targetElevation;
    unsigned long lastElevationTime = 0;
    unsigned long lastFlywheelToggleTime = 0;
    unsigned long lastFlywheelRampTime = 0;
    float currentFlywheelPower = 0;
    float flywheelSpeedFactor = 0;
    unsigned long lastDBElev = 0, lastDBFW = 0, lastDBFWChange = 0, lastDBConv = 0; // DB is for debounce
    const float speedFac[NUM_SPEED_INCREMENTS] = { 0.0, 0.45, 0.75}; // the power levels are truly 0.3, 0.75, 1 because its FLYWHEEL_SPEED_FULL + value in array

  public:
    Quarterback(
      uint8_t flywheelPin, 
      uint8_t conveyorPin, 
      uint8_t elevationPin
    );
    void setup();
    void action() override; //! robot subclass must override action
    void toggleFlywheels();
    float rampFW(float requestedpwr);
    void aim(QBAim dir);
    void toggleConveyor();
    void changeFWSpeed(SpeedStatus speed);
    void update();
};

#endif // QUARTERBACK_H
