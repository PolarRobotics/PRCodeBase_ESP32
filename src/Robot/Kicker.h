#pragma once

#ifndef KICKER_H
#define KICKER_H

#include <MotorInterface.h>
#include <Robot/Robot.h>
#include <ps5Controller.h> // ESP PS5 library, access using global instance `ps5`

#define KICKER_COUNTS_PER_ENCODER_REV 11
#define KICKER_COUNTS_PER_ARM_REV 1188
#define KICKER_COUNTS_PER_ARM_DEGREE 3.3

/**
 * @brief Kicker header file
 * @authors Andrew Nelson
 */
class Kicker : public Robot {
private:
  bool enabled; // safety feature
  u_int16_t angleZero; // angle of the limit switch
  uint8_t kickerPin;
  uint8_t limitSwitchPin;
  static uint8_t kickerEncoderPinA;
  static uint8_t kickerEncoderPinB;
  static uint8_t kickerEncoderStateB;
  static int32_t currentKickerEncoderCount;
  MotorInterface windupMotor;

public:
  Kicker(
    uint8_t kickerPin, 
    u_int8_t limitSwitchPin, 
    uint8_t kickerEncoderPinA, 
    u_int8_t kickerEncoderPinB
  );
  void action() override; //! robot subclass must override action
  void enable();
  void test();
  void turnForward();
  void turnReverse();
  void stop();
  void homeKickingArm();
  void adjustAngle(int angle);
  static void kickerEncoderISR();
  uint16_t getCurrentAngle();
};

#endif // KICKER_H
