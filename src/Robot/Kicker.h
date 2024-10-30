#pragma once

#ifndef KICKER_H
#define KICKER_H

#include <MotorInterface.h>
#include <Robot/Robot.h>
#include <ps5Controller.h> // ESP PS5 library, access using global instance `ps5`

/**
 * @brief Kicker header file
 * @authors Andrew Nelson
 */
class Kicker : public Robot {
private:
  bool enabled; // safety feature
  uint8_t kickerPin;
  uint8_t limitSwitchPin;
  uint8_t kickerEncoderPin;
  MotorInterface windupMotor;

public:
  Kicker(uint8_t kickerPin, u_int8_t limitSwitchPin, uint8_t kickerEncoderPin);
  void action() override; //! robot subclass must override action
  void enable();
  void test();
  void turnForward();
  void turnReverse();
  void stop();
  void homeKickingArm();
  void adjustAngle(int angle);
  uint16_t getCurrentAngle();
};

#endif // KICKER_H
