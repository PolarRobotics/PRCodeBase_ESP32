#include "Kicker.h"

/**
 * @brief Kicker v2 Special Motor Control
 * 
 * Controls the special motor that winds the kicker arm automatically upon the robot start.
 * 
 * Intended behavior is for the robot to turn on, wind the arm until it hits the limit switch, and
 * then fall back a certain number of degrees (~15 degrees for now).
 */
Kicker::Kicker(uint8_t kickerPin) {
  enabled = false;
  this->kickerPin = kickerPin;
  windupMotor.attach(kickerPin);
}

/**
 * @brief Manual Action
 * 
 * Manually control the motor using the Triangle and X (cross) buttons.
 * Triangle for winding
 * Cross for unwinding
 */
void Kicker::action() {
  // Control the motor on the kicker
  if (ps5.Triangle())
    turnForward();
  else if (ps5.Cross())
    turnReverse();
  else
    stop();
}

/**
 * @brief Enable Function
 * 
 * Enables the kicker for safety purposes.
 */
void Kicker::enable() {
  enabled = true;
}

/**
 * @brief Turns motor forward
 * 
 * Turns the motor forward by writing the windupMotor SPECBOT_1 (D18) pin to -0.5
 */
void Kicker::turnForward() {
  if (enabled) {
    windupMotor.write(-0.5);
  }
}

/**
 * @brief Turns motor reverse
 * 
 * Turns the motor backwards by writing the windupMotor SPECBOT_1 (D18) pin to 0.5
 */
void Kicker::turnReverse() {
  if (enabled) {
    windupMotor.write(0.5);
  }
}

/**
 * @brief Stop Motor
 * 
 * Stops the motor by writing the windupMotor SPECBOT_1 (D18) pin to 0
 */
void Kicker::stop() {
  if (enabled) {
    windupMotor.write(0);
  }
}

/**
 * @brief Automatically Wind on Startup
 * 
 * This function will run when the kicker starts. It will automatically wind the kicker arm
 * to a certain level until it hits the limit switch. Then it will automatically adjust to a certain
 * degree from the zero point (where the limit switch is).
 * 
 * @author Corbin Hibler
 */
