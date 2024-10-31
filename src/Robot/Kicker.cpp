#include "Kicker.h"

/**
 * @brief Kicker v2 Special Motor Control
 * @authors Andrew Nelson, Corbin Hibler
 * 
 * Controls the special motor that winds the kicker arm automatically upon the robot start.
 * 
 * Intended behavior is for the robot to turn on, wind the arm until it hits the limit switch, and
 * then fall back a certain number of degrees (~15 degrees for now).
 * 
 * @param kickerPin The pin of the kicker arm's motor
 * @param limitswitchPin the pin of the kicker arm's limit switch
 * @param kickerEncoderPinA The pin of signal A from the encoder
 * @param kickerEncoderPinB THe pin of signal B from the encoder
 */
Kicker::Kicker(uint8_t kickerPin, uint8_t limitSwitchPin, uint8_t kickerEncoderPinA, uint8_t kickerEncoderPinB) {
  enabled = false;
  this->kickerPin = kickerPin;
  this->limitSwitchPin = limitSwitchPin;
  windupMotor.attach(kickerPin);
  
  // Encoder Setup
  this->kickerEncoderPinA = kickerEncoderPinA;
  this->kickerEncoderPinB = kickerEncoderPinB;
  this->currentKickerEncoderCount = 0;
  pinMode(kickerEncoderPinA, INPUT_PULLUP);
  pinMode(kickerEncoderPinB, INPUT);
  attachInterrupt(kickerEncoderPinA, kickerEncoderISR, RISING);
}


// "Define" static members to satisfy linker
uint8_t Kicker::kickerEncoderPinA;
uint8_t Kicker::kickerEncoderPinB;
uint8_t Kicker::kickerEncoderStateB;
int32_t Kicker::currentKickerEncoderCount;

void Kicker::kickerEncoderISR() {
  kickerEncoderStateB = digitalRead(kickerEncoderPinB);

  if (kickerEncoderStateB == 1) {
    currentKickerEncoderCount++;
  } else if (kickerEncoderStateB == 0) {
    currentKickerEncoderCount--;
  }
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
 * @author Corbin Hibler
 * 
 * This function will run when the kicker starts. It will automatically wind the kicker arm
 * to a certain level until it hits the limit switch. Then it will automatically adjust to a certain
 * degree from the zero point (where the limit switch is).
 * 
 */
void Kicker::homeKickingArm() {
  if(enabled) {
    if (digitalRead(limitSwitchPin) == 1) {
      stop();
      angleZero = getCurrentAngle();
      adjustAngle(15);
    } else {
      windupMotor.write(0.5);
    }
  }
}

/**
 * @brief Reverse Motor a Specified Number of Degrees
 * @author Corbin Hibler
 *  
 * Adjusts the angle of the kicker arm from where it currently is back to a number of degrees,
 * in the counter-clockwise direction.
 *  
 *        <- 90
 *         ___
 *    |   /   \  ^
 *    v   | O |  | 0
 *   180  \___/
 *          
 *       -> 270
 * 
 * @param angle The angle that you want to add to the current angle of the kicker arm.
 */
void Kicker::adjustAngle(int angle) {
  if (enabled) {
    uint16_t desiredAngle = angleZero + angle;
    windupMotor.write(-0.5);

    // Keep rotating until desiredAngle is reached
    if (getCurrentAngle() >= desiredAngle) {
      stop();
    }
  }
}


/**
 * @brief Gets Current Angle of the Motor from Encoder
 * @author Corbin Hibler
 * 
 * The gear ratio of the motor to the shaft is 1:108
 * The encoder resolution is 11 counts per revolution of the motor
 * Therefore the encoder resolution of the shaft is 11*108 = 1188 counts per revolution
 * 1188 / 360 = 3.3 counts per degree 
 * 
 * 
 * @return The current angle that the motor is at
 */
uint16_t Kicker::getCurrentAngle() {
  float valueCHA = digitalRead(kickerEncoderPinA);
  float valueCHB = digitalRead(kickerEncoderPinB);

  return -1;
}