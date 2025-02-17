#include <Arduino.h>
#include "Drive/Drive.h"
#include "Robot/MotorControl.h"

/**
 * @brief Drive Class, base class for specialized drive classes, this configuration is intended for the standard linemen.
 * this class takes the stick input, scales the turning value for each motor and ramps that value over time,
 * then sets the ramped value to the motors
 * @authors Rhys Davies (@rdavies02), Max Phillips (@RyzenFromFire)
 *
 * @class
 *    2 motor configuration shown below
 *
 *               ^
 *               | Fwd
 *       _________________ 
 *      |        _        |
 *      |       |O|       |       O: represents the Omniwheel, a wheel that can turn on 2 axis
 *      |       |_|       |       L: represents the left Wheel, powered by the left motor via a chain
 *      |  _           _  |            - the left motor would turn ccw to move the bot forward
 *      | |L|         |R| |       R: represents the right Wheel, powered by the right motor via a chain
 *      | |_|         |_| |            - the right motor would turn cw to move the bot forward
 *      |_________________|
 *
 * @todo
 *  - add a turning radius parameter, needed for the kicker
 *  - add mechanium driving code, for the new center, needed next semester (Spring 2023)
 *
 * Default configuration:
 * @param leftmotorpin the arduino pin needed for the left motor, needed for servo
 * @param rightmotorpin the arduino pin needed for the right motor, needed for servo
*/

Drive::Drive() {
  Drive(lineman, {big_ampflow, 1, 9, 6, 36});
}

Drive::Drive(BotType botType, MotorType motorType) {
  Drive(botType, {motorType, 1, 9, 6, 36});
}

Drive::Drive(BotType botType, drive_param_t driveParams, bool hasEncoders, int turnFunction) {
  this->botType = botType;
  this->hasEncoders = hasEncoders;
  this->motorType = driveParams.motor_type;
  this->gearRatio = driveParams.gear_ratio;
  this->wheelBase = driveParams.wheel_base;
  this->R_Min = driveParams.r_min;
  this->R_Max = driveParams.r_max;

  if (botType == quarterback_old) {
    this->BIG_BOOST_PCT = 0.8; 
    this->BIG_NORMAL_PCT = 0.4; 
    this->BIG_SLOW_PCT = 0.3;
  } else if (botType == center) {
    this->BIG_BOOST_PCT = 0.5;  
    this->BIG_NORMAL_PCT = 0.4; 
    this->BIG_SLOW_PCT = 0.25;
  } else {
    this->BIG_BOOST_PCT = 0.7;  
    this->BIG_NORMAL_PCT = 0.6; 
    this->BIG_SLOW_PCT = 0.3;
  }

  // initialize arrays
  for (int i = 0; i < NUM_MOTORS; i++) {
    requestedMotorPower[i] = 0.0f;
    lastRampPower[i] = 0.0f;
    turnMotorValues[i] = 0.0f;
  }

  if (botType != mecanum_center) {
    // initialize parameters for turning model
    omega = 0;
    omega_L = 0, omega_R = 0;
    R = 0.0f;
    // R_Max = 24.0f;
    // R_Max = 36.0f;
    // R_Min = wheelBase/2 + 4;
    min_RPM = 200;
    // max_RPM = M1.Percent2RPM(1);
    // max_RPM = M1.max_rpm;

    // initialize turn sensitivity variables
    enableTurnSensitivity = turnFunction; // 0 for linear, 1 for Rhys's function, 2 for cubic
    turnSensitivityScalar = 0.49; // Range: (0, 0.5) really [0.01, 0.49]
    domainAdjustment = 1/log((1-(turnSensitivityScalar + 0.5))/(turnSensitivityScalar + 0.5));
    
  } 

}

void Drive::setupMotors(uint8_t lpin, uint8_t rpin) {
    //this->motorPins[0] = lpin, this->motorPins[1] = rpin;
    // this->M1 = new MotorControl(motorType, false, this->gearRatio);
    // this->M2 = new MotorControl(motorType, false, this->gearRatio);

    // M1->setup(lpin), M2->setup(rpin);
    M1.setup(lpin, this->motorType, this->hasEncoders, this->gearRatio);
    M2.setup(rpin, this->motorType, this->hasEncoders, this->gearRatio);
}

/**
 * setupMotors
 * @brief to be called when setting up a motor with an encoder
 * 
 * 
*/
void Drive::setupMotors(uint8_t lpin, uint8_t rpin, uint8_t left_enc_a_pin, uint8_t left_enc_b_pin, uint8_t right_enc_a_pin, uint8_t right_enc_b_pin) {
    //this->motorPins[0] = lpin, this->motorPins[1] = rpin;
    // this->M1 = new MotorControl(motorType, true, this->gearRatio);
    // this->M2 = new MotorControl(motorType, true, this->gearRatio);
    
    M1.setup(lpin, this->motorType, this->hasEncoders, this->gearRatio, left_enc_a_pin, left_enc_b_pin);
    M2.setup(rpin, this->motorType, this->hasEncoders, this->gearRatio, right_enc_a_pin, right_enc_b_pin);
}

void Drive::setMotorType(MotorType motorType) {
    this->motorType = motorType;
}


/**
 * setStickPwr takes the stick values passed in and normalizes them to values between -1 and 1
 * and sets this value to the private variables stickFwdRev and stickTurn respectively
 * @author Rhys Davies
 * Created: 9-12-2022
 *
 * @param leftY the forward backward value from the left stick an unsigned 8-bit float (0 to 255)
 * @param rightX the left right value from the right stick an unsigned 8-bit float (0 to 255)
*/
void Drive::setStickPwr(int8_t leftY, int8_t rightX) {
    // left stick all the way forward is 0, backward is 255
    // +: forward, -: backward. needs to be negated so that forward is forward and v.v.; subtracting 1 bumps into correct range
    stickForwardRev = (leftY / 127.5f);
    stickTurn = (rightX / 127.5f);

    // stick deadzones
    // set to zero (no input) if within the set deadzone
    // subtacting STICK_DEADZONE and deviding by 1-STICK_DEADZONE normalize the inputs to use the full 0-1 range
    if (fabs(stickForwardRev) < STICK_DEADZONE)
      stickForwardRev = 0;
    else if (stickForwardRev > 0)
      stickForwardRev = (stickForwardRev - STICK_DEADZONE) / (1 - STICK_DEADZONE);
    else if (stickForwardRev < 0)
      stickForwardRev = (stickForwardRev + STICK_DEADZONE) / (1 - STICK_DEADZONE);
    
    if (fabs(stickTurn) < STICK_DEADZONE)
      stickTurn = 0;
    else if (stickTurn > 0)
      stickTurn = (stickTurn - STICK_DEADZONE) / (1 - STICK_DEADZONE);
    else if (stickTurn < 0)
      stickTurn = (stickTurn + STICK_DEADZONE) / (1 - STICK_DEADZONE);

}

float Drive::getForwardPower() {
    return stickForwardRev;
}

float Drive::getTurnPower() {
    return stickTurn;
}

/**
 * @brief setSpeedScalar sets the internal variable to the requested percent power, this is what the motor power gets multiplied by,
 * this is where the boost, normal and slow scalars get passed in
 * @author Rhys Davies
 * Created: 9-12-2022
 *
 * @param bns input speed choice Drive::Boost, Drive::Normal, Drive::Slow
*/
void Drive::setSpeedScalar(Speed bns) {
    // set the scalar to zero if the requested value is greater than 1, this is not entirely necessary, but is a safety
    if (bns == Speed::BRAKE) 
        speedScalar = BRAKE_BUTTON_PCT;
    else
        // Grab the predefined bns values from the 
        speedScalar = MOTORTYPE_BNS_ARRAY[static_cast<uint8_t>(motorType)][static_cast<uint8_t>(bns)];
}

/**
 * @brief setSpeedValue overrides the default predefined values from MOTORTYPE_BNS_ARRAY
*/
void Drive::setSpeedValue(float speed_pct) {
    this->speedScalar = constrain(speed_pct, -1, 1);
}

float Drive::getSpeedScalar() {
    return this->speedScalar;
}

/**
 * generateTurnScalar takes the input stick power and scales the max turning power allowed with the forward power input
 * @authors Grant Brautigam, Rhys Davies, Max Phillips
 * Created: 9-12-2022
*/
void Drive::generateMotionValues(float tankModePct) {
    if (fabs(stickForwardRev) < STICK_DEADZONE) { // fwd stick is zero
        if (fabs(stickTurn) < STICK_DEADZONE) { // turn stick is zero
            requestedMotorPower[0] = 0, requestedMotorPower[1] = 0; // not moving, set motors to zero
        } else if (stickTurn > STICK_DEADZONE) { // turning right, but not moving forward much so use tank mode
            requestedMotorPower[0] = speedScalar * abs(stickTurn)  * tankModePct;
            requestedMotorPower[1] = -speedScalar * abs(stickTurn) * tankModePct;
        } else if (stickTurn < -STICK_DEADZONE) { // turning left, but not moving forward muchso use tank mode
            requestedMotorPower[0] = -speedScalar * abs(stickTurn) * tankModePct;
            requestedMotorPower[1] = speedScalar * abs(stickTurn)  * tankModePct;
        } // no general else since encountered infinite loop
    } else { // fwd stick is not zero
        if (fabs(stickTurn) < STICK_DEADZONE) { // turn stick is zero
            // just move forward directly
            requestedMotorPower[0] = speedScalar * stickForwardRev;
            requestedMotorPower[1] = speedScalar * stickForwardRev;
        } else { // moving forward and turning
            /*
            if the sticks are not in any of the edge cases tested for above (when both sticks are not 0),
            a value must be calculated to determine how to scale the motor that is doing the turning.
            i.e.: if the user moves the left stick all the way forward (stickFwdRev = 1), and they are attempting
            to turn right. The left motor should get set to 1 and the right motor should get set to
            some value less than 1, this value is determined by the function calcTurningMotorValue
            */
            if(stickTurn > STICK_DEADZONE) { // turn Right
                // switch(abs((speedScalar * stickForwardRev)) > abs(lastRampPower[0])) {
                //     case true: calcTurning(stickTurn, abs(lastRampPower[0])); break;
                //     case false: calcTurning(stickTurn, abs(speedScalar * stickForwardRev)); break;
                // }
                calcTurning(abs(stickTurn), abs(speedScalar * stickForwardRev));

                requestedMotorPower[0] = copysign(turnMotorValues[0], stickForwardRev);
                requestedMotorPower[1] = copysign(turnMotorValues[1], stickForwardRev);
            } else if(stickTurn < -STICK_DEADZONE) { // turn Left
                // switch(abs((speedScalar * stickForwardRev)) > abs(lastRampPower[1])) {
                //     case true: calcTurning(stickTurn, abs(lastRampPower[1])); break;
                //     case false: calcTurning(stickTurn, abs(speedScalar * stickForwardRev)); break;
                // }

                calcTurning(abs(stickTurn), abs(speedScalar * stickForwardRev));
                
                requestedMotorPower[0] = copysign(turnMotorValues[1], stickForwardRev);
                requestedMotorPower[1] = copysign(turnMotorValues[0], stickForwardRev);
            }
        }
    }
}


/**
 * @brief calcTurning generates power values for each motor to achieve a given turn
 * @authors Grant Brautigam, Rhys Davies
 * Created: 9-12-2022
 * updated: 10-6-2023
 * Mathematical model:
 * Whitepaper: https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
 * 
 * Add link to Desmos in future
 * 
 * Differential drive turning model
 * 
 * omega_R = (omega/R)*(R + l/2)
 * omega_L = (omega/R)*(R - l/2)
 *
 * omega_R: right wheel angular velocity
 * omega_R: left wheel angular velocity
 * omega: the rate of rotation around the ICC (Instantaneous Center of Curvature)
 * l: distance between each wheel (wheelbase) 
 * R: distance the ICC from the center of the wheelbase (l/2)
 * 
 * @param stickTrn the absoulte value of the current turning stick input
 * @param fwdLinPwr the non-turning motor value from the previous loop, which was actually sent to the motor
 */
void Drive::calcTurning(float stickTrn, float fwdLinPwr) {
    //R_Min = R_Min + abs(stickForwardRev)*(R_High_Min - R_Min); // start of turn scaling 
    if (enableTurnSensitivity == 0) // linear
        scaledSensitiveTurn = stickTrn;
    else if(enableTurnSensitivity == 1) // rhys's function
        scaledSensitiveTurn = log((1-(turnSensitivityScalar * stickTrn + 0.5))/(turnSensitivityScalar * stickTrn + 0.5)) * domainAdjustment;
    else // cubic
        scaledSensitiveTurn = pow(stickTrn, 3);
        
    // Calculate the R value from the stick turn input
    R = (1-scaledSensitiveTurn)*(R_Max-R_Min) + R_Min;
    
    // calculate the requested angular velocity for the robot 
    omega = abs(M1.Percent2RPM(fwdLinPwr));

    // calculate the rpm for the left wheel
    omega_L = (omega/R)*(R+(wheelBase/2));
    // calculate the rpm for the right wheel
    omega_R = (omega/R)*(R-(wheelBase/2));

    // ensure the left wheel RPM doesnt go below the min or above the max RPM
    omega_L = constrain(omega_L, min_RPM, M1.max_rpm);
    // ensure the left wheel RPM doesnt go below the min or above the max RPM
    omega_R = constrain(omega_R, min_RPM, M1.max_rpm);

    turnMotorValues[0] = M1.RPM2Percent(omega_L);
    turnMotorValues[1] = M2.RPM2Percent(omega_R);
}

void Drive::emergencyStop() {
    // M1->writelow(), M2->writelow();
    // M1.writelow(), M2.writelow();

    M1.write(0); 
    M2.write(0);
}

void Drive::printSetup() {
    Serial.print(F("\nDrive::printSetup():"));
    Serial.print(F("\nMotorType: "));
    Serial.print(getMotorTypeString(this->motorType));
    Serial.print(F("\nGearRatio: "));
    Serial.print(this->gearRatio);
    Serial.print(F("\nR_Min: "));
    Serial.print(this->R_Min);
    Serial.print(F("\nR_Max: "));
    Serial.print(this->R_Max);
    Serial.print(F("\nMin RPM: "));
    Serial.print(this->min_RPM);
    Serial.print(F("\nMAX RPM: "));
    Serial.print(M1.max_rpm);
    Serial.print(F("\nTurnSensitivityMode: "));
    Serial.print(enableTurnSensitivity);
    Serial.print(F("\nEncoders: "));
    Serial.print(F("\nHas Encoders? "));
    Serial.print(this->hasEncoders ? F("True") : F("False"));

    Serial.print(F("\n"));
}

/**
 * prints the internal variables to the serial monitor in a clean format,
 * this function exists out of pure laziness to not have to comment out all the print statments
 * @author
 * Updated:
*/
void Drive::printDebugInfo() {
    Serial.print(F("L_Hat_Y: "));
    Serial.print(stickForwardRev);
    Serial.print(F("  R_HAT_X: "));
    Serial.print(stickTurn);

    // Serial.print(F("  |  Turn: "));
    // Serial.print(lastTurnPwr);

    // Serial.print(F("  |  Left ReqPwr: "));
    // Serial.print(requestedMotorPower[0]);
    // Serial.print(F("  Right ReqPwr: "));
    // Serial.print(requestedMotorPower[1]);

    Serial.print(F("  |  Omega: "));
    Serial.print(omega);

    Serial.print(F("  omega_L: "));
    Serial.print(omega_L);
    Serial.print(F("  omega_R: "));
    Serial.print(omega_R);

    // Serial.print(F("  lastRampTime "));
    // Serial.print(lastRampTime[0]);
    // Serial.print(F("  requestedPower "));
    // Serial.print(requestedPower);
    // Serial.print(F("  current "));
    // Serial.print(currentRampPower[0]);
    // Serial.print(F("  requestedPower - currentRampPower "));
    // Serial.println(requestedPower - currentRampPower[mtr], 10);

    Serial.print(F("  Left Motor: "));
    Serial.print(requestedMotorPower[0]);
    Serial.print(F("  Right: "));
    Serial.print(requestedMotorPower[1]);

    //Serial.print(F("  scaledSensitiveTurn: "));
    //Serial.print(scaledSensitiveTurn);

    Serial.print(F("\n"));
}
/**
 * @brief Prints variables to the serial monitor in a csv format
 * This function is important for data acquisition
 * The options below are configurable, change them as you need
 * Remember to adhere to printing guidelines under PR-Docs
 * @author Corbin Hibler
 * Updated: 2023-10-30
*/
void Drive::printCsvInfo() {
    Serial.print(F("header1,")); // name of value to be used as header
    Serial.print(1);             // variable you want to track
    Serial.print(F(",header2,")); 
    Serial.print(2);
    Serial.print(F(",header3,"));
    Serial.print(3);
    Serial.print(F(",header4,"));
    Serial.print(4);
    Serial.print(F(",header5,"));
    Serial.println(5); // last line is -ALWAYS- println or else the python script will break
}
/**
 * @brief updates the motors after calling all the functions to generate
 * turning and scaling motor values, the intention of this is so the
 * programmer doesnt have to call all the functions, this just handles it,
 * reducing clutter in the main file.
 * DO NOT CALL THIS FUNCTION UNTIL setStickPwr and setSpeedScalar have been called before update
 * @author Rhys Davies
 * Created: 9-12-2022
*/
void Drive::update() {
    // !TODO Clean up when robots are rewired:
    if (this->botType == runningback) {
        // Generate turning motion
        generateMotionValues();
        //printDebugInfo();

        // calculate the value to set to the motors to based on the acceleration rate
        requestedMotorPower[0] = M1.ramp(requestedMotorPower[0], RB_ACCELERATION_RATE);
        requestedMotorPower[1] = M2.ramp(requestedMotorPower[1], RB_ACCELERATION_RATE);

        // Set the ramp value to a function, needed for generateMotionValues
        lastRampPower[0] = requestedMotorPower[0];
        lastRampPower[1] = requestedMotorPower[1];

        // TODO: move to MotorController
        // Make sure the min values written to the motor are not zero, gives the motor enough to break the deadband
        requestedMotorPower[0] = fabs(requestedMotorPower[0]) < MOTOR_ZERO_OFFST ? 0 : requestedMotorPower[0];
        requestedMotorPower[1] = fabs(requestedMotorPower[1]) < MOTOR_ZERO_OFFST ? 0 : requestedMotorPower[1];

        // Write the ramped value to the motor via MotorInterface
        M1.write(requestedMotorPower[0]);
        M2.write(-requestedMotorPower[1]);
    }
    else { // CASE FOR ANY OTHER ROBOT
        // Generate turning motion
        generateMotionValues(RB_TANK_MODE_PCT);
        //printDebugInfo();

        // calculate the value to set to the motors to based on the acceleration rate
        requestedMotorPower[0] = M1.ramp(requestedMotorPower[0], ACCELERATION_RATE);
        requestedMotorPower[1] = M2.ramp(requestedMotorPower[1], ACCELERATION_RATE);

        // Set the ramp value to a function, needed for generateMotionValues
        lastRampPower[0] = requestedMotorPower[0];
        lastRampPower[1] = requestedMotorPower[1];
        
        // Write the ramped value to the motor via MotorInterface
        M1.write(requestedMotorPower[0]);
        M2.write(requestedMotorPower[1]);
    }
    
    trackingMotorPower[0] = requestedMotorPower[0];
    trackingMotorPower[1] = requestedMotorPower[1];
}

int Drive::getMotorWifiValue(int motorRequested) {
    int valueToReturn = 0;
    if (motorRequested >= 0 && motorRequested < NUM_MOTORS) {
        float value = trackingMotorPower[motorRequested] * 100;
        valueToReturn = value;
    }
    return valueToReturn;
}

