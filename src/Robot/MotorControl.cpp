#include <Arduino.h>

#include "MotorControl.h"

/**
 * @brief 
 * A class similar to the servo class, implements a method of choosing timers and channels 
 * based on the number of motors attached, uses writeMicros, etc. Utilizes the LedC built-in functions
 * 
 * since the sabretooth is expecting a duty cycle difference (time the signal is high for) 
 * I wrote a function to convert the time on value to a duty cycle, this would need to be moved into the Control class,
 * 
 * Goals/ Design specs: 
 *  - be able to declare multiple motor objects, with pin definitions.
 *  - avoid two motors on one pin and timers overlapping
 *  - write a value to the motor, maybe -100 100, we can play around with this
 *  - needs to be usable in drive and special bot classes
 * 
 * Example uses:
 * Definition: the motor object type can simply be defined by naming the motor object using the Motor type:
 *      Motor ExampleMotorObj;
 *   
 * and the pin can be attached by calling attach():
 *      ExampleMotorObj.attach();
 * 
 * to write to the motor simply pass a float between -1 and 1 into write()
 *      ExampleMotorObj.write(power);
 * 
 * where -1, 0 and 1 correspond to the throttles in the reverse, stop and forward directions
 * 
 * !TODO:
 * allow for us values outside the 1000us to 2000us, allow user to input the min and max values and pwr2Duty will account for that range
 * 
 * @author Rhys Davies 
 */
MotorControl::MotorControl() {  
  requestedRPM = 0;
  lastRampTime = millis();
}



/**
 * @brief setup the given pin to the next free channel, returns channel number or 255 if failure
 * @author Rhys Davies
 * Updated 2-26-2023
 * 
 * @param mot_pin the pin the motor is connected to
 * 
 * @return uint8_t the channel number the pin is attached to, 255 if failure
 */
uint8_t MotorControl::setup(int mot_pin, MotorType type, float gearRatio) {
  
  this->motor_type = type;
  this->gear_ratio = gearRatio;

  // Calculate the max rpm by multiplying the nominal motor RPM by the gear ratio
  this->max_rpm = int(MOTOR_MAX_RPM_ARR[static_cast<uint8_t>(this->motor_type)] * this->gear_ratio);

  // call the logic to attach the motor pin and setup, return 255 on an error
  return Motor.attach(mot_pin, MIN_PWM_US, MAX_PWM_US);
}

/**
 * @brief write, wrapper function for MotorInterface, to be removed in future
 * !TODO: remove in future
*/
void MotorControl::write(float pct) {
  Motor.write(pct);
}

void MotorControl::sendRPM(int rpm){
  if (rpm < 0)
    negativeDir = true;
  else 
    negativeDir = false;
  
  coeff = getMotorCurveCoeff(motor_type, negativeDir);

  pct = coeff.a*pow(rpm, coeff.b);

  Motor.write(pct);
}

int MotorControl::Percent2RPM(float pct)
{
  // float temp = constrain(pct, -1, 1);
  return this->max_rpm * constrain(pct, -1.0f, 1.0f);
}

float MotorControl::RPM2Percent(int rpm) {
  // int temp = constrain(rpm, -this->max_rpm, this->max_rpm);
  if (rpm == 0)
    return 0.0f; 
  return constrain(rpm, -this->max_rpm, this->max_rpm) / float(this->max_rpm);
}

/**
 * @brief ramp slowly increases the motor power each iteration of the main loop,
 * this function is critical in ensuring the bot has proper traction with the floor,
 * think of it as the slope y=mx+b
 *
 * FUTURE: none...this is perfection (...atm...:0 )
 *
 * @authors Grant Brautigam, Julia DeVore, Lena Frate
 * Created: fall 2023
 *
 * @param requestedPower, accelRate
 * @return int
 */
float MotorControl::ramp(float requestedPower,  float accelRate) {
    timeElapsed = millis() - lastRampTime;
    // Serial.print("  time elapsed: ");
    // Serial.print(timeElapsed);

    // Serial.print("  acceleration: ");
    // Serial.print(accelRate);

    // Serial.print("  requested power: ");
    // Serial.print(requestedPower);

    // Serial.print("  currentPower: ");
    // Serial.print(currentPower);

    // Serial.print("\n");

    lastRampTime = millis();
    if (requestedPower > requestedRPM) // need to speed up
    {
        requestedRPM = requestedRPM + accelRate * timeElapsed;
        if (requestedRPM > requestedPower) 
            requestedRPM = requestedPower; // to prevent you from speeding up past the requested speed
    }
    else // need to slow down
    {
        requestedRPM = requestedRPM - accelRate * timeElapsed; 
        if (requestedRPM < requestedPower) 
            requestedRPM = requestedPower; // to prevent you from slowing down below the requested speed
    }
    
    return requestedRPM;

}

