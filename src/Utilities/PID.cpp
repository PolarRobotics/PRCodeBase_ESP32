#include <Arduino.h>
#include "Utilities/PID.h"
#include "PID.h"

// Public methods
PID::PID(float k_p, float k_i, float k_d, float error_thresh,
  float error_min, float error_max)
{
  this->k_p = k_p;
  this->k_i = k_i;
  this->k_d = k_d;
  this->error_threshold = error_thresh;

  if (error_min > error_max) {
    Serial.println(F("Utilities/PID.cpp: Error min larger than max value..."));
    error_minmax[0] = error_max;
    error_minmax[1] = error_min;
  } else {
    error_minmax[0] = error_min;
    error_minmax[1] = error_max;
  }
}

void PID::setCLState(bool state) {
  this->cl_enable = state;
}

bool PID::setProcessVar(int measured_value) {
  this->process_variable = measured_value;
}

bool PID::setSetpoint(float commanded_value) {

}

/**
 * PILoop is the closed loop controller. this is the main function for CL
 * 
 * @authors Grant Brautigam, Rhys Davies
 * Created: 11-19-2023
 * Updated: 2-5-2025
 */
float PID::PILoop() {
  if (abs(process_variable) >= error_threshold) { // the motor wants to stop, skip and reset the PI loop  
    error = k_p*process_variable + k_i*integrate(process_variable);
    // Serial.println(motorDiffCorrection);
  } else {
    error = 0;
    integralReset();
  }

  return constrain(error, error_minmax[0], error_minmax[1]);
}

// Private methods:

/**
 * integrate uses trapizodial intgration to calculate the running integral sum for the PI controller
 * @authors Grant Brautigam, Rhys Davies
 * Created: 11-19-2023
 * Updated: 2-5-2025
 * 
 * @param current_error
 * @return returns the integral sum of all current and previous values
 */
float PID::integrate(int current_error) {
  integral_sum = integral_sum + (current_error + prev_error); //*(millis()-prev_integral_time)/100;
  prev_integral_time = millis();
  prev_error = current_error;
  
  return integral_sum; 
}

void PID::integralReset() {
  integral_sum = 0;
  prev_error = 0;
  prev_integral_time = millis();
}
