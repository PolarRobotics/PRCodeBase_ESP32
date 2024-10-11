#pragma once

#include <Arduino.h>
#include <PolarRobotics.h>
// #include <MotorInterface.h>
#include <MotorInterface.h>


// Enum for Increasing or Decreasing Flywheel Speed
enum SpeedStatus {
  INCREASE, DECREASE
};

class MotorControl {
private:
  MotorType motor_type; // the type of motor to be assigned to this object
  float gear_ratio;     // the input / output gear ratio
  // float deadZone = 0.001;

  bool uesCurveFit = false;

  // Servo:
  MotorInterface Motor;

  // write()
  float pct;

  // for ramp
  
  float requestedRPM;     
  float lastRampTime;
  float timeElapsed;

  // for sendRPM
  coeffs_t coeff;
  bool negativeDir = false;
  float pct = 0;

  //PILoop
  int adjusted_speed;
  int error;
  float k_p;
  float k_i;
  float deadZone; //needs to be peramiterized

  //setTargetSpeed
  int ramped_speed;
  int set_speed;
  bool CL_enable;

  //getCurrentSpeed
  int current_speed;

  //integrate
  int prev_current_error;
  int integral_sum;
  unsigned long prev_integral_time;



public:
  int max_rpm;          // the motor max rpm * the gear ratio 
  MotorControl();
  uint8_t setup(int mot_pin, MotorType type = big_ampflow, float gearRatio = 1); 

  void write(float pwr);
  void writeRPM(int rpm);
  void stop();

  int ramp(int requestedPower, float accelRate);
  
  int Percent2RPM(float pct);
  float RPM2Percent(int rpm);
  
  void setTargetSpeed(int target_rpm);
  void setCurrentSpeed(int speed);

  void sendRPM(int rpm);

  void setTargetSpeed(int target_rpm);
  int PILoop(int target_speed);
  int integrate(int current_error);
  void integrateReset();

};