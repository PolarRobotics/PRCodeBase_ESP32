#include <Arduino.h>

// template <typename TYPE>
class PID {
private:
  // "Constants"
  float k_p;
  float k_i;
  float k_d;
  float error_threshold;

  float error_minmax[2]; //sets bounds on the output of the controller
  // float max_err;
  // float min_;

  float process_variable; //measured data from the sensor
  float set_point; // commanded value for the controller to try and match
  float error; //difference between commanded value and actual measured value
  bool cl_enable; // enable or disables the CL calculations
  
  // Integration
  float prev_error;
  float integral_sum;
  unsigned long prev_integral_time;
  
  float integrate(int current_error);
  void integralReset();
public:
  PID(float k_p, float k_i, float k_d = 0.0f, float error_thresh, 
    float error_min, float error_max);
  void setCLState(bool state);
  bool setProcessVar(int measured_value);
  bool setSetpoint(float commanded_value);
  float PILoop();
};
