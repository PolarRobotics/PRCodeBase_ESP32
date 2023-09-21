#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#define NUM_MOTOR_TYPES 4

#include <Utilities/Pair.h>

typedef enum {
  big,      // MOTOR_TYPE value of 0 for 24v AmpFlow motor E30-400
  small,    // MOTOR_TYPE value of 1 for 24v AmpFlow motor E30-150
  mecanum,  // MOTOR_TYPE value of 2 for small 12v mecanum motors
  falcon    // MOTOR_TYPE value of 3 for the falcon motors on the runningback
} MotorType;

const char* getMotorTypeString(MotorType type);

#endif // MOTOR_TYPES_H