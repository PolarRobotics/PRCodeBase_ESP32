#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

//! TODO: move to a location that makes more sense, maybe driveTypes.h, we may want to make a Enums.h header file so we can keep all enums together in one place
/** TurnFunction
 * enum to keep track of the desired turning function to use from the bot configuration
 * linear: the stick turning is a linear mapping to the R value in the turning algorithm
 * sigmoid: the stick turning is a sigmoid mapping to the R value in the turning algorithm
 * cubic: the stick turning is a x^3 mapping to the R value in the turning algorithm
 */
typedef enum {
  linear,
  sigmoid,
  cubic
} TurnFunction;

/**
 * @brief drive_param
 * a list of parameters pulled from Robot Config for bot-specific parameters related to drive especially with turning
 * 
 * used in Drive and RobotConfig
*/
typedef struct drive_param {
    MotorType motor_type;
    float gear_ratio;
    float wheel_base;
    float r_min;
    float r_max;
    bool has_encoders;
    TurnFunction turn_function;
    bool has_gyroscope; // needed for DriveStraight if an Adafruit MPU6050
} drive_param_t;

#endif // DRIVE_PARAMETERS_H