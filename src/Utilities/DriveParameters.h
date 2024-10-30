#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

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