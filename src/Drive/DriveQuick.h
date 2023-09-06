#include <Arduino.h>
// the minimum power that can be written to the motor, prevents stalling
#define MOTOR_ZERO_OFFST 0.05
#define RB_ACCELERATION_RATE 0.03f //default: 0.0375f, Runningback old: 0.03f, 0.015f
#include <Drive/Drive.h>

class DriveQuick : public Drive {
    private:
        float falcon_motor_pwr[NUM_MOTORS];
        float r, falconTurnPwr, max;
    public:
        void generateMotionValues();
        // void setMaxPWR();
        void update();
        void printDebugInfo();
};