#ifndef MOTOR_PROFILE_HPP
#define MOTOR_PROFILE_HPP

#include "Arduino.h"
#include <initializer_list>

struct MotorKnownRpm
{
    uint8_t speed;
    uint16_t rpm;
};

class MotorProfile
{
protected:
    float *motorCurveCoefficients;
    uint8_t coefficientsSize = 0;
    uint16_t wheelCircumfrence = 1;

    uint8_t rpmToSpeed(uint16_t rpm);

public:
    MotorProfile();
    ~MotorProfile(){};

    void setSpeedRatio(uint8_t speed);
    void setSpeedRPM(uint16_t rpm);
    void setSpeedCmPerSec(uint16_t cmps);

    uint16_t getMaxRpm();

    void calculateMotorCurve(struct MotorKnownRpm points[], uint8_t len);
};

#endif