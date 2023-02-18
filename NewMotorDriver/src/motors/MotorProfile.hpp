#ifndef MOTOR_PROFILE_HPP
#define MOTOR_PROFILE_HPP

#include "Arduino.h"

class MotorProfile
{
protected:
    uint16_t clipRpmForward = 0;
    uint16_t clipRpmBackward = 0;

public:
    MotorProfile();
    ~MotorProfile() {}

    virtual int16_t rpmToRatio(int16_t rpm) = 0;

    virtual uint16_t getMaxPossibleRpmForward() = 0;
    virtual uint16_t getMaxPossibleRpmBackward() = 0;

    void setMaxRpmForward(uint16_t rpm);
    void setMaxRpmBackward(uint16_t rpm);

    void loop(){};
};

#endif