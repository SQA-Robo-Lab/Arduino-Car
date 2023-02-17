#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "Arduino.h"
#include "MotorProfile.hpp"

enum MotorSpeedUnit
{
    /**
     * Ration of the full speed. Where -255 is full speed backwards, 0 is stopped and 255 is full speed forward
     */
    MOTOR_SPEED_UNIT_RATIO,
    MOTOR_SPEED_UNIT_RPM,
    MOTOR_SPEED_UNIT_CM_PER_SEC
};

class Motor
{
protected:
    MotorSpeedUnit unit;
    MotorProfile *profile = nullptr;

public:
    Motor(MotorSpeedUnit speedUnit);
    ~Motor(){};

    void setProfile(MotorProfile *profile);
    void setSpeed(int32_t speed);

    void loop(){};

protected:
    virtual void setSpeedRatioInternal(int16_t speedRatio);
};

#endif