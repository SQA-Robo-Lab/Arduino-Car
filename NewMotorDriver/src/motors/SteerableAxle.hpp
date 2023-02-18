#ifndef STEERABLE_AXLE_HPP
#define STEERABLE_AXLE_HPP

#include "Arduino.h"

class SteerableAxle
{
protected:
    uint8_t maxLeftAngle;
    uint8_t maxRightAngle;

public:
    SteerableAxle(uint8_t maxSteeringLeftAngle, uint8_t maxSteeringRightAngle);
    ~SteerableAxle() {}

    virtual void setAngle(int8_t angle) = 0;
};

#endif