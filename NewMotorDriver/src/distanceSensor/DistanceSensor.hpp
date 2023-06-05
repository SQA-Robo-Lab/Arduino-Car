#ifndef DISTANCE_SENSOR_HPP
#define DISTANCE_SENSOR_HPP

#include "Arduino.h"

#define DISTANCE_SENSOR_NO_OBJECT UINT16_MAX

class DistanceSensor
{
public:
    DistanceSensor(){};
    ~DistanceSensor(){};

    virtual uint16_t getDistanceToClosestMm() = 0;
    virtual int8_t getAngleToClosest() = 0;
};

#endif