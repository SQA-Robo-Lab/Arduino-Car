#ifndef BRIGHTNESS_SENSOR_HPP
#define BRIGHTNESS_SENSOR_HPP

#include "Arduino.h"

class BrightnessSensor
{
public:
    BrightnessSensor(){};
    ~BrightnessSensor(){};

    virtual uint8_t getValues(float *array, uint8_t maxLen) = 0;
    virtual uint8_t numberOfSensors() = 0;
};

#endif