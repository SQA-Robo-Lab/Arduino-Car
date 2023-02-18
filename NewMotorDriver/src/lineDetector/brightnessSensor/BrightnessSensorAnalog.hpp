#ifndef BRIGHTNESS_SENSOR_ANALOG_HPP
#define BRIGHTNESS_SENSOR_ANALOG_HPP

#include "BrightnessSensor.hpp"

class BrightnessSensorAnalog : public BrightnessSensor
{
private:
    uint8_t pinsLen;
    uint8_t *pins;

public:
    BrightnessSensorAnalog(uint8_t *pinNumbers, uint8_t numberOfPins);
    ~BrightnessSensorAnalog();

    uint8_t getValues(uint8_t *array, uint8_t maxLen) override;
    uint8_t numberOfSensors() override;
};

#endif