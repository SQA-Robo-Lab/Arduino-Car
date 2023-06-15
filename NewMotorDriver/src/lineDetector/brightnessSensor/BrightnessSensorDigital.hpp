#ifndef BRIGHTNESS_SENSOR_DIGITAL_HPP
#define BRIGHTNESS_SENSOR_DIGITAL_HPP

#include "BrightnessSensor.hpp"

class BrightnessSensorDigital : public BrightnessSensor
{
private:
    uint8_t pinsLen;
    uint8_t *pins;

public:
    BrightnessSensorDigital(uint8_t *pinNumbers, uint8_t numberOfPins, bool pullup = false);
    ~BrightnessSensorDigital();

    uint8_t getValues(float *array, uint8_t maxLen) override;
    uint8_t numberOfSensors() override;
};

#endif