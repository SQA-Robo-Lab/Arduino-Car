#ifndef BRIGHTNESS_SENSOR_DIGITAL_HPP
#define BRIGHTNESS_SENSOR_DIGITAL_HPP

#include "BrightnessSensor.hpp"

class BrightnessSensorDigital : public BrightnessSensor
{
private:
    uint8_t pinsLen;
    uint8_t *pins;
    bool inverted;

public:
    BrightnessSensorDigital(uint8_t *pinNumbers, uint8_t numberOfPins, bool invert = false, bool pullup = false);
    ~BrightnessSensorDigital();

    uint8_t getValues(uint8_t *array, uint8_t maxLen) override;
    uint8_t numberOfSensors() override;
};

#endif