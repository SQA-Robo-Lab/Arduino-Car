#ifndef BRIGHTNESS_SENSOR_ANALOG_HPP
#define BRIGHTNESS_SENSOR_ANALOG_HPP

#include "BrightnessSensor.hpp"

typedef uint8_t (*MappingFunction)(uint8_t oldVal);

class BrightnessSensorAnalog : public BrightnessSensor
{
private:
    uint8_t pinsLen;
    uint8_t *pins;

    MappingFunction *mappingFunctions = nullptr;

public:
    BrightnessSensorAnalog(uint8_t *pinNumbers, uint8_t numberOfPins);
    ~BrightnessSensorAnalog();

    uint8_t getValues(uint8_t *array, uint8_t maxLen) override;
    uint8_t numberOfSensors() override;

    /**
     * Number of functions must EXACTLY match number of pins
     */
    void setTranslationFunctions(MappingFunction *functions);
};

#endif