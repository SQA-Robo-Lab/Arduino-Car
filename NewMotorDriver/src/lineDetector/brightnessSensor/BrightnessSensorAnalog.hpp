#ifndef BRIGHTNESS_SENSOR_ANALOG_HPP
#define BRIGHTNESS_SENSOR_ANALOG_HPP

#include "BrightnessSensor.hpp"

typedef float (*MappingFunction)(uint16_t oldVal);

struct BrightnessThresholds
{
    uint16_t blackThrethold;
    uint16_t whiteThrethold;
};

class BrightnessSensorAnalog : public BrightnessSensor
{
private:
    uint8_t pinsLen;
    uint8_t *pins;

    BrightnessThresholds *thresholds = nullptr;
    MappingFunction *mappingFunctions = nullptr;

public:
    BrightnessSensorAnalog(uint8_t *pinNumbers, uint8_t numberOfPins, BrightnessThresholds *briThretholds = nullptr);
    ~BrightnessSensorAnalog();

    uint8_t getValues(float *array, uint8_t maxLen) override;
    uint8_t numberOfSensors() override;

    /**
     * Number of functions must EXACTLY match number of pins
     * Setting this will unset the Thretholds
     */
    void setTranslationFunctions(MappingFunction *functions);

    /**
     * Number of threshold structs must EXACTLY match number of pins
     * Setting this will unset the translation functions
     */
    void setThresholds(BrightnessThresholds *briThretholds);
};

#endif