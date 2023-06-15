#include "BrightnessSensorAnalog.hpp"

#define BRIGHTNESS_SENSOR_DEBUG

float floatMap(float val, float inMin, float inMax, float outMin, float outMax)
{
    return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

BrightnessSensorAnalog::BrightnessSensorAnalog(uint8_t *pinNumbers, uint8_t numberOfPins, BrightnessThresholds *briThretholds = nullptr)
    : pinsLen(numberOfPins), thresholds(briThretholds)
{
    this->pins = new uint8_t[numberOfPins];
    memcpy(this->pins, pinNumbers, numberOfPins);
}
BrightnessSensorAnalog::~BrightnessSensorAnalog()
{
    delete[] this->pins;
}

uint8_t BrightnessSensorAnalog::getValues(float *array, uint8_t maxLen)
{
    uint16_t i = 0;
#ifdef BRIGHTNESS_SENSOR_DEBUG
    Serial.print("Raw brightness values: ");
#endif
    for (; i < this->pinsLen && i < maxLen; i++)
    {
        uint16_t val = analogRead(this->pins[i]);
#ifdef BRIGHTNESS_SENSOR_DEBUG
        Serial.print(val);
        Serial.print("; ");
#endif
        float resVal = val;
        if (this->mappingFunctions != nullptr)
        {
            resVal = this->mappingFunctions[i](val);
        }
        else if (this->thresholds != nullptr)
        {
            resVal = floatMap(val, this->thresholds[i].blackThrethold, this->thresholds[i].whiteThrethold, 0, 1);
        }
        else
        {
            resVal = floatMap(val, 0, 1024, 0, 1);
        }
        array[i] = resVal;
    }
#ifdef BRIGHTNESS_SENSOR_DEBUG
    Serial.println();
#endif
    return i;
}

uint8_t BrightnessSensorAnalog::numberOfSensors()
{
    return this->pinsLen;
}

void BrightnessSensorAnalog::setTranslationFunctions(MappingFunction *functions)
{
    this->mappingFunctions = functions;
    this->thresholds = nullptr;
}

void BrightnessSensorAnalog::setThresholds(BrightnessThresholds *briThretholds)
{
    this->thresholds = briThretholds;
    this->mappingFunctions = nullptr;
}