#include "BrightnessSensorAnalog.hpp"

BrightnessSensorAnalog::BrightnessSensorAnalog(uint8_t *pinNumbers, uint8_t numberOfPins)
    : pinsLen(numberOfPins)
{
    this->pins = new uint8_t[numberOfPins];
    memcpy(this->pins, pinNumbers, numberOfPins);
}
BrightnessSensorAnalog::~BrightnessSensorAnalog()
{
    delete[] this->pins;
}

uint8_t BrightnessSensorAnalog::getValues(uint8_t *array, uint8_t maxLen)
{
    uint16_t i = 0;
    for (; i < this->pinsLen && i < maxLen; i++)
    {
        uint8_t val = map(analogRead(this->pins[i]), 0, 1024, 0, 256);
        if (this->mappingFunctions != nullptr)
        {
            val = this->mappingFunctions[i](val);
        }
        array[i] = val;
    }
    return i;
}
uint8_t BrightnessSensorAnalog::numberOfSensors()
{
    return this->pinsLen;
}

void BrightnessSensorAnalog::setTranslationFunctions(MappingFunction *functions)
{
    this->mappingFunctions = functions;
}