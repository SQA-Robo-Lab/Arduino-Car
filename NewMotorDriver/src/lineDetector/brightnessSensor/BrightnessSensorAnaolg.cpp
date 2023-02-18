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
        array[i] = map(analogRead(this->pins[i]), 0, 1024, 0, 256);
    }
    return i;
}
uint8_t BrightnessSensorAnalog::numberOfSensors()
{
    return this->pinsLen;
}
