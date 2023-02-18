#include "BrightnessSensorDigital.hpp"

BrightnessSensorDigital::BrightnessSensorDigital(uint8_t *pinNumbers, uint8_t numberOfPins, bool invert, bool pullup)
    : pinsLen(numberOfPins), inverted(invert)
{
    this->pins = new uint8_t[numberOfPins];
    memcpy(this->pins, pinNumbers, numberOfPins);
    for (uint16_t i = 0; i < numberOfPins; i++)
    {
        pinMode(this->pins[i], pullup ? INPUT : INPUT_PULLUP);
    }
}
BrightnessSensorDigital::~BrightnessSensorDigital()
{
    delete[] this->pins;
}

uint8_t BrightnessSensorDigital::getValues(uint8_t *array, uint8_t maxLen)
{
    uint16_t i = 0;
    for (; i < this->pinsLen && i < maxLen; i++)
    {
        if (digitalRead(this->pins[i]) xor this->inverted)
        {
            array[i] = 255;
        }
        else
        {
            array[i] = 0;
        }
    }
    return i;
}
uint8_t BrightnessSensorDigital::numberOfSensors()
{
    return this->pinsLen;
}
