#include "MotorDcHBridge.hpp"
#include "Arduino.h"

MotorDcHBridge::MotorDcHBridge(MotorSpeedUnit speedUnit, uint8_t speedEnablePin, uint8_t polarityDirectionPin1, uint8_t polarityDirectionPin2)
    : Motor(speedUnit), speedPin(speedEnablePin), polarityPin1(polarityDirectionPin1), polarityPin2(polarityDirectionPin2)
{
    pinMode(this->speedPin, OUTPUT);
    pinMode(this->polarityPin1, OUTPUT);
    pinMode(this->polarityPin2, OUTPUT);
}

void MotorDcHBridge::setSpeedRatioInternal(int16_t speedRatio)
{
    Serial.print("Settting ratio to ");
    Serial.println(speedRatio);
    if (speedRatio == 0)
    {
        digitalWrite(this->polarityPin1, 0);
        digitalWrite(this->polarityPin2, 0);
        digitalWrite(this->speedPin, 1);
    }
    else
    {
        if (speedRatio < 0)
        {
            digitalWrite(this->polarityPin1, 0);
            digitalWrite(this->polarityPin2, 1);
            analogWrite(this->speedPin, min(255, max(0, -speedRatio)));
        }
        else
        {
            digitalWrite(this->polarityPin1, 1);
            digitalWrite(this->polarityPin2, 0);
            analogWrite(this->speedPin, min(255, max(0, speedRatio)));
        }
    }
}
