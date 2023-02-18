#include "Motor.hpp"
#include "Arduino.h"

// #define MOTOR_DEBUG

Motor::Motor(MotorSpeedUnit speedUnit)
    : unit(speedUnit)
{
}

void Motor::setProfile(MotorProfile *profile)
{
    this->profile = profile;
}

void Motor::setSpeed(int32_t speed)
{
    int16_t speedRatio = 0;
    switch (this->unit)
    {
    case MOTOR_SPEED_UNIT_RPM:
        if (this->profile != nullptr)
        {
            speedRatio = this->profile->rpmToRatio(speed);
#ifdef MOTOR_DEBUG
            Serial.print("Translated rpm ");
            Serial.print(speed);
            Serial.print(" into ratio ");
            Serial.println(speedRatio);
#endif
        }
        else
        {
            speedRatio = static_cast<int16_t>(speed);
        }
        break;
    case MOTOR_SPEED_UNIT_CM_PER_SEC:
        // todo: implement translation using profile
        break;
    case MOTOR_SPEED_UNIT_RATIO:
    default:
#ifdef MOTOR_DEBUG
        Serial.print("Settting speed to ");
        Serial.println(speed);
#endif
        speedRatio = static_cast<int16_t>(speed);
        break;
    }
    this->setSpeedRatioInternal(min(255, max(-255, speedRatio)));
}

void Motor::loop()
{
    if (this->profile != nullptr)
    {
        profile->loop();
    }
}