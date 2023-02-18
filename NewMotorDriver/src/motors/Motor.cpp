#include "Motor.hpp"
#include "Arduino.h"

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
            Serial.print("Translated rpm ");
            Serial.print(speed);
            Serial.print(" into ratio ");
            Serial.println(speedRatio);
        }
        break;
    case MOTOR_SPEED_UNIT_CM_PER_SEC:
        // todo: implement translation using profile
        break;
    case MOTOR_SPEED_UNIT_RATIO:
    default:
        Serial.print("Settting speed to ");
        Serial.println(speed);
        speedRatio = static_cast<int16_t>(speed);
        break;
    }
    this->setSpeedRatioInternal(min(255, max(-255, speedRatio)));
}
