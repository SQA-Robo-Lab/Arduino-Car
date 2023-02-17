#include "Motor.hpp"
#include "Arduino.h"

Motor::Motor(MotorSpeedUnit speedUnit)
{
}

void Motor::setProfile(MotorProfile *profile)
{
    this->profile = profile;
}

void Motor::setSpeed(int32_t speed)
{
    Serial.print("Settting speed to ");
    Serial.println(speed);
    switch (this->unit)
    {
    case MOTOR_SPEED_UNIT_RPM:
        // todo: implement translation using profile
        break;
    case MOTOR_SPEED_UNIT_CM_PER_SEC:
        // todo: implement translation using profile
        break;
    case MOTOR_SPEED_UNIT_RATIO:
    default:
        if (this->profile == nullptr)
        {
            this->setSpeedRatioInternal(static_cast<int16_t>(speed));
        }
        else
        {
            // todo: implement motor profile
        }
        break;
    }
}
