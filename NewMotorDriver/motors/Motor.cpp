#include "Motor.hpp"

Motor::Motor(MotorSpeedUnit speedUnit)
{
}

void Motor::setProfile(MotorProfile *profile)
{
    this->profile = profile;
}

void Motor::setSpeed(int32_t speed)
{
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
            this->setSpeedRatioInternal(speed);
        }
        else
        {
            // todo: implement motor profile
        }
        break;
    }
}
