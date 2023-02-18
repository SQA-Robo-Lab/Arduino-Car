#include "MotorProfile.hpp"

MotorProfile::MotorProfile()
{
}

void MotorProfile::setMaxRpmForward(uint16_t rpm)
{
    this->clipRpmForward = rpm;
}
void MotorProfile::setMaxRpmBackward(uint16_t rpm)
{
    this->clipRpmBackward = rpm;
}