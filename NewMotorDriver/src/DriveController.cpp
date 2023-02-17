#include "DriveController.hpp"

DriveController::DriveController()
{
}

void DriveController::drive(int8_t angle, int16_t speed)
{
    this->setAngle(angle);
    this->setSpeed(speed);
}

void DriveController::pause()
{
    this->setSpeedInternal(0);
}
void DriveController::resume()
{
    this->setSpeedInternal(speed);
}

void DriveController::setSpeed(int16_t speed)
{
    this->speed = speed;
    this->setSpeedInternal(speed);
}
void DriveController::setAngle(int8_t angle)
{
    this->angle = angle;
    this->setAngleInternal(angle);
}

int16_t DriveController::getSpeed()
{
    return this->speed;
}
int8_t DriveController::getAngle()
{
    return this->angle;
}