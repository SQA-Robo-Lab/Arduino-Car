#include "TurnSteeringDriveController.hpp"

TurnSteeringDriveController::TurnSteeringDriveController(
    Motor &rearLeftMotor, Motor &rearRightMotor, SteerableAxle &steeringMotor,
    uint8_t wheelDistanceWidthMm, uint8_t wheelDistanceLengthMm)
    : rearLeft(rearLeftMotor), rearRight(rearRightMotor), steering(steeringMotor),
      carWidthMm(wheelDistanceWidthMm), carLengthMm(wheelDistanceLengthMm)
{
}

void TurnSteeringDriveController::setSpeedInternal(int16_t speed)
{
    if (this->angle == 0)
    {
        this->rearLeft.setSpeed(speed);
        this->rearRight.setSpeed(speed);
        return;
    }

    float innerRadius = tan((90.0 - abs(this->angle)) * PI / 180.0) * this->carLengthMm;
    float middleRadius = innerRadius + this->carWidthMm / 2;
    float outerWheelSpeedFactor = (innerRadius + this->carWidthMm) / middleRadius;
    float innerWheelSpeedFactor = innerRadius / middleRadius;
    if (this->angle > 0)
    {
        this->rearLeft.setSpeed(speed * outerWheelSpeedFactor);
        this->rearRight.setSpeed(speed * innerWheelSpeedFactor);
    }
    else
    {
        this->rearLeft.setSpeed(speed * innerWheelSpeedFactor);
        this->rearRight.setSpeed(speed * outerWheelSpeedFactor);
    }
}

void TurnSteeringDriveController::setAngleInternal(int8_t angle)
{
    this->steering.setAngle(angle);
    this->setSpeedInternal(this->getSpeed());
}

void TurnSteeringDriveController::loop()
{
    this->rearLeft.loop();
    this->rearRight.loop();
}