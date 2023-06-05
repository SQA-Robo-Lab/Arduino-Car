#include "FixedWheelDriveController.hpp"

FixedWheelDriveController::FixedWheelDriveController(
    Motor &frontLeftMotor, Motor &frontRightMotor,
    Motor &rearLeftMotor, Motor &rearRightMotor,
    uint8_t wheelDistanceWidthMm, uint8_t wheelDistanceLengthMm)
    : frontLeft(frontLeftMotor), frontRight(frontRightMotor),
      rearLeft(rearLeftMotor), rearRight(rearRightMotor),
      carWidthMm(wheelDistanceWidthMm), carLengthMm(wheelDistanceLengthMm)
{
}

FixedWheelDriveController::FixedWheelDriveController(
    Motor &leftMotor, Motor &rightMotor,
    uint8_t wheelDistanceWidthMm, uint8_t wheelDistanceLengthMm)
    : FixedWheelDriveController(leftMotor, rightMotor, leftMotor, rightMotor, wheelDistanceWidthMm, wheelDistanceLengthMm)
{
}

void FixedWheelDriveController::setSpeedInternal(int16_t speed)
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

void FixedWheelDriveController::setAngleInternal(int8_t angle)
{
    // todo
    this->setSpeedInternal(this->getSpeed());
}

void FixedWheelDriveController::loop()
{
    this->frontLeft.loop();
    this->frontRight.loop();
    this->rearLeft.loop();
    this->rearRight.loop();
}