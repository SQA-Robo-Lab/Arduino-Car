#include "TurnSteeringDriveController.hpp"

TurnSteeringDriveController::TurnSteeringDriveController(
    Motor rearLeftMotor, Motor rearRightMotor, Servo steering,
    uint8_t wheelDistanceWidthMm, uint8_t wheelDistanceLengthMm, uint8_t maxSteeringLeftAngle, uint8_t maxSteeringRightAngle)
    : rearLeft(rearLeftMotor), rearRight(rearRightMotor), servo(steering),
      carWidthMm(wheelDistanceWidthMm), carLengthMm(wheelDistanceLengthMm), maxLeftAngle(maxSteeringLeftAngle), maxRightAngle(maxSteeringRightAngle)
{
}

void TurnSteeringDriveController::setSpeedInternal(int16_t speed)
{
    // todo: speed adjustment for curve
    this->rearLeft.setSpeed(speed);
    this->rearRight.setSpeed(speed);
}

void TurnSteeringDriveController::setAngleInternal(int8_t angle)
{
    // todo: speed adjustment for curve
    this->servo.write(map(angle, (-1) * ((int16_t)this->maxLeftAngle), this->maxRightAngle, 0, 180));
}
