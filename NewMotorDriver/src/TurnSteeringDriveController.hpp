#ifndef TURN_STEERING_DRIVE_CONTROLLER_HPP
#define TURN_STEERING_DRIVE_CONTROLLER_HPP

#include "DriveController.hpp"
#include "motors/Motor.hpp"
#include "Servo.h"

struct ServoMotor
{
    const uint8_t dataPin;
    const uint8_t leftMaxAngle;
    const uint8_t rightMaxAngle;
};

class TurnSteeringDriveController : public DriveController
{
private:
    Motor &rearLeft;
    Motor &rearRight;
    Servo &servo;
    uint8_t carWidthMm;
    uint8_t carLengthMm;
    uint8_t maxLeftAngle;
    uint8_t maxRightAngle;

public:
    TurnSteeringDriveController(
        Motor &rearLeftMotor, Motor &rearRightMotor, Servo &steering,
        uint8_t wheelDistanceWidthMm, uint8_t wheelDistanceLengthMm, uint8_t maxSteeringLeftAngle, uint8_t maxSteeringRightAngle);
    ~TurnSteeringDriveController(){};

    void loop() override;

protected:
    void setSpeedInternal(int16_t speed) override;
    void setAngleInternal(int8_t angle) override;
};

#endif