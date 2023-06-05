#ifndef FIXED_WHEEL_DRIVE_CONTROLLER_HPP
#define FIXED_WHEEL_DRIVE_CONTROLLER_HPP

#include "DriveController.hpp"
#include "../motors/Motor.hpp"

class FixedWheelDriveController : public DriveController
{
private:
    Motor &frontLeft;
    Motor &frontRight;
    Motor &rearLeft;
    Motor &rearRight;
    uint8_t carWidthMm;
    uint8_t carLengthMm;

public:
    FixedWheelDriveController(
        Motor &frontLeftMotor, Motor &frontRightMotor,
        Motor &rearLeftMotor, Motor &rearRightMotor,
        uint8_t wheelDistanceWidthMm, uint8_t wheelDistanceLengthMm);
    FixedWheelDriveController(
        Motor &leftMotor, Motor &rightMotor,
        uint8_t wheelDistanceWidthMm, uint8_t wheelDistanceLengthMm);
    ~FixedWheelDriveController(){};

    void loop() override;

protected:
    void setSpeedInternal(int16_t speed) override;
    void setAngleInternal(int8_t angle) override;
};

#endif