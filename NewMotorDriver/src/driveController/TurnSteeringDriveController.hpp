#ifndef TURN_STEERING_DRIVE_CONTROLLER_HPP
#define TURN_STEERING_DRIVE_CONTROLLER_HPP

#include "DriveController.hpp"
#include "../motors/Motor.hpp"
#include "../motors/SteerableAxle.hpp"

class TurnSteeringDriveController : public DriveController
{
private:
    Motor &rearLeft;
    Motor &rearRight;
    SteerableAxle &steering;
    uint8_t carWidthMm;
    uint8_t carLengthMm;

public:
    TurnSteeringDriveController(
        Motor &rearLeftMotor, Motor &rearRightMotor, SteerableAxle &steeringMotor,
        uint8_t wheelDistanceWidthMm, uint8_t wheelDistanceLengthMm);
    ~TurnSteeringDriveController(){};

    void loop() override;

protected:
    void setSpeedInternal(int16_t speed) override;
    void setAngleInternal(int8_t angle) override;
};

#endif