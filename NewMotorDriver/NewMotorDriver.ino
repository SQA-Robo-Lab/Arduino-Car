#include "TurnSteeringDriveController.hpp"
#include "motors/MotorDcHBridge.hpp"

Servo serv;

TurnSteeringDriveController dc(
    MotorDcHBridge(MOTOR_SPEED_UNIT_RATIO, 1, 2, 3),
    MotorDcHBridge(MOTOR_SPEED_UNIT_RATIO, 1, 2, 3),
    serv,
    120,
    100,
    30,
    30);

void setup()
{
}

void loop()
{
}