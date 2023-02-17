#include "src/TurnSteeringDriveController.hpp"
#include "src/motors/MotorDcHBridge.hpp"

Servo serv;

MotorDcHBridge rearLeft(MOTOR_SPEED_UNIT_RATIO, 1, 2, 3);
MotorDcHBridge rearRight(MOTOR_SPEED_UNIT_RATIO, 1, 2, 3);

TurnSteeringDriveController dc(
    rearLeft,
    rearRight,
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