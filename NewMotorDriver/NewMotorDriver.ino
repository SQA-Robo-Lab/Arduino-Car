#include "src/TurnSteeringDriveController.hpp"
#include "src/motors/MotorDcHBridge.hpp"

Servo serv;

MotorDcHBridge rearLeft(MOTOR_SPEED_UNIT_RATIO, 6, 8, 9);
MotorDcHBridge rearRight(MOTOR_SPEED_UNIT_RATIO, 11, 12, 13);

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
    serv.attach(2);
    Serial.begin(9600);
    Serial.println("Started");
}

void loop()
{
    while (Serial.available())
    {
        char c = Serial.read();
        switch (c)
        {
        case 'a':
            dc.setAngle(10);
            break;
        case 'b':
            dc.setAngle(-10);
            break;
        case 'c':
            dc.setAngle(30);
            break;
        case 'd':
            dc.setAngle(-30);
            break;
        case 'e':
            dc.setAngle(0);
            break;
        case '5':
            dc.setSpeed(0);
            break;
        case '6':
            dc.setSpeed(64);
            break;
        case '7':
            dc.setSpeed(128);
            break;
        case '8':
            dc.setSpeed(192);
            break;
        case '9':
            dc.setSpeed(255);
            break;
        case '4':
            dc.setSpeed(-64);
            break;
        case '3':
            dc.setSpeed(-128);
            break;
        case '2':
            dc.setSpeed(-192);
            break;
        case '1':
            dc.setSpeed(-255);
            break;
        }
    }

    dc.loop();
}