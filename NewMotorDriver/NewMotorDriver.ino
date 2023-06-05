#include "src/driveController/TurnSteeringDriveController.hpp"
#include "src/motors/MotorDcHBridge.hpp"
#include "src/motors/PolyCurveMotorProfile.hpp"
#include "src/lineDetector/LinearSensorEdgeDetector.hpp"
#include "src/lineDetector/brightnessSensor/BrightnessSensorAnalog.hpp"
#include "lineFollowing.hpp"
#include "src/motors/ServoAxle.hpp"
#include "Servo.h"
#include "Arduino.h"

#include "WifiDebug.hpp"
WifiDebug *debug;

const PROGMEM int16_t rpmLut[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 64, 64, 64, 64, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};

ServoAxle serv(2, 40, 39, 19, 66, 109);

PolyCurveMotorProfile profile;

MotorDcHBridge rearLeft(MOTOR_SPEED_UNIT_RPM, 6, 7, 8);
MotorDcHBridge rearRight(MOTOR_SPEED_UNIT_RPM, 11, 12, 10);

TurnSteeringDriveController dc(
    rearLeft,
    rearRight,
    serv,
    120,
    100);

// left: weiß: 30-34, tape: 32-36, schwarz: 9-13, schwarzTape: 18-21 => min: 30 max: 21 => offset: 13, factor:
// middle: weiß: 42-47, tape: 42-48, schwarz: 13-19, schwarzTape: 23-28 => min: 42, max: 28 => offset: 0, factor: 1
// right: weiß: 24-30, tape: 22-32, schwarz: 7-10, schwarzTape: 13-19 => min: 22, max: 19 => offset: 18, factor:

// printed
//  left: weiß: 31-45, schwarz: 11 => min: 11 max: 31 => offset: 13, factor:
//  middle: weiß: 44-55, schwarz: 14 => min: 14, max: 44 => offset: 0, factor: 1
//  right: weiß: 28-40, schwarz: 8 => min: 8, max: 28 => offset: 18, factor:
uint8_t pins[] = {A0, A1, A2};
MappingFunction mappingFns[] = {
    [](uint8_t val)
    {
        // long after = map(val, 21, 30, 0, 100); // painted
        long after = map(val, 11, 31, 0, 100); // printed
        //  Serial.print("Left: ");
        //  Serial.println(after);
        // return val;
        return static_cast<uint8_t>(after);
    },
    [](uint8_t val)
    {
        // long after = map(val, 21, 30, 0, 100); // painted
        long after = map(val, 14, 44, 0, 100); // printed
        // Serial.print("Middle: ");
        // Serial.println(val);
        // return val;
        return static_cast<uint8_t>(after);
    },
    [](uint8_t val)
    {
        // long after = map(val, 21, 30, 0, 100); // painted
        long after = map(val, 8, 28, 0, 100); // printed
        // Serial.print("Right: ");
        // Serial.println(after);
        // return val;
        return static_cast<uint8_t>(after);
    }};
BrightnessSensorAnalog sensor(pins, 3);

LinearSensorEdgeDetector ld(sensor, 16, 100, 0, false);

void setup()
{ // middle: 66, right: 109, left: 19
    debug = new WifiDebug();

    profile.setMotorCurve(const_cast<int16_t *>(rpmLut), MOTOR_CURVE_TYPE_PROGMEM);
    // rearLeft.setProfile(&profile);
    // rearRight.setProfile(&profile);
    Serial.begin(115200);
    Serial.println("Started");
    dc.setSpeed(64);
    sensor.setTranslationFunctions(mappingFns);
}

void loop()
{
    int16_t readAngle = INT16_MAX;
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
        /*if (c == '-')
        {
            readAngle *= (-1);
        }
        else if (c >= '0' && c <= '9')
        {
            if (readAngle == INT16_MAX)
            {
                readAngle = c - '0';
            }
            else if (readAngle == -INT16_MAX)
            {
                readAngle = -(c - '0');
            }
            else
            {
                readAngle = readAngle * 10 + (c - '0');
            }
        }*/
    }
    if (readAngle != INT16_MAX && readAngle != -INT16_MAX)
    {
        Serial.print("Writing servo: ");
        Serial.println(readAngle);
        serv.setAngle(readAngle);
    }

    dc.loop();

    int8_t pos = ld.getLinePositionMm();
    int8_t angle = ld.getLineAngle();
    /*Serial.print("Detected line pos: ");
    Serial.print(pos);
    Serial.print("mm; Legacy: ");
    Serial.println(ld.positionToLegacy(pos, angle));*/

    lineFollowing(dc, ld);
    delay(50);
}