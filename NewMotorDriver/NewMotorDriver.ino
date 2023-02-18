#include "src/driveController/TurnSteeringDriveController.hpp"
#include "src/motors/MotorDcHBridge.hpp"
#include "src/motors/PolyCurveMotorProfile.hpp"
#include "src/lineDetector/LinearSensorLineDetector.hpp"
#include "src/lineDetector/brightnessSensor/BrightnessSensorAnalog.hpp"
#include "lineFollowing.hpp"
#include "src/motors/ServoAxle.hpp"
#include "Servo.h"

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

// left: weiß: 224-225, tape: 221-224, schwarz: 240-244, schwarzTape: 234-236 => min: 221, max: 244 => offset: 13, factor:
// middle: weiß: 208-210, tape: 211-213, schwarz: 234-240, schwarzTape: 222-230 => min: 208, max: 240 => offset: 0, factor: 1
// right: weiß: 229-230, tape: 226-228, schwarz: 240-246, schwarzTape: 239-241 => min: 226, max: 246 => offset: 18, factor:
uint8_t pins[] = {A0, A1, A2};
MappingFunction mappingFns[] = {
    [](uint8_t val)
    {
        Serial.print("Left: ");
        long after = map(val, 11, 34, 15, 47);
        Serial.println(after);
        return static_cast<uint8_t>(after);
    },
    [](uint8_t val)
    {
        Serial.print("Middle: ");
        Serial.println(val);
        return val;
    },
    [](uint8_t val)
    {
        Serial.print("Right: ");
        long after = map(val, 9, 29, 15, 47);
        Serial.println(after);
        return static_cast<uint8_t>(after);
    }};
BrightnessSensorAnalog sensor(pins, 3);

LinearSensorLineDetector ld(sensor, 16, 47, 19);

void setup()
{ // middle: 66, right: 109, left: 19
    profile.setMotorCurve(const_cast<int16_t *>(rpmLut), MOTOR_CURVE_TYPE_PROGMEM);
    // rearLeft.setProfile(&profile);
    // rearRight.setProfile(&profile);
    Serial.begin(115200);
    Serial.println("Started");
    dc.setSpeed(70);
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
    Serial.print("Detected line pos: ");
    Serial.print(pos);
    Serial.print("mm; Legacy: ");
    Serial.println(ld.positionToLegacy(pos, angle));

    lineFollowing(dc, ld);
    delay(50);
}