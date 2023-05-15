#include "Arduino.h"
#include "lineFollowing.hpp"

#define FRONT_TRIGGER_PIN 13
#define FRONT_ECHO_PIN 4
#define REAR_TRIGGER_PIN 13
#define REAR_ECHO_PIN 4
#define ECHO_TIMEOUT_US 10000

// #define QUADRATIC_STEERING

uint16_t calculateDistanceInCm(unsigned long duration)
{
    if (duration == 0)
    {
        return (ECHO_TIMEOUT_US / 2) * 0.0343;
    }
    else
    {
        return (duration / 2) * 0.0343;
    }
}

uint16_t getFrontDistance()
{
    pinMode(FRONT_TRIGGER_PIN, OUTPUT);
    pinMode(FRONT_ECHO_PIN, INPUT);
    // clear the trigger pin
    digitalWrite(FRONT_TRIGGER_PIN, LOW);
    delayMicroseconds(5);
    // issue the trigger
    digitalWrite(FRONT_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    // stop the trigger
    digitalWrite(FRONT_TRIGGER_PIN, LOW);

    int duration = pulseIn(FRONT_ECHO_PIN, HIGH, ECHO_TIMEOUT_US);

    return calculateDistanceInCm(duration);
}

void lineFollowing(DriveController &dc, LineDetector &ld)
{
    uint16_t d = getFrontDistance();

    /*Serial.print("Front distance: ");
    Serial.println(d);*/

    if (d < 10 && dc.getSpeed() != 0)
    {
        dc.pause();
    }
    else if (d > 10 && dc.getSpeed() == 0)
    {
        dc.resume();
    }

    int8_t lp = ld.getLinePositionMm();

    if (lp != LINE_DETECTOR_NO_LINE_FOUND)
    {
#ifdef QUADRATIC_STEERING
        long angle = lp;
        angle = (30 * angle * angle) / 256;
        if (lp < 0)
        {
            angle *= -1;
        }
        Serial.print("Setting angle to ");
        Serial.println(angle);
        dc.setAngle(angle);
#else
        dc.setAngle(lp * 30 / 16);
#endif
    }
}