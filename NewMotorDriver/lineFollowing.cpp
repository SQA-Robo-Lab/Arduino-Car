#include "Arduino.h"
#include "lineFollowing.hpp"

// #define TIMED_STEERING

#ifdef TIMED_STEERING
#define MS_UNTIL_MAX_ANGLE 1000
#define MIN_ANGLE 5
#define MAX_ANGLE 25
float lastAngle = 0;
unsigned long lastTime = millis();
#endif

void lineFollowing(DriveController &dc, LineDetector &ld, DistanceSensor &distFront, DistanceSensor &distRear)
{
    uint16_t d = distFront.getDistanceToClosestMm();

    if (d < 100 && dc.getSpeed() != 0)
    {
        dc.pause();
    }
    else if (d > 100 && dc.getSpeed() == 0)
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
#elif defined(TIMED_STEERING)
        unsigned long thisTime = millis();
        float newAngle = lastAngle;
        if ((lastAngle < 0 && lp >= 0) || (lastAngle > 0 && lp <= 0))
        {
            newAngle = 0;
        }
        else
        {
            lastAngle = abs(lastAngle);
            float dstAngle = abs(lp) * MAX_ANGLE / 16;
            if (dstAngle > lastAngle)
            {
                float incAngle = (thisTime - lastTime) * dstAngle / MS_UNTIL_MAX_ANGLE;
                newAngle = lastAngle + incAngle;
            }
            else
            {
                newAngle = dstAngle;
            }
            newAngle = min(MAX_ANGLE, newAngle);
            if (lp < 0)
            {
                newAngle = -newAngle;
            }
        }
        dc.setAngle(newAngle);
        lastAngle = newAngle;
        lastTime = thisTime;
#else
        // dc.setAngle(lp * 20 / 16);
#endif
    }
}