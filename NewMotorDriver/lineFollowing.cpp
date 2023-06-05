#include "Arduino.h"
#include "lineFollowing.hpp"

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
#else
        dc.setAngle(lp * 30 / 16);
#endif
    }
}