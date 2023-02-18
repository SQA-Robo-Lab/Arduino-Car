#include "LineDetector.hpp"

LineDetector::LineDetector()
{
}

LinePosition LineDetector::positionToLegacy(int8_t pos, int8_t angle)
{
    if (pos == LINE_DETECTOR_NO_LINE_FOUND || angle == LINE_DETECTOR_NO_LINE_FOUND)
    {
        return LINE_POSITION_UNKNOWN; // todo: fix legacy
    }
    if (pos > LINE_POSITION_THRESHOLD_HIGH)
    {
        return LEFT_OF_LINE;
    }
    else if (pos < -LINE_POSITION_THRESHOLD_HIGH)
    {
        return RIGHT_OF_LINE;
    }
    else if (pos > LINE_POSITION_THRESHOLD_LOW)
    {
        if (angle < -LINE_ANGLE_THRESHOLD_LOW)
        {
            return ON_LINE;
        }
        else if (angle < -LINE_ANGLE_THRESHOLD_HIGH)
        {
            return RIGHT_OF_LINE;
        }
        else
        {
            return LEFT_OF_LINE;
        }
    }
    else if (pos < -LINE_POSITION_THRESHOLD_LOW)
    {
        if (angle > LINE_ANGLE_THRESHOLD_LOW)
        {
            return ON_LINE;
        }
        else if (angle > LINE_ANGLE_THRESHOLD_HIGH)
        {
            return LEFT_OF_LINE;
        }
        else
        {
            return RIGHT_OF_LINE;
        }
    }
    else if (angle > LINE_ANGLE_THRESHOLD_LOW)
    {
        return LEFT_OF_LINE;
    }
    else if (angle < -LINE_ANGLE_THRESHOLD_LOW)
    {
        return RIGHT_OF_LINE;
    }
    return ON_LINE;
}

LinePosition LineDetector::getLegacyPosition()
{
    int8_t pos = this->getLinePositionMm();
    int8_t angle = this->getLineAngle();
    return this->positionToLegacy(pos, angle);
}