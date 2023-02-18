#ifndef LINE_DETECTOR_HPP
#define LINE_DETECTOR_HPP

#include "Arduino.h"

#define LINE_POSITION_THRESHOLD_LOW 12
#define LINE_POSITION_THRESHOLD_HIGH 45
#define LINE_ANGLE_THRESHOLD_LOW 10
#define LINE_ANGLE_THRESHOLD_HIGH 20

#define LINE_DETECTOR_NO_LINE_FOUND -128

typedef enum
{
    LEFT_OF_LINE,
    ON_LINE,
    RIGHT_OF_LINE,
} LinePosition;

class LineDetector
{

public:
    LineDetector();
    ~LineDetector(){};

    LinePosition getLegacyPosition();
    virtual int8_t getLinePositionMm() = 0;
    virtual int8_t getLineAngle() = 0;
};

#endif