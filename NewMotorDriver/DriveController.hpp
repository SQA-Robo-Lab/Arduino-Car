#ifndef DRIVE_CONTROLLER_HPP
#define DRIVE_CONTROLLER_HPP

#include "Arduino.h"

class DriveController
{
public:
    DriveController();
    ~DriveController();

    void drive(int8_t angle, int16_t speed);
    void pause();
    void resume();

    void setSpeed(int16_t speed);
    void setAngle(int8_t angle);
};

#endif