#ifndef DRIVE_CONTROLLER_HPP
#define DRIVE_CONTROLLER_HPP

#include "Arduino.h"

class DriveController
{
protected:
    int16_t speed = 0;
    int8_t angle = 0;

public:
    DriveController();
    ~DriveController(){};

    void drive(int8_t angle, int16_t speed);
    void pause();
    void resume();

    void setSpeed(int16_t speed);
    void setAngle(int8_t angle);
    int16_t getSpeed();
    int8_t getAngle();

protected:
    virtual void setSpeedInternal(int16_t speed);
    virtual void setAngleInternal(int8_t angle);
};

#endif