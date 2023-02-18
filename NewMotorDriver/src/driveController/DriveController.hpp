#ifndef DRIVE_CONTROLLER_HPP
#define DRIVE_CONTROLLER_HPP

#include "Arduino.h"

enum DriveControllerState
{
    DRIVE_CONTROLLER_STATE_STOPPED,
    DRIVE_CONTROLLER_STATE_PAUSED,
    DRIVE_CONTROLLER_STATE_DRIVING,
};

class DriveController
{
protected:
    int16_t speed = 0;
    int8_t angle = 0;
    DriveControllerState state = DRIVE_CONTROLLER_STATE_STOPPED;

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

    virtual void loop() = 0;

protected:
    virtual void setSpeedInternal(int16_t speed) = 0;
    virtual void setAngleInternal(int8_t angle) = 0;
};

#endif