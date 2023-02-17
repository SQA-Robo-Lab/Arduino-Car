#ifndef MOTOR_PROFILE_HPP
#define MOTOR_PROFILE_HPP

#include "Arduino.h"

struct MotorKnownRpm
{
    uint8_t speed;
    uint16_t rpm;
};

enum MotorCurveLutType
{
    MOTOR_CURVE_TYPE_NONE,
    MOTOR_CURVE_TYPE_RAM,
    MOTOR_CURVE_TYPE_RAM_ALLOCATED,
#ifdef ARDUINO_ARCH_AVR
    MOTOR_CURVE_TYPE_PROGMEM
#endif
};

class MotorProfile
{
protected:
    uint16_t *rpmLut = nullptr;
    MotorCurveLutType rpmLutType = MOTOR_CURVE_TYPE_NONE;
    uint16_t wheelCircumfrence = 1;

    uint8_t rpmToSpeed(uint16_t rpm);

public:
    MotorProfile(uint16_t wheelCircumfrence = 1);
    ~MotorProfile();

    void setSpeedRatio(uint8_t speed);
    void setSpeedRPM(uint16_t rpm);
    void setSpeedCmPerSec(uint16_t cmps);

    uint16_t getMaxRpm();

    void calculateMotorCurve(struct MotorKnownRpm points[], uint8_t len);
    void setMotorCurve(uint16_t *lut, MotorCurveLutType type);
};

#endif