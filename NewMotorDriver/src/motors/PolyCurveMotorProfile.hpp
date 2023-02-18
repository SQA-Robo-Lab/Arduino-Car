#ifndef POLY_CURVE_MOTOR_PROFILE_HPP
#define POLY_CURVE_MOTOR_PROFILE_HPP

#include "Arduino.h"
#include "MotorProfile.hpp"

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

class PolyCurveMotorProfile : public MotorProfile
{
protected:
    int16_t *rpmLut = nullptr;
    MotorCurveLutType rpmLutType = MOTOR_CURVE_TYPE_NONE;

public:
    PolyCurveMotorProfile();
    ~PolyCurveMotorProfile();

    int16_t rpmToRatio(int16_t rpm) override;

    uint16_t getMaxPossibleRpmForward() override;
    uint16_t getMaxPossibleRpmBackward() override;

    void calculateMotorCurve(struct MotorKnownRpm points[], uint8_t len);
    void setMotorCurve(int16_t *lut, MotorCurveLutType type);

private:
    int16_t getLutValueAt(int16_t i);
};

#endif