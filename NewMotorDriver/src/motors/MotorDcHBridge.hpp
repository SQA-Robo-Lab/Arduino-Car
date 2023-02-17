#ifndef MOTOR_DC_H_BRIDGE
#define MOTOR_DC_H_BRIDGE

#include "Motor.hpp"

class MotorDcHBridge : public Motor
{
private:
    uint8_t speedPin;
    uint8_t polarityPin1;
    uint8_t polarityPin2;

public:
    MotorDcHBridge(MotorSpeedUnit speedUnit, uint8_t speedEnablePin, uint8_t polarityDirectionPin1, uint8_t polarityDirectionPin2);

protected:
    void setSpeedRatioInternal(int16_t speedRatio) override;
};

#endif