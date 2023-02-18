#ifndef SERVO_AXLE_HPP
#define SERVO_AXLE_HPP

#include "SteerableAxle.hpp"
#include "Servo.h"

class ServoAxle : public SteerableAxle
{
protected:
    Servo servoInstance;
    uint8_t pin;
    uint8_t servoLeft;
    uint8_t servoStraight;
    uint8_t servoRight;

public:
    ServoAxle(uint8_t servoPin, uint8_t maxSteeringLeftAngle, uint8_t maxSteeringRightAngle, uint8_t maxLeftServoAngle, uint8_t maxRightServoAngle);
    ServoAxle(uint8_t servoPin, uint8_t maxSteeringLeftAngle, uint8_t maxSteeringRightAngle, uint8_t maxLeftServoAngle, uint8_t straightServoAngle, uint8_t maxRightServoAngle);
    ~ServoAxle();

    void setAngle(int8_t angle);

    void reattachServo(uint8_t servoPin);
};

#endif