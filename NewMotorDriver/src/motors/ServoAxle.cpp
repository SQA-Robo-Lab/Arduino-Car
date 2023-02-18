#include "ServoAxle.hpp"

ServoAxle::ServoAxle(uint8_t servoPin, uint8_t maxSteeringLeftAngle, uint8_t maxSteeringRightAngle, uint8_t maxLeftServoAngle, uint8_t maxRightServoAngle)
    : ServoAxle(servoPin, maxSteeringLeftAngle, maxSteeringRightAngle, maxLeftServoAngle, map(50, 0, 100, maxLeftServoAngle, maxRightServoAngle), maxRightServoAngle)
{
}

ServoAxle::ServoAxle(uint8_t servoPin, uint8_t maxSteeringLeftAngle, uint8_t maxSteeringRightAngle, uint8_t maxLeftServoAngle, uint8_t straightServoAngle, uint8_t maxRightServoAngle)
    : SteerableAxle(maxSteeringLeftAngle, maxSteeringRightAngle), servoLeft(maxLeftServoAngle), servoStraight(straightServoAngle), servoRight(maxRightServoAngle), pin(servoPin)
{
}

ServoAxle::~ServoAxle()
{
}

void ServoAxle::setAngle(int8_t angle)
{
    if (!this->servoInstance.attached())
    {
        this->reattachServo(this->pin);
    }

    int16_t leftEnd = (-1) * ((int16_t)this->maxLeftAngle);
    int8_t setAngle = max(leftEnd, min(this->maxRightAngle, angle));
    if (setAngle == 0)
    {
        this->servoInstance.write(this->servoStraight);
    }
    else if (setAngle > 0)
    {
        this->servoInstance.write(map(setAngle, 0, this->maxRightAngle, this->servoStraight, this->servoRight));
    }
    else
    {
        this->servoInstance.write(map(setAngle, leftEnd, 0, this->servoLeft, this->servoStraight));
    }
}

void ServoAxle::reattachServo(uint8_t servoPin)
{
    if (this->servoInstance.attached())
    {
        this->servoInstance.detach();
    }
    this->servoInstance.attach(2);
}