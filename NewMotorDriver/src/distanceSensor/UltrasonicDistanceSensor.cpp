#include "UltrasonicDistanceSensor.hpp"

UltrasonicDistanceSensor::UltrasonicDistanceSensor(uint8_t triggerPin, uint8_t echoPin, uint16_t maxDistance)
    : trigger(triggerPin), echo(echoPin)
{
    if (maxDistance > 11141)
    {
        maxDistance = 11141;
    }
    this->timeoutUs = static_cast<uint16_t>(static_cast<unsigned long>(maxDistance) * (1000 * 2) / SPEED_OF_SOUND_MM_PER_MSEC);
}

uint16_t UltrasonicDistanceSensor::echoTimeToMm(unsigned long echoTimeUs)
{
    if (echoTimeUs == 0)
    {
        return DISTANCE_SENSOR_NO_OBJECT;
    }
    else
    {
        return (echoTimeUs - 10) * SPEED_OF_SOUND_MM_PER_MSEC / (1000 * 2);
    }
}

uint16_t UltrasonicDistanceSensor::getDistanceToClosestMm()
{
    unsigned long pulseUs;
    if (this->trigger == this->echo)
    {
        pinMode(this->trigger, OUTPUT);
        pinMode(this->echo, INPUT);
        digitalWrite(this->trigger, LOW);
        delayMicroseconds(5);
        digitalWrite(this->trigger, HIGH);
        delayMicroseconds(10);
        digitalWrite(this->trigger, LOW);
    }
    else
    {
        pinMode(this->trigger, OUTPUT);
        digitalWrite(this->trigger, LOW);
        delayMicroseconds(5);
        digitalWrite(this->trigger, HIGH);
        delayMicroseconds(10);
        digitalWrite(this->trigger, LOW);
        pinMode(this->echo, INPUT);
    }
    pulseUs = pulseIn(this->echo, HIGH, this->timeoutUs);

    return this->echoTimeToMm(pulseUs);
}

int8_t UltrasonicDistanceSensor::getAngleToClosest()
{
    return 0;
}