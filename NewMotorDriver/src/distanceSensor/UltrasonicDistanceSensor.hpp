#ifndef ULTRASONIC_DISTANCE_SENSOR_HPP
#define ULTRASONIC_DISTANCE_SENSOR_HPP

#include "DistanceSensor.hpp"

#define SPEED_OF_SOUND_MM_PER_MSEC 340

class UltrasonicDistanceSensor : DistanceSensor
{
private:
    uint8_t trigger;
    uint8_t echo;
    uint16_t timeoutUs;

protected:
    uint16_t echoTimeToMm(uint16_t echoTimeUs);

public:
    UltrasonicDistanceSensor(uint8_t triggerPin, uint8_t echoPin, uint16_t maxDistance = 1000);
    UltrasonicDistanceSensor(uint8_t triggerEchoPin, uint16_t maxDistance = 1000);
    ~UltrasonicDistanceSensor(){};

    uint16_t getDistanceToClosestMm() override;
    int8_t getAngleToClosest() override;
};

#endif