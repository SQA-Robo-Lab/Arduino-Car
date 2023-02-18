#ifndef LINEAR_SENSOR_LINE_DETECTOR_HPP
#define LINEAR_SENSOR_LINE_DETECTOR_HPP

#include "LineDetector.hpp"
#include "brightnessSensor/BrightnessSensor.hpp"

class LinearSensorLineDetector : public LineDetector
{
private:
    BrightnessSensor **sensors;
    uint8_t sensorsLen;
    uint8_t totalSensors;
    uint8_t sensorDistance;

public:
    LinearSensorLineDetector(BrightnessSensor &sensor, uint8_t sensorDistanceMm);
    LinearSensorLineDetector(BrightnessSensor **multipleSensors, uint8_t numberOfSensors, uint8_t sensorDistanceMm);
    ~LinearSensorLineDetector();

    int8_t getLinePositionMm() override;
    int8_t getLineAngle() override;
};

#endif