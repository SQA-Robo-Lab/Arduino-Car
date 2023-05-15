#ifndef LINEAR_SENSOR_EDGE_DETECTOR_HPP
#define LINEAR_SENSOR_EDGE_DETECTOR_HPP

#include "LineDetector.hpp"
#include "brightnessSensor/BrightnessSensor.hpp"

#define BRIGHTNESS_SENSOR_THRETHOLD 20

class LinearSensorEdgeDetector : public LineDetector
{
private:
    BrightnessSensor **sensors;
    uint8_t sensorsLen;
    uint8_t totalSensors;
    uint8_t sensorDistance;
    uint8_t whiteThrethold;
    uint8_t blackThrethold;
    bool leftEdge;

public:
    LinearSensorEdgeDetector(BrightnessSensor &sensor, uint8_t sensorDistanceMm, uint8_t definetlyWhiteThrethold, uint8_t definetlyBlackThrethold, bool followLeftEdge);
    LinearSensorEdgeDetector(BrightnessSensor **multipleSensors, uint8_t numberOfSensors, uint8_t sensorDistanceMm, uint8_t definetlyWhiteThrethold, uint8_t definetlyBlackThrethold, bool followLeftEdge);
    ~LinearSensorEdgeDetector();

    int8_t getLinePositionMm() override;
    int8_t getLineAngle() override;
};

#endif