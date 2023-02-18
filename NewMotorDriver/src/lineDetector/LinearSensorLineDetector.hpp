#ifndef LINEAR_SENSOR_LINE_DETECTOR_HPP
#define LINEAR_SENSOR_LINE_DETECTOR_HPP

#include "LineDetector.hpp"

class LinearSensorLineDetector : public LineDetector
{
public:
    LinearSensorLineDetector();
    ~LinearSensorLineDetector(){};

    int8_t getLinePositionMm() override;
    int8_t getLineAngle() override;
};

#endif