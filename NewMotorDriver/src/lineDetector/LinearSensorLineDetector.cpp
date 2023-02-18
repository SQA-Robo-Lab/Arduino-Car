#include "LinearSensorLineDetector.hpp"

LinearSensorLineDetector::LinearSensorLineDetector(BrightnessSensor &sensor, uint8_t sensorDistanceMm)
    : sensorsLen(1), sensorDistance(sensorDistanceMm)
{
    this->sensors = new BrightnessSensor *[1];
    this->sensors[0] = &sensor;
    this->totalSensors = sensor.numberOfSensors();
}
LinearSensorLineDetector::LinearSensorLineDetector(BrightnessSensor **multipleSensors, uint8_t numberOfSensors, uint8_t sensorDistanceMm)
    : sensorsLen(numberOfSensors), sensorDistance(sensorDistanceMm)
{
    this->sensors = new BrightnessSensor *[numberOfSensors];
    memcpy(this->sensors, multipleSensors, numberOfSensors);
    this->totalSensors = 0;
    for (uint16_t i = 0; i < numberOfSensors; i++)
    {
        this->totalSensors += this->sensors[i]->numberOfSensors();
    }
}
LinearSensorLineDetector::~LinearSensorLineDetector()
{
    delete[] this->sensors;
}

int8_t LinearSensorLineDetector::getLinePositionMm()
{
    uint8_t values[this->totalSensors];
    uint32_t nextIndex = 0;
    for (uint16_t i = 0; i < this->sensorsLen; i++)
    {
        nextIndex += this->sensors[i]->getValues(values + nextIndex, this->totalSensors - nextIndex);
    }
    uint8_t maxValue = BRIGHTNESS_SENSOR_THRETHOLD;
    int32_t maxSensorIndex = -1;
    for (nextIndex = 0; nextIndex < this->totalSensors; nextIndex++)
    {
        if (values[nextIndex] >= maxValue)
        {
            maxSensorIndex = nextIndex;
            maxValue = values[nextIndex];
        }
    }

    if (maxSensorIndex == -1)
    {
        return LINE_DETECTOR_NO_LINE_FOUND;
    }
    else
    {
        return maxSensorIndex * this->sensorDistance - ((this->totalSensors - 1) * this->sensorDistance) / 2;
    }
}

int8_t LinearSensorLineDetector::getLineAngle()
{
    return 0;
}
