#include "LinearSensorLineDetector.hpp"

#define LINE_SENSOR_DEBUG

LinearSensorLineDetector::LinearSensorLineDetector(BrightnessSensor &sensor, uint8_t sensorDistanceMm, uint8_t minBrightnessThrethold, bool invertVals = false)
    : sensorsLen(1), sensorDistance(sensorDistanceMm), invert(invertVals), brightnessThrethold(minBrightnessThrethold)
{
    this->sensors = new BrightnessSensor *[1];
    this->sensors[0] = &sensor;
    this->totalSensors = sensor.numberOfSensors();
}
LinearSensorLineDetector::LinearSensorLineDetector(BrightnessSensor **multipleSensors, uint8_t numberOfSensors, uint8_t sensorDistanceMm, uint8_t minBrightnessThrethold, bool invertVals = false)
    : sensorsLen(numberOfSensors), sensorDistance(sensorDistanceMm), invert(invertVals), brightnessThrethold(minBrightnessThrethold)
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
    uint8_t maxValue = this->brightnessThrethold;
    int32_t maxSensorIndex = -1;

#ifdef LINE_SENSOR_DEBUG
    Serial.print("Brigtness values: ");
#endif
    for (nextIndex = 0; nextIndex < this->totalSensors; nextIndex++)
    {
        uint8_t currVal = this->invert ? 255 - values[nextIndex] : values[nextIndex];
#ifdef LINE_SENSOR_DEBUG
        Serial.print(currVal);
        Serial.print("; ");
#endif
        if (currVal >= maxValue)
        {
            maxSensorIndex = nextIndex;
            maxValue = currVal;
        }
    }
#ifdef LINE_SENSOR_DEBUG
    Serial.println();
#endif

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
