#include "LinearSensorLineDetector.hpp"

#define LINE_SENSOR_DEBUG

LinearSensorLineDetector::LinearSensorLineDetector(BrightnessSensor &sensor, uint8_t sensorDistanceMm, uint8_t definetlyWhiteThrethold, uint8_t definetlyBlackThrethold)
    : sensorsLen(1), sensorDistance(sensorDistanceMm), whiteThrethold(definetlyWhiteThrethold), blackThrethold(definetlyBlackThrethold)
{
    this->sensors = new BrightnessSensor *[1];
    this->sensors[0] = &sensor;
    this->totalSensors = sensor.numberOfSensors();
}
LinearSensorLineDetector::LinearSensorLineDetector(BrightnessSensor **multipleSensors, uint8_t numberOfSensors, uint8_t sensorDistanceMm, uint8_t definetlyWhiteThrethold, uint8_t definetlyBlackThrethold)
    : sensorsLen(numberOfSensors), sensorDistance(sensorDistanceMm), whiteThrethold(definetlyWhiteThrethold), blackThrethold(definetlyBlackThrethold)
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

#ifdef LINE_SENSOR_DEBUG
    Serial.print("Brigtness values: ");
#endif

    int8_t pos = LINE_DETECTOR_NO_LINE_FOUND;
    float maxVal = 0.2;
    float lowerValForMax = 0;
    uint8_t numLargerHalf = 0;

    int16_t levelDiff = static_cast<int16_t>(this->blackThrethold) - static_cast<int16_t>(this->whiteThrethold);
    float prevVal = (static_cast<float>(values[0]) - this->whiteThrethold) / levelDiff;
    prevVal = min(1, max(0, prevVal));
    if (prevVal > 0.5)
    {
        numLargerHalf++;
    }
#ifdef LINE_SENSOR_DEBUG
    Serial.print(prevVal);
    Serial.print("; ");
#endif

    for (nextIndex = 1; nextIndex < this->totalSensors; nextIndex++)
    {
        float currVal = (static_cast<float>(values[nextIndex]) - this->whiteThrethold) / levelDiff;
        currVal = min(1, max(0, currVal));

#ifdef LINE_SENSOR_DEBUG
        Serial.print(currVal);
        Serial.print("; ");
#endif

        if (currVal > 0.5)
        {
            numLargerHalf++;
        }

        if (currVal > maxVal || prevVal > maxVal || (currVal == maxVal && prevVal > lowerValForMax) || (prevVal == maxVal && currVal > lowerValForMax))
        {
            if (currVal > prevVal)
            {
                maxVal = currVal;
                lowerValForMax = prevVal;
            }
            else
            {
                maxVal = prevVal;
                lowerValForMax = currVal;
            }
            float largerVal = max(currVal, prevVal);
            float ratio = ((currVal / largerVal) - (prevVal / largerVal) + 1) / 2;
            pos = floor((ratio + (nextIndex - 1) - ((this->totalSensors - 1) / 2)) * this->sensorDistance);
        }

        prevVal = currVal;
    }
#ifdef LINE_SENSOR_DEBUG
    Serial.println();
#endif

    return pos;
}

int8_t LinearSensorLineDetector::getLineAngle()
{
    return 0;
}
