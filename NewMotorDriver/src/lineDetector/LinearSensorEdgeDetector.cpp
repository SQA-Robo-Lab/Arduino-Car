#include "LinearSensorEdgeDetector.hpp"

#define LINE_SENSOR_DEBUG

float floatMap(float val, float inMin, float inMax, float outMin, float outMax)
{
    return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

LinearSensorEdgeDetector::LinearSensorEdgeDetector(BrightnessSensor &sensor, uint8_t sensorDistanceMm, uint8_t definetlyWhiteThrethold, uint8_t definetlyBlackThrethold, bool followLeftEdge)
    : sensorsLen(1), sensorDistance(sensorDistanceMm), whiteThrethold(definetlyWhiteThrethold), blackThrethold(definetlyBlackThrethold), leftEdge(followLeftEdge)
{
    this->sensors = new BrightnessSensor *[1];
    this->sensors[0] = &sensor;
    this->totalSensors = sensor.numberOfSensors();
}
LinearSensorEdgeDetector::LinearSensorEdgeDetector(BrightnessSensor **multipleSensors, uint8_t numberOfSensors, uint8_t sensorDistanceMm, uint8_t definetlyWhiteThrethold, uint8_t definetlyBlackThrethold, bool followLeftEdge)
    : sensorsLen(numberOfSensors), sensorDistance(sensorDistanceMm), whiteThrethold(definetlyWhiteThrethold), blackThrethold(definetlyBlackThrethold), leftEdge(followLeftEdge)
{
    this->sensors = new BrightnessSensor *[numberOfSensors];
    memcpy(this->sensors, multipleSensors, numberOfSensors);
    this->totalSensors = 0;
    for (uint8_t i = 0; i < numberOfSensors; i++)
    {
        this->totalSensors += this->sensors[i]->numberOfSensors();
    }
}
LinearSensorEdgeDetector::~LinearSensorEdgeDetector()
{
    delete[] this->sensors;
}

int8_t LinearSensorEdgeDetector::getLinePositionMm()
{
    uint8_t values[this->totalSensors];
    uint32_t nextIndex = 0;

    for (uint8_t i = 0; i < this->sensorsLen; i++)
    {
        nextIndex += this->sensors[i]->getValues(values + nextIndex, this->totalSensors - nextIndex);
    }

#ifdef LINE_SENSOR_DEBUG
    Serial.print("Brigtness values (raw,mapped): ");
#endif

    float fValues[this->totalSensors];
    for (uint8_t i = 0; i < this->totalSensors; i++)
    {
        float mapped = floatMap(values[i], this->whiteThrethold, this->blackThrethold, 0, 1);
#ifdef LINE_SENSOR_DEBUG
        Serial.print("(");
        Serial.print(values[i]);
        Serial.print(",");
        Serial.print(mapped);
        Serial.print("); ");
#endif
        fValues[this->leftEdge ? i : this->totalSensors - i - 1] = mapped;
    }
#ifdef LINE_SENSOR_DEBUG
    Serial.println();
#endif

    uint8_t firstNonWhite = 255;
    uint8_t firstBlackIndex = 255;
    for (uint8_t i = 0; i < this->totalSensors; i++)
    {
        if (firstBlackIndex == 255 && fValues[i] >= 1)
        {
            firstBlackIndex = i;
        }
        if (firstNonWhite == 255 && fValues[i] > 0)
        {
            firstNonWhite = i;
        }
    }

    float linePosRel = NAN;
    if (firstBlackIndex == firstNonWhite + 1)
    {
        linePosRel = firstBlackIndex - fValues[firstNonWhite] - (this->totalSensors * 0.5);
    }
    else if (firstNonWhite != 255 && firstNonWhite < this->totalSensors - 1)
    {
        float firstNonWhiteValue = fValues[firstNonWhite];
        float nextValue = fValues[firstNonWhite + 1];
        float scale = 1.0 / (firstNonWhiteValue + nextValue);
        firstNonWhiteValue *= scale;
        nextValue *= scale;
        float posBetween = (-firstNonWhiteValue + nextValue) / 2.0 + 1;
        linePosRel = firstNonWhite + posBetween - (this->totalSensors * 0.5);
    }
    else if (firstNonWhite != 255)
    {
        linePosRel = this->totalSensors * 0.5;
    }

    if (linePosRel != NAN)
    {
        linePosRel = this->leftEdge ? linePosRel : -linePosRel;
        int8_t pos = round(linePosRel * this->sensorDistance);
        return pos;
    }
    else
    {
        return LINE_DETECTOR_NO_LINE_FOUND;
    }
}

int8_t LinearSensorEdgeDetector::getLineAngle()
{
    return 0;
}