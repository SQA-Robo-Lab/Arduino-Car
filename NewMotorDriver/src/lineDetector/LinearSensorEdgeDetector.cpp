#include "LinearSensorEdgeDetector.hpp"

#define LINE_SENSOR_DEBUG

#define NO_SENSOR_INDEX 255

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
    // Serial.println();
#endif

    uint8_t firstNonWhite = NO_SENSOR_INDEX;
    uint8_t firstBlackIndex = NO_SENSOR_INDEX;
    for (uint8_t i = 0; i < this->totalSensors; i++)
    {
        if (firstBlackIndex == NO_SENSOR_INDEX && fValues[i] >= 1)
        {
            firstBlackIndex = i;
        }
        if (firstNonWhite == NO_SENSOR_INDEX && fValues[i] > 0.1)
        {
            firstNonWhite = i;
        }
    }

#ifdef LINE_SENSOR_DEBUG
    Serial.print("; First non white: ");
    Serial.print(firstNonWhite);
    Serial.print("; First black: ");
    Serial.print(firstBlackIndex);
#endif

    float linePosRel = NAN;
    if (firstBlackIndex == firstNonWhite + 1)
    {
        Serial.print("; A");
        // edge between first non white and first black
        linePosRel = firstBlackIndex - fValues[firstNonWhite];
    }
    else if (firstNonWhite != NO_SENSOR_INDEX && firstBlackIndex != NO_SENSOR_INDEX)
    {
        if (firstBlackIndex > 0)
        {
            if (firstNonWhite == firstBlackIndex)
            {
                Serial.print("; B");
                // edge is (somewhere) between first black and previous
                linePosRel = firstBlackIndex - 0.5;
            }
            else
            {
                Serial.print("; C");
                // edge between first black and one previous
                linePosRel = firstBlackIndex - fValues[firstNonWhite];
            }
        }
        else
        {
            Serial.print("; D");
            // black is at edge of sensor array; edge of line outside
            linePosRel = 0;
        }
    }
    else if (firstNonWhite != NO_SENSOR_INDEX && firstBlackIndex == NO_SENSOR_INDEX)
    {
        // no black found; edge either between first non white and next one or previous one (depending on value of next one)
        float firstNonWhiteValue = fValues[firstNonWhite];
        float nextValue = firstNonWhite < this->totalSensors - 1 ? fValues[firstNonWhite + 1] : 0;

        if (nextValue > 0)
        {
            Serial.print("; E");
            // sensor after first non white is darker => line edge must be between that and the first non white
            float scale = 1.0 / (firstNonWhiteValue + nextValue);
            firstNonWhiteValue *= scale;
            nextValue *= scale;
            float posBetween = (-firstNonWhiteValue + nextValue) / 2.0 + 0.5;
            linePosRel = firstNonWhite + posBetween;
        }
        else
        {
            if (firstNonWhite > 0)
            {
                Serial.print("; F");
                // sensor after first non white is white (as is the one previous) => line edge must be between previous and first non white
                linePosRel = firstNonWhite - 0.5;
            }
            else
            {
                Serial.print("; G");
                // sensor after first non white is white and first non white is the first sensor => line edge is propably outside
                linePosRel = 0;
            }
        }
    }

    if (linePosRel != NAN)
    {
        linePosRel -= (this->totalSensors - 1) * 0.5;
        linePosRel = this->leftEdge ? linePosRel : -linePosRel;
#ifdef LINE_SENSOR_DEBUG
        Serial.print("; Detected pos: ");
        Serial.println(linePosRel);
#endif
        int8_t pos = round(linePosRel * this->sensorDistance);
        return pos;
    }
    else
    {
#ifdef LINE_SENSOR_DEBUG
        Serial.println("; Detected pos: nan");
#endif
        return LINE_DETECTOR_NO_LINE_FOUND;
    }
}

int8_t LinearSensorEdgeDetector::getLineAngle()
{
    return 0;
}