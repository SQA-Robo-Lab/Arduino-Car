#include "PolyCurveMotorProfile.hpp"

#define POLY_CURVE_BIN_SEARCH_DEBUG

void polynomialRegression(float xList[], float yList[], uint8_t len, float *solution)
{
    float matrix[len][len + 1];

    for (uint8_t y = 0; y < len; y++)
    {
        float tmpPow = 1;
        for (uint8_t x = 0; x < len; x++)
        {
            matrix[y][x] = tmpPow;
            tmpPow *= xList[y];
        }
        matrix[y][len] = yList[y];
    }

    for (uint8_t y = 0; y < len; y++)
    {
        uint8_t divCol = -1;
        float factor = 0;
        for (uint8_t x = 0; x < len + 1; x++)
        {
            if (divCol == -1 && matrix[y][x] != 0)
            {
                divCol = x;
                factor = 1.0 / matrix[y][x];
                matrix[y][x] = 1;
            }
            else if (divCol >= 0)
            {
                matrix[y][x] = factor * matrix[y][x];
            }
        }

        for (uint8_t y_sub = 0; y_sub < len; y_sub++)
        {
            if (y_sub != y && matrix[y_sub][divCol] != 0)
            {
                factor = matrix[y_sub][divCol] / matrix[y][divCol];
                for (uint8_t x_sub = 0; x_sub < len + 1; x_sub++)
                {
                    matrix[y_sub][x_sub] -= factor * matrix[y][x_sub];
                }
            }
        }
    }

    for (uint8_t y = 0; y < len; y++)
    {
        solution[y] = matrix[y][len];
    }
}

float calculatePolynomial(float x, float *coefficients, uint8_t numCoefficients)
{
    float tmpPow = 1;
    float currVal = 0;
    for (uint8_t i = 0; i < numCoefficients; i++)
    {
        currVal += tmpPow * coefficients[i];
        tmpPow *= x;
    }
}

PolyCurveMotorProfile::PolyCurveMotorProfile()
{
}

int16_t PolyCurveMotorProfile::getLutValueAt(int16_t i)
{
    switch (this->rpmLutType)
    {
    case MOTOR_CURVE_TYPE_RAM_ALLOCATED:
    case MOTOR_CURVE_TYPE_RAM:
        return this->rpmLut[i];

#ifdef ARDUINO_ARCH_AVR
    case MOTOR_CURVE_TYPE_PROGMEM:
        return pgm_read_word_near(this->rpmLut + i);
        break;
#endif
    }
}

void PolyCurveMotorProfile::calculateMotorCurve(struct MotorKnownRpm points[], uint8_t len)
{
    float rpmList[len];
    float speedList[len];

    for (uint8_t y = 0; y < len; y++)
    {
        rpmList[y] = points[y].rpm;
        speedList[y] = points[y].speed;
    }

    float rpmForSpeedSolution[len];
    polynomialRegression(speedList, rpmList, len, rpmForSpeedSolution);

    if (this->rpmLut == nullptr)
    {
        this->rpmLut = new int16_t[512];
    }
    else if (this->rpmLutType != MOTOR_CURVE_TYPE_RAM && this->rpmLutType != MOTOR_CURVE_TYPE_RAM_ALLOCATED)
    {
        this->rpmLut = new int16_t[512];
    }
    this->rpmLutType = MOTOR_CURVE_TYPE_RAM_ALLOCATED;

    for (int16_t i = -256; i < 256; i++)
    {
        float res = calculatePolynomial(i, rpmForSpeedSolution, len);
        if ((res <= 0 && i > 0) || (res >= 0 && i < 0))
        {
            res = 0;
        }
        else if (i > -256 && res < this->rpmLut[i + 256 - 1])
        {
            res = this->rpmLut[i + 256 - 1];
        }
        this->rpmLut[i + 256] = floor(res);
    }
}

int16_t PolyCurveMotorProfile::rpmToRatio(int16_t rpm)
{
    if (rpm == 0)
    {
        return 0;
    }
    if (this->rpmLut == nullptr)
    {
        return min(255, max(-255, rpm));
    }
    else
    {
#ifdef POLY_CURVE_BIN_SEARCH_DEBUG
        Serial.println("Starting binary search");
#endif
        int16_t lastFoundVal = 0;
        int16_t lastIndex = 0;
        for (int16_t currIndex = 256;;)
        {
            uint8_t jumpSize = abs(currIndex - lastIndex) / 2;
            int16_t currVal = this->getLutValueAt(currIndex);
#ifdef POLY_CURVE_BIN_SEARCH_DEBUG
            Serial.print("Next jump size: ");
            Serial.print(jumpSize);
            Serial.print("; Current value: ");
            Serial.print(currVal);
#endif
            if (jumpSize == 0)
            {
                if (abs(lastFoundVal - rpm) < abs(currVal - rpm))
                {
#ifdef POLY_CURVE_BIN_SEARCH_DEBUG
                    Serial.print("; Ending, last val closer, returning ");
                    Serial.println(lastIndex - 256);
#endif
                    return lastIndex - 256;
                }
                else
                {
#ifdef POLY_CURVE_BIN_SEARCH_DEBUG
                    Serial.print("; Ending, current val closer, returning ");
                    Serial.println(currIndex - 256);
#endif
                    return currIndex - 256;
                }
            }

            lastIndex = currIndex;
            lastFoundVal = currVal;

            if (rpm > currVal)
            {
#ifdef POLY_CURVE_BIN_SEARCH_DEBUG
                Serial.println("; Jumping up");
#endif
                currIndex += jumpSize;
            }
            else if (rpm < currVal)
            {
#ifdef POLY_CURVE_BIN_SEARCH_DEBUG
                Serial.println("; Jumping down");
#endif
                currIndex -= jumpSize;
            }
            else
            {
#ifdef POLY_CURVE_BIN_SEARCH_DEBUG
                Serial.print("; Ending, exact hit at index ");
                Serial.println(currIndex - 256);
#endif
                return currIndex - 256;
            }
        }
    }
}

uint16_t PolyCurveMotorProfile::getMaxPossibleRpmForward()
{
    if (this->rpmLut == nullptr)
    {
        return 255;
    }
    else
    {
        return this->getLutValueAt(255 + 256);
    }
}
uint16_t PolyCurveMotorProfile::getMaxPossibleRpmBackward()
{
    if (this->rpmLut == nullptr)
    {
        return -256;
    }
    else
    {
        return this->getLutValueAt(0);
    }
}

PolyCurveMotorProfile::~PolyCurveMotorProfile()
{
    if (this->rpmLutType == MOTOR_CURVE_TYPE_RAM_ALLOCATED && this->rpmLut != nullptr)
    {
        delete[] this->rpmLut;
        this->rpmLutType = MOTOR_CURVE_TYPE_NONE;
    }
}

void PolyCurveMotorProfile::setMotorCurve(int16_t *lut, MotorCurveLutType type)
{
    if (this->rpmLutType == MOTOR_CURVE_TYPE_RAM_ALLOCATED && this->rpmLut != nullptr)
    {
        delete[] this->rpmLut;
    }

    if (lut == nullptr)
    {
        this->rpmLut = nullptr;
        this->rpmLutType = MOTOR_CURVE_TYPE_NONE;
        return;
    }

    this->rpmLut = lut;
    this->rpmLutType = type;
}