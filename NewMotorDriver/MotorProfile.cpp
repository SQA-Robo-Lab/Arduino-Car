#include "MotorProfile.hpp"

MotorProfile::MotorProfile()
{
}

void MotorProfile::calculateMotorCurve(struct MotorKnownRpm points[], uint8_t len)
{
    float matrix[len][len + 1];

    for (uint8_t y = 0; y < len; y++)
    {
        float tmpPow = 1;
        for (uint8_t x = 0; x < len; x++)
        {
            matrix[y][x] = tmpPow;
            tmpPow *= points[y].rpm;
        }
        matrix[y][len] = points[y].speed;
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

    delete[] (this->motorCurveCoefficients);
    this->motorCurveCoefficients = new float[len];
    for (uint8_t y = 0; y < len; y++)
    {
        this->motorCurveCoefficients[y] = matrix[y][len];
    }
    this->coefficientsSize = len;
}

uint8_t MotorProfile::rpmToSpeed(uint16_t rpm)
{
    if (this->coefficientsSize == 0)
    {
        return min(255, rpm);
    }
    else
    {
        float tmpPow = 1;
        float currVal = 0;
        for (uint8_t x = 0; x < this->coefficientsSize; x++)
        {
            currVal += tmpPow * this->motorCurveCoefficients[x];
            tmpPow *= rpm;
        }
        return floor(currVal);
    }
}