#include "MotorProfile.hpp"

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

MotorProfile::MotorProfile(uint16_t wheelCircumfrence)
{
    this->wheelCircumfrence = wheelCircumfrence;
}

void MotorProfile::calculateMotorCurve(struct MotorKnownRpm points[], uint8_t len)
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
        this->rpmLut = new uint16_t[256];
    }
    else if (this->rpmLutType != MOTOR_CURVE_TYPE_RAM)
    {
        this->rpmLut = new uint16_t[256];
    }
    this->rpmLutType = MOTOR_CURVE_TYPE_RAM;

    for (uint16_t i = 0; i < 256; i++)
    {
    }
}

uint8_t MotorProfile::rpmToSpeed(uint16_t rpm)
{
    if (rpm == 0)
    {
        return 0;
    }
    if (this->rpmLut == nullptr)
    {
        return min(255, rpm);
    }
    else
    {
        return 0; // todo
    }
}

uint16_t MotorProfile::getMaxRpm()
{
    return 255;
}

MotorProfile::~MotorProfile()
{
    if (this->rpmLutType == MOTOR_CURVE_TYPE_RAM_ALLOCATED && this->rpmLut != nullptr)
    {
        delete[] this->rpmLut;
        this->rpmLutType = MOTOR_CURVE_TYPE_NONE;
    }
}