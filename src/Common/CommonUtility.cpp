#include "CommonUtility.h"

double inputModulus(double val, double min, double max)
{
    // Keeps the value within the interval min to max
    // This should be used on the rotation readings for consistency purposes

    double value = val;
    double modulus = max - min;

    int numMax = (int)((value - min) / modulus);
    value -= numMax * modulus;

    int numMin = (int)((value - max) / modulus);
    value -= numMin * modulus;

    return value;
}

double clamp(double val, double min, double max)
{
    double value = val;
    if (value > max)
    {
        value = max;
    }
    else if (value < min)
    {
        value = min;
    }
    return value;
}