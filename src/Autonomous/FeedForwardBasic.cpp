#include <cmath>
#ifndef FEEDFORWARD //I think this needs to be added to be used in multiple files
#define FEEDFORWARD

class FeedForwardBasic {
    public:
    double kS;
    double kV;
    double kA;

    FeedForwardBasic(double kStaticVoltage, double kVoltsPerInchesPerSecond, double kVoltsPerInchesPerSecondSquared) : 
    kS(kStaticVoltage), 
    kV(kVoltsPerInchesPerSecond), 
    kA(kVoltsPerInchesPerSecondSquared)
    {}

    double calculateVolts(double targetInPerSec) {
        return (kS + kV * targetInPerSec); //Implement kA later...
    }

    double simpleCalculateVolts(double targetInPerSec) {
        return ((targetInPerSec / 76.6) * 12);
    }

};
#endif